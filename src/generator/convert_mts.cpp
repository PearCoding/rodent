#include "convert_mts.h"

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <cstring>
#include <limits>
#include <sstream>

#include "bvh.h"
#include "impala.h"
#include "export_image.h"
#include "spectral.h"
#include "platform.h"

#include "tinyparser-mitsuba.h"

/* Notice: Rodent only supports a small subset of the Mitsuba (0.6 and 2.0) project files */

using namespace TPM_NAMESPACE;

struct LoadInfo {
    std::string Filename;
    std::string Dir;
    ::Target Target;
    size_t MaxPathLen;
    size_t SPP;
    bool EmbreeBVH;
    bool Fusion;
    bool EnablePadding;
    SpectralUpsampler* Upsampler;
};

struct Material {
    uint32_t MeshId = 0;
    std::shared_ptr<Object> BSDF;
    std::shared_ptr<Object> Light;
};

inline bool operator==(const Material& a, const Material& b) {
    return a.MeshId == b.MeshId && a.BSDF == b.BSDF && a.Light == b.Light;
}

class MaterialHash {
public:
    size_t operator()(const Material &s) const {
        size_t h1 = std::hash<decltype(s.BSDF)>()(s.BSDF);
        size_t h2 = std::hash<decltype(s.Light)>()(s.Light);
        size_t h3 = std::hash<decltype(s.MeshId)>()(s.MeshId);
        return (h1 ^ (h2 << 1)) ^ (h3 << 1);
    }
};

struct Shape {
    size_t VtxOffset;
    size_t ItxOffset;
    size_t VtxCount;
    size_t ItxCount;
    ::Material Material;
};

struct GenContext {
    std::vector<Shape> Shapes;
    std::vector<Material> Materials;
    std::unordered_set<std::shared_ptr<Object>> Textures;
    obj::TriMesh Mesh;
    BBox SceneBBox;
    float SceneDiameter = 0.0f;
};

inline bool is_simple_brdf(const std::string &brdf)
{
    return brdf == "diffuse";
}

inline void setup_integrator(const Object &obj, const LoadInfo& info, std::ostream &os)
{
    // TODO: Use the sensor sample count as ssp
    for (const auto &child : obj.anonymousChildren()) {
        if (child->type() != OT_INTEGRATOR)
            continue;

        if (child->pluginType() == "path") {
            size_t mpl = child->property("max_depth").getInteger(info.MaxPathLen);
            os << "    let renderer = make_path_tracing_renderer(" << mpl << " /*max_path_len*/, " << info.SPP << " /*spp*/);\n";
            return;
        }
    }

    warn("No known integrator specified, therefore using path tracer");
    os << "    let renderer = make_path_tracing_renderer(" << info.MaxPathLen << " /*max_path_len*/, " << info.SPP << " /*spp*/);\n";
    //os << "     let renderer = make_debug_renderer();\n";
}

inline void setup_camera(const Object &obj, const LoadInfo& info, std::ostream &os)
{
    // TODO: Extract default settings?
    // Setup camera
    os << "\n    // Camera\n"
       << "    let camera = make_perspective_camera(\n"
       << "        math,\n"
       << "        settings.eye,\n"
       << "        make_mat3x3(settings.right, settings.up, settings.dir),\n"
       << "        settings.width,\n"
       << "        settings.height\n"
       << "    );\n";
}

inline float3 applyRotationScale(const Transform& t, const float3& v) {
    float3 res;
    for(int i = 0; i < 3; ++i) {
        res[i] = 0;
        for(int j = 0; j < 3; ++j)
            res[i] += t(i,j)*v[j];
    }
    return res;
}

inline float3 applyTransformAffine(const Transform& t, const float3& v) {
    return applyRotationScale(t, v) + float3(t(0,3),t(1,3),t(2,3));
}

// Apply inverse of transpose of orthogonal part of the transform
// which is the original orthogonal part if non-uniform scale is prohibited.
// TODO: We are ignoring non-uniform scale properties
inline float3 applyNormalTransform(const Transform& t, const float3& v) {
    return applyRotationScale(t, v);
}

// Unpack bsdf such that twosided materials are ignored and texture nodes are registered
inline std::shared_ptr<Object> add_bsdf(const std::shared_ptr<Object>& elem, GenContext& ctx) {
    if(elem->pluginType() == "twosided") {
        if(elem->anonymousChildren().size() != 1)
            error("Invalid twosided bsdf");
        //warn("Ignoring twosided bsdf");
        return add_bsdf(elem->anonymousChildren().front(), ctx);
    } else {
        for(const auto& child: elem->namedChildren()) {
            if(child.second->type() == OT_TEXTURE)
                ctx.Textures.insert(child.second);
        }

        return elem;
    }
}

// Unpack emission such that texture nodes are registered
inline std::shared_ptr<Object> add_light(const std::shared_ptr<Object>& elem, GenContext& ctx) {
    for(const auto& child: elem->namedChildren()) {
        if(child.second->type() == OT_TEXTURE)
            ctx.Textures.insert(child.second);
    }

    return elem;
}

inline constexpr std::array<uint32_t, 8> map_rectangle_index(const std::array<uint32_t, 4>& points) {
    return {points[0],points[1],points[2],0, points[2],points[3],points[0],0};
}

template<size_t N>
inline void insert_index(obj::TriMesh& mesh, const std::array<uint32_t,N>& arr) {
    mesh.indices.insert(mesh.indices.end(), arr.begin(), arr.end());
}

inline void add_rectangle(obj::TriMesh& mesh, const std::array<float3, 4>& points, const float3& N) {
    uint32_t off = mesh.vertices.size();
    mesh.vertices.insert(mesh.vertices.end(), points.begin(), points.end());
    mesh.normals.insert(mesh.normals.end(), {N,N,N,N});
    mesh.texcoords.insert(mesh.texcoords.end(), {float2(0,0),float2(0,1),float2(1,1),float2(1,0)});
    mesh.face_normals.insert(mesh.face_normals.end(), {N,N});
    insert_index(mesh, map_rectangle_index({0+off,1+off,2+off,3+off}));
}

inline obj::TriMesh setup_mesh_rectangle(const Object& elem, const LoadInfo& info) {
    const float3 N = float3(0,0,1);
    obj::TriMesh mesh;
    add_rectangle(mesh, {float3(-1,-1,0),float3(1,-1,0),float3(1,1,0),float3(-1,1,0)}, N);
    return mesh;
}

inline obj::TriMesh setup_mesh_cube(const Object& elem, const LoadInfo& info) {
    const float3 NZ = float3(0,0,1);
    const float3 NY = float3(0,1,0);
    const float3 NX = float3(1,0,0);

    // TODO: Fix order (is it?)
    obj::TriMesh mesh;
    add_rectangle(mesh, {float3(-1,-1,-1),float3(1,-1,-1),float3(1,1,-1),float3(-1,1,-1)}, -NZ);
    add_rectangle(mesh, {float3(-1,-1,1),float3(-1,1,1),float3(1,1,1),float3(1,-1,1)}, NZ);

    add_rectangle(mesh, {float3(1,-1,-1),float3(1,1,-1),float3(1,1,1),float3(1,-1,1)}, NX);
    add_rectangle(mesh, {float3(-1,-1,-1),float3(-1,-1,1),float3(-1,1,1),float3(-1,1,-1)}, -NX);

    add_rectangle(mesh, {float3(-1,-1,-1),float3(1,-1,-1),float3(1,-1,1),float3(-1,-1,1)}, -NY);
    add_rectangle(mesh, {float3(-1,1,-1),float3(-1,1,1),float3(1,1,1),float3(1,1,-1)}, NY);
    return mesh;
}

inline obj::TriMesh setup_mesh_obj(const Object& elem, const LoadInfo& info) {
    obj::File file;
    std::string filename = info.Dir + "/" + elem.property("filename").getString();
    if(!obj::load_obj(FilePath(filename), file)) {
        warn("Can not load shape given by file '", filename, "'");
        return obj::TriMesh();
    }

    return obj::compute_tri_mesh(file, 0);           
}

static void setup_shapes(const Object& elem, const LoadInfo& info, GenContext& ctx, std::ostream &os) {
    std::unordered_map<Material, uint32_t, MaterialHash> unique_mats;

    for(const auto& child : elem.anonymousChildren()) {
        if(child->type() != OT_SHAPE)
            continue;

        obj::TriMesh child_mesh;
        if (child->pluginType() == "rectangle") {
            child_mesh = setup_mesh_rectangle(*child, info);
        } else if (child->pluginType() == "cube") {
            child_mesh = setup_mesh_cube(*child, info);
        } else if(child->pluginType() == "obj") {
            child_mesh = setup_mesh_obj(*child, info);
        } else {
            warn("Can not load shape type '", child->pluginType(), "'");
            continue;
        }

        if(child_mesh.vertices.empty())
            continue;
        
        auto transform = child->property("to_world").getTransform();
        for(size_t i = 0; i < child_mesh.vertices.size(); ++i)
            child_mesh.vertices[i] = applyTransformAffine(transform, child_mesh.vertices[i]);
        for(size_t i = 0; i < child_mesh.normals.size(); ++i)
            child_mesh.normals[i] = applyNormalTransform(transform, child_mesh.normals[i]);
        for(size_t i = 0; i < child_mesh.face_normals.size(); ++i)
            child_mesh.face_normals[i] = applyNormalTransform(transform, child_mesh.face_normals[i]);
            
        Shape shape;
        shape.VtxOffset = ctx.Mesh.vertices.size();
        shape.ItxOffset = ctx.Mesh.indices.size();
        shape.VtxCount  = child_mesh.vertices.size();
        shape.ItxCount  = child_mesh.indices.size();
        
        // Setup material & light
        for(const auto& inner_child : child->anonymousChildren()) {
            if(inner_child->type() == OT_BSDF)
                shape.Material.BSDF = add_bsdf(inner_child, ctx);
            else if(inner_child->type() == OT_EMITTER) {
                shape.Material.Light = add_light(inner_child, ctx);
                shape.Material.MeshId = ctx.Shapes.size();
            }
        }

        if(!unique_mats.count(shape.Material)) {
            unique_mats.emplace(shape.Material, ctx.Materials.size());
            ctx.Materials.emplace_back(shape.Material);
        }

        obj::replace_material_tri_mesh(child_mesh, unique_mats.at(shape.Material));
        obj::combine_into_tri_mesh(ctx.Mesh, child_mesh); 
        ctx.Shapes.emplace_back(std::move(shape));
    }

    if(ctx.Shapes.empty()) {
        error("No mesh available");
        return;
    }

    ::info("Generating merged triangle mesh");
    os << "\n    // Triangle mesh\n"
       << "    let vertices     = device.load_buffer(\"data/vertices.bin\");\n"
       << "    let normals      = device.load_buffer(\"data/normals.bin\");\n"
       << "    let face_normals = device.load_buffer(\"data/face_normals.bin\");\n"
       << "    let face_area    = device.load_buffer(\"data/face_area.bin\");\n"
       << "    let indices      = device.load_buffer(\"data/indices.bin\");\n"
       << "    let texcoords    = device.load_buffer(\"data/texcoords.bin\");\n"
       << "    let tri_mesh     = TriMesh {\n"
       << "        vertices:     @ |i| vertices.load_vec3(i),\n"
       << "        normals:      @ |i| normals.load_vec3(i),\n"
       << "        face_normals: @ |i| face_normals.load_vec3(i),\n"
       << "        face_area:    @ |i| face_area.load_f32(i),\n"
       << "        triangles:    @ |i| { let (i, j, k, _) = indices.load_int4(i); (i, j, k) },\n"
       << "        attrs:        @ |_| (false, @ |j| vec2_to_4(texcoords.load_vec2(j), 0.0f, 0.0f)),\n"
       << "        num_attrs:    1,\n"
       << "        num_tris:     " << ctx.Mesh.indices.size() / 4 << "\n"
       << "    };\n"
       << "    let bvh = device.load_bvh(\"data/bvh.bin\");\n";

    write_tri_mesh(ctx.Mesh, info.EnablePadding);

    // Generate BVHs
    if (must_build_bvh(info.Filename, info.Target))
    {
        ::info("Generating BVH for '", info.Filename, "'");
        std::remove("data/bvh.bin");
        if (info.Target == Target::NVVM_STREAMING || info.Target == Target::NVVM_MEGAKERNEL ||
            info.Target == Target::AMDGPU_STREAMING || info.Target == Target::AMDGPU_MEGAKERNEL)
        {
            std::vector<typename BvhNTriM<2, 1>::Node> nodes;
            std::vector<typename BvhNTriM<2, 1>::Tri> tris;
            build_bvh<2, 1>(ctx.Mesh, nodes, tris);
            write_bvh(nodes, tris);
        }
        else if (info.Target == Target::GENERIC || info.Target == Target::ASIMD || info.Target == Target::SSE42)
        {
            std::vector<typename BvhNTriM<4, 4>::Node> nodes;
            std::vector<typename BvhNTriM<4, 4>::Tri> tris;
#ifdef ENABLE_EMBREE_BVH
            if (embree_bvh)
                build_embree_bvh<4>(ctx.Mesh, nodes, tris);
            else
#endif
                build_bvh<4, 4>(ctx.Mesh, nodes, tris);
            write_bvh(nodes, tris);
        }
        else
        {
            std::vector<typename BvhNTriM<8, 4>::Node> nodes;
            std::vector<typename BvhNTriM<8, 4>::Tri> tris;
#ifdef ENABLE_EMBREE_BVH
            if (embree_bvh)
                build_embree_bvh<8>(ctx.Mesh, nodes, tris);
            else
#endif
                build_bvh<8, 4>(ctx.Mesh, nodes, tris);
            write_bvh(nodes, tris);
        }
        std::ofstream bvh_stamp("data/bvh.stamp");
        bvh_stamp << int(info.Target) << " " << info.Filename;
    }
    else
    {
        ::info("Reusing existing BVH for '", info.Filename, "'");
    }

    // Calculate scene bounding box
    ctx.SceneBBox = BBox::empty();
    for(size_t i = 0; i < ctx.Mesh.vertices.size(); ++i)
        ctx.SceneBBox.extend(ctx.Mesh.vertices[i]);
    ctx.SceneDiameter = length(ctx.SceneBBox.max - ctx.SceneBBox.min);
}

static void setup_textures(const Object& elem, const LoadInfo& info, const GenContext& ctx, std::ostream &os) {
    if(ctx.Textures.empty())
        return;

    ::info("Generating images for '", info.Filename, "'");
    os << "\n    // Images\n";    
    for(const auto& tex: ctx.Textures) {
        if(tex->pluginType() != "bitmap")
            continue;

        std::string filename = tex->property("filename").getString();
        if(filename.empty()) {
            warn("Invalid texture found");
            continue;
        }
        auto name = fix_file(filename);
        auto c_name = export_image(info.Upsampler, info.Dir + "/" + name);
        os << "    let image_" << make_id(name) << " = device.load_img(\"" << c_name.path() << "\");\n";
        os << "    let tex_" << make_id(name) << " = make_texture(math, make_repeat_border(), make_bilinear_filter(), image_" << make_id(name) << ");\n";
    }
}

static std::string extractTexture(const std::shared_ptr<Object>& tex, const LoadInfo& info, const GenContext& ctx);
static std::string extractMaterialPropertySpectral(const std::shared_ptr<Object>& obj, const std::string& name, const LoadInfo& info, const GenContext& ctx, float def = 0.0f) {
    std::stringstream sstream;

    auto prop = obj->property(name);
    if(prop.isValid()) {
        switch(prop.type()) {
        case PT_INTEGER:
            sstream << "make_spectrum_const(" << escape_f32(prop.getInteger()) << ")";
            break;
        case PT_NUMBER:
            sstream << "make_spectrum_const(" << escape_f32(prop.getNumber()) << ")";
            break;
        case PT_RGB: {
            auto v_rgb = prop.getRGB();
            if(v_rgb.r > 1 || v_rgb.g > 1 || v_rgb.b > 1) {
                rgb r_rgb;
                float power;
                info.Upsampler->upsample_emissive_rgb(rgb(v_rgb.r,v_rgb.g,v_rgb.b), r_rgb, power);
                sstream << "spectrum_mulf(make_coeff_spectrum(math, " << escape_f32(r_rgb.x) << ", " << escape_f32(r_rgb.y) << ", " << escape_f32(r_rgb.z) << "), " << escape_f32(power) << ")";

            } else {
                auto r_rgb = info.Upsampler->upsample_rgb(rgb(v_rgb.r,v_rgb.g,v_rgb.b));
                sstream << "make_coeff_spectrum(math, " << escape_f32(r_rgb.x) << ", " << escape_f32(r_rgb.y) << ", " << escape_f32(r_rgb.z) << ")";
            }
        } break;
        case PT_SPECTRUM: {
            auto spec = prop.getSpectrum();
            if(spec.isUniform()) {
                sstream << "make_spectrum_const(" << escape_f32(spec.uniformValue()) << ")";
            } else {
                sstream << "{ "; 
                sstream << "let wvls = [";
                for(size_t i = 0; i < spec.wavelengths().size(); ++i)
                    sstream << escape_f32(spec.wavelengths()[i]) << ",";
                sstream << "]; let weights = [";
                for(size_t i = 0; i < spec.weights().size(); ++i)
                    sstream << escape_f32(spec.weights()[i]) << ",";
                sstream << "]; make_data_spectrum(math, wvls, weights, " << spec.weights().size() << ")";
                sstream << "}";
            }
        } break;
        case PT_BLACKBODY: {
            auto blk = prop.getBlackbody();
            sstream << "spectrum_mulf(make_blackbody_spectrum(math, " << escape_f32(blk.temperature)<< "), " << escape_f32(blk.scale)  << ")";
        } break;
        default:
            warn("Unknown property type");
            sstream << "make_spectrum_none()";
            break;
        }
    } else {
        auto tex = obj->namedChild(name);
        if(!tex) {
            sstream << "make_spectrum_const(" << escape_f32(def) << ")";
        } else {
            if(tex->type() == OT_TEXTURE) {
                sstream << extractTexture(tex, info, ctx);
            } else {
                warn("Invalid child type");
                sstream << "make_spectrum_none()";
            }
        }
    }

    return sstream.str();
}

static std::string extractMaterialPropertyIOR(const std::shared_ptr<Object>& obj, const std::string& name, const LoadInfo& info, const GenContext& ctx, float def = 0.0f) {
    std::stringstream sstream;

    auto prop = obj->property(name);
        switch(prop.type()) {
        case PT_INTEGER:
            sstream << "make_const_refractive_index(" << escape_f32(prop.getInteger()) << ")";
            break;
        case PT_NUMBER:
            sstream << "make_const_refractive_index(" << escape_f32(prop.getNumber()) << ")";
            break;
        case PT_NONE:// TODO: What about textures?
            sstream << "make_const_refractive_index(" << escape_f32(def) << ")";
            break;
        default:
            sstream << "make_spectral_refractive_index(" << extractMaterialPropertySpectral(obj, name, info, ctx, def) << ")";
            break;
        }

    return sstream.str();
}

static std::string extractMaterialPropertyIllum(const std::shared_ptr<Object>& obj, const std::string& name, const LoadInfo& info, const GenContext& ctx, float def = 0.0f) {
    std::stringstream sstream;

    auto prop = obj->property(name);
        switch(prop.type()) {
        case PT_INTEGER:
            sstream << "make_d65_illum(" << escape_f32(prop.getInteger()) << ")";
            break;
        case PT_NUMBER:
            sstream << "make_d65_illum(" << escape_f32(prop.getNumber()) << ")";
            break;
        case PT_RGB: {
            auto v_rgb = prop.getRGB();
            if(v_rgb.r > 1 || v_rgb.g > 1 || v_rgb.b > 1) {
                rgb r_rgb;
                float power;
                info.Upsampler->upsample_emissive_rgb(rgb(v_rgb.r,v_rgb.g,v_rgb.b), r_rgb, power);
                sstream << "make_colored_d65_illum(" << escape_f32(power) << ", make_coeff_spectrum(math, " << escape_f32(r_rgb.x) << ", " << escape_f32(r_rgb.y) << ", " << escape_f32(r_rgb.z) << "))";

            } else {
                auto r_rgb = info.Upsampler->upsample_rgb(rgb(v_rgb.r,v_rgb.g,v_rgb.b));
                sstream << "make_colored_d65_illum(1.0f, make_coeff_spectrum(math, " << escape_f32(r_rgb.x) << ", " << escape_f32(r_rgb.y) << ", " << escape_f32(r_rgb.z) << "))";
            }
        } break;
        default:
            return extractMaterialPropertySpectral(obj, name, info, ctx, def);
        }

    return sstream.str();
}

static std::string extractTexture(const std::shared_ptr<Object>& tex, const LoadInfo& info, const GenContext& ctx) {
    std::stringstream sstream;
    if(tex->pluginType() == "bitmap") {
        std::string filename = tex->property("filename").getString();
        if(filename.empty()) {
            warn("Invalid texture found");
            sstream << "make_spectrum_none()";
        } else {
            sstream << "tex_" << make_id(fix_file(filename)) << "(vec4_to_2(surf.attr(0)))";
        }
    } else if (tex->pluginType() == "checkerboard") {
        sstream << "eval_checkerboard_texture(math, make_repeat_border(), " 
            << extractMaterialPropertySpectral(tex, "color0", info, ctx, 0.4f) << ", " 
            << extractMaterialPropertySpectral(tex, "color1", info, ctx, 0.2f) << ", vec4_to_2(surf.attr(0)))";
    } else {
        warn("Invalid texture type '", tex->pluginType(), "'");
    }
    return sstream.str();
}

static void setup_materials(const Object& elem, const LoadInfo& info, const GenContext& ctx, std::ostream &os) {
    if(ctx.Materials.empty())
        return;

    ::info("Generating lights for '", info.Filename, "'");
    os << "\n    // Emission\n";
    size_t light_counter = 0;
    for(const auto& mat: ctx.Materials) {
        if(!mat.Light)
            continue;

        const auto& shape = ctx.Shapes[mat.MeshId];
        os << "    let light_" << light_counter << " = make_trimesh_light(math, tri_mesh, "
            << (shape.ItxOffset/4) << ", " << (shape.ItxCount/4) << ", "
            << extractMaterialPropertyIllum(mat.Light, "radiance", info, ctx) << ");\n";
        ++light_counter;
    }

    ::info("Generating materials for '", info.Filename, "'");
    os << "\n    // Materials\n";

    light_counter = 0;
    for(size_t i = 0; i < ctx.Materials.size(); ++i) {
        const auto& mat = ctx.Materials[i];
        os << "    let material_" << i << " : Shader = @ |ray, hit, surf| {\n";
        if(!mat.BSDF) {
            os << "        let bsdf = make_black_bsdf();\n";
        } else if(mat.BSDF->pluginType() == "diffuse" ||
            mat.BSDF->pluginType() == "roughdiffuse"/*TODO*/) {
            os << "        let bsdf = make_diffuse_bsdf(math, surf, " << extractMaterialPropertySpectral(mat.BSDF, "reflectance", info, ctx) << ");\n";
        } else if(mat.BSDF->pluginType() == "dielectric" ||
            mat.BSDF->pluginType() == "roughdielectric"/*TODO*/) {
            os << "        let bsdf = make_glass_bsdf(math, surf, " 
            << extractMaterialPropertyIOR(mat.BSDF, "ext_ior", info, ctx, 1.5046f) << ", " 
            << extractMaterialPropertyIOR(mat.BSDF, "int_ior", info, ctx, 1.000277f) << ", " 
            << extractMaterialPropertySpectral(mat.BSDF, "specular_reflectance", info, ctx, 1.0f) << ", " 
            << extractMaterialPropertySpectral(mat.BSDF, "specular_transmittance", info, ctx, 1.0f) << ");\n";
        } else if(mat.BSDF->pluginType() == "conductor" ||
            mat.BSDF->pluginType() == "roughconductor"/*TODO*/) {
            os << "        let bsdf = make_conductor_bsdf(math, surf, " 
            << extractMaterialPropertySpectral(mat.BSDF, "eta", info, ctx, 0.63660f) << ", " 
            << extractMaterialPropertySpectral(mat.BSDF, "k", info, ctx, 2.7834f) << ", " // TODO: Better defaults?
            << extractMaterialPropertySpectral(mat.BSDF, "specular_reflectance", info, ctx, 1.0f) << ");\n";
        }else {
            warn("Unknown bsdf '", mat.BSDF->pluginType(), "'");
            os << "        let bsdf = make_black_bsdf();\n";
        }

        if (mat.Light)
            os << "        make_emissive_material(surf, bsdf, light_" << light_counter << ")\n";
        else
            os << "        make_material(bsdf)\n";
        os << "    };\n";

        if(mat.Light)
            ++light_counter;
    }
}

static size_t setup_lights(const Object& elem, const LoadInfo& info, const GenContext& ctx, std::ostream &os) {
    ::info("Generating lights for '", info.Filename, "'");
    size_t light_count = 0;
    // Make sure area lights are the first ones
    for(const auto& m: ctx.Materials) {
        if(m.Light)
            ++light_count;
    }

    for(const auto& child : elem.anonymousChildren()) {
        if(child->type() != OT_EMITTER)
            continue;

        if(child->pluginType() == "point") {
            auto pos = child->property("position").getVector();
            os << "    let light_" << light_count << " = make_point_light(math, make_vec3(" 
                    << escape_f32(pos.x) << ", " << escape_f32(pos.y) << ", " << escape_f32(pos.z) << "), " 
                    << extractMaterialPropertyIllum(child, "intensity", info, ctx, 1.0f) << ");\n";
        } else if(child->pluginType() == "area") {
            warn("Area emitter without a shape is not allowed");
            continue;
        } else if(child->pluginType() == "directional") {// TODO: By to_world?
            auto dir = child->property("direction").getVector();
            os << "    let light_" << light_count << " = make_directional_light(math, make_vec3(" 
                    << escape_f32(dir.x) << ", " << escape_f32(dir.y) << ", " << escape_f32(dir.z) << "), " 
                    << escape_f32(ctx.SceneDiameter) << ", "
                    << extractMaterialPropertyIllum(child, "irradiance", info, ctx, 1.0f) << ");\n";
        } else if(child->pluginType() == "sun") {// TODO
            warn("Sun emitter is approximated by directional light");
            auto dir = child->property("sun_direction").getVector();
            auto power = child->property("scale").getNumber(1.0f);
            os << "    let light_" << light_count << " = make_directional_light(math, make_vec3(" 
                    << escape_f32(dir.x) << ", " << escape_f32(dir.y) << ", " << escape_f32(dir.z) << "), "  
                    << escape_f32(ctx.SceneDiameter) << ", "
                    << "make_d65_illum(" << escape_f32(power) << "));\n";
        } else {
            warn("Unknown emitter type '", child->pluginType(), "'");
            continue;
        }

        ++light_count;
    }


    os << "\n    // Lights\n";
    if(light_count == 0) { // Camera light
        os << "    let lights = @ |_| make_camera_light(math, camera, make_spectrum_identity());\n";
    }else{
        os << "    let lights = @ |i| match i {\n";
        for(size_t i = 0; i < light_count; ++i) {
        if (i == light_count - 1)
            os << "        _ => light_" << i << "\n";
        else
            os << "        " << i << " => light_" << i << ",\n";
        }
        os << "    };\n";
    }

    return light_count;
}

static void convert_scene(const Scene &scene, const LoadInfo& info, std::ostream &os)
{
    GenContext ctx;

    setup_integrator(scene, info, os);
    setup_camera(scene, info, os);
    setup_shapes(scene, info, ctx, os);
    setup_textures(scene, info, ctx, os);
    setup_materials(scene, info, ctx, os);
    size_t light_count = setup_lights(scene, info, ctx, os);
    
    os << "\n    // Geometries\n"
       << "    let geometries = @ |i| match i {\n";
    for (uint32_t s = 0; s < ctx.Materials.size(); ++s) {
        os << "        ";
        if (s != ctx.Materials.size() - 1 )
            os << s;
        else
            os << "_";
        os << " => make_tri_mesh_geometry(math, tri_mesh, material_" << s << "),\n";
    }
    os << "    };\n";

    os << "\n    // Scene\n"
       << "    let scene = Scene {\n"
       << "        num_geometries: " << ctx.Materials.size() << ",\n"
       << "        num_lights:     " << light_count << ",\n"
       << "        geometries:     @ |i| geometries(i),\n"
       << "        lights:         @ |i| lights(i),\n"
       << "        camera:         camera,\n"
       << "        bvh:            bvh\n"
       << "    };\n";
}

bool convert_mts(const std::string &file_name, Target target,
                 size_t dev, size_t max_path_len, size_t spp, bool embree_bvh, bool fusion,
                 SpectralUpsampler *upsampler, std::ostream &os)
{
    info("Converting MTS file '", file_name, "'");

    try {
        SceneLoader loader;
        loader.addArgument("SPP", std::to_string(spp));
        loader.addArgument("MAX_PATH_LENGTH", std::to_string(max_path_len));

        auto scene = loader.loadFromFile(file_name);
        FilePath path(file_name);

        create_directory("data/");
        create_directory("data/textures");

        os << "//------------------------------------------------------------------------------------\n"
           << "// Generated from '" << path.file_name() << "' with the scene conversion tool\n"
           << "//------------------------------------------------------------------------------------\n\n";

        os << "struct Settings {\n"
           << "    eye: Vec3,\n"
           << "    dir: Vec3,\n"
           << "    up: Vec3,\n"
           << "    right: Vec3,\n"
           << "    width: f32,\n"
           << "    height: f32\n"
           << "};\n";

        os << "\nextern fn get_spp() -> i32 { " << spp << " }\n";

        os << "\nextern fn render(settings: &Settings, iter: i32) -> () {\n";

        LoadInfo info;
        info.Filename      = file_name;
        info.Dir           = path.base_name();
        info.Target        = target;
        info.MaxPathLen    = max_path_len;
        info.SPP           = spp;
        info.EmbreeBVH     = embree_bvh;
        info.Fusion        = fusion;
        info.EnablePadding = target == Target::NVVM_STREAMING ||
                             target == Target::NVVM_MEGAKERNEL ||
                             target == Target::AMDGPU_STREAMING ||
                             target == Target::AMDGPU_MEGAKERNEL;
        info.Upsampler     = upsampler;

        switch (target)
        {
        case Target::GENERIC:
            os << "    let device   = make_cpu_default_device();\n";
            break;
        case Target::AVX2:
            os << "    let device   = make_avx2_device(false);\n";
            break;
        case Target::AVX2_EMBREE:
            os << "    let device   = make_avx2_device(true);\n";
            break;
        case Target::AVX:
            os << "    let device   = make_avx_device();\n";
            break;
        case Target::SSE42:
            os << "    let device   = make_sse42_device();\n";
            break;
        case Target::ASIMD:
            os << "    let device   = make_asimd_device();\n";
            break;
        case Target::NVVM_STREAMING:
            os << "    let device   = make_nvvm_device(" << dev << ", true);\n";
            break;
        case Target::NVVM_MEGAKERNEL:
            os << "    let device   = make_nvvm_device(" << dev << ", false);\n";
            break;
        case Target::AMDGPU_STREAMING:
            os << "    let device   = make_amdgpu_device(" << dev << ", true);\n";
            break;
        case Target::AMDGPU_MEGAKERNEL:
            os << "    let device   = make_amdgpu_device(" << dev << ", false);\n";
            break;
        default:
            assert(false);
            break;
        }

        os << "    let math     = device.intrinsics;\n";

        convert_scene(scene, info, os);

        os << "\n"
        << "    renderer(scene, device, iter);\n"
        << "    device.present();\n"
        << "}\n";
    }
    catch (const std::exception &e)
    {
        error("Invalid MTS file: ", e.what());
        return false;
    }

    info("Scene was converted successfully");
    return true;
}