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
    ::Target Target;
    size_t MaxPathLen;
    size_t SPP;
    bool EmbreeBVH;
    bool Fusion;
    bool EnablePadding;
    SpectralUpsampler* Upsampler;
};

inline bool is_simple_brdf(const std::string &brdf)
{
    return brdf == "diffuse";
}

void setup_integrator(const Object &obj, const LoadInfo& info, std::ostream &os)
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

void setup_shapes(const Object& elem, const LoadInfo& info, std::ostream &os) {
    obj::TriMesh mesh;

    for(const auto& child : elem.anonymousChildren()) {
        if(child->type() != OT_SHAPE)
            continue;

        if (child->pluginType() != "obj") {
            warn("Can not load shape type '", child->pluginType(), "'");
            continue;
        }

        obj::File file;
        std::string filename = FilePath(info.Filename).base_name() + "/" + child->property("filename").getString();
        if(!obj::load_obj(FilePath(filename), file)) {
            warn("Can not load shape given by file '", filename, "'");
            continue;
        }

        obj::TriMesh child_mesh = obj::compute_tri_mesh(file, 0);

        auto transform = child->property("to_world").getTransform();
        for(size_t i = 0; i < mesh.vertices.size(); ++i)
            mesh.vertices[i] = applyTransformAffine(transform, mesh.vertices[i]);
        for(size_t i = 0; i < mesh.normals.size(); ++i)
            mesh.normals[i] = applyNormalTransform(transform, mesh.normals[i]);
        for(size_t i = 0; i < mesh.face_normals.size(); ++i)
            mesh.face_normals[i] = applyNormalTransform(transform, mesh.face_normals[i]);

        obj::combine_into_tri_mesh(mesh, child_mesh); 
    }

    if(mesh.indices.empty()) {
        error("No mesh available");
        return;
    }

    ::info("Generating union triangle mesh");
    os << "\n    // Triangle mesh\n"
       << "    let vertices     = device.load_buffer(\"data/vertices.bin\");\n"
       << "    let normals      = device.load_buffer(\"data/normals.bin\");\n"
       << "    let face_normals = device.load_buffer(\"data/face_normals.bin\");\n"
       << "    let indices      = device.load_buffer(\"data/indices.bin\");\n"
       << "    let texcoords    = device.load_buffer(\"data/texcoords.bin\");\n"
       << "    let tri_mesh     = TriMesh {\n"
       << "        vertices:     @ |i| vertices.load_vec3(i),\n"
       << "        normals:      @ |i| normals.load_vec3(i),\n"
       << "        face_normals: @ |i| face_normals.load_vec3(i),\n"
       << "        triangles:    @ |i| { let (i, j, k, _) = indices.load_int4(i); (i, j, k) },\n"
       << "        attrs:        @ |_| (false, @ |j| vec2_to_4(texcoords.load_vec2(j), 0.0f, 0.0f)),\n"
       << "        num_attrs:    1,\n"
       << "        num_tris:     " << mesh.indices.size() / 4 << "\n"
       << "    };\n"
       << "    let bvh = device.load_bvh(\"data/bvh.bin\");\n";

    write_tri_mesh(mesh, info.EnablePadding);

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
            build_bvh<2, 1>(mesh, nodes, tris);
            write_bvh(nodes, tris);
        }
        else if (info.Target == Target::GENERIC || info.Target == Target::ASIMD || info.Target == Target::SSE42)
        {
            std::vector<typename BvhNTriM<4, 4>::Node> nodes;
            std::vector<typename BvhNTriM<4, 4>::Tri> tris;
#ifdef ENABLE_EMBREE_BVH
            if (embree_bvh)
                build_embree_bvh<4>(tri_mesh, nodes, tris);
            else
#endif
                build_bvh<4, 4>(mesh, nodes, tris);
            write_bvh(nodes, tris);
        }
        else
        {
            std::vector<typename BvhNTriM<8, 4>::Node> nodes;
            std::vector<typename BvhNTriM<8, 4>::Tri> tris;
#ifdef ENABLE_EMBREE_BVH
            if (embree_bvh)
                build_embree_bvh<8>(tri_mesh, nodes, tris);
            else
#endif
                build_bvh<8, 4>(mesh, nodes, tris);
            write_bvh(nodes, tris);
        }
        std::ofstream bvh_stamp("data/bvh.stamp");
        bvh_stamp << int(info.Target) << " " << info.Filename;
    }
    else
    {
        ::info("Reusing existing BVH for '", info.Filename, "'");
    }
}

void convert_scene(const Scene &scene, const LoadInfo& info, std::ostream &os)
{
    setup_integrator(scene, info, os);
    // TODO: Extract camera "default" values
    setup_shapes(scene, info, os);
    // TODO: Write out materials and associate them with geometry
    // TODO: Write out lights and associate them with geometry
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