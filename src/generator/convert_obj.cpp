#include "convert_obj.h"

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

static bool operator==(const obj::Material &a, const obj::Material &b)
{
    return a.ka == b.ka &&
           a.kd == b.kd &&
           a.ks == b.ks &&
           a.ke == b.ke &&
           a.ns == b.ns &&
           a.ni == b.ni &&
           a.tf == b.tf &&
           // ignored: a.tr == b.tr &&
           // ignored: a.d == b.d &&
           a.illum == b.illum &&
           // ignored: a.map_ka == b.map_ka &&
           a.map_kd == b.map_kd &&
           a.map_ks == b.map_ks &&
           a.map_ke == b.map_ke &&
           // ignored: a.map_bump == b.map_bump &&
           // ignored: a.map_d == b.map_d;
           true;
}

inline bool is_simple(const obj::Material &mat)
{
    return mat.illum != 5 && mat.illum != 7 &&           // Must be diffuse
           mat.ke == rgb(0.0f) && mat.map_ke == "" &&    // Must not be emitting
           mat.map_kd == "" && mat.map_ks == "" &&       // Must not contain any texture
           (mat.kd != rgb(0.0f) || mat.ks != rgb(0.0f)); // Must not be completely black
}

static size_t cleanup_obj(SpectralUpsampler* upsampler, obj::File &obj_file, obj::MaterialLib &mtl_lib)
{
    // Create a dummy material
    auto &dummy_mat = mtl_lib[""];
    dummy_mat.ka = upsampler->upsample_rgb(rgb(0.0f));
    dummy_mat.kd = upsampler->upsample_rgb(rgb(0.0f, 1.0f, 1.0f));
    dummy_mat.ks = upsampler->upsample_rgb(rgb(0.0f));
    dummy_mat.ke = upsampler->upsample_rgb(rgb(0.0f));
    dummy_mat.ns = 1.0f;
    dummy_mat.ni = 1.0f;
    dummy_mat.tf = upsampler->upsample_rgb(rgb(0.0f));
    dummy_mat.tr = 1.0f;
    dummy_mat.d = 1.0f;
    dummy_mat.illum = 2;
    dummy_mat.map_ka = "";
    dummy_mat.map_kd = "";
    dummy_mat.map_ks = "";
    dummy_mat.map_ke = "";
    dummy_mat.map_bump = "";
    dummy_mat.map_d = "";

    // Check that all materials exist
    for (auto &mtl_name : obj_file.materials)
    {
        if (mtl_name != "" && !mtl_lib.count(mtl_name))
        {
            warn("Missing material definition for '", mtl_name, "'. Replaced by dummy material.");
            mtl_name = "";
        }
    }

    // Remap identical materials (avoid duplicates)
    std::unordered_map<std::string, std::string> mtl_remap;
    for (size_t i = 0; i < obj_file.materials.size(); ++i)
    {
        auto &mtl1_name = obj_file.materials[i];
        auto &mtl1 = mtl_lib[mtl1_name];
        if (mtl_remap.count(mtl1_name) != 0)
            continue;
        for (size_t j = i + 1; j < obj_file.materials.size(); ++j)
        {
            auto &mtl2_name = obj_file.materials[j];
            auto &mtl2 = mtl_lib[mtl2_name];
            if (mtl1 == mtl2)
                mtl_remap.emplace(mtl2_name, mtl1_name);
        }
    }
    // Record unused materials
    std::unordered_set<std::string> used_mtls;
    for (auto &obj : obj_file.objects)
    {
        for (auto &group : obj.groups)
        {
            for (auto &face : group.faces)
            {
                auto mtl_name = obj_file.materials[face.material];
                auto it = mtl_remap.find(mtl_name);
                if (it != mtl_remap.end())
                    mtl_name = it->second;
                used_mtls.emplace(mtl_name);
            }
        }
    }

    // Remap indices/materials
    size_t num_complex = obj_file.materials.size();
    if (used_mtls.size() != obj_file.materials.size())
    {
        std::vector<std::string> new_materials = obj_file.materials;
        new_materials.erase(std::remove_if(new_materials.begin(), new_materials.end(), [&](auto &mtl_name) {
                                return used_mtls.count(mtl_name) == 0;
                            }),
                            new_materials.end());
        // Put simple materials at the end
        num_complex = std::partition(new_materials.begin(), new_materials.end(), [&](auto &mtl_name) {
                          return !is_simple(mtl_lib[mtl_name]);
                      }) -
                      new_materials.begin();
        std::vector<uint32_t> mtl_id_remap;
        for (auto mtl_name : obj_file.materials)
        {
            auto it = mtl_remap.find(mtl_name);
            if (it != mtl_remap.end())
                mtl_name = it->second;

            auto new_mtl_index = std::find(new_materials.begin(), new_materials.end(), mtl_name) - new_materials.begin();
            mtl_id_remap.emplace_back(new_mtl_index);
        }
        for (auto &obj : obj_file.objects)
        {
            for (auto &group : obj.groups)
            {
                for (auto &face : group.faces)
                {
                    assert(face.material < mtl_id_remap.size());
                    face.material = mtl_id_remap[face.material];
                    assert(face.material < new_materials.size());
                }
            }
        }
        std::swap(obj_file.materials, new_materials);
        info("Removed ", new_materials.size() - obj_file.materials.size(), " unused/duplicate material(s)");
        info("The scene has ", num_complex, " complex material(s), and ", new_materials.size() - num_complex, " simple material(s)");
    }
    return num_complex;
}

bool convert_obj(const std::string &file_name, Target target,
                size_t dev, size_t max_path_len, size_t spp, bool embree_bvh, bool fusion,
                SpectralUpsampler* upsampler, std::ostream &os)
{
    info("Converting OBJ file '", file_name, "'");
    obj::File obj_file;
    obj::MaterialLib mtl_lib;
    FilePath path(file_name);
    if (!obj::load_obj(path, obj_file))
    {
        error("Invalid OBJ file '", file_name, "'");
        return false;
    }
    for (auto lib_name : obj_file.mtl_libs)
    {
        auto mtl_name = path.base_name() + "/" + lib_name;
        if (!obj::load_mtl(mtl_name, mtl_lib))
        {
            error("Invalid MTL file '", mtl_name, "'");
            return false;
        }
    }

    size_t num_complex = cleanup_obj(upsampler, obj_file, mtl_lib);
    size_t num_mats = obj_file.materials.size();

    std::unordered_map<std::string, size_t> images;
    bool has_map_ke = false;
    for (auto &pair : mtl_lib)
    {
        auto &mat = pair.second;
        if (mat.map_kd != "")
            images.emplace(mat.map_kd, images.size());
        if (mat.map_ks != "")
            images.emplace(mat.map_kd, images.size());
        if (mat.map_ke != "")
            images.emplace(mat.map_ke, images.size()), has_map_ke = true;
    }

    auto tri_mesh = compute_tri_mesh(obj_file, 0);

    // Generate images
    std::vector<std::string> image_names(images.size());
    for (auto &pair : images)
        image_names[pair.second] = pair.first;

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

    bool enable_padding = target == Target::NVVM_STREAMING ||
                          target == Target::NVVM_MEGAKERNEL ||
                          target == Target::AMDGPU_STREAMING ||
                          target == Target::AMDGPU_MEGAKERNEL;
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

    os << "    let renderer = make_path_tracing_renderer(" << max_path_len << " /*max_path_len*/, " << spp << " /*spp*/);\n"
       //<< "    let renderer = make_whitefurnance_renderer();\n"
       << "    let math     = device.intrinsics;\n";

    // Setup camera
    os << "\n    // Camera\n"
       << "    let camera = make_perspective_camera(\n"
       << "        math,\n"
       << "        settings.eye,\n"
       << "        make_mat3x3(settings.right, settings.up, settings.dir),\n"
       << "        settings.width,\n"
       << "        settings.height\n"
       << "    );\n";

    // Setup triangle mesh
    info("Generating triangle mesh for '", file_name, "'");
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
       << "        num_tris:     " << tri_mesh.indices.size() / 4 << "\n"
       << "    };\n"
       << "    let bvh = device.load_bvh(\"data/bvh.bin\");\n";

    // Simplify materials if necessary
    num_complex = fusion ? num_complex : num_mats;
    bool has_simple = num_complex < num_mats;
    if (has_simple)
    {
        info("Simple materials will be fused");
        size_t num_tris = tri_mesh.indices.size() / 4;
        std::vector<float3> simple_kd(num_tris);
        std::vector<float3> simple_ks(num_tris);
        std::vector<float> simple_ns(num_tris);
        for (size_t i = 0, j = 0; i < tri_mesh.indices.size(); i += 4, j++)
        {
            auto &geom_id = tri_mesh.indices[i + 3];
            if (geom_id >= num_complex)
            {
                auto &mat = mtl_lib[obj_file.materials[geom_id]];
                assert(is_simple(mat));
                simple_kd[j] = upsampler->upsample_rgb(mat.kd);
                simple_ks[j] = upsampler->upsample_rgb(mat.ks);
                simple_ns[j] = mat.ns;
                geom_id = num_complex;
            }
            else
            {
                simple_kd[j] = upsampler->upsample_rgb(float3(0.1f, 0.05f, 0.01f));
                simple_ks[j] = upsampler->upsample_rgb(float3(0.1f, 0.05f, 0.01f));
                simple_ns[j] = 1.0f;
            }
        }
        write_buffer("data/simple_kd.bin", pad_buffer(simple_kd, enable_padding, sizeof(float) * 4));
        write_buffer("data/simple_ks.bin", pad_buffer(simple_ks, enable_padding, sizeof(float) * 4));
        write_buffer("data/simple_ns.bin", simple_ns);
    }

    write_tri_mesh(tri_mesh, enable_padding);

    // Generate BVHs
    if (must_build_bvh(file_name, target))
    {
        info("Generating BVH for '", file_name, "'");
        std::remove("data/bvh.bin");
        if (target == Target::NVVM_STREAMING || target == Target::NVVM_MEGAKERNEL ||
            target == Target::AMDGPU_STREAMING || target == Target::AMDGPU_MEGAKERNEL)
        {
            std::vector<typename BvhNTriM<2, 1>::Node> nodes;
            std::vector<typename BvhNTriM<2, 1>::Tri> tris;
            build_bvh<2, 1>(tri_mesh, nodes, tris);
            write_bvh(nodes, tris);
        }
        else if (target == Target::GENERIC || target == Target::ASIMD || target == Target::SSE42)
        {
            std::vector<typename BvhNTriM<4, 4>::Node> nodes;
            std::vector<typename BvhNTriM<4, 4>::Tri> tris;
#ifdef ENABLE_EMBREE_BVH
            if (embree_bvh)
                build_embree_bvh<4>(tri_mesh, nodes, tris);
            else
#endif
                build_bvh<4, 4>(tri_mesh, nodes, tris);
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
                build_bvh<8, 4>(tri_mesh, nodes, tris);
            write_bvh(nodes, tris);
        }
        std::ofstream bvh_stamp("data/bvh.stamp");
        bvh_stamp << int(target) << " " << file_name;
    }
    else
    {
        info("Reusing existing BVH for '", file_name, "'");
    }

    // Generate images
    info("Generating images for '", file_name, "'");
    os << "\n    // Images\n";
    for (size_t i = 0; i < images.size(); i++) {
        auto name = fix_file(image_names[i]);
        auto c_name = export_image(upsampler, path.base_name() + "/" + name);
        os << "    let image_" << make_id(name) << " = device.load_img(\"" << c_name.path() << "\");\n";
    }

    // Lights
    std::vector<int> light_ids(tri_mesh.indices.size() / 4, 0);
    os << "\n    // Lights\n";
    size_t num_lights = 0;
    std::vector<float3> light_colors;
    std::vector<float3> light_verts;
    std::vector<float3> light_norms;
    std::vector<float> light_areas;
    std::vector<float> light_powers;
    for (size_t i = 0; i < tri_mesh.indices.size(); i += 4)
    {
        // Do not leave this array undefined, even if this triangle is not a light
        light_ids[i / 4] = 0;

        auto &mtl_name = obj_file.materials[tri_mesh.indices[i + 3]];
        if (mtl_name == "")
            continue;
        auto &mat = mtl_lib.find(mtl_name)->second;
        if (mat.ke == rgb(0.0f) && mat.map_ke == "")
            continue;

        rgb kec;
        float kec_power;
        upsampler->upsample_emissive_rgb(mat.ke, kec, kec_power);
        
        auto &v0 = tri_mesh.vertices[tri_mesh.indices[i + 0]];
        auto &v1 = tri_mesh.vertices[tri_mesh.indices[i + 1]];
        auto &v2 = tri_mesh.vertices[tri_mesh.indices[i + 2]];

        light_ids[i / 4] = num_lights++;
        if (has_map_ke)
        {
            os << "    let light" << num_lights - 1 << " = make_triangle_light(\n"
               << "        math,\n"
               << "        make_vec3(" << v0.x << "f, " << v0.y << "f, " << v0.z << "f),\n"
               << "        make_vec3(" << v1.x << "f, " << v1.y << "f, " << v1.z << "f),\n"
               << "        make_vec3(" << v2.x << "f, " << v2.y << "f, " << v2.z << "f),\n";
            if (mat.map_ke != "")// TODO
            {
                os << "        make_texture(math, make_repeat_border(), make_bilinear_filter(), image_" << make_id(image_names[images[mat.map_ke]]) << ")\n";
            }
            else
            {
                os << "        make_colored_d65_illum(" << escape_f32(kec_power) << ", make_coeff_spectrum(math, " << escape_f32(kec.x) << ", " << escape_f32(kec.y) << ", " << escape_f32(kec.z) << "))\n";
            }
            os << "    );\n";
        }
        else
        {
            auto n = cross(v1 - v0, v2 - v0);
            auto inv_area = 1.0f / (0.5f * length(n));
            n = normalize(n);
            light_verts.emplace_back(v0);
            light_verts.emplace_back(v1);
            light_verts.emplace_back(v2);
            light_norms.emplace_back(n);
            light_areas.emplace_back(inv_area);
            light_colors.emplace_back(kec);
            light_powers.emplace_back(kec_power);
        }
    }
    if (has_map_ke || num_lights == 0)
    {
        if (num_lights != 0)
        {
            os << "    let lights = @ |i| match i {\n";
            for (size_t i = 0; i < num_lights; ++i)
            {
                if (i == num_lights - 1)
                    os << "        _ => light" << i << "\n";
                else
                    os << "        " << i << " => light" << i << ",\n";
            }
            os << "    };\n";
        }
        else
        {
            os << "    let lights = @ |_| make_point_light(math, make_vec3(0.0f, 0.0f, 0.0f), make_spectrum_none());\n";
        }
    }
    else
    {
        write_buffer("data/light_verts.bin", pad_buffer(light_verts, enable_padding, sizeof(float) * 4));
        write_buffer("data/light_areas.bin", light_areas);
        write_buffer("data/light_norms.bin", pad_buffer(light_norms, enable_padding, sizeof(float) * 4));
        write_buffer("data/light_colors.bin", pad_buffer(light_colors, enable_padding, sizeof(float) * 4));
        write_buffer("data/light_powers.bin", light_powers);

        os << "    let light_verts = device.load_buffer(\"data/light_verts.bin\");\n"
           << "    let light_areas = device.load_buffer(\"data/light_areas.bin\");\n"
           << "    let light_norms = device.load_buffer(\"data/light_norms.bin\");\n"
           << "    let light_colors = device.load_buffer(\"data/light_colors.bin\");\n"
           << "    let light_powers = device.load_buffer(\"data/light_powers.bin\");\n"
           << "    let lights = @ |i| {\n"
           << "        make_precomputed_triangle_light(\n"
           << "            math,\n"
           << "            light_verts.load_vec3(i * 3 + 0),\n"
           << "            light_verts.load_vec3(i * 3 + 1),\n"
           << "            light_verts.load_vec3(i * 3 + 2),\n"
           << "            light_norms.load_vec3(i),\n"
           << "            light_areas.load_f32(i),\n"
           << "            make_colored_d65_illum(light_powers.load_f32(i), make_coeff_spectrum_v(math, light_colors.load_vec3(i)))\n"
           << "        )\n"
           << "    };\n";
    }

    write_buffer("data/light_ids.bin", light_ids);

    os << "\n    // Mapping from primitive to light source\n"
       << "    let light_ids = device.load_buffer(\"data/light_ids.bin\");\n";

    // Generate shaders
    info("Generating materials for '", file_name, "'");
    os << "\n    // Shaders\n";
    for (auto &mtl_name : obj_file.materials)
    {
        auto it = mtl_lib.find(mtl_name);
        assert(it != mtl_lib.end());

        auto &mat = it->second;

        // Stop at the first simple material (they have been moved to the end of the array)
        if (has_simple && is_simple(mat))
            break;

        const auto ckd = upsampler->upsample_rgb(mat.kd);
        const auto cks = upsampler->upsample_rgb(mat.ks);
        const auto ctf = upsampler->upsample_rgb(mat.tf);

        bool has_emission = mat.ke != rgb(0.0f) || mat.map_ke != "";
        os << "    let shader_" << make_id(mtl_name) << " : Shader = @ |ray, hit, surf| {\n";
        if (mat.illum == 5)
        {
            os << "        let bsdf = make_mirror_bsdf(math, surf, make_coeff_spectrum(math, " << escape_f32(cks.x) << ", " << escape_f32(cks.y) << ", " << escape_f32(cks.z) << "));\n";
        }
        else if (mat.illum == 7)
        {
            os << "        let refrac_index =  make_const_refractive_index(" << mat.ni << "f);\n"
               << "        let bsdf = make_glass_bsdf(math, surf, make_const_refractive_index(1.0f), refrac_index, "
               << "make_coeff_spectrum(math, " << escape_f32(cks.x) << ", " << escape_f32(cks.y) << ", " << escape_f32(cks.z) << "), make_coeff_spectrum(math, " << escape_f32(ctf.x) << ", " << escape_f32(ctf.y) << ", " << escape_f32(ctf.z) << "));\n";
        }
        else
        {
            bool has_diffuse = mat.kd != rgb(0.0f) || mat.map_kd != "";
            bool has_specular = mat.ks != rgb(0.0f) || mat.map_ks != "";

            if (has_diffuse)
            {
                if (mat.map_kd != "")
                {
                    os << "        let diffuse_texture = make_texture(math, make_repeat_border(), make_bilinear_filter(), image_" << make_id(image_names[images[mat.map_kd]]) << ");\n";
                    os << "        let kd = diffuse_texture(vec4_to_2(surf.attr(0)));\n";
                }
                else
                {
                    os << "        let kd = make_coeff_spectrum(math, " << escape_f32(ckd.x) << ", " << escape_f32(ckd.y) << ", " << escape_f32(ckd.z) << ");\n";
                }
                os << "        let diffuse = make_diffuse_bsdf(math, surf, kd);\n";
            }
            if (has_specular)
            {
                if (mat.map_ks != "")
                {
                    os << "        let specular_texture = make_texture(math, make_repeat_border(), make_bilinear_filter(), image_" << make_id(image_names[images[mat.map_ks]]) << ");\n";
                    os << "        let ks = specular_texture(vec4_to_2(surf.attr(0)));\n";
                }
                else
                {
                    os << "        let ks = make_coeff_spectrum(math, " << escape_f32(cks.x) << ", " << escape_f32(cks.y) << ", " << escape_f32(cks.z) << ");\n";
                }
                os << "        let ns = " << escape_f32(mat.ns) << ";\n";
                os << "        let specular = make_phong_bsdf(math, surf, ks, ns);\n";
            }
            os << "        let bsdf = ";
            if (has_diffuse && has_specular)
            {
                os << "{\n"
                   << "            let lum_ks = ks.value(560.0f);\n"
                   << "            let lum_kd = kd.value(560.0f);\n"
                   << "            let k = select(lum_ks + lum_kd == 0.0f, 0.0f, lum_ks / (lum_ks + lum_kd));\n"
                   << "            make_mix_bsdf(diffuse, specular, k)\n"
                   << "        };\n";
            }
            else if (has_diffuse || has_specular)
            {
                if (has_specular)
                    os << "specular;\n";
                else
                    os << "diffuse;\n";
            }
            else
            {
                os << "make_black_bsdf();\n";
            }
        }
        if (has_emission)
        {
            os << "        make_emissive_material(surf, bsdf, lights(light_ids.load_i32(hit.prim_id)))\n";
        }
        else
        {
            os << "        make_material(bsdf)\n";
        }
        os << "    };\n";
    }

    if (has_simple)
    {
        os << "\n    // Simple materials data\n"
           << "    let simple_kd = device.load_buffer(\"data/simple_kd.bin\");\n"
           << "    let simple_ks = device.load_buffer(\"data/simple_ks.bin\");\n"
           << "    let simple_ns = device.load_buffer(\"data/simple_ns.bin\");\n";
    }

    // Generate geometries
    os << "\n    // Geometries\n"
       << "    let geometries = @ |i| match i {\n";
    for (uint32_t mat = 0; mat < num_complex; ++mat)
    {
        os << "        ";
        if (mat != num_complex - 1 || has_simple)
            os << mat;
        else
            os << "_";
        os << " => make_tri_mesh_geometry(math, tri_mesh, shader_" << make_id(obj_file.materials[mat]) << "),\n";
    }
    if (has_simple)
        os << "        _ => make_tri_mesh_geometry(math, tri_mesh, @ |ray, hit, surf| {\n"
           << "            let ckd = simple_kd.load_vec3(hit.prim_id);\n"
           << "            let cks = simple_ks.load_vec3(hit.prim_id);\n"
           << "            let kd = make_coeff_spectrum_v(ckd);\n"
           << "            let ks = make_coeff_spectrum_v(cks);\n"
           << "            let ns = simple_ns.load_f32(hit.prim_id);\n"
           << "            let diffuse = make_diffuse_bsdf(math, surf, kd);\n"
           << "            let specular = make_phong_bsdf(math, surf, ks, ns);\n"
           << "            let lum_ks = ckd.z;\n" // Approx?
           << "            let lum_kd = cks.z;\n"
           << "            make_material(make_mix_bsdf(diffuse, specular, lum_ks / (lum_ks + lum_kd)))\n"
           << "        })\n";
    os << "    };\n";

    // Scene
    os << "\n    // Scene\n"
       << "    let scene = Scene {\n"
       << "        num_geometries: " << std::min(num_complex + 1, num_mats) << ",\n"
       << "        num_lights:     " << num_lights << ",\n"
       << "        geometries:     @ |i| geometries(i),\n"
       << "        lights:         @ |i| lights(i),\n"
       << "        camera:         camera,\n"
       << "        bvh:            bvh\n"
       << "    };\n";

    os << "\n"
       << "    renderer(scene, device, iter);\n"
       << "    device.present();\n"
       << "}\n";

    info("Scene was converted successfully");
    return true;
}