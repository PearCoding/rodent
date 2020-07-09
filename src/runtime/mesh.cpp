#include <fstream>
#include <iostream> 
#include <cstring>
#include <cstdlib>
#include <cctype>

#include "common.h"
#include "mesh.h"

namespace mesh {

void compute_face_normals(const std::vector<uint32_t>& indices,
                                 const std::vector<float3>& vertices,
                                 std::vector<float3>& face_normals,
                                 std::vector<float>& face_area,
                                 size_t first_index) {
    bool hasBadArea = false;
    for (auto i = first_index, k = indices.size(); i < k; i += 4) {
        const float3& v0 = vertices[indices[i + 0]];
        const float3& v1 = vertices[indices[i + 1]];
        const float3& v2 = vertices[indices[i + 2]];
        const float3 N   = cross(v1 - v0, v2 - v0);
        float lN   = length(N);
        if(lN < 0.00000001f) {
            lN = 1.0f;
            hasBadArea = true;
        }
        face_normals[i / 4] = N / lN;
        face_area[i / 4] = 0.5f * lN;
    }

    if(hasBadArea)
        warn("Triangle mesh contains triangles with zero area");
}

void compute_vertex_normals(const std::vector<uint32_t>& indices,
                                   const std::vector<float3>& face_normals,
                                   std::vector<float3>& normals,
                                   size_t first_index) {
    for (auto i = first_index, k = indices.size(); i < k; i += 4) {
        float3& n0 = normals[indices[i + 0]];
        float3& n1 = normals[indices[i + 1]];
        float3& n2 = normals[indices[i + 2]];
        const float3& n = face_normals[i / 4];
        n0 += n;
        n1 += n;
        n2 += n;
    }
}

void fix_normals(TriMesh& tri_mesh) {
    // Re-normalize all the values in the OBJ file to handle invalid meshes
    bool fixed_normals = false;
    for (auto& n : tri_mesh.normals) {
        auto len2 = lensqr(n);
        if (len2 <= std::numeric_limits<float>::epsilon() || std::isnan(len2)) {
            fixed_normals = true;
            n = float3(0.0f, 1.0f, 0.0f);
        } else
            n = n * (1.0f / std::sqrt(len2));
    }

    if (fixed_normals)
        warn("Some normals were incorrect and thus had to be replaced with arbitrary values.");
}

void flip_normals(TriMesh& tri_mesh)
{
    for (auto& n : tri_mesh.face_normals)
        n = -n;
    for (auto& n : tri_mesh.normals)
        n = -n;
}

void scale(TriMesh& tri_mesh, float scale)
{
    for (auto& v : tri_mesh.vertices)
        v *= scale;
}

void merge(TriMesh& dst, const TriMesh& src) {
    size_t idx_offset = dst.indices.size();
    size_t vtx_offset = dst.vertices.size();

    if(idx_offset == 0) {
        dst = src;
        return;
    }

    dst.vertices.insert(dst.vertices.end(),  src.vertices.begin(), src.vertices.end());
    dst.normals.insert(dst.normals.end(),  src.normals.begin(), src.normals.end());
    dst.texcoords.insert(dst.texcoords.end(),  src.texcoords.begin(), src.texcoords.end());
    dst.face_normals.insert(dst.face_normals.end(),  src.face_normals.begin(), src.face_normals.end());
    dst.face_area.insert(dst.face_area.end(),  src.face_area.begin(), src.face_area.end());
    
    dst.indices.resize(idx_offset + src.indices.size());
    for(size_t i = 0; i < src.indices.size(); i += 4) {
        dst.indices[idx_offset + i + 0] = src.indices[i + 0] + vtx_offset;
        dst.indices[idx_offset + i + 1] = src.indices[i + 1] + vtx_offset;
        dst.indices[idx_offset + i + 2] = src.indices[i + 2] + vtx_offset;
        dst.indices[idx_offset + i + 3] = src.indices[i + 3]; // Material
    }
}

void replace_material(TriMesh& tri_mesh, uint32_t m_idx) {
    for(size_t i = 0; i < tri_mesh.indices.size(); i += 4)
       tri_mesh.indices[i + 3] = m_idx; // Material
}
} // namespace obj
