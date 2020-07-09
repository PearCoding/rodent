#pragma once

#include <vector>

#include "float3.h"
#include "color.h"
#include "file_path.h"

namespace mesh {

struct TriMesh {
    std::vector<float3>   vertices;
    std::vector<uint32_t> indices;
    std::vector<float3>   normals;
    std::vector<float3>   face_normals;
    std::vector<float>    face_area;
    std::vector<float2>   texcoords;
};

void compute_face_normals(const std::vector<uint32_t>& indices,
                                 const std::vector<float3>& vertices,
                                 std::vector<float3>& face_normals,
                                 std::vector<float>& face_area,
                                 size_t first_index);
void compute_vertex_normals(const std::vector<uint32_t>& indices,
                                const std::vector<float3>& face_normals,
                                std::vector<float3>& normals,
                                size_t first_index);
void fix_normals(TriMesh& tri_mesh);
void flip_normals(TriMesh& tri_mesh);
void scale(TriMesh& tri_mesh, float scale);
void merge(TriMesh& dst, const TriMesh& src);
void replace_material(TriMesh& tri_mesh, uint32_t m_idx);

} // namespace mesh
