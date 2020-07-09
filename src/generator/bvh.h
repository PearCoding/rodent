#pragma once

#include "driver/interface.h"
#include "runtime/bvh.h"
#include "runtime/obj.h"
#include "runtime/buffer.h"

#ifdef ENABLE_EMBREE_BVH
#include "runtime/embree_bvh.h"
#endif

#include "target.h"

template <size_t N, size_t M>
struct BvhNTriM
{
};

template <>
struct BvhNTriM<8, 4>
{
    using Node = Node8;
    using Tri = Tri4;
};

template <>
struct BvhNTriM<4, 4>
{
    using Node = Node4;
    using Tri = Tri4;
};

template <>
struct BvhNTriM<2, 1>
{
    using Node = Node2;
    using Tri = Tri1;
};

template <size_t N, size_t M>
class BvhNTriMAdapter
{
    struct CostFn
    {
        static float leaf_cost(int count, float area)
        {
            return count * area;
        }
        static float traversal_cost(float area)
        {
            return area;
        }
    };

    using BvhBuilder = SplitBvhBuilder<N, CostFn>;
    using Adapter = BvhNTriMAdapter;
    using Node = typename BvhNTriM<N, M>::Node;
    using Tri = typename BvhNTriM<N, M>::Tri;

    std::vector<Node> &nodes_;
    std::vector<Tri> &tris_;
    BvhBuilder builder_;

public:
    BvhNTriMAdapter(std::vector<Node> &nodes, std::vector<Tri> &tris)
        : nodes_(nodes), tris_(tris)
    {
    }

    void build(const mesh::TriMesh &tri_mesh, const std::vector<::Tri> &tris)
    {
        builder_.build(tris, NodeWriter(*this), LeafWriter(*this, tris, tri_mesh.indices), M / 2);
    }

#ifdef STATISTICS
    void print_stats() const override
    {
        builder_.print_stats();
    }
#endif

private:
    struct NodeWriter
    {
        Adapter &adapter;

        NodeWriter(Adapter &adapter)
            : adapter(adapter)
        {
        }

        template <typename BBoxFn>
        int operator()(int parent, int child, const BBox & /*parent_bb*/, size_t count, BBoxFn bboxes)
        {
            auto &nodes = adapter.nodes_;

            size_t i = nodes.size();
            nodes.emplace_back();

            if (parent >= 0 && child >= 0)
            {
                assert(parent >= 0 && parent < nodes.size());
                assert(child >= 0 && child < N);
                nodes[parent].child[child] = i + 1;
            }

            assert(count >= 2 && count <= N);

            for (size_t j = 0; j < count; j++)
            {
                const BBox &bbox = bboxes(j);
                nodes[i].bounds[0][j] = bbox.min.x;
                nodes[i].bounds[2][j] = bbox.min.y;
                nodes[i].bounds[4][j] = bbox.min.z;

                nodes[i].bounds[1][j] = bbox.max.x;
                nodes[i].bounds[3][j] = bbox.max.y;
                nodes[i].bounds[5][j] = bbox.max.z;
            }

            for (size_t j = count; j < N; ++j)
            {
                nodes[i].bounds[0][j] = std::numeric_limits<float>::infinity();
                nodes[i].bounds[2][j] = std::numeric_limits<float>::infinity();
                nodes[i].bounds[4][j] = std::numeric_limits<float>::infinity();

                nodes[i].bounds[1][j] = -std::numeric_limits<float>::infinity();
                nodes[i].bounds[3][j] = -std::numeric_limits<float>::infinity();
                nodes[i].bounds[5][j] = -std::numeric_limits<float>::infinity();

                nodes[i].child[j] = 0;
            }

            return i;
        }
    };

    struct LeafWriter
    {
        Adapter &adapter;
        const std::vector<::Tri> &in_tris;
        const std::vector<uint32_t> &indices;

        LeafWriter(Adapter &adapter, const std::vector<::Tri> &in_tris, const std::vector<uint32_t> &indices)
            : adapter(adapter), in_tris(in_tris), indices(indices)
        {
        }

        template <typename RefFn>
        void operator()(int parent, int child, const BBox & /*leaf_bb*/, size_t ref_count, RefFn refs)
        {
            auto &nodes = adapter.nodes_;
            auto &tris = adapter.tris_;

            nodes[parent].child[child] = ~tris.size();

            // Group triangles by packets of M
            for (size_t i = 0; i < ref_count; i += M)
            {
                const size_t c = i + M <= ref_count ? M : ref_count - i;

                Tri tri;
                std::memset(&tri, 0, sizeof(Tri));
                for (size_t j = 0; j < c; j++)
                {
                    const int id = refs(i + j);
                    auto &in_tri = in_tris[id];
                    const float3 e1 = in_tri.v0 - in_tri.v1;
                    const float3 e2 = in_tri.v2 - in_tri.v0;
                    const float3 n = cross(e1, e2);
                    tri.v0[0][j] = in_tri.v0.x;
                    tri.v0[1][j] = in_tri.v0.y;
                    tri.v0[2][j] = in_tri.v0.z;

                    tri.e1[0][j] = e1.x;
                    tri.e1[1][j] = e1.y;
                    tri.e1[2][j] = e1.z;

                    tri.e2[0][j] = e2.x;
                    tri.e2[1][j] = e2.y;
                    tri.e2[2][j] = e2.z;

                    tri.n[0][j] = n.x;
                    tri.n[1][j] = n.y;
                    tri.n[2][j] = n.z;

                    tri.prim_id[j] = id;
                    tri.geom_id[j] = indices[id * 4 + 3];
                }

                for (size_t j = c; j < 4; j++)
                    tri.prim_id[j] = 0xFFFFFFFF;

                tris.emplace_back(tri);
            }
            assert(ref_count > 0);
            tris.back().prim_id[M - 1] |= 0x80000000;
        }
    };
};

template <>
class BvhNTriMAdapter<2, 1>
{
    struct CostFn
    {
        static float leaf_cost(int count, float area)
        {
            return count * area;
        }
        static float traversal_cost(float area)
        {
            return area;
        }
    };

    using BvhBuilder = SplitBvhBuilder<2, CostFn>;
    using Adapter = BvhNTriMAdapter;
    using Node = Node2;
    using Tri = Tri1;

    std::vector<Node> &nodes_;
    std::vector<Tri> &tris_;
    BvhBuilder builder_;

public:
    BvhNTriMAdapter(std::vector<Node> &nodes, std::vector<Tri> &tris)
        : nodes_(nodes), tris_(tris)
    {
    }

    void build(const mesh::TriMesh &tri_mesh, const std::vector<::Tri> &tris)
    {
        builder_.build(tris, NodeWriter(*this), LeafWriter(*this, tris, tri_mesh.indices), 2);
    }

#ifdef STATISTICS
    void print_stats() const override
    {
        builder_.print_stats();
    }
#endif

private:
    struct NodeWriter
    {
        Adapter &adapter;

        NodeWriter(Adapter &adapter)
            : adapter(adapter)
        {
        }

        template <typename BBoxFn>
        int operator()(int parent, int child, const BBox & /*parent_bb*/, size_t count, BBoxFn bboxes)
        {
            auto &nodes = adapter.nodes_;

            size_t i = nodes.size();
            nodes.emplace_back();

            if (parent >= 0 && child >= 0)
            {
                assert(parent >= 0 && parent < nodes.size());
                assert(child >= 0 && child < 2);
                nodes[parent].child[child] = i + 1;
            }

            assert(count >= 1 && count <= 2);

            const BBox &bbox1 = bboxes(0);
            nodes[i].bounds[0] = bbox1.min.x;
            nodes[i].bounds[2] = bbox1.min.y;
            nodes[i].bounds[4] = bbox1.min.z;
            nodes[i].bounds[1] = bbox1.max.x;
            nodes[i].bounds[3] = bbox1.max.y;
            nodes[i].bounds[5] = bbox1.max.z;

            if (count == 2)
            {
                const BBox &bbox2 = bboxes(1);
                nodes[i].bounds[6] = bbox2.min.x;
                nodes[i].bounds[8] = bbox2.min.y;
                nodes[i].bounds[10] = bbox2.min.z;
                nodes[i].bounds[7] = bbox2.max.x;
                nodes[i].bounds[9] = bbox2.max.y;
                nodes[i].bounds[11] = bbox2.max.z;
            }
            else
            {
                nodes[i].bounds[6] = std::numeric_limits<float>::infinity();
                nodes[i].bounds[8] = std::numeric_limits<float>::infinity();
                nodes[i].bounds[10] = std::numeric_limits<float>::infinity();
                nodes[i].bounds[7] = -std::numeric_limits<float>::infinity();
                nodes[i].bounds[9] = -std::numeric_limits<float>::infinity();
                nodes[i].bounds[11] = -std::numeric_limits<float>::infinity();
            }

            return i;
        }
    };

    struct LeafWriter
    {
        Adapter &adapter;
        const std::vector<::Tri> &in_tris;
        const std::vector<uint32_t> &indices;

        LeafWriter(Adapter &adapter, const std::vector<::Tri> &in_tris, const std::vector<uint32_t> &indices)
            : adapter(adapter), in_tris(in_tris), indices(indices)
        {
        }

        template <typename RefFn>
        void operator()(int parent, int child, const BBox & /*leaf_bb*/, size_t ref_count, RefFn refs)
        {
            auto &nodes = adapter.nodes_;
            auto &tris = adapter.tris_;

            nodes[parent].child[child] = ~tris.size();

            for (int i = 0; i < ref_count; i++)
            {
                const int ref = refs(i);
                auto &tri = in_tris[ref];
                auto e1 = tri.v0 - tri.v1;
                auto e2 = tri.v2 - tri.v0;
                auto n = cross(e1, e2);
                int geom_id = indices[ref * 4 + 3];
                tris.emplace_back(Tri1{
                    {tri.v0.x, tri.v0.y, tri.v0.z}, 0, {e1.x, e1.y, e1.z}, geom_id, {e2.x, e2.y, e2.z}, ref});
            }

            // Add sentinel
            tris.back().prim_id |= 0x80000000;
        }
    };
};

template <typename T>
inline std::vector<uint8_t> pad_buffer(const std::vector<T> &elems, bool enable, size_t size)
{
    std::vector<uint8_t> new_elems;
    if (!enable)
    {
        new_elems.resize(sizeof(T) * elems.size());
        memcpy(new_elems.data(), elems.data(), sizeof(T) * elems.size());
        return new_elems;
    }
    assert(size >= sizeof(T));
    new_elems.resize(size * elems.size(), 0);
    uint8_t *ptr = new_elems.data();
    for (auto &elem : elems)
    {
        memcpy(ptr, &elem, sizeof(T));
        ptr += size;
    }
    return new_elems;
}

inline void write_tri_mesh(const mesh::TriMesh &tri_mesh, bool enable_padding)
{
    write_buffer("data/vertices.bin", pad_buffer(tri_mesh.vertices, enable_padding, sizeof(float) * 4));
    write_buffer("data/normals.bin", pad_buffer(tri_mesh.normals, enable_padding, sizeof(float) * 4));
    write_buffer("data/face_normals.bin", pad_buffer(tri_mesh.face_normals, enable_padding, sizeof(float) * 4));
    write_buffer("data/face_area.bin", tri_mesh.face_area);
    write_buffer("data/indices.bin", tri_mesh.indices);
    write_buffer("data/texcoords.bin", pad_buffer(tri_mesh.texcoords, enable_padding, sizeof(float) * 4));
}

template <size_t N, size_t M>
inline void build_bvh(const mesh::TriMesh &tri_mesh,
                      std::vector<typename BvhNTriM<N, M>::Node> &nodes,
                      std::vector<typename BvhNTriM<N, M>::Tri> &tris)
{
    BvhNTriMAdapter<N, M> adapter(nodes, tris);
    auto num_tris = tri_mesh.indices.size() / 4;
    std::vector<::Tri> in_tris(num_tris);
    for (size_t i = 0; i < num_tris; i++)
    {
        auto &v0 = tri_mesh.vertices[tri_mesh.indices[i * 4 + 0]];
        auto &v1 = tri_mesh.vertices[tri_mesh.indices[i * 4 + 1]];
        auto &v2 = tri_mesh.vertices[tri_mesh.indices[i * 4 + 2]];
        in_tris[i] = Tri(v0, v1, v2);
    }
    adapter.build(tri_mesh, in_tris);
}

template <typename Node, typename Tri>
inline void write_bvh(std::vector<Node> &nodes, std::vector<Tri> &tris)
{
    std::ofstream of("data/bvh.bin", std::ios::app | std::ios::binary);
    size_t node_size = sizeof(Node);
    size_t tri_size = sizeof(Tri);
    of.write((char *)&node_size, sizeof(uint32_t));
    of.write((char *)&tri_size, sizeof(uint32_t));
    write_buffer(of, nodes);
    write_buffer(of, tris);
    info("BVH with ", nodes.size(), " node(s), ", tris.size(), " tri(s)");
}

inline bool must_build_bvh(const std::string &name, Target target)
{
    std::ifstream bvh_stamp("data/bvh.stamp", std::fstream::in);
    if (bvh_stamp)
    {
        int bvh_target;
        bvh_stamp >> bvh_target;
        if (bvh_target != (int)target)
            return true;
        std::string bvh_name;
        bvh_stamp >> bvh_name;
        if (bvh_name != name)
            return true;
        return false;
    }
    return true;
}