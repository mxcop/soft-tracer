#pragma once

#include "aabb.h"
#include "ray.h"
#include "vv.h"

/*
 * The BVH implementation is heavily inspired by a series of articles from Jacco Bikker.
 * Source: https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/
 */

class Bvh {
   public:
    /* NOTE: a cache line is usually 64 bytes */
    /* This node is 32 bytes, so 2 perfectly fit into a cache line */
    struct alignas(32) Node {
        union {
            f128 aabb_min4;
            struct {
                glm::vec3 aabb_min;
                u32 left_first;
            };
        };
        union {
            f128 aabb_max4;
            struct {
                glm::vec3 aabb_max;
                u32 prim_count;
            };
        };

        bool is_leaf() const { return prim_count; }
    };

   private:
    Node* nodes = nullptr;
    u16 root_idx = 0, nodes_used = 2;
    u16 size = 2;

    // TODO: use a vector of indices instead.
    // So we don't need to modify the actual vector.
    std::vector<VoxelVolume> prims;

    void subdivide(Bvh::Node& node, int lvl);

   public:
    Bvh(){};
    explicit Bvh(const std::vector<VoxelVolume>& new_prims);
    Bvh(u32 size, const std::vector<VoxelVolume>& new_prims);
    ~Bvh() {
        if (nodes) delete[] nodes;
    };

    void build(const std::vector<VoxelVolume>& prims);

    /**
     * @brief Evaluate the surface area heuristic of a node along an axis with a certain split
     * position.
     */
    f32 evaluate_sah(const Node& node, i32 axis, f32 pos) const;
    f32 find_best_split_plane(const Node& node, i32& axis, f32& pos) const;

    f32 intersect(const Ray& ray) const;
};
