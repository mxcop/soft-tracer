#pragma once

#include "aabb.h"
#include "ray.h"
#include "vv.h"

struct alignas(32) AABB_256 {
    union {
        f256 corners[2][3];
        struct {
            f256 min[3];
            f256 max[3];
        };
    };
};

/**
 * @brief Octuple Volume Hierarchy.
 * A Bounding Volume Hierarchy where each node,
 * has 8 child nodes, optimized for AVX 256 bit registers.
 */
class Ovh {
   public:
    struct Node {
        /* If this is a leaf, it will have the primitive AABB */
        /* If this is not a leaf, it will have the children AABB */
        AABB_256 x8_aabb;

        /* Index of the first child node (-1 if no children) */
        i32 first_child;
        bool is_leaf() const { return first_child < 0; }
    };

   private:
    Node* nodes = nullptr;
    u32 root_idx = 0, nodes_used = 1;
    u32 size = 1;

    // TODO: use a vector of indices instead.
    // So we don't need to modify the actual vector.
    std::vector<VoxelVolume> prims;

    /**
     * @brief Compute the bounding box of a node.
     */
    AABB compute_node_bb(const Node& node, u32 first_prim, u32 prim_count) const;

    void subdivide(Node& node, const AABB& node_bb, u32 first_prim, u32 prim_count, int lvl);
    void build(const std::vector<VoxelVolume>& prims);

   public:
    Ovh(){};
    Ovh(u32 size, const std::vector<VoxelVolume>& new_prims);

    bool intersect(const Ray& ray) const;
};
