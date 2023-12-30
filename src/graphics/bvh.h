#pragma once

#include "aabb.h"
#include "ray.h"
#include "vv.h"

class Bvh {
   public:
    struct Node {
        AABB aabb;
        u32 left_child, right_child;
        u32 first_prim, prim_count;
    };

   private:
    Node* nodes = nullptr;
    u32 root_idx = 0, nodes_used = 1;
    u32 size = 1;

    // TODO: use a vector of indices instead.
    // So we don't need to modify the actual vector.
    std::vector<VoxelVolume> prims;

    void subdivide(Bvh::Node& node, int lvl);
    void build(const std::vector<VoxelVolume>& prims);

    bool intersect(const Ray& ray, u32 node_idx) const;

   public:
    Bvh(){};
    Bvh(u32 size, const std::vector<VoxelVolume>& new_prims);

    bool intersect(const Ray& ray) const;
};
