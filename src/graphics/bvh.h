#pragma once

#include "aabb.h"
#include "ray.h"
#include "vv.h"

class Bvh {
   public:
    /* NOTE: a cache line is usually 64 bytes */
    /* This node is 32 bytes, so 2 perfectly fit into a cache line */
    struct Node {
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
    u16 root_idx = 0, nodes_used = 1;
    u16 size = 1;

    // TODO: use a vector of indices instead.
    // So we don't need to modify the actual vector.
    std::vector<VoxelVolume> prims;

    void subdivide(Bvh::Node& node, int lvl);
    void build(const std::vector<VoxelVolume>& prims);

    // bool intersect(const Ray& ray, u16 node_idx) const;

   public:
    Bvh(){};
    Bvh(u32 size, const std::vector<VoxelVolume>& new_prims);

    bool intersect(const Ray& ray) const;
};
