#pragma once

#include "aabb.h"
#include "ray.h"
#include "vv.h"

#include <xmmintrin.h>
#include <immintrin.h>

typedef __m256 f256;

struct AABB_256 {
    union {
        f256 corners[2][3];
        struct {
            f256 min[3];
            f256 max[3];
        };
    };
};

class Bvh {
   public:
    struct Node {
        AABB aabb;
        AABB_256* prim_aabb = nullptr;
        u32 left_child, right_child;
        u32 first_prim, prim_count;

        ~Node() { if (prim_aabb) delete prim_aabb; }
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
