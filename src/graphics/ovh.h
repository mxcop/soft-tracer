#pragma once

#include "aabb.h"
#include "ray.h"
#include "vv.h"

struct AABB_256 {
    union {
        f256 corners[2][3];
        struct {
            f256 min[3];
            f256 max[3];
        };
    };
};

struct RAY_256 {
    union {
        f256 cache[6];
        struct {
            f256 origin[3];
            f256 dir_inv[3];
        };
    };
};

class Ovh {
   public:
    struct Node {
        AABB aabb;
        /* If this is a leaf, it will have the primitive AABB */
        AABB_256 prim_aabb;
        /* If this is not a leaf, it will have the children AABB */
        AABB_256 child_aabb;

        /* Child indices */
        u32 childern[8]{};
        u32 first_prim, prim_count;
        bool leaf = false;

        bool is_leaf() const { return leaf; }
    };

   private:
    Node* nodes = nullptr;
    u32 root_idx = 0, nodes_used = 1;
    u32 size = 1;

    // TODO: use a vector of indices instead.
    // So we don't need to modify the actual vector.
    std::vector<VoxelVolume> prims;
    // std::vector<u32> indices;

    void update_node_bb(Node& node);

    void subdivide(Node& node, int lvl);
    void build(const std::vector<VoxelVolume>& prims);

   public:
    Ovh(){};
    Ovh(u32 size, const std::vector<VoxelVolume>& new_prims);

    bool intersect(const Ray& ray) const;
};
