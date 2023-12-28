#pragma once

#include "vv.h"

class Bvh {
   public:
    struct Node {
        /* Bounding box */
        glm::vec3 bbmin, bbmax;
        u32 left_child, right_child;
        u32 first_prim, prim_count;
    };

   private:
    Node* nodes;
    u32 root_idx = 0, nodes_used = 1;
    u32 size = 1;

    void build(const std::vector<VoxelVolume>& prim);

   public:
    Bvh(u32 size);
};
