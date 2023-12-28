#include "bvh.h"

Bvh::Bvh(u32 size) : size(size), nodes(new Node[size * 2 - 1]{}) { build(); }

void update_node_bb(Bvh::Node& node, const std::vector<VoxelVolume>& prim) {
    node.bbmin = glm::vec3(1e30f), node.bbmax = glm::vec3(-1e30f);
    for (u32 first = node.first_prim, i = 0; i < node.prim_count; ++i) {
        const VoxelVolume& primative = prim[first + i];
        //node.bbmin.x = std::min(node.bbmin.x, primative.min.x);
        //node.bbmin.y = std::min(node.bbmin.y, primative.min.y);
        //node.bbmin.z = std::min(node.bbmin.z, primative.min.z);
        //node.bbmax.x = std::max(node.bbmax.x, primative.max.x);
        //node.bbmax.y = std::max(node.bbmax.y, primative.max.y);
        //node.bbmax.z = std::max(node.bbmax.z, primative.max.z);
        
        // TODO: use VoxelVolume.get_bounds() to get the min and max.
    }
}

void Bvh::build(const std::vector<VoxelVolume>& prim) {
    /* initialize the root node */
    Node& root = nodes[root_idx];
    root.left_child = root.right_child = 0;
    root.first_prim = 0, root.prim_count = size;
    update_node_bb(root, prim);

    /* start the recursive subdivide */

    // TODO: write recursive subdivide(); function to build the BVH.
}
