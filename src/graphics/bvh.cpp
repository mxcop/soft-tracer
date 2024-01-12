#include "bvh.h"

#include <utility> /* std::swap */

#include "vv.h"
#include "ray.h"

Bvh::Bvh(u32 size, const std::vector<VoxelVolume>& new_prims)
    : size(size), nodes(new Node[size * 2 - 1]{}) {
    build(new_prims);
}

void update_node_bb(Bvh::Node& node, const std::vector<VoxelVolume>& prims) {
    node.aabb_min = glm::vec3(BIG_F32);
    node.aabb_max = glm::vec3(-BIG_F32);
    for (u32 i = 0; i < node.prim_count; ++i) {
        const VoxelVolume& prim = prims[node.left_first + i];

        node.aabb_min.x = std::min(node.aabb_min.x, prim.aabb_min.x);
        node.aabb_min.y = std::min(node.aabb_min.y, prim.aabb_min.y);
        node.aabb_min.z = std::min(node.aabb_min.z, prim.aabb_min.z);
        node.aabb_max.x = std::max(node.aabb_max.x, prim.aabb_max.x);
        node.aabb_max.y = std::max(node.aabb_max.y, prim.aabb_max.y);
        node.aabb_max.z = std::max(node.aabb_max.z, prim.aabb_max.z);
    }
}

f32 Bvh::evaluate_sah(const Node& node, i32 axis, f32 pos) const {
    AABB l_aabb, r_aabb;
    i32 l_cnt = 0, r_cnt = 0;
    for (u32 i = 0; i < node.prim_count; ++i) {
        const VoxelVolume& prim = prims[node.left_first + i];
        if (prim.center()[axis] < pos) {
            l_cnt++;
            l_aabb.grow(prim.aabb_min);
            l_aabb.grow(prim.aabb_max);
        } else {
            r_cnt++;
            r_aabb.grow(prim.aabb_min);
            r_aabb.grow(prim.aabb_max);
        }
    }
    f32 cost = l_cnt * l_aabb.area() + r_cnt * r_aabb.area();
    return cost > 0.0f ? cost : BIG_F32;
}

void Bvh::subdivide(Bvh::Node& node, int lvl) {
    printf("node: {%i} [%i]\n", lvl, node.prim_count);
    if (node.prim_count <= 2u) return;

    /* Determine split based on SAH */
    i32 best_axis = -1;
    f32 best_pos = 0, best_cost = BIG_F32;
    for (i32 axis = 0; axis < 3; ++axis) {
        for (u32 i = 0; i < node.prim_count; ++i) {
            const VoxelVolume& prim = prims[node.left_first + i];
            f32 candidate_pos = prim.center()[axis];
            f32 cost = evaluate_sah(node, axis, candidate_pos);
            if (cost < best_cost) best_pos = candidate_pos, best_axis = axis, best_cost = cost;
        }
    }

    /* Calculate parent node cost */
    glm::vec3 e = node.aabb_max - node.aabb_min;
    f32 parent_area = e.x * e.y + e.y * e.z + e.z * e.x;
    f32 parent_cost = node.prim_count * parent_area;
    if (best_cost >= parent_cost) return; /* Split would not be worth it */

    /* Determine which primitives lie on which side */
    int i = node.left_first;
    int j = i + node.prim_count - 1;
    while (i <= j) {
        if (prims[i].center()[best_axis] < best_pos) {
            i++;
        } else {
            std::swap(prims[i], prims[j--]);
        }
    }

    int left_count = i - node.left_first;
    if (left_count == 0 || left_count == node.prim_count) return;

    /* Initialize child nodes */
    int left_child_idx = nodes_used++;
    int right_child_idx = nodes_used++;
    nodes[left_child_idx].left_first = node.left_first;
    nodes[left_child_idx].prim_count = left_count;
    nodes[right_child_idx].left_first = i;
    nodes[right_child_idx].prim_count = node.prim_count - left_count;
    node.left_first = left_child_idx;
    node.prim_count = 0;

    update_node_bb(nodes[left_child_idx], prims);
    update_node_bb(nodes[right_child_idx], prims);

    /* Continue subdiving */
    subdivide(nodes[left_child_idx], lvl + 1);
    subdivide(nodes[right_child_idx], lvl + 1);
}

void Bvh::build(const std::vector<VoxelVolume>& new_prims) {
    prims = new_prims;

    /* Initialize the root node */
    Node& root = nodes[root_idx];
    root.left_first = 0, root.prim_count = size;
    update_node_bb(root, prims);

    /* Start the recursive subdivide */
    subdivide(root, 0);
}

bool Bvh::intersect(const Ray& ray) const {
    const Node *node = &nodes[root_idx], *node_stack[64];
    u32 stack_ptr = 0;
    for (;;) {
        /* If the current node is a leaf... */
        if (node->is_leaf()) {
            /* Check if we hit any primitives */
            for (u32 i = 0; i < node->prim_count; i++) {
                const VoxelVolume& prim = prims[node->left_first + i];
                float dist = ray.intersects_aabb_sse(prim.aabb_min4, prim.aabb_max4);
                if (dist < BIG_F32) return true;
            }

            /* Decend down the stack */
            if (stack_ptr == 0) break;
            node = node_stack[--stack_ptr];
            continue;
        }

        /* In case we're not a leaf, see if we intersect the child nodes */
        const Node* child1 = &nodes[node->left_first];
        const Node* child2 = &nodes[node->left_first + 1];
        float dist1 = ray.intersects_aabb_sse(child1->aabb_min4, child1->aabb_max4);
        float dist2 = ray.intersects_aabb_sse(child2->aabb_min4, child2->aabb_max4);

        /* Put the smallest distance upfront */
        if (dist1 > dist2) {
            std::swap(dist1, dist2);
            std::swap(child1, child2);
        }

        /* Add child nodes to be checks if they were intersected */
        if (dist1 == BIG_F32) {
            if (stack_ptr == 0) break;
            node = node_stack[--stack_ptr];
        } else {
            node = child1;
            if (dist2 != BIG_F32) node_stack[stack_ptr++] = child2;
        }
    }

    return false;
}
