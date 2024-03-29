#include "bvh.h"

#include <utility> /* std::swap */

#include "vv.h"
#include "ray.h"

Bvh::Bvh(const std::vector<VoxelVolume>& new_prims) : size(new_prims.size()) { build(new_prims); }
Bvh::Bvh(u32 size, const std::vector<VoxelVolume>& new_prims) : size(size) { build(new_prims); }

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

#define SAH_FULL_SWEEP 0
#define SAH_BINS 1
struct Bin {
    AABB aabb;
    u32 prim_count = 0;
};

f32 Bvh::find_best_split_plane(const Node& node, i32& axis, f32& pos) const {
    f32 best_cost = BIG_F32;
#if SAH_FULL_SWEEP
    for (i32 a = 0; a < 3; ++a) {
        for (u32 i = 0; i < node.prim_count; ++i) {
            const VoxelVolume& prim = prims[node.left_first + i];
            f32 candidate_pos = prim.center()[a];
            f32 cost = evaluate_sah(node, a, candidate_pos);
            if (cost < best_cost) pos = candidate_pos, axis = a, best_cost = cost;
        }
    }
#elif SAH_BINS
    constexpr i32 BINS = 8;
    for (i32 a = 0; a < 3; ++a) {
        /* Get the min and max of all primitives in the node */
        f32 bmin = BIG_F32, bmax = -BIG_F32;
        for (u32 i = 0; i < node.prim_count; ++i) {
            const glm::vec3 prim_center = prims[node.left_first + i].center();
            bmin = std::min(bmin, prim_center[a]);
            bmax = std::max(bmax, prim_center[a]);
        }
        if (bmin == bmax) continue;

        /* Populate the bins */
        Bin bins[BINS];
        f32 scale = BINS / (bmax - bmin);
        for (u32 i = 0; i < node.prim_count; ++i) {
            const VoxelVolume& prim = prims[node.left_first + i];
            i32 bin_idx = std::min(BINS - 1, static_cast<i32>((prim.center()[a] - bmin) * scale));
            bins[bin_idx].prim_count++;
            bins[bin_idx].aabb.grow(prim.aabb_min);
            bins[bin_idx].aabb.grow(prim.aabb_max);
        }

        /* Gather data for the planes between the bins */
        f32 l_areas[BINS - 1], r_areas[BINS - 1];
        i32 l_counts[BINS - 1], r_counts[BINS - 1];
        AABB l_aabb, r_aabb;
        i32 l_sum = 0, r_sum = 0;
        for (i32 i = 0; i < BINS - 1; ++i) {
            /* Left-side */
            l_sum += bins[i].prim_count;
            l_counts[i] = l_sum;
            l_aabb.grow(bins[i].aabb);
            l_areas[i] = l_aabb.area();
            /* Right-side */
            r_sum += bins[BINS - 1 - i].prim_count;
            r_counts[BINS - 2 - i] = r_sum;
            r_aabb.grow(bins[BINS - 1 - i].aabb);
            r_areas[BINS - 2 - i] = r_aabb.area();
        }

        /* Calculate the SAH cost function for all planes */
        scale = (bmax - bmin) / BINS;
        for (i32 i = 0; i < BINS - 1; ++i) {
            f32 plane_cost = l_counts[i] * l_areas[i] + r_counts[i] * r_areas[i];
            if (plane_cost < best_cost) {
                axis = a;
                pos = bmin + scale * (i + 1);
                best_cost = plane_cost;
            }
        }
    }
#else
    /* 8 candidates results in a decent tree */
    constexpr u32 UNIFORM_CANDIDATES = 8;
    for (i32 a = 0; a < 3; ++a) {
        /* Get the min and max of all primitives in the node */
        f32 bmin = BIG_F32, bmax = -BIG_F32;
        for (u32 i = 0; i < node.prim_count; ++i) {
            const glm::vec3 prim_center = prims[node.left_first + i].center();
            bmin = std::min(bmin, prim_center[a]);
            bmax = std::max(bmax, prim_center[a]);
        }
        if (bmin == bmax) continue;

        /* Evaluate the SAH of the uniform candidates */
        f32 scale = (bmax - bmin) / UNIFORM_CANDIDATES;
        for (u32 i = 1; i < UNIFORM_CANDIDATES; ++i) {
            f32 candidatePos = bmin + i * scale;
            f32 cost = evaluate_sah(node, a, candidatePos);
            if (cost < best_cost) pos = candidatePos, axis = a, best_cost = cost;
        }
    }
#endif
    return best_cost;
}

void Bvh::subdivide(Bvh::Node& node, int lvl) {
    // printf("node: {%i} [%i]\n", lvl, node.prim_count);
    if (node.prim_count <= 2u) return;

    /* Determine split based on SAH */
    i32 split_axis = -1;
    f32 split_t = 0;
    f32 split_cost = find_best_split_plane(node, split_axis, split_t);

    /* Calculate parent node cost */
    glm::vec3 e = node.aabb_max - node.aabb_min;
    f32 parent_area = e.x * e.x + e.y * e.y + e.z * e.z;
    f32 parent_cost = node.prim_count * parent_area;
    if (split_cost >= parent_cost) return; /* Split would not be worth it */

    /* Determine which primitives lie on which side */
    int i = node.left_first;
    int j = i + node.prim_count - 1;
    while (i <= j) {
        if (prims[i].center()[split_axis] < split_t) {
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
    size = prims.size();
    nodes = (Node*)_aligned_malloc(sizeof(Node) * size * 2, sizeof(Node) * 2);
    if (!nodes) return;

    /* Initialize the root node */
    Node& root = nodes[root_idx];
    root.left_first = 0, root.prim_count = size;
    nodes_used = 2; /* Skip the second node, for better child node cache alignment */
    update_node_bb(root, prims);

    /* Start the recursive subdivide */
    subdivide(root, 0);
}

f32 Bvh::intersect(const Ray& ray) const {
    const Node *node = &nodes[root_idx], *node_stack[64];
    f32 mind = BIG_F32;
    u32 steps = 0u;
    for (u32 stack_ptr = 0;;) {
        /* If the current node is a leaf... */
        if (node->is_leaf()) {
            /* Check if we hit any primitives */
            for (u32 i = 0; i < node->prim_count; ++i) {
                const VoxelVolume& prim = prims[node->left_first + i];
                glm::vec2 intr = ray.intersection_aabb_sse(prim.aabb_min4, prim.aabb_max4);
                steps++;

                /* Hit occured */
                if (intr.x < BIG_F32 && prim.data[0].size()> 0) {
                    intr.x = prim.intersect(ray, intr.x, intr.y);
                }
                mind = std::min(intr.x, mind);
            }

            /* Decend down the stack */
            if (stack_ptr == 0) break;
            node = node_stack[--stack_ptr];
            continue;
        }

        /* In case we're not a leaf, see if we intersect the child nodes */
        const Node* child1 = &nodes[node->left_first];
        const Node* child2 = &nodes[node->left_first + 1];

        /* This function SHOULD BE inlined, otherwise it causes cache misses for the "node_stack" */
        f32 dist1 = ray.intersects_aabb_sse(child1->aabb_min4, child1->aabb_max4);
        f32 dist2 = ray.intersects_aabb_sse(child2->aabb_min4, child2->aabb_max4);
        steps++;

        /* Child to be traversed first should be the closest one */
        if (dist1 > dist2) {
            std::swap(dist1, dist2);
            std::swap(child1, child2);
        }

        /* Traverse child nodes if they were intersected */
        if (dist1 >= BIG_F32) {
            /* Decend down the stack */
            if (stack_ptr == 0) break;
            node = node_stack[--stack_ptr];
        } else {
            node = child1;
            if (dist2 < BIG_F32) {
                node_stack[stack_ptr++] = child2;
            }
        }
    }

    if (mind < BIG_F32) return mind;
    return ((f32)steps / 64.0f) * ray.t;
    // return (mind < BIG_F32) ? mind : BIG_F32;
}
