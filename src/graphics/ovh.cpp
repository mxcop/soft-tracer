#include "ovh.h"

#include <utility> /* std::swap */

#include "vv.h"
#include "ray.h"

Ovh::Ovh(u32 size, const std::vector<VoxelVolume>& new_prims)
    : size(size), nodes(new Node[size * 2 - 1]{}) {
    build(new_prims);
}

AABB Ovh::compute_node_bb(const Node& node, u32 first_prim, u32 prim_count) const {
    AABB bb = AABB(glm::vec3(1e30f), glm::vec3(-1e30f));
    for (u32 first = first_prim, i = 0; i < prim_count; ++i) {
        const VoxelVolume& prim = prims[first + i];

        bb.min.x = std::min(bb.min.x, prim.aabb.min.x);
        bb.min.y = std::min(bb.min.y, prim.aabb.min.y);
        bb.min.z = std::min(bb.min.z, prim.aabb.min.z);
        bb.max.x = std::max(bb.max.x, prim.aabb.max.x);
        bb.max.y = std::max(bb.max.y, prim.aabb.max.y);
        bb.max.z = std::max(bb.max.z, prim.aabb.max.z);
    }
    return bb;
}

void Ovh::subdivide(Node& node, const AABB& node_bb, u32 first_prim, u32 prim_count, int lvl) {
    // printf("node: [%i] (%i)\n", lvl, node.prim_count);

    if (prim_count <= 8) {
        /* Cache the SIMD AABB structure */
        node.x8_aabb = AABB_256();
        VoxelVolume* vvs = &prims[first_prim];
        for (u32 i = 0; i < prim_count; ++i) {
            node.x8_aabb.min[0].m256_f32[i] = vvs[i].aabb.min.x;
            node.x8_aabb.min[1].m256_f32[i] = vvs[i].aabb.min.y;
            node.x8_aabb.min[2].m256_f32[i] = vvs[i].aabb.min.z;
            node.x8_aabb.max[0].m256_f32[i] = vvs[i].aabb.max.x;
            node.x8_aabb.max[1].m256_f32[i] = vvs[i].aabb.max.y;
            node.x8_aabb.max[2].m256_f32[i] = vvs[i].aabb.max.z;
        }
        node.first_child = -1;
        return;
    }

    glm::vec3 extent = node_bb.max - node_bb.min;
    glm::vec3 split = node_bb.min + extent * 0.5f;

    /* Sort the primatives by their octant */
    for (u32 i = 0; i < prim_count - 1; i++) {
        for (u32 j = first_prim; j < first_prim + prim_count - i - 1; j++) {
            glm::vec3 d = prims[j].pos - split;
            u32 octant_a = ((d.x > 0) * 2 + (d.y > 0)) * 2 + (d.z > 0);

            d = prims[j + 1].pos - split;
            u32 octant_b = ((d.x > 0) * 2 + (d.y > 0)) * 2 + (d.z > 0);

            if (octant_a > octant_b) {
                std::swap(prims[j], prims[j + 1]);
            }
        }
    }

    /* Find the size and index of each octant / child node */
    u32 octant_sizes[8] = {};
    u32 octant_indices[8] = {};
    for (u32 i = 0; i < prim_count; i++) {
        glm::vec3 d = prims[first_prim + i].pos - split;
        u32 octant = ((d.x > 0) * 2 + (d.y > 0)) * 2 + (d.z > 0);
        if (octant_sizes[octant] == 0) {
            octant_indices[octant] = first_prim + i;
        }
        octant_sizes[octant]++;
    }

    /* Create the child nodes and compute their bounding boxes */
    node.first_child = nodes_used;
    AABB child_bb[8] = {};
    for (u32 i = 0; i < 8; ++i) {
        u32 child_idx = nodes_used++;
        Node& child = nodes[child_idx];

        child_bb[i] = compute_node_bb(child, octant_indices[i], octant_sizes[i]);
    }

    /* Subdivide the child nodes */
    for (u32 i = 0; i < 8; i++) {
        subdivide(nodes[node.first_child + i], child_bb[i], octant_indices[i], octant_sizes[i],
                  lvl + 1);
    }

    /* Cache the SIMD AABB structure */
    node.x8_aabb = AABB_256();
    for (u32 i = 0; i < 8; ++i) {
        const AABB& aabb = child_bb[i];
        node.x8_aabb.min[0].m256_f32[i] = aabb.min.x;
        node.x8_aabb.min[1].m256_f32[i] = aabb.min.y;
        node.x8_aabb.min[2].m256_f32[i] = aabb.min.z;
        node.x8_aabb.max[0].m256_f32[i] = aabb.max.x;
        node.x8_aabb.max[1].m256_f32[i] = aabb.max.y;
        node.x8_aabb.max[2].m256_f32[i] = aabb.max.z;
    }
}

void Ovh::build(const std::vector<VoxelVolume>& new_prims) {
    prims = new_prims;

    /* initialize the root node */
    Node& root = nodes[root_idx];
    root.first_child = 0;
    AABB root_bb = compute_node_bb(root, 0, size);

    /* start the recursive subdivide */
    subdivide(root, root_bb, 0, size, 0);
}

/**
 * @brief Compute the intersections between a ray and 8 axis aligned bounding boxes.
 *
 * @returns A byte where each bit represents an intersection.
 */
static u8 ray_aabb_intersect_x8(const Ray& ray, const AABB_256& aabb) {
    f256 tmin = _mm256_set1_ps(0.0f);
    f256 tmax = _mm256_set1_ps(INFINITY);

    for (u32 i = 0; i < 3; ++i) {
        const f256& bmin = aabb.corners[ray.sign[i]][i];
        const f256& bmax = aabb.corners[!ray.sign[i]][i];

        const f256& origin = ray.cache.origin[i];
        const f256& dir_inv = ray.cache.dir_inv[i];

        const f256 dmin = _mm256_mul_ps(_mm256_sub_ps(bmin, origin), dir_inv);
        const f256 dmax = _mm256_mul_ps(_mm256_sub_ps(bmax, origin), dir_inv);

        tmin = _mm256_max_ps(dmin, tmin);
        tmax = _mm256_min_ps(dmax, tmax);
    }

    const f256 result = _mm256_sub_ps(tmin, tmax);
    return _mm256_movemask_ps(result);
}

bool Ovh::intersect(const Ray& ray) const {
    Node *node = &nodes[root_idx], *node_stack[64];
    u32 stack_ptr = 0;
    for (;;) {
        /* Compute the intersection mask */
        u8 int_mask = ray_aabb_intersect_x8(ray, node->x8_aabb);

        /* Continue if there were no hits */
        if (!int_mask) {
            if (stack_ptr == 0) break;
            node = node_stack[--stack_ptr];
            continue;
        }

        /* If this node is a leaf, we hit a primitive */
        if (node->is_leaf()) return true;

        /* Else add all octant intersections onto the stack */
        for (u32 i = 0; i < 8; i++) {
            if (int_mask & (1 << i)) {
                node_stack[stack_ptr++] = &nodes[node->first_child + i];
            }
        }
        node = node_stack[--stack_ptr];
    }
    return false;
}
