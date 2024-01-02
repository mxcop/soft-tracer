#include "ovh.h"

#include <utility> /* std::swap */

#include "vv.h"
#include "ray.h"

Ovh::Ovh(u32 size, const std::vector<VoxelVolume>& new_prims)
    : size(size), nodes(new Node[size * 2 - 1]{}) {
    build(new_prims);
}

void Ovh::update_node_bb(Node& node) {
    node.aabb = AABB(glm::vec3(1e30f), glm::vec3(-1e30f));
    for (u32 first = node.first_prim, i = 0; i < node.prim_count; ++i) {
        const VoxelVolume& prim = prims[first + i];

        node.aabb.min.x = std::min(node.aabb.min.x, prim.aabb.min.x);
        node.aabb.min.y = std::min(node.aabb.min.y, prim.aabb.min.y);
        node.aabb.min.z = std::min(node.aabb.min.z, prim.aabb.min.z);
        node.aabb.max.x = std::max(node.aabb.max.x, prim.aabb.max.x);
        node.aabb.max.y = std::max(node.aabb.max.y, prim.aabb.max.y);
        node.aabb.max.z = std::max(node.aabb.max.z, prim.aabb.max.z);
    }
}

void Ovh::subdivide(Node& node, int lvl) {
    // printf("node: [%i] (%i)\n", lvl, node.prim_count);

    if (node.prim_count <= 8) {
        /* Cache the SIMD AABB structure */
        node.prim_aabb = AABB_256();
        VoxelVolume* vvs = &prims[node.first_prim];
        for (u32 i = 0; i < node.prim_count; ++i) {
            node.prim_aabb.min[0].m256_f32[i] = vvs[i].aabb.min.x;
            node.prim_aabb.min[1].m256_f32[i] = vvs[i].aabb.min.y;
            node.prim_aabb.min[2].m256_f32[i] = vvs[i].aabb.min.z;
            node.prim_aabb.max[0].m256_f32[i] = vvs[i].aabb.max.x;
            node.prim_aabb.max[1].m256_f32[i] = vvs[i].aabb.max.y;
            node.prim_aabb.max[2].m256_f32[i] = vvs[i].aabb.max.z;
        }
        node.leaf = true;
        return;
    }

    glm::vec3 extent = node.aabb.max - node.aabb.min;
    glm::vec3 split = node.aabb.min + extent * 0.5f;

    /* Sort the primatives by their octant */
    for (u32 i = 0; i < node.prim_count - 1; i++) {
        for (u32 j = node.first_prim; j < node.first_prim + node.prim_count - i - 1; j++) {
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
    for (u32 i = 0; i < node.prim_count; i++) {
        glm::vec3 d = prims[node.first_prim + i].pos - split;
        u32 octant = ((d.x > 0) * 2 + (d.y > 0)) * 2 + (d.z > 0);
        if (octant_sizes[octant] == 0) {
            octant_indices[octant] = node.first_prim + i;
        }
        octant_sizes[octant]++;
    }

    /* Create the child nodes */
    for (u32 i = 0; i < 8; ++i) {
        u32 child_idx = nodes_used++;
        node.childern[i] = child_idx;

        Node& child = nodes[child_idx];
        child.first_prim = octant_indices[i];
        child.prim_count = octant_sizes[i];

        update_node_bb(child);
        subdivide(child, lvl + 1);
    }
    node.prim_count = 0;

    /* Cache the SIMD AABB structure */
    node.child_aabb = AABB_256();
    for (u32 i = 0; i < 8; ++i) {
        const AABB& aabb = nodes[node.childern[i]].aabb;
        node.child_aabb.min[0].m256_f32[i] = aabb.min.x;
        node.child_aabb.min[1].m256_f32[i] = aabb.min.y;
        node.child_aabb.min[2].m256_f32[i] = aabb.min.z;
        node.child_aabb.max[0].m256_f32[i] = aabb.max.x;
        node.child_aabb.max[1].m256_f32[i] = aabb.max.y;
        node.child_aabb.max[2].m256_f32[i] = aabb.max.z;
    }
}

void Ovh::build(const std::vector<VoxelVolume>& new_prims) {
    prims = new_prims;

    /* initialize the root node */
    Node& root = nodes[root_idx];
    memset(root.childern, 0, 8 * sizeof(u32));
    root.first_prim = 0, root.prim_count = size;
    update_node_bb(root);

    /* start the recursive subdivide */
    subdivide(root, 0);
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
    Node *node = &nodes[root_idx], *node_stack[128];
    u32 stack_ptr = 0;
    while (1) {
        if (node->is_leaf()) {
            if (ray_aabb_intersect_x8(ray, node->prim_aabb)) return true;

            if (stack_ptr == 0)
                break;
            else
                node = node_stack[--stack_ptr];
            continue;
        }

        /* Compute the intersection mask */
        u8 int_mask = ray_aabb_intersect_x8(ray, node->child_aabb);

        /* Continue if there were no hits */
        if (!int_mask) {
            if (stack_ptr == 0)
                break;
            else
                node = node_stack[--stack_ptr];
            continue;
        }

        /* Add all hits onto the stack */
        for (u32 i = 0; i < 8; i++) {
            if (int_mask & (1 << i)) {
                node_stack[stack_ptr++] = &nodes[node->childern[i]];
            }
        }
        node = node_stack[--stack_ptr];
    }
    return false;
}
