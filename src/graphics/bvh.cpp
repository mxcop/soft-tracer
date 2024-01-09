#include "bvh.h"

#include <utility> /* std::swap */

#include "vv.h"
#include "ray.h"

Bvh::Bvh(u32 size, const std::vector<VoxelVolume>& new_prims)
    : size(size), nodes(new Node[size * 2 - 1]{}) {
    /* Ensure that the number of primitives doesn't overflow */
    assert(new_prims.size() <= 0xFFFF && "too many primitives!");

    build(new_prims);
}

void update_node_bb(Bvh::Node& node, const std::vector<VoxelVolume>& prims) {
    node.aabb_min = glm::vec3(1e30f);
    node.aabb_max = glm::vec3(-1e30f);
    for (u32 first = node.left_first, i = 0; i < node.prim_count; ++i) {
        const VoxelVolume& prim = prims[first + i];

        node.aabb_min.x = std::min(node.aabb_min.x, prim.aabb_min.x);
        node.aabb_min.y = std::min(node.aabb_min.y, prim.aabb_min.y);
        node.aabb_min.z = std::min(node.aabb_min.z, prim.aabb_min.z);
        node.aabb_max.x = std::max(node.aabb_max.x, prim.aabb_max.x);
        node.aabb_max.y = std::max(node.aabb_max.y, prim.aabb_max.y);
        node.aabb_max.z = std::max(node.aabb_max.z, prim.aabb_max.z);
    }
}

void Bvh::subdivide(Bvh::Node& node, int lvl) {
    printf("node: {%i} [%i]\n", lvl, node.prim_count);
    if (node.prim_count <= 2u) return;

    glm::vec3 extent = node.aabb_max - node.aabb_min;
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;
    float split_t = node.aabb_min[axis] + extent[axis] * 0.5f;

    int i = node.left_first;
    int j = i + node.prim_count - 1;
    while (i <= j) {
        if (prims[i].aabb_min[axis] < split_t) {
            i++;
        } else {
            std::swap(prims[i], prims[j--]);
        }
    }

    int left_count = i - node.left_first;
    if (left_count == 0 || left_count == node.prim_count) return;

    int left_child_idx = nodes_used++;
    int right_child_idx = nodes_used++;
    node.left_first = left_child_idx;
    nodes[left_child_idx].left_first = node.left_first;
    nodes[left_child_idx].prim_count = left_count;
    //node.right_child = right_child_idx;
    nodes[right_child_idx].left_first = i;
    nodes[right_child_idx].prim_count = node.prim_count - left_count;
    node.prim_count = 0;

    update_node_bb(nodes[left_child_idx], prims);
    update_node_bb(nodes[right_child_idx], prims);

    subdivide(nodes[left_child_idx], lvl + 1);
    subdivide(nodes[right_child_idx], lvl + 1);
}

void Bvh::build(const std::vector<VoxelVolume>& new_prims) {
    prims = new_prims;

    /* initialize the root node */
    Node& root = nodes[root_idx];
    root.left_first = 0, root.prim_count = size;
    update_node_bb(root, prims);

    /* start the recursive subdivide */
    subdivide(root, 0);
}

/**
 * @brief Ray to Oriented Bounding Box intersection test.
 */
static float ray_to_obb(const glm::vec3& ro, const glm::vec3& rd, const glm::vec3& bbmin,
                        const glm::vec3& bbmax, const glm::mat4& model) {
    float t_min = 0.0, t_max = INFINITY;

    /* "model[3]" holds the world position of the box */
    glm::vec3 delta = glm::vec3(model[3]) - ro;
    const glm::vec3 corners[2]{bbmin, bbmax};

    /* loop to be unrolled by the compiler */
    for (int d = 0; d < 3; ++d) {
        glm::vec3 axis = glm::vec3(model[d]);
        float e = glm::dot(axis, delta), f_inv = 1.0f / glm::dot(rd, axis);

        float t1 = (e + corners[0][d]) * f_inv;
        float t2 = (e + corners[1][d]) * f_inv;

        /* swap t1 & t2 so t1 is always the smallest */
        if (t1 > t2) {
            float temp = t1;
            t1 = t2, t2 = temp;
        }

        t_min = std::max(t1, t_min);
        t_max = std::min(t2, t_max);

        /* early out check */
        if (t_max < t_min) return -1.0f;
    }
    return t_min;
}

/**
 * @brief Ray to Axis Aligned Bounding Box intersection test.
 */
static bool ray_to_aabb(const glm::vec3& ro, const glm::vec3& rd, const glm::vec3& bbmin,
                        const glm::vec3& bbmax) {
    float tmin = 0.0, tmax = INFINITY;

    const glm::vec3 corners[2]{bbmin, bbmax};
    glm::vec3 inv_dir = 1.0f / rd;

    for (int d = 0; d < 3; ++d) {
        bool sign = signbit(inv_dir[d]);
        float bmin = corners[sign][d];
        float bmax = corners[!sign][d];

        float dmin = (bmin - ro[d]) * inv_dir[d];
        float dmax = (bmax - ro[d]) * inv_dir[d];

        tmin = std::max(dmin, tmin);
        tmax = std::min(dmax, tmax);
        /* Early out check, saves a lot of compute */
        if (tmax < tmin) return false;
    }

    return tmin < tmax;
}

static inline float _mm256_hadd(const f256& a) {
    f256 t1 = _mm256_hadd_ps(a, a);
    f256 t2 = _mm256_hadd_ps(t1, t1);
    __m128 t3 = _mm256_extractf128_ps(t2, 1);
    __m128 t4 = _mm_add_ss(_mm256_castps256_ps128(t2), t3);
    return _mm_cvtss_f32(t4);
}

//static bool oct_ray_to_aabb(const Ray& ray, const AABB_256* aabb) {
//    /* TODO: maybe cache these large vectors? */
//    /* Turns out this is very slow, but it's still for the moment better here */
//    /* Because this way we only do it *if* we reach a leaf in the BVH */
//    f256 origin[3], dir_inv[3];
//    for (u32 i = 0; i < 3; ++i) {
//        origin[i] = _mm256_broadcast_ss(&ray.origin[i]);
//        dir_inv[i] = _mm256_broadcast_ss(&ray.inv_dir[i]);
//    }
//
//    f256 tmin = _mm256_set1_ps(0.0f);
//    f256 tmax = _mm256_set1_ps(1'000'000.0f);
//
//    for (u32 i = 0; i < 3; ++i) {
//        const f256 bmin = aabb->corners[ray.sign[i]][i];
//        const f256 bmax = aabb->corners[!ray.sign[i]][i];
//
//        const f256 dmin = _mm256_mul_ps(_mm256_sub_ps(bmin, origin[i]), dir_inv[i]);
//        const f256 dmax = _mm256_mul_ps(_mm256_sub_ps(bmax, origin[i]), dir_inv[i]);
//
//        // const f256 dmin = _mm256_mul_ps(_mm256_sub_ps(bmin, ray.aabb_cache[i]), ray.aabb_cache[3
//        // + i]); const f256 dmax = _mm256_mul_ps(_mm256_sub_ps(bmax, ray.aabb_cache[i]),
//        // ray.aabb_cache[3 + i]);
//
//        tmin = _mm256_max_ps(dmin, tmin);
//        tmax = _mm256_min_ps(dmax, tmax);
//    }
//
//    /* Use a mask to remove non-intersections (tmin <= tmax) */
//    const f256 mask = _mm256_cmp_ps(tmin, tmax, _CMP_LE_OQ);
//    const f256 result = _mm256_blendv_ps(_mm256_set1_ps(0.0f), tmin, mask);
//
//    return _mm256_hadd(result);
//}

//bool Bvh::intersect(const Ray& ray, u32 node_idx) const {
//    Node& node = nodes[node_idx];
//    if (ray.intersects_aabb(node.aabb) < 0.0f) return false;
//
//    if (node.left_child == 0 && node.right_child == 0) {
//        float mind = INFINITY;
//        for (u16 i = 0; i < node.prim_count; i++) {
//            const AABB& prim_aabb = prim_aabbs[node.first_prim + i];
//            float dist = ray.intersects_aabb(prim_aabb);
//            if (dist >= 0.0f) {
//                mind = std::min(mind, dist);
//            }
//        }
//        if (mind < INFINITY) return true;
//    } else {
//        if (intersect(ray, node.left_child)) return true;
//        if (intersect(ray, node.right_child)) return true;
//    }
//    return false;
//}

// bool Bvh::intersect(const Ray& ray) const { return intersect(ray, root_idx); }

bool Bvh::intersect(const Ray& ray) const {
    Node *node = &nodes[root_idx], *stack[128];
    u32 stack_ptr = 0;
    while (1) {
        if (node->is_leaf()) {
            float mind = INFINITY;
            for (u16 i = 0; i < node->prim_count; i++) {
                const VoxelVolume& prim = prims[node->left_first + i];
                float dist = ray.intersects_aabb_sse(prim.aabb_min4, prim.aabb_max4);
                if (dist >= 0.0f) {
                    mind = std::min(mind, dist);
                }
            }
            if (mind < INFINITY) return true;

            if (stack_ptr == 0)
                break;
            else
                node = stack[--stack_ptr];
            continue;
        }
        Node* child1 = &nodes[node->left_first];
        Node* child2 = &nodes[node->left_first + 1];
        float dist1 = ray.intersects_aabb_sse(child1->aabb_min4, child1->aabb_max4);
        float dist2 = ray.intersects_aabb_sse(child2->aabb_min4, child2->aabb_max4);
        if (dist1 < 0.0f && dist2 < 0.0f) {
            if (stack_ptr == 0)
                break;
            else
                node = stack[--stack_ptr];
            continue;
        }
        if (dist1 >= 0.0f) {
            node = child1;
            if (dist2 >= 0.0f) stack[stack_ptr++] = child2;
        } else {
            node = child2;
        }
    }
    return false;
}
