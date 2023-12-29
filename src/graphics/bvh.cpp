#include "bvh.h"

#include <utility> /* std::swap */

Bvh::Bvh(u32 size, const std::vector<VoxelVolume>& new_prims)
    : size(size), nodes(new Node[size * 2 - 1]{}) {
    build(new_prims);
}

void update_node_bb(Bvh::Node& node, const std::vector<VoxelVolume>& prims) {
    node.bbmin = glm::vec3(1e30f), node.bbmax = glm::vec3(-1e30f);
    for (u32 first = node.first_prim, i = 0; i < node.prim_count; ++i) {
        const VoxelVolume& prim = prims[first + i];
        glm::vec3 vvmin, vvmax;
        prim.get_bounds(vvmin, vvmax);

        node.bbmin.x = std::min(node.bbmin.x, vvmin.x);
        node.bbmin.y = std::min(node.bbmin.y, vvmin.y);
        node.bbmin.z = std::min(node.bbmin.z, vvmin.z);
        node.bbmax.x = std::max(node.bbmax.x, vvmax.x);
        node.bbmax.y = std::max(node.bbmax.y, vvmax.y);
        node.bbmax.z = std::max(node.bbmax.z, vvmax.z);
    }
}

void Bvh::subdivide(Bvh::Node& node, int lvl) {
    printf("node: {%i} [%i] (%f, %f, %f) <> (%f, %f, %f)\n", lvl, node.prim_count, node.bbmin.x,
           node.bbmin.y, node.bbmin.z, node.bbmax.x, node.bbmax.y, node.bbmax.z);
    if (node.prim_count <= 2) return;

    glm::vec3 extent = node.bbmax - node.bbmin;
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;
    float split_t = node.bbmin[axis] + extent[axis] * 0.5f;

    int i = node.first_prim;
    int j = i + node.prim_count - 1;
    while (i <= j) {
        if (prims[i].pos[axis] < split_t) {
            i++;
        } else {
            std::swap(prims[i], prims[j--]);
        }
    }

    int left_count = i - node.first_prim;
    if (left_count == 0 || left_count == node.prim_count) return;

    int left_child_idx = nodes_used++;
    int right_child_idx = nodes_used++;
    node.left_child = left_child_idx;
    nodes[left_child_idx].first_prim = node.first_prim;
    nodes[left_child_idx].prim_count = left_count;
    node.right_child = right_child_idx;
    nodes[right_child_idx].first_prim = i;
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
    root.left_child = root.right_child = 0;
    root.first_prim = 0, root.prim_count = size;
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

bool Bvh::intersect(const glm::vec3& ro, const glm::vec3& rd, u32 node_idx) const {
    Node& node = nodes[node_idx];
    if (!ray_to_aabb(ro, rd, node.bbmin, node.bbmax)) return false;
    if (node.left_child == 0 && node.right_child == 0) {
        for (u32 i = 0; i < node.prim_count; i++) {
            const VoxelVolume& prim = prims[node.first_prim + i];
            if (ray_to_aabb(ro, rd, prim.min, prim.max)) {
                return true;
            }
        }
    } else {
        if (intersect(ro, rd, node.left_child)) return true;
        if (intersect(ro, rd, node.right_child)) return true;
    }
    return false;
}

bool Bvh::intersect(const glm::vec3& ro, const glm::vec3& rd) const {
    return intersect(ro, rd, root_idx);
}
