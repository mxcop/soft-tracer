#include "vv.h"

#include <glm/gtx/quaternion.hpp>

VoxelVolume::VoxelVolume(glm::vec3 pos, glm::ivec3 size, glm::vec3 rot) {
    glm::vec3 half_size = (glm::vec3(size) / VOXELS_PER_UNIT) / 2.0f;
    // aabb = AABB(pos - half_size, pos + half_size);
    aabb_min = pos;
    aabb_max = pos + glm::vec3(size) / VOXELS_PER_UNIT;
}

VoxelVolume::VoxelVolume(glm::vec3 pos, glm::ivec3 size, glm::vec3 rot, std::vector<u8> voxels) : voxels(voxels) {
    glm::vec3 half_size = (glm::vec3(size) / VOXELS_PER_UNIT) / 2.0f;
    // aabb = AABB(pos - half_size, pos + half_size);
    aabb_min = pos;
    aabb_max = pos + glm::vec3(size) / VOXELS_PER_UNIT;
}

void VoxelVolume::get_bounds(glm::vec3& out_min, glm::vec3& out_max) const {
    //glm::mat4 rot_mtx = glm::toMat4(glm::quat(rot));

    // glm::vec3 half_size = (glm::vec3(size) / VOXELS_PER_UNIT) / 2.0f;
    //out_min = -half_size;
    //out_max = half_size;

    //glm::mat4 model = glm::mat4(1.0f);
    //model = glm::translate(model, pos);
    //model *= rot_mtx;

    //out_min = model * glm::vec4(out_min, 1.0f);
    //out_max = model * glm::vec4(out_max, 1.0f);

    out_min = aabb_min;
    out_max = aabb_max;
}

glm::vec3 VoxelVolume::center() const { return aabb_min + (aabb_max - aabb_min) * 0.5f; }

glm::mat4 VoxelVolume::get_model() const {
    glm::mat4 model = glm::mat4(1.0f);
    // model = glm::translate(model, pos);
    //glm::mat4 rot_mtx = glm::toMat4(glm::quat(rot));
    //model *= rot_mtx;
    return model;
}

f32 VoxelVolume::intersect(const Ray& ray, const f32 tmin) const {
    const glm::ivec3 prim_size = (aabb_max - aabb_min) * VOXELS_PER_UNIT;
    const int prim_max = prim_size.x * prim_size.y * prim_size.z;

    /* Move up to the edge of the bounding box */
    const glm::vec3 p = ray.origin + ray.dir * (tmin + 0.1f);

    /* Voxel position */
    glm::vec3 vp = (p - aabb_min) * VOXELS_PER_UNIT;
    glm::ivec3 idx = glm::floor(vp);

    /* Ray direction sign mask */
    const glm::vec3 srd = glm::sign(ray.dir);
    glm::vec3 sd = (glm::vec3(idx) - vp + srd) * ray.inv_dir;

    for (int i = 0; i < 256; ++i) {
        size_t ii = ((size_t)idx.z * prim_size.x * prim_size.y) + ((size_t)idx.y * prim_size.x) + idx.x;
        if (ii < 0 || ii >= prim_max) {
            break;
        }
        /* Index the voxel data */
        u8 voxel = voxels[ii];

        if (voxel > 0) {
            return voxel * 50.0f;
        }

        /* Compute the step mask */
        glm::vec3 yzx = sd.yzx, zxy = sd.zxy;
        glm::vec3 mask = glm::lessThanEqual(sd, glm::min(yzx, zxy));
        // vec3 mask = vec3(lessThanEqual(sd.xyz, min(sd.yzx, sd.zxy)));

        /* Step to the next voxel */
        sd += mask * srd * ray.inv_dir;
        idx += mask * srd;
        vp += mask * ray.dir;

        /* Check if we're still within the bounding volume */
        if (glm::any(glm::lessThan(idx, glm::ivec3(0))) ||
            glm::any(glm::greaterThanEqual(idx, prim_size))) {
            break;
        }
    }
    return BIG_F32; // change to 10'000.0f for outline
}
