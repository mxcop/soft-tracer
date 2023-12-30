#include "vv.h"

#include <glm/gtx/quaternion.hpp>

VoxelVolume::VoxelVolume(glm::vec3 pos, glm::ivec3 size, glm::vec3 rot)
    : pos(pos), size(size), rot(rot) {
    glm::vec3 half_size = (glm::vec3(size) / VOXELS_PER_UNIT) / 2.0f;
    aabb = AABB(pos - half_size, pos + half_size);
}

void VoxelVolume::get_bounds(glm::vec3& out_min, glm::vec3& out_max) const {
    //glm::mat4 rot_mtx = glm::toMat4(glm::quat(rot));

    glm::vec3 half_size = (glm::vec3(size) / VOXELS_PER_UNIT) / 2.0f;
    //out_min = -half_size;
    //out_max = half_size;

    //glm::mat4 model = glm::mat4(1.0f);
    //model = glm::translate(model, pos);
    //model *= rot_mtx;

    //out_min = model * glm::vec4(out_min, 1.0f);
    //out_max = model * glm::vec4(out_max, 1.0f);

    out_min = pos - half_size;
    out_max = pos + half_size;
}

glm::mat4 VoxelVolume::get_model() const {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, pos);
    //glm::mat4 rot_mtx = glm::toMat4(glm::quat(rot));
    //model *= rot_mtx;
    return model;
}
