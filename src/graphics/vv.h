#pragma once

constexpr float VOXELS_PER_UNIT = 8;

struct VoxelVolume {
    /* Center position of the volume */
    glm::vec3 pos;
    /* Size of the volume in voxels */
    glm::ivec3 size;
    glm::vec3 rot;

    glm::vec3 min, max;

    VoxelVolume(glm::vec3 pos, glm::ivec3 size, glm::vec3 rot);

    void get_bounds(glm::vec3& out_min, glm::vec3& out_max) const;
    glm::mat4 get_model() const;
};
