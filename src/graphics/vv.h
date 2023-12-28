#pragma once

struct VoxelVolume {
    glm::vec3 pos;
    glm::ivec3 size;
    glm::vec3 rot;

    void get_bounds(glm::vec3& out_min, glm::vec3& out_max) const;
};
