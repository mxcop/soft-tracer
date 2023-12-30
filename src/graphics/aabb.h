#pragma once

struct AABB {
    union {
        glm::vec3 corners[2];
        struct {
            glm::vec3 min, max;
        };
    };

    AABB() = default;
    AABB(glm::vec3 min, glm::vec3 max);
};
