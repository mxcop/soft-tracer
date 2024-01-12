#pragma once

struct AABB {
    union {
        glm::vec3 corners[2] = {glm::vec3(BIG_F32), glm::vec3(-BIG_F32)};
        struct {
            glm::vec3 min, max;
        };
    };

    AABB() = default;
    AABB(glm::vec3 min, glm::vec3 max);

    /**
     * @brief Grow AABB to include given point.
     */
    void grow(glm::vec3 p);
    glm::vec3 center() const;
    float area() const;
};
