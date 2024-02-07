#include "aabb.h"

AABB::AABB(f32 min, f32 max) : min(min), max(max) {}

AABB::AABB(glm::vec3 min, glm::vec3 max) : min(min), max(max) {}

void AABB::grow(glm::vec3 p) {
    min = glm::min(min, p);
    max = glm::max(max, p);
}

void AABB::grow(const AABB& aabb) {
    if (aabb.min.x != 1e30f) {
        this->grow(aabb.min);
        this->grow(aabb.max);
    }
}

glm::vec3 AABB::center() const { return min + (max - min) * 0.5f; }

float AABB::area() const {
    glm::vec3 e = max - min;
    return e.x * e.x + e.y * e.y + e.z * e.z;
}
