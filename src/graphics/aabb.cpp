#include "aabb.h"

AABB::AABB(glm::vec3 min, glm::vec3 max) : min(min), max(max) {}

void AABB::grow(glm::vec3 p) {
    min = glm::vec3(std::min(min.x, p.x), std::min(min.y, p.y), std::min(min.z, p.z)),
    max = glm::vec3(std::max(max.x, p.x), std::max(max.y, p.y), std::max(max.z, p.z));
}

glm::vec3 AABB::center() const { return min + (max - min) * 0.5f; }

float AABB::area() const {
    glm::vec3 e = max - min;
    return e.x * e.y + e.y * e.z + e.z * e.x;
}
