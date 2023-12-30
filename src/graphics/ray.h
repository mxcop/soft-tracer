#pragma once

struct AABB;

struct Ray {
    glm::vec3 origin;
    glm::vec3 dir, inv_dir;
    glm::ivec3 sign;

    Ray() = delete;
    Ray(const glm::vec3& origin, const glm::vec3& dir);

    /**
     * @returns True if the ray intersects the given AABB.
     */
    bool intersects_aabb(const AABB& aabb) const;
};
