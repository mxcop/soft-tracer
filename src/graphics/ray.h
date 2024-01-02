#pragma once

struct AABB;

struct Ray {
    glm::vec3 origin;
    glm::vec3 dir, inv_dir;
    glm::ivec3 sign;

    union {
        f256 cache[6];
        struct {
            f256 origin[3];
            f256 dir_inv[3];
        };
    } cache;

    Ray() = delete;
    Ray(const glm::vec3& origin, const glm::vec3& dir);

    /**
     * @returns -1.0f if there was no intersection, otherwise the distance.
     */
    float intersects_aabb(const AABB& aabb) const;
};
