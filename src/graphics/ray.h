#pragma once

struct AABB;

struct Ray {
    union {
        struct {
            glm::vec3 origin;
            f32 _;
        };
        f128 origin_4;
    };
    union {
        struct {
            glm::vec3 dir;
            f32 _;
        };
        f128 dir_4;
    };
    union {
        struct {
            glm::vec3 inv_dir;
            f32 _;
        };
        f128 inv_dir_4;
    };

    Ray() = delete;
    Ray(const glm::vec3& origin, const glm::vec3& dir);

    Ray(const Ray&) = delete;
    Ray(Ray&&) = default;
    Ray& operator=(const Ray&) = delete;
    Ray& operator=(Ray&&) = default;

    /**
     * @returns -1.0f if there was no intersection, otherwise the distance.
     */
    float intersects_aabb(const AABB& aabb) const;

    float intersects_aabb_sse(const f128 bmin4, const f128 bmax4) const;
};
