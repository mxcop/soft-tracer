#include "ray.h"

#include "aabb.h"

static inline glm::ivec3 signbit(glm::vec3 _X) throw() {
    return glm::ivec3(signbit(_X.x), signbit(_X.y), signbit(_X.z));
}

Ray::Ray(const glm::vec3& origin, const glm::vec3& dir)
    : origin(origin), dir(dir), inv_dir(1.0f / dir), sign(signbit(dir)) {
    for (u32 i = 0; i < 3; ++i) {
        cache.origin[i] = _mm256_set1_ps(origin[i]);
        cache.dir_inv[i] = _mm256_set1_ps(inv_dir[i]);
    }
}

float Ray::intersects_aabb(const AABB& aabb) const {
    float tmin = 0.0, tmax = INFINITY;

    for (int d = 0; d < 3; ++d) {
        bool sign = signbit(inv_dir[d]);
        float bmin = aabb.corners[sign][d];
        float bmax = aabb.corners[!sign][d];

        float dmin = (bmin - origin[d]) * inv_dir[d];
        float dmax = (bmax - origin[d]) * inv_dir[d];

        tmin = std::max(dmin, tmin);
        tmax = std::min(dmax, tmax);
        /* Early out check, saves a lot of compute */
        if (tmax < tmin) return -1.0f;
    }

    return tmin;
}
