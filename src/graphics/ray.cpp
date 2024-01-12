#include "ray.h"

#include "aabb.h"

Ray::Ray(const glm::vec3& origin, const glm::vec3& dir)
    : origin(origin), dir(dir), inv_dir(1.0f / dir) {}

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

float Ray::intersects_aabb_sse(const f128 bmin4, const f128 bmax4) const {
    static const f128 mask4 = _mm_cmpeq_ps(_mm_setzero_ps(), _mm_set_ps(1, 0, 0, 0));
    const f128 t1 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bmin4, mask4), origin_4), inv_dir_4);
    const f128 t2 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bmax4, mask4), origin_4), inv_dir_4);
    const f128 vmax4 = _mm_max_ps(t1, t2), vmin4 = _mm_min_ps(t1, t2);
    const float tmax = std::min(vmax4.m128_f32[0], std::min(vmax4.m128_f32[1], vmax4.m128_f32[2]));
    const float tmin = std::max(vmin4.m128_f32[0], std::max(vmin4.m128_f32[1], vmin4.m128_f32[2]));
    return (tmax >= tmin) ? tmin : BIG_F32;
}
