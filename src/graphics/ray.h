#pragma once

struct AABB;

struct alignas(64) Ray {
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
    f32 t = 10000.0f;

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
    float intersects_aabb(const glm::vec3& min, const glm::vec3& max) const;

    static inline float _min(float x, float y) { return x < y ? x : y; }
    static inline float _max(float x, float y) { return x > y ? x : y; }

#if 1
    /* Adapted from <https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/> */
    float intersects_aabb_sse(const f128 bmin4, const f128 bmax4) const {
        /* Idea to use fmsub to save 1 instruction came from
         * <http://www.joshbarczak.com/blog/?p=787> */
        const f128 rd = _mm_mul_ps(origin_4, inv_dir_4);
        const f128 t1 = _mm_fmsub_ps(bmin4, inv_dir_4, rd);
        const f128 t2 = _mm_fmsub_ps(bmax4, inv_dir_4, rd);

        /* Find the near and far intersection point */
        const f128 vmax4 = _mm_max_ps(t1, t2), vmin4 = _mm_min_ps(t1, t2);
        const f128 tmax4 = _mm_min_ps(vmax4, _mm_movehl_ps(vmax4, vmax4));
        const f32 tmax = _min(tmax4.m128_f32[0], vmax4.m128_f32[1]);
        const f128 tmin4 = _mm_max_ps(vmin4, _mm_movehl_ps(vmin4, vmin4));
        const f32 tmin = _max(tmin4.m128_f32[0], vmin4.m128_f32[1]);

        const bool hit = (tmax > 0 && tmin < t && tmin < tmax);
        return hit ? tmin : BIG_F32;
    }
#else
    /* Adapted from <https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/> */
    float intersects_aabb_sse(const f128 bmin4, const f128 bmax4) const {
        /* IMPORTANT! the mask saves a lot of CPU cycles,
         * it removes the 4th garbage element in the vectors. */

        /* Instead of "mask4 = _mm_cmpeq_ps(_mm_setzero_ps(), _mm_set_ps(1, 0, 0, 0))" */
        /* Using this bit hack is slightly faster in practice */
        const i128 mask4 = _mm_set_epi32(0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF);
        const f128 fmask4 = reinterpret_cast<const f128&>(mask4); /* bit hack */
        const f128 t1 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bmin4, fmask4), origin_4), inv_dir_4);
        const f128 t2 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bmax4, fmask4), origin_4), inv_dir_4);

        const f128 vmax4 = _mm_max_ps(t1, t2), vmin4 = _mm_min_ps(t1, t2);
        const float tmax = _min(vmax4.m128_f32[0], _min(vmax4.m128_f32[1], vmax4.m128_f32[2]));
        const float tmin = _max(vmin4.m128_f32[0], _max(vmin4.m128_f32[1], vmin4.m128_f32[2]));

        const bool hit = (tmax > 0 && tmin < t && tmin < tmax);
        return hit ? tmin : BIG_F32;
    }
    #endif

    glm::vec2 intersects_aabb2_avx(const f128& amin4, const f128& amax4, const f128& bmin4,
                                   const f128& bmax4) const;
};

inline glm::vec2 Ray::intersects_aabb2_avx(const f128& amin4, const f128& amax4, const f128& bmin4,
                                           const f128& bmax4) const {
    const f256 min8 = _mm256_set_m128(bmin4, amin4);
    const f256 max8 = _mm256_set_m128(bmax4, amax4);

    const f256 origin_8 = _mm256_broadcast_ps(&origin_4);
    const f256 inv_dir_8 = _mm256_broadcast_ps(&inv_dir_4);

    const f256 rd = _mm256_mul_ps(origin_8, inv_dir_8);
    const f256 t1 = _mm256_fmsub_ps(min8, inv_dir_8, rd);
    const f256 t2 = _mm256_fmsub_ps(max8, inv_dir_8, rd);

    // const f256 origin_8 = _mm256_broadcast_ps(&origin_4);
    // const f256 inv_dir_8 = _mm256_broadcast_ps(&inv_dir_4);
    // const f256 t1 = _mm256_mul_ps(_mm256_sub_ps(min8, origin_8), inv_dir_8);
    // const f256 t2 = _mm256_mul_ps(_mm256_sub_ps(max8, origin_8), inv_dir_8);

    const f256 vmax8 = _mm256_max_ps(t1, t2), vmin8 = _mm256_min_ps(t1, t2);

    const f256 tmax8 = _mm256_min_ps(vmax8, _mm256_movehdup_ps(vmax8));
    const f256 tmin8 = _mm256_max_ps(vmin8, _mm256_movehdup_ps(vmin8));
    const f32 tmina = tmin8.m256_f32[0] > vmin8.m256_f32[2] ? tmin8.m256_f32[0] : vmin8.m256_f32[2];
    const f32 tmaxa = tmax8.m256_f32[0] < vmax8.m256_f32[2] ? tmax8.m256_f32[0] : vmax8.m256_f32[2];
    const f32 tminb = tmin8.m256_f32[4] > vmin8.m256_f32[6] ? tmin8.m256_f32[4] : vmin8.m256_f32[6];
    const f32 tmaxb = tmax8.m256_f32[4] < vmax8.m256_f32[6] ? tmax8.m256_f32[4] : vmax8.m256_f32[6];

    const bool hita = (tmaxa > 0 && tmaxa >= tmina);
    const bool hitb = (tmaxb > 0 && tmaxb >= tminb);

    // const f32 tmaxa = dmin(vmax8.m256_f32[0], dmin(vmax8.m256_f32[1], vmax8.m256_f32[2]));
    // const f32 tmina = dmax(vmin8.m256_f32[0], dmax(vmin8.m256_f32[1], vmin8.m256_f32[2]));
    // const f32 tmaxb = dmin(vmax8.m256_f32[4], dmin(vmax8.m256_f32[5], vmax8.m256_f32[6]));
    // const f32 tminb = dmax(vmin8.m256_f32[4], dmax(vmin8.m256_f32[5], vmin8.m256_f32[6]));
    return {hita ? tmina : BIG_F32, hitb ? tminb : BIG_F32};
}