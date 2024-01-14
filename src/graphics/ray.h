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
    f32 t = 2000.0f;

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

    float intersects_aabb_sse(const f128 bmin4, const f128 bmax4) const;

    inline glm::vec2 intersects_aabb2_avx(const f128& amin4, const f128& amax4, const f128& bmin4,
        const f128& bmax4) const {
        /* IMPORTANT! the mask saves a lot of CPU cycles,
         * it removes the 4th garbage element in the vectors. */

        /* Instead of "mask4 = _mm_cmpeq_ps(_mm_setzero_ps(), _mm_set_ps(1, 0, 0, 0))" */
        /* Using this bit hack is slightly faster in practice */
        const i256 mask8 = _mm256_set_epi32(0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
                                            0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF);
        const f256 fmask8 = reinterpret_cast<const f256&>(mask8); /* bit hack */

        const f256 min8 = _mm256_and_ps(_mm256_set_m128(bmin4, amin4), fmask8);
        const f256 max8 = _mm256_and_ps(_mm256_set_m128(bmax4, amax4), fmask8);

        const f256 origin_8 = _mm256_broadcast_ps(&origin_4);
        const f256 inv_dir_8 = _mm256_broadcast_ps(&inv_dir_4);
        const f256 t1 = _mm256_mul_ps(_mm256_sub_ps(min8, origin_8), inv_dir_8);
        const f256 t2 = _mm256_mul_ps(_mm256_sub_ps(max8, origin_8), inv_dir_8);

        #define dmin(a, b) (a < b ? a : b)
        #define dmax(a, b) (a > b ? a : b)

        const f256 vmax8 = _mm256_max_ps(t1, t2), vmin8 = _mm256_min_ps(t1, t2);
        const float tmaxa = dmin(vmax8.m256_f32[0], dmin(vmax8.m256_f32[1], vmax8.m256_f32[2]));
        const float tmina = dmax(vmin8.m256_f32[0], dmax(vmin8.m256_f32[1], vmin8.m256_f32[2]));
        const float tmaxb = dmin(vmax8.m256_f32[4], dmin(vmax8.m256_f32[5], vmax8.m256_f32[6]));
        const float tminb = dmax(vmin8.m256_f32[4], dmax(vmin8.m256_f32[5], vmin8.m256_f32[6]));
        return {(tmaxa > 0 && tmina < t && tmaxa >= tmina) ? tmina : BIG_F32,
                (tmaxb > 0 && tminb < t && tmaxb >= tminb) ? tminb : BIG_F32};
    }
};
