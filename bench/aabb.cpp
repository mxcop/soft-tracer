#include "aabb.h"

#include <glm/glm.hpp>
#include <random>
#include <chrono>

/**
 * @brief Ray to Axis Aligned Bounding Box intersection test.
 */
static bool ray_to_aabb(const glm::vec3& ro, const glm::vec3& rd, const glm::vec3& bbmin,
                        const glm::vec3& bbmax) {
    float tmin = 0.0, tmax = INFINITY;

    const glm::vec3 corners[2]{bbmin, bbmax};
    glm::vec3 inv_dir = 1.0f / rd;

    for (int d = 0; d < 3; ++d) {
        bool sign = signbit(inv_dir[d]);
        float bmin = corners[sign][d];
        float bmax = corners[!sign][d];

        float dmin = (bmin - ro[d]) * inv_dir[d];
        float dmax = (bmax - ro[d]) * inv_dir[d];

        tmin = std::max(dmin, tmin);
        tmax = std::min(dmax, tmax);
        /* Early out check, saves a lot of compute */
        if (tmax < tmin) return false;
    }

    return tmin < tmax;
}

std::random_device seed;
std::mt19937 gen(seed());

static void bm_aabb_naive(benchmark::State& state) {
    std::uniform_real_distribution<float> rand_s(-100, 100);
    std::uniform_real_distribution<float> rand_u(0, 100);

    glm::vec3 ro = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
    glm::vec3 rd = glm::normalize(glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen)));

    for (auto _ : state) {
        glm::vec3 bbmin = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
        glm::vec3 bbmax = bbmin + glm::vec3(rand_u(gen), rand_u(gen), rand_u(gen));

        auto start = std::chrono::high_resolution_clock::now();
        benchmark::DoNotOptimize(ray_to_aabb(ro, rd, bbmin, bbmax));
        auto end = std::chrono::high_resolution_clock::now();

        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
        state.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK(bm_aabb_naive)->UseManualTime();

static bool ray_aabb_intersect_avx(f256 ro, f256 rd, f256 bbmin, f256 bbmax) {
    //    f256 rayOrigin = _mm256_loadu_ps(&ray.origin[0]);
    //    f256 rayDirection = _mm256_loadu_ps(&ray.inv_dir[0]);
    //    f256 boxMin = _mm256_loadu_ps(&bbmin[0]);
    //    f256 boxMax = _mm256_loadu_ps(&bbmax[0]);

    f256 tmin = _mm256_sub_ps(bbmin, ro);
    f256 tmax = _mm256_sub_ps(bbmax, ro);

    static const f256 bmask = _mm256_castsi256_ps(_mm256_set1_epi32(-1));

    if (_mm256_movemask_ps(
            _mm256_or_ps(_mm256_cmp_ps(tmin, rd, _CMP_GT_OQ),
                         _mm256_cmp_ps(tmax, _mm256_xor_ps(rd, bmask), _CMP_GT_OQ))) != 0)
        return false;

    tmin = _mm256_max_ps(tmin, _mm256_setzero_ps());
    tmax = _mm256_min_ps(tmax, _mm256_set1_ps(-1.0f));
    tmin = _mm256_mul_ps(tmin, rd);
    tmax = _mm256_mul_ps(tmax, rd);

    tmin = _mm256_max_ps(tmin, _mm256_setzero_ps());
    tmax = _mm256_min_ps(tmax, _mm256_set1_ps(-1.0f));

    return _mm256_movemask_ps(_mm256_cmp_ps(tmax, tmin, _CMP_GE_OQ)) != 0;
}

static void bm_aabb_avx(benchmark::State& state) {
    std::uniform_real_distribution<float> rand_s(-100, 100);
    std::uniform_real_distribution<float> rand_u(0, 100);

    glm::vec3 ro = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
    glm::vec3 rd = glm::normalize(glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen)));
    Ray ray = Ray(ro, rd);

    for (auto _ : state) {
        glm::vec3 bbmin = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
        glm::vec3 bbmax = bbmin + glm::vec3(rand_u(gen), rand_u(gen), rand_u(gen));

        f256 rayOrigin = _mm256_loadu_ps(&ray.origin[0]);
        f256 rayDirection = _mm256_loadu_ps(&ray.inv_dir[0]);
        f256 boxMin = _mm256_loadu_ps(&bbmin[0]);
        f256 boxMax = _mm256_loadu_ps(&bbmax[0]);

        auto start = std::chrono::high_resolution_clock::now();
        benchmark::DoNotOptimize(ray_aabb_intersect_avx(rayOrigin, rayDirection, boxMin, boxMax));
        auto end = std::chrono::high_resolution_clock::now();

        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
        state.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK(bm_aabb_avx)->UseManualTime();

/* Copied from <https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/> */
inline static float intersects_aabb_sse_jacco(const f128 bbmin, const f128 bbmax, const f128 ro,
                                        const f128 ird) {
    static __m128 mask4 = _mm_cmpeq_ps(_mm_setzero_ps(), _mm_set_ps(1, 0, 0, 0));
    __m128 t1 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bbmin, mask4), ro), ird);
    __m128 t2 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bbmax, mask4), ro), ird);
    __m128 vmax4 = _mm_max_ps(t1, t2), vmin4 = _mm_min_ps(t1, t2);
    float tmax = fmin(vmax4.m128_f32[0], fmin(vmax4.m128_f32[1], vmax4.m128_f32[2]));
    float tmin = fmax(vmin4.m128_f32[0], fmax(vmin4.m128_f32[1], vmin4.m128_f32[2]));
    if (tmax >= tmin && tmin < 1000.0f && tmax > 0)
        return tmin;
    else
        return 1e30f;
}

static void bm_aabb_sse_jacco(benchmark::State& state) {
    std::uniform_real_distribution<float> rand_s(-100, 100);
    std::uniform_real_distribution<float> rand_u(0, 100);

    glm::vec3 ro = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
    glm::vec3 rd = glm::normalize(glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen)));
    Ray ray = Ray(ro, rd);

    for (auto _ : state) {
        glm::vec3 bbmin = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
        glm::vec3 bbmax = bbmin + glm::vec3(rand_u(gen), rand_u(gen), rand_u(gen));

        f128 rayOrigin = _mm_loadu_ps(&ray.origin[0]);
        f128 rayDirection = _mm_loadu_ps(&ray.inv_dir[0]);
        f128 boxMin = _mm_loadu_ps(&bbmin[0]);
        f128 boxMax = _mm_loadu_ps(&bbmax[0]);

        auto start = std::chrono::high_resolution_clock::now();
        float r = intersects_aabb_sse_jacco(boxMin, boxMax, rayOrigin, rayDirection);
        auto end = std::chrono::high_resolution_clock::now();
        benchmark::DoNotOptimize(r);

        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
        state.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK(bm_aabb_sse_jacco)->UseManualTime();

/* Adapted from <https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/> */
inline static float intersects_aabb_sse(const f128 bbmin, const f128 bbmax, const f128 ro,
                                        const f128 ird) {
    /* Idea to use fmsub to save 1 instruction came from <http://www.joshbarczak.com/blog/?p=787> */
    const f128 rd = _mm_mul_ps(ro, ird);
    const f128 t1 = _mm_fmsub_ps(bbmin, ird, rd);
    const f128 t2 = _mm_fmsub_ps(bbmax, ird, rd);

    const f128 vmax4 = _mm_max_ps(t1, t2), vmin4 = _mm_min_ps(t1, t2);
    const f32 tmax = dmin(vmax4.m128_f32[0], dmin(vmax4.m128_f32[1], vmax4.m128_f32[2]));
    const f32 tmin = dmax(vmin4.m128_f32[0], dmax(vmin4.m128_f32[1], vmin4.m128_f32[2]));

    const bool hit = (tmax > 0 && tmax >= tmin);
    return hit ? tmin : BIG_F32;
}

static void bm_aabb_sse(benchmark::State& state) {
    std::uniform_real_distribution<float> rand_s(-100, 100);
    std::uniform_real_distribution<float> rand_u(0, 100);

    glm::vec3 ro = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
    glm::vec3 rd = glm::normalize(glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen)));
    Ray ray = Ray(ro, rd);

    for (auto _ : state) {
        glm::vec3 bbmin = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
        glm::vec3 bbmax = bbmin + glm::vec3(rand_u(gen), rand_u(gen), rand_u(gen));

        f128 rayOrigin = _mm_loadu_ps(&ray.origin[0]);
        f128 rayDirection = _mm_loadu_ps(&ray.inv_dir[0]);
        f128 boxMin = _mm_loadu_ps(&bbmin[0]);
        f128 boxMax = _mm_loadu_ps(&bbmax[0]);

        auto start = std::chrono::high_resolution_clock::now();
        float r = intersects_aabb_sse(boxMin, boxMax, rayOrigin, rayDirection);
        auto end = std::chrono::high_resolution_clock::now();
        benchmark::DoNotOptimize(r);

        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
        state.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK(bm_aabb_sse)->UseManualTime();

static void bm_aabb_naive_x8(benchmark::State& state) {
    std::uniform_real_distribution<float> rand_s(-100, 100);
    std::uniform_real_distribution<float> rand_u(0, 100);

    glm::vec3 ro = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
    glm::vec3 rd = glm::normalize(glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen)));

    for (auto _ : state) {
        for (int i = 0; i < 8; i++) {
            glm::vec3 bbmin = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
            glm::vec3 bbmax = bbmin + glm::vec3(rand_u(gen), rand_u(gen), rand_u(gen));

            auto start = std::chrono::high_resolution_clock::now();
            benchmark::DoNotOptimize(ray_to_aabb(ro, rd, bbmin, bbmax));
            auto end = std::chrono::high_resolution_clock::now();

            auto elapsed_seconds =
                std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
            state.SetIterationTime(elapsed_seconds.count());
        }
    }
}
BENCHMARK(bm_aabb_naive_x8)->UseManualTime();

static bool oct_ray_to_aabb(const Ray& ray, const AABB_256* aabb) {
    /* TODO: maybe cache these large vectors? */
    f256 origin[3], dir_inv[3];
    for (u32 i = 0; i < 3; ++i) {
        origin[i] = _mm256_broadcast_ss(&ray.origin[i]);
        dir_inv[i] = _mm256_broadcast_ss(&ray.inv_dir[i]);
    }

    f256 tmin = _mm256_set1_ps(0.0f);
    f256 tmax = _mm256_set1_ps(1'000'000.0f);

    for (u32 i = 0; i < 3; ++i) {
        f256 bmin = aabb->corners[ray.sign[i]][i];
        f256 bmax = aabb->corners[!ray.sign[i]][i];

        f256 dmin = _mm256_mul_ps(_mm256_sub_ps(bmin, origin[i]), dir_inv[i]);
        f256 dmax = _mm256_mul_ps(_mm256_sub_ps(bmax, origin[i]), dir_inv[i]);

        tmin = _mm256_max_ps(dmin, tmin);
        tmax = _mm256_min_ps(dmax, tmax);
    }

    /* Use a mask to remove non-intersections (tmin <= tmax) */
    f256 mask = _mm256_cmp_ps(tmin, tmax, _CMP_LE_OQ);
    f256 result = _mm256_blendv_ps(_mm256_set1_ps(0.0f), tmin, mask);

    f32 results[8];
    _mm256_store_ps(results, result);
    /* If any result is > 0 there was an intersection */
    return (results[0] + results[1] + results[2] + results[3] + results[4] + results[5] +
            results[6] + results[7]) > 0.0f;
}

static void bm_aabb_simd_x8(benchmark::State& state) {
    std::uniform_real_distribution<float> rand_s(-100, 100);
    std::uniform_real_distribution<float> rand_u(0, 100);

    glm::vec3 ro = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
    glm::vec3 rd = glm::normalize(glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen)));
    Ray ray = Ray(ro, rd);

    for (auto _ : state) {
        AABB_256 aabb = AABB_256();
        for (u32 i = 0; i < 8; ++i) {
            glm::vec3 bbmin = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
            glm::vec3 bbmax = bbmin + glm::vec3(rand_u(gen), rand_u(gen), rand_u(gen));

            aabb.min[0].m256_f32[i] = bbmin.x;
            aabb.min[1].m256_f32[i] = bbmin.y;
            aabb.min[2].m256_f32[i] = bbmin.z;
            aabb.max[0].m256_f32[i] = bbmax.x;
            aabb.max[1].m256_f32[i] = bbmax.y;
            aabb.max[2].m256_f32[i] = bbmax.z;
        }

        auto start = std::chrono::high_resolution_clock::now();
        benchmark::DoNotOptimize(oct_ray_to_aabb(ray, &aabb));
        auto end = std::chrono::high_resolution_clock::now();

        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
        state.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK(bm_aabb_simd_x8)->UseManualTime();

static inline float _mm256_hadd(const f256& a) {
    f256 t1 = _mm256_hadd_ps(a, a);
    f256 t2 = _mm256_hadd_ps(t1, t1);
    __m128 t3 = _mm256_extractf128_ps(t2, 1);
    __m128 t4 = _mm_add_ss(_mm256_castps256_ps128(t2), t3);
    return _mm_cvtss_f32(t4);
}

static bool oct_ray_to_aabb_faster(const Ray& ray, const AABB_256* aabb, const f256* cache) {
    f256 tmin = _mm256_set1_ps(0.0f);
    f256 tmax = _mm256_set1_ps(INFINITY);

    for (u32 i = 0; i < 3; ++i) {
        const f256 bmin = aabb->corners[ray.sign[i]][i];
        const f256 bmax = aabb->corners[!ray.sign[i]][i];

        const f256 dmin = _mm256_mul_ps(_mm256_sub_ps(bmin, cache[i]), cache[3 + i]);
        const f256 dmax = _mm256_mul_ps(_mm256_sub_ps(bmax, cache[i]), cache[3 + i]);

        tmin = _mm256_max_ps(dmin, tmin);
        tmax = _mm256_min_ps(dmax, tmax);

        /* Use a mask to remove non-intersections (tmin <= tmax) */
        // const f256 mask = _mm256_cmp_ps(tmin, tmax, _CMP_LE_OQ);
        // const f256 result = _mm256_blendv_ps(_mm256_set1_ps(0.0f), tmin, mask);

        // if (_mm256_hadd(result)) return true;

        // 1 : tmax < tmin
        // 0 : tmax > tmin
        const f256 result = _mm256_sub_ps(tmin, tmax);

        // if (tmax < tmin) return false;
        if (_mm256_movemask_ps(result)) return false;
    }
    return true;
    /* Use a mask to remove non-intersections (tmin <= tmax) */
    // const f256 mask = _mm256_cmp_ps(tmin, tmax, _CMP_LE_OQ);
    // const f256 result = _mm256_blendv_ps(_mm256_set1_ps(0.0f), tmin, mask);

    /* If any result is > 0 there was an intersection */
    // return _mm256_hadd(result);
}

static void bm_aabb_simd_fast_x8(benchmark::State& state) {
    std::uniform_real_distribution<float> rand_s(-100, 100);
    std::uniform_real_distribution<float> rand_u(0, 100);

    glm::vec3 ro = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
    glm::vec3 rd = glm::normalize(glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen)));
    Ray ray = Ray(ro, rd);

    f256 cache[6];
    for (u32 i = 0; i < 3; ++i) {
        cache[i] = _mm256_broadcast_ss(&ray.origin[i]);
        cache[3 + i] = _mm256_broadcast_ss(&ray.inv_dir[i]);
    }

    for (auto _ : state) {
        AABB_256 aabb = AABB_256();
        for (u32 i = 0; i < 8; ++i) {
            glm::vec3 bbmin = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
            glm::vec3 bbmax = bbmin + glm::vec3(rand_u(gen), rand_u(gen), rand_u(gen));

            aabb.min[0].m256_f32[i] = bbmin.x;
            aabb.min[1].m256_f32[i] = bbmin.y;
            aabb.min[2].m256_f32[i] = bbmin.z;
            aabb.max[0].m256_f32[i] = bbmax.x;
            aabb.max[1].m256_f32[i] = bbmax.y;
            aabb.max[2].m256_f32[i] = bbmax.z;
        }

        auto start = std::chrono::high_resolution_clock::now();
        benchmark::DoNotOptimize(oct_ray_to_aabb_faster(ray, &aabb, cache));
        auto end = std::chrono::high_resolution_clock::now();

        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
        state.SetIterationTime(elapsed_seconds.count());
    }
}
BENCHMARK(bm_aabb_simd_fast_x8)->UseManualTime();

BENCHMARK_MAIN();
