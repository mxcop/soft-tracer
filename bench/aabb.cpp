#include "aabb.h"

#include <glm/glm.hpp>
#include <random>

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

        state.ResumeTiming();
        benchmark::DoNotOptimize(ray_to_aabb(ro, rd, bbmin, bbmax));
        state.PauseTiming();
    }
}
BENCHMARK(bm_aabb_naive);

static void bm_aabb_naive_x8(benchmark::State& state) {
    std::uniform_real_distribution<float> rand_s(-100, 100);
    std::uniform_real_distribution<float> rand_u(0, 100);

    glm::vec3 ro = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
    glm::vec3 rd = glm::normalize(glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen)));
    
    for (auto _ : state) {
        for (int i = 0; i < 8; i++)
        {
            glm::vec3 bbmin = glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen));
            glm::vec3 bbmax = bbmin + glm::vec3(rand_u(gen), rand_u(gen), rand_u(gen));

            state.ResumeTiming();
            benchmark::DoNotOptimize(ray_to_aabb(ro, rd, bbmin, bbmax));
            state.PauseTiming();
        }
    }
}
BENCHMARK(bm_aabb_naive_x8);

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

        state.ResumeTiming();
        benchmark::DoNotOptimize(oct_ray_to_aabb(ray, &aabb));
        state.PauseTiming();
    }
}
BENCHMARK(bm_aabb_simd_x8);

static bool oct_ray_to_aabb(const Ray& ray, const AABB_256* aabb, const f256* cache) {
    f256 tmin = _mm256_set1_ps(0.0f);
    f256 tmax = _mm256_set1_ps(1'000'000.0f);

    for (u32 i = 0; i < 3; ++i) {
        f256 bmin = aabb->corners[ray.sign[i]][i];
        f256 bmax = aabb->corners[!ray.sign[i]][i];

        f256 dmin = _mm256_mul_ps(_mm256_sub_ps(bmin, cache[i]), cache[3 + i]);
        f256 dmax = _mm256_mul_ps(_mm256_sub_ps(bmax, cache[i]), cache[3 + i]);

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

static void bm_aabb_simd_cache_x8(benchmark::State& state) {
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

        state.ResumeTiming();
        benchmark::DoNotOptimize(oct_ray_to_aabb(ray, &aabb, cache));
        state.PauseTiming();
    }
}
BENCHMARK(bm_aabb_simd_cache_x8);

BENCHMARK_MAIN();
