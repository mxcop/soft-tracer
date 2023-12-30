#pragma once

#include <benchmark/benchmark.h>
#include <glm/glm.hpp>

#include <xmmintrin.h>
#include <immintrin.h>

#include <stdint.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

typedef float f32;
typedef double f64;

typedef __m256 f256;

struct AABB_256 {
    union {
        f256 corners[2][3];
        struct {
            f256 min[3];
            f256 max[3];
        };
    };
};

static inline glm::ivec3 signbit(glm::vec3 _X) throw() {
    return glm::ivec3(signbit(_X.x), signbit(_X.y), signbit(_X.z));
}

struct Ray {
    glm::vec3 origin;
    glm::vec3 dir, inv_dir;
    glm::ivec3 sign;

    Ray() = delete;
    Ray(const glm::vec3& origin, const glm::vec3& dir) : origin(origin), dir(dir), inv_dir(1.0f / dir), sign(signbit(dir)) {};
};
