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

#include <xmmintrin.h>
#include <immintrin.h>

typedef __m256 f256;
typedef __m256i i256;
typedef __m128 f128;
typedef __m128i i128;

constexpr f32 BIG_F32 = 1e30f;

#include <vector>
#include <array>
#include <memory>

#define GLM_SWIZZLE
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

union vf128 {
    struct {
        f32 x, y, z, w;
    };
    f32 cmp[4];
    f128 mm;
    struct {
        glm::vec3 v3;
        f32 _;
    };
};
