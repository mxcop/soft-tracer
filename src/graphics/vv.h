#pragma once

#include "aabb.h"
#include "ray.h"

constexpr int LEVELS_OF_DETAIL = 4;

struct VoxelVolume {
    /* Center position of the volume */
    // glm::vec3 pos;
    /* Size of the volume in voxels */
    // glm::ivec3 size;
    // glm::vec3 rot;

    union {
        f128 aabb_min4;
        struct {
            glm::vec3 aabb_min;
            f32 _;
        };
    };
    union {
        f128 aabb_max4;
        struct {
            glm::vec3 aabb_max;
            f32 _;
        };
    };
    std::array<std::vector<u8>, LEVELS_OF_DETAIL> data;

    // AABB aabb;

    VoxelVolume(glm::vec3 pos, glm::ivec3 size, glm::vec3 rot);
    VoxelVolume(glm::vec3 pos, glm::ivec3 size, glm::vec3 rot, std::vector<u8> voxels);

    void get_bounds(glm::vec3& out_min, glm::vec3& out_max) const;
    glm::vec3 center() const;
    glm::mat4 get_model() const;

    static inline float _min(float x, float y) { return x < y ? x : y; }
    static inline float _max(float x, float y) { return x > y ? x : y; }

    static inline glm::vec3 _vmin(glm::vec3 x, float y) {
        return glm::vec3(_min(x.x, y), _min(x.y, y), _min(x.z, y));
    }
    static inline glm::vec3 _vmax(glm::vec3 x, float y) {
        return glm::vec3(_max(x.x, y), _max(x.y, y), _max(x.z, y));
    }

    static inline glm::vec3 mulinv(glm::vec3 v) { 
        glm::vec3 vr = 1.0f / v;
        if (isinf(vr.x)) vr.x = 0.0f;
        if (isinf(vr.y)) vr.y = 0.0f;
        if (isinf(vr.z)) vr.z = 0.0f;
        return vr;
    }

#if 1
    f32 intersect(const Ray& ray, const f32 tmin, const f32 tmax) const;
#else
    inline f32 intersect(const Ray& ray, const f32 tmin, const f32 dummy) const {
        /* Calculate the size of the voxel volume in voxels */
        glm::vec3 extend = (aabb_max - aabb_min) * VOXELS_PER_UNIT;
        
        /* Calculate the starting voxel index */
        glm::vec3 ray_pos = ((ray.origin + ray.dir * (tmin + 0.01f)) - aabb_min) *
        VOXELS_PER_UNIT; glm::vec3 vox_pos = glm::floor(ray_pos);
        
        /* Clamp the starting voxel index inside the voxel volume */
        vox_pos = glm::clamp(vox_pos, glm::vec3(0), extend - glm::vec3(1));
        
        /* Which direction each axis will step in -1 or 1 */
        glm::vec3 step = glm::sign(ray.dir);
        
        /* Indicates how far we must move (in units of t) to equal the width of a voxel */
        glm::vec3 delta = glm::abs(ray.inv_dir);
        
        /* Determine t at which the ray crosses the first voxel boundary */
        glm::vec3 tmax = (glm::vec3(vox_pos) + step - ray_pos) * ray.inv_dir;
        
        constexpr int MAX_STEPS = 128;
        int i = 0;
        for (; i < MAX_STEPS; ++i) {
            /* Check the current voxel */
            u8 voxel = fetch_voxel(vox_pos, extend, 0);
            if (voxel > 0) {
                // return ((float)i / MAX_STEPS) * ray.t;
                return tmin;
            }
        
            /* Amanatides & Woo */
            /* <http://www.cse.yorku.ca/~amana/research/grid.pdf> */
            if (tmax.x < tmax.y) {
                if (tmax.x < tmax.z) {
                    ray_pos.x += step.x;
                    vox_pos = glm::floor(ray_pos);
                    if (vox_pos.x < 0 || vox_pos.x >= extend.x) break;
                    tmax.x += delta.x;
                } else {
                    ray_pos.z += step.z;
                    vox_pos = glm::floor(ray_pos);
                    if (vox_pos.z < 0 || vox_pos.z >= extend.z) break;
                    tmax.z += delta.z;
                }
            } else {
                if (tmax.y < tmax.z) {
                    ray_pos.y += step.y;
                    vox_pos = glm::floor(ray_pos);
                    if (vox_pos.y < 0 || vox_pos.y >= extend.y) break;
                    tmax.y += delta.y;
                } else {
                    ray_pos.z += step.z;
                    vox_pos = glm::floor(ray_pos);
                    if (vox_pos.z < 0 || vox_pos.z >= extend.z) break;
                    tmax.z += delta.z;
                }
            }
        }
        // return ((float)i / MAX_STEPS) * ray.t;
        return BIG_F32;
    }
#endif

   private:
    u8 fetch_voxel(glm::vec3 pos, glm::vec3 size, int lod) const {
        /*if (glm::any(glm::lessThan(pos, glm::vec3(0))) ||
            glm::any(glm::greaterThanEqual(pos, size))) {
            return 0;
        }*/
        int i = (pos.z * size.x * size.y) + (pos.y * size.x) + pos.x;
        return data[lod][i];
    }
};
