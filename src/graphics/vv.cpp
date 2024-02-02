#include "vv.h"

#include <glm/gtx/quaternion.hpp>

VoxelVolume::VoxelVolume(glm::vec3 pos, glm::ivec3 size, glm::vec3 rot) {
    glm::vec3 half_size = (glm::vec3(size) / VOXELS_PER_UNIT) / 2.0f;
    // aabb = AABB(pos - half_size, pos + half_size);
    aabb_min = pos;
    aabb_max = pos + glm::vec3(size) / VOXELS_PER_UNIT;
}

VoxelVolume::VoxelVolume(glm::vec3 pos, glm::ivec3 size, glm::vec3 rot, std::vector<u8> voxels) {
    glm::vec3 half_size = (glm::vec3(size) / VOXELS_PER_UNIT) / 2.0f;
    // aabb = AABB(pos - half_size, pos + half_size);
    aabb_min = pos;
    aabb_max = pos + glm::vec3(size) / VOXELS_PER_UNIT;

    glm::ivec3 vsize = (aabb_max - aabb_min) * VOXELS_PER_UNIT;

    data[0] = voxels; /* L1 */
    data[1].resize(data[0].size() / 8);
    data[2].resize(data[1].size() / 8);

    for (int z = 0; z < vsize.z; z += 2) {
        for (int y = 0; y < vsize.y; y += 2) {
            for (int x = 0; x < vsize.x; x += 2) {
                u32 sum = 0u;
                u32 div = 1u;
                {
                    u8 d = voxels[x + vsize.x * (y + vsize.z * z)];
                    if (d > 0) div++;
                    sum += d;
                }
                {
                    u8 d = voxels[(x + 1) + vsize.x * (y + vsize.z * z)];
                    if (d > 0) div++;
                    sum += d;
                }
                {
                    u8 d = voxels[(x + 1) + vsize.x * ((y + 1) + vsize.z * z)];
                    if (d > 0) div++;
                    sum += d;
                }
                {
                    u8 d = voxels[(x + 1) + vsize.x * ((y + 1) + vsize.z * (z + 1))];
                    if (d > 0) div++;
                    sum += d;
                }
                {
                    u8 d = voxels[x + vsize.x * ((y + 1) + vsize.z * z)];
                    if (d > 0) div++;
                    sum += d;
                }
                {
                    u8 d = voxels[x + vsize.x * ((y + 1) + vsize.z * (z + 1))];
                    if (d > 0) div++;
                    sum += d;
                }
                {
                    u8 d = voxels[x + vsize.x * (y + vsize.z * (z + 1))];
                    if (d > 0) div++;
                    sum += d;
                }
                {
                    u8 d = voxels[(x + 1) + vsize.x * (y + vsize.z * (z + 1))];
                    if (d > 0) div++;
                    sum += d;
                }
                sum /= div;
                data[1][(x / 2) + (vsize.x / 2) * ((y / 2) + (vsize.z / 2) * (z / 2))] =
                    static_cast<u8>(sum); /* L2 */
            }
        }
    }
    vsize /= 2;
    for (int z = 0; z < vsize.z; z += 2) {
        for (int y = 0; y < vsize.y; y += 2) {
            for (int x = 0; x < vsize.x; x += 2) {
                u32 sum = 0u;
                u32 div = 1u;
                {
                    u8 d = data[1][x + vsize.x * (y + vsize.z * z)];
                    if (d > 0) div++;
                    sum += d;
                }
                {
                    u8 d = data[1][(x + 1) + vsize.x * (y + vsize.z * z)];
                    if (d > 0) div++;
                    sum += d;
                }
                {
                    u8 d = data[1][(x + 1) + vsize.x * ((y + 1) + vsize.z * z)];
                    if (d > 0) div++;
                    sum += d;
                }
                {
                    u8 d = data[1][(x + 1) + vsize.x * ((y + 1) + vsize.z * (z + 1))];
                    if (d > 0) div++;
                    sum += d;
                }
                {
                    u8 d = data[1][x + vsize.x * ((y + 1) + vsize.z * z)];
                    if (d > 0) div++;
                    sum += d;
                }
                {
                    u8 d = data[1][x + vsize.x * ((y + 1) + vsize.z * (z + 1))];
                    if (d > 0) div++;
                    sum += d;
                }
                {
                    u8 d = data[1][x + vsize.x * (y + vsize.z * (z + 1))];
                    if (d > 0) div++;
                    sum += d;
                }
                {
                    u8 d = data[1][(x + 1) + vsize.x * (y + vsize.z * (z + 1))];
                    if (d > 0) div++;
                    sum += d;
                }
                sum /= div;
                data[2][(x / 2) + (vsize.x / 2) * ((y / 2) + (vsize.z / 2) * (z / 2))] =
                    static_cast<u8>(sum); /* L3 */
            }
        }
    }
}

void VoxelVolume::get_bounds(glm::vec3& out_min, glm::vec3& out_max) const {
    // glm::mat4 rot_mtx = glm::toMat4(glm::quat(rot));

    // glm::vec3 half_size = (glm::vec3(size) / VOXELS_PER_UNIT) / 2.0f;
    // out_min = -half_size;
    // out_max = half_size;

    // glm::mat4 model = glm::mat4(1.0f);
    // model = glm::translate(model, pos);
    // model *= rot_mtx;

    // out_min = model * glm::vec4(out_min, 1.0f);
    // out_max = model * glm::vec4(out_max, 1.0f);

    out_min = aabb_min;
    out_max = aabb_max;
}

glm::vec3 VoxelVolume::center() const { return aabb_min + (aabb_max - aabb_min) * 0.5f; }

glm::mat4 VoxelVolume::get_model() const {
    glm::mat4 model = glm::mat4(1.0f);
    // model = glm::translate(model, pos);
    // glm::mat4 rot_mtx = glm::toMat4(glm::quat(rot));
    // model *= rot_mtx;
    return model;
}

#define SHOW_STEPS 1

//void traverse(vec2 p0, vec2 p1) {
//    float lod = 4.0, lodinv = 1.0 / 4.0;
//    vec2 rd = p1 - p0;
//    vec2 p = floor(p0 * lodinv) * lod;
//    vec2 rdinv = 1.0 / rd;
//    vec2 stp = sign(rd) * lod;
//    vec2 delta = min(rdinv * stp, 1.0);
//    // start at intersection of ray with initial cell
//    vec2 t_max = abs((p + max(stp, vec2(0.0)) - p0) * rdinv);
//
//    float t = 0.0;
//    for (int i = 0; i < 128; ++i) {
//        set_source_rgba(0.2, 0.5, 1.0, 0.5);
//        vec2 pos = floor((p0 + (t + 0.000001) * rd) * lodinv) * lod;
//        rectangle(pos.x, pos.y, lod, lod);
//        fill();
//
//        t = min(t_max.x, t_max.y);
//        if (t > 1.0) break;
//
//        set_source_rgb(vec3(0.0));
//        circle(p0 + t * rd, 0.15);
//        fill();
//
//        vec2 cmp = step(t_max.xy, t_max.yx);
//        t_max += delta * cmp;
//        p += stp * cmp;
//
//        if (lod == 4.0 && i >= 2) {
//            lod = 2.0, lodinv = 1.0 / 2.0;
//            stp *= 0.5, delta *= 0.5;
//            p = floor((p0 + (t + 0.000001) * rd) * lodinv) * lod;
//            t_max = abs((p + max(stp, vec2(0.0)) - p0) * rdinv);
//        } else if (lod == 2.0 && i >= 6) {
//            lod = 1.0, lodinv = 1.0;
//            stp *= 0.5, delta *= 0.5;
//            p = floor((p0 + (t + 0.000001) * rd) * lodinv) * lod;
//            t_max = abs((p + max(stp, vec2(0.0)) - p0) * rdinv);
//        }
//    }
//}

//f32 intersect(const Ray& ray, const f32 tmin, const f32 tmax) {
//    float lod = 4, lodinv = 1.0f / 4;
//    glm::vec3 volume = (aabb_max - aabb_min) * VOXELS_PER_UNIT;
//
//    /* Calculate the ray start, end, and extend */
//    glm::vec3 p0 = ((ray.origin + ray.dir * tmin) - aabb_min) * VOXELS_PER_UNIT;
//    glm::vec3 p1 = ((ray.origin + ray.dir * tmax) - aabb_min) * VOXELS_PER_UNIT;
//    glm::vec3 extend = p1 - p0;
//    glm::vec3 inv_extend = 1.0f / extend;
//
//    /* Values required for traversal */
//    glm::vec3 pos = glm::floor(p0 * lodinv) * lod;
//    glm::vec3 step = glm::sign(extend) * lod;
//    glm::vec3 delta = glm::min(inv_extend * step, 1.0f);
//    glm::vec3 tvox = glm::abs((pos + glm::max(step, 0.0f) - p0) * inv_extend);
//    
//    constexpr int MAX_STEPS = 128;
//    f32 t = 0.0f;
//    for (u32 i = 0; i < MAX_STEPS; ++i) {
//        /* Get the "t" at the next voxel intersection */
//        t = std::min(std::min(tvox.x, tvox.y), tvox.z);
//        if (t > 1.0f) break;
//
//        /* Read the current voxel */
//        u8 voxel = fetch_voxel(pos * lodinv, extend, 2);
//
//        /* Move to the next voxel */
//        if (tvox.x < tvox.y) {
//            if (tvox.x < tvox.z) {
//                pos.x += step.x;
//                if (pos.x < 0 || pos.x >= volume.x) break;
//                tvox.x += delta.x;
//            } else {
//                pos.z += step.z;
//                if (pos.z < 0 || pos.z >= volume.z) break;
//                tvox.z += delta.z;
//            }
//        } else {
//            if (tvox.y < tvox.z) {
//                pos.y += step.y;
//                if (pos.y < 0 || pos.y >= volume.y) break;
//                tvox.y += delta.y;
//            } else {
//                pos.z += step.z;
//                if (pos.z < 0 || pos.z >= volume.z) break;
//                tvox.z += delta.z;
//            }
//        }
//    }
//
//    /* Calculate the size of the voxel volume in voxels */
//    glm::vec3 extend = (aabb_max - aabb_min) * VOXELS_PER_UNIT;
//
//    /* Calculate the starting voxel index */
//    glm::vec3 ray_pos = ((ray.origin + ray.dir * (tmin + 0.01f)) - aabb_min) * VOXELS_PER_UNIT;
//    glm::vec3 vox_pos = glm::floor(ray_pos);
//
//    /* Clamp the starting voxel index inside the voxel volume */
//    vox_pos = glm::clamp(vox_pos, glm::vec3(0), extend - glm::vec3(1));
//
//    /* Which direction each axis will step in -1 or 1 */
//    glm::vec3 step = glm::sign(ray.dir);
//
//    /* Indicates how far we must move (in units of t) to equal the width of a voxel */
//    glm::vec3 delta = glm::abs(ray.inv_dir);
//
//    /* Determine t at which the ray crosses the first voxel boundary */
//    glm::vec3 tmax = (glm::vec3(vox_pos) + step - ray_pos) * ray.inv_dir;
//
//    constexpr int MAX_STEPS = 128;
//    int i = 0;
//    for (; i < MAX_STEPS; ++i) {
//        /* Check the current voxel */
//        u8 voxel = fetch_voxel(vox_pos, extend, lod);
//        if (voxel > 0) {
//            
//#if SHOW_STEPS
//            return ((float)i / MAX_STEPS) * ray.t;
//#endif
//            return glm::distance(ray.origin, ray_pos / VOXELS_PER_UNIT + aabb_min);
//        }
//
//        /* Amanatides & Woo */
//        /* <http://www.cse.yorku.ca/~amana/research/grid.pdf> */
//        if (tmax.x < tmax.y) {
//            if (tmax.x < tmax.z) {
//                ray_pos.x += step.x;
//                vox_pos = glm::floor(ray_pos);
//                if (vox_pos.x < 0 || vox_pos.x >= extend.x) break;
//                tmax.x += delta.x;
//            } else {
//                ray_pos.z += step.z;
//                vox_pos = glm::floor(ray_pos);
//                if (vox_pos.z < 0 || vox_pos.z >= extend.z) break;
//                tmax.z += delta.z;
//            }
//        } else {
//            if (tmax.y < tmax.z) {
//                ray_pos.y += step.y;
//                vox_pos = glm::floor(ray_pos);
//                if (vox_pos.y < 0 || vox_pos.y >= extend.y) break;
//                tmax.y += delta.y;
//            } else {
//                ray_pos.z += step.z;
//                vox_pos = glm::floor(ray_pos);
//                if (vox_pos.z < 0 || vox_pos.z >= extend.z) break;
//                tmax.z += delta.z;
//            }
//        }
//    }
//#if SHOW_STEPS
//    return ((float)i / MAX_STEPS) * ray.t;
//#endif
//    return BIG_F32;
//}

// const glm::ivec3 prim_size = (aabb_max - aabb_min) * VOXELS_PER_UNIT;
// const int prim_max = prim_size.x * prim_size.y * prim_size.z;
//
///* Move up to the edge of the bounding box */
// const glm::vec3 sp = ray.origin + ray.dir * (tmin + 0.0001f);
// glm::vec3 p = sp;
//
///* Voxel position */
// const glm::vec3 vp = (sp - aabb_min) * VOXELS_PER_UNIT;
// glm::ivec3 idx = glm::min(glm::max(glm::ivec3(glm::floor(vp)), glm::ivec3(0)), prim_size);
//
///* Ray direction sign mask */
// glm::vec3 srd = glm::sign(ray.dir);
// glm::vec3 sd = (glm::vec3(idx) - vp + .5f - .5f * srd) * ray.inv_dir;
//
// for (int i = 0; i < 256; ++i) {
//     size_t ii = ((size_t)idx.z * prim_size.x * prim_size.y) + ((size_t)idx.y * prim_size.x) +
//     idx.x;
//
//     /* Index the voxel data */
//     u8 voxel = voxels[ii];
//
//     if (voxel > 0) {
//         return tmin + glm::distance(sp, p);
//     }
//
//     /* Compute the step mask */
//     glm::vec3 yzx = sd.yzx, zxy = sd.zxy;
//     glm::vec3 mask = glm::lessThanEqual(sd, glm::min(yzx, zxy));
//
//     /* Step to the next voxel */
//     sd += mask * srd * ray.inv_dir;
//     idx += mask * srd;
//     p += mask * srd * (ray.dir / VOXELS_PER_UNIT);
//
//     /* Check if we're still within the bounding volume */
//     if (glm::any(glm::lessThan(idx, glm::ivec3(0))) ||
//         glm::any(glm::greaterThanEqual(idx, prim_size))) {
//         break;
//     }
// }
// return BIG_F32;

// glm::vec3 normal;
// glm::vec3 ray_pos = ray.origin + ray.dir * (tmin + 0.1f);
// glm::ivec3 ray_idx = glm::floor(ray.origin);
// glm::vec3 ray_dir_sign = glm::sign(ray.dir);
//
///* Idk exactly how this works, so let's call it "magic" */
// glm::vec3 ray_magic = (glm::vec3(ray_idx) - ray.origin + .5f + .5f * ray_dir_sign) * ray.inv_dir;
//
// float t;
// for (int i = 0; i < 128; i++) {
//     glm::vec3 yzx = ray_magic.xyz, zxy = ray_magic.zxy;
//     glm::vec3 ray_step = glm::step(ray_magic, yzx) * glm::step(ray_magic, zxy);
//     ray_magic += ray_step * ray.inv_dir * ray_dir_sign;
//     ray_idx += ray_step * ray_dir_sign;
//     normal = ray_step * -ray_dir_sign;
//
//     glm::vec3 d = (glm::vec3(ray_idx) - ray.origin + .5f - .5f * ray_dir_sign) * ray.inv_dir;
//     t = std::max(d.x, std::max(d.y, d.z));
//
//     if (map(p) || t > ray.t) break;
// }
