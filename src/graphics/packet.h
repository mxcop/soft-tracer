#pragma once

struct alignas(16) FrustumPlane {
    glm::vec3 normal;
    glm::vec3 point;
    float d;
};

/* (Infinite) Frustum without a near and far plane */
struct InfFrustum {
    glm::vec4 planes[6] = {};
    glm::vec3 points[8] = {};

    InfFrustum() = default;
    InfFrustum(const glm::mat4& vp);
    InfFrustum(glm::vec3 fo, glm::vec3 fd, glm::vec3 tl, glm::vec3 tr, glm::vec3 bl, glm::vec3 br);

    //void init(float angle, float ratio, float near_dist, float far_dist);
    //void init(glm::vec3 cam_pos, glm::vec3 ray_dir, glm::vec3 ray_tl, glm::vec3 ray_tr,
    //          glm::vec3 ray_br,
    //                      glm::vec3 ray_bl);
    //void init(glm::mat4 vp);
    //void update(glm::vec3& origin, glm::vec3& lookat, glm::vec3& up);

    bool intersect_point(glm::vec3 p) const;
    bool intersect_sphere(glm::vec3 p, float r) const;

    enum Planes {
        Left = 0,
        Right,
        Bottom,
        Top,
        Near,
        Far,
        Count,
        Combinations = Count * (Count - 1) / 2
    };

    template <Planes i, Planes j>
    struct ij2k {
        enum { k = i * (9 - i) / 2 + j - 1 };
    };

    template <Planes a, Planes b, Planes c>
    glm::vec3 intersection(const glm::vec3* crosses) {
        float D = glm::dot(glm::vec3(planes[a]), crosses[ij2k<b, c>::k]);
        glm::vec3 res =
            glm::mat3(crosses[ij2k<b, c>::k], -crosses[ij2k<a, c>::k], crosses[ij2k<a, b>::k]) *
            glm::vec3(planes[a].w, planes[b].w, planes[c].w);
        return res * (-1.0f / D);
    }

    // bool intersect_aabb(glm::vec3 bbmin, glm::vec3 bbmax) const;
};

/* SIMD SSE Ray packet */
struct RayPacket {
    f128 rox, roy, roz; /* Origin */
    f128 rdx, rdy, rdz; /* Inverse Direction */

    RayPacket() = delete;
    RayPacket(glm::vec3 ro, glm::vec3 r1, glm::vec3 r2, glm::vec3 r3, glm::vec3 r4) {
        rox = _mm_set_ps1(ro.x), roy = _mm_set_ps1(ro.y), roz = _mm_set_ps1(ro.z);
        rdx = _mm_set_ps(r1.x, r2.x, r3.x, r4.x);
        rdy = _mm_set_ps(r1.y, r2.y, r3.y, r4.y);
        rdz = _mm_set_ps(r1.z, r2.z, r3.z, r4.z);
    }

    //int intersect_nodes(const f128* nodes) {
    //    f128 tmin = _mm_setzero_ps();
    //    f128 tmax = _mm_set_ps1(BIG_F32);

    //    for (u32 i = 0; i < 3; ++i) {
    //        const f256 bmin = aabb->corners[ray.sign[i]][i];
    //        const f256 bmax = aabb->corners[!ray.sign[i]][i];

    //        const f256 dmin = _mm_mul_ps(_mm_sub_ps(bmin, cache[i]), cache[3 + i]);
    //        const f256 dmax = _mm_mul_ps(_mm_sub_ps(bmax, cache[i]), cache[3 + i]);

    //        tmin = _mm_max_ps(dmin, tmin);
    //        tmax = _mm_min_ps(dmax, tmax);

    //        /* Use a mask to remove non-intersections (tmin <= tmax) */
    //        // const f256 mask = _mm256_cmp_ps(tmin, tmax, _CMP_LE_OQ);
    //        // const f256 result = _mm256_blendv_ps(_mm256_set1_ps(0.0f), tmin, mask);

    //        // if (_mm256_hadd(result)) return true;

    //        // 1 : tmax < tmin
    //        // 0 : tmax > tmin
    //        const f256 result = _mm256_sub_ps(tmin, tmax);

    //        // if (tmax < tmin) return false;
    //        if (_mm256_movemask_ps(result)) return false;
    //    }
    //    return true;
    //}
};
