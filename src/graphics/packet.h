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

struct RayPacket {};
