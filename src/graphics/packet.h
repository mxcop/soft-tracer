#pragma once

struct alignas(16) FrustumPlane {
    glm::vec3 normal;
    float d;
};

/* (Infinite) Frustum without a near and far plane */
struct alignas(64) InfFrustum {
    /* Top, Right, Bottom, Left, in that order */
    FrustumPlane planes[6];

    InfFrustum() = default;
    /* Create frustum from 3 ray directions, top left, bottom right, and bottom left */
    InfFrustum(glm::vec3 origin, glm::vec3 tl_dir, glm::vec3 tr_dir, glm::vec3 br_dir,
               glm::vec3 bl_dir);
    InfFrustum(const glm::mat4& m);

    bool intersect_aabb(glm::vec3 bbmin, glm::vec3 bbmax) const;
};

struct RayPacket {};
