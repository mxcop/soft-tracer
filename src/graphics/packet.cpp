#include "packet.h"

InfFrustum::InfFrustum(const glm::mat4& vp) {
    glm::mat4 m = glm::transpose(vp);
    planes[Left] = m[3] + m[0];
    planes[Right] = m[3] - m[0];
    planes[Bottom] = m[3] + m[1];
    planes[Top] = m[3] - m[1];
    planes[Near] = m[3] + m[2];
    planes[Far] = m[3] - m[2];

    glm::vec3 crosses[] = {glm::cross(glm::vec3(planes[Left]), glm::vec3(planes[Right])),
                           glm::cross(glm::vec3(planes[Left]), glm::vec3(planes[Bottom])),
                           glm::cross(glm::vec3(planes[Left]), glm::vec3(planes[Top])),
                           glm::cross(glm::vec3(planes[Left]), glm::vec3(planes[Near])),
                           glm::cross(glm::vec3(planes[Left]), glm::vec3(planes[Far])),
                           glm::cross(glm::vec3(planes[Right]), glm::vec3(planes[Bottom])),
                           glm::cross(glm::vec3(planes[Right]), glm::vec3(planes[Top])),
                           glm::cross(glm::vec3(planes[Right]), glm::vec3(planes[Near])),
                           glm::cross(glm::vec3(planes[Right]), glm::vec3(planes[Far])),
                           glm::cross(glm::vec3(planes[Bottom]), glm::vec3(planes[Top])),
                           glm::cross(glm::vec3(planes[Bottom]), glm::vec3(planes[Near])),
                           glm::cross(glm::vec3(planes[Bottom]), glm::vec3(planes[Far])),
                           glm::cross(glm::vec3(planes[Top]), glm::vec3(planes[Near])),
                           glm::cross(glm::vec3(planes[Top]), glm::vec3(planes[Far])),
                           glm::cross(glm::vec3(planes[Near]), glm::vec3(planes[Far]))};

    points[0] = intersection<Left, Bottom, Near>(crosses);
    points[1] = intersection<Left, Top, Near>(crosses);
    points[2] = intersection<Right, Bottom, Near>(crosses);
    points[3] = intersection<Right, Top, Near>(crosses);
    points[4] = intersection<Left, Bottom, Far>(crosses);
    points[5] = intersection<Left, Top, Far>(crosses);
    points[6] = intersection<Right, Bottom, Far>(crosses);
    points[7] = intersection<Right, Top, Far>(crosses);

    for (int i = 0; i < 6; i++) {
        float len = glm::length(glm::vec3(planes[i]));
        planes[i] /= len;
    }
}

glm::vec4 calc_plane(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3) {
    glm::vec3 n = glm::normalize(glm::cross(p2 - p1, p3 - p1));
    float d = -glm::dot(n, p1);
    return glm::vec4(n, d);
}

InfFrustum::InfFrustum(glm::vec3 fo, glm::vec3 fd, glm::vec3 tl, glm::vec3 tr, glm::vec3 bl,
                       glm::vec3 br) {
    const float NEAR = 0.01f, FAR = 10000.0f;

    // Corners order: near bottom left, near bottom right, near top right, near top left,
    //                 far bottom left, far bottom right, far top right, far top left
    glm::vec3 corners[8]{fo + bl * NEAR, fo + br * NEAR, fo + tr * NEAR, fo + tl * NEAR,
                         fo + bl * FAR,  fo + br * FAR,  fo + tr * FAR,  fo + tl * FAR};

    planes[0] = calc_plane(corners[0], corners[1], corners[2]);  // Bottom
    planes[1] = calc_plane(corners[7], corners[6], corners[5]);  // Top
    planes[2] = calc_plane(corners[3], corners[2], corners[6]);  // Right
    planes[3] = calc_plane(corners[0], corners[4], corners[7]);  // Left
    planes[4] = calc_plane(corners[0], corners[3], corners[7]);  // Near
    planes[5] = calc_plane(corners[4], corners[5], corners[6]);  // Far
}

bool InfFrustum::intersect_point(glm::vec3 p) const {
    for (int i = 0; i < 6; i++) {
        if (glm::dot(planes[i], glm::vec4(p.x, p.y, p.z, 1.0f)) < 0.0f) {
            return false;
        }
    }
    return true;
}

std::pair<float, float> ProjectInterval(const glm::vec3& axis, const glm::vec3& bbmin,
                                        const glm::vec3& bbmax) {
    float r = glm::dot(axis, bbmax);
    float l = glm::dot(axis, bbmin);
    return std::make_pair(std::min(l, r), std::max(l, r));
}

bool InfFrustum::intersect_sphere(glm::vec3 p, float r) const {
    // for (int i = 0; i < 6; i++) {
    //     if (glm::dot(planes[i], glm::vec4(p.x, p.y, p.z, 1.0f)) < -r) {
    //         return false;
    //     }
    // }
    // glm::vec3 minp = {0, 0, 0}, maxp = {32, 32, 32};
    // for (int i = 0; i < 6; i++) {
    //     if ((glm::dot(planes[i], glm::vec4(minp.x, minp.y, minp.z, 1.0f)) < 0.0) &&
    //         (glm::dot(planes[i], glm::vec4(maxp.x, minp.y, minp.z, 1.0f)) < 0.0) &&
    //         (glm::dot(planes[i], glm::vec4(minp.x, maxp.y, minp.z, 1.0f)) < 0.0) &&
    //         (glm::dot(planes[i], glm::vec4(maxp.x, maxp.y, minp.z, 1.0f)) < 0.0) &&
    //         (glm::dot(planes[i], glm::vec4(minp.x, minp.y, maxp.z, 1.0f)) < 0.0) &&
    //         (glm::dot(planes[i], glm::vec4(maxp.x, minp.y, maxp.z, 1.0f)) < 0.0) &&
    //         (glm::dot(planes[i], glm::vec4(minp.x, maxp.y, maxp.z, 1.0f)) < 0.0) &&
    //         (glm::dot(planes[i], glm::vec4(maxp.x, maxp.y, maxp.z, 1.0f)) < 0.0)) {
    //         return false;
    //     }
    // }

    // glm::vec3 bbmin = {0, 0, 0}, bbmax = {64, 64, 64};
    // for (int i = 0; i < 6; i++) {
    //     glm::vec3 axis = planes[i];
    //     auto intervalAABB = ProjectInterval(axis, bbmin, bbmax);
    //     float d1 = glm::dot(axis, bbmin) + planes[i].w;
    //     float d2 = glm::dot(axis, bbmax) + planes[i].w;

    //    if ((d1 > 0 && d2 >= 0) || (d1 >= 0 && d2 > 0)) {
    //        continue;
    //    }

    //    if (intervalAABB.first > d2 || intervalAABB.second < d1) {
    //        return false;
    //    }
    //}

    glm::vec3 minp = {0, 0, 0}, maxp = {96, 96, 96};
    glm::vec3 bbmin = {0, 0, 0}, bbmax = {64, 64, 64};

    /* Check if AABB inside Frustum */
    for (int i = 0; i < 4; i++) {
        int out = 0;
        out += ((glm::dot(planes[i], glm::vec4(minp.x, minp.y, minp.z, 1.0f)) < 0.0) ? 1 : 0);
        out += ((glm::dot(planes[i], glm::vec4(maxp.x, minp.y, minp.z, 1.0f)) < 0.0) ? 1 : 0);
        out += ((glm::dot(planes[i], glm::vec4(minp.x, maxp.y, minp.z, 1.0f)) < 0.0) ? 1 : 0);
        out += ((glm::dot(planes[i], glm::vec4(maxp.x, maxp.y, minp.z, 1.0f)) < 0.0) ? 1 : 0);
        out += ((glm::dot(planes[i], glm::vec4(minp.x, minp.y, maxp.z, 1.0f)) < 0.0) ? 1 : 0);
        out += ((glm::dot(planes[i], glm::vec4(maxp.x, minp.y, maxp.z, 1.0f)) < 0.0) ? 1 : 0);
        out += ((glm::dot(planes[i], glm::vec4(minp.x, maxp.y, maxp.z, 1.0f)) < 0.0) ? 1 : 0);
        out += ((glm::dot(planes[i], glm::vec4(maxp.x, maxp.y, maxp.z, 1.0f)) < 0.0) ? 1 : 0);
        if (out == 8) return false;
    }

    //glm::vec3 corners[] = {
    //    glm::vec3(1, 0, 0),
    //    glm::vec3(0, 1, 0), glm::vec3(0, 0, 1),
    //    glm::vec3(-1, 0, 0), glm::vec3(0, -1, 0), glm::vec3(0, 0, -1)
    //};

    ///* Check if Frustum inside AABB */
    //for (int i = 0; i < 6; i++) {
    //    int out = 0;
    //    out += ((glm::dot(corners[i], points[0]) < 0.0) ? 1 : 0);
    //    out += ((glm::dot(corners[i], points[1]) < 0.0) ? 1 : 0);
    //    out += ((glm::dot(corners[i], points[2]) < 0.0) ? 1 : 0);
    //    out += ((glm::dot(corners[i], points[3]) < 0.0) ? 1 : 0);
    //    out += ((glm::dot(corners[i], points[4]) < 0.0) ? 1 : 0);
    //    out += ((glm::dot(corners[i], points[5]) < 0.0) ? 1 : 0);
    //    out += ((glm::dot(corners[i], points[6]) < 0.0) ? 1 : 0);
    //    out += ((glm::dot(corners[i], points[7]) < 0.0) ? 1 : 0);
    //    if (out == 8) return false;
    //}

    // int out;
    // out = 0;
    // for (int i = 0; i < 8; i++) out += ((points[i].x > maxp.x) ? 1 : 0);
    // if (out == 8) return false;
    // out = 0;
    // for (int i = 0; i < 8; i++) out += ((points[i].x < minp.x) ? 1 : 0);
    // if (out == 8) return false;
    // out = 0;
    // for (int i = 0; i < 8; i++) out += ((points[i].y > maxp.y) ? 1 : 0);
    // if (out == 8) return false;
    // out = 0;
    // for (int i = 0; i < 8; i++) out += ((points[i].y < minp.y) ? 1 : 0);
    // if (out == 8) return false;
    // out = 0;
    // for (int i = 0; i < 8; i++) out += ((points[i].z > maxp.z) ? 1 : 0);
    // if (out == 8) return false;
    // out = 0;
    // for (int i = 0; i < 8; i++) out += ((points[i].z < minp.z) ? 1 : 0);
    // if (out == 8) return false;

    return true;
}
