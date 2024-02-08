#include "packet.h"

InfFrustum::InfFrustum(glm::vec3 origin, glm::vec3 tl_dir, glm::vec3 tr_dir, glm::vec3 br_dir,
                       glm::vec3 bl_dir) {
    planes[0].normal = glm::cross(tl_dir, tr_dir);  /* Top */
    planes[2].normal = -glm::cross(bl_dir, br_dir); /* Bottom */
    planes[3].normal = -glm::cross(tl_dir, bl_dir); /* Left */
    planes[1].normal = glm::cross(tr_dir, br_dir);  /* Right */

    // Assuming corners are ordered in a specific way:
    //  0: Near bottom left,  1: Near bottom right,  2: Near top left,  3: Near top right,
    //  4: Far bottom left,  5: Far bottom right,  6: Far top left,  7: Far top right

    //glm::vec3 corners[8] = {origin + bl_dir, origin + br_dir, origin + tl_dir, origin + tr_dir,
    //                        origin + bl_dir * 10000.0f, origin + br_dir * 10000.0f,
    //                        origin + tl_dir * 10000.0f, origin + tr_dir * 10000.0f
    //};

    //// Left plane
    //planes[0].normal = glm::normalize(glm::cross(corners[2] - corners[0], corners[3] - corners[1]));
    //planes[0].d = -glm::dot(planes[0].normal, corners[0]);

    //// Right plane
    //planes[1].normal = glm::normalize(glm::cross(corners[3] - corners[1], corners[2] - corners[0]));
    //planes[1].d = -glm::dot(planes[1].normal, corners[1]);

    //// Bottom plane
    //planes[2].normal = glm::normalize(glm::cross(corners[1] - corners[0], corners[5] - corners[4]));
    //planes[2].d = -glm::dot(planes[2].normal, corners[0]);

    //// Top plane
    //planes[3].normal = glm::normalize(glm::cross(corners[5] - corners[4], corners[6] - corners[2]));
    //planes[3].d = -glm::dot(planes[3].normal, corners[2]);

    //// Near plane
    //planes[4].normal = glm::normalize(glm::cross(corners[2] - corners[0], corners[1] - corners[3]));
    //planes[4].d = -glm::dot(planes[4].normal, corners[0]);

    //// Far plane
    //planes[5].normal = glm::normalize(glm::cross(corners[3] - corners[1], corners[6] - corners[7]));
    //planes[5].d = -glm::dot(planes[5].normal, corners[7]);

    /* Compute "d" value for each frustum plane */
    for (u32 i = 0; i < 4; ++i) {
        planes[i].normal = glm::normalize(planes[i].normal);
        planes[i].d = glm::dot(planes[i].normal, origin);
    }
}

InfFrustum::InfFrustum(const glm::mat4& _m) {
    glm::mat4 m = glm::transpose(_m);

    planes[0].normal.x = m[0][3] + m[0][0];
    planes[0].normal.y = m[1][3] + m[1][0];
    planes[0].normal.z = m[2][3] + m[2][0];
    planes[0].d = m[3][3] + m[3][0];

    planes[1].normal.x = m[0][3] - m[0][0];
    planes[1].normal.y = m[1][3] - m[1][0];
    planes[1].normal.z = m[2][3] - m[2][0];
    planes[1].d = m[3][3] - m[3][0];
    
    planes[2].normal.x = m[0][3] - m[0][1];
    planes[2].normal.y = m[1][3] - m[1][1];
    planes[2].normal.z = m[2][3] - m[2][1];
    planes[2].d = m[3][3] - m[3][1];
    
    planes[3].normal.x = m[0][3] + m[0][1];
    planes[3].normal.y = m[1][3] + m[1][1];
    planes[3].normal.z = m[2][3] + m[2][1];
    planes[3].d = m[3][3] + m[3][1];

    planes[4].normal.x = m[0][2];
    planes[4].normal.y = m[1][2];
    planes[4].normal.z = m[2][2];
    planes[4].d = m[3][2];

    planes[5].normal.x = m[0][3] - m[0][2];
    planes[5].normal.y = m[1][3] - m[1][2];
    planes[5].normal.z = m[2][3] - m[2][2];
    planes[5].d = m[3][3] - m[3][2];

    for (u32 i = 0; i < 6; ++i) {
        float length = sqrt((planes[i].normal.x * planes[i].normal.x) +
                            (planes[i].normal.y * planes[i].normal.y) +
                            (planes[i].normal.z * planes[i].normal.z));
        planes[i].normal.x /= length;
        planes[i].normal.y /= length;
        planes[i].normal.z /= length;
        planes[i].d /= length;
    }
}

static inline bool IsPointInsidePlane(const glm::vec3& point, const FrustumPlane& plane) {
    return plane.normal.x * point.x + plane.normal.y * point.y + plane.normal.z * point.z >=
           plane.d;
}

static inline bool IsAABBInsidePlane(const glm::vec3& bbmin, const glm::vec3& bbmax,
                                     const FrustumPlane& plane) {
    return (IsPointInsidePlane({bbmin.x, bbmin.y, bbmin.z}, plane) ||
            IsPointInsidePlane({bbmax.x, bbmin.y, bbmin.z}, plane) ||
            IsPointInsidePlane({bbmin.x, bbmax.y, bbmin.z}, plane) ||
            IsPointInsidePlane({bbmax.x, bbmax.y, bbmin.z}, plane));
}


bool InfFrustum::intersect_aabb(glm::vec3 bbmin, glm::vec3 bbmax) const {

    //{
    //    int i = 2;
    //    glm::vec3 vmin, vmax;
    //    if (planes[i].normal.x > 0) {
    //        vmin.x = bbmin.x, vmax.x = bbmax.x;
    //    } else {
    //        vmin.x = bbmax.x, vmax.x = bbmin.x;
    //    }

    //    if (planes[i].normal.y > 0) {
    //        vmin.y = bbmin.y, vmax.y = bbmax.y;
    //    } else {
    //        vmin.y = bbmax.y, vmax.y = bbmin.y;
    //    }

    //    if (planes[i].normal.z > 0) {
    //        vmin.z = bbmin.z, vmax.z = bbmax.z;
    //    } else {
    //        vmin.z = bbmax.z, vmax.z = bbmin.z;
    //    }

    //    const f32 dmin = glm::dot(planes[i].normal, vmin) + planes[i].d;
    //    // const f32 dmax = glm::dot(planes[i].normal, vmax) + planes[i].d;
    //    if (dmin > 0)
    //        return false;
    //    else
    //        return true;
    //}

    //glm::vec3 vmin, vmax;
    //for (u32 i = 0; i < 4; ++i) {
        //if (planes[i].normal.x > 0) {
        //    vmin.x = bbmin.x, vmax.x = bbmax.x;
        //} else {
        //    vmin.x = bbmax.x, vmax.x = bbmin.x;
        //}

        //if (planes[i].normal.y > 0) {
        //    vmin.y = bbmin.y, vmax.y = bbmax.y;
        //} else {
        //    vmin.y = bbmax.y, vmax.y = bbmin.y;
        //}

        //if (planes[i].normal.z > 0) {
        //    vmin.z = bbmin.z, vmax.z = bbmax.z;
        //} else {
        //    vmin.z = bbmax.z, vmax.z = bbmin.z;
        //}

        //const f32 dmin = glm::dot(planes[i].normal, bbmin) + planes[i].d;
        //if (dmin > 0) return false;

        // if (not IsPointInsidePlane(bbmin, planes[i])) return false;

        //u32 in_cnt = 8;
        //for (u32 j = 0; j < 8; ++j) {
        //    glm::vec3 corner;
        //    corner.x = (j & 1) ? bbmax.x : bbmin.x;
        //    corner.y = (j & 2) ? bbmax.y : bbmin.y;
        //    corner.z = (j & 4) ? bbmax.z : bbmin.z;

        //    float projection = glm::dot(planes[i].normal, corner) + planes[i].d;
        //    if (projection < 0.0f) {
        //        in_cnt--;
        //    } else {
        //        break;
        //    }
        //}
        //if (in_cnt == 0) return false;

        //const f32 dmin = glm::dot(planes[i].normal, vmin) + planes[i].d;
        //const f32 dmax = glm::dot(planes[i].normal, vmax) + planes[i].d;
        //if (dmin > 0) return false;

        //if (dmax >= 0) {
        //    // Calculate the distance from the AABB to the plane
        //    float distance = abs((glm::dot(planes[i].normal, vmax) + planes[i].d));
        //    if (distance < info.minDistance) {
        //        info.minDistance = distance;
        //    }
        //}
    //}
    //return true;

    //u32 in_cnt = 8;
    //for (u32 j = 0; j < 8; j++) {
    //    glm::vec3 corner;
    //    corner.x = (j & 1) ? bbmax.x : bbmin.x;
    //    corner.y = (j & 2) ? bbmax.y : bbmin.y;
    //    corner.z = (j & 4) ? bbmax.z : bbmin.z;

    //    for (u32 i = 0; i < 4; ++i) {
    //        float d = glm::dot(planes[i].normal, corner);
    //        if (d < planes[i].d) {
    //            in_cnt--;
    //            break;
    //        }
    //    }
    //}
    //return in_cnt > 0;

    bool intr = false;
    for (u32 i = 0; i < 6; i++) {
        glm::vec3 normal = planes[i].normal;
        float d = planes[i].d;

        glm::vec3 nearCorner, farCorner;
        if (normal.x > 0) {
            nearCorner.x = bbmax.x;
            farCorner.x = bbmin.x;
        }
        if (normal.y > 0) {
            nearCorner.y = bbmax.y;
            farCorner.y = bbmin.y;
        }
        if (normal.z > 0) {
            nearCorner.z = bbmax.z;
            farCorner.z = bbmin.z;
        }

        float nearDist = glm::dot(normal, nearCorner) + d;
        float farDist = glm::dot(normal, farCorner) + d;

        if (nearDist < 0) {
            return false;
        }

        //if (nearDist < 0 && farDist < 0) {
        //    // Both corners are behind the plane, so the AABB is outside the frustum.
        //    return false;
        //} else if (nearDist < 0 || farDist < 0) {
        //    // One corner is behind the plane, so the AABB intersects the frustum.
        //    intr = true;
        //}
    }
    return true;
}
