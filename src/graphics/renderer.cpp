#include "renderer.h"

#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <imgui.h>

#include "ray.h"

Renderer::Renderer(int screen_width, int screen_height)
    : screen_width(screen_width),
      screen_height(screen_height),
      buffer(new GLuint[screen_width * screen_height]{}) {
    glGenTextures(1, &buffer_handle);
    glBindTexture(GL_TEXTURE_2D, buffer_handle);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, screen_width, screen_height, 0, GL_RGBA,
                 GL_UNSIGNED_INT, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    std::vector<VoxelVolume> vvv;
    //vvv.emplace_back(glm::vec3(0.0f), glm::ivec3(8), glm::vec3(0.0f));
    //vvv.emplace_back(glm::vec3(4.0f, 0.0f, 0.0f), glm::ivec3(8), glm::vec3(0.0f));
    //vvv.emplace_back(glm::vec3(-4.0f, 0.0f, 0.0f), glm::ivec3(8), glm::vec3(0.0f));

    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            for (int z = 0; z < 8; z++) {
                vvv.emplace_back(glm::vec3(x * 2.0f, z * 2.0f, y * 2.0f), glm::ivec3(8),
                                 glm::vec3(0.0f));
            }
        }
    }

    bvh = Bvh(vvv.size(), vvv);
}

Renderer::~Renderer() { delete[] buffer; }

/**
 * @brief Ray to Axis Aligned Bounding Box intersection test.
 */
//static bool ray_to_aabb(const Ray& ray, const Box& box) {
//    float tmin = 0.0, tmax = INFINITY;
//
//    for (int d = 0; d < 3; ++d) {
//        bool sign = signbit(ray.dir_inv[d]);
//        float bmin = box.corners[sign][d];
//        float bmax = box.corners[!sign][d];
//
//        float dmin = (bmin - ray.origin[d]) * ray.dir_inv[d];
//        float dmax = (bmax - ray.origin[d]) * ray.dir_inv[d];
//
//        tmin = std::max(dmin, tmin);
//        tmax = std::min(dmax, tmax);
//        /* Early out check, saves a lot of compute */
//        if (tmax < tmin) return false;
//    }
//
//    return tmin < tmax;
//}

/**
 * @brief Ray to Oriented Bounding Box intersection test.
 */
//static float ray_to_obb(const glm::vec3& ro, const glm::vec3& rd, const Box& box,
//                        const glm::mat4& model) {
//    float t_min = 0.0, t_max = INFINITY;
//
//    /* "model[3]" holds the world position of the box */
//    glm::vec3 delta = glm::vec3(model[3]) - ro;
//
//    /* loop to be unrolled by the compiler */
//    for (int d = 0; d < 3; ++d) {
//        glm::vec3 axis = glm::vec3(model[d]);
//        float e = glm::dot(axis, delta), f_inv = 1.0f / glm::dot(rd, axis);
//
//        float t1 = (e + box.corners[0][d]) * f_inv;
//        float t2 = (e + box.corners[1][d]) * f_inv;
//
//        /* swap t1 & t2 so t1 is always the smallest */
//        if (t1 > t2) {
//            float temp = t1;
//            t1 = t2, t2 = temp;
//        }
//
//        t_min = std::max(t1, t_min);
//        t_max = std::min(t2, t_max);
//
//        /* early out check */
//        if (t_max < t_min) return -1.0f;
//    }
//    return t_min;
//}

static uint32_t lerp_color(uint32_t color1, uint32_t color2, float t) {
    constexpr uint32_t RBmask = 0xff00ff00;
    constexpr uint32_t GAmask = 0x00ff00ff;
    constexpr uint32_t one_q8 = 1u << 8;  // a fixed point rep of 1.0 with 8 fractional bits
    assert(t >= 0.0f && t <= 1.0f);
    uint32_t t_q8 = t * one_q8;
    uint32_t rb1 = (color1 & RBmask) >> 8;
    uint32_t rb2 = (color2 & RBmask) >> 8;
    uint32_t ga1 = (color1 & GAmask);
    uint32_t ga2 = (color2 & GAmask);

    uint32_t rb = ((rb1 * (one_q8 - t_q8)) + (rb2 * t_q8)) & RBmask;
    uint32_t ga = (((ga1 * (one_q8 - t_q8)) + (ga2 * t_q8)) >> 8) & GAmask;
    return rb | ga;
}

//static GLuint trace(const glm::vec4& ro, const glm::vec3& rd, const Box& box,
//                    const glm::mat4& box_model) {
//    float dist = ray_to_obb(ro, rd, box, box_model);
//    if (dist > 0.0f) {
//        return lerp_color(0xFFFFFFFF, 0x000000FF, dist / 1000.0f);
//    }
//    return 0x101010FF;
//}
static GLuint trace(const Ray& ray, const Bvh& bvh) {
    if (bvh.intersect(ray)) {
        return 0xFF0000FF;
    }
    return 0x101010FF;
}

void Renderer::render(float dt, float time, glm::vec3 cam_pos, glm::vec3 cam_dir) {
    glm::mat4 proj =
        glm::perspective(70.0f, static_cast<float>(screen_width) / screen_height, 0.01f, 100.0f);
    glm::mat4 view = glm::lookAt(cam_pos, cam_pos + cam_dir, glm::vec3(0, 1, 0));

    glm::mat4 ndc_to_world = glm::inverse(proj * view);
    glm::vec4 cam_pos_4 = glm::vec4(cam_pos, 0.0);

    glm::mat4 box_model = glm::mat4(1.0f);
    box_model = glm::rotate(box_model, time * 0.5f, glm::vec3(0.0f, 0.5f, 0.5f));
    // glm::mat4 box_model_inv = glm::inverse(box_model);

    //Box box = {};
    //box.corners[0] = glm::vec3(-2.5f);
    //box.corners[1] = glm::vec3(2.5f);

    for (int y = 0; y < screen_height; y++) {
        for (int x = 0; x < screen_width; x++) {
            glm::vec4 ray_end_ndc(((float)x / (float)screen_width - 0.5f) * 2.0f,
                                  ((float)y / (float)screen_height - 0.5f) * 2.0f, 0.0, 1.0f);
            glm::vec4 ray_end_world = ndc_to_world * ray_end_ndc;
            ray_end_world /= ray_end_world.w;

            /* NOTE: this normalize is very expensive! */
            glm::vec3 ray_dir = glm::normalize(ray_end_world - cam_pos_4);
            Ray ray = Ray(cam_pos, ray_dir);

            // buffer[x + y * screen_width] = trace(cam_pos_4, ray_dir, box, box_model);
            buffer[x + y * screen_width] = trace(ray, bvh);
        }
    }

    draw_aabb(glm::vec3(5.0f), glm::vec3(10.0f), 0x00FF00FF, view, proj);

    //{
    //    glm::vec3 p0 = {5.0f, 5.0f, 5.0f}, p1 = {10.0f, 5.0f, 5.0f};
    //    draw_world_line(p0, p1, 0xFF0000FF, view, proj);
    //}
    //{
    //    glm::vec3 p0 = {5.0f, 5.0f, 5.0f}, p1 = {5.0f, 10.0f, 5.0f};
    //    draw_world_line(p0, p1, 0xFF0000FF, view, proj);
    //}
    //{
    //    glm::vec3 p0 = {5.0f, 5.0f, 5.0f}, p1 = {5.0f, 5.0f, 10.0f};
    //    draw_world_line(p0, p1, 0xFF0000FF, view, proj);
    //}
}

void Renderer::draw_world_line(glm::vec3 p0, glm::vec3 p1, u32 c, glm::mat4 view, glm::mat4 proj) {
    glm::vec3 pp0 = glm::project(p0, view, proj, glm::vec4(0, 0, screen_width, screen_height));
    glm::vec3 pp1 = glm::project(p1, view, proj, glm::vec4(0, 0, screen_width, screen_height));

    draw_line(glm::ivec2(pp0), glm::ivec2(pp1), c);
}

void Renderer::draw_line(glm::ivec2 p0, glm::ivec2 p1, u32 c) {
    int dx = abs(p1.x - p0.x);
    int sx = p0.x < p1.x ? 1 : -1;
    int dy = -abs(p1.y - p0.y);
    int sy = p0.y < p1.y ? 1 : -1;
    int error = dx + dy;

    for (;;) {
        if (p0.x >= 0 && p0.y >= 0 && p0.x < screen_width && p0.y < screen_height)
            buffer[p0.x + p0.y * screen_width] = c;

        if (p0.x == p1.x && p0.y == p1.y) break;
        int e2 = 2 * error;
        if (e2 >= dy) {
            if (p0.x == p1.x) break;
            error += dy;
            p0.x += sx;
        }
        if (e2 <= dx) {
            if (p0.y == p1.y) break;
            error += dx;
            p0.y += sy;
        }
    }
}

void Renderer::draw_aabb(glm::vec3 min, glm::vec3 max, u32 c, glm::mat4 view, glm::mat4 proj) {
    glm::vec3 p0 = min;
    glm::vec3 p1 = glm::vec3(min.x, min.y, max.z);
    glm::vec3 p2 = glm::vec3(min.x, max.y, min.z);
    glm::vec3 p3 = glm::vec3(max.x, min.y, min.z);
    glm::vec3 p4 = max;
    glm::vec3 p5 = glm::vec3(max.x, max.y, min.z);
    glm::vec3 p6 = glm::vec3(max.x, min.y, max.z);
    glm::vec3 p7 = glm::vec3(min.x, max.y, max.z);

    draw_world_line(p0, p1, c, view, proj);
    draw_world_line(p0, p2, c, view, proj);
    draw_world_line(p0, p3, c, view, proj);
    draw_world_line(p1, p6, c, view, proj);
    draw_world_line(p3, p6, c, view, proj);
    draw_world_line(p1, p7, c, view, proj);

    draw_world_line(p4, p5, c, view, proj);
    draw_world_line(p4, p6, c, view, proj);
    draw_world_line(p4, p7, c, view, proj);
    draw_world_line(p5, p2, c, view, proj);
    draw_world_line(p7, p2, c, view, proj);
    draw_world_line(p5, p3, c, view, proj);
}

GLuint Renderer::get_buf() {
    glBindTexture(GL_TEXTURE_2D, buffer_handle);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, screen_width, screen_height, GL_RGBA,
                    GL_UNSIGNED_INT_8_8_8_8, buffer);
    glBindTexture(GL_TEXTURE_2D, 0);
    return buffer_handle;
}
