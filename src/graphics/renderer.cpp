#include "renderer.h"

#include <chrono>
#include <random>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <imgui.h>

#include "ray.h"
#include "loaders/vox.h"

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

#if 0
    // for (int y = 0; y < 2; y++) {
    //     for (int x = 0; x < 2; x++) {
    //         for (int z = 0; z < 2; z++) {
    //             vvv.emplace_back(glm::vec3(x * 4.0f, z * 4.0f, y * 4.0f), glm::ivec3(8),
    //                              glm::vec3(0.0f));
    //         }
    //     }
    // }
    std::random_device seed;
    std::mt19937 gen(seed());
    std::uniform_real_distribution<float> rand_s(32, 64);

    for (int y = -8; y < 8; y++) {
        for (int x = -8; x < 8; x++) {
            vvv.emplace_back(glm::vec3(x * (128 / 8.0f), 0.0f, y * (128 / 8.0f)),
                             glm::ivec3(128, rand_s(gen), 128),
                             glm::vec3(0.0f));
        }
    }
#else
    // std::random_device seed;
    // std::mt19937 gen(seed());
    // std::uniform_real_distribution<float> rand_s(-10, 10);

    // for (u32 i = 0; i < 16; i++) {
    //     vvv.emplace_back(glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen)), glm::ivec3(8),
    //                      glm::vec3(0.0f));
    // }

    std::vector<u8> teapot_vox = load_vox_model("public/models/teapot.vox");

    vvv.emplace_back(glm::vec3(0.0f, 0.0f, 0.0f), glm::ivec3(8, 8, 8), glm::vec3(0.0f));
    vvv.emplace_back(glm::vec3(2.0f, 0.0f, 0.0f), glm::ivec3(16, 16, 16), glm::vec3(0.0f));
    vvv.emplace_back(glm::vec3(5.0f, 0.0f, 0.0f), glm::ivec3(32, 32, 32), glm::vec3(0.0f));
    vvv.emplace_back(glm::vec3(/*0.0f*/ 11.0f, 0.0f, 0.0f), glm::ivec3(64, 64, 64),
                     glm::vec3(0.0f), teapot_vox);
#endif

    auto start_time = std::chrono::steady_clock::now();
    // bvh = Bvh(vvv.size(), vvv);
    bvh = Bvh();
    bvh.build(vvv);
    auto end_time = std::chrono::steady_clock::now();
    db_build_time = (end_time - start_time).count() * 0.001f;
}

Renderer::~Renderer() { delete[] buffer; }

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

static GLuint trace(const Ray& ray, const Bvh* bvh) {
    f32 dist = bvh->intersect(ray);
    if (dist < BIG_F32) {
        return lerp_color(0xFFFFFFFF, 0x000000FF, dist / (ray.t + 10.0f));
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

    // Box box = {};
    // box.corners[0] = glm::vec3(-2.5f);
    // box.corners[1] = glm::vec3(2.5f);

#if 1
    constexpr i32 TILE_SIZE = 4;

    for (int y = 0; y < screen_height; y += TILE_SIZE) {
        for (int x = 0; x < screen_width; x += TILE_SIZE) {
            for (int v = 0; v < TILE_SIZE; v++) {
                for (int u = 0; u < TILE_SIZE; u++) {
                    int ix = x + u;
                    int iy = y + v;
                    glm::vec4 ray_end_ndc(((float)ix / (float)screen_width - 0.5f) * 2.0f,
                                          ((float)iy / (float)screen_height - 0.5f) * 2.0f, 0.0,
                                          1.0f);
                    glm::vec4 ray_end_world = ndc_to_world * ray_end_ndc;
                    ray_end_world /= ray_end_world.w;

                    /* NOTE: this normalize is very expensive! */
                    glm::vec3 ray_dir = glm::normalize(glm::vec3(ray_end_world - cam_pos_4));
                    Ray ray = Ray(cam_pos, ray_dir);

                    // buffer[x + y * screen_width] = trace(cam_pos_4, ray_dir, box, box_model);
                    buffer[ix + iy * screen_width] = trace(ray, &bvh);
                }
            }
        }
    }
#else
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
#endif

    // draw_aabb(glm::vec3(5.0f), glm::vec3(10.0f), 0x00FF00FF, view, proj);

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
