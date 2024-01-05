#include "renderer.h"

#include <random>
#include <algorithm>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <imgui.h>

#include "ray.h"

std::atomic<u32> traces_finished = 0;
std::atomic<bool> tracing_finished = false;
//std::atomic_bool kill_threads = false;

static GLuint trace(const Ray& ray, const Ovh& bvh) {
    if (bvh.intersect(ray)) {
        return 0xFF0000FF;
    }
    return 0x101010FF;
}

struct ThreadBundle {
    GLuint* buffer;
    std::condition_variable* wake_cond;
    std::mutex* wake_mutex;
    const glm::mat4* ndc_to_world;
    const glm::vec4* cam_pos;
    ThreadBundle(GLuint* buffer, const glm::mat4* ndc_to_world,
                 const glm::vec4* cam_pos, std::condition_variable* wake_cond,
                 std::mutex* wake_mutex)
        : buffer(buffer), ndc_to_world(ndc_to_world), cam_pos(cam_pos), wake_cond(wake_cond), wake_mutex(wake_mutex){};
};

static void trace_thread(ThreadBundle bundle, u32 thread_id, const Ovh& bvh, glm::ivec2 screen) {
    const u32 pixels = screen.x * screen.y;

    const float inv_scr_w = 1.0f / screen.x;
    const float inv_scr_h = 1.0f / screen.y;

    const u32 rows = screen.y / NUM_THREADS;
    const u32 offset = rows * thread_id;

    //u32* thread_buffer = (u32*)_malloca(rows * screen.x * sizeof(u32));
    std::vector<GLuint> thread_buffer(rows * screen.x);
    //thread_buffer.resize(rows * screen.x * 2);

    for (;;) {
        //u32 i = trace_cnt.fetch_add(1) + 1;
        //if (i >= pixels) {
        //    std::unique_lock<std::mutex> lock(*bundle.wake_mutex);
        //    bundle.wake_cond->wait(lock);
        //}
        std::unique_lock<std::mutex> lock(*bundle.wake_mutex);
        bundle.wake_cond->wait(lock);

        for (u32 y = offset; y < offset + rows; y++) {
            for (u32 x = 0; x < screen.x; x++) {
                glm::vec4 ray_end_ndc((x * inv_scr_w - 0.5f) * 2.0f, (y * inv_scr_h - 0.5f) * 2.0f,
                                      0.0, 1.0f);
                glm::vec4 ray_end_world = *bundle.ndc_to_world * ray_end_ndc;
                ray_end_world /= ray_end_world.w;

                /* NOTE: this normalize is very expensive! */
                glm::vec3 ray_dir =
                    ray_end_world - *bundle.cam_pos;  // glm::normalize(ray_end_world - *cam_pos);
                Ray ray = Ray(*bundle.cam_pos, ray_dir);

                thread_buffer[x + (y - offset) * screen.x] = trace(ray, bvh);
            }
        }

        /* WARNING: this is NOT thread safe access! */
        /* However in this case it is safe because no two threads will access the same pixel
         */
        memcpy(&bundle.buffer[offset * screen.x], &thread_buffer.front(),
               (rows * screen.x) * sizeof(u32));

        traces_finished.fetch_add(1);
        if (traces_finished.load() >= NUM_THREADS) {
            tracing_finished = true;
            tracing_finished.notify_one();
        }
        
        //copies_finished.fetch_add(1);

        //int x = i % screen.x;
        //int y = i * inv_scr_w;

        
    }
}

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

    // for (int y = 0; y < 32; y++) {
    //     for (int x = 0; x < 32; x++) {
    //         for (int z = 0; z < 32; z++) {
    //             vvv.emplace_back(glm::vec3(x * 4.0f, z * 4.0f, y * 4.0f), glm::ivec3(8),
    //                              glm::vec3(0.0f));
    //         }
    //     }
    // }

    std::random_device seed;
    std::mt19937 gen(seed());
    std::uniform_real_distribution<float> rand_s(-100, 100);

    for (u32 i = 0; i < 1024; i++)
    {
        vvv.emplace_back(glm::vec3(rand_s(gen), rand_s(gen), rand_s(gen)), glm::ivec3(8), glm::vec3(0.0f));
    }
    
    bvh = Ovh(vvv.size(), vvv);

    ThreadBundle bundle(buffer, &ndc_to_world, &cam_pos, &wake_cond, &wake_mutex);

    for (u32 i = 0; i < NUM_THREADS - 1; ++i) {
        // trace_threads.emplace_back();
        auto th = std::thread(trace_thread, bundle, i + 1, bvh, glm::ivec2(screen_width, screen_height));
        // trace_threads.emplace_back(std::move(th));
        th.detach();
    }
}

Renderer::~Renderer() {
    //kill_threads = true;
    //for (u32 i = 0; i < 8; ++i) {
    //    trace_threads[i].join();
    //}
    delete[] buffer; 
}

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

void Renderer::render(float dt, float time, glm::vec3 cam_pos, glm::vec3 cam_dir) {
    glm::mat4 proj =
        glm::perspective(70.0f, static_cast<float>(screen_width) / screen_height, 0.01f, 100.0f);
    glm::mat4 view = glm::lookAt(cam_pos, cam_pos + cam_dir, glm::vec3(0, 1, 0));

    this->ndc_to_world = glm::inverse(proj * view);
    this->cam_pos = glm::vec4(cam_pos, 0.0);

    traces_finished.store(0);
    tracing_finished = false;
    // copies_finished.store(0);
    wake_cond.notify_all();

    //const int pixels = screen_width * screen_height;

    //for(;;) {
    //    u32 i = ++trace_cnt;
    //    if (i >= pixels) break;

    //    int x = i % screen_width;
    //    int y = i / screen_width;

    //    glm::vec4 ray_end_ndc(((float)x / (float)screen_width - 0.5f) * 2.0f,
    //                            ((float)y / (float)screen_height - 0.5f) * 2.0f, 0.0, 1.0f);
    //    glm::vec4 ray_end_world = ndc_to_world * ray_end_ndc;
    //    ray_end_world /= ray_end_world.w;

    //    /* NOTE: this normalize is very expensive! */
    //    glm::vec3 ray_dir = ray_end_world - glm::vec4(cam_pos, 0.0);// glm::normalize(ray_end_world - glm::vec4(cam_pos, 0.0));
    //    Ray ray = Ray(cam_pos, ray_dir);

    //    /* WARNING: this is NOT thread safe access! */
    //    /* However in this case it is safe because no two threads will access the same pixel */
    //    buffer[x + y * screen_width] = trace(ray, bvh);
    //}

    // glm::mat4 box_model = glm::mat4(1.0f);
    // box_model = glm::rotate(box_model, time * 0.5f, glm::vec3(0.0f, 0.5f, 0.5f));
    // glm::mat4 box_model_inv = glm::inverse(box_model);

    //Box box = {};
    //box.corners[0] = glm::vec3(-2.5f);
    //box.corners[1] = glm::vec3(2.5f);

    const float inv_scr_w = 1.0f / screen_width;
    const float inv_scr_h = 1.0f / screen_height;

    const int rows = screen_height / NUM_THREADS;
    const int offset = rows * 0;

    for (u32 y = offset; y < offset + rows; y++) {
        for (u32 x = 0; x < screen_width; x++) {
            glm::vec4 ray_end_ndc((x * inv_scr_w - 0.5f) * 2.0f,
                                  (y * inv_scr_h - 0.5f) * 2.0f, 0.0, 1.0f);
            glm::vec4 ray_end_world = ndc_to_world * ray_end_ndc;
            ray_end_world /= ray_end_world.w;

            /* NOTE: this normalize is very expensive! */
            glm::vec3 ray_dir =
                ray_end_world -
                glm::vec4(cam_pos, 0.0);  // glm::normalize(ray_end_world - cam_pos_4);
            Ray ray = Ray(cam_pos, ray_dir);

            // buffer[x + y * screen_width] = trace(cam_pos_4, ray_dir, box, box_model);
            buffer[x + y * screen_width] = trace(ray, bvh);
        }
    }

    traces_finished.fetch_add(1);
    if (traces_finished.load() < NUM_THREADS) {
        tracing_finished.wait(false);
    }

    //copies_finished.fetch_add(1);
    //while (traces_finished.load() < NUM_THREADS);
    //while (copies_finished.load() < NUM_THREADS);

    draw_aabb(glm::vec3(5.0f), glm::vec3(10.0f), 0x00FF00FF, view, proj);
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
