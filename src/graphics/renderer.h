#pragma once

#include <mutex>
#include <thread>
#include <glad/glad.h>

#include "ovh.h"

constexpr u32 NUM_THREADS = 2;

class Renderer {
    int screen_width, screen_height;
    GLuint* buffer = nullptr;
    GLuint buffer_handle = 0;

    glm::mat4 ndc_to_world;
    glm::vec4 cam_pos;
    std::vector<std::thread> trace_threads;

    std::condition_variable wake_cond;
    std::mutex wake_mutex;

    void draw_world_line(glm::vec3 p0, glm::vec3 p1, u32 c, glm::mat4 view, glm::mat4 proj);
    void draw_line(glm::ivec2 p0, glm::ivec2 p1, u32 c);
    void draw_aabb(glm::vec3 min, glm::vec3 max, u32 c, glm::mat4 view, glm::mat4 proj);

    Ovh bvh;

   public:
    Renderer() = delete;
    Renderer(int screen_width, int screen_height);
    ~Renderer();

    void render(float dt, float time, glm::vec3 cam_pos, glm::vec3 cam_dir);
    GLuint get_buf();
};
