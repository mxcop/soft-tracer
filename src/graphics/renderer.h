#pragma once

#include <glad/glad.h>

#include "bvh.h"

class Renderer {
    int screen_width, screen_height;
    GLuint* buffer = nullptr;
    GLuint buffer_handle = 0;

    void draw_world_line(glm::vec3 p0, glm::vec3 p1, u32 c, glm::mat4 view, glm::mat4 proj);
    void draw_line(glm::ivec2 p0, glm::ivec2 p1, u32 c);
    void draw_aabb(glm::vec3 min, glm::vec3 max, u32 c, glm::mat4 view, glm::mat4 proj);

    Bvh bvh;

   public:
    f32 db_build_time = 0.0f;

    Renderer() = delete;
    Renderer(int screen_width, int screen_height);
    ~Renderer();

    void render(float dt, float time, glm::vec3 cam_pos, glm::vec3 cam_dir);
    GLuint get_buf();

    inline glm::vec3 make_ray(const glm::vec3& origin, const glm::mat4& ndc_to_world, f32 x,
                              f32 y) {
        glm::vec4 ray_end_ndc((x / screen_width - 0.5f) * 2.0f, (y / screen_height - 0.5f) * 2.0f,
                              0.0, 1.0f);
        glm::vec4 ray_end_world = ndc_to_world * ray_end_ndc;
        glm::vec3 ray_end = glm::vec3(ray_end_world) / ray_end_world.w;

        glm::vec3 ray_dir = ray_end - origin;
        if (ray_dir.x == 0) ray_dir.x = 0.000001f;
        if (ray_dir.y == 0) ray_dir.y = 0.000001f;
        if (ray_dir.z == 0) ray_dir.z = 0.000001f;

        /* NOTE: this normalize is very expensive! */
        return glm::normalize(ray_dir);
    }
};
