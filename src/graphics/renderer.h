#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>

class Renderer {
    int screen_width, screen_height;
    GLuint* buffer = nullptr;
    GLuint buffer_handle = 0;

   public:
    Renderer() = delete;
    Renderer(int screen_width, int screen_height);
    ~Renderer();

    void render(float dt, float time, glm::vec3 cam_pos, glm::vec3 cam_dir);
    GLuint get_buf();
};
