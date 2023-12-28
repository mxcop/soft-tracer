#pragma once

#include <glad/glad.h>

/**
 * @brief A quad that covers the entire screen. (for debugging only)
 */
class ScreenQuad
{
    /* [OpenGL] Vertex buffer object handle */
    GLuint vbo_handle = 0;
    /* [OpenGL] Vertex array object handle */
    GLuint vao_handle = 0;
    int vertices = 0;

  public:
    ScreenQuad();
    ~ScreenQuad();

    /* Not copy-able */
    ScreenQuad(const ScreenQuad&) = delete;
    ScreenQuad& operator=(const ScreenQuad&) = delete;

    void draw() const;
};
