#include "screen-quad.h"

constexpr float quad[] = {-1.0f, -1.0f, 0.0f, 1.0f, -1.0f, 0.0f, -1.0f, 1.0f, 0.0f,
                          -1.0f, 1.0f,  0.0f, 1.0f, -1.0f, 0.0f, 1.0f,  1.0f, 0.0f};

ScreenQuad::ScreenQuad()
{
    /* Generate and bind buffer */
    glGenVertexArrays(1, &vao_handle);
    glBindVertexArray(vao_handle);

    /* Generate and bind buffer */
    glGenBuffers(1, &vbo_handle);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_handle);

    /* Upload buffer data */
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);

    /* Position attribute */
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    /* Unbind buffer */
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    vertices = (sizeof(quad) / sizeof(GLfloat)) / 3;
}

ScreenQuad::~ScreenQuad()
{
    glDeleteBuffers(1, &vbo_handle);
    glDeleteVertexArrays(1, &vao_handle);
}

void ScreenQuad::draw() const
{
    glBindBuffer(GL_ARRAY_BUFFER, vbo_handle);
    glBindVertexArray(vao_handle);
    glDrawArrays(GL_TRIANGLES, 0, vertices);
}
