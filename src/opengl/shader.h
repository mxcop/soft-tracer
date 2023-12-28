#pragma once

#include <memory> /* std::shared_ptr */

#include <glad/glad.h>
#include <glm/glm.hpp>

/**
 * @brief Shader interface.
 */
class Shader
{
    /* Shader resource handle */
    std::shared_ptr<GLuint> handle;

  public:
    Shader() = default;
    /**
     * @param vert_path Path to the vertex shader source file.
     * @param frag_path Path to the fragment shader source file.
     */
    Shader(const char* vert_path, const char* frag_path);

    /**
     * @brief Compile the shader from source.
     *
     * @param vert_src Vertex shader source code.
     * @param frag_src Fragment shader source code.
     * @return True if the shader is loaded.
     */
    bool from_src(const char* vert_src, const char* frag_src);

    /* @return True if the shader is loaded. */
    bool loaded() const
    {
        return *handle;
    };

    /**
     * @brief Start using this shader program.
     */
    void use() const;

    /* Uniform functions */
    void set_float(const char* name, float value) const;
    void set_int(const char* name, int value) const;
    void set_vec2f(const char* name, const glm::vec2& vec) const;
    void set_vec3f(const char* name, const glm::vec3& vec) const;
    void set_mat4(const char* name, const glm::mat4& matrix) const;
};
