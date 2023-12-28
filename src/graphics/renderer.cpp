#include "renderer.h"

#include <glm/glm.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>

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
}

Renderer::~Renderer() { delete[] buffer; }

/// Parametric representation of a ray.
struct Ray {
    glm::vec3 origin;
    glm::vec3 dir;
    glm::vec3 dir_inv;
};

/// An axis-aligned bounding box.
struct Box {
    glm::vec3 corners[2];
};

/**
 * @brief Ray to Axis Aligned Bounding Box intersection test.
 */
static bool ray_to_aabb(const Ray& ray, const Box& box) {
    float tmin = 0.0, tmax = INFINITY;

    for (int d = 0; d < 3; ++d) {
        bool sign = signbit(ray.dir_inv[d]);
        float bmin = box.corners[sign][d];
        float bmax = box.corners[!sign][d];

        float dmin = (bmin - ray.origin[d]) * ray.dir_inv[d];
        float dmax = (bmax - ray.origin[d]) * ray.dir_inv[d];

        tmin = std::max(dmin, tmin);
        tmax = std::min(dmax, tmax);
        /* Early out check, saves a lot of compute */
        if (tmax < tmin) return false;
    }

    return tmin < tmax;
}

/**
 * @brief Ray to Oriented Bounding Box intersection test.
 */
static float ray_to_obb(const glm::vec3& ro, const glm::vec3& rd, const Box& box,
                        const glm::mat4& model) {
    float t_min = 0.0, t_max = INFINITY;

    /* "model[3]" holds the world position of the box */
    glm::vec3 delta = glm::vec3(model[3]) - ro;

    /* loop to be unrolled by the compiler */
    for (int d = 0; d < 3; ++d) {
        glm::vec3 axis = glm::vec3(model[d]);
        float e = glm::dot(axis, delta), f_inv = 1.0f / glm::dot(rd, axis);

        float t1 = (e + box.corners[0][d]) * f_inv;
        float t2 = (e + box.corners[1][d]) * f_inv;

        /* swap t1 & t2 so t1 is always the smallest */
        if (t1 > t2) {
            float temp = t1;
            t1 = t2, t2 = temp;
        }

        t_min = std::max(t1, t_min);
        t_max = std::min(t2, t_max);

        /* early out check */
        if (t_max < t_min) return -1.0f;
    }
    return t_min;
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

static GLuint trace(const glm::vec4& ro, const glm::vec3& rd, const Box& box,
                    const glm::mat4& box_model) {
    float dist = ray_to_obb(ro, rd, box, box_model);
    if (dist > 0.0f) {
        return lerp_color(0xFFFFFFFF, 0x000000FF, dist / 1000.0f);
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

    Box box = {};
    box.corners[0] = glm::vec3(-2.5f);
    box.corners[1] = glm::vec3(2.5f);

    for (int y = 0; y < screen_height; y++) {
        for (int x = 0; x < screen_width; x++) {
            glm::vec4 ray_end_ndc(((float)x / (float)screen_width - 0.5f) * 2.0f,
                                  ((float)y / (float)screen_height - 0.5f) * 2.0f, 0.0, 1.0f);
            glm::vec4 ray_end_world = ndc_to_world * ray_end_ndc;
            ray_end_world /= ray_end_world.w;

            /* NOTE: this normalize is very expensive! */
            glm::vec3 ray_dir = glm::normalize(ray_end_world - cam_pos_4);

            buffer[x + y * screen_width] = trace(cam_pos_4, ray_dir, box, box_model);
        }
    }
}

GLuint Renderer::get_buf() {
    glBindTexture(GL_TEXTURE_2D, buffer_handle);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, screen_width, screen_height, GL_RGBA,
                    GL_UNSIGNED_INT_8_8_8_8, buffer);
    glBindTexture(GL_TEXTURE_2D, 0);
    return buffer_handle;
}
