#include <stdio.h>
#include <chrono>

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <glad/glad.h>
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_win32.h>
#include <windows.h>

#include "opengl/screen-quad.h"
#include "opengl/shader.h"
#include "graphics/renderer.h"

constexpr int WINDOW_WIDTH = 1280;
constexpr int WINDOW_HEIGHT = 720;
// constexpr int WINDOW_WIDTH = 1920;
// constexpr int WINDOW_HEIGHT = 1080;

void imgui_init(SDL_Window* window, SDL_GLContext context);

static bool ray_to_aabb(const glm::vec3& ro, const glm::vec3& rd, const glm::vec3& bbmin,
                        const glm::vec3& bbmax) {
    float tmin = 0.0, tmax = INFINITY;

    const glm::vec3 corners[2]{bbmin, bbmax};
    glm::vec3 inv_dir = 1.0f / rd;

    for (int d = 0; d < 3; ++d) {
        bool sign = signbit(inv_dir[d]);
        float bmin = corners[sign][d];
        float bmax = corners[!sign][d];

        float dmin = (bmin - ro[d]) * inv_dir[d];
        float dmax = (bmax - ro[d]) * inv_dir[d];

        tmin = max(dmin, tmin);
        tmax = min(dmax, tmax);
        /* Early out check, saves a lot of compute */
        if (tmax < tmin) return false;
    }

    return tmin < tmax;
}

int main(int argc, char* argv[]) {
    /* Set process to high prio */
    // SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);

    SetProcessDPIAware(); /* <- handle high DPI screens on Windows */
    ImGui_ImplWin32_EnableDpiAwareness();

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        printf("error: sdl failed to init.\n%s\n", SDL_GetError());
        return -1;
    }

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    SDL_Window* window = SDL_CreateWindow("soft-tracer", WINDOW_WIDTH, WINDOW_HEIGHT,
                                          SDL_WINDOW_OPENGL | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "nearest");

    SDL_GLContext context = SDL_GL_CreateContext(window);

    int version = gladLoadGL();

    imgui_init(window, context);

    SDL_SetRelativeMouseMode(SDL_TRUE);
    SDL_CaptureMouse(SDL_TRUE);

    Shader buffer_shader("public/shaders/buffer.vert", "public/shaders/buffer.frag");
    ScreenQuad screen = ScreenQuad();

    Renderer* renderer = new Renderer(WINDOW_WIDTH, WINDOW_HEIGHT);

    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glm::vec3 cam_pos = glm::vec3(-110.0f, 327.0f, -144.0f); // glm::vec3(32.72f, 94.15f, 110.79f);
    float pitch = -35.0f, yaw = 41.6f; // float pitch = -33.67f, yaw = -59.1f;
    bool w = false, a = false, s = false, d = false, shift = false, space = false;
    bool running = true;
    auto prev_time = std::chrono::high_resolution_clock::now();
    float time = 0.0f;
    while (running) {
        auto curr_time = std::chrono::high_resolution_clock::now();
        float dt = (curr_time - prev_time).count() * 1e-9;
        time += dt;
        prev_time = curr_time;

        /* SDL Input handling */
        SDL_Event event;
        float mdx = 0.0f, mdy = 0.0f;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            switch (event.type) {
                case SDL_EVENT_QUIT:
                    running = false;
                    break;
                case SDL_EVENT_KEY_UP: /* Key up */
                    if (event.key.keysym.sym == SDLK_ESCAPE) running = false;
                    if (event.key.keysym.sym == SDLK_w) w = false;
                    if (event.key.keysym.sym == SDLK_a) a = false;
                    if (event.key.keysym.sym == SDLK_s) s = false;
                    if (event.key.keysym.sym == SDLK_d) d = false;
                    if (event.key.keysym.sym == SDLK_LSHIFT) shift = false;
                    if (event.key.keysym.sym == SDLK_SPACE) space = false;
                    break;
                case SDL_EVENT_KEY_DOWN:
                    if (event.key.keysym.sym == SDLK_w) w = true;
                    if (event.key.keysym.sym == SDLK_a) a = true;
                    if (event.key.keysym.sym == SDLK_s) s = true;
                    if (event.key.keysym.sym == SDLK_d) d = true;
                    if (event.key.keysym.sym == SDLK_LSHIFT) shift = true;
                    if (event.key.keysym.sym == SDLK_SPACE) space = true;
                    break;
                case SDL_EVENT_MOUSE_MOTION:
                    mdx += event.motion.xrel;
                    mdy += event.motion.yrel;
                    break;
                default:
                    break;
            }
        }

        pitch = max(min(pitch - mdy * dt * 4.0f, 89.0f), -89.0f);
        yaw += mdx * dt * 4.0f;

        glm::vec3 cam_dir = glm::vec3(0);
        float xzLen = cos(glm::radians(pitch));
        cam_dir.x = xzLen * cos(glm::radians(yaw));
        cam_dir.y = sin(glm::radians(pitch));
        cam_dir.z = xzLen * sin(glm::radians(yaw));
        cam_dir = glm::normalize(cam_dir);

        constexpr float MOVE_SPEED = 24.0f;
        if (w) cam_pos += cam_dir * dt * MOVE_SPEED;
        if (s) cam_pos -= cam_dir * dt * MOVE_SPEED;
        if (d) cam_pos += glm::normalize(glm::cross(cam_dir, glm::vec3(0, 1, 0))) * dt * MOVE_SPEED;
        if (a) cam_pos -= glm::normalize(glm::cross(cam_dir, glm::vec3(0, 1, 0))) * dt * MOVE_SPEED;
        if (shift) cam_pos += glm::vec3(0, -1, 0) * dt * MOVE_SPEED;
        if (space) cam_pos += glm::vec3(0, 1, 0) * dt * MOVE_SPEED;

        glClear(GL_COLOR_BUFFER_BIT);

        /* Start the render frame */
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        auto start_time = std::chrono::high_resolution_clock::now();
        renderer->render(dt, time, cam_pos, cam_dir);
        auto end_time = std::chrono::high_resolution_clock::now();
        double render_time_us = static_cast<double>((end_time - start_time).count()) / 1000.0;
        double render_time_ms = render_time_us / 1000.0;
        double render_time_s = render_time_ms / 1000.0;

        /* Window position */
        constexpr float PADDING = 10.0f;
        ImVec2 work_pos = ImGui::GetMainViewport()->WorkPos;
        ImGui::SetNextWindowPos(ImVec2(work_pos.x + PADDING, work_pos.y + PADDING),
                                ImGuiCond_Always,
                         ImVec2(0.0f, 0.0f));

        constexpr ImGuiWindowFlags overlay_flags =
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
            ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
            ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove;

        /* Draw content */
        ImGui::SetNextWindowBgAlpha(0.35f);
        if (ImGui::Begin("Debug overlay", nullptr, overlay_flags)) {
            ImGui::Text("Debug overlay\n");
            ImGui::Separator();
            ImGui::Text("FPS: %.1f", 1.0f / dt);
            ImGui::Separator();
            ImGui::Text("Ray/s: %.2fM", ((WINDOW_WIDTH * WINDOW_HEIGHT) / render_time_s) / 1'000'000.0);
            ImGui::Text("Ray time (mean): %.2fns",
                        (render_time_us / (WINDOW_WIDTH * WINDOW_HEIGHT)) * 1'000.0);
            ImGui::Text("Ray time (goal): %.2fns",
                        (0.0166666 / (double)(WINDOW_WIDTH * WINDOW_HEIGHT)) * 1.0e+9);
            ImGui::Separator();
            ImGui::Text("BVH build: %.2fms", renderer->db_build_time * 0.001f);
            ImGui::Text("Frame time: %.2fms", render_time_ms);
            ImGui::Separator();
            ImGui::Text("cam pos: %.2f, %.2f, %.2f", cam_pos.x, cam_pos.y, cam_pos.z);
            ImGui::Text("cam angle: %.2f, %.2f", pitch, yaw);
        }
        ImGui::End();

        /* Draw the screen buffer */
        GLuint screen_buf = renderer->get_buf();
        buffer_shader.use();
        buffer_shader.set_int("buffer_sampler", 0);
        glActiveTexture(0);
        glBindTexture(GL_TEXTURE_2D, screen_buf);
        screen.draw();
        glBindTexture(GL_TEXTURE_2D, 0);

        /* Render ImGui */
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        /* Swap SDL buffers */
        SDL_GL_SwapWindow(window);
    }

    delete renderer;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

void imgui_init(SDL_Window* window, SDL_GLContext context) {
    IMGUI_CHECKVERSION();
    ImGuiContext* ctx = ImGui::CreateContext();
    ImGui::SetCurrentContext(ctx);
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls
    ImGui::StyleColorsDark();

    /* Setup platform / renderer backends */
    ImGui_ImplSDL3_InitForOpenGL(window, context);
    ImGui_ImplOpenGL3_Init("#version 130");
}
