# SDL
# https://github.com/libsdl-org/SDL
set(SDL_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_STATIC ON CACHE BOOL "" FORCE)
set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
set(SDL_TESTS OFF CACHE BOOL "" FORCE)
set(SDL_DISABLE_INSTALL ON CACHE BOOL "" FORCE)
set(SDL_DISABLE_INSTALL_DOCS ON CACHE BOOL "" FORCE)
set(SDL_INSTALL_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("sdl")
target_link_libraries(main PRIVATE SDL3-static)

# GLAD (OpenGL 4.0)
# https://glad.dav1d.de
add_subdirectory("glad")
target_link_libraries(main PRIVATE glad)

# GLM
# https://github.com/g-truc/glm
add_subdirectory("glm")
target_link_libraries(main PRIVATE glm)

# IMGUI
# https://github.com/ocornut/imgui
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
add_library(imgui STATIC)

target_sources(imgui
PRIVATE
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/imgui.cpp
PRIVATE
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
    ${IMGUI_DIR}/backends/imgui_impl_sdl3.cpp
    ${IMGUI_DIR}/backends/imgui_impl_win32.cpp
)

target_include_directories(imgui
    PUBLIC ${IMGUI_DIR}
    PUBLIC ${IMGUI_DIR}/backends
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/sdl/include
)
target_link_libraries(main PRIVATE imgui)

# BENCHMARK
# https://github.com/google/benchmark
# set(BENCHMARK_DOWNLOAD_DEPENDENCIES ON CACHE BOOL "" FORCE)
# add_subdirectory("../extern/benchmark")
