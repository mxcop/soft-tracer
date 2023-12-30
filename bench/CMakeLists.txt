# BENCHMARK
# https://github.com/google/benchmark
set(BENCHMARK_DOWNLOAD_DEPENDENCIES ON CACHE BOOL "" FORCE)
add_subdirectory("../extern/benchmark" "benchmark")

# GLM
# https://github.com/g-truc/glm
# add_subdirectory("../extern/glm" "glm")

# AABB Benchmark
add_executable (bench_aabb "aabb.cpp" "aabb.h")
target_link_libraries(bench_aabb glm)
target_link_libraries(bench_aabb benchmark::benchmark)
