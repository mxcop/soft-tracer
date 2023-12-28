cmake_minimum_required(VERSION 3.15)
project(tracer)

# C++ standard version
set(CMAKE_CXX_STANDARD 20 CACHE STRING "" FORCE)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_OPT_ARCH_NATIVE_SUPPORTED)
if (COMPILER_OPT_ARCH_NATIVE_SUPPORTED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
else()
    CHECK_CXX_COMPILER_FLAG("/arch:AVX2" COMPILER_OPT_ARCH_AVX_SUPPORTED)
    if(COMPILER_OPT_ARCH_AVX_SUPPORTED)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX2")
    endif()
    CHECK_CXX_COMPILER_FLAG("/arch:AVX" COMPILER_OPT_ARCH_AVX_SUPPORTED)
    if(COMPILER_OPT_ARCH_AVX_SUPPORTED)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX")
    endif()
endif()

# Program executable target
add_executable(main)
set_target_properties(main PROPERTIES OUTPUT_NAME ${CMAKE_PROJECT_NAME})

# Add DEBUG compiler macro
target_compile_definitions(main PRIVATE "DEBUG=$<CONFIG:Debug>")
# Add NDEBUG compiler macro
target_compile_definitions(main PRIVATE "NDEBUG=$<CONFIG:Release>")
# Add WINDOWS compiler macro
target_compile_definitions(main PRIVATE "WINDOWS=$<STREQUAL:${CMAKE_SYSTEM_NAME},Windows>")

# Post build command (copies the asset files)
add_custom_command(TARGET main PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory 
    ${PROJECT_SOURCE_DIR}/public 
    ${PROJECT_BINARY_DIR}/$<CONFIG>/public || (exit 0) # ignore failed copy.
)

# Includes
target_include_directories(main PRIVATE "./src/")

# Glob all source files
file(GLOB_RECURSE SOURCES
	"src/*.cpp"
)

# Add source files to the program
target_sources(main PRIVATE
	${SOURCES}
)

# Sub-directories
add_subdirectory("extern")
