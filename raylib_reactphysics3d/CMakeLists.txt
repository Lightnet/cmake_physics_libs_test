cmake_minimum_required(VERSION 3.14)
project(drop_cube LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

file(DOWNLOAD
    "https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.7/CPM.cmake"
    "${CMAKE_BINARY_DIR}/CPM_0.40.7.cmake"
)
include(${CMAKE_BINARY_DIR}/CPM_0.40.7.cmake)

CPMAddPackage(
    NAME raylib
    GITHUB_REPOSITORY raysan5/raylib
    GIT_TAG 5.5
    OPTIONS
        "BUILD_EXAMPLES OFF"
        "BUILD_GAMES OFF"
        "PLATFORM_DESKTOP ON"
        "USE_EXTERNAL_GLFW OFF"
)

CPMAddPackage(
    NAME reactphysics3d
    GITHUB_REPOSITORY DanielChappuis/reactphysics3d
    GIT_TAG v0.10.2
    OPTIONS
        "RP3D_COMPILE_TESTBED OFF"
        "RP3D_COMPILE_TESTS OFF"
        "RP3D_DOUBLE_PRECISION_ENABLED OFF"
)

add_executable(drop_cube src/main.cpp)

target_link_libraries(drop_cube PRIVATE
    raylib
    reactphysics3d
)

target_include_directories(drop_cube PRIVATE
    ${raylib_SOURCE_DIR}/src
    ${reactphysics3d_SOURCE_DIR}/include
)

if(WIN32)
    target_link_libraries(drop_cube PRIVATE
        winmm
        gdi32
        opengl32
    )
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /Ob0 /Od")
endif()

# Explicitly disable debug renderer
target_compile_definitions(drop_cube PRIVATE
    RP3D_DISABLE_DEBUG_RENDERER  # Custom define to exclude debug renderer
)

set(CMAKE_GENERATOR "Visual Studio 17 2022" CACHE STRING "CMake generator" FORCE)