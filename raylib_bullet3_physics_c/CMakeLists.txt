cmake_minimum_required(VERSION 3.14) # FetchContent requires 3.14+
project(DropCubeTest)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Force consistent runtime library settings for Windows
if (MSVC)
    # Use MDd for Debug and MD for Release
    set(CMAKE_CXX_FLAGS_DEBUG "/MDd /Zi /Ob0 /Od /RTC1" CACHE STRING "Debug flags" FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE "/MD /O2 /Ob2 /DNDEBUG" CACHE STRING "Release flags" FORCE)
    # Ensure these flags propagate to dependencies
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL" CACHE STRING "MSVC runtime library" FORCE)
endif()

# Include FetchContent module
include(FetchContent)

# Fetch Bullet3 from GitHub and disable examples/demos
FetchContent_Declare(
    bullet3
    GIT_REPOSITORY https://github.com/bulletphysics/bullet3.git
    GIT_TAG        3.25 # Use 'master' or a specific tag like '3.25'
)
set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "Disable Bullet2 demos" FORCE)
set(BUILD_BULLET3 OFF CACHE BOOL "Disable Bullet3 examples" FORCE)
set(BUILD_EXTRAS OFF CACHE BOOL "Disable extra Bullet projects" FORCE)
set(BUILD_UNIT_TESTS OFF CACHE BOOL "Disable Bullet unit tests" FORCE)
# Ensure Bullet3 uses the same runtime library
if (MSVC)
    set(USE_MSVC_RUNTIME_LIBRARY_DLL ON CACHE BOOL "Use dynamic runtime for Bullet3" FORCE)
endif()
FetchContent_MakeAvailable(bullet3)

# Fetch raylib 5.5 from GitHub and disable examples
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.5 # Specific tag for raylib 5.5
)
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable raylib examples" FORCE)
# raylib typically uses MDd/MD by default, no extra runtime tweak needed
FetchContent_MakeAvailable(raylib)

# Add executable
add_executable(${PROJECT_NAME} main.cpp)

# Include directories for Bullet3
target_include_directories(${PROJECT_NAME} PRIVATE
    ${bullet3_SOURCE_DIR}/src
)

# Include directories for raylib
target_include_directories(${PROJECT_NAME} PRIVATE
    ${raylib_SOURCE_DIR}/src
)

# Link libraries (Bullet3)
target_link_libraries(${PROJECT_NAME} PRIVATE
    BulletDynamics
    BulletCollision
    LinearMath
)

# Link raylib and platform-specific libraries
target_link_libraries(${PROJECT_NAME} PRIVATE raylib)
if (WIN32)
    # Windows-specific libraries for raylib
    target_link_libraries(${PROJECT_NAME} PRIVATE opengl32 gdi32 winmm)
endif()

# Ensure Bullet3 libraries are built
add_dependencies(${PROJECT_NAME} BulletDynamics BulletCollision LinearMath)

# Set output directory for Windows (optional)
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)