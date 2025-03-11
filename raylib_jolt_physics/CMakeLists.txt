cmake_minimum_required(VERSION 3.20)
project(CubeDownTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ensure dynamic runtime for Debug and Release builds
if(MSVC)
    # Replace /MT with /MD in compiler flags
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MDd")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MD")
endif()

include(FetchContent)

# Fetch and configure Jolt Physics
FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics
    GIT_TAG v5.2.0
    SOURCE_SUBDIR Build
)
set(BUILD_EXAMPLES OFF CACHE BOOL "Build JoltPhysics examples" FORCE)
set(BUILD_UNIT_TESTS OFF CACHE BOOL "Build JoltPhysics unit tests" FORCE)
set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "Use static MSVC runtime" FORCE) # Ensure dynamic runtime
FetchContent_MakeAvailable(JoltPhysics)

# Fetch and configure raylib
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib
    GIT_TAG 5.5
)
set(BUILD_EXAMPLES OFF CACHE BOOL "Build raylib examples" FORCE)
FetchContent_MakeAvailable(raylib)

# Add your executable
add_executable(${PROJECT_NAME} main.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE 
    Jolt
    raylib
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${joltphysics_SOURCE_DIR}
    ${joltphysics_SOURCE_DIR}/Jolt
    ${raylib_SOURCE_DIR}/src
)

# Define WIN32_LEAN_AND_MEAN for all targets
add_definitions(-DWIN32_LEAN_AND_MEAN)