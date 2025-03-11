cmake_minimum_required(VERSION 3.12)
project(CubeDropSimulation LANGUAGES C CXX)

# Enable FetchContent to download dependencies
include(FetchContent)

# Fetch ODE 0.16.6
FetchContent_Declare(
    ode
    GIT_REPOSITORY https://bitbucket.org/odedevs/ode.git
    GIT_TAG 0.16.6
)
set(ODE_DOUBLE_PRECISION OFF CACHE BOOL "Use single precision" FORCE)
set(ODE_BUILD_TESTS OFF CACHE BOOL "Disable ODE tests" FORCE)
set(ODE_WITH_DEMOS OFF CACHE BOOL "Disable ODE demos" FORCE)
set(ODE_WITH_GIMPACT OFF CACHE BOOL "Disable GIMPACT" FORCE)
set(ODE_WITH_OPCODE OFF CACHE BOOL "Use simpler collision" FORCE)
FetchContent_MakeAvailable(ode)

# Fetch Raylib 5.5
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
)
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable Raylib examples" FORCE)
FetchContent_MakeAvailable(raylib)

# Define the executable
add_executable(cube_drop main.c)

# Link libraries to the executable
target_link_libraries(cube_drop PRIVATE 
    ${ode_BINARY_DIR}/Debug/ode_singled.lib  # Explicit path to ODE library
    raylib
)

# Include directories for ODE
target_include_directories(cube_drop PRIVATE 
    ${ode_SOURCE_DIR}/include
    ${ode_BINARY_DIR}/include
)

# Set C standard
set_target_properties(cube_drop PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)

# Post-build step to copy DLLs to the output directory
add_custom_command(TARGET cube_drop POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${ode_BINARY_DIR}/Debug/ode_singled.dll"  # Copy ODE DLL
        "$<TARGET_FILE_DIR:cube_drop>"             # Destination: where cube_drop.exe is
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${raylib_BINARY_DIR}/raylib/Debug/raylib.dll"  # Copy Raylib DLL
        "$<TARGET_FILE_DIR:cube_drop>"                  # Destination: where cube_drop.exe is
    COMMENT "Copying ODE and Raylib DLLs to output directory"
)

# Debug info
message(STATUS "ODE Source Dir: ${ode_SOURCE_DIR}")
message(STATUS "ODE Binary Dir: ${ode_BINARY_DIR}")
message(STATUS "Linking against: ${ode_BINARY_DIR}/Debug/ode_singled.lib")
message(STATUS "Raylib Binary Dir: ${raylib_BINARY_DIR}")