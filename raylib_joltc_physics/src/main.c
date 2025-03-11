#include <stdio.h>
#include "raylib.h"
#include "joltc.h"

int main() {
    // Initialize Jolt Physics
    if (!JPH_Init()) {
        printf("Failed to initialize Jolt Physics\n");
        return 1;
    }
    printf("Jolt Physics initialized\n");

    // Initialize raylib window
    const int screenWidth = 800;
    const int screenHeight = 600;
    InitWindow(screenWidth, screenHeight, "Cube Drop Test");
    SetTargetFPS(60);

    // Set up camera
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 10.0f, 10.0f, 10.0f }; // Camera position
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };     // Look at origin
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };         // Up vector
    camera.fovy = 45.0f;                                // Field of view
    camera.projection = CAMERA_PERSPECTIVE;

    // Simple cube position (falling)
    Vector3 cubePosition = { 0.0f, 10.0f, 0.0f }; // Start above ground
    float cubeVelocity = 0.0f;                    // Vertical velocity
    const float gravity = -9.81f;                 // Gravity acceleration
    const float cubeSize = 2.0f;                  // Cube size
    const float groundY = 0.0f;                   // Ground level

    // Main game loop
    while (!WindowShouldClose()) {
        // Update physics (simple gravity simulation)
        float deltaTime = GetFrameTime();
        cubeVelocity += gravity * deltaTime;           // Apply gravity
        cubePosition.y += cubeVelocity * deltaTime;    // Update position

        // Collision with ground
        if (cubePosition.y - cubeSize / 2 < groundY) {
            cubePosition.y = groundY + cubeSize / 2;  // Stop at ground
            cubeVelocity = 0.0f;                      // Reset velocity
        }

        // Begin drawing
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        // Draw cube
        DrawCube(cubePosition, cubeSize, cubeSize, cubeSize, BLUE);

        // Draw ground plane
        DrawPlane((Vector3){ 0.0f, 0.0f, 0.0f }, (Vector2){ 20.0f, 20.0f }, GRAY);

        // Draw grid for reference
        DrawGrid(10, 1.0f);

        EndMode3D();

        // Draw FPS
        DrawFPS(10, 10);

        EndDrawing();
    }

    // Cleanup
    JPH_Shutdown();
    CloseWindow();

    return 0;
}