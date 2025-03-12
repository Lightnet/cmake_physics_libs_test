// Standard library includes first
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <random>

#define WIN32_LEAN_AND_MEAN
#define NOCOLOR
#define NOGDI
#define NOUSER

#include <reactphysics3d/reactphysics3d.h>

namespace rl {
    #include "raylib.h"
    #include "raymath.h"
}

using namespace reactphysics3d;

// Helper function to compute Euler angles from a quaternion (in radians)
Vector3 getEulerAngles(const Quaternion& q) {
    float x = q.x, y = q.y, z = q.z, w = q.w;

    // Pitch (x-axis rotation)
    float sinPitch = 2.0f * (w * x + y * z);
    float cosPitch = 1.0f - 2.0f * (x * x + y * y);
    float pitch = std::atan2(sinPitch, cosPitch);

    // Yaw (y-axis rotation)
    float sinYaw = 2.0f * (w * y - z * x);
    float yaw = std::asin(sinYaw);
    if (std::abs(sinYaw) >= 1.0f) {
        yaw = std::copysign(PI / 2.0f, sinYaw); // Handle singularity
    }

    // Roll (z-axis rotation)
    float sinRoll = 2.0f * (w * z + x * y);
    float cosRoll = 1.0f - 2.0f * (y * y + z * z);
    float roll = std::atan2(sinRoll, cosRoll);

    return Vector3(pitch, yaw, roll);
}

// Function to reset cube position and apply random rotation
void resetCube(RigidBody* cubeBody, Vector3 initialPos) {
    Transform resetTransform(initialPos, Quaternion::identity());
    cubeBody->setTransform(resetTransform);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(0.0f, 2.0f * PI);
    float yaw = dist(gen);
    float pitch = dist(gen);
    float roll = dist(gen);
    Quaternion randomRot = Quaternion::fromEulerAngles(pitch, yaw, roll);
    Transform currentTransform = cubeBody->getTransform();
    currentTransform.setOrientation(randomRot);
    cubeBody->setTransform(currentTransform);

    cubeBody->setLinearVelocity(Vector3(0.0f, 0.0f, 0.0f));
    cubeBody->setAngularVelocity(Vector3(0.0f, 0.0f, 0.0f));
}

int main() {
    // Initialize Raylib window
    const int screenWidth = 800;
    const int screenHeight = 600;
    rl::InitWindow(screenWidth, screenHeight, "Cube Drop Test");
    rl::SetTargetFPS(60);

    // Initialize ReactPhysics3D
    PhysicsCommon physicsCommon;
    PhysicsWorld* world = physicsCommon.createPhysicsWorld();

    // Create ground (static body)
    Vector3 groundPos(0.0f, -2.0f, 0.0f);
    Quaternion groundOrientation = Quaternion::identity();
    Transform groundTransform(groundPos, groundOrientation);
    RigidBody* groundBody = world->createRigidBody(groundTransform);
    groundBody->setType(BodyType::STATIC);
    BoxShape* groundShape = physicsCommon.createBoxShape(Vector3(10.0f, 0.5f, 10.0f));
    groundBody->addCollider(groundShape, Transform::identity());

    // Create cube (dynamic body)
    Vector3 cubeInitialPos(0.0f, 5.0f, 0.0f);
    Transform cubeTransform(cubeInitialPos, Quaternion::identity());
    RigidBody* cubeBody = world->createRigidBody(cubeTransform);
    cubeBody->setType(BodyType::DYNAMIC);
    BoxShape* cubeShape = physicsCommon.createBoxShape(Vector3(0.5f, 0.5f, 0.5f));
    cubeBody->addCollider(cubeShape, Transform::identity());
    cubeBody->setMass(1.0f);

    // Generate cube mesh
    rl::Mesh cubeMesh = rl::GenMeshCube(1.0f, 1.0f, 1.0f);
    rl::Model cubeModel = rl::LoadModelFromMesh(cubeMesh);

    // Camera setup for Raylib
    rl::Camera3D camera{};
    camera.position = rl::Vector3{ 10.0f, 10.0f, 10.0f };
    camera.target = rl::Vector3{ 0.0f, 0.0f, 0.0f };
    camera.up = rl::Vector3{ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = rl::CAMERA_PERSPECTIVE;

    while (!rl::WindowShouldClose()) {
        // Reset cube on R key press
        if (rl::IsKeyPressed(rl::KEY_R)) {
            resetCube(cubeBody, cubeInitialPos);
        }

        // Update physics
        world->update(1.0f / 60.0f);

        // Get cube transform from physics engine
        Transform cubeTransformUpdated = cubeBody->getTransform();
        Vector3 cubePos = cubeTransformUpdated.getPosition();
        Quaternion cubeRot = cubeTransformUpdated.getOrientation();

        // Convert quaternion to Euler angles for display (in degrees)
        Vector3 eulerAngles = getEulerAngles(cubeRot) * (180.0f / PI);

        // Convert ReactPhysics3D transform to Raylib matrix
        rl::Matrix transform = rl::MatrixIdentity();
        transform = rl::MatrixMultiply(transform, rl::QuaternionToMatrix({ cubeRot.x, cubeRot.y, cubeRot.z, cubeRot.w }));
        transform = rl::MatrixMultiply(transform, rl::MatrixTranslate(cubePos.x, cubePos.y, cubePos.z));
        cubeModel.transform = transform;

        // Begin drawing
        rl::BeginDrawing();
        rl::ClearBackground(rl::RAYWHITE);

        rl::BeginMode3D(camera);
        {
            rl::DrawCubeV(rl::Vector3{ groundPos.x, groundPos.y, groundPos.z },
                          rl::Vector3{ 20.0f, 1.0f, 20.0f }, rl::GRAY);
            rl::DrawModel(cubeModel, rl::Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, rl::RED);
            rl::DrawGrid(10, 1.0f);
        }
        rl::EndMode3D();

        // Draw text information
        int textY = 10;
        const int textSpacing = 20;

        // Position
        std::ostringstream posStream;
        posStream << std::fixed << std::setprecision(2)
                  << "Position: (" << cubePos.x << ", " << cubePos.y << ", " << cubePos.z << ")";
        rl::DrawText(posStream.str().c_str(), 10, textY, 20, rl::BLACK);
        textY += textSpacing;

        // Rotation (Euler angles in degrees)
        std::ostringstream rotStream;
        rotStream << std::fixed << std::setprecision(2)
                  << "Rotation: (Pitch: " << eulerAngles.x << ", Yaw: " << eulerAngles.y << ", Roll: " << eulerAngles.z << ")";
        rl::DrawText(rotStream.str().c_str(), 10, textY, 20, rl::BLACK);
        textY += textSpacing;

        // Input instructions
        rl::DrawText("Press R to reset position and randomize rotation", 10, textY, 20, rl::DARKGRAY);
        textY += textSpacing;

        // Draw FPS
        rl::DrawFPS(screenWidth - 100, 10);

        rl::EndDrawing();
    }

    // Cleanup physics
    world->destroyRigidBody(cubeBody);
    world->destroyRigidBody(groundBody);
    physicsCommon.destroyPhysicsWorld(world);

    // Cleanup Raylib
    rl::UnloadModel(cubeModel);
    rl::CloseWindow();

    return 0;
}