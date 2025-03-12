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

//#include <raylib.h>
#include <reactphysics3d/reactphysics3d.h>

namespace rl {
  #include "raylib.h"
  #include "raymath.h"
}


using namespace reactphysics3d;

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
  Vector3 cubePos(0.0f, 5.0f, 0.0f);
  Transform cubeTransform(cubePos, Quaternion::identity());
  RigidBody* cubeBody = world->createRigidBody(cubeTransform);
  cubeBody->setType(BodyType::DYNAMIC);
  BoxShape* cubeShape = physicsCommon.createBoxShape(Vector3(0.5f, 0.5f, 0.5f));
  cubeBody->addCollider(cubeShape, Transform::identity());
  cubeBody->setMass(1.0f);

  // Camera setup for Raylib
  rl::Camera3D camera{};
  camera.position = rl::Vector3{ 10.0f, 10.0f, 10.0f };
  camera.target = rl::Vector3{ 0.0f, 0.0f, 0.0f };
  camera.up = rl::Vector3{ 0.0f, 1.0f, 0.0f };
  camera.fovy = 45.0f;
  camera.projection = rl::CAMERA_PERSPECTIVE;

  while (!rl::WindowShouldClose()) {
    // Update physics
    world->update(1.0f / 60.0f);

    // Get cube position from physics engine
    Transform cubeTransformUpdated = cubeBody->getTransform();
    Vector3 cubePosUpdated = cubeTransformUpdated.getPosition();

    // Begin drawing
    rl::BeginDrawing();
    rl::ClearBackground(rl::RAYWHITE);

    BeginMode3D(camera);
        {
            // Draw ground
            rl::DrawCubeV(rl::Vector3{ groundPos.x, groundPos.y, groundPos.z },
              rl::Vector3{ 20.0f, 1.0f, 20.0f }, rl::GRAY);

            // Draw cube at updated physics position
            rl::DrawCubeV(rl::Vector3{ cubePosUpdated.x, cubePosUpdated.y, cubePosUpdated.z },
              rl::Vector3{ 1.0f, 1.0f, 1.0f }, rl::RED);

            // Draw grid for reference
            rl::DrawGrid(10, 1.0f);
        }
        rl::EndMode3D();


    // Draw FPS
    rl::DrawFPS(10, 10);
    rl::EndDrawing();

  }

  // Cleanup physics
  world->destroyRigidBody(cubeBody);
  world->destroyRigidBody(groundBody);
  physicsCommon.destroyPhysicsWorld(world);

  // Cleanup Raylib
  rl::CloseWindow();

    return 0;
}