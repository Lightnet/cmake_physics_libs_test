#include "raylib.h"
#include <btBulletDynamicsCommon.h>
#include <cstdlib> // For rand()
#include <ctime>   // For seeding rand()

// Function to generate a random float between -range and +range
float randomFloat(float range) {
    return ((float)rand() / RAND_MAX) * 2 * range - range;
}

int main() {
    // Seed random number generator
    srand((unsigned int)time(nullptr));

    // Initialize raylib
    const int screenWidth = 800;
    const int screenHeight = 600;
    InitWindow(screenWidth, screenHeight, "Drop Cube Test - Bullet3 & raylib");
    SetTargetFPS(60);

    // Set up camera
    Camera3D camera = { 0 };
    camera.position = { 0.0f, 10.0f, 10.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // Initialize Bullet3 physics
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btDefaultCollisionConfiguration* collisionConfig = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfig);
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
    dynamicsWorld->setGravity(btVector3(0, -9.81f, 0)); // Gravity in m/s^2

    // Ground (static plane)
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundRbInfo(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody* groundRb = new btRigidBody(groundRbInfo);
    dynamicsWorld->addRigidBody(groundRb);

    // Cube (dynamic body)
    btCollisionShape* cubeShape = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));
    btDefaultMotionState* cubeMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 5, 0)));
    btScalar mass = 1.0f;
    btVector3 cubeInertia(0, 0, 0);
    cubeShape->calculateLocalInertia(mass, cubeInertia);
    btRigidBody::btRigidBodyConstructionInfo cubeRbInfo(mass, cubeMotionState, cubeShape, cubeInertia);
    btRigidBody* cubeRb = new btRigidBody(cubeRbInfo);
    dynamicsWorld->addRigidBody(cubeRb);

    // Main loop
    while (!WindowShouldClose()) {
        // Step the physics simulation
        dynamicsWorld->stepSimulation(1.0f / 60.0f, 10);

        // Reset cube position and apply random rotation on 'R' key press
        if (IsKeyPressed(KEY_R)) {
            btTransform resetTransform;
            resetTransform.setIdentity();
            resetTransform.setOrigin(btVector3(0, 5, 0));

            float angleX = randomFloat(3.14159f);
            float angleY = randomFloat(3.14159f);
            float angleZ = randomFloat(3.14159f);
            btQuaternion randomRotation;
            randomRotation.setEulerZYX(angleZ, angleY, angleX);
            resetTransform.setRotation(randomRotation);

            cubeRb->setWorldTransform(resetTransform);
            cubeRb->setLinearVelocity(btVector3(0, 0, 0));
            cubeRb->setAngularVelocity(btVector3(0, 0, 0));
            cubeRb->activate(true);
        }

        // Get cube position
        btTransform cubeTransform;
        cubeRb->getMotionState()->getWorldTransform(cubeTransform);
        Vector3 cubePos = {
            (float)cubeTransform.getOrigin().getX(),
            (float)cubeTransform.getOrigin().getY(),
            (float)cubeTransform.getOrigin().getZ()
        };

        // Update camera
        UpdateCamera(&camera, CAMERA_ORBITAL);

        // Render
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
        // Draw ground
        DrawPlane({0, 0, 0}, {10, 10}, GRAY);
        // Draw cube (no rotation support in DrawCubeV, using basic DrawCube)
        DrawCube(cubePos, 1.0f, 1.0f, 1.0f, BLUE);
        DrawCubeWires(cubePos, 1.0f, 1.0f, 1.0f, BLACK);
        DrawGrid(10, 1.0f);
        EndMode3D();

        DrawFPS(10, 10);
        EndDrawing();
    }

    // Cleanup Bullet3
    dynamicsWorld->removeRigidBody(cubeRb);
    dynamicsWorld->removeRigidBody(groundRb);
    delete cubeRb;
    delete cubeShape;
    delete cubeMotionState;
    delete groundRb;
    delete groundShape;
    delete groundMotionState;
    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfig;
    delete broadphase;

    // Cleanup raylib
    CloseWindow();

    return 0;
}