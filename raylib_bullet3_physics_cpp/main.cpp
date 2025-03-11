#include "raylib.h"
#include "raymath.h"
#include <btBulletDynamicsCommon.h>
#include <cstdlib>
#include <ctime>
#include <stdio.h>

float randomFloat(float range) {
    return ((float)rand() / RAND_MAX) * 2 * range - range;
}

int main() {
    srand((unsigned int)time(nullptr));

    InitWindow(800, 600, "Drop Cube Test - Bullet3 & raylib");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = { 0.0f, 10.0f, 10.0f }; // Initial position
    camera.target = { 0.0f, 0.0f, 0.0f };     // Initial target
    camera.up = { 0.0f, 1.0f, 0.0f };         // Initial up vector
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btDefaultCollisionConfiguration* collisionConfig = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfig);
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
    dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));

    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundRbInfo(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody* groundRb = new btRigidBody(groundRbInfo);
    dynamicsWorld->addRigidBody(groundRb);

    btCollisionShape* cubeShape = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f));
    btDefaultMotionState* cubeMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 5, 0)));
    btScalar mass = 1.0f;
    btVector3 cubeInertia(0, 0, 0);
    cubeShape->calculateLocalInertia(mass, cubeInertia);
    btRigidBody::btRigidBodyConstructionInfo cubeRbInfo(mass, cubeMotionState, cubeShape, cubeInertia);
    btRigidBody* cubeRb = new btRigidBody(cubeRbInfo);
    dynamicsWorld->addRigidBody(cubeRb);

    Model cubeModel = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));

    char debugText[256];
    bool mouseCaptured = false; // Track mouse capture state

    // Initially capture the mouse
    DisableCursor();
    mouseCaptured = true;

    while (!WindowShouldClose()) {
        dynamicsWorld->stepSimulation(1.0f / 60.0f, 10);

        // Reset cube with 'R'
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

        // Reset camera with '1'
        if (IsKeyPressed(KEY_ONE)) {
            camera.position = { 0.0f, 10.0f, 10.0f };
            camera.target = { 0.0f, 0.0f, 0.0f };
            camera.up = { 0.0f, 1.0f, 0.0f };
        }

        // Toggle mouse capture with Escape
        if (IsKeyPressed(KEY_ESCAPE)) {
            if (mouseCaptured) {
                EnableCursor();
                mouseCaptured = false;
            } else {
                DisableCursor();
                mouseCaptured = true;
            }
        }

        btTransform cubeTransform;
        cubeRb->getMotionState()->getWorldTransform(cubeTransform);
        Vector3 cubePos = {
            (float)cubeTransform.getOrigin().getX(),
            (float)cubeTransform.getOrigin().getY(),
            (float)cubeTransform.getOrigin().getZ()
        };
        btQuaternion cubeRot = cubeTransform.getRotation();
        Quaternion raylibRot = { cubeRot.x(), cubeRot.y(), cubeRot.z(), cubeRot.w() };
        Matrix rotMatrix = QuaternionToMatrix(raylibRot);
        Matrix transMatrix = MatrixTranslate(cubePos.x, cubePos.y, cubePos.z);
        cubeModel.transform = MatrixMultiply(rotMatrix, transMatrix);

        // Update camera with free mode
        UpdateCamera(&camera, CAMERA_FREE);

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
        DrawPlane({0, 0, 0}, {10, 10}, GRAY);
        DrawModel(cubeModel, {0, 0, 0}, 1.0f, BLUE);
        DrawModelWires(cubeModel, {0, 0, 0}, 1.0f, BLACK);
        DrawGrid(10, 1.0f);
        EndMode3D();

        // Draw debug info
        sprintf(debugText, "Pos: (%.2f, %.2f, %.2f)", cubePos.x, cubePos.y, cubePos.z);
        DrawText(debugText, 10, 30, 20, DARKGRAY);
        sprintf(debugText, "Rot: (%.2f, %.2f, %.2f, %.2f)", cubeRot.x(), cubeRot.y(), cubeRot.z(), cubeRot.w());
        DrawText(debugText, 10, 50, 20, DARKGRAY);

        DrawFPS(10, 10);
        DrawText("WASD: Move, Mouse: Look, Q/E: Up/Down", 10, 70, 10, DARKGRAY);
        DrawText("R: Reset Cube, 1: Reset Camera, Esc: Toggle Mouse", 10, 90, 10, DARKGRAY);

        EndDrawing();
    }

    UnloadModel(cubeModel);
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

    CloseWindow();
    return 0;
}