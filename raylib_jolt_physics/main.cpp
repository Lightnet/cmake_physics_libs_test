#define WIN32_LEAN_AND_MEAN
#define NOGDICAPMASKS
#define NOVIRTUALKEYCODES
#define NOWINMESSAGES
#define NOWINSTYLES
#define NOSYSMETRICS
#define NOMENUS
#define NOICONS
#define NOKEYSTATES
#define NOSYSCOMMANDS
#define NORASTEROPS
#define NOSHOWWINDOW
#define OEMRESOURCE
#define NOATOM
#define NOCLIPBOARD
#define NOCOLOR
#define NOCTLMGR
#define NODRAWTEXT
#define NOGDI
#define NOKERNEL
#define NOUSER
#define NOMB
#define NOMEMMGR
#define NOMETAFILE
#define NOMINMAX
#define NOMSG
#define NOOPENFILE
#define NOSCROLL
#define NOSERVICE
#define NOSOUND
#define NOTEXTMETRIC
#define NOWH
#define NOWINOFFSETS
#define NOCOMM
#define NOKANJI
#define NOHELP
#define NOPROFILER
#define NODEFERWINDOWPOS
#define NOMCX
#include <windows.h>

namespace rl {
  #include "raylib.h"
}

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <iostream>

using namespace JPH;

// Define physics layers
namespace Layers {
    constexpr ObjectLayer NON_MOVING = 0;
    constexpr ObjectLayer MOVING = 1;
};

// Define broad phase layers
namespace BroadPhaseLayers {
    constexpr BroadPhaseLayer NON_MOVING(0);
    constexpr BroadPhaseLayer MOVING(1);
    constexpr uint NUM_LAYERS(2);
};

// Simple object vs broadphase layer filter implementation
class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter {
public:
    virtual bool ShouldCollide(ObjectLayer inObjectLayer, BroadPhaseLayer inBroadPhaseLayer) const override {
        switch (inObjectLayer) {
            case Layers::NON_MOVING:
                return inBroadPhaseLayer == BroadPhaseLayers::MOVING;
            case Layers::MOVING:
                return true;
            default:
                return false;
        }
    }
};

int main() {
    std::cout << "Starting program...\n";

    // Initialize Jolt Physics
    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();
    std::cout << "Jolt initialized.\n";

    // Initialize raylib
    const int screenWidth = 800;
    const int screenHeight = 600;
    rl::InitWindow(screenWidth, screenHeight, "Cube Down Test");
    if (!rl::IsWindowReady()) {
        std::cerr << "Failed to initialize window!\n";
        return -1;
    }
    std::cout << "Window initialized.\n";
    rl::SetTargetFPS(60);
    rl::SetExitKey(0); // Disable ESC key; close with X button

    rl::Camera3D camera = { 0 };
    camera.position = { 0.0f, 10.0f, 10.0f };
    camera.target = { 0.0f, 0.0f, 0.0f };
    camera.up = { 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = rl::CAMERA_PERSPECTIVE;
    std::cout << "Camera initialized.\n";

    // Jolt systems
    TempAllocatorImpl temp_allocator(10 * 1024 * 1024);
    JobSystemThreadPool job_system(cMaxPhysicsJobs, cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);
    std::cout << "Jolt job system initialized.\n";

    BroadPhaseLayerInterfaceTable broad_phase_layer_interface(2, BroadPhaseLayers::NUM_LAYERS);
    broad_phase_layer_interface.MapObjectToBroadPhaseLayer(Layers::NON_MOVING, BroadPhaseLayers::NON_MOVING);
    broad_phase_layer_interface.MapObjectToBroadPhaseLayer(Layers::MOVING, BroadPhaseLayers::MOVING);

    ObjectLayerPairFilterTable object_layer_filter(2);
    object_layer_filter.EnableCollision(Layers::NON_MOVING, Layers::MOVING);
    object_layer_filter.EnableCollision(Layers::MOVING, Layers::MOVING);
    object_layer_filter.DisableCollision(Layers::NON_MOVING, Layers::NON_MOVING);

    ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_filter;

    PhysicsSystem physics;
    physics.Init(
        1024,    // max bodies
        1024,    // num body mutexes
        65536,   // max body pairs
        1024,    // max contact constraints
        broad_phase_layer_interface,
        object_vs_broadphase_filter,
        object_layer_filter
    );
    std::cout << "Physics system initialized.\n";

    // Create floor
    BodyCreationSettings floor_settings(
        new BoxShape(Vec3(100.0f, 1.0f, 100.0f)),
        Vec3(0.0f, -1.0f, 0.0f),
        Quat::sIdentity(),
        EMotionType::Static,
        Layers::NON_MOVING
    );
    BodyInterface& body_interface = physics.GetBodyInterface();
    BodyID floor_id = body_interface.CreateAndAddBody(floor_settings, EActivation::DontActivate);
    if (!floor_id.IsInvalid()) {
        std::cout << "Floor created with ID: " << floor_id.GetIndexAndSequenceNumber() << "\n";
    } else {
        std::cerr << "Failed to create floor!\n";
    }

    // Create cube
    BodyCreationSettings cube_settings(
        new BoxShape(Vec3(0.5f, 0.5f, 0.5f)),
        Vec3(0.0f, 10.0f, 0.0f),
        Quat::sIdentity(),
        EMotionType::Dynamic,
        Layers::MOVING
    );
    cube_settings.mLinearDamping = 0.1f;
    cube_settings.mAngularDamping = 0.1f;
    BodyID cube_id = body_interface.CreateAndAddBody(cube_settings, EActivation::Activate);
    if (!cube_id.IsInvalid()) {
        std::cout << "Cube created with ID: " << cube_id.GetIndexAndSequenceNumber() << "\n";
    } else {
        std::cerr << "Failed to create cube!\n";
    }

    int frame_count = 0;
    while (!rl::WindowShouldClose()) {
        // Update physics
        physics.Update(
            1.0f / 60.0f,    // delta time
            1,               // collision steps
            &temp_allocator, // temp allocator
            &job_system      // job system
        );

        Vec3 cube_pos = body_interface.GetCenterOfMassPosition(cube_id);
        std::cout << "Frame " << frame_count << ": Cube at (" 
                  << cube_pos.GetX() << ", " << cube_pos.GetY() << ", " << cube_pos.GetZ() << ")\n";

        // Render
        rl::BeginDrawing();
        rl::ClearBackground(rl::RAYWHITE);

        rl::BeginMode3D(camera);
        rl::DrawCube({0.0f, -1.0f, 0.0f}, 200.0f, 2.0f, 200.0f, rl::GRAY); // Floor
        rl::DrawCube({cube_pos.GetX(), cube_pos.GetY(), cube_pos.GetZ()}, 
                     1.0f, 1.0f, 1.0f, rl::RED); // Cube
        rl::DrawCubeWires({cube_pos.GetX(), cube_pos.GetY(), cube_pos.GetZ()}, 
                          1.0f, 1.0f, 1.0f, rl::BLACK); // Cube outline
        rl::EndMode3D();

        rl::DrawFPS(10, 10);
        rl::DrawText("Cube Falling Test", 10, 40, 20, rl::BLACK); // Add visible text
        rl::EndDrawing();

        frame_count++;
    }

    std::cout << "Cleaning up...\n";
    body_interface.RemoveBody(cube_id);
    body_interface.DestroyBody(cube_id);
    body_interface.RemoveBody(floor_id);
    body_interface.DestroyBody(floor_id);

    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
    JPH::UnregisterTypes();

    rl::CloseWindow();
    std::cout << "Program ended. Press enter to exit...\n";
    std::cin.get();

    return 0;
}