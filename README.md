
# License: MIT

# Required:
 * CMake
 * VS2022
 
# Information:

  Testing libs physics code.
  
  Note testing 3D world physics.
  
  Using the raylib to render test to check physics position and rotate. Note have not test rotate few projects yet due to limited of AI model cool down.
  
# CMake:
  Note that build order is important as follow the script line by line. To be compile by order build.

  
# Projects:
 * raylib_bullet3_physics_cpp (working)
 * raylib_jolt_physics (working)
 * raylib_ode_physics (working)
 * raylib_reactphysics3d (working)
  
## raylib:
  Note if you using the VS2022 there will be conflict windows.h with raylib.h as well raymath.h
  
  It depend on the physics lib config there will be conflicts.

### c:
  c raylib is useable.
  
### c++:

  c++ not useable. Due to namespace and conflict of the windows.h. As well depend on the physics.

```
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

namespace rl {
    #include "raylib.h"
    #include "raymath.h"
}

int main() {
  // Initialize Raylib window
  const int screenWidth = 800;
  const int screenHeight = 600;
  rl::InitWindow(screenWidth, screenHeight, "Cube Drop Test");
  rl::SetTargetFPS(60);

  while (!rl::WindowShouldClose()) {
    rl::BeginDrawing();
    rl::ClearBackground(rl::RAYWHITE);


    // Draw FPS
    rl::DrawFPS(10, 10);
    rl::EndDrawing();

  }

  // Cleanup Raylib
  rl::CloseWindow();

  return 0;
}
```
It can be found in github, reddit and other posts.