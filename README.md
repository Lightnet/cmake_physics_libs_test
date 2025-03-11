
# License: MIT

# Required:
 * CMake
 * VS2022
 
# Information:

  Testing libs physics code.
  
  Note testing 3D world physics.
  
  Using the raylib to render test to check physics position and rotate. Note have not test rotate few projects yet due to limited of AI model cool down.
  
# Projects:
 * raylib_bullet3_physics_cpp (working)
 * raylib_jolt_physics (working)
 * raylib_joltc_physics (N/A)
 * raylib_ode_physics (working)
  
# raylib c/c++:
  Note if you using the VS2022 there will be conflict windows.h with raylib.h as well raymath.h

  c raylib is useable.
  
  c++ not useable. Due to namespace and conflict of the windows.h.
  
  It depend on the physics lib config there will be conflicts.
  
### c++:
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
```
It can be found in github, reddit and other posts.