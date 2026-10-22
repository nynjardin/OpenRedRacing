#pragma once
#include "comfort_camera.h"

namespace bigred {
struct SimTerrain;
// Read-only world queries, in game axes and whole world units (+Y down).
bool camera_under_roof(const SimTerrain& terrain, CameraPoint eye);
double camera_arm_fraction(const SimTerrain& terrain, CameraPoint focus, CameraPoint eye);
} // namespace bigred
