// The game's chase camera - a port of the original's camera mode 4.
//
//
// Three things about it decide everything else:
//
//   * it runs on the SAME 28 ms tick as the physics, never on the rendered frame. The original
//     gates its update on the very same parity
//     counter that halves the 14 ms timer for the handling. Display interpolation is separate
//     and never feeds back into this controller;
//   * the "camera lag" is not a smoothing constant, and there is nothing to tune. The camera
//     only ever moves ALONG the line to the vehicle, and only when that line is too long. It
//     never slides sideways. So when the car turns, the camera stays put and the car swings out
//     to the side of the frame, and it is the car DRIVING AWAY that drags the camera in behind
//     it. A rope, not a spring;
//   * the controller that does swing the camera round behind the car (ChaseCamera::orbit) is an
//     ACQUISITION step, not the steady state. In the original nothing arms it during ordinary
//     driving - only the viewpoint key does - and running it every tick would cancel the lag.
//
// The state is in the GAME's frame, like VehicleSim: +Y points down, world X is not mirrored,
// positions are in whole world units. camera_view() converts on the way out.
#pragma once

#include "../sim/vehicle.h"
#include "camera_sample.h"

#include <cstdint>

namespace bigred {

// One of the three viewpoints the player cycles through, verbatim from the original's table
// (a second table there holds the same three rows).
//
// ⚠️ The three distances are in QUARTER world units: every comparison the original makes
// against them is against `4 * distance`, because its distance routine returns its length
// shifted left by 2. Divided out, preset 0 is 375 units behind at a standstill, 1375 flat out, never under
// 500 - considerably further back than the framing this replaced.
struct CameraPreset {
    int32_t eye_height;   // used by the transition modes, not by mode 4
    int32_t aim_height;   // how far ABOVE the vehicle the camera aims, world units
    int32_t dist_base;    // quarter units
    int32_t dist_factor;  // quarter units added in proportion to speed
    int32_t dist_min;     // quarter units, a floor on the target AND the nudge trigger
};
extern const CameraPreset kCameraPresets[3];

// The aim height inside a tunnel. It replaces the preset's: under a bridge deck
// the camera has to look at the car, not at the soffit.
constexpr int32_t kTunnelAimHeight = 90;

// The acquisition counter, as the viewpoint key arms it. Bit 7 means "not lined up yet"; the low seven are a counter
// the tick walks down by 4, so an acquisition lasts at least 25 ticks (700 ms).
constexpr uint8_t kAcquireArmed = 0xe4;

// What the renderer needs, and all it needs. Interpolating between two of these is what makes
// a 35.7 Hz camera look right at any refresh rate.
struct CameraState {
    int32_t x = 0, y = 0, z = 0;  // game frame, world units, +Y down
    int32_t yaw = 0, pitch = 0;   // BAM16, the bearing to the vehicle
};

class ChaseCamera {
public:
    CameraState now;
    int preset = 0;       // 0..2
    bool settled = false; // the aim point is within 10 units on all three axes

    // Plants the camera 2500 units behind the vehicle, pitched down a notch. The
    // distance controller then walks it in over about a second and a half - that opening move
    // is what the closing-rate limit exists for.
    void reset(const VehicleSim& car, const SimTerrain& terrain);

    // One 28 ms tick - the original's mode-4 update and its tail. Call it exactly where VehicleSim::step is
    // called, never once per frame.
    void step(const VehicleSim& car, const SimTerrain& terrain);

    // The viewpoint key. In the original this is the ONE thing that arms the acquisition
    // controller (it writes 0xe4 to the acquisition counter), so it re-lines the camera up as well as
    // changing the distances.
    void cycle_preset() {
        preset = (preset + 1) % 3;
        acquire = kAcquireArmed;
    }

    // Is the CAMERA itself inside a tunnel bore this tick? Not the car's tunnel flag - see
    // clamp_over_ground for why the two differ at every tunnel exit.
    bool under_roof() const { return under_roof_; }

private:
    bool under_roof_ = false;
    void update_under_roof(const SimTerrain& terrain);
    int32_t last_dist4 = 0; // last tick's distance, still in quarter units
    uint8_t acquire = 0;    // bit 7: not lined up yet; low 7: ticks left, minus 4 a tick

    // Slide the camera `dist` units along its own yaw, tilted by `pitch`.
    void slide(int32_t dist, int32_t pitch);
    // Swing the camera around the vehicle by `dyaw`, keeping its horizontal
    // distance and its height. Acquisition only - see the header note.
    void orbit(const VehicleSim& car, int32_t dyaw);
    // The original's ground clearance, promoted from a panning-only detail to an always-on rule. See the .cpp.
    // `ease` false plants it at the clearance at once (the opening shot); true climbs to it.
    void clamp_over_ground(const VehicleSim& car, const SimTerrain& terrain, bool ease);
    // The direction the view points, and the yaw the acquisition reads back.
    void publish_bearing(const VehicleSim& car);
    // The vehicle, lifted by the preset's aim height - or by the tunnel one.
    void aim_point(const VehicleSim& car, int32_t& ax, int32_t& ay, int32_t& az) const;
};

// An eye and a basis in RENDER space (+Y up, world X mirrored about (map_w - 1) * 512), built
// from two consecutive tick states and how far the clock has travelled between them. `alpha`
// is the simulation accumulator over the tick length, in [0, 1].
struct CameraView {
    float eye[3] = {0.0f, 0.0f, 0.0f};
    float forward[3] = {0.0f, 0.0f, 1.0f};
    float right[3] = {1.0f, 0.0f, 0.0f};
    float up[3] = {0.0f, 1.0f, 0.0f};
};
CameraView camera_view(const CameraState& prev, const CameraState& cur, float alpha, int map_w,
                       float render_scale);
CameraView camera_view(const CameraSample& sample, int map_w, float render_scale);

// THE CABIN VIEW - camera mode 1. The eye sits at the
// vehicle's `internal` offset, turned with the vehicle; the view keeps its heading, and its pitch
// and roll follow the body's only in part (7/8 and 5/6), pitched down by `internal[3]`.
//
// ⚠️ Built from the vehicle's INTERPOLATED pose, not from a CameraState: the original's camera
// holds whole world units, and an eye rounded to the unit, thirty units from a dashboard drawn
// from the fractional pose, would visibly wobble against it.
struct VehiclePose;
CameraView cabin_view(const VehiclePose& pose, const int32_t (&internal)[4], int map_w,
                      float render_scale);

} // namespace bigred
