#include "chase_camera.h"

#include "../formats/terrain.h"
#include "../sim/fixed.h"
#include "vehicle_render.h"   // VehiclePose, vehicle_rotation - the cabin view

#include <algorithm>
#include <cmath>

namespace bigred {

// The original's own three presets, read straight out of the original game. The player cycles them with a key in the
// original; `C` does it here.
const CameraPreset kCameraPresets[3] = {
    {110, 130, 1500, 4000, 2000},
    {180, 700, 4000, 3000, 4000},
    {300, 1400, 8000, 2000, 6000},
};

namespace {

// The camera compares distances in quarter units throughout - the original's distance routine
// shifts its length left by 2 before returning it. Kept as one named constant so the factor never goes missing
// from one comparison and not another.
constexpr int32_t kQuarter = 4;

// The original's hard ceiling on the follow distance: 0x1c00 quarter units = 1792 world units.
constexpr int32_t kMaxDist4 = 0x1c00;

// How fast the target may close in, in quarter units per tick. This is the whole of the
// start-of-race pull-in: the camera is planted 2500 units back and walks in 37.5 units a tick.
constexpr int32_t kCloseRate = 150;

// The dead zone of the acquisition controller, and its rate, both in BAM16. 0x400 is 5.6
// degrees; 0x100 per 28 ms tick is about 50 degrees a second.
constexpr int32_t kYawDeadZone = 0x400;
constexpr int32_t kYawRate = 0x100;

// The original's opening shot: pitched 0x800 down, 2500 units behind, and 10000 quarter units
// on the clock so the closing limiter has something to walk down from.
constexpr int32_t kOpeningPitch = 0x800;
constexpr int32_t kOpeningDist4 = 10000;

// The clearance the camera keeps over the terrain - the original's own value, 0x32.
constexpr int32_t kGroundClearance = 50;
// Ours: how close the lens may get to the ground while it eases back up to kGroundClearance.
// It has to stay above the near plane's reach (kNearPlaneWorld, level_scene.h) or the terrain is
// cut open again.
constexpr int32_t kHardClearance = 20;

int16_t wrap16(int32_t a) { return static_cast<int16_t>(static_cast<uint16_t>(a)); }

} // namespace

void ChaseCamera::slide(int32_t dist, int32_t pitch) {
    // As in the original: with a zero pitch this is a plain horizontal step along the yaw; the only
    // caller that passes one is the opening placement.
    const int32_t flat = fx::cos16(pitch);
    now.x += fx::mul15(fx::mul15(fx::sin16(now.yaw), flat), dist);
    now.z += fx::mul15(fx::mul15(fx::cos16(now.yaw), flat), dist);
    now.y += fx::mul15(fx::sin16(pitch), dist);
}

void ChaseCamera::aim_point(const VehicleSim& car, int32_t& ax, int32_t& ay, int32_t& az) const {
    // The vehicle, lifted by the preset's height - or by a flat 90 when it is
    // under something, because in a tunnel that height would aim at the soffit.
    ax = car.world_x();
    ay = car.world_y() -
         (car.in_tunnel ? kTunnelAimHeight : kCameraPresets[preset].aim_height);
    az = car.world_z();
}

void ChaseCamera::orbit(const VehicleSim& car, int32_t dyaw) {
    // The original's orbit, and it is worth reading twice: the camera is TELEPORTED onto the vehicle's
    // X/Z (its height is left alone), the bearing is turned by `dyaw`, and it is pushed back
    // out by the horizontal distance it had. The result is an exact circular sweep about the
    // car - no drift in or out, which is why this and the distance controller never fight.
    const int32_t dx = car.world_x() - now.x;
    const int32_t dz = car.world_z() - now.z;
    const int32_t hdist = fx::hypot32(dx, dz);
    now.yaw = wrap16(fx::atan2_bam16(dx, dz) + dyaw);
    now.x = car.world_x();
    now.z = car.world_z();
    slide(-hdist, 0);
}

void ChaseCamera::clamp_over_ground(const VehicleSim& car, const SimTerrain& terrain, bool ease) {
    // ⚠️ DELIBERATE ADDITION, asked for by the user on 2026-09-15 ("it also has to be blocked
    // by the scenery"). The original has this code, but only reaches it while
    // the player is panning the view by hand, so in ordinary driving its camera will happily
    // sink through a hillside. The clearance is its number, not one of ours.
    //
    // Skipped in a tunnel, exactly as the original skips it there: down inside one the terrain
    // is ABOVE the camera and this would fling it up through the roof.
    //
    // ⚠️ AND SKIPPED WHILE THE CAMERA ITSELF IS IN THE BORE. The car's flag alone drops at the
    // mouth while the camera, a few hundred units back, is still inside under the hill: the
    // clamp then threw it over the hilltop in one tick - +1086 units on DIRT3, the user's "elle
    // monte d'un coup a la sortie de tunnel" of 2026-09-23. The bench's TUNNELS block measures it.
    if (car.in_tunnel || under_roof_ || !terrain.valid()) return;
    const int32_t ground = terrain.height_fx(now.x << 9, now.z << 9) >> 9;
    const int32_t ceiling = ground - kGroundClearance; // +Y down: subtracting is rising
    const int32_t hard = ground - kHardClearance;
    if (!ease || hard < now.y) {
        // Below the hard floor (or planting the camera): no easing, the lens would be in the
        // ground. On continuous terrain this only ever moves it by one tick's rise.
        if (ceiling < now.y) now.y = (ease ? hard : ceiling);
    } else if (ceiling < now.y) {
        // Between the two: climb a quarter of the way to the original's 50-unit clearance per
        // tick instead of snapping to it, so every bump of the heightmap under the camera is
        // not passed straight to the picture. Rounded up so it always arrives.
        now.y -= (now.y - ceiling + 3) / 4;
    }
}

void ChaseCamera::update_under_roof(const SimTerrain& terrain) {
    // The car's own tunnel test (SimTerrain::gather_candidates), asked at the camera's position
    // with a small cube for a body so the tube's box test still trims the bounding circle - a
    // camera beside a tunnel is not in it.
    static const VehicleBox kLens = [] {
        VehicleBox b;
        b.xmin = b.ytop = b.zmin = -16;
        b.xmax = b.ybottom = b.zmax = 16;
        b.radius = 16;
        b.build();
        return b;
    }();
    under_roof_ = terrain.valid() &&
                  terrain.gather_candidates(now.x << 9, now.y << 9, now.z << 9, kLens.radius,
                                            &kLens, 0).inside;
}

void ChaseCamera::publish_bearing(const VehicleSim& car) {
    // The bearing, which the original computes once per RENDERED frame. Two fields come out of it:
    // the direction the view points, and - the part that is easy to miss - the yaw the
    // acquisition controller reads back as "where is the camera relative to the car?". Running
    // it once a tick instead is the same answer without the frame dependence.
    const int32_t dx = car.world_x() - now.x, dz = car.world_z() - now.z;
    now.yaw = wrap16(fx::atan2_bam16(dx, dz));
    // THE LOOK-AHEAD (found in the original, 2026-09-27; the user felt it
    // before anyone found it: "la caméra anticipe légèrement la direction à prendre"). The view
    // does not point straight at the car: it turns a QUARTER of the way toward the car's own
    // heading, the angle first folded so that a car seen nose-on shifts nothing (by
    // quadrant: 0 and 3 keep it, 1 takes 0x7fff - a, 2 takes -0x8000 - a), then
    // scaled by the speed up to 4000 - which a moving car always has. Up to 22.5 degrees: in
    // a bend the picture swings toward the way out before the rope has dragged the camera
    // round. The same field is what the acquisition and the 5-unit nudge read back, in the
    // original as here.
    {
        int32_t s = car.speed < 0 ? -car.speed : car.speed;
        if (s > 0xfa0) s = 0xfa0;
        const int32_t a = static_cast<int16_t>(car.yaw_bam16() - now.yaw);
        int32_t f = a;
        switch ((a >> 14) & 3) {
        case 1: f = 0x7fff - a; break;
        case 2: f = -0x8000 - a; break;
        default: break;
        }
        f = static_cast<int16_t>(f >> 2);
        now.yaw = wrap16(now.yaw + static_cast<int16_t>(f * s / 0xfa0));
    }
    // ⚠️ fx::atan2_bam16 returns an UNSIGNED BAM16 in [0, 65535]. Sign it BEFORE comparing it
    // with anything: clamping the raw value pins every upward look at exactly +45 degrees,
    // which is how this was found on the bench.
    int32_t pitch = wrap16(fx::atan2_bam16(car.world_y() - now.y, fx::hypot32(dx, dz)));
    if (pitch > 0x2000) pitch = 0x2000; // the original's clamp: 45 degrees either way
    if (pitch < -0x2000) pitch = -0x2000;
    // The original only moves the pitch once the bearing has drifted more than 0x41 off it -
    // a sixth of a degree of hysteresis, enough to kill the one-unit jitter of a settled car.
    const int32_t err = pitch - now.pitch;
    if (err > 0x41 || err < -0x41) now.pitch = pitch;
}

void ChaseCamera::reset(const VehicleSim& car, const SimTerrain& terrain) {
    // The mode-4 entry: pitched down a notch, lined up with the car's heading,
    // and planted 2500 units behind it. The distance controller walks it in from there over
    // about a second and a half - that opening move is not decoration, it is what the 150-per-
    // tick closing limit exists for.
    now.yaw = wrap16(car.yaw_bam16());
    now.x = car.world_x();
    now.y = car.world_y() - kCameraPresets[preset].aim_height;
    now.z = car.world_z();
    slide(-(kOpeningDist4 / kQuarter), kOpeningPitch);
    last_dist4 = kOpeningDist4;
    acquire = 0;
    settled = false;
    update_under_roof(terrain);
    clamp_over_ground(car, terrain, false);
    now.pitch = kOpeningPitch; // the mode-4 entry seeds it; the bearing takes over from here
    publish_bearing(car);
}

void ChaseCamera::step(const VehicleSim& car, const SimTerrain& terrain) {
    const CameraPreset& p = kCameraPresets[preset];

    // 1. Too close: shove the camera back 5 units along its own bearing. The head of the
    //    original's update, and it is a nudge rather than a snap - it simply runs every tick it is
    //    true, which is enough to keep the car from climbing into the lens.
    {
        const int32_t dx = car.world_x() - now.x, dz = car.world_z() - now.z;
        if (fx::hypot32(dx, dz) * kQuarter < p.dist_min) slide(-5, 0);
    }

    // 2. The follow distance, in quarter units. Speed alone drives it, saturating at 20000 -
    //    well below what a JEEP reaches, so the camera is at full stretch before the car is.
    int32_t want = p.dist_base + fx::muldiv(p.dist_factor,
                                            (car.speed > 20000) ? 20000 : car.speed, 20000);
    if (want < p.dist_min) want = p.dist_min;
    // ...and it may not close in faster than kCloseRate per tick on last tick's real distance,
    // unless the speed ramp has already topped out. Braking hard would otherwise yank the
    // camera into the car's boot, and a race would start with a jump cut instead of a pull-in.
    if (want < last_dist4 && want < p.dist_base + p.dist_factor &&
        last_dist4 - want > kCloseRate)
        want = last_dist4 - kCloseRate;
    if (want > kMaxDist4) want = kMaxDist4;

    // 3. THE ROPE, and this is the whole feel of the thing. `cur4` is four times the distance
    //    to the aim point. When it exceeds the target, the camera slides ALONG that line by
    //    exactly the fraction that closes the gap; when it does not, the camera does not move
    //    at all. It never moves sideways. So a car that turns is not followed - it swings out
    //    to the side of the frame and the camera is dragged in behind it as it drives away.
    //    THAT is the camera lag, and it costs nothing to compute and has no tuning constant.
    int32_t ax, ay, az;
    aim_point(car, ax, ay, az);
    const int32_t vx = ax - now.x, vy = ay - now.y, vz = az - now.z;
    const int32_t cur4 = fx::hypot3(vx, vy, vz) * kQuarter;
    int32_t mx = 0, my = 0, mz = 0;
    if (cur4 > want && cur4 > 0) {
        const int32_t f = static_cast<int32_t>((static_cast<int64_t>(cur4 - want) << 15) / cur4);
        mx = fx::mul15(vx, f);
        my = fx::mul15(vy, f);
        mz = fx::mul15(vz, f);
    }

    // 4. The vertical is normally part of that rope and nothing more. It gets its own slack
    //    first-order chase in two cases only: while the view is being re-acquired, and under a
    //    tunnel roof, where it tightens to an eighth so the camera dives in with the car.
    if (acquire != 0 || car.in_tunnel) my = car.in_tunnel ? (vy / 8) : (vy / 30);

    now.x += mx;
    now.y += my;
    now.z += mz;

    settled = (vx > -10 && vx < 10) && (vy > -10 && vy < 10) && (vz > -10 && vz < 10);

    // 5. Acquisition - reached only while the acquisition counter is armed. ⚠️ It is NOT the
    //    steady-state behaviour: in the original nothing arms it during ordinary driving, so
    //    it fires when the player changes viewpoint and at no other time. Swinging the camera
    //    round behind the car every tick would destroy the lag the rope produces.
    //
    //    ⚠️ The two directions are not symmetric in the original: it subtracts the dead zone
    //    before testing both branches, so an error one way always gets the full rate while the
    //    other eases in over its last 512 units. Ported as written.
    if (acquire != 0) {
        if (acquire & 0x7f) acquire = static_cast<uint8_t>(acquire - 4);
        const int16_t d = wrap16(car.yaw_bam16() - now.yaw);
        if (d > kYawDeadZone) {
            const int32_t excess = d - kYawDeadZone;
            orbit(car, (excess > 0x200) ? kYawRate : (excess / 2));
        } else if (d < -kYawDeadZone) {
            orbit(car, -kYawRate);
        } else {
            acquire &= 0x7f; // lined up: the counter alone now decides when this stops
        }
    }

    // 6. Never under the ground. See clamp_over_ground - this one is ours.
    update_under_roof(terrain);
    clamp_over_ground(car, terrain, true);

    last_dist4 = wrap16(cur4);
    publish_bearing(car);
}

CameraView cabin_view(const VehiclePose& pose, const int32_t (&internal)[4], int map_w,
                      float render_scale) {
    constexpr float kBamToRad = 6.283185307179586f / 65536.0f;
    float body[9];
    vehicle_rotation(pose.pitch * kBamToRad, pose.yaw * kBamToRad, pose.roll * kBamToRad, body);
    const float ox = static_cast<float>(internal[0]), oy = static_cast<float>(internal[1]),
                oz = static_cast<float>(internal[2]);
    const float gx = pose.x + body[0] * ox + body[1] * oy + body[2] * oz;
    const float gy = pose.y + body[3] * ox + body[4] * oy + body[5] * oz;
    const float gz = pose.z + body[6] * ox + body[7] * oy + body[8] * oz;

    // The head's damping. Inside +/-90 degrees the angle is scaled by num/den; past it, the
    // distance to the half turn is scaled by (2 den - num)/den instead, which meets the first
    // rule at exactly 90 degrees (the original's: 7/8 and 9/8 for the pitch, 5/6 and 7/6 roll).
    const auto damp = [](float bam, float num, float den) {
        float a = std::fmod(bam, 65536.0f);
        if (a >= 32768.0f) a -= 65536.0f;
        if (a < -32768.0f) a += 65536.0f;
        if (a > -16384.0f && a < 16384.0f) return a * num / den;
        const float k = (2.0f * den - num) / den;
        return a > 0.0f ? 32768.0f - (32768.0f - a) * k : (a + 32768.0f) * k - 32768.0f;
    };
    // The published pitch is held to +/-0x2000, whatever the mode, as in the original.
    float pitch = damp(pose.pitch, 7.0f, 8.0f) - static_cast<float>(internal[3]);
    pitch = std::max(-8192.0f, std::min(8192.0f, pitch));
    const float roll = damp(pose.roll, 5.0f, 6.0f);
    float m[9];
    vehicle_rotation(pitch * kBamToRad, pose.yaw * kBamToRad, roll * kBamToRad, m);

    // Columns of `m`: the eye's right, its DOWN (game +Y) and its forward, in game axes. Render
    // space mirrors X and flips Y: (x, y, z) -> (-x, -y, z).
    CameraView v;
    const float mirror_axis =
        static_cast<float>(map_w - 1) * static_cast<float>(kTerrainTexelWorldSize);
    v.eye[0] = (mirror_axis - gx) * render_scale;
    v.eye[1] = -gy * render_scale;
    v.eye[2] = gz * render_scale;
    v.forward[0] = -m[2];
    v.forward[1] = -m[5];
    v.forward[2] = m[8];
    v.right[0] = -m[0];
    v.right[1] = -m[3];
    v.right[2] = m[6];
    v.up[0] = m[1];     // up = -down, then mirrored: (-(-d.x), -(-d.y), -d.z)
    v.up[1] = m[4];
    v.up[2] = -m[7];
    return v;
}

CameraView camera_view(const CameraState& prev, const CameraState& cur, float alpha, int map_w,
                       float render_scale) {
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    auto mix = [alpha](int32_t a, int32_t b) {
        return static_cast<float>(a) + (static_cast<float>(b) - static_cast<float>(a)) * alpha;
    };
    // Angles take the short way round: a bearing crossing due north steps 65535 -> 0 and a
    // straight interpolation would spin the view a full turn between two ticks.
    auto mix_angle = [alpha](int32_t a, int32_t b) {
        const int32_t d = static_cast<int16_t>(static_cast<uint16_t>(b - a));
        return static_cast<float>(a) + static_cast<float>(d) * alpha;
    };

    // A `recover` puts the vehicle back over its grid slot, which can be the far side of the
    // map, and the camera is re-planted with it. Interpolating across that smears the whole
    // world past the screen; snapping is what a player expects of a respawn.
    const int64_t dx = cur.x - prev.x, dy = cur.y - prev.y, dz = cur.z - prev.z;
    const bool jumped = (dx * dx + dy * dy + dz * dz) > (4000LL * 4000LL);

    const float gx = jumped ? static_cast<float>(cur.x) : mix(prev.x, cur.x);
    const float gy = jumped ? static_cast<float>(cur.y) : mix(prev.y, cur.y);
    const float gz = jumped ? static_cast<float>(cur.z) : mix(prev.z, cur.z);
    const float yaw = jumped ? static_cast<float>(cur.yaw) : mix_angle(prev.yaw, cur.yaw);
    const float pitch = jumped ? static_cast<float>(cur.pitch) : mix_angle(prev.pitch, cur.pitch);

    return camera_view(CameraSample{gx, gy, gz, yaw, pitch}, map_w, render_scale);
}

CameraView camera_view(const CameraSample& sample, int map_w, float render_scale) {
    constexpr double kBamToRad = 6.283185307179586 / 65536.0;
    const double ya = sample.yaw * kBamToRad, pa = sample.pitch * kBamToRad;
    const float cp = std::cos(pa), sp = std::sin(pa);

    CameraView v;
    const float mirror_axis =
        static_cast<float>(map_w - 1) * static_cast<float>(kTerrainTexelWorldSize);
    v.eye[0] = static_cast<float>((mirror_axis - sample.x) * render_scale);
    v.eye[1] = static_cast<float>(-sample.y * render_scale);
    v.eye[2] = static_cast<float>(sample.z * render_scale);

    // Game axes: x = sin(yaw), z = cos(yaw), y = sin(pitch) with +Y DOWN. Render space mirrors
    // X and flips Y, which is exactly the transform the decor and the vehicle already use.
    v.forward[0] = -std::sin(ya) * cp;
    v.forward[1] = -sp;
    v.forward[2] = std::cos(ya) * cp;

    auto norm = [](float o[3]) {
        const float n = std::sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]);
        if (n > 0.0f) { o[0] /= n; o[1] /= n; o[2] /= n; }
    };
    norm(v.forward);
    // right = forward x world_up, up = right x forward - the same basis map_preview builds, so
    // the sky shader can be handed either one without knowing which camera it came from.
    v.right[0] = -v.forward[2];
    v.right[1] = 0.0f;
    v.right[2] = v.forward[0];
    norm(v.right);
    v.up[0] = v.right[1] * v.forward[2] - v.right[2] * v.forward[1];
    v.up[1] = v.right[2] * v.forward[0] - v.right[0] * v.forward[2];
    v.up[2] = v.right[0] * v.forward[1] - v.right[1] * v.forward[0];
    norm(v.up);
    return v;
}

} // namespace bigred
