#include "comfort_camera.h"

#include <algorithm>
#include <cmath>

namespace bigred {
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double bam = 65536.0 / (2*pi);
constexpr double distances[] = {1400, 1800, 2300};
constexpr double heights[] = {420, 700, 1000};
double short_angle(double angle) { return std::remainder(angle, 2*pi); }
double held_goal(double current, double target, double radius) {
    return target - std::clamp(target-current, -radius, radius);
}
double length(CameraPoint p) { return std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z); }
} // namespace

void ComfortCamera::damp(Spring& spring, double goal, double rate, double seconds) {
    // Exact critically damped spring for a constant goal: stable even on a slow frame.
    const double error = spring.value-goal;
    const double change = spring.velocity+rate*error;
    const double decay = std::exp(-rate*seconds);
    spring.value = goal+(error+change*seconds)*decay;
    spring.velocity = (spring.velocity-rate*change*seconds)*decay;
}

void ComfortCamera::reset(const ComfortTarget& target) {
    held_ = previous_ = target.position;
    filtered_velocity_ = {};
    x_ = {held_.x,0}; y_ = {held_.y,0}; z_ = {held_.z,0};
    orbit_ = {target.yaw/bam,0};
    confined_ = target.confined;
    const int preset = std::clamp(target.preset,0,2);
    distance_ = {confined_ ? 650 : distances[preset],0};
    height_ = {confined_ ? 90 : heights[preset],0};
    yaw_goal_ = orbit_.value;
    pitch_goal_ = std::atan2(height_.value-(confined_ ? 30 : 80),distance_.value);
    aim_yaw_ = {yaw_goal_,0}; aim_pitch_ = {pitch_goal_,0};
    clear_seconds_ = 0;
    arm_fraction_ = 1;
    initialized_ = true;
}

void ComfortCamera::advance(const ComfortTarget& target, CameraPoint velocity, double seconds) {
    const double offset_x = target.position.x-held_.x, offset_z = target.position.z-held_.z;
    const double offset = std::hypot(offset_x,offset_z);
    if (offset > 30) {
        held_.x = target.position.x-offset_x*30/offset;
        held_.z = target.position.z-offset_z*30/offset;
    }
    held_.y = held_goal(held_.y,target.position.y,60);
    damp(x_,held_.x,8,seconds); damp(z_,held_.z,8,seconds);
    damp(y_,held_.y,4,seconds); // heavier vertical follow rejects rotor/terrain bobbing

    // Bound the follow lag at high speed: the subject must not cross the eye
    // during a fast reverse or dive. Normal hover motion stays in the dead zones.
    const double dx = x_.value-target.position.x, dz = z_.value-target.position.z;
    const double lag = std::hypot(dx,dz);
    if (lag > 500) {
        x_.value = target.position.x+dx*500/lag;
        z_.value = target.position.z+dz*500/lag;
    }
    y_.value = std::clamp(y_.value,target.position.y-350,target.position.y+350);

    const double blend = 1-std::exp(-8*seconds);
    filtered_velocity_.x += (velocity.x-filtered_velocity_.x)*blend;
    filtered_velocity_.z += (velocity.z-filtered_velocity_.z)*blend;
    if (std::hypot(filtered_velocity_.x,filtered_velocity_.z) > 250) {
        double heading = std::atan2(filtered_velocity_.x,filtered_velocity_.z);
        double delta = short_angle(heading-orbit_.value);
        // Reversing or rotating in place must not whip the camera through 180 degrees.
        if (std::abs(delta) > pi/2) heading += pi;
        const double before = orbit_.value;
        damp(orbit_,before+short_angle(heading-before),3.5,seconds);
        constexpr double max_turn = pi/3; // 60 degrees/second
        orbit_.value = before+std::clamp(orbit_.value-before,-max_turn*seconds,max_turn*seconds);
        orbit_.velocity = std::clamp(orbit_.velocity,-max_turn,max_turn);
    } else {
        orbit_.velocity *= std::exp(-8*seconds);
    }

    if (target.confined) { confined_ = true; clear_seconds_ = 0; }
    else if (confined_ && (clear_seconds_ += seconds) >= 0.35) confined_ = false;
    const int preset = std::clamp(target.preset,0,2);
    damp(distance_,confined_ ? 650 : distances[preset],8,seconds);
    damp(height_,confined_ ? 90 : heights[preset],8,seconds);
}

CameraSample ComfortCamera::update(const ComfortTarget& target, double seconds, bool cut,
                                  const Obstruction& obstruction) {
    const CameraPoint difference{target.position.x-previous_.x,target.position.y-previous_.y,
                                 target.position.z-previous_.z};
    const bool reset_needed = !initialized_ || cut || length(difference)>4000;
    if (reset_needed) reset(target);
    seconds = std::clamp(seconds,0.0,0.25);
    const CameraPoint start = previous_;
    const CameraPoint velocity = seconds>0
        ? CameraPoint{(target.position.x-start.x)/seconds,(target.position.y-start.y)/seconds,
                      (target.position.z-start.z)/seconds} : CameraPoint{};
    // Small substeps evaluate the dead zones on the same timescale at 30 or 240 Hz.
    const int steps = std::max(1,int(std::ceil(seconds*240)));
    for (int i=1; i<=steps; ++i) {
        ComfortTarget intermediate = target;
        const double t = double(i)/steps;
        intermediate.position = {start.x+(target.position.x-start.x)*t,
                                 start.y+(target.position.y-start.y)*t,
                                 start.z+(target.position.z-start.z)*t};
        advance(intermediate,velocity,seconds/steps);
    }
    previous_ = target.position;

    const CameraPoint focus{target.position.x,target.position.y-(confined_ ? 30 : 80),target.position.z};
    const CameraPoint desired{x_.value-std::sin(orbit_.value)*distance_.value,
                              y_.value-height_.value,
                              z_.value-std::cos(orbit_.value)*distance_.value};
    // Even a fully blocked focus must retain an eye/aim separation. This is a
    // close-view fallback, not permission to divide by a zero-length boom.
    const double boom = length({desired.x-focus.x,desired.y-focus.y,desired.z-focus.z});
    const double minimum = std::min(1.0,80/std::max(boom,1.0));
    const double allowed = obstruction ? std::clamp(obstruction(focus,desired),minimum,1.0) : 1;
    if (allowed < arm_fraction_) arm_fraction_ = allowed; // collision retracts immediately
    else arm_fraction_ += (allowed-arm_fraction_)*(1-std::exp(-seconds/0.25));
    const CameraPoint eye{focus.x+(desired.x-focus.x)*arm_fraction_,
                          focus.y+(desired.y-focus.y)*arm_fraction_,
                          focus.z+(desired.z-focus.z)*arm_fraction_};
    const double dx=focus.x-eye.x, dy=focus.y-eye.y, dz=focus.z-eye.z;
    const double horizontal = std::hypot(dx,dz);
    const double desired_yaw = horizontal>1 ? std::atan2(dx,dz) : aim_yaw_.value;
    const double desired_pitch = std::clamp(std::atan2(dy,std::max(horizontal,1.0)),-pi*4/9,pi*4/9);
    constexpr double yaw_zone = pi/120, pitch_zone = pi/90; // 1.5 / 2 degree framing zones
    yaw_goal_ = held_goal(yaw_goal_,yaw_goal_+short_angle(desired_yaw-yaw_goal_),yaw_zone);
    pitch_goal_ = held_goal(pitch_goal_,desired_pitch,pitch_zone);
    if (reset_needed || seconds==0) {
        aim_yaw_ = {desired_yaw,0}; aim_pitch_ = {desired_pitch,0};
        yaw_goal_ = desired_yaw; pitch_goal_ = desired_pitch;
    } else {
        // Aim is independent from the orbit. Faster than position follow, while
        // the framing zones keep small object movements from panning the world.
        const int aim_steps = std::max(1,int(std::ceil(seconds*240)));
        for (int i=0; i<aim_steps; ++i) {
            damp(aim_yaw_,yaw_goal_,12,seconds/aim_steps);
            damp(aim_pitch_,pitch_goal_,12,seconds/aim_steps);
        }
    }
    current_ = {eye.x,eye.y,eye.z,aim_yaw_.value*bam,aim_pitch_.value*bam};
    return current_;
}
} // namespace bigred
