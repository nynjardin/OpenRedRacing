#pragma once

#include "camera_sample.h"
#include <functional>

namespace bigred {
struct CameraPoint { double x = 0, y = 0, z = 0; };
struct ComfortTarget {
    CameraPoint position;
    double yaw = 0; // BAM16, used to place the rig on activation only
    int preset = 0;
    bool confined = false;
};

// A separate render-time rig: a damped position anchor and orbit heading,
// followed by independent object framing. Body pitch/roll never drive it.
// Targets must use the same interpolated pose that the renderer draws.
class ComfortCamera {
public:
    using Obstruction = std::function<double(CameraPoint focus, CameraPoint desired_eye)>;
    CameraSample update(const ComfortTarget& target, double seconds, bool cut = false,
                        const Obstruction& obstruction = {});
    const CameraSample& current() const { return current_; }
    bool ready() const { return initialized_; }

private:
    struct Spring { double value = 0, velocity = 0; };
    static void damp(Spring& spring, double goal, double rate, double seconds);
    void reset(const ComfortTarget& target);
    void advance(const ComfortTarget& target, CameraPoint velocity, double seconds);
    CameraPoint held_, filtered_velocity_, previous_;
    Spring x_, y_, z_, orbit_, distance_, height_, aim_yaw_, aim_pitch_;
    double yaw_goal_ = 0, pitch_goal_ = 0, clear_seconds_ = 0, arm_fraction_ = 1;
    bool initialized_ = false, confined_ = false;
    CameraSample current_;
};
} // namespace bigred
