// A camera eye and aim as the display reads it: world position and BAM16 angles, unwrapped.
// Presentation only - the integer chase controller still runs on the 28 ms tick.
#pragma once

namespace bigred {

struct CameraSample {
    double x = 0, y = 0, z = 0;
    double yaw = 0, pitch = 0; // BAM16
};

} // namespace bigred
