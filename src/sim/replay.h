// The opponents: recorded ghosts, not an AI.
//
// A `.REC` is the DELTA of a car's pose from one record to the next - position in world units
// and the three render angles in BAM16 - written by the original's recorder while a developer
// drove. Played back, each ghost starts from ITS OWN grid pose and adds the deltas up; the pose
// shown between two records is interpolated. Everything here is integer and follows the
// original, so a ghost is bit-identical on every machine of a network game.
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "../formats/bigred_archive.h" // Blob
#include "vehicle.h"

namespace bigred {

struct ReplayTrack {
    // The file's 40-byte header.
    int type = 0;            // byte 0: 2 = 12-byte records, 3 = 14 (the 7th field below)
    // Bytes 28, 30 and 31 are the racer's person, logo and colour, as the original's loader
    // reads them; a type-2 file takes its person and logo from its slot (N of `N.REC`) instead.
    int character = 0;       // byte 28 - the recorded person/head
    int vehicle_index = 0;   // byte 29 / 2 - which of the level's `car` lines the ghost drives
    int logo = 0;            // byte 30 - the recorded logo
    int colour = 0;          // byte 31 - the team/body colour
    // x, y, z, pitch, yaw, roll deltas, then (type 3) the flame counter in the low byte of the
    // 7th field and the damage percent in its high byte - not deltas, absolute.
    std::vector<std::array<int16_t, 7>> records;

    bool valid() const { return !records.empty(); }
};

// Parses `N.REC`. An empty track on anything that does not add up: a missing ghost must never
// stop a race. Supplying the original `N.REC` block lets the type-2 loader override match the
// retail format; callers with a standalone stream may leave it unset.
ReplayTrack parse_replay(const Blob& rec, int slot = -1);

// The playback rates. The original starts every ghost at 0x200 per 28 ms tick (0x1000 = one
// record, so one record every eight ticks - the rate the recorder ran at, i.e. real time), and
// its rubber band moves it within the level's `speed` line: min, normal, max.
struct ReplayRates {
    int32_t min = 0x120, normal = 0x200, max = 0x300;   // the original's own defaults
};
ReplayRates parse_replay_rates(const std::string& level_txt);

// The grid block the HUMAN takes. The original draws both at every level load as
// `rand() % 6` (the C runtime's LCG); a `spos <single>, <championship>` line
// overrides the draw. The original then hands the player the car
// standing on that block: the ghost recorded for it is not raced.
struct StartPlaces {
    bool present = false;
    int32_t single = 0, championship = 0;
};
StartPlaces parse_start_places(const std::string& level_txt);

// One ghost's playback state - the original's per-ghost accumulator bank, its time and its
// rate.
struct GhostDriver {
    const ReplayTrack* track = nullptr;
    int32_t acc[6] = {};     // x, y, z (world units), pitch, yaw, roll (BAM16)
    int32_t time = 0;        // 12-bit fixed point: record index << 12
    int32_t rate = 0x200;
    int32_t index = 0;       // the record the accumulators have reached

    // The ghost starts where the car stands - its grid pose.
    void start(const ReplayTrack& t, const VehicleSim& car);
    // The original's playback step plus the pose half of its ghost update: advance one 28 ms
    // tick and write the pose, the velocities it implies and the two carried bytes into `car`.
    void tick(VehicleSim& car);
};

} // namespace bigred
