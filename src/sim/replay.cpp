#include "replay.h"

#include <cstdlib>
#include <string>

#include "fixed.h"

namespace bigred {

namespace {

int32_t s16(const Blob& b, size_t o) {
    return static_cast<int32_t>(static_cast<int16_t>(b[o] | (b[o + 1] << 8)));
}

// `&1e0` is hex, anything else decimal - the level scripts' own number syntax.
bool read_number(const std::string& s, size_t& pos, int32_t& out) {
    while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t' || s[pos] == ',')) ++pos;
    if (pos >= s.size() || s[pos] == ';') return false;
    const bool hex = s[pos] == '&';
    if (hex) ++pos;
    const char* start = s.c_str() + pos;
    char* end = nullptr;
    const long v = std::strtol(start, &end, hex ? 16 : 10);
    if (end == start) return false;
    pos += static_cast<size_t>(end - start);
    out = static_cast<int32_t>(v);
    return true;
}

// Arithmetic shift of a product, as SAR does it.
int32_t mul_shift12(int32_t a, int32_t b) {
    return static_cast<int32_t>((static_cast<int64_t>(a) * b) >> 12);
}

} // namespace

ReplayTrack parse_replay(const Blob& b, int slot) {
    ReplayTrack t;
    if (b.size() < 40) return t;
    t.type = b[0];
    t.character = b[28];
    t.vehicle_index = static_cast<int8_t>(b[29]) / 2;
    t.logo = b[30];
    t.colour = b[31];
    if (t.type == 2 && slot >= 0) {
        t.character = slot;
        t.logo = slot;
    }
    const int count = s16(b, 32) & 0xFFFF;
    const size_t rs = (t.type == 2) ? 12 : 14;      // as in the original: type 2 is the short form
    if (b.size() < 40 + static_cast<size_t>(count) * rs) return ReplayTrack();
    t.records.resize(static_cast<size_t>(count));
    for (int r = 0; r < count; ++r) {
        const size_t o = 40 + static_cast<size_t>(r) * rs;
        std::array<int16_t, 7>& rec = t.records[static_cast<size_t>(r)];
        for (int k = 0; k < 6; ++k) rec[static_cast<size_t>(k)] = static_cast<int16_t>(s16(b, o + 2 * k));
        rec[6] = (rs == 14) ? static_cast<int16_t>(s16(b, o + 12)) : 0;
    }
    return t;
}

namespace {

// Calls `f(line, pos)` for every line of `txt` whose first word is exactly `word`, `pos` just
// past it.
template <typename F>
void for_each_directive(const std::string& txt, const char* word, F f) {
    const std::string w(word);
    size_t pos = 0;
    while (pos < txt.size()) {
        size_t end = txt.find('\n', pos);
        if (end == std::string::npos) end = txt.size();
        const std::string line = txt.substr(pos, end - pos);
        pos = end + 1;
        size_t p = line.find_first_not_of(" \t");
        if (p == std::string::npos || line.compare(p, w.size(), w) != 0) continue;
        p += w.size();
        if (p < line.size() && line[p] != ' ' && line[p] != '\t') continue;   // not "speedX"
        f(line, p);
    }
}

} // namespace

ReplayRates parse_replay_rates(const std::string& txt) {
    ReplayRates r;
    for_each_directive(txt, "speed", [&](const std::string& line, size_t p) {
        // The original reads min, normal and max, in that order.
        int32_t a = 0, b = 0, c = 0;
        if (read_number(line, p, a) && read_number(line, p, b) && read_number(line, p, c)) {
            r.min = a;
            r.normal = b;
            r.max = c;
        }
    });
    return r;
}

StartPlaces parse_start_places(const std::string& txt) {
    StartPlaces s;
    for_each_directive(txt, "spos", [&](const std::string& line, size_t p) {
        // The original reads the single race's first, then the championship's.
        int32_t a = 0, b = 0;
        if (read_number(line, p, a) && read_number(line, p, b)) {
            s.present = true;
            s.single = a;
            s.championship = b;
        }
    });
    return s;
}

void GhostDriver::start(const ReplayTrack& t, const VehicleSim& car) {
    track = &t;
    // As in the original: the accumulators take the car's own pose, time and index restart at 0, the
    // rate at 0x200 whatever the level's `speed` line says - the rubber band moves it later.
    acc[0] = car.px >> 9;
    acc[1] = car.py >> 9;
    acc[2] = car.pz >> 9;
    acc[3] = static_cast<int16_t>(car.pitch >> 8);
    acc[4] = static_cast<int16_t>(car.yaw >> 8);
    acc[5] = static_cast<int16_t>(car.roll >> 8);
    time = 0;
    index = 0;
    rate = 0x200;
}

void GhostDriver::tick(VehicleSim& car) {
    if (!track || !track->valid()) return;
    const int32_t count = static_cast<int32_t>(track->records.size());
    auto rec = [&](int32_t i) {
        static const std::array<int16_t, 7> kZero = {};
        return (i >= 0 && i < count) ? track->records[static_cast<size_t>(i)] : kZero;
    };
    // As in the original: the time advances by the rate, and never past the last
    // record - a ghost that has run out of tape stays where its last delta left it.
    time += rate;
    if (time < 0) time = 0;
    if (time >= (count << 12)) time = (count << 12) - 1;
    const int32_t target = time >> 12;
    const int32_t frac = time & 0xFFF;
    // Walk the accumulators to the target, one record at a time, either way.
    while (index != target) {
        if (index < target) {
            const auto r = rec(index);
            for (int k = 0; k < 6; ++k) acc[k] += r[static_cast<size_t>(k)];
            ++index;
        } else {
            const auto r = rec(index - 1);
            for (int k = 0; k < 6; ++k) acc[k] -= r[static_cast<size_t>(k)];
            --index;
        }
    }
    // The pose: the accumulators plus the fraction of the NEXT record already travelled.
    const auto r = rec(target);
    const int32_t x = acc[0] + mul_shift12(r[0], frac);
    const int32_t y = acc[1] + mul_shift12(r[1], frac);
    const int32_t z = acc[2] + mul_shift12(r[2], frac);
    const int16_t pitch16 = static_cast<int16_t>(acc[3] + mul_shift12(r[3], frac));
    const int16_t yaw16 = static_cast<int16_t>(acc[4] + mul_shift12(r[4], frac));
    const int16_t roll16 = static_cast<int16_t>(acc[5] + mul_shift12(r[5], frac));
    // The original copies it into the physics fields, which is what everything else reads.
    car.px = static_cast<int32_t>(static_cast<uint32_t>(x) << 9);
    car.py = static_cast<int32_t>(static_cast<uint32_t>(y) << 9);
    car.pz = static_cast<int32_t>(static_cast<uint32_t>(z) << 9);
    car.pitch = static_cast<int32_t>(pitch16) << 8;
    car.yaw = static_cast<int32_t>(yaw16) << 8;
    car.roll = static_cast<int32_t>(roll16) << 8;
    // The rates, for whatever reads them: record delta scaled by the playback rate, BAM24.
    car.pitch_rate = mul_shift12(r[3], rate) << 8;
    car.yaw_rate = mul_shift12(r[4], rate) << 8;
    car.roll_rate = mul_shift12(r[5], rate) << 8;
    // The steering the sound and the wheels read: the yaw rate over 128, truncated, +/-256.
    int32_t steer = car.yaw_rate / 128;
    if (steer < -256) steer = -256;
    if (steer > 256) steer = 256;
    car.steer_axis = steer;
    const int32_t dx = mul_shift12(r[0], rate);
    const int32_t dy = mul_shift12(r[1], rate);
    const int32_t dz = mul_shift12(r[2], rate);
    car.speed = (fx::hypot32(dx, dz) & 0xFFFF) << 9;
    car.vy = dy << 10;
    car.travel = fx::atan2_bam16(dx, dz) & 0xFFFF;   // X is the sine axis
    // The 7th field: the nitro flame's flipbook phase and the damage gauge, as recorded.
    const uint16_t carried = static_cast<uint16_t>(r[6]);
    car.flame_phase = static_cast<uint8_t>(carried & 0xFF);
    car.damage_percent = (carried >> 8) & 0xFF;
}

} // namespace bigred
