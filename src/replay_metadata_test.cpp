#include "sim/replay.h"

#include <cstdint>
#include <iostream>
#include <string>

namespace {

using bigred::Blob;
using bigred::ReplayTrack;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (condition) return;
    std::cerr << "replay metadata: " << what << '\n';
    ++failures;
}

Blob replay(int type, int character, int vehicle, int logo, int colour, int count = 1) {
    const std::size_t record_size = type == 2 ? 12 : 14;
    Blob bytes(40 + static_cast<std::size_t>(count) * record_size);
    bytes[0] = static_cast<std::uint8_t>(type);
    bytes[28] = static_cast<std::uint8_t>(character);
    bytes[29] = static_cast<std::uint8_t>(vehicle);
    bytes[30] = static_cast<std::uint8_t>(logo);
    bytes[31] = static_cast<std::uint8_t>(colour);
    bytes[32] = static_cast<std::uint8_t>(count & 0xff);
    bytes[33] = static_cast<std::uint8_t>((count >> 8) & 0xff);
    for (int i = 0; i < count; ++i) {
        const auto at = 40u + static_cast<std::size_t>(i) * record_size;
        bytes[at] = 1;
        bytes[at + 1] = 0;
    }
    return bytes;
}

void type3_uses_independent_character_and_colour() {
    const ReplayTrack first = bigred::parse_replay(replay(3, 0, 2, 0, 2), 4);
    check(first.valid(), "type 3 sample is valid");
    check(first.character == 0, "type 3 byte 28 is character");
    check(first.vehicle_index == 1, "type 3 byte 29 maps to a vehicle");
    check(first.logo == 0, "type 3 byte 30 is logo");
    check(first.colour == 2, "type 3 byte 31 is colour");

    const ReplayTrack second = bigred::parse_replay(replay(3, 3, 0, 3, 5));
    check(second.character == 3, "second type 3 character");
    check(second.logo == 3, "second type 3 logo");
    check(second.colour == 5, "second type 3 colour");
}

void type2_overrides_character_and_logo_only() {
    const ReplayTrack raw = bigred::parse_replay(replay(2, 1, 4, 6, 3));
    check(raw.character == 1 && raw.logo == 6, "standalone type 2 keeps raw selectors");
    check(raw.colour == 3, "standalone type 2 keeps colour");

    const ReplayTrack blocked = bigred::parse_replay(replay(2, 1, 4, 6, 3), 4);
    check(blocked.character == 4, "type 2 block overrides character");
    check(blocked.vehicle_index == 2, "type 2 keeps vehicle byte 29");
    check(blocked.logo == 4, "type 2 block overrides logo");
    check(blocked.colour == 3, "type 2 block keeps byte 31 colour");
}

void malformed_tracks_have_no_records() {
    check(!bigred::parse_replay(Blob()).valid(), "empty input has no records");
    check(!bigred::parse_replay(replay(3, 0, 0, 0, 0, 0)).valid(),
          "zero-count track has no records");
    Blob truncated = replay(3, 3, 0, 3, 5);
    truncated.pop_back();
    check(!bigred::parse_replay(truncated).valid(), "truncated payload has no records");
}

} // namespace

int main() {
    type3_uses_independent_character_and_colour();
    type2_overrides_character_and_logo_only();
    malformed_tracks_have_no_records();
    if (failures) {
        std::cerr << "replay metadata: " << failures << " failure(s)\n";
        return 1;
    }
    std::cout << "replay metadata: all checks passed\n";
    return 0;
}
