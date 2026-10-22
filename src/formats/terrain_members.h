// Resolve a level archive's terrain family before the scene loader looks for
// canonical MAP.* names.
//
// Every shipped level except RUF1/2/3 uses MAP.<suffix>. Those three use a
// single-digit stem (1.MPH, 2.CHT, 3.COL, ...). This helper selects one
// complete MPH+MPC+COL family, then optionally creates missing MAP aliases for
// the known terrain/collision/scenery suffixes. Original members are kept.
#pragma once

#include "formats/game_files.h"

#include <array>
#include <cstddef>
#include <string>
#include <string_view>

namespace bigred {

// The complete set observed on canonical and alternate-stem retail levels.
// MPH, MPC and COL are required; the remaining five are optional.
inline constexpr std::array<const char*, 8> kTerrainMemberSuffixes = {
    ".MPH", ".MPC", ".COL", ".CHT", ".CSE", ".DAT", ".SHL", ".GRP",
};

struct TerrainMemberSelection {
    // Selected family's source stem ("MAP", "1", "2", ...).
    std::string stem;
    // Source member names, parallel to kTerrainMemberSuffixes. Empty means the
    // optional member is absent. These always name original archive entries.
    std::array<std::string, kTerrainMemberSuffixes.size()> names{};
    // True only when normalize_terrain_members created MAP aliases.
    bool canonicalized = false;
};

// Select without modifying the archive. The complete canonical MAP family is
// preferred. If no MAP family member exists at all, exactly one complete
// alternate-stem family is required. A partial canonical family is an error,
// because filling it from another stem would create a mixed family. If no
// canonical member exists, multiple complete alternates are ambiguous and
// partial-only candidates are rejected. When `declared_stem` is non-empty (the
// level script's `map` token), that complete family is selected after the
// canonical precedence check. A missing or partial declared family is an error,
// never a reason to fall back to another complete family. Stems are compared
// case-insensitively using the archive's uppercase convention.
// `error` is cleared on success.
bool select_terrain_members(const Archive& archive, TerrainMemberSelection& selection,
                            std::string& error,
                            std::string_view declared_stem = {});

// Select, then copy every present member of an alternate selected family to
// the matching missing MAP.<suffix> name. Only kTerrainMemberSuffixes are
// considered; recordings, scripts, models and textures are never renamed.
// A complete canonical family is left byte-for-byte and key-for-key unchanged.
// Failure leaves `archive` unchanged.
bool normalize_terrain_members(Archive& archive, TerrainMemberSelection& selection,
                               std::string& error,
                               std::string_view declared_stem = {});

} // namespace bigred
