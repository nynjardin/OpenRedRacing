#include "formats/terrain_members.h"

#include <algorithm>
#include <cctype>
#include <map>
#include <set>
#include <string_view>
#include <utility>
#include <vector>

namespace bigred {
namespace {

constexpr std::string_view kCanonicalStem = "MAP";
constexpr std::size_t kRequiredCount = 3; // MPH, MPC, COL

std::string NormalizeDeclaredStem(std::string_view declared) {
    while (!declared.empty() &&
           std::isspace(static_cast<unsigned char>(declared.front()))) {
        declared.remove_prefix(1);
    }
    while (!declared.empty() &&
           std::isspace(static_cast<unsigned char>(declared.back()))) {
        declared.remove_suffix(1);
    }
    std::string normalized;
    normalized.reserve(declared.size());
    for (unsigned char c : declared) {
        normalized.push_back(static_cast<char>(std::toupper(c)));
    }
    return normalized;
}

std::string Join(const std::set<std::string>& names) {
    std::string joined;
    for (const std::string& name : names) {
        if (!joined.empty()) joined += ", ";
        joined += name;
    }
    return joined;
}

bool IsKnownSuffix(std::string_view suffix) {
    return std::find_if(kTerrainMemberSuffixes.begin(), kTerrainMemberSuffixes.end(),
                        [&](const char* known) { return suffix == known; }) !=
           kTerrainMemberSuffixes.end();
}

// Find the suffix boundary without trusting a member name's other dots.
std::size_t StemAndSuffix(const std::string& name, std::string_view& stem,
                          std::string_view& suffix) {
    const std::size_t dot = name.rfind('.');
    if (dot == std::string::npos) {
        stem = {};
        suffix = {};
        return std::string::npos;
    }
    stem = std::string_view(name).substr(0, dot);
    suffix = std::string_view(name).substr(dot);
    return dot;
}

std::array<bool, kTerrainMemberSuffixes.size()> FindFamily(const Archive& archive,
                                                           const std::string& stem) {
    std::array<bool, kTerrainMemberSuffixes.size()> present{};
    for (std::size_t i = 0; i < kTerrainMemberSuffixes.size(); ++i) {
        present[i] = archive.find(stem + kTerrainMemberSuffixes[i]) != archive.end();
    }
    return present;
}

bool Complete(const std::array<bool, kTerrainMemberSuffixes.size()>& present) {
    for (std::size_t i = 0; i < kRequiredCount; ++i) {
        if (!present[i]) return false;
    }
    return true;
}

bool Any(const std::array<bool, kTerrainMemberSuffixes.size()>& present) {
    return std::find(present.begin(), present.end(), true) != present.end();
}

bool AnyRequired(const std::array<bool, kTerrainMemberSuffixes.size()>& present) {
    return std::find(present.begin(), present.begin() + kRequiredCount, true) !=
           present.begin() + kRequiredCount;
}

std::set<std::string> MissingRequired(
    const std::array<bool, kTerrainMemberSuffixes.size()>& present) {
    std::set<std::string> missing;
    for (std::size_t i = 0; i < kRequiredCount; ++i) {
        if (!present[i]) missing.insert(kTerrainMemberSuffixes[i]);
    }
    return missing;
}

void FillSelection(const std::string& stem,
                   const std::array<bool, kTerrainMemberSuffixes.size()>& present,
                   TerrainMemberSelection& selection) {
    selection = {};
    selection.stem = stem;
    for (std::size_t i = 0; i < kTerrainMemberSuffixes.size(); ++i) {
        if (present[i]) selection.names[i] = stem + kTerrainMemberSuffixes[i];
    }
}

} // namespace

bool select_terrain_members(const Archive& archive, TerrainMemberSelection& selection,
                            std::string& error, std::string_view declared_stem) {
    selection = {};
    error.clear();
    const std::string declared = NormalizeDeclaredStem(declared_stem);

    const auto canonical = FindFamily(archive, std::string(kCanonicalStem));
    if (Any(canonical)) {
        if (!Complete(canonical)) {
            const std::set<std::string> missing = MissingRequired(canonical);
            error = "partial canonical terrain family MAP: missing " + Join(missing);
            return false;
        }
        FillSelection(std::string(kCanonicalStem), canonical, selection);
        return true;
    }

    std::map<std::string, std::array<bool, kTerrainMemberSuffixes.size()>> candidates;
    for (const auto& [name, blob] : archive) {
        (void)blob;
        std::string_view stem;
        std::string_view suffix;
        if (StemAndSuffix(name, stem, suffix) == std::string::npos || stem.empty() ||
            !IsKnownSuffix(suffix)) {
            continue;
        }
        auto& present = candidates[std::string(stem)];
        for (std::size_t i = 0; i < kTerrainMemberSuffixes.size(); ++i) {
            if (suffix == kTerrainMemberSuffixes[i]) present[i] = true;
        }
    }

    if (!declared.empty()) {
        const auto found = candidates.find(declared);
        if (found == candidates.end() || !Any(found->second)) {
            error = "declared terrain family '" + declared + "' not found";
            return false;
        }
        if (!Complete(found->second)) {
            const std::set<std::string> missing = MissingRequired(found->second);
            error = "partial declared terrain family '" + declared + "': missing " +
                    Join(missing);
            return false;
        }
        FillSelection(declared, found->second, selection);
        return true;
    }

    std::vector<std::string> complete;
    std::set<std::string> partial_required;
    std::set<std::string> optional_only;
    for (const auto& [stem, present] : candidates) {
        if (Complete(present)) {
            complete.push_back(stem);
        } else if (AnyRequired(present)) {
            partial_required.insert(stem);
        } else if (Any(present)) {
            optional_only.insert(stem);
        }
    }

    if (complete.size() > 1) {
        error = "ambiguous complete terrain families: " + Join(
            std::set<std::string>(complete.begin(), complete.end()));
        return false;
    }
    if (complete.empty()) {
        if (!partial_required.empty()) {
            error = "partial terrain family candidates: " + Join(partial_required);
        } else if (!optional_only.empty()) {
            error = "terrain family candidates lack MPH/MPC/COL: " + Join(optional_only);
        } else {
            error = "no complete terrain family (MPH+MPC+COL)";
        }
        return false;
    }

    FillSelection(complete.front(), candidates.at(complete.front()), selection);
    return true;
}

bool normalize_terrain_members(Archive& archive, TerrainMemberSelection& selection,
                               std::string& error, std::string_view declared_stem) {
    TerrainMemberSelection selected;
    if (!select_terrain_members(archive, selected, error, declared_stem)) {
        selection = {};
        return false;
    }

    if (selected.stem == kCanonicalStem) {
        selected.canonicalized = false;
        selection = std::move(selected);
        return true;
    }

    std::vector<std::pair<std::string, Blob>> aliases;
    aliases.reserve(kTerrainMemberSuffixes.size());
    for (std::size_t i = 0; i < kTerrainMemberSuffixes.size(); ++i) {
        if (selected.names[i].empty()) continue;
        const std::string canonical_name =
            std::string(kCanonicalStem) + kTerrainMemberSuffixes[i];
        if (archive.find(canonical_name) != archive.end()) {
            error = "cannot create " + canonical_name + " from " + selected.names[i] +
                    ": canonical member already exists";
            selection = {};
            return false;
        }
        const auto source = archive.find(selected.names[i]);
        if (source == archive.end()) {
            error = "selected terrain member disappeared: " + selected.names[i];
            selection = {};
            return false;
        }
        aliases.emplace_back(canonical_name, source->second);
    }

    // Build a complete copy first so allocation or insertion failure cannot
    // leave a half-normalized archive behind. This path runs only for the three
    // alternate-stem retail levels, not for canonical MAP families.
    Archive normalized = archive;
    for (auto& [name, blob] : aliases) {
        normalized.emplace(std::move(name), std::move(blob));
    }
    archive = std::move(normalized);
    selected.canonicalized = true;
    selection = std::move(selected);
    return true;
}

} // namespace bigred
