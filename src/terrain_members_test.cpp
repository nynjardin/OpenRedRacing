#include "formats/bigred_archive.h"
#include "formats/terrain_members.h"

#include <algorithm>
#include <cctype>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace {

using bigred::Archive;
using bigred::Blob;
using bigred::TerrainMemberSelection;

int failures = 0;

void Check(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAIL " << message << '\n';
    ++failures;
}

Blob Bytes(std::initializer_list<unsigned> values) {
    Blob out;
    out.reserve(values.size());
    for (unsigned value : values) out.push_back(static_cast<std::uint8_t>(value));
    return out;
}

Archive Family(const std::string& stem, bool optional = false) {
    Archive archive;
    archive.emplace(stem + ".MPH", Bytes({1, 2, 3}));
    archive.emplace(stem + ".MPC", Bytes({4, 5, 6}));
    archive.emplace(stem + ".COL", Bytes({7, 8, 9}));
    if (optional) {
        archive.emplace(stem + ".CHT", Bytes({10}));
        archive.emplace(stem + ".CSE", Bytes({11}));
        archive.emplace(stem + ".DAT", Bytes({12}));
        archive.emplace(stem + ".SHL", Bytes({13}));
        archive.emplace(stem + ".GRP", Bytes({14}));
    }
    return archive;
}

bool Same(const Blob& a, const Blob& b) {
    return a == b;
}

void TestCanonicalUntouched() {
    Archive archive = Family("MAP", true);
    archive.emplace("MAP.TXT", Bytes({99}));
    const Archive before = archive;
    TerrainMemberSelection selection;
    std::string error;
    Check(bigred::normalize_terrain_members(archive, selection, error),
          "canonical family normalizes");
    Check(error.empty(), "canonical family clears error");
    Check(selection.stem == "MAP", "canonical family selected");
    Check(!selection.canonicalized, "canonical family adds no aliases");
    Check(archive == before, "canonical family is byte-for-byte unchanged");
}

void TestAlternateAliasesOnlyKnownMembers() {
    Archive archive = Family("1", true);
    archive.emplace("1.REC", Bytes({90}));
    archive.emplace("1.TXT", Bytes({91}));
    archive.emplace("1.TM", Bytes({92}));
    archive.emplace("TOPMAP.SPR", Bytes({93}));
    const Archive original = archive;

    TerrainMemberSelection selection;
    std::string error;
    Check(bigred::normalize_terrain_members(archive, selection, error),
          "alternate complete family normalizes");
    Check(error.empty(), "alternate family clears error");
    Check(selection.stem == "1", "alternate stem selected");
    Check(selection.canonicalized, "alternate family reports aliases");

    for (const char* suffix : bigred::kTerrainMemberSuffixes) {
        const std::string canonical = std::string("MAP") + suffix;
        const std::string source = std::string("1") + suffix;
        Check(archive.find(canonical) != archive.end(),
              std::string("missing alternate alias ") + canonical);
        Check(Same(archive.at(canonical), original.at(source)),
              std::string("alternate alias copied source ") + source);
        Check(archive.find(source) != archive.end(),
              std::string("source member retained ") + source);
    }
    for (const char* untouched : {"1.REC", "1.TXT", "1.TM", "TOPMAP.SPR"}) {
        Check(archive.at(untouched) == original.at(untouched),
              std::string("unrelated member unchanged ") + untouched);
        Check(archive.find(std::string("MAP") + std::string(untouched).substr(1)) ==
                  archive.end(),
              std::string("unrelated member not aliased ") + untouched);
    }
}

void TestPartialCanonicalRejectedWithoutFallback() {
    Archive archive;
    archive.emplace("MAP.MPH", Bytes({1}));
    archive.emplace("MAP.MPC", Bytes({2}));
    const Archive alternate = Family("1", true);
    archive.insert(alternate.begin(), alternate.end());
    const Archive before = archive;

    TerrainMemberSelection selection;
    std::string error;
    Check(!bigred::normalize_terrain_members(archive, selection, error),
          "partial canonical family rejected");
    Check(error.find("partial canonical") != std::string::npos,
          "partial canonical error is explicit");
    Check(archive == before, "partial canonical rejection is atomic");
}

void TestOptionalCanonicalMemberStillRejectsFallback() {
    Archive archive;
    archive.emplace("MAP.CHT", Bytes({8}));
    const Archive alternate = Family("1", true);
    archive.insert(alternate.begin(), alternate.end());
    const Archive before = archive;

    TerrainMemberSelection selection;
    std::string error;
    Check(!bigred::normalize_terrain_members(archive, selection, error),
          "optional-only canonical family rejects alternate fallback");
    Check(error.find("partial canonical") != std::string::npos &&
              error.find(".MPH") != std::string::npos &&
              error.find(".MPC") != std::string::npos &&
              error.find(".COL") != std::string::npos,
          "optional-only canonical error names every required suffix");
    Check(archive == before, "optional-only canonical rejection is atomic");
}

void TestPartialWithoutCompleteRejected() {
    Archive archive;
    archive.emplace("1.MPH", Bytes({1}));
    archive.emplace("1.MPC", Bytes({2}));
    archive.emplace("2.MPH", Bytes({3}));
    const Archive before = archive;

    TerrainMemberSelection selection;
    std::string error;
    Check(!bigred::normalize_terrain_members(archive, selection, error),
          "partial-only candidates rejected");
    Check(error.find("partial terrain family") != std::string::npos,
          "partial-only error is explicit");
    Check(archive == before, "partial-only rejection is atomic");
}

void TestAmbiguousAlternatesRejected() {
    Archive archive = Family("1", true);
    const Archive second = Family("2", true);
    archive.insert(second.begin(), second.end());
    const Archive before = archive;

    TerrainMemberSelection selection;
    std::string error;
    Check(!bigred::normalize_terrain_members(archive, selection, error),
          "multiple complete alternate families rejected");
    Check(error.find("ambiguous complete terrain families") != std::string::npos,
          "ambiguous-family error is explicit");
    Check(archive == before, "ambiguous rejection is atomic");
}

void TestCanonicalPrecedence() {
    Archive archive = Family("MAP", true);
    const Archive first = Family("1", true);
    const Archive second = Family("2");
    archive.insert(first.begin(), first.end());
    archive.insert(second.begin(), second.end());
    archive.emplace("3.MPH", Bytes({77}));
    const Archive before = archive;

    TerrainMemberSelection selection;
    std::string error;
    Check(bigred::normalize_terrain_members(archive, selection, error),
          "complete canonical family wins");
    Check(selection.stem == "MAP", "canonical precedence reports MAP");
    Check(!selection.canonicalized, "canonical precedence adds no aliases");
    Check(archive == before, "canonical precedence changes nothing");
}

void TestDeclaredStemResolvesAmbiguity() {
    Archive archive = Family("1", true);
    const Archive second = Family("2", true);
    archive.insert(second.begin(), second.end());
    archive.emplace("3.MPH", Bytes({77}));
    const Archive before = archive;

    TerrainMemberSelection selection;
    std::string error;
    Check(bigred::normalize_terrain_members(archive, selection, error, "2"),
          "declared stem resolves multiple complete families");
    Check(selection.stem == "2", "declared stem selects requested family");
    Check(selection.canonicalized, "declared alternate family aliases MAP");
    for (const char* suffix : bigred::kTerrainMemberSuffixes) {
        const std::string canonical = std::string("MAP") + suffix;
        const std::string source = std::string("2") + suffix;
        Check(archive.at(canonical) == before.at(source),
              std::string("declared alias uses requested family ") + source);
    }
}

void TestDeclaredMissingDoesNotFallback() {
    Archive archive = Family("1", true);
    const Archive before = archive;
    TerrainMemberSelection selection;
    std::string error;
    Check(!bigred::normalize_terrain_members(archive, selection, error, "2"),
          "missing declared family does not fall back to another complete family");
    Check(error.find("declared terrain family '2' not found") != std::string::npos,
          "missing declared family error is explicit");
    Check(archive == before, "missing declared family rejection is atomic");
}

void TestDeclaredPartialDoesNotFallback() {
    Archive archive = Family("1", true);
    archive.emplace("2.MPH", Bytes({77}));
    archive.emplace("2.MPC", Bytes({78}));
    const Archive before = archive;
    TerrainMemberSelection selection;
    std::string error;
    Check(!bigred::normalize_terrain_members(archive, selection, error, "2"),
          "partial declared family does not fall back to another complete family");
    Check(error.find("partial declared terrain family '2'") != std::string::npos &&
              error.find(".COL") != std::string::npos,
          "partial declared family error is explicit");
    Check(archive == before, "partial declared family rejection is atomic");
}

void TestDeclaredStemIsCaseInsensitive() {
    Archive archive = Family("RUF");
    TerrainMemberSelection selection;
    std::string error;
    Check(bigred::normalize_terrain_members(archive, selection, error, " ruf "),
          "declared stem is trimmed and uppercased");
    Check(selection.stem == "RUF", "declared stem reports normalized source stem");
    Check(archive.find("MAP.MPH") != archive.end(),
          "case-insensitive declared stem creates MAP aliases");
}

void TestUniqueCompleteWithPartialCandidate() {
    Archive archive = Family("2", true);
    archive.emplace("1.MPH", Bytes({77}));
    TerrainMemberSelection selection;
    std::string error;
    Check(bigred::normalize_terrain_members(archive, selection, error),
          "unique complete family selected beside a partial candidate");
    Check(selection.stem == "2", "unique complete stem selected");
    Check(archive.find("MAP.MPH") != archive.end(),
          "unique complete family aliases MAP.MPH");
    Check(archive.find("1.MPH") != archive.end(),
          "unselected partial candidate retained");
}

void TestOptionalMembersStayInSelectedFamily() {
    Archive archive = Family("1");
    archive.emplace("2.CHT", Bytes({88}));
    archive.emplace("2.GRP", Bytes({89}));

    TerrainMemberSelection selection;
    std::string error;
    Check(bigred::normalize_terrain_members(archive, selection, error),
          "required-only family normalizes beside unrelated optional members");
    Check(archive.find("MAP.MPH") != archive.end(), "required-only alias MPH exists");
    Check(archive.find("MAP.MPC") != archive.end(), "required-only alias MPC exists");
    Check(archive.find("MAP.COL") != archive.end(), "required-only alias COL exists");
    Check(archive.find("MAP.CHT") == archive.end(),
          "unrelated 2.CHT is not aliased into the selected 1 family");
    Check(archive.find("MAP.GRP") == archive.end(),
          "unrelated 2.GRP is not aliased into the selected 1 family");
}

void TestInsertionOrderDoesNotChangeSelection() {
    Archive a;
    const Archive one = Family("1", true);
    const Archive two = Family("2", true);
    a.insert(two.begin(), two.end());
    a.insert(one.begin(), one.end());
    a.emplace("3.MPH", Bytes({3}));

    Archive b;
    b.insert(one.begin(), one.end());
    b.emplace("3.MPH", Bytes({3}));
    b.insert(two.begin(), two.end());

    TerrainMemberSelection sa;
    TerrainMemberSelection sb;
    std::string ea;
    std::string eb;
    Check(!bigred::select_terrain_members(a, sa, ea), "first order remains ambiguous");
    Check(!bigred::select_terrain_members(b, sb, eb), "second order remains ambiguous");
    Check(ea == eb, "ambiguous-family error is deterministic across insertion order");
}

// The archive reader takes UTF-8 paths. Narrow std::filesystem conversions use the ANSI
// code page on Windows, which cannot round-trip an accented install folder.
std::filesystem::path PathFromUtf8(const std::string& utf8) {
    return std::filesystem::path(std::u8string(utf8.begin(), utf8.end()));
}

std::string Utf8(const std::filesystem::path& path) {
    const std::u8string u8 = path.u8string();
    return std::string(u8.begin(), u8.end());
}

std::string UpperExtension(const std::filesystem::path& path) {
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return extension;
}

void TestRetailLevels(const std::filesystem::path& game_dir) {
    std::vector<std::filesystem::path> levels;
    for (const auto& entry : std::filesystem::directory_iterator(game_dir)) {
        if (entry.is_regular_file() && UpperExtension(entry.path()) == ".LEV") {
            levels.push_back(entry.path());
        }
    }
    std::sort(levels.begin(), levels.end());
    Check(levels.size() == 26, "retail install has 26 level archives");

    std::size_t alternate = 0;
    for (const auto& path : levels) {
        Archive archive;
        try {
            archive = bigred::unpack_file(Utf8(path));
        } catch (const std::exception& e) {
            Check(false, path.filename().string() + ": " + e.what());
            continue;
        }
        const std::string stem = path.stem().string();
        const bool alternate_stem =
            stem == "RUF1" || stem == "RUF2" || stem == "RUF3";
        const std::string declared = alternate_stem ? stem.substr(3) : std::string();
        TerrainMemberSelection selection;
        std::string error;
        if (!bigred::normalize_terrain_members(archive, selection, error, declared)) {
            Check(false, path.filename().string() + ": " + error);
            continue;
        }
        for (const char* suffix : bigred::kTerrainMemberSuffixes) {
            Check(archive.find(std::string("MAP") + suffix) != archive.end(),
                  path.filename().string() + " has MAP" + suffix);
        }
        if (alternate_stem) {
            ++alternate;
            Check(selection.stem == declared,
                  stem + " selects declared alternate stem " + declared);
            Check(selection.canonicalized, stem + " reports canonicalization");
            for (const char* suffix : bigred::kTerrainMemberSuffixes) {
                Check(archive.find(declared + suffix) != archive.end(),
                      stem + " retains original " + declared + suffix);
            }
        } else {
            Check(selection.stem == "MAP", stem + " selects canonical MAP");
            Check(!selection.canonicalized, stem + " needs no aliases");
        }
    }
    Check(alternate == 3, "RUF1/2/3 are the three alternate-stem levels");
}

} // namespace

int main(int argc, char** argv) {
    TestCanonicalUntouched();
    TestAlternateAliasesOnlyKnownMembers();
    TestPartialCanonicalRejectedWithoutFallback();
    TestOptionalCanonicalMemberStillRejectsFallback();
    TestPartialWithoutCompleteRejected();
    TestAmbiguousAlternatesRejected();
    TestCanonicalPrecedence();
    TestDeclaredStemResolvesAmbiguity();
    TestDeclaredMissingDoesNotFallback();
    TestDeclaredPartialDoesNotFallback();
    TestDeclaredStemIsCaseInsensitive();
    TestUniqueCompleteWithPartialCandidate();
    TestOptionalMembersStayInSelectedFamily();
    TestInsertionOrderDoesNotChangeSelection();
    // `-` is the build's default install, as for the other tools: a shell mangles an accented
    // folder name on its way through argv.
    if (argc > 1) {
        std::string game_dir = argv[1];
#ifdef OPENBIGRED_GAME_DIR
        if (game_dir == "-") game_dir = OPENBIGRED_GAME_DIR;
#endif
        TestRetailLevels(PathFromUtf8(game_dir));
    }

    if (failures != 0) {
        std::cerr << failures << " terrain-member test(s) failed\n";
        return 1;
    }
    std::cout << "terrain-member synthetic checks passed";
    if (argc > 1) std::cout << "; retail level checks passed";
    std::cout << '\n';
    return 0;
}
