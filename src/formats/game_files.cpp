#include "formats/game_files.h"
#include "formats/terrain_members.h"

#include <algorithm>
#include <cctype>
#include <deque>
#include <iterator>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

namespace bigred {

// A build-time choice (CMake's OPENBIGRED_GAME_DIR), empty unless a developer sets it.
#ifdef OPENBIGRED_GAME_DIR
const char* const kDefaultGameDir = OPENBIGRED_GAME_DIR;
#else
const char* const kDefaultGameDir = "";
#endif

std::string game_path(const std::string& game_dir, const std::string& file) {
    return game_dir + "\\" + file;
}

namespace {

#ifdef _WIN32
std::string to_utf8(const wchar_t* w) {
    const int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    if (n <= 1) return {};
    std::string s(static_cast<size_t>(n), '\0');
    WideCharToMultiByte(CP_UTF8, 0, w, -1, s.data(), n, nullptr, nullptr);
    s.pop_back(); // the terminator
    return s;
}
#endif

// The folder of the running executable, UTF-8, no trailing separator. Empty if unknown.
std::string exe_dir() {
#ifdef _WIN32
    wchar_t buf[MAX_PATH * 4];
    const DWORD n = GetModuleFileNameW(nullptr, buf, static_cast<DWORD>(std::size(buf)));
    if (n == 0 || n >= std::size(buf)) return {};
    std::string path = to_utf8(buf);
    const size_t cut = path.find_last_of("\\/");
    return cut == std::string::npos ? std::string() : path.substr(0, cut);
#else
    return {};
#endif
}

std::string working_dir() {
#ifdef _WIN32
    wchar_t buf[MAX_PATH * 4];
    const DWORD n = GetCurrentDirectoryW(static_cast<DWORD>(std::size(buf)), buf);
    if (n == 0 || n >= std::size(buf)) return {};
    return to_utf8(buf);
#else
    return ".";
#endif
}

std::string parent_of(std::string dir) {
    while (!dir.empty() && (dir.back() == '\\' || dir.back() == '/')) dir.pop_back();
    const size_t cut = dir.find_last_of("\\/");
    // "C:" alone is a drive, not a folder: stop there.
    if (cut == std::string::npos || cut < 2) return {};
    return dir.substr(0, cut);
}

} // namespace

std::string find_game_dir(std::string* tried) {
    std::vector<std::string> bases;
    for (const std::string& start : {exe_dir(), working_dir()}) {
        std::string d = start;
        for (int up = 0; up < 4 && !d.empty(); ++up, d = parent_of(d)) bases.push_back(d);
    }
    std::vector<std::string> candidates;
    for (const std::string& b : bases) {
        candidates.push_back(b);
        candidates.push_back(game_path(b, "BIGRED"));
    }
    candidates.push_back("C:\\BIGRED");
    candidates.push_back(kDefaultGameDir);

    std::vector<std::string> seen;
    for (std::string c : candidates) {
        while (c.size() > 3 && (c.back() == '\\' || c.back() == '/')) c.pop_back();
        if (c.empty() || std::find(seen.begin(), seen.end(), c) != seen.end()) continue;
        seen.push_back(c);
        if (tried) *tried += "  " + c + "\n";
        if (file_exists_utf8(game_path(c, "ALWAYS.WAP"))) return c;
    }
    return {};
}

namespace {

constexpr size_t kLevelsKept = 2;

std::mutex g_mutex;
std::unordered_map<std::string, ArchivePtr> g_archives;
std::deque<std::string> g_levels; // the .LEV paths in g_archives, oldest first
std::unordered_map<std::string, ArchivePtr> g_added; // add_game_archive's, never dropped

bool is_level(const std::string& file) {
    if (file.size() < 4) return false;
    std::string ext = file.substr(file.size() - 4);
    for (char& c : ext) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return ext == ".LEV";
}

std::string declared_map_stem(const Archive& archive, const std::string& file) {
    const auto slash = file.find_last_of("/\\");
    std::string script = file.substr(slash == std::string::npos ? 0 : slash + 1);
    script = script.substr(0, script.find_last_of('.')) + ".TXT";
    for (char& c : script) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    const Blob* bytes = find_member(archive, script);
    if (!bytes) return {};
    std::istringstream lines(std::string(bytes->begin(), bytes->end()));
    std::string line, stem;
    while (std::getline(lines, line)) {
        const auto comment = std::min(line.find(';'), line.find("//"));
        if (comment != std::string::npos) line.erase(comment);
        std::istringstream words(line);
        std::string key, value;
        if (!(words >> key)) continue;
        for (char& c : key) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (key == "map") {
            if (!(words >> value)) throw std::runtime_error(script + ": map needs a member stem");
            stem = std::move(value);
        }
    }
    return stem;
}

} // namespace

ArchivePtr game_archive(const std::string& game_dir, const std::string& file) {
    const std::string path = game_path(game_dir, file);
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (auto it = g_added.find(path); it != g_added.end()) return it->second;
        if (auto it = g_archives.find(path); it != g_archives.end()) return it->second;
    }
    // Unpacked outside the lock: two threads asking for the same archive at once both unpack
    // it and the second one's copy is dropped, which costs time but not correctness.
    Archive members = unpack_file(path);
    if (is_level(file)) {
        TerrainMemberSelection terrain;
        std::string error;
        if (!normalize_terrain_members(members, terrain, error, declared_map_stem(members, file)))
            throw std::runtime_error(file + ": " + error);
    }
    auto archive = std::make_shared<const Archive>(std::move(members));
    std::lock_guard<std::mutex> lock(g_mutex);
    auto [it, inserted] = g_archives.emplace(path, archive);
    if (inserted && is_level(file)) {
        g_levels.push_back(path);
        if (g_levels.size() > kLevelsKept) {
            g_archives.erase(g_levels.front());
            g_levels.pop_front();
        }
    }
    return it->second;
}

void add_game_archive(const std::string& game_dir, const std::string& file, ArchivePtr archive) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_added[game_path(game_dir, file)] = std::move(archive);
}

ArchivePtr game_archive_or_empty(const std::string& game_dir, const std::string& file) {
    try {
        return game_archive(game_dir, file);
    } catch (const std::exception&) {
        static const ArchivePtr empty = std::make_shared<const Archive>();
        return empty;
    }
}

const Blob* find_member(const Archive& archive, const std::string& name) {
    auto it = archive.find(name);
    return it == archive.end() ? nullptr : &it->second;
}

} // namespace bigred
