#include "level_catalog.h"

#include <algorithm>
#include <cctype>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include "formats/game_files.h"
#include "formats/level_script.h"
#include "formats/localised_text.h"
#include "formats/tourn.h"

namespace bigred {
namespace {

// `TOURN.WAP`'s own catalogue: which circuits the game offers, in which order, and what each
// one plays. Absent or unreadable is not fatal - the list then keeps its built-in order and no
// circuit asks for a particular piece of music.
// ⚠️ UPPERCASED, because the game's own alphabet has no lowercase at all
// (src/formats/sprite.h) - and the shipped tables mix the two freely: "England" next to
// "CHILE", "Easy - a dirt track..." next to a shouted heading.
std::string upper_of(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

} // namespace

TournCatalog load_tourn_catalog(const std::string& game_dir) {
    const Blob* txt = find_member(*game_archive_or_empty(game_dir, "TOURN.WAP"), "TOURN.TXT");
    if (!txt) return {};
    try {
        return parse_tourn_catalog(std::string(txt->begin(), txt->end()));
    } catch (const std::exception&) {
        return {};
    }
}

int level_max_racers(const std::string& name) {
    // physics_bench's GRILLE block, "N validee(s) par extend_grid", plus the level's six.
    // CHOPPER2: a hill and obstacles behind and beside the grid, no extra place holds a car.
    struct Cap {
        const char* level;
        int racers;
    };
    static const Cap kCaps[] = {
        {"CHOPPER2", 6}, {"MOON2", 9}, {"SNOW3", 11},
        // Extra grid slots have not been measured on the rough-terrain courses.
        {"RUF1", 6}, {"RUF2", 6}, {"RUF3", 6},
    };
    for (const Cap& c : kCaps)
        if (name == c.level) return c.racers;
    return 12;
}

namespace {
std::vector<LevelEntry> scan_levels_uncached(const std::string& game_dir);

// The circuits add_level() declared, per game folder and by name. An ordered map, so the list
// comes out the same on every run.
struct Added {
    LevelEntry entry;
    std::shared_ptr<const LevelFiles> files;
};
std::map<std::string, std::map<std::string, Added>>& added_levels() {
    static std::map<std::string, std::map<std::string, Added>> added;
    return added;
}
const Added* find_added(const std::string& game_dir, const std::string& name) {
    const auto& all = added_levels();
    const auto dir = all.find(game_dir);
    if (dir == all.end()) return nullptr;
    const auto it = dir->second.find(name);
    return it == dir->second.end() ? nullptr : &it->second;
}
} // namespace

std::vector<LevelEntry> scan_levels(const std::string& game_dir) {
    static std::string cached_dir;
    static std::vector<LevelEntry> cached;
    static bool have = false;
    if (!have || cached_dir != game_dir) {
        cached = scan_levels_uncached(game_dir);
        cached_dir = game_dir;
        have = true;
    }
    std::vector<LevelEntry> out = cached;
    if (const auto dir = added_levels().find(game_dir); dir != added_levels().end()) {
        for (const auto& [name, a] : dir->second) {
            out.erase(std::remove_if(out.begin(), out.end(),
                                     [&](const LevelEntry& e) { return e.name == name; }),
                      out.end());
            out.push_back(a.entry);
        }
    }
    return out;
}

void add_level(const std::string& game_dir, LevelEntry entry, LevelFiles files) {
    if (entry.title.empty()) entry.title = entry.name;
    const std::string name = entry.name;
    auto shared = std::make_shared<const LevelFiles>(std::move(files));
    // Its files answer to `<NAME>.LEV` from now on, for every reader (formats/game_files.h).
    add_game_archive(game_dir, name + ".LEV", ArchivePtr(shared, &shared->files));
    added_levels()[game_dir][name] = Added{std::move(entry), std::move(shared)};
}

ArchivePtr level_files(const std::string& game_dir, const std::string& name) {
    return game_archive_or_empty(game_dir, name + ".LEV");
}

void level_map_size(const std::string& game_dir, const std::string& name, int& w, int& h) {
    w = h = 0;
    if (const Added* a = find_added(game_dir, name)) {
        w = a->files->map_w;
        h = a->files->map_h;
    }
}

namespace {
std::vector<LevelEntry> scan_levels_uncached(const std::string& game_dir) {
    // ⚠️ NOT a directory scan. `TEST1.LEV` ships with the game and is a developers' sandbox, and
    // a scan would also pick up whatever the user drops in the folder. This is the shipped set.
    static const char* kLevels[] = {
        "CHOPPER1", "CHOPPER2", "CHOPPER3", "CITY1", "CITY2", "CITY3", "CITY4",
        "DIRT1", "DIRT2", "DIRT3", "MOON1", "MOON2", "MOON3", "RIG1", "RIG3", "RIG4",
        "RUF1", "RUF2", "RUF3", "SNOW1", "SNOW2", "SNOW3", "WATER1", "WATER2", "WATER3",
        "TEST1",
    };

    std::vector<LevelEntry> out;
    for (const char* name : kLevels) {
        ArchivePtr lev_archive;
        try {
            lev_archive = game_archive(game_dir, std::string(name) + ".LEV");
        } catch (const std::exception&) {
            continue; // not installed: silently not on the menu
        }
        const Archive& lev = *lev_archive;
        // game_archive resolves a complete terrain family to MAP.*. RUF1/2/3
        // use the script-declared 1.*, 2.* and 3.* families in the retail files.
        if (lev.find("MAP.MPH") == lev.end() || lev.find("MAP.MPC") == lev.end() ||
            lev.find("MAP.COL") == lev.end()) {
            continue;
        }
        std::string txt;
        auto it = lev.find(std::string(name) + ".TXT");
        if (it != lev.end()) txt.assign(it->second.begin(), it->second.end());
        std::vector<std::string> cars = parse_car_list(txt);
        // ⚠️ FEHELI IS NOT A RACER. Almost every circuit lists it, and it is the PRESENTATION
        // helicopter - no colour variants, no interior, no wreck, which is how
        // it was identified. A level names it so its model gets loaded for
        // the track fly-by, not so anyone can pick it. The UFO is the same thing on the MOON
        // circuits. Kept if it is somehow the only entry.
        {
            std::vector<std::string> racers;
            for (const std::string& c : cars) if (!is_presentation_vehicle(c)) racers.push_back(c);
            if (!racers.empty()) cars = racers;
        }
        if (cars.empty()) continue; // nothing to drive: not a race circuit
        LevelEntry entry;
        entry.name = name;
        entry.max_racers = level_max_racers(name);
        entry.cars = std::move(cars);
        out.push_back(std::move(entry));
    }

    // Its name, its music and its order, from TOURN.TXT and the player's own language.
    const TournCatalog tourn = load_tourn_catalog(game_dir);
    TextTable text;
    text.load(game_dir, config_language(game_dir));
    TextTable english;
    english.load(game_dir, "ENG");
    for (LevelEntry& e : out) {
        e.title = e.name;
        if (const TournTrack* t = tourn.find(e.name)) {
            e.intro_track = t->intro_track;
            e.music_track = t->music_track;
            e.globe_x = t->globe_x;
            e.globe_y = t->globe_y;
            e.planet = t->planet;
            e.name_index = t->name_index;
            e.description = upper_of(text.text(t->view_text));
            e.difficulty = difficulty_of(english.text(t->view_text));
            const std::string place = upper_of(text.text(t->globe_text));
            if (!place.empty()) e.title = place;
        }
    }
    if (!tourn.single_order.empty()) {
        // ⚠️ THE ORDER IS THE GAME'S, not the alphabet's: `TOURN.TXT`'s `single` section lists
        // the circuits in the order a lone player is offered them, easiest first (RIG1, DIRT2,
        // RUF1, ...). Anything it does not name keeps its place at the end.
        const auto rank = [&tourn](const std::string& name) {
            const auto it = std::find(tourn.single_order.begin(), tourn.single_order.end(), name);
            return it == tourn.single_order.end()
                       ? tourn.single_order.size()
                       : static_cast<size_t>(it - tourn.single_order.begin());
        };
        std::stable_sort(out.begin(), out.end(), [&rank](const LevelEntry& a, const LevelEntry& b) {
            return rank(a.name) < rank(b.name);
        });
    }
    return out;
}
} // namespace

} // namespace bigred
