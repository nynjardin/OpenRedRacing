#include "race_level.h"

#include <algorithm>
#include <cstdio>

#include "formats/vehicle_script.h"
#include "scene/level_scene.h"

namespace bigred {

void build_race_level(const LevelScene& scene, const std::string& level, RaceLevel& out,
                      const std::string& game_dir) {
    out = RaceLevel();
    if (auto it = scene.lev.find(level + ".TXT"); it != scene.lev.end())
        out.level_txt.assign(it->second.begin(), it->second.end());
    const std::string& txt = out.level_txt;
    out.cars = parse_car_list(txt);

    SimTerrain& t = out.terrain;
    t.mph = scene.mph_data;
    t.mpc = scene.mpc_data;
    t.cht = (scene.cht_size >= 512) ? scene.cht_data : nullptr;
    t.w = scene.map_w;
    t.h = scene.map_h;
    const std::vector<uint8_t> land = parse_land_table(txt);
    for (size_t i = 0; i < 16 && i < land.size(); ++i) t.land[i] = land[i];
    auto ft_of = [&scene](const std::string& shape) -> const Blob* {
        auto it = scene.lev.find(shape + ".FT");
        return it == scene.lev.end() ? nullptr : &it->second;
    };
    // Ramps, bridges, tunnel floors and platforms - the only scenery the original drives on -
    // and everything driven INTO: trees, fences, walls, posts, the destructible props.
    t.objects = build_sim_objects(scene.instances, scene.shape_names_by_slot, ft_of);
    t.blockers = build_sim_blockers(scene.instances, scene.shape_names_by_slot, ft_of);
    t.build_blocker_grid();

    // The ring (MAP.CSE), the gates no car may be put back on (`unsafe`), and the checkpoints
    // themselves - the `CHECKP` boxes that fire a checkpoint and count a lap (sim/race.h).
    if (auto cse = scene.lev.find("MAP.CSE"); cse != scene.lev.end())
        out.gates = parse_gates(cse->second.data(), cse->second.size());
    mark_unsafe_gates(out.gates, parse_unsafe_gates(txt));
    out.lines = build_check_lines(scene.instances, scene.shape_names_by_slot, ft_of);
    mark_checkpoint_gates(out.gates, out.lines);

    out.grid = parse_grid(txt);
    // Places 6..11, for a network race of more than six, checked with the level's first
    // racing vehicle (FEHELI is the presentation helicopter, never a racer).
    {
        VehicleTuning probe;
        bool have = false;
        if (!game_dir.empty())
            for (const std::string& c : out.cars)
                if (!is_presentation_vehicle(c)) {
                    have = load_vehicle(game_dir, c, probe);
                    break;
                }
        extend_grid(out.grid, out.terrain, have ? &probe : nullptr);
    }
    out.nitros = parse_nitros(txt);
    out.origin.x = scene.map_w * kTerrainTexelWorldSize / 2;
    out.origin.z = scene.map_h * kTerrainTexelWorldSize / 2;
    out.laps = parse_laps(txt);
    out.countdown_ticks = parse_countdown_ticks(txt);
    out.rates = parse_replay_rates(txt);
}

bool build_race_field(const std::string& game_dir, const LevelScene& scene, const RaceLevel& level,
                      const std::vector<FieldHuman>& humans, bool ghosts, RaceField& out) {
    out = RaceField();
    const int n = static_cast<int>(humans.size());
    if (n == 0) return false;
    // Which human stands on which block. Two humans on one block is the caller's bug; the
    // second one moves to the first free block rather than starting inside the first.
    std::vector<int> block_of(static_cast<size_t>(n), -1);
    {
        std::vector<bool> taken(kMaxRacers, false);
        for (int h = 0; h < n; ++h) {
            int g = humans[static_cast<size_t>(h)].grid;
            if (g < 0 || g >= kMaxRacers || taken[static_cast<size_t>(g)]) {
                g = 0;
                while (g < kMaxRacers && taken[static_cast<size_t>(g)]) ++g;
            }
            if (g >= kMaxRacers) return false;   // more humans than blocks
            taken[static_cast<size_t>(g)] = true;
            block_of[static_cast<size_t>(h)] = g;
        }
    }

    // The ghosts first, into a full grid, so we know whether there are any before deciding
    // the size of the field.
    std::vector<ReplayTrack> tracks(kGridRacers);
    std::vector<int> ghost_vehicle(kGridRacers, -1);
    std::vector<int> colours(kGridRacers, 0);
    for (int b = 0; b < kGridRacers; ++b) {
        colours[static_cast<size_t>(b)] = b % 6;
        auto rec = scene.lev.find(std::to_string(b) + ".REC");
        if (rec == scene.lev.end()) continue;
        ReplayTrack t = parse_replay(rec->second, b);
        if (!t.valid()) continue;
        // The block's dot keeps the colour of its recording even when a human takes it, so
        // the six dots stay six colours. (The original moves colours round by index;
        // a human's body colour is a free tint, not one of the six.)
        colours[static_cast<size_t>(b)] = t.colour % 6;
        const bool human_here =
            std::find(block_of.begin(), block_of.end(), b) != block_of.end();
        if (!ghosts || human_here) continue;
        if (t.vehicle_index < 0 || static_cast<size_t>(t.vehicle_index) >= level.cars.size())
            continue;
        ghost_vehicle[static_cast<size_t>(b)] = t.vehicle_index;
        tracks[static_cast<size_t>(b)] = std::move(t);
    }
    int ghost_count = 0;
    for (int b = 0; b < kGridRacers; ++b)
        if (ghost_vehicle[static_cast<size_t>(b)] >= 0) ++ghost_count;

    // No ghost: the humans close up onto the first blocks, keeping their order.
    if (ghost_count == 0) {
        std::vector<int> order(static_cast<size_t>(n));
        for (int h = 0; h < n; ++h) order[static_cast<size_t>(h)] = h;
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
            return block_of[static_cast<size_t>(a)] < block_of[static_cast<size_t>(b)];
        });
        for (int k = 0; k < n; ++k) block_of[static_cast<size_t>(order[static_cast<size_t>(k)])] = k;
    }
    int size = 0;
    for (int b : block_of) size = std::max(size, b + 1);
    if (ghost_count > 0) size = std::max(size, kGridRacers);

    const size_t s = static_cast<size_t>(size);
    out.tunings.assign(s, VehicleTuning());
    out.vhc.assign(s, std::make_shared<const Archive>());
    out.vehicle.assign(s, std::string());
    out.tracks.assign(s, ReplayTrack());
    out.racers.assign(s, RacerSetup());
    out.human.assign(s, -1);
    out.session_of.assign(static_cast<size_t>(n), -1);
    out.colour.assign(s, 0);
    for (size_t b = 0; b < s; ++b) out.colour[b] = b < colours.size() ? colours[b] : static_cast<int>(b % 6);

    for (int h = 0; h < n; ++h) {
        const FieldHuman& fh = humans[static_cast<size_t>(h)];
        const size_t b = static_cast<size_t>(block_of[static_cast<size_t>(h)]);
        std::string name = fh.vehicle;
        if (name.empty()) name = level.cars.empty() ? std::string("JEEP") : level.cars.front();
        if (!load_vehicle(game_dir, name, out.tunings[b], &out.vhc[b])) {
            std::fprintf(stderr, "Could not load vehicle %s.VHC\n", name.c_str());
            return false;
        }
        out.vehicle[b] = name;
        out.racers[b] = RacerSetup{&out.tunings[b], fh.damage_option, nullptr};
        out.human[b] = h;
        out.session_of[static_cast<size_t>(h)] = static_cast<int>(b);
    }
    for (size_t b = 0; b < s && b < ghost_vehicle.size() && ghost_count > 0; ++b) {
        if (ghost_vehicle[b] < 0) continue;
        const std::string& name = level.cars[static_cast<size_t>(ghost_vehicle[b])];
        if (!load_vehicle(game_dir, name, out.tunings[b], &out.vhc[b])) continue;
        out.tracks[b] = std::move(tracks[b]);
        out.vehicle[b] = name;
        // A ghost takes the first human's damage option, as the single race always gave it
        // the player's. Nothing reads it: a replayed car takes no damage of its own.
        out.racers[b] = RacerSetup{&out.tunings[b], humans.front().damage_option, &out.tracks[b]};
        ++out.ghosts;
    }
    return true;
}

} // namespace bigred
