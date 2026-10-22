// physics_bench - a windowless bench for the vehicle simulation.
//
// The handling model can only be judged two ways: by the user driving it, and by MEASURING it.
// This is the second. It is a real target rather than a scratch file because the first version
// of it was written into a temporary directory and lost, and the next session had to
// reconstruct every measurement from scratch.
//
// Usage: physics_bench [game_dir] [LEVEL] [VEHICLE] [--bloc=NAME[,NAME...]]
//        defaults WATER1 / the level's first car / every block. A block's NAME is the one of its
//        `// --- NAME ---` marker below (REST, SESSION, INSTANTANE, SOUND...); an unknown name
//        is an error, so a typo cannot pass for "nothing to report".
//
// Three measurements, each answering a question that came out of a user report:
//
//   REST    does a vehicle left alone come to a dead stop? A position that moves at zero
//           velocity means the push-back is injecting motion.
//   SLOPES  on ground of increasing steepness, does the body settle at the terrain's own angle,
//           and do its four bottom corners end up on the surface? A body resting on ONE corner
//           makes the righting torque fire every tick, which is what sends it into a somersault
//           off the next ramp.
//   JUMP    drive flat out across the map and watch the pitch rate. In the original the angular
//           rates are FROZEN in the air and the ground is what rights the vehicle,
//           so what matters is the rate at take-off.
//   CAMERA  the chase camera is on the same tick as the physics, so it belongs on the same
//           bench. How far back does it actually sit at speed, how far behind the car does it
//           swing in a turn, and does it stay over the ground? All three are the numbers the
//           user is judging when they say the framing feels wrong.
//   TERRAIN does the ground the renderer DRAWS agree with the ground the car drives on? Sampled
//           at the centre of every emitted triangle, because the corners agree whichever way the
//           cell is split and only the inside tells them apart. Must stay at 0.
//   NITRO   the boost is not only a flame: what it is worth in speed, how long it lasts, how much
//           of it the level grants, and the fact that it holds the throttle and locks the brake.
//   SOUND   every `hnd fx` line of the vehicle, resolved against the files on disk. No audio
//           device is opened: the question is whether the DATA lines up, which is where the
//           failures are - a sample in the wrong folder is silent and looks like nothing.
//   RAMP-GATE  is a drivable wedge a floor or a wall? A ramp is both, and which one it is is
//           decided per tick by a height test (SimObject::ramp_gate). The block walks the
//           profile with no integration, then drives at the thing from each end: the LIFT says
//           whether the body was teleported to the top, the SPEED whether it was stopped.
//   DEGATS  what one full-speed frontal impact costs, as a percentage of the damage gauge, and
//           how many of them the car survives. The user's yardstick for the damage model.
//   SPOT    the ground effects. Which `spot` lines the environment declares, whether their
//           flipbooks baked, and - driving for real - whether anything is ever emitted. A puff of
//           dust is gated on the surface type AND on a wheel touching the ground, so "it compiles
//           and nothing appears" is the expected failure and the only one worth measuring.
//
#include "formats/level_script.h"
#include "formats/sprite.h"
#include "formats/vehicle_script.h"
#include "scene/chase_camera.h"
#include "scene/dashboard.h"
#include "formats/threedw.h"
#include "scene/level_scene.h"
#include "formats/texture.h" // kTmWidth/kTmHeight, for reporting the flame crops in texels
#include "audio/sound_bank.h"
#include "audio/vehicle_audio.h"
#include "scene/particle_render.h"
#include "scene/vehicle_render.h"
#include "sim/fixed.h"
#include "sim/lockstep.h"
#include "sim/race.h"
#include "sim/race_session.h"
#include "sim/snapshot.h"
#include "game/championship.h"
#include "game/level_catalog.h"
#include "game/split_layout.h"
#include "scene/camera_obstruction.h"
#include "scene/character_model.h"
#include "formats/localised_text.h"
#include "sim/replay.h"
#include "sim/vehicle.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <array>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

// Degrees, from the BAM24 the body angles use and the BAM16 the terrain angles use.
double deg24(int32_t a) { return a * 360.0 / 16777216.0; }
double deg16(int32_t a) { return a * 360.0 / 65536.0; }

// The steepest height step across a cell, in MAP.MPH height bytes - which is exactly what the
// slope table is indexed by, so it is the right unit for this bench.
int cell_step(const bigred::SimTerrain& t, int gx, int gz) {
    auto h = [&](int x, int z) {
        return static_cast<int>(t.mph[static_cast<size_t>(z & 0xFF) * t.w + (x & 0xFF)]);
    };
    const int c = h(gx, gz);
    int worst = 0;
    worst = std::max(worst, std::abs(h(gx + 1, gz) - c));
    worst = std::max(worst, std::abs(h(gx, gz + 1) - c));
    worst = std::max(worst, std::abs(h(gx + 1, gz + 1) - c));
    worst = std::max(worst, std::abs(h(gx - 1, gz) - c));
    return worst;
}

// A 24-bit BMP of `w` x `h` RGB triplets, top row first - for the blocks whose answer is a
// picture (DASH, GLYPHES). No PNG encoder is linked into the bench.
void write_bmp(const std::string& path, int w, int h, const std::vector<uint8_t>& rgb) {
    const size_t row = (static_cast<size_t>(w) * 3 + 3) & ~static_cast<size_t>(3);
    std::vector<uint8_t> f(54 + row * static_cast<size_t>(h), 0);
    const auto put32 = [&](size_t at, uint32_t x) {
        for (int k = 0; k < 4; ++k) f[at + k] = static_cast<uint8_t>(x >> (8 * k));
    };
    f[0] = 'B'; f[1] = 'M';
    put32(2, static_cast<uint32_t>(f.size())); put32(10, 54); put32(14, 40);
    put32(18, static_cast<uint32_t>(w)); put32(22, static_cast<uint32_t>(h));
    f[26] = 1; f[28] = 24;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const size_t s = (static_cast<size_t>(y) * w + x) * 3;
            const size_t o = 54 + static_cast<size_t>(h - 1 - y) * row + static_cast<size_t>(x) * 3;
            f[o] = rgb[s + 2]; f[o + 1] = rgb[s + 1]; f[o + 2] = rgb[s];
        }
    bigred::write_file_utf8(path, f);
}

void settle(bigred::VehicleSim& car, int ticks) {
    const bigred::VehicleInput idle;
    for (int i = 0; i < ticks; ++i) car.step(idle);
}

// --bloc: the blocks asked for (none = all of them), and every name the run has checked, so
// that main() can refuse one that does not exist.
std::set<std::string> g_blocs_wanted;
std::set<std::string> g_blocs_known;

bool bloc(const char* name) {
    g_blocs_known.insert(name);
    return g_blocs_wanted.empty() || g_blocs_wanted.count(name) != 0;
}

} // namespace

int main(int argc, char* argv[]) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a.rfind("--bloc=", 0) != 0) {
            args.push_back(a);
            continue;
        }
        for (size_t at = 7; at <= a.size();) {
            const size_t comma = std::min(a.find(',', at), a.size());
            if (comma > at) g_blocs_wanted.insert(a.substr(at, comma - at));
            at = comma + 1;
        }
    }
    // "-" means "the default install": the path has a non-ASCII character in it, and passing it
    // through a shell mangles it, so a placeholder is the reliable way to reach the later
    // arguments.
    const std::string game_dir =
        (args.size() > 0 && args[0] != "-") ? args[0] : bigred::find_game_dir();
    const std::string level = (args.size() > 1) ? args[1] : "WATER1";
    std::string vehicle_name = (args.size() > 2) ? args[2] : "";

    bigred::LevelScene scene;
    // BENCH_DECOR=verbose: the scene's own load traces too - which `.TM` each shape asked for.
    if (const char* d = std::getenv("BENCH_DECOR"); d && std::string(d) == "verbose")
        bigred::g_verbose_logs = true;
    // BENCH_SHAPE=NAME: the scene's per-shape dump (faces, groups, textures, vertices) - the
    // same trace as map_preview's --diag=NAME, without a window.
    if (const char* d = std::getenv("BENCH_SHAPE")) bigred::g_diag_shape_filter = d;
    // BENCH_NOLINES=1: the decor without its tag=11 lines, to compare against.
    if (std::getenv("BENCH_NOLINES")) bigred::g_decor_lines = false;
    if (!scene.load(game_dir, level)) {
        std::fprintf(stderr, "level load failed\n");
        return 1;
    }

    // DECOR (BENCH_DECOR=1): every shape the level places, and what the scene turned it into -
    // flat triangles, textured faces, billboards. A shape placed N times that yields none of the
    // three is decor the player cannot see. Then exit: it is a question about the level, not
    // about a vehicle.
    if (std::getenv("BENCH_DECOR")) {
        std::vector<int> placed(scene.shape_names_by_slot.size(), 0);
        std::map<int, int> categories;
        for (const bigred::SceneryInstance& in : scene.instances)
            if (in.shape_index >= 0 && static_cast<size_t>(in.shape_index) < placed.size())
                ++placed[static_cast<size_t>(in.shape_index)];
        const auto dat = scene.lev.find("MAP.DAT");
        std::printf("=== DECOR %s === %zu instance(s) of %zu record(s) in MAP.DAT (%zu bytes), "
                    "%zu shape slot(s)\n",
                    level.c_str(), scene.instances.size(),
                    dat == scene.lev.end() ? size_t{0} : dat->second.size() / 22,
                    dat == scene.lev.end() ? size_t{0} : dat->second.size(), placed.size());
        for (size_t s = 0; s < placed.size(); ++s) {
            const size_t tris = s < scene.shapes_by_slot.size() ? scene.shapes_by_slot[s].size() : 0;
            const size_t tex = s < scene.shape_tex_by_slot.size() ? scene.shape_tex_by_slot[s].faces.size() : 0;
            const size_t spr = s < scene.shape_branches_by_slot.size()
                                   ? scene.shape_branches_by_slot[s].sprites.size() : 0;
            const bool empty = s < scene.shape_empty_by_slot.size() && scene.shape_empty_by_slot[s];
            std::string cats;
            for (const bigred::SceneryInstance& in : scene.instances)
                if (in.shape_index == static_cast<int>(s)) {
                    const std::string c = std::to_string(in.collision_category_byte);
                    if (cats.find(" " + c + " ") == std::string::npos && cats.size() < 40)
                        cats += " " + c + " ";
                }
            // How far over the terrain cell under it each instance stands (MAP.DAT's y is -up):
            // a large negative is a shape buried in the ground.
            int lo = 1 << 30, hi = -(1 << 30);
            for (const bigred::SceneryInstance& in : scene.instances) {
                if (in.shape_index != static_cast<int>(s) || !scene.mph_data) continue;
                const int gx = std::max(0, std::min(scene.map_w - 1, static_cast<int>(in.x) / 512));
                const int gz = std::max(0, std::min(scene.map_h - 1, static_cast<int>(in.z) / 512));
                const int ground = scene.mph_data[static_cast<size_t>(gz) * scene.map_w + gx] * 16;
                const int over = static_cast<int>(-in.y) - ground;
                lo = std::min(lo, over);
                hi = std::max(hi, over);
            }
            // Its tag=11 lines, straight from the file: drawn as thin bars since 2026-09-28.
            size_t nlines = 0;
            if (auto it = scene.lev.find(scene.shape_names_by_slot[s] + ".3DW"); it != scene.lev.end())
                nlines = bigred::parse_3dw(it->second.data(), it->second.size()).lines.size();
            if (nlines > 0) std::printf("    lines %zu\n", nlines);
            if (placed[s] > 0) std::printf("    over ground %d..%d\n", lo, hi);
            // Draped over the ground (place_scenery): flat-coloured and entirely flat. Only the
            // rigid categories are - the conforming ones follow their own probes.
            if (tris > 0) {
                const auto& sh = scene.shapes_by_slot[s];
                bool flat = true;
                for (const auto& t : sh)
                    if (std::fabs(t.y + t.ly - (sh.front().y + sh.front().ly)) > 0.5f) { flat = false; break; }
                if (flat) std::printf("    flat (draped where rigid)\n");
            }
            std::printf("%3zu %-10s x%-4d tris %-5zu tex %-4zu spr %-3zu%s  cat[%s]%s\n", s,
                        scene.shape_names_by_slot[s].c_str(), placed[s], tris, tex, spr,
                        empty ? " EMPTY" : "", cats.c_str(),
                        placed[s] > 0 && tris == 0 && tex == 0 && spr == 0 && !empty ? "  <== INVISIBLE" : "");
        }
        // The destructibles, per shape: how many vertices the intact state and the last step of
        // the destruction draw, summed over its instances (the wreck buffer's own ranges).
        {
            std::map<std::string, std::array<long, 3>> w;   // pieces, intact, last step
            for (const bigred::LevelScene::WreckPiece& wp : scene.wrecks) {
                const bigred::SceneryInstance& in = scene.instances[static_cast<size_t>(wp.instance)];
                std::array<long, 3>& a = w[scene.shape_names_by_slot[static_cast<size_t>(in.shape_index)]];
                ++a[0];
                a[1] += wp.intact.second;
                if (!wp.steps.empty()) a[2] += wp.steps.back().second;
            }
            for (const auto& kv : w)
                std::printf("wreck %-10s x%-4ld intact %-6ld last step %ld\n", kv.first.c_str(),
                            kv.second[0], kv.second[1], kv.second[2]);
            std::printf("world vbuf %zu vertices\n", scene.vbuf.size());
            // The start lights: which instances, and each of the four states' vertex count.
            std::printf("start lights %zu instance(s):", scene.start_lights.size());
            for (int i : scene.start_lights)
                std::printf(" %s", scene.shape_names_by_slot[static_cast<size_t>(
                                       scene.instances[static_cast<size_t>(i)].shape_index)].c_str());
            std::printf("  states");
            for (const auto& r : scene.lights_ranges) std::printf(" %d", r.second);
            std::printf("\n");
        }
        // The shapes the archive ships that MAP.SHL never names: drawn, if at all, by something
        // other than the placement - a script line, a group, a vehicle.
        std::vector<std::string> unplaced;
        for (const auto& kv : scene.lev) {
            const std::string& n = kv.first;
            if (n.size() < 5 || n.compare(n.size() - 4, 4, ".3DW") != 0) continue;
            const std::string stem = n.substr(0, n.size() - 4);
            if (std::find(scene.shape_names_by_slot.begin(), scene.shape_names_by_slot.end(), stem) ==
                scene.shape_names_by_slot.end())
                unplaced.push_back(stem);
        }
        std::sort(unplaced.begin(), unplaced.end());
        std::printf("not in MAP.SHL:");
        for (const std::string& n : unplaced) std::printf(" %s", n.c_str());
        std::printf("\n");
        for (const std::string& n : unplaced) {
            const bigred::Blob& b = scene.lev.at(n + ".3DW");
            const bigred::ThreeDW m = bigred::parse_3dw(b.data(), b.size());
            std::printf("  %s: %zu vertices, %zu/%d faces, %zu sprites, %zu lines, %zu crops, "
                        "texture '%s', clean end %d\n",
                        n.c_str(), m.vertices.size(), m.faces.size(), m.expected_face_count,
                        m.sprites.size(), m.lines.size(), m.texture_variants.size(),
                        m.embedded_texture_name.c_str(), m.reached_clean_end ? 1 : 0);
        }
        // BENCH_RAW=NAME: that shape's `.3DW`, int32 by int32, eight to a line with the offset
        // of the first - the display list as the parser sees it.
        if (const char* raw = std::getenv("BENCH_RAW")) {
            auto it = scene.lev.find(std::string(raw) + ".3DW");
            if (it == scene.lev.end()) {
                std::printf("%s.3DW: not in this level's archive\n", raw);
            } else {
                const bigred::Blob& b = it->second;
                const size_t n = b.size() / 4;
                for (size_t i = 0; i < n; ++i) {
                    int32_t v;
                    std::memcpy(&v, b.data() + i * 4, 4);
                    if (i % 8 == 0) std::printf("\n%4zu:", i);
                    std::printf(" %7d", v);
                }
                std::printf("\n");
            }
        }
        if (std::string(std::getenv("BENCH_DECOR")) == "txt") {
            // The level script whole: what it asks for besides the placement.
            auto it = scene.lev.find(level + ".TXT");
            if (it != scene.lev.end())
                std::fwrite(it->second.data(), 1, it->second.size(), stdout);
            std::printf("\n--- archive:");
            for (const auto& kv : scene.lev) std::printf(" %s", kv.first.c_str());
            std::printf("\n");
        }
        return 0;
    }

    std::string level_txt;
    {
        auto it = scene.lev.find(level + ".TXT");
        if (it != scene.lev.end()) level_txt.assign(it->second.begin(), it->second.end());
    }
    const std::vector<std::string> cars = bigred::parse_car_list(level_txt);
    const bigred::NitroReserve nitros = bigred::parse_nitros(level_txt);
    if (vehicle_name.empty()) vehicle_name = cars.empty() ? std::string("JEEP") : cars.front();

    bigred::VehicleTuning tuning;
    bigred::ArchivePtr vhc;
    if (!bigred::load_vehicle(game_dir, vehicle_name, tuning, &vhc)) {
        std::fprintf(stderr, "vehicle %s load failed\n", vehicle_name.c_str());
        return 1;
    }

    bigred::SimTerrain ter;
    ter.mph = scene.mph_data;
    ter.mpc = scene.mpc_data;
    ter.cht = (scene.cht_size >= 512) ? scene.cht_data : nullptr;
    ter.w = scene.map_w;
    ter.h = scene.map_h;
    // BENCH_NOAIRCAP=1: the whole bench with deliberate deviation 3 off, as `-NOAIRCAP` does
    // in the game - to compare the JUMP / LANDING blocks with and without it.
    // The rules the bench runs under (sim/race_rules.h): the game's defaults, and the
    // variants asked for below. Every car and every session of the bench is given them.
    bigred::RaceRules rules;
    if (std::getenv("BENCH_DEV2")) {
        rules.righting_tolerance = true;
        std::printf("(ancien ecart assume n 2 REMIS : BENCH_DEV2)\n");
    }
    if (const char* v = std::getenv("BENCH_AIRCAP")) {
        rules.air_rate_cap_value = std::atoi(v);
        std::printf("(borne en l'air de l'ecart n 3 : %d, BENCH_AIRCAP)\n", rules.air_rate_cap_value);
    }
    if (const char* v = std::getenv("BENCH_AIRROLL")) {
        rules.air_roll_cap_value = std::atoi(v);
        std::printf("(borne en l'air du ROULIS : %d, BENCH_AIRROLL)\n", rules.air_roll_cap_value);
    }
    if (const char* v = std::getenv("BENCH_AIRDAMP")) {
        rules.air_damp_shift = std::atoi(v);
        std::printf("(amortissement en l'air 1/2^%d par tick, BENCH_AIRDAMP)\n", rules.air_damp_shift);
    }
    if (const char* v = std::getenv("BENCH_AIRDAMP_ROLL")) {
        rules.air_roll_damp_shift = std::atoi(v);
        std::printf("(amortissement du ROULIS en l'air 1/2^%d, BENCH_AIRDAMP_ROLL)\n", rules.air_roll_damp_shift);
    }
    if (std::getenv("BENCH_NOAIRCAP")) {
        rules.air_rate_cap = false;
        std::printf("(ecart assume n 3 DESACTIVE : BENCH_NOAIRCAP)\n");
    }
    {
        const std::vector<uint8_t> land = bigred::parse_land_table(level_txt);
        for (size_t i = 0; i < 16 && i < land.size(); ++i) ter.land[i] = land[i];
    }
    ter.objects = bigred::build_sim_objects(
        scene.instances, scene.shape_names_by_slot,
        [&scene](const std::string& shape) -> const bigred::Blob* {
            auto it = scene.lev.find(shape + ".FT");
            return it == scene.lev.end() ? nullptr : &it->second;
        });
    ter.blockers = bigred::build_sim_blockers(
        scene.instances, scene.shape_names_by_slot,
        [&scene](const std::string& shape) -> const bigred::Blob* {
            auto it = scene.lev.find(shape + ".FT");
            return it == scene.lev.end() ? nullptr : &it->second;
        });
    ter.build_blocker_grid();

    // FIXCAM - the circuit's trackside cameras (`fixcam`, formats/level_script.h): how many,
    // and where each sits against the ground under it. The finished players watch the race
    // through them (RaceScreen), so a camera buried in the terrain would show nothing.
    if (bloc("FIXCAM")) {
        const std::vector<bigred::FixedCamera> cams = bigred::parse_fixed_cameras(level_txt);
        int under = 0;
        int32_t lo = 1 << 30, hi = -(1 << 30);
        for (const bigred::FixedCamera& c : cams) {
            const int32_t ground = ter.height_fx(c.x << 9, c.z << 9) >> 9;
            const int32_t above = ground - c.y;   // +Y down: positive = above the ground
            lo = std::min(lo, above);
            hi = std::max(hi, above);
            if (above < 0) ++under;
        }
        std::printf("FIXCAM   %zu camera(s) ; hauteur au-dessus du sol %d .. %d ; sous le sol : %d\n",
                    cams.size(), cams.empty() ? 0 : lo, cams.empty() ? 0 : hi, under);
        for (size_t k = 0; k < cams.size() && k < 3; ++k)
            std::printf("         %-24s %6d %6d %6d\n", cams[k].name.c_str(), cams[k].x, cams[k].y,
                        cams[k].z);
        std::printf("\n");
    }

    const bigred::StartingGrid start_grid = bigred::parse_grid(level_txt);
    bigred::RespawnPoint grid = bigred::grid_slot(start_grid, 1, 0);
    if (!start_grid.present) {
        grid.x = scene.map_w * 512 / 2;
        grid.z = scene.map_h * 512 / 2;
    }

    std::printf("=== %s / %s === %zu drivable object(s), %zu blocking one(s)\n", level.c_str(),
                vehicle_name.c_str(), ter.objects.size(), ter.blockers.size());
    std::printf("    body box x %d..%d z %d..%d, query radius %d\n\n", tuning.box.xmin,
                tuning.box.xmax, tuning.box.zmin, tuning.box.zmax, tuning.box.radius);

    // --- OBJECTS ------------------------------------------------------------------------
    // What the level's drivable objects actually ARE, by shape. Without this the count alone
    // says nothing: 98 objects on WATER1 could be 98 ramps or 98 tree roots, and which one it
    // is changes what a handling report means.
    if (bloc("OBJECTS")) {
        std::map<std::string, std::pair<int, std::string>> by_shape;
        for (const bigred::SceneryInstance& inst : scene.instances) {
            const uint8_t raw = inst.collision_category_byte;
            const bool drivable = raw == 0x34 || raw == 0xf6 || raw == 0x80 || raw == 0x81 ||
                                  raw == 0x83 || raw == 0x84 || raw == 0xf8 || raw == 0xf9 ||
                                  raw == 0xfa || raw == 0xfb;
            if (!drivable) continue;
            if (inst.shape_index < 0 ||
                static_cast<size_t>(inst.shape_index) >= scene.shape_names_by_slot.size())
                continue;
            const std::string& name = scene.shape_names_by_slot[inst.shape_index];
            auto& e = by_shape[name];
            ++e.first;
            char buf[32];
            std::snprintf(buf, sizeof buf, "cat=%02X", raw);
            e.second = buf;
        }
        std::printf("OBJECTS  drivable instances by shape\n");
        for (const auto& kv : by_shape)
            std::printf("  %-12s x%-4d %s\n", kv.first.c_str(), kv.second.first,
                        kv.second.second.c_str());
        // The SLAB family (the scenery pass's cases 2, 7, 0xf, 0x15, 0x16, 0x18, 0x21) is not ported:
        // bridges and decks the car can neither drive on nor hit. Listed so a report about one
        // of them can be traced to it.
        {
            std::map<std::string, std::pair<int, int>> slabs;
            for (const bigred::SceneryInstance& inst : scene.instances) {
                const int idx = bigred::collision_category_index(inst.collision_category_byte);
                if (idx != 2 && idx != 7 && idx != 0xf && idx != 0x15 && idx != 0x16 &&
                    idx != 0x18 && idx != 0x21)
                    continue;
                if (inst.shape_index < 0 ||
                    static_cast<size_t>(inst.shape_index) >= scene.shape_names_by_slot.size())
                    continue;
                auto& e = slabs[scene.shape_names_by_slot[inst.shape_index]];
                ++e.first;
                e.second = idx;
            }
            for (const auto& kv : slabs)
                std::printf("  slab %-10s x%-4d cat %d\n", kv.first.c_str(),
                            kv.second.first, kv.second.second);
            // Each slab instance: where it is, the slab of Y it fills, and the terrain under it
            // and around it - to tell a bridge deck from a roof.
            for (const bigred::SimObject& o : ter.blockers) {
                if (!o.slab) continue;
                const auto& inst = scene.instances[static_cast<size_t>(o.instance_index)];
                const std::string& nm = scene.shape_names_by_slot[static_cast<size_t>(inst.shape_index)];
                auto ground = [&](int32_t x, int32_t z) {
                    return ter.height_fx(x << 9, z << 9) >> 9;
                };
                std::printf("    %-9s at (%d,%d,%d) yaw &%04X  Y %d..%d  r %d  %s  ground %d "
                            "(+-800: %d %d %d %d)\n",
                            nm.c_str(), o.x, o.y, o.z, o.yaw_bam16 & 0xFFFF, o.y + o.y_top,
                            o.y + o.y_bottom, o.radius,
                            o.use_box ? "box" : "silhouette", ground(o.x, o.z),
                            ground(o.x + 800, o.z), ground(o.x - 800, o.z),
                            ground(o.x, o.z + 800), ground(o.x, o.z - 800));
            }
        }
        // Every drivable object whose category has its own logic (0x12/0x1a - RIG1's WOODBRIJ):
        // its whole drivable profile, raw.
        for (const bigred::SimObject& o : ter.objects) {
            if (!o.bridge) continue;
            const auto& inst = scene.instances[static_cast<size_t>(o.instance_index)];
            std::printf("  bridge %-9s at (%d,%d,%d) yaw &%04X x %d..%d  half-depth %d/%d  "
                        "0x28/2a %d/%d  deck 0x2c/2e %d/%d  flags %02X  0x32 %d  r %d  ground %d\n",
                        scene.shape_names_by_slot[static_cast<size_t>(inst.shape_index)].c_str(),
                        o.x, o.y, o.z, o.yaw_bam16 & 0xFFFF, o.x_low, o.x_high, o.depth_low,
                        o.depth_high, o.roof_height, o.roof_slope, o.height, o.slope, o.flags,
                        o.past_z_lift, o.radius, ter.height_fx(o.x << 9, o.z << 9) >> 9);
        }
        int plat = 0;
        for (const bigred::SimObject& o : ter.objects) {
            if (!o.platform) continue;
            if (plat++ >= 3) break;
            std::printf("  platform at (%d,%d,%d) lift=%d radius=%d\n", o.x, o.y, o.z,
                        o.platform_lift, o.radius);
        }
        std::printf("\n");
    }

    // --- RACE ---------------------------------------------------------------------------
    // The checkpoint ring, the grid and the countdown - everything that turns driving into a
    // race, measured without a window. Three things are worth reading here:
    //
    //   * the ring is WATERTIGHT. Every gate must overlap the next one, or a car can slip
    //     between them and stop progressing. The walk below drives the ring by its own anchors
    //     and reports any gate the search cannot reach from its predecessor.
    //   * the grid slots are ON the track, i.e. inside some gate.
    //   * the countdown, in seconds, because that is the number the player feels.
    if (bloc("RACE")) {
        std::vector<bigred::Gate> gates;
        auto cse = scene.lev.find("MAP.CSE");
        if (cse != scene.lev.end())
            gates = bigred::parse_gates(cse->second.data(), cse->second.size());
        const std::vector<int> unsafe = bigred::parse_unsafe_gates(level_txt);
        bigred::mark_unsafe_gates(gates, unsafe);
        const int laps = bigred::parse_laps(level_txt);
        const int cd = bigred::parse_countdown_ticks(level_txt);
        // The `CHECKP` lines - what FIRES a checkpoint (the line handler, reached from the collision
        // router's cases 10 and 14). The gates only place the car.
        std::vector<bigred::CheckLine> check_lines = bigred::build_check_lines(
            scene.instances, scene.shape_names_by_slot,
            [&](const std::string& shape) -> const bigred::Blob* {
                auto it = scene.lev.find(shape + ".FT");
                return it == scene.lev.end() ? nullptr : &it->second;
            });
        bigred::mark_checkpoint_gates(gates, check_lines);
        int last_ordinal = -1;
        for (const bigred::CheckLine& c : check_lines) last_ordinal = std::max(last_ordinal, c.ordinal());
        std::printf("RACE     %zu gate(s), %d lap(s), countdown %d ticks = %.2f s, %zu unsafe%s",
                    gates.size(), laps, cd, cd * bigred::kGameTickMs / 1000.0, unsafe.size(),
                    unsafe.empty() ? "" : " (");
        for (size_t i = 0; i < unsafe.size(); ++i)
            std::printf("%s%d%s", i ? "," : "", unsafe[i], i + 1 == unsafe.size() ? ")" : "");
        std::printf("\n");
        std::printf("  %zu ligne(s) CHECKP, numeros d'ordre 0..%d : %d checkpoint(s) par tour + "
                    "la ligne d'arrivee\n",
                    check_lines.size(), last_ordinal, last_ordinal);

        // Each line as the original reads it: the variant byte (offset 19) is
        // the ORDINAL, bit 0x80 on the lap's last one; offset 21 goes to the
        // vehicle's gate window. `porte` is the gate it is attributed to, `~` when no gate contains
        // it and the nearest one stands in.
        {
            if (!check_lines.empty()) {
                const bigred::CheckLine& c = check_lines.front();
                std::printf("  boite CHECKP.FT : x %d..%d  z %d..%d  sommet %d\n", c.box_xmin,
                            c.box_xmax, c.box_zmin, c.box_zmax, c.y_top);
            }
            for (const bigred::CheckLine& c : check_lines)
                std::printf("  CHECKP (%6d,%6d) cap=&%04X  ordre=%3d%s  %s  octet21=%3d  porte %d%s\n",
                            c.x, c.z, static_cast<unsigned>(c.yaw_bam16), c.ordinal(),
                            c.last() ? " (dernier)" : "          ",
                            c.height_gated ? "hauteur" : "       ", c.window, c.gate,
                            c.in_gate ? "" : "~");
        }

        // Gate 0 is the start/finish line. Print the head of the ring so a wrong decode shows up
        // as nonsense coordinates rather than as a lap that never completes.
        for (size_t i = 0; i < gates.size() && i < 4; ++i) {
            const bigred::Gate& g = gates[i];
            // `len` is the span ACROSS the track and `half` the half-depth ALONG it - see
            // race.h. Printed with those names so the table cannot be misread again.
            std::printf("  gate %2zu at (%6d,%6d) cap=%4d largeur=%5d demi-profondeur=%5d%+d %s\n",
                        i, g.x, g.z, g.angle, g.length, g.half_depth, g.half_depth_delta,
                        g.safe ? "" : "UNSAFE");
        }

        // Watertight? Consecutive gates must overlap, or a car can slip between them and stop
        // progressing.
        //
        // ⚠️ MEASURED ON THE ROAD CENTRE, NOT ON THE ANCHOR. A gate's anchor is its
        // across-track ORIGIN - one verge - so it sits off the road by half the gate's width,
        // and asking whether the next anchor is inside this gate answers a question about the
        // verges. The point that matters is the middle of the road, which is exactly what
        // gate_respawn returns.
        //
        // What comes out of it is that the ring is a TILING, not a chain of overlapping
        // corridors: consecutive slabs meet face to face, within a car's length, all the way
        // round every circuit.
        int holes = 0;
        std::vector<int32_t> gaps;
        gaps.reserve(gates.size());
        for (size_t i = 0; i < gates.size(); ++i) {
            const bigred::Gate& next = gates[(i + 1) % gates.size()];
            const bigred::RespawnPoint mid_self = bigred::gate_respawn(gates[i], 0x2000);
            const bigred::RespawnPoint mid_next = bigred::gate_respawn(next, 0x2000);
            const double dx = static_cast<double>(mid_next.x) - mid_self.x;
            const double dz = static_cast<double>(mid_next.z) - mid_self.z;
            const int32_t reach = (gates[i].half_depth + gates[i].half_depth_delta / 2) +
                                  (next.half_depth + next.half_depth_delta / 2);
            const int32_t gap = static_cast<int32_t>(std::sqrt(dx * dx + dz * dz)) - reach;
            gaps.push_back(gap);
            if (gap > 0) ++holes;
        }
        std::sort(gaps.begin(), gaps.end());
        std::printf("  pavage : ecart entre faces consecutives median %d, min %d, max %d "
                    "(>0 = trou) ; %d paire(s) ne se touchent pas\n",
                    gaps[gaps.size() / 2], gaps.front(), gaps.back(), holes);

        // The grid, and whether its slots land on the ring.
        const bigred::StartingGrid sg = bigred::parse_grid(level_txt);
        std::printf("  grid origin (%d,%d) heading &%04X, %s\n", sg.x, sg.z,
                    static_cast<unsigned>(sg.yaw_bam16 & 0xFFFF),
                    sg.present ? "present" : "ABSENT");
        for (int racers : {2, 6}) {
            for (int slot = 0; slot < racers; ++slot) {
                const bigred::RespawnPoint r = bigred::grid_slot(sg, racers, slot);
                int in_gate = -1;
                int32_t across_track = 0;
                for (size_t g = 0; g < gates.size(); ++g) {
                    const bigred::GateProbe hit = bigred::probe_gate(gates[g], r.x, r.z);
                    if (hit.inside) {
                        in_gate = static_cast<int>(g);
                        across_track = hit.across_track;
                        break;
                    }
                }
                std::printf("  %d-up slot %d at (%6d,%6d) yaw &%04X -> gate %2d (a %d unites "
                            "en travers de la piste)\n",
                            racers, slot, r.x, r.z, static_cast<unsigned>(r.yaw_bam16), in_gate,
                            across_track);
            }
        }

        // And the ring's own heading against the grid's: they must agree at the start line, or
        // the field is pointing the wrong way. The gate angle is a 1024-index a quarter turn off
        // the vehicle's BAM16 yaw - the original respawn's `(angle << 6) + 0x4000`.
        if (!gates.empty()) {
            const int32_t gate_yaw = bigred::gate_yaw_bam16(gates[0]);
            const int32_t diff = ((gate_yaw - (sg.yaw_bam16 & 0xFFFF)) + 0x8000) & 0xFFFF;
            std::printf("  gate 0 yaw &%04X vs grid &%04X - %.1f deg apart\n",
                        static_cast<unsigned>(gate_yaw),
                        static_cast<unsigned>(sg.yaw_bam16 & 0xFFFF),
                        (diff - 0x8000) * 360.0 / 65536.0);
        }

        // Does the RING alone hold together? Walk a probe through the middle of every gate in
        // order, twice round: every gate must register, and the wrap past gate 0 must count.
        // The `CHECKP` lines are left out of this one on purpose - they are what counts a lap
        // on a level that has them, and a probe TELEPORTED from gate to gate never touches a
        // line 100 units thick. This is the ring on its own, i.e. the whole of CITY3 and RUF*.
        if (!gates.empty()) {
            bigred::RaceState race;
            race.configure(gates, laps, 1, 1);   // a one-tick countdown: start straight away
            bigred::VehicleSim probe;
            probe.reset(tuning, ter, rules, grid, bigred::NitroReserve{});
            const bigred::VehicleSim* field[1] = {&probe};
            race.tick(field, 1);                 // burns the countdown
            int crossings = 0;
            for (int turn = 0; turn < 2; ++turn) {
                for (size_t i = 0; i < gates.size(); ++i) {
                    const bigred::RespawnPoint at = bigred::gate_respawn(gates[i], 0x2000);
                    probe.px = static_cast<int64_t>(at.x) << 9;
                    probe.pz = static_cast<int64_t>(at.z) << 9;
                    race.tick(field, 1);
                    if (race.progress(0).gate_crossed) ++crossings;
                }
            }
            const bigred::RacerProgress& r = race.progress(0);
            std::printf("  anneau seul, deux tours : %d portes enregistrees sur %zu pas, "
                        "%d tour(s) comptes, porte finale %d/%zu\n",
                        crossings, gates.size() * 2, r.lap, r.reached, gates.size() - 1);
        }

        // THE LAP, DRIVEN. A probe rolls from the grid round the ring - road centre to road
        // centre, 128 units a step, facing where it goes, on the ground - touching the `CHECKP`
        // lines on the way exactly as a car does. That is the only way to exercise them: they
        // are 100 units thick, and nothing that jumps from gate to gate ever meets one.
        //
        // `skip_ordinal` >= 0 lifts the probe over every piece of that line (the steps within
        // 1 500 units of one are not taken), which is what cutting a corner past a checkpoint
        // does. ALT is asked 2 500 units after the first such jump.
        struct Drive {
            int beeps = 0, missed = 0, lap_events = 0, laps = 0, off_ring = 0, steps = 0;
            int grid_gate = -1, alt_gate = -1;
            std::string trace;   // "ordinal@gate" per event, "!" before a missed one
        };
        auto drive = [&](int rings, int skip_ordinal) {
            Drive d;
            const bigred::StartingGrid sg2 = bigred::parse_grid(level_txt);
            const bigred::RespawnPoint start = bigred::grid_slot(sg2, 1, 0);
            bigred::RaceState race;
            race.configure(gates, laps, 1, 1, check_lines);
            bigred::VehicleSim probe;
            probe.reset(tuning, ter, rules, start, bigred::NitroReserve{});
            probe.px = static_cast<int64_t>(start.x) << 9;
            probe.pz = static_cast<int64_t>(start.z) << 9;
            const bigred::VehicleSim* field[1] = {&probe};
            race.tick(field, 1);   // brule le decompte, et voit la voiture sur la grille
            d.grid_gate = race.progress(0).gate;
            // ⚠️ LE TRAJET PART DE LA GRILLE ET VA VERS L'AVANT - dans la direction de la
            // grille, pas vers « la porte de la grille + 1 ». Sur CITY1 la grille est sous un
            // CROISEMENT : la recherche la trouve dans la porte 16, qui est le pont au-dessus, et
            // filer vers la 17 traversait le decor en touchant les lignes 3 et 4 hors ordre. La
            // premiere porte visee est celle dont le milieu de route est le plus proche d'un
            // point 3 000 unites devant la grille.
            const int n_gates = static_cast<int>(gates.size());
            int first = 0;
            {
                const double a = static_cast<double>(start.yaw_bam16 & 0xFFFF) * 6.283185307179586 / 65536.0;
                const double ax = start.x + 3000.0 * std::sin(a), az = start.z + 3000.0 * std::cos(a);
                double best = -1.0;
                for (int k = 0; k < n_gates; ++k) {
                    const bigred::RespawnPoint m =
                        bigred::gate_respawn(gates[static_cast<size_t>(k)], 0x2000);
                    const double e = (m.x - ax) * (m.x - ax) + (m.z - az) * (m.z - az);
                    if (best < 0 || e < best) { best = e; first = k; }
                }
            }
            std::vector<std::pair<int32_t, int32_t>> path;
            path.emplace_back(start.x, start.z);
            for (int k = 0; k <= rings * n_gates; ++k) {
                const bigred::RespawnPoint m =
                    bigred::gate_respawn(gates[static_cast<size_t>((first + k) % n_gates)], 0x2000);
                path.emplace_back(m.x, m.z);
            }
            int since_skip = -1;
            for (size_t k = 1; k < path.size(); ++k) {
                const int32_t x0 = path[k - 1].first, z0 = path[k - 1].second;
                const int32_t dx = path[k].first - x0, dz = path[k].second - z0;
                const int steps = std::max(
                    1, static_cast<int>(std::sqrt(static_cast<double>(dx) * dx +
                                                  static_cast<double>(dz) * dz) / 128.0));
                probe.yaw = (bigred::fx::atan2_bam16(dx, dz) & 0xFFFF) << 8;
                for (int s = 1; s <= steps; ++s) {
                    const int32_t x = x0 + static_cast<int32_t>(static_cast<int64_t>(dx) * s / steps);
                    const int32_t z = z0 + static_cast<int32_t>(static_cast<int64_t>(dz) * s / steps);
                    bool lifted = false;
                    if (skip_ordinal >= 0) {
                        for (const bigred::CheckLine& c : check_lines) {
                            if (c.ordinal() != skip_ordinal) continue;
                            const int64_t ex = x - c.x, ez = z - c.z;
                            if (ex * ex + ez * ez < 1500LL * 1500LL) lifted = true;
                        }
                    }
                    if (lifted) {
                        if (since_skip < 0) since_skip = 0;
                        continue;
                    }
                    probe.px = static_cast<int64_t>(x) << 9;
                    probe.pz = static_cast<int64_t>(z) << 9;
                    probe.py = ter.valid() ? ter.height_fx(probe.px, probe.pz) : probe.py;
                    race.tick(field, 1);
                    ++d.steps;
                    const bigred::RacerProgress& p = race.progress(0);
                    if (p.gate < 0) ++d.off_ring;
                    if (p.checkpoint_crossed) ++d.beeps;
                    if (p.missed_check_now) ++d.missed;
                    // The sequence of lines as the probe meets them - what makes a failure
                    // readable. Only printed for a plain drive that went wrong.
                    if ((p.checkpoint_crossed || p.missed_check_now || p.lap_crossed) &&
                        d.trace.size() < 400) {
                        char buf[64];
                        std::snprintf(buf, sizeof buf, " %s%d@%d", p.missed_check_now ? "!" : "",
                                      p.last_touched & 0x7f, p.gate);
                        d.trace += buf;
                    }
                    if (p.lap_crossed) ++d.lap_events;
                    // ALT asked AT THE MOMENT THE BANNER GOES UP - the next line has just been
                    // touched out of order, which is when a player presses it. Asked any
                    // earlier, the skipped line is still close behind and every rule finds it;
                    // asked here, on a circuit with few lines, it can be more than half the
                    // ring back (user, 2026-09-23: "ALT me ramène derrière le checkpoint qui
                    // affiche MISSED, pas au checkpoint manqué").
                    if (since_skip >= 0 && d.alt_gate < 0 && p.missed_check_now)
                        d.alt_gate = race.respawn_gate(0);
                }
            }
            d.laps = race.progress(0).lap;
            return d;
        };

        // Un tour conduit : la ligne d'arrivee est franchie une fois au depart (ce qui ARME le
        // tour 1, sans rien compter), puis une fois par anneau. Deux anneaux = deux tours, et
        // chaque checkpoint sonne une fois par tour.
        if (!gates.empty()) {
            const Drive d = drive(2, -1);
            if (check_lines.empty()) {
                std::printf("  tour conduit : pas de ligne CHECKP, l'anneau seul compte - %d tour(s) "
                            "sur deux anneaux, %d/%d pas hors anneau\n",
                            d.laps, d.off_ring, d.steps);
            } else {
                std::printf("  tour conduit depuis la grille (porte %d) : %d tour(s) (attendu 2), "
                            "%d bip(s) (attendu %d), %d CHECKPOINT RATE (attendu 0), "
                            "%d/%d pas hors anneau\n",
                            d.grid_gate, d.laps, d.beeps, 2 * last_ordinal, d.missed, d.off_ring,
                            d.steps);
                if (d.laps != 2 || d.missed != 0 || d.beeps != 2 * last_ordinal)
                    std::printf("    lignes rencontrees (ordre@porte, ! = hors ordre) :%s\n",
                                d.trace.c_str());
            }
        }

        // --- RATER UN CHECKPOINT -------------------------------------------------------------
        //
        // Sauter CHAQUE ligne a tour de role, d'un bond, sur trois anneaux : aucun tour ne doit
        // compter, la banniere doit se lever (la ligne suivante est touchee hors ordre), et ALT
        // - demande au moment ou la banniere se leve - doit ramener devant la ligne sautee.
        // Toutes les lignes et pas seulement la 1 : c'est sur une ligne eloignee de la suivante
        // que la regle du demi-anneau cedait.
        if (!gates.empty() && last_ordinal >= 1) {
            const int n_gates = static_cast<int>(gates.size());
            int ok = 0;
            for (int k = 1; k <= last_ordinal; ++k) {
                const Drive d = drive(3, k);
                // Where "devant la ligne k" is: the gate that contains it, or one gate before
                // the nearest one when no gate does - RaceState::expected_line_gate's own rule.
                int want = -1;
                for (const bigred::CheckLine& c : check_lines) {
                    if (c.ordinal() != k || c.gate < 0) continue;
                    if (c.in_gate) { want = c.gate; break; }
                    if (want < 0) want = (c.gate - 1 + n_gates) % n_gates;
                }
                while (want >= 0 && !gates[static_cast<size_t>(want)].safe)
                    want = (want - 1 + n_gates) % n_gates;
                const bool good = d.laps == 0 && d.missed >= 1 && d.alt_gate == want;
                if (good) ++ok;
                else
                    std::printf("  ligne %d sautee : %d tour(s) comptes (attendu 0), %d CHECKPOINT "
                                "RATE (attendu >=1), ALT ramene porte %d (attendu %d)\n",
                                k, d.laps, d.missed, d.alt_gate, want);
            }
            std::printf("  lignes sautees une a une : %d/%d correctes (0 tour, banniere, ALT devant "
                        "la ligne sautee)\n", ok, last_ordinal);
        } else if (!gates.empty()) {
            // CITY3 et les RUF : pas une seule ligne `CHECKP`. Un tour n'y demande que la ligne
            // d'arrivee de l'anneau, et c'est correct - il n'y a rien a rater.
            std::printf("  aucun checkpoint a rater : ce circuit n'a pas de ligne `CHECKP`\n");
        }

        // Et le sens interdit : on recule le long de l'anneau. Le seuil est 140 ticks de 14 ms,
        // soit 70 pas de simulation - la banniere doit tomber peu apres.
        if (gates.size() > 5) {
            const int n_gates = static_cast<int>(gates.size());
            {
                bigred::RaceState back;
                back.configure(gates, laps, 1, 1);
                bigred::VehicleSim rev;
                rev.reset(tuning, ter, rules, grid, bigred::NitroReserve{});
                const bigred::VehicleSim* rfield[1] = {&rev};
                for (int k = 0; k < 6; ++k) {   // quelques portes vers l'avant d'abord
                    const bigred::RespawnPoint m =
                        bigred::gate_respawn(gates[static_cast<size_t>(k % n_gates)], 0x2000);
                    rev.px = static_cast<int64_t>(m.x) << 9;
                    rev.pz = static_cast<int64_t>(m.z) << 9;
                    back.tick(rfield, 1);
                }
                int ticks_to_warn = -1;
                for (int t = 0; t < 200 && ticks_to_warn < 0; ++t) {
                    // Sur place, en marche arriere dans la porte : la latérale decroit.
                    const bigred::RespawnPoint m = bigred::gate_respawn(
                        gates[static_cast<size_t>(5)], 0x2000 - std::min(0x1f00, 32 * t));
                    rev.px = static_cast<int64_t>(m.x) << 9;
                    rev.pz = static_cast<int64_t>(m.z) << 9;
                    back.tick(rfield, 1);
                    if (back.progress(0).wrong_way_now) ticks_to_warn = t + 1;
                }
                std::printf("  sens interdit : leve apres %d pas de simulation (attendu ~%d)\n",
                            ticks_to_warn, bigred::kWrongWayTicks / 2);
            }
            {
                // A l'arret, jamais : une voiture garee (celle qui a fini, apres la ligne) ne
                // recule pas. L'original sort sans toucher au compteur quand la paire
                // (porte, laterale) n'a pas bouge (utilisateur, 2026-09-30).
                bigred::RaceState park;
                park.configure(gates, laps, 1, 1);
                bigred::VehicleSim still;
                still.reset(tuning, ter, rules, grid, bigred::NitroReserve{});
                const bigred::VehicleSim* sfield[1] = {&still};
                const bigred::RespawnPoint m = bigred::gate_respawn(gates[3], 0x2000);
                still.px = static_cast<int64_t>(m.x) << 9;
                still.pz = static_cast<int64_t>(m.z) << 9;
                bool raised = false;
                for (int t = 0; t < 400; ++t) {
                    park.tick(sfield, 1);
                    raised = raised || park.progress(0).wrong_way;
                }
                std::printf("  a l'arret 400 pas : sens interdit %s\n",
                            raised ? "LEVE (FAUX)" : "jamais (bon)");
            }
        }

        std::printf("\n");
    }

    // --- MINIMAP ------------------------------------------------------------------------
    // TOPMAP.SPR's projection, as the original computes it and carried by the glyph header - see
    // formats/sprite.h, TopmapProjection. The test that the reading is right: every gate's road
    // centre must land on a DRAWN pixel of the map. It goes through topmap_pixel() itself, the
    // function the HUD draws with, so what passes here is what is on screen.
    if (bloc("MINIMAP")) {
        auto it = scene.lev.find("TOPMAP.SPR");
        const std::vector<bigred::SpriteGlyph> tm =
            it == scene.lev.end() ? std::vector<bigred::SpriteGlyph>()
                                  : bigred::decode_spr(it->second.data(), it->second.size());
        if (tm.empty() || !tm[0].valid()) {
            std::printf("MINIMAP  pas de TOPMAP.SPR\n\n");
        } else {
            const bigred::SpriteGlyph& g0 = tm[0];
            const bigred::TopmapProjection proj = bigred::topmap_projection(g0);
            std::printf("MINIMAP  %zu glyphe(s), %dx%d, centre case A=%d B=%d, echelle=%d "
                        "(%d unites monde par pixel)\n", tm.size(), g0.width, g0.height,
                        proj.a, proj.b, proj.scale, proj.scale * 4);
            std::vector<bigred::Gate> gts;
            if (auto cse = scene.lev.find("MAP.CSE"); cse != scene.lev.end())
                gts = bigred::parse_gates(cse->second.data(), cse->second.size());
            int on = 0, in = 0;
            for (const bigred::Gate& gt : gts) {
                const bigred::RespawnPoint m = bigred::gate_respawn(gt, 0x2000);
                int px = 0, py = 0;
                bigred::topmap_pixel(proj, m.x, m.z, px, py);
                if (px < 0 || py < 0 || px >= g0.width || py >= g0.height) continue;
                ++in;
                bool hit = false;   // drawn within one pixel: the outline is thin
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int qx = px + dx, qy = py + dy;
                        if (qx >= 0 && qy >= 0 && qx < g0.width && qy < g0.height &&
                            g0.pixels[static_cast<size_t>(qy) * g0.width + qx] != 0)
                            hit = true;
                    }
                if (hit) ++on;
            }
            std::printf("  %d porte(s) sur %zu dans l'image, %d sur un pixel dessine\n\n", in,
                        gts.size(), on);
        }
    }

    // --- GRILLE -------------------------------------------------------------------------
    // "The car starts in the MIDDLE of the starting blocks, not on the first one." The painted
    // `S_GRID` marks are the only ground truth for where a place is, so this block prints them
    // and every grid slot in the SAME frame: the grid's own, `along` forward the way the grid
    // heads (X is the SINE axis in this engine), `across` to the right of that.
    if (bloc("GRILLE")) {
        bigred::StartingGrid sgr = bigred::parse_grid(level_txt);
        bigred::extend_grid(sgr, ter, &tuning);
        const double a = static_cast<double>(sgr.yaw_bam16 & 0xFFFF) * 6.283185307179586 / 65536.0;
        const double fx = std::sin(a), fz = std::cos(a);
        auto local = [&](double x, double z, double& along, double& across) {
            const double dx = x - sgr.x, dz = z - sgr.z;
            along = dx * fx + dz * fz;
            across = dx * fz - dz * fx;
        };
        std::printf("GRILLE   origine (%d,%d) cap &%04X %s\n", sgr.x, sgr.z,
                    static_cast<unsigned>(sgr.yaw_bam16 & 0xFFFF), sgr.present ? "" : "ABSENTE");
        for (size_t si = 0; si < scene.shape_names_by_slot.size(); ++si) {
            const std::string& nm = scene.shape_names_by_slot[si];
            if (nm != "S_GRID" && nm != "START" && nm != "S_LINES") continue;
            for (const bigred::SceneryInstance& in : scene.instances) {
                if (in.shape_index != static_cast<int>(si)) continue;
                double al, ac;
                local(in.x, in.z, al, ac);
                if (std::fabs(al) > 8000 || std::fabs(ac) > 8000) continue;
                std::printf("  %-8s avant %7.0f  travers %7.0f  y %6.0f  cap &%04X var %d\n",
                            nm.c_str(), al, ac, in.y,
                            static_cast<unsigned>(static_cast<uint16_t>(in.angle_y)),
                            in.variant_index);
            }
        }
        for (int racers : {1, 2}) {
            const int n = racers == 2 ? 2 : 6;
            for (int slot = 0; slot < n; ++slot) {
                const bigred::RespawnPoint r = bigred::grid_slot(sgr, racers == 2 ? 2 : 6, slot);
                double al, ac;
                local(r.x, r.z, al, ac);
                std::printf("  place %d/%d avant %7.0f  travers %7.0f  cap &%04X  sol %d\n", slot,
                            n, al, ac, static_cast<unsigned>(r.yaw_bam16),
                            ter.height_fx(r.x << 9, r.z << 9) >> 9);
            }
        }
        // TWELVE PLACES: the network game's, 6..11 being the level's grid continued backwards
        // (grid_slot). Each one is judged three ways: its ground against place 0's, the nearest
        // obstacle, and a car put on it and left 60 ticks with no input - a good place leaves
        // it where it was put.
        {
            const int32_t g0 = ter.height_fx(bigred::grid_slot(sgr, 12, 0).x << 9,
                                             bigred::grid_slot(sgr, 12, 0).z << 9) >> 9;
            int bad = 0;
            for (int slot = 0; slot < 12; ++slot) {
                const bigred::RespawnPoint r = bigred::grid_slot(sgr, 12, slot);
                double al, ac;
                local(r.x, r.z, al, ac);
                const int32_t g = ter.height_fx(r.x << 9, r.z << 9) >> 9;
                double nearest = 1e9;
                for (const bigred::SimObject& o : ter.blockers) {
                    const double dx = o.x - r.x, dz = o.z - r.z;
                    nearest = std::min(nearest, std::sqrt(dx * dx + dz * dz));
                }
                bigred::VehicleSim car;
                car.reset(tuning, ter, rules, r, bigred::NitroReserve{});
                car.place_on_grid();
                const int32_t x0 = car.world_x(), y0 = car.world_y(), z0 = car.world_z();
                for (int t = 0; t < 60; ++t) car.step(bigred::VehicleInput{});
                const double moved = std::sqrt(
                    std::pow(car.world_x() - x0, 2.0) + std::pow(car.world_y() - y0, 2.0) +
                    std::pow(car.world_z() - z0, 2.0));
                const bool ok = moved < 40.0 && nearest > 200.0 && std::abs(g - g0) < 300;
                if (!ok && slot >= 6) ++bad;
                std::printf("  12 places : %2d avant %6.0f travers %5.0f  sol %+5d  obstacle a %5.0f"
                            "  derive %4.0f%s%s\n",
                            slot, al, ac, g - g0, nearest, moved, slot >= 6 ? "  (prolongee)" : "",
                            ok ? "" : "  <-- A REVOIR");
            }
            std::printf("  places prolongees a revoir : %d / 6, %d validee(s) par extend_grid\n", bad, sgr.extra_count);
        }
        // "The car starts sunk into the ground." Put it on place 0 the way the race does, then
        // let the physics run with no input: a body that starts right should not move, and the
        // underside of its box (`y + ybottom`, +Y down) should sit on the ground from tick 0.
        {
            bigred::VehicleSim car;
            car.reset(tuning, ter, rules, bigred::grid_slot(sgr, 6, 0), bigred::NitroReserve{});
            car.place_on_grid();
            auto under = [&] {
                return car.world_y() + tuning.box.ybottom -
                       (ter.height_fx(static_cast<int32_t>(car.px), static_cast<int32_t>(car.pz)) >> 9);
            };
            const int32_t y0 = car.world_y(), gap0 = under();
            for (int t = 0; t < 60; ++t) car.step(bigred::VehicleInput{});
            std::printf("  pose sur la place 0 : dessous de caisse - sol = %d (0 = pose), apres 60 "
                        "ticks sans commande %d, le corps a bouge de %d en Y (ybottom %d)\n",
                        gap0, under(), car.world_y() - y0, tuning.box.ybottom);
        }
        std::printf("\n");
    }

    // --- SESSION ------------------------------------------------------------------------
    // The common ground of the opponents and the network game: a
    // full grid of six in one RaceSession. Three questions, each a failure that would only show
    // with several cars:
    //   * DETERMINISM - two sessions fed the same inputs, one directly and one through a
    //     LockstepInputs with a 3-tick delay whose packets arrive out of order and late, must
    //     agree on checksum() at EVERY tick. The first tick they do not is printed;
    //   * the GRID - nobody may start inside anybody else;
    //   * the CONTACTS - cars that push each other must end up apart, not inside one another.
    if (bloc("SESSION") || bloc("REPRISE") || bloc("CHAMPIONNAT") || bloc("PILOTES") || bloc("SPLIT") || bloc("INSTANTANE")) {
        std::vector<bigred::Gate> sg_gates;
        if (auto cse = scene.lev.find("MAP.CSE"); cse != scene.lev.end())
            sg_gates = bigred::parse_gates(cse->second.data(), cse->second.size());
        bigred::mark_unsafe_gates(sg_gates, bigred::parse_unsafe_gates(level_txt));
        std::vector<bigred::CheckLine> sg_lines = bigred::build_check_lines(
            scene.instances, scene.shape_names_by_slot,
            [&](const std::string& shape) -> const bigred::Blob* {
                auto it = scene.lev.find(shape + ".FT");
                return it == scene.lev.end() ? nullptr : &it->second;
            });
        bigred::mark_checkpoint_gates(sg_gates, sg_lines);
        const bigred::StartingGrid sg = bigred::parse_grid(level_txt);
        bigred::RespawnPoint origin;
        origin.x = scene.map_w * 512 / 2;
        origin.z = scene.map_h * 512 / 2;
        constexpr int kRacers = bigred::kGridRacers;
        const std::vector<bigred::RacerSetup> field(kRacers, bigred::RacerSetup{&tuning, 30});
        bigred::SceneryState wa, wb;
        wa.resize(scene.instances.size());
        wb.resize(scene.instances.size());
        // The script: everyone floors it, and steers in bursts chosen by a per-player LCG.
        auto script = [](int player, uint32_t t) {
            bigred::VehicleInput in;
            in.accel = true;
            uint32_t s = 2654435761u * (player + 1) + (t / 20) * 40503u;
            s ^= s >> 13;
            s *= 2246822519u;
            s ^= s >> 16;
            in.left = (s % 3) == 0;
            in.right = (s % 3) == 1;
            in.nitro = (s % 17) == 0;
            return in;
        };
        bigred::RaceSession a, b;
        // Twice: the one-sided contact of a race against the ghosts (the car contact's normal
        // mode), then the two-body one of a race between humans (modes 0x20/0x23).
        for (int two_body = 0; two_body < 2 && bloc("SESSION"); ++two_body) {
        wa = bigred::SceneryState();
        wb = bigred::SceneryState();
        wa.resize(scene.instances.size());
        wb.resize(scene.instances.size());
        a.rules = rules;
        a.configure(ter, &wa, sg_gates, bigred::parse_laps(level_txt),
                    bigred::parse_countdown_ticks(level_txt), sg_lines, sg, nitros, field, origin);
        b.rules = rules;
        b.configure(ter, &wb, sg_gates, bigred::parse_laps(level_txt),
                    bigred::parse_countdown_ticks(level_txt), sg_lines, sg, nitros, field, origin);
        a.two_body_contacts = b.two_body_contacts = (two_body != 0);

        auto touching = [&](const bigred::RaceSession& s, int i, int j) {
            const int64_t dx = s.car(i).world_x() - s.car(j).world_x();
            const int64_t dz = s.car(i).world_z() - s.car(j).world_z();
            const int64_t r = tuning.box.radius * 2;
            return dx * dx + dz * dz < r * r / 4;   // centres closer than one radius: inside
        };
        int start_overlaps = 0;
        for (int i = 0; i < kRacers; ++i)
            for (int j = i + 1; j < kRacers; ++j)
                if (touching(a, i, j)) ++start_overlaps;

        bigred::LockstepInputs lk;
        lk.configure(kRacers, 3);
        const int kTicks = 1500;
        int first_diff = -1, stalls = 0, contacts = 0, bangs = 0;
        int contact_by[kRacers] = {};
        int run[kRacers] = {}, longest = 0, longest_car = -1, longest_end = 0;
        int32_t longest_speed = 0;
        for (int t = 1; t <= kTicks; ++t) {
            // What each player sends this tick is for tick t + delay. Players arrive in a
            // different order every tick, and player 5's packet is always one tick late.
            const uint32_t target = lk.schedule(static_cast<uint32_t>(t));
            for (int k = 0; k < kRacers; ++k) {
                const int p = (k + t) % kRacers;
                if (p == 5) continue;
                lk.submit(p, target, bigred::pack_input(script(p, target)));
            }
            if (t > 1) lk.submit(5, target - 1, bigred::pack_input(script(5, target - 1)));
            lk.submit(5, target - 1, 0xFF);   // a resent packet: must be ignored
            if (t == 1) lk.submit(5, target - 1, bigred::pack_input(script(5, target - 1)));

            bigred::VehicleInput direct[kRacers], viaq[kRacers];
            for (int p = 0; p < kRacers; ++p)
                direct[p] = (t > lk.delay()) ? script(p, static_cast<uint32_t>(t))
                                             : bigred::VehicleInput();
            if (!lk.ready(static_cast<uint32_t>(t))) { ++stalls; continue; }
            lk.inputs_for(static_cast<uint32_t>(t), viaq);
            lk.release_before(static_cast<uint32_t>(t));
            a.tick(direct);
            b.tick(viaq);
            if (first_diff < 0 && a.checksum() != b.checksum()) first_diff = t;
            for (int i = 0; i < kRacers; ++i) {
                if (a.car(i).vehicle_hit_sound >= 0) ++bangs;
                if (a.car(i).vehicle_hit >= 0) {
                    ++contacts;
                    ++contact_by[i];
                    if (++run[i] > longest) {
                        longest = run[i];
                        longest_car = i;
                        longest_end = t;
                        longest_speed = a.car(i).speed_per_second();
                    }
                } else {
                    run[i] = 0;
                }
            }
        }
        int end_overlaps = 0;
        for (int i = 0; i < kRacers; ++i)
            for (int j = i + 1; j < kRacers; ++j)
                if (touching(a, i, j)) ++end_overlaps;
        std::printf("SESSION  %s, %d concurrents, %d ticks : empreintes %s, %d attente(s) lockstep\n",
                    two_body ? "choc A DEUX CORPS" : "choc d'un seul cote", kRacers, kTicks,
                    first_diff < 0 ? "IDENTIQUES a chaque tick"
                                   : ("DIVERGENTES au tick " + std::to_string(first_diff)).c_str(),
                    stalls);
        std::printf("  depart : %d paire(s) l'une dans l'autre ; fin : %d ; %d contact(s) "
                    "voiture-voiture (par voiture :", start_overlaps, end_overlaps, contacts);
        for (int i = 0; i < kRacers; ++i) std::printf(" %d", contact_by[i]);
        // The original's hit 2/3 only sounds on the first tick against a given car.
        std::printf(") ; %d son(s) de choc", bangs);
        std::printf("\n  plus longue serie de contacts : %d ticks (voiture %d, finie au tick %d, "
                    "vitesse %d u/s)\n  tours :", longest, longest_car, longest_end, longest_speed);
        for (int i = 0; i < kRacers; ++i) std::printf(" %d", a.race().progress(i).lap);
        std::printf("  places :");
        for (int i = 0; i < kRacers; ++i) std::printf(" %d", a.race().progress(i).place);
        std::printf("  empreinte finale %08X\n\n", a.checksum());
        }

        // REPRISE - the original respawn's "is the spot occupied" retry. Six cars drive the start of
        // lap 1 in a bunch, then ALL press the recover key on the same tick, at several moments.
        // Counted: pairs whose PLAIN respawn spots (respawn_for, no retry) would touch, and
        // pairs actually touching once they have been put back - which must be none.
        if (bloc("REPRISE")) {
            bigred::SceneryState wr;
            wr.resize(scene.instances.size());
            bigred::RaceSession r;
            r.rules = rules;
            r.configure(ter, &wr, sg_gates, bigred::parse_laps(level_txt),
                        bigred::parse_countdown_ticks(level_txt), sg_lines, sg, nitros, field, origin);
            int plain_clash = 0, after_clash = 0, recovers = 0;
            bool started_seen = false;
            int since = 0;
            for (int t = 1; t <= 3000; ++t) {
                if (r.race().started()) {
                    if (!started_seen) since = 0;
                    started_seen = true;
                    ++since;
                }
                const bool press = started_seen && since > 0 && since % 250 == 0;
                bigred::VehicleInput in[kRacers];
                for (int p = 0; p < kRacers; ++p) {
                    in[p] = script(p, static_cast<uint32_t>(t));
                    in[p].recover = press;
                }
                if (press) {
                    bigred::RespawnPoint plain[kRacers];
                    for (int i = 0; i < kRacers; ++i)
                        plain[i] = r.race().respawn_for(i, r.start(i));
                    for (int i = 0; i < kRacers; ++i)
                        for (int j = i + 1; j < kRacers; ++j) {
                            const int64_t dx = plain[i].x - plain[j].x, dz = plain[i].z - plain[j].z;
                            const int64_t rr = static_cast<int64_t>(tuning.box.radius) * 2;
                            if (dx * dx + dz * dz < rr * rr / 2) ++plain_clash;
                        }
                }
                r.tick(in);
                if (press) {
                    for (int i = 0; i < kRacers; ++i)
                        if (r.respawned(i)) ++recovers;
                    for (int i = 0; i < kRacers; ++i)
                        for (int j = 0; j < kRacers; ++j)
                            if (i != j && r.car(j).blocks_spot(tuning, r.car(i).world_x(),
                                                               r.car(i).world_z(),
                                                               r.car(i).yaw >> 8))
                                ++after_clash;
                }
            }
            std::printf("REPRISE  ALT pour les 6 au meme tick, 12 fois : %d remise(s) en piste ; "
                        "places simples qui se touchent : %d paire(s) ; apres la reprise : %d "
                        "paire(s) (attendu 0)\n",
                        recovers, plain_clash, after_clash / 2);
            // ...and against the SCENERY (SimTerrain::spot_blocked): every safe
            // gate's plain spot, at three positions across the road, facing down the track.
            // How many of them would drop the car onto a tree, a wall, a post.
            int spots = 0, on_obstacle = 0;
            for (const bigred::Gate& g : sg_gates) {
                if (!g.safe) continue;
                for (int32_t lat : {0x1000, 0x2000, 0x3000}) {
                    const bigred::RespawnPoint at = bigred::gate_respawn(g, lat);
                    ++spots;
                    if (ter.spot_blocked(at.x, at.z, at.yaw_bam16, tuning.box)) ++on_obstacle;
                }
            }
            // The yardstick checked the other way: a car set down ON an obstacle's own centre
            // must be refused. Without this a test that never fires would read as "clear road".
            int self_tested = 0, self_blocked = 0;
            for (const bigred::SimObject& o : ter.blockers) {
                if (!o.hard) continue;
                ++self_tested;
                if (ter.spot_blocked(o.x, o.z, 0, tuning.box)) ++self_blocked;
            }
            std::printf("         contre le decor : %d place(s) de reprise sur %d touchent un obstacle "
                        "(la reprise les evite desormais) ; posee au centre d'un obstacle : "
                        "refusee %d fois sur %d\n\n",
                        on_obstacle, spots, self_blocked, self_tested);
        }

        // CHAMPIONNAT - the championship's rule (game/championship.h) played against the
        // shipped TOURN.TXT: always first, always second, and "lose the first race of every
        // round, win the rest". No race is driven; only the bookkeeping is under test. This
        // is also the CTest gate, so any mismatch must make the process fail.
        if (bloc("CHAMPIONNAT")) {
            bigred::TournCatalog cat;
            if (const bigred::Blob* txt = bigred::find_member(
                    *bigred::game_archive_or_empty(game_dir, "TOURN.WAP"), "TOURN.TXT"))
                cat = bigred::parse_tourn_catalog(std::string(txt->begin(), txt->end()));

            int failures = 0;
            const auto check = [&](bool condition, const std::string& what) {
                if (!condition) {
                    std::printf("  FAIL %s\n", what.c_str());
                    ++failures;
                }
                return condition;
            };

            // The catalogue is the authority for the 24 tournament circuits. RUF1/2/3
            // are ordinary members of it; their map planes merely use the stems 1/2/3.
            // Intersecting with scan_levels also excludes non-tournament menu extras such
            // as CITY3 and TEST1 without hard-coding another circuit list here.
            const std::vector<bigred::LevelEntry> scanned = bigred::scan_levels(game_dir);
            std::set<std::string> scanned_names;
            for (const bigred::LevelEntry& level : scanned) scanned_names.insert(level.name);
            std::vector<std::string> playable;
            std::string missing;
            for (const bigred::TournTrack& t : cat.tracks) {
                if (scanned_names.count(t.level)) {
                    playable.push_back(t.level);
                } else {
                    if (!missing.empty()) missing += ", ";
                    missing += t.level;
                }
            }

            check(cat.tracks.size() == 24, "TOURN.TXT declares exactly 24 tracks");
            check(playable.size() == 24,
                  "all 24 tournament tracks are playable" +
                      (missing.empty() ? std::string() : " (missing " + missing + ")"));
            if (!check(cat.rounds.size() == 7, "TOURN.TXT declares exactly 7 rounds"))
                return 1;

            static constexpr int expectedSize[7] = {3, 3, 4, 4, 4, 5, 1};
            static constexpr int expectedNeed[7] = {1, 1, 2, 2, 2, 3, 1};
            for (std::size_t r = 0; r < cat.rounds.size(); ++r) {
                check(cat.rounds[r].levels.size() ==
                          static_cast<std::size_t>(expectedSize[r]),
                      "round " + std::to_string(r + 1) + " keeps its authored size");
                check(cat.rounds[r].need == expectedNeed[r],
                      "round " + std::to_string(r + 1) + " keeps its authored need");
            }

            std::printf("CHAMPIONNAT %zu manche(s) :", cat.rounds.size());
            for (const bigred::TournRound& r : cat.rounds)
                std::printf(" [%d: %zu circuit(s), need %d]", r.number, r.levels.size(), r.need);
            std::printf("\n");
            {
                // Each circuit's description, where its difficulty is written.
                bigred::TextTable text;
                text.load(game_dir, bigred::config_language(game_dir));
                for (const bigred::TournRound& r : cat.rounds)
                    for (const std::string& l : r.levels) {
                        const bigred::TournTrack* t = cat.find(l);
                        std::printf("  manche %d  %-9s %s\n", r.number, l.c_str(),
                                    t ? text.text(t->view_text).substr(0, 60).c_str() : "?");
                    }
            }
            // A transition is the one race that finishes a round, or the race that
            // eliminates/champions the player. Continue outcomes are represented by the
            // exact total race count.
            auto play = [&](const char* label, auto place_of, int expectedRaces,
                            int expectedRound, bool expectedWon,
                            std::initializer_list<bigred::Championship::Outcome> expected) {
                bigred::Championship c;
                c.start(cat.rounds, playable);
                int races = 0;
                std::vector<bigred::Championship::Outcome> transitions;
                while (!c.over() && races < 100) {
                    const std::vector<std::string> av = c.available();
                    if (av.empty()) break;
                    const int k = c.raced();
                    const auto outcome = c.record(av.front(), place_of(c.round(), k));
                    if (outcome != bigred::Championship::Outcome::Continue)
                        transitions.push_back(outcome);
                    ++races;
                }
                const std::vector<bigred::Championship::Outcome> want(expected);
                const bool ok = races == expectedRaces && c.round() == expectedRound &&
                                c.won() == expectedWon && c.over() &&
                                transitions == want;
                check(ok, std::string(label) + " follows the authored outcome sequence");
                std::printf("  %-34s %d course(s), manche %d/%d, %s\n", label, races,
                            c.round() + 1, c.rounds(),
                            c.won() ? "CHAMPION" : (c.over() ? "elimine" : "EN COURS (bloque ?)"));
            };
            {
                // THE RANDOM CHAMPIONSHIP: 500 draws. Each must use every playable circuit of
                // the shipped rounds exactly once, keep the shipped sizes and needs, and run
                // easy -> hard. With all 24 circuits the English descriptions split 7/9/8;
                // after stable-sorting each round the exact slices are:
                //   3E, 3E, E+3M, 4M, 2M+2H, 5H, H.
                bigred::TextTable english;
                english.load(game_dir, "ENG");
                std::vector<std::pair<std::string, int>> pd;
                for (const std::string& l : playable) {
                    const bigred::TournTrack* t = cat.find(l);
                    pd.emplace_back(l, t ? bigred::difficulty_of(english.text(t->view_text)) : -1);
                }
                int n_easy = 0, n_med = 0, n_hard = 0, n_unknown = 0;
                for (const auto& p : pd)
                    (p.second == 0 ? n_easy : p.second == 1 ? n_med : p.second == 2 ? n_hard : n_unknown)++;
                check(pd.size() == 24, "random championship has all 24 playable circuits");
                check(n_easy == 7 && n_med == 9 && n_hard == 8 && n_unknown == 0,
                      "retail difficulty split is 7 easy, 9 medium, 8 hard");
                static constexpr int expectedDifficulty[7][5] = {
                    {0, 0, 0, -1, -1},
                    {0, 0, 0, -1, -1},
                    {0, 1, 1, 1, -1},
                    {1, 1, 1, 1, -1},
                    {1, 1, 2, 2, -1},
                    {2, 2, 2, 2, 2},
                    {2, -1, -1, -1, -1},
                };
                bigred::Championship ref;
                ref.start(cat.rounds, playable);
                int bad = 0;
                std::string sample;
                for (uint32_t seed = 1; seed <= 500; ++seed) {
                    bigred::Championship c;
                    c.start_random(cat.rounds, pd, seed);
                    std::vector<std::string> seen;
                    int last = -1;
                    bool ok = c.rounds() == ref.rounds();
                    for (int r = 0; ok && r < c.rounds(); ++r) {
                        const bigred::TournRound& cr = c.round_list()[static_cast<size_t>(r)];
                        const bigred::TournRound& rr = ref.round_list()[static_cast<size_t>(r)];
                        ok = cr.levels.size() == rr.levels.size() && cr.need == rr.need;
                        for (std::size_t i = 0; ok && i < cr.levels.size(); ++i) {
                            const std::string& l = cr.levels[i];
                            int d = -1;
                            for (const auto& p : pd)
                                if (p.first == l) d = p.second;
                            if (d < last) ok = false;
                            last = d;
                            if (std::find(seen.begin(), seen.end(), l) != seen.end()) ok = false;
                            seen.push_back(l);
                            if (i >= 5 || d != expectedDifficulty[r][i]) ok = false;
                        }
                    }
                    if (seen.size() != playable.size()) ok = false;
                    if (!ok) ++bad;
                    if (seed == 1)
                        for (const bigred::TournRound& cr : c.round_list()) {
                            sample += " [";
                            for (const std::string& l : cr.levels) sample += l + " ";
                            sample += "]";
                        }
                }
                std::printf("  tirage : %d facile(s), %d moyen(s), %d difficile(s), %d inconnu(s) ; "
                            "500 tirages, %d fautif(s)\n  exemple :%s\n",
                            n_easy, n_med, n_hard, n_unknown, bad, sample.c_str());
                check(bad == 0, "500 seeded random championships match the retail slices");
            }

            using Outcome = bigred::Championship::Outcome;
            play("toujours premier", [](int, int) { return 1; }, 12, 6, true,
                 {Outcome::NextRound, Outcome::NextRound, Outcome::NextRound,
                  Outcome::NextRound, Outcome::NextRound, Outcome::NextRound,
                  Outcome::Champion});
            play("toujours deuxieme", [](int, int) { return 2; }, 3, 0, false,
                 {Outcome::Eliminated});
            play("perd la 1re course de chaque manche", [](int, int k) { return k == 0 ? 3 : 1; },
                 18, 6, false,
                 {Outcome::NextRound, Outcome::NextRound, Outcome::NextRound,
                  Outcome::NextRound, Outcome::NextRound, Outcome::NextRound,
                  Outcome::Eliminated});

            if (failures) {
                std::printf("CHAMPIONNAT FAIL: %d championship regression(s)\n", failures);
                return 1;
            }
            std::printf("CHAMPIONNAT PASS: 24 tracks, 7 authored rounds, 3 outcome scenarios, "
                        "500 random draws\n");
            std::printf("\n");
        }

        // PILOTES - the six drivers of CHARSCN.SCN (scene/character_model.h): body and head
        // parse to their end, and the head sits ON the shoulders - its lowest point within a
        // few units of the body's highest, which is what the original's two heights promise.
        // Game frame, true units (.3DW halved), +Y down.
        if (bloc("PILOTES")) {
            int bad = 0;
            std::printf("PILOTES\n");
            for (int c = 0; c < bigred::kCharacterCount; ++c) {
                bigred::CharacterShapes sh;
                const bigred::CharacterInfo& info = bigred::character_info(c);
                if (!bigred::load_character_shapes(game_dir, c, sh)) {
                    std::printf("  %-8s introuvable\n", info.first_name);
                    ++bad;
                    continue;
                }
                int body_top = 1 << 30, head_bottom = -(1 << 30);
                for (const auto& v : sh.body.vertices) body_top = std::min(body_top, v.y);
                for (const auto& v : sh.head.vertices) head_bottom = std::max(head_bottom, v.y);
                const float gap = (-sh.head_rise + head_bottom * 0.5f) - body_top * 0.5f;
                const bool ok = sh.body.reached_clean_end && sh.head.reached_clean_end &&
                                gap > -2.0f && gap < 8.0f && sh.mouth_tm.size() == 49152;
                if (!ok) ++bad;
                std::printf("  %-8s %-7s + %s%s  tete %.0f au-dessus, recouvrement du cou %.1f  %s\n",
                            info.first_name, info.body, info.head, info.head_coloured ? "x" : "",
                            sh.head_rise, gap, ok ? "ok" : "FAUX");
            }
            std::printf("  %d fautif(s)\n\n", bad);
        }

        // SPLIT - the split screen's layout (game/split_layout.h), the user's own: 2 = top and
        // bottom, 3 = three corners and the map in the fourth, 4 = corners. Every view inside
        // the window, none overlapping another or the map.
        if (bloc("SPLIT")) {
            int bad = 0;
            for (int n = 1; n <= 4; ++n) {
                const bigred::SplitLayout l = bigred::split_layout(n, 1920, 1080);
                std::vector<bigred::SplitRect> all = l.views;
                if (l.has_map) all.push_back(l.map);
                bool ok = static_cast<int>(l.views.size()) == n && l.has_map == (n == 3);
                for (size_t a = 0; a < all.size(); ++a) {
                    const bigred::SplitRect& r = all[a];
                    ok = ok && r.x >= 0 && r.y >= 0 && r.x + r.w <= 1920 && r.y + r.h <= 1080;
                    for (size_t b = a + 1; b < all.size(); ++b) {
                        const bigred::SplitRect& q = all[b];
                        if (r.x < q.x + q.w && q.x < r.x + r.w && r.y < q.y + q.h && q.y < r.y + r.h)
                            ok = false;
                    }
                }
                if (n == 2) ok = ok && l.views[0].w == 1920 && l.views[1].y > l.views[0].y;
                if (n == 3) ok = ok && l.map.x > 0 && l.map.y > 0;
                if (!ok) ++bad;
                std::printf("SPLIT %d joueur(s) :", n);
                for (const bigred::SplitRect& r : l.views)
                    std::printf(" [%d,%d %dx%d]", r.x, r.y, r.w, r.h);
                if (l.has_map)
                    std::printf(" carte [%d,%d %dx%d]", l.map.x, l.map.y, l.map.w, l.map.h);
                std::printf("  %s\n", ok ? "ok" : "FAUX");
            }
            // Two players side by side: two full-height columns, left then right, apart.
            {
                const bigred::SplitLayout l = bigred::split_layout(2, 1920, 1080, 2, true);
                const bool ok = l.views.size() == 2 && !l.has_map && l.views[0].x == 0 &&
                                l.views[0].h == 1080 && l.views[1].h == 1080 &&
                                l.views[0].w + 2 <= l.views[1].x &&
                                l.views[1].x + l.views[1].w == 1920;
                if (!ok) ++bad;
                std::printf("SPLIT 2 cote a cote : [%d,%d %dx%d] [%d,%d %dx%d]  %s\n",
                            l.views[0].x, l.views[0].y, l.views[0].w, l.views[0].h, l.views[1].x,
                            l.views[1].y, l.views[1].w, l.views[1].h, ok ? "ok" : "FAUX");
            }
            std::printf("  %d fautif(s)\n\n", bad);
        }

        // INSTANTANE - the network resync's snapshot (sim/snapshot.h). Race A drives 900 ticks;
        // race B drives 400 ticks of OTHER inputs, so that everything in it is wrong, then
        // thaws A's snapshot. From there both are fed the same inputs and must agree on
        // checksum() at every tick - and B's own snapshot must be byte for byte A's. A field
        // forgotten by snapshot.cpp shows up as a divergence some ticks later.
        if (bloc("INSTANTANE")) {
            bigred::SceneryState w1, w2;
            w1.resize(scene.instances.size());
            w2.resize(scene.instances.size());
            bigred::RaceSession s1, s2;
            s1.rules = rules;
            s1.configure(ter, &w1, sg_gates, bigred::parse_laps(level_txt),
                         bigred::parse_countdown_ticks(level_txt), sg_lines, sg, nitros, field, origin);
            s2.rules = rules;
            s2.configure(ter, &w2, sg_gates, bigred::parse_laps(level_txt),
                         bigred::parse_countdown_ticks(level_txt), sg_lines, sg, nitros, field, origin);
            bigred::VehicleInput in[kRacers];
            for (int t = 1; t <= 900; ++t) {
                for (int p = 0; p < kRacers; ++p) {
                    in[p] = script(p, static_cast<uint32_t>(t));
                    in[p].recover = (t % 300) == 0 && p == 2;
                }
                s1.tick(in);
            }
            for (int t = 1; t <= 400; ++t) {
                for (int p = 0; p < kRacers; ++p) in[p] = script(p + 3, static_cast<uint32_t>(t));
                s2.tick(in);
            }
            const std::vector<uint8_t> snap = bigred::save_race(s1);
            const bool loaded = bigred::load_race(s2, snap.data(), snap.size());
            const bool same_bytes = bigred::save_race(s2) == snap;
            int first_diff = -1;
            for (int t = 901; t <= 2400 && loaded; ++t) {
                for (int p = 0; p < kRacers; ++p) {
                    in[p] = script(p, static_cast<uint32_t>(t));
                    in[p].recover = (t % 300) == 0 && p == 4;
                }
                s1.tick(in);
                s2.tick(in);
                if (first_diff < 0 && s1.checksum() != s2.checksum()) first_diff = t;
            }
            std::printf("INSTANTANE %zu octets ; charge : %s ; relu a l'identique : %s ; "
                        "1500 ticks ensuite : %s\n\n",
                        snap.size(), loaded ? "oui" : "NON", same_bytes ? "oui" : "NON",
                        first_diff < 0 ? "IDENTIQUES"
                                       : ("DIVERGENT au tick " + std::to_string(first_diff)).c_str());
        }
    }

    // --- REPLAY -------------------------------------------------------------------------
    // The opponents are recorded ghosts. The original loads
    // `N.REC`: a 40-byte header (type at 0, record count as int16 at 32), then `count` records
    // of 12 bytes (type 2) or 14 (type 3). It starts every ghost at its own grid pose
    // and adds one record's six deltas (x, y, z, pitch, yaw, roll) per 0x1000 of
    // playback time, the time advancing 0x200 per 28 ms tick. The test that this reading is
    // right: integrated from grid slot N, ghost N must drive ROUND THE RING, gate after gate.
    if (bloc("REPLAY")) {
        const bigred::StartingGrid rg = bigred::parse_grid(level_txt);
        std::vector<bigred::Gate> rgates;
        if (auto cse = scene.lev.find("MAP.CSE"); cse != scene.lev.end())
            rgates = bigred::parse_gates(cse->second.data(), cse->second.size());
        std::printf("REPLAY   les fantomes\n");
        // The script lines that steer the ghosts: `speed` (case 0x14, the rubber band's three
        // playback rates), `heli` (slot 6, the fly-by) and `car` (which vehicles the level has).
        {
            size_t pos = 0;
            while (pos < level_txt.size()) {
                size_t end = level_txt.find('\n', pos);
                if (end == std::string::npos) end = level_txt.size();
                std::string ln = level_txt.substr(pos, end - pos);
                pos = end + 1;
                size_t a = ln.find_first_not_of(" \t");
                if (a == std::string::npos) continue;
                ln = ln.substr(a);
                if (ln.rfind("speed", 0) == 0 || ln.rfind("heli", 0) == 0 || ln.rfind("car", 0) == 0 ||
                    ln.rfind("spos", 0) == 0)
                    std::printf("  script : %s\n", ln.c_str());
            }
        }
        for (int n = 0; n <= 6; ++n) {
            auto it = scene.lev.find(std::to_string(n) + ".REC");
            if (it == scene.lev.end()) continue;
            const bigred::Blob& b = it->second;
            if (b.size() < 40) { std::printf("  %d.REC trop court\n", n); continue; }
            auto s16 = [&](size_t o) {
                return static_cast<int32_t>(static_cast<int16_t>(b[o] | (b[o + 1] << 8)));
            };
            const int type = b[0];
            const int count = s16(32) & 0xFFFF;
            const size_t rs = type == 2 ? 12 : 14;
            const bool size_ok = b.size() == 40 + static_cast<size_t>(count) * rs;
            // The ghost's own start. Slot 6 is the fly-by helicopter (`heli` line), not a racer.
            const bigred::RespawnPoint st = bigred::grid_slot(rg, 6, n < 6 ? n : 0);
            int32_t x = st.x, z = st.z, yaw = st.yaw_bam16;
            int32_t y = ter.height_fx(st.x << 9, st.z << 9) >> 9;
            int on_ring = 0, gate_changes = 0, last_gate = -1, max_nitro = 0, max_dmg = 0;
            int32_t max_step = 0;
            for (int r = 0; r < count; ++r) {
                const size_t o = 40 + static_cast<size_t>(r) * rs;
                const int32_t dx = s16(o), dy = s16(o + 2), dz = s16(o + 4);
                x += dx;
                y += dy;
                z += dz;
                yaw = (yaw + s16(o + 8)) & 0xFFFF;
                max_step = std::max(max_step, static_cast<int32_t>(std::sqrt(
                                                  static_cast<double>(dx) * dx + static_cast<double>(dz) * dz)));
                if (rs == 14) {
                    max_nitro = std::max(max_nitro, static_cast<int>(b[o + 12]));
                    max_dmg = std::max(max_dmg, static_cast<int>(b[o + 13]));
                }
                int g = -1;
                for (size_t k = 0; k < rgates.size() && g < 0; ++k)
                    if (bigred::probe_gate(rgates[k], x, z).inside) g = static_cast<int>(k);
                if (g >= 0) {
                    ++on_ring;
                    if (g != last_gate) { ++gate_changes; last_gate = g; }
                }
            }
            std::printf("  %d.REC type %d, %d enregistrement(s) %s, octets 28-31 = %d %d %d %d ; "
                        "sur l'anneau %d/%d, %d changement(s) de porte, pas maxi %d u, "
                        "arrivee (%d,%d,%d), nitro maxi %d, degats maxi %d%%\n",
                        n, type, count, size_ok ? "(taille exacte)" : "(TAILLE FAUSSE)", b[28],
                        static_cast<int8_t>(b[29]), b[30], b[31], on_ring, count, gate_changes,
                        max_step, x, y, z, max_nitro, max_dmg);
        }

        // THE ORIGINAL'S GRAVITY, READ OFF ITS OWN GHOSTS. A .REC is the original game driving: one
        // pose every 8 ticks (224 ms). While a ghost is in the air, its height is a parabola, so
        // the second difference of y over three airborne records is the gravity the original
        // really applied, times 8 * 8 ticks^2. Compared with `cogs[3]` of the vehicle the
        // header names: if the two agree, a "lighter" feel is not in the gravity. Airborne here
        // means more than 60 units over the terrain at all three records (a bridge or a
        // platform would read as a flight - hence medians, not means).
        std::printf("VOL      la gravite de l'original, mesuree sur ses fantomes\n");
        for (int n = 0; n < bigred::kGridRacers; ++n) {
            auto it = scene.lev.find(std::to_string(n) + ".REC");
            if (it == scene.lev.end()) continue;
            const bigred::ReplayTrack t = bigred::parse_replay(it->second);
            if (!t.valid()) continue;
            const std::string veh = (t.vehicle_index >= 0 &&
                                     static_cast<size_t>(t.vehicle_index) < cars.size())
                                        ? cars[static_cast<size_t>(t.vehicle_index)]
                                        : vehicle_name;
            bigred::VehicleTuning vt;
            if (!bigred::load_vehicle(game_dir, veh, vt)) continue;
            const bigred::RespawnPoint st = bigred::grid_slot(rg, 6, n);
            int32_t x = st.x, y = ter.height_fx(st.x << 9, st.z << 9) >> 9, z = st.z;
            std::vector<int32_t> ys, clear;
            for (const auto& r : t.records) {
                x += r[0];
                y += r[1];
                z += r[2];
                ys.push_back(y);
                clear.push_back((ter.height_fx(x << 9, z << 9) >> 9) - y);
            }
            std::vector<int32_t> d2;
            int flights = 0, longest = 0, run = 0;
            for (size_t i = 0; i < ys.size(); ++i) {
                if (clear[i] > 60) {
                    if (++run == 1) ++flights;
                    longest = std::max(longest, run);
                } else {
                    run = 0;
                }
                if (i >= 2 && clear[i] > 60 && clear[i - 1] > 60 && clear[i - 2] > 60)
                    d2.push_back(ys[i] - 2 * ys[i - 1] + ys[i - 2]);
            }
            // cogs[3] is in 1/512 unit per tick^2: times 64 ticks^2 per record^2.
            const double ours = vt.gravity() * 64.0 / 512.0;
            if (d2.empty()) {
                std::printf("  %d.REC %-8s aucun vol de 3 enregistrements (%d vol(s))\n", n,
                            veh.c_str(), flights);
                continue;
            }
            std::vector<int32_t> sorted = d2;
            std::sort(sorted.begin(), sorted.end());
            const int32_t med = sorted[sorted.size() / 2];
            std::printf("  %d.REC %-8s %d vol(s), le plus long %d enr. (%.1f s) ; courbure "
                        "mediane %d u/enr^2 sur %zu mesures (quartiles %d..%d) - la notre %.0f, "
                        "rapport %.2f\n",
                        n, veh.c_str(), flights, longest, longest * 0.224, med, d2.size(),
                        sorted[sorted.size() / 4], sorted[sorted.size() * 3 / 4], ours,
                        ours > 0 ? med / ours : 0.0);
        }

        // SAUT - THE SAME JUMP, TWICE. The user finds the original's cars "lighter": longer
        // jumps, more bounce, less slide (2026-09-27). VOL above says the gravity is not it. So
        // each ghost flight is flown again by OUR physics from the ghost's own state three
        // records (24 ticks) before take-off - pose and velocity as GhostDriver + dress_replayed
        // leave them - throttle held, no steering, and both are sampled every 8 ticks, the
        // recording's own rate. ⚠️ A FLIGHT IS WHERE THE PATH IS BALLISTIC (second difference of
        // y within 25% of the gravity), NOT WHERE THE BODY IS HIGH OVER THE TERRAIN: roads on
        // these levels are objects standing over it, and "high over the terrain" read a whole
        // raised road as one long flight.
        std::printf("SAUT     les memes sauts, par l'original (fantome) et par notre physique\n");
        {
            using P3 = std::array<int32_t, 4>;   // x, y, z, and the body pitch (BAM16, signed)
            struct Flight { int recs = 0, rise = 0, dist = 0, launch = 0, runs = 0, v[5] = {}, drift = 0,
                            pmin = 0, pmax = 0; };
            auto ballistic = [](const std::vector<P3>& p, int32_t g64, size_t i) {
                const int32_t d2 = p[i + 1][1] - 2 * p[i][1] + p[i - 1][1];
                return d2 * 4 >= g64 * 3 && d2 * 4 <= g64 * 5;
            };
            // Samples every 8 ticks. The first arc at or after `from`.
            auto measure = [&](const std::vector<P3>& p, int32_t g64, size_t from) {
                Flight f;
                // The pitch the body reaches over the whole window, both ways - the boat that
                // tips over its nose off WATER1's ramp is read here.
                for (const P3& q : p) {
                    f.pmin = std::min(f.pmin, q[3]);
                    f.pmax = std::max(f.pmax, q[3]);
                }
                // The horizontal speed over the first five records: the run-up and the lip.
                for (size_t k = 0; k < 5 && k + 1 < p.size(); ++k)
                    f.v[k] = static_cast<int>(std::sqrt(std::pow(double(p[k + 1][0] - p[k][0]), 2) +
                                                        std::pow(double(p[k + 1][2] - p[k][2]), 2)));
                size_t i = std::max<size_t>(from, 1);
                while (i + 1 < p.size() && !ballistic(p, g64, i)) ++i;
                if (i + 1 >= p.size()) return f;
                const size_t s = i - 1;   // the last sample before the arc
                f.launch = p[s][1] - p[s + 1][1];   // + = going up, units per 8 ticks
                while (i + 1 < p.size() && ballistic(p, g64, i)) {
                    f.rise = std::max(f.rise, p[s][1] - p[i][1]);
                    ++i;
                }
                f.recs = static_cast<int>(i - s);
                f.dist = static_cast<int>(std::sqrt(std::pow(double(p[i][0] - p[s][0]), 2) +
                                                    std::pow(double(p[i][2] - p[s][2]), 2)));
                // Arcs begun again in the next 8 samples (1.8 s): the bounce.
                for (size_t k = i + 1; k + 1 < p.size() && k < i + 9; ++k)
                    if (ballistic(p, g64, k) && !ballistic(p, g64, k - 1)) ++f.runs;
                return f;
            };
            // Which input turns the car toward a larger yaw: checked by the drift it leaves.
            constexpr int kSautSteerSign = 1;
            double sum_drift = 0;
            double sum[3][5] = {}, sum_pmin[3] = {}, sum_pmax[3] = {};
            int compared = 0, skipped_nitro = 0;
            bool traced = false;
            for (int n = 0; n < bigred::kGridRacers; ++n) {
                auto it = scene.lev.find(std::to_string(n) + ".REC");
                if (it == scene.lev.end()) continue;
                const bigred::ReplayTrack t = bigred::parse_replay(it->second);
                if (!t.valid()) continue;
                const std::string veh = (t.vehicle_index >= 0 &&
                                         static_cast<size_t>(t.vehicle_index) < cars.size())
                                            ? cars[static_cast<size_t>(t.vehicle_index)]
                                            : vehicle_name;
                bigred::VehicleTuning vt;
                if (!bigred::load_vehicle(game_dir, veh, vt)) continue;
                const int32_t g64 = vt.gravity() * 64 / 512;
                // The ghost's record boundaries.
                std::vector<P3> gp(1, P3{0, 0, 0});
                for (const auto& r : t.records)
                    gp.push_back({gp.back()[0] + r[0], gp.back()[1] + r[1], gp.back()[2] + r[2],
                                  static_cast<int16_t>(gp.back()[3] + r[3])});
                // Take-offs: an arc of 3 samples or more after 4 that are not one.
                std::vector<size_t> starts;
                for (size_t j = 5; j + 4 < gp.size(); ++j)
                    if (ballistic(gp, g64, j) && ballistic(gp, g64, j + 1) &&
                        ballistic(gp, g64, j + 2) && !ballistic(gp, g64, j - 1) &&
                        !ballistic(gp, g64, j - 2) && !ballistic(gp, g64, j - 3) &&
                        !ballistic(gp, g64, j - 4))
                        starts.push_back(j);
                for (size_t j : starts) {
                    const size_t from_rec = j - 4;   // three records before the arc's first sample
                    // Flown twice by our physics: as the game runs it, and with the terrain
                    // push computed exactly as the original does (no deviation 4).
                    auto fly = [&](bool faithful, bool trace = false) {
                        ter.faithful_probe = faithful;
                        bigred::VehicleSim c;
                        c.reset(vt, ter, rules, bigred::grid_slot(rg, 6, n), bigred::NitroReserve{});
                        c.place_on_grid();
                        bigred::GhostDriver gd;
                        gd.start(t, c);
                        for (size_t k = 0; k < from_rec * 8; ++k) {
                            gd.tick(c);
                            c.dress_replayed();
                        }
                        // ⚠️ GhostDriver writes vy = dy << 10, TWICE the per-tick rate in 1/512
                        // (a display value, as the original leaves it). Driven from there, the
                        // car would leave every ramp with double its upward speed.
                        c.vy /= 2;
                        // The recording carries no controls: a ghost at speed was on the
                        // throttle, so ours starts with the pedal already down rather than
                        // ramping it up from nothing through the run-up (256 = full scale).
                        c.throttle_fwd = c.throttle_axis = 256;
                        // The ghost goes on beside it, and ours steers for the ghost's heading:
                        // driven straight, it met the ramp somewhere else than the ghost did.
                        bigred::VehicleSim g2 = c;
                        std::vector<P3> op(1, P3{c.world_x(), c.world_y(), c.world_z(),
                                                 static_cast<int16_t>(c.pitch_bam16())});
                        bigred::VehicleInput in;
                        in.accel = true;
                        int drift = 0;
                        if (trace)
                            std::printf("      trace : tick vitesse   vfwd   vlat     vy  con near sol "
                                        "pente  poussee      y  push x/y/z (1/512)   tangage\n");
                        for (int k = 1; k <= 8 * 20; ++k) {
                            gd.tick(g2);
                            g2.dress_replayed();
                            const int32_t dyaw = static_cast<int16_t>(g2.yaw_bam16() - c.yaw_bam16());
                            in.right = dyaw * kSautSteerSign > 0x80;
                            in.left = dyaw * kSautSteerSign < -0x80;
                            c.step(in);
                            if (k == 24)
                                drift = static_cast<int>(std::hypot(double(c.world_x() - g2.world_x()),
                                                                    double(c.world_z() - g2.world_z())));
                            if (trace && k <= 40)
                                std::printf("      %11d %7d %6d %6d %6d  %02X  %02X   %d %6d %7d %6d %6d %6d %6d %6.1f\n", k,
                                            c.speed, c.vfwd, c.vlat, c.vy, c.contact,
                                            c.near_ground, c.land, c.terrain_pitch, c.thrust,
                                            c.world_y(), c.push_xyz(0), c.push_xyz(1), c.push_xyz(2),
                                            static_cast<int16_t>(c.pitch_bam16()) * 360.0 / 65536);
                            if (k % 8 == 0)
                                op.push_back({c.world_x(), c.world_y(), c.world_z(),
                                              static_cast<int16_t>(c.pitch_bam16())});
                        }
                        ter.faithful_probe = false;
                        Flight fl = measure(op, g64, 1);
                        fl.drift = drift;
                        return fl;
                    };
                    const std::vector<P3> gpart(
                        gp.begin() + static_cast<long>(from_rec),
                        gp.begin() + static_cast<long>(std::min(gp.size(), from_rec + 21)));
                    // A ghost under NITRO in the run-up is not comparable with a car on the
                    // throttle alone: the 7th field's low byte is the flame, 0 when off.
                    bool boosted = false;
                    for (size_t r = from_rec; r < std::min(t.records.size(), j + 6); ++r)
                        boosted = boosted || (t.type == 3 && (t.records[r][6] & 0xFF) != 0);
                    if (boosted) {
                        ++skipped_nitro;
                        continue;
                    }
                    const Flight a = measure(gpart, g64, 1), b = fly(false), f = fly(true);
                    std::printf("  %d.REC %-8s enr. %4zu : original %2d enr., monte %4d, %5d u, "
                                "elan %+4d | nous %2d, %4d, %5d, %+4d | sans ecart 4 %2d, %4d, "
                                "%5d, %+4d\n",
                                n, veh.c_str(), j, a.recs, a.rise, a.dist, a.launch, b.recs,
                                b.rise, b.dist, b.launch, f.recs, f.rise, f.dist, f.launch);
                    std::printf("      vitesse par enr. : original %d %d %d %d %d | nous %d %d %d %d %d\n",
                                a.v[0], a.v[1], a.v[2], a.v[3], a.v[4], b.v[0], b.v[1], b.v[2],
                                b.v[3], b.v[4]);
                    std::printf("      tangage : original %+.1f..%+.1f deg | nous %+.1f..%+.1f deg\n",
                                a.pmin * 360.0 / 65536, a.pmax * 360.0 / 65536,
                                b.pmin * 360.0 / 65536, b.pmax * 360.0 / 65536);
                    // The fastest run-up of the level, traced tick by tick: where the speed goes.
                    // BENCH_SAUT_TRACE=N traces the jump of record N instead (the "enr." column).
                    const char* want = std::getenv("BENCH_SAUT_TRACE");
                    if (want ? static_cast<size_t>(std::atoi(want)) == j
                             : (a.v[0] >= 530 && !traced)) {
                        traced = true;
                        fly(false, true);
                    }
                    sum_drift += b.drift;
                    const Flight* ab[3] = {&a, &b, &f};
                    for (int w = 0; w < 3; ++w) {
                        sum[w][0] += ab[w]->recs;
                        sum[w][1] += ab[w]->rise;
                        sum[w][2] += ab[w]->dist;
                        sum[w][3] += ab[w]->launch;
                        sum[w][4] += ab[w]->runs;
                        sum_pmin[w] += ab[w]->pmin;
                        sum_pmax[w] += ab[w]->pmax;
                    }
                    ++compared;
                }
            }
            if (compared > 0) {
                const char* who[3] = {"original    ", "nous        ", "sans ecart 4"};
                std::printf("  MOYENNE sur %d saut(s) (%d ecarte(s) : fantome en nitro), ecart au "
                            "fantome au decollage %.0f u :\n",
                            compared, skipped_nitro, sum_drift / compared);
                for (int w = 0; w < 3; ++w)
                    std::printf("    %s %.1f enr. en l'air, monte %.0f, %.0f u, elan %+.0f, "
                                "rebonds %.2f, vitesse horizontale %.0f u/enr., tangage extreme "
                                "%+.1f / %+.1f deg\n",
                                who[w], sum[w][0] / compared, sum[w][1] / compared,
                                sum[w][2] / compared, sum[w][3] / compared, sum[w][4] / compared,
                                sum[w][0] > 0 ? sum[w][2] / sum[w][0] : 0.0,
                                sum_pmin[w] / compared * 360.0 / 65536.0,
                                sum_pmax[w] / compared * 360.0 / 65536.0);
            } else {
                std::printf("  aucun saut trouve sur les fantomes de ce niveau\n");
            }
        }

        // RETOURNE - does OUR car end up on its roof driving the ghosts' own line? The user,
        // with deviations 2 and 3 off: "impossible to do a lap of DIRT1 without ending up on
        // the roof". JUMP drives blind across the map and never saw it. Here each ghost's run
        // is cut into 11-second stretches; each one starts from the ghost's own state (as
        // SAUT does: throttle down, vy halved) and steers for the ghost's heading. Counted: the
        // stretches in which the body goes over - roll past 90 degrees or the pitch fold. The
        // original's own drivers never do, on these lines.
        {
            int stretches = 0, over = 0;
            for (int n = 1; n < bigred::kGridRacers; ++n) {
                auto it = scene.lev.find(std::to_string(n) + ".REC");
                if (it == scene.lev.end()) continue;
                const bigred::ReplayTrack t = bigred::parse_replay(it->second);
                if (!t.valid()) continue;
                const std::string veh = (t.vehicle_index >= 0 &&
                                         static_cast<size_t>(t.vehicle_index) < cars.size())
                                            ? cars[static_cast<size_t>(t.vehicle_index)]
                                            : vehicle_name;
                bigred::VehicleTuning vt;
                if (!bigred::load_vehicle(game_dir, veh, vt)) continue;
                // BENCH_PIVOT=N: the pivot N units LOWER in the box (the user's "the centre of
                // gravity looks above the car", 2026-09-27) - an experiment, not the original.
                if (const char* pv = std::getenv("BENCH_PIVOT")) {
                    const int32_t d = std::atoi(pv);
                    for (auto& cn : vt.box.corner) cn[1] -= d;
                    vt.box.ybottom -= d;
                    vt.box.ytop -= d;
                }
                const size_t total = t.records.size() * 8;
                for (size_t from = 400; from + 400 < total; from += 400) {
                    bigred::VehicleSim c;
                    c.reset(vt, ter, rules, bigred::grid_slot(rg, 6, n), bigred::NitroReserve{});
                    c.place_on_grid();
                    bigred::GhostDriver gd;
                    gd.start(t, c);
                    for (size_t k = 0; k < from; ++k) {
                        gd.tick(c);
                        c.dress_replayed();
                    }
                    c.vy /= 2;
                    c.throttle_fwd = c.throttle_axis = 256;
                    bigred::VehicleSim g2 = c;
                    bigred::VehicleInput in;
                    in.accel = true;
                    bool went_over = false;
                    for (int k = 0; k < 400 && !went_over; ++k) {
                        gd.tick(g2);
                        g2.dress_replayed();
                        const int32_t dyaw = static_cast<int16_t>(g2.yaw_bam16() - c.yaw_bam16());
                        in.right = dyaw > 0x80;
                        in.left = dyaw < -0x80;
                        const bool was = c.upside_down;
                        c.step(in);
                        if (c.rolled_over || c.upside_down != was) went_over = true;
                    }
                    ++stretches;
                    if (went_over) ++over;
                }
            }
            std::printf("RETOURNE sur la trajectoire des fantomes : %d troncon(s) de 11 s sur %d "
                        "finissent sur le toit\n", over, stretches);
        }

        // THE RACE, WITH THEM IN IT. The player on grid block 0, standing still; ghosts 1-5 on
        // their own blocks, each on the vehicle its header names, played back by the RaceSession
        // exactly as the game will. Every ghost must finish the race on its own, in a time a
        // human would recognise as a race, and two sessions must agree on every tick.
        std::vector<bigred::ReplayTrack> tracks(bigred::kGridRacers);
        std::vector<bigred::VehicleTuning> tunings(bigred::kGridRacers);
        std::vector<bigred::RacerSetup> field(bigred::kGridRacers);
        field[0] = bigred::RacerSetup{&tuning, 30, nullptr};
        int ghosts = 0;
        for (int n = 1; n < bigred::kGridRacers; ++n) {
            auto it = scene.lev.find(std::to_string(n) + ".REC");
            if (it == scene.lev.end()) continue;
            tracks[static_cast<size_t>(n)] = bigred::parse_replay(it->second);
            const bigred::ReplayTrack& t = tracks[static_cast<size_t>(n)];
            if (!t.valid()) continue;
            const std::string veh = (t.vehicle_index >= 0 &&
                                     static_cast<size_t>(t.vehicle_index) < cars.size())
                                        ? cars[static_cast<size_t>(t.vehicle_index)]
                                        : vehicle_name;
            if (!bigred::load_vehicle(game_dir, veh, tunings[static_cast<size_t>(n)])) continue;
            field[static_cast<size_t>(n)] =
                bigred::RacerSetup{&tunings[static_cast<size_t>(n)], 30, &t};
            ++ghosts;
        }
        if (ghosts > 0) {
            std::vector<bigred::Gate> gg = rgates;
            bigred::mark_unsafe_gates(gg, bigred::parse_unsafe_gates(level_txt));
            std::vector<bigred::CheckLine> gl = bigred::build_check_lines(
                scene.instances, scene.shape_names_by_slot,
                [&](const std::string& shape) -> const bigred::Blob* {
                    auto f = scene.lev.find(shape + ".FT");
                    return f == scene.lev.end() ? nullptr : &f->second;
                });
            bigred::mark_checkpoint_gates(gg, gl);
            bigred::RespawnPoint origin;
            origin.x = scene.map_w * 512 / 2;
            origin.z = scene.map_h * 512 / 2;
            const bigred::ReplayRates rates = bigred::parse_replay_rates(level_txt);
            bigred::SceneryState w1, w2;
            w1.resize(scene.instances.size());
            w2.resize(scene.instances.size());
            bigred::RaceSession s1, s2;
            s1.rules = rules;
            s1.configure(ter, &w1, gg, bigred::parse_laps(level_txt),
                         bigred::parse_countdown_ticks(level_txt), gl, rg, nitros, field, origin,
                         rates);
            s2.rules = rules;
            s2.configure(ter, &w2, gg, bigred::parse_laps(level_txt),
                         bigred::parse_countdown_ticks(level_txt), gl, rg, nitros, field, origin,
                         rates);
            const bigred::VehicleInput idle[bigred::kGridRacers] = {};
            int diverged = -1, t = 0;
            int32_t rate_lo = 0x7fffffff, rate_hi = 0;
            long dress_ticks = 0, dress_rolling = 0, dress_ground = 0, dress_skid = 0,
                 dress_moving = 0, dress_particles = 0;
            int32_t dress_spin_prev[bigred::kGridRacers] = {};
            bigred::ParticleField fx_field;
            {
                const std::string env_name = bigred::env_name_for_level(level);
                if (const bigred::Blob* txt = bigred::find_member(
                        *bigred::game_archive_or_empty(game_dir, env_name + ".ENV"), env_name + ".TXT"))
                    fx_field.configure(bigred::parse_spot_effects(std::string(txt->begin(), txt->end())));
            }
            int ghost_missed_past[bigred::kGridRacers] = {}, ghost_missed_order[bigred::kGridRacers] = {};
            int ghost_reached_prev[bigred::kGridRacers], ghost_reached_since[bigred::kGridRacers] = {},
                ghost_stall_max[bigred::kGridRacers] = {};
            for (int& r : ghost_reached_prev) r = -2;
            int alt_checks = 0, alt_forward = 0;
            bool ghost1_skipped = false;
            for (; t < 20000; ++t) {
                s1.tick(idle);
                s2.tick(idle);
                if (diverged < 0 && s1.checksum() != s2.checksum()) diverged = t;
                // The DRESSING (VehicleSim::dress_replayed): what the display reads of a ghost
                // besides its pose - wheels turning, ground under it, a slide, dust.
                for (int i = 1; i < bigred::kGridRacers; ++i) {
                    if (!s1.is_ghost(i) || !s1.race().started()) continue;
                    // The original's own drivers take every line: a MISSED CHECK on them is a
                    // false alarm of ours - the line driven past (event 0x28) or
                    // touched out of order (event 0x26).
                    const bigred::RacerProgress& gp_ = s1.race().progress(i);
                    // The longest the ring left a ghost's progress standing still, in ticks: a
                    // stuck `reached` freezes the placings and the rubber band with it.
                    if (gp_.reached != ghost_reached_prev[i]) {
                        if (i == 1 && std::getenv("BENCH_REACHED"))
                            std::printf("    [fantome 1] tick %d : porte atteinte %d -> %d (dans la "
                                        "porte %d, fenetre %d..%d, derniere ligne %d)\n",
                                        t, ghost_reached_prev[i], gp_.reached, gp_.gate,
                                        gp_.window_start, gp_.window, gp_.last_check);
                        ghost_reached_prev[i] = gp_.reached;
                        ghost_reached_since[i] = t;
                    }
                    if (!gp_.finished)   // a ghost home waits for the others, standing still
                        ghost_stall_max[i] = std::max(ghost_stall_max[i], t - ghost_reached_since[i]);
                    // ALT SENDING A CAR FORWARD (user, 2026-09-27: "ALT m'a ramené plus en avant,
                    // parfois juste avant la ligne d'arrivée"). Every 50 ticks, where would ALT
                    // put this ghost? Ahead of the gate it is standing in - by the ring, less than
                    // half of it - is the defect; the first offender is traced.
                    if (i == 1 && gp_.skipped_line != ghost1_skipped && std::getenv("BENCH_ALT")) {
                        ghost1_skipped = gp_.skipped_line;
                        std::printf("    [fantome 1] tick %d : ligne sautee -> %d (porte %d, porte "
                                    "d'avant %d, derniere ligne %d, derniere touchee %d, fenetre "
                                    "%d..%d, depasse %d, desordre %d)\n",
                                    t, gp_.skipped_line ? 1 : 0, gp_.gate, gp_.prev_gate,
                                    gp_.last_check, gp_.last_touched, gp_.window_start, gp_.window,
                                    gp_.missed_window_now ? 1 : 0, gp_.missed_check_now ? 1 : 0);
                    }
                    if (t % 50 == 0 && gp_.gate >= 0 && !gp_.finished) {
                        const int rg_ = s1.race().respawn_gate(i);
                        const int n_ = static_cast<int>(s1.race().gates().size());
                        if (rg_ >= 0 && n_ > 0) {
                            const int fwd = (rg_ - gp_.gate + n_) % n_;
                            ++alt_checks;
                            if (fwd > 1 && fwd < n_ / 2) {
                                ++alt_forward;
                                if (alt_forward <= 6)
                                    std::printf("    [ALT en avant] fantome %d tick %d : dans la porte "
                                                "%d, atteinte %d, ALT -> porte %d (derniere ligne %d, "
                                                "ligne sautee %d, fenetre %d..%d)\n",
                                                i, t, gp_.gate, gp_.reached, rg_, gp_.last_check,
                                                gp_.skipped_line ? 1 : 0, gp_.window_start, gp_.window);
                            }
                        }
                    }
                    if (gp_.missed_window_now) ++ghost_missed_past[i];
                    if (gp_.missed_check_now) ++ghost_missed_order[i];
                    const bigred::VehicleSim& c = s1.car(i);
                    ++dress_ticks;
                    if (c.wheel_spin != dress_spin_prev[i]) ++dress_rolling;
                    dress_spin_prev[i] = c.wheel_spin;
                    if (c.near_ground) ++dress_ground;
                    if (c.skidding) ++dress_skid;
                    if (c.moving) ++dress_moving;
                    fx_field.emit(s1.car(i), static_cast<uint32_t>(t));
                }
                fx_field.tick();
                fx_field.tick();
                dress_particles += fx_field.live_count();
                rate_lo = std::min(rate_lo, s1.ghost_rate());
                rate_hi = std::max(rate_hi, s1.ghost_rate());
                bool all = true;
                for (int i = 1; i < bigred::kGridRacers; ++i)
                    if (s1.is_ghost(i) && !s1.race().progress(i).finished) all = false;
                if (all) break;
            }
            std::printf("  ALT : %d position(s) testee(s), %d enverrai(en)t la voiture EN AVANT\n",
                        alt_checks, alt_forward);
            std::printf("  course : joueur immobile + %d fantome(s) (vitesses %s: %X/%X/%X), "
                        "%d ticks, empreintes %s, rythme %X..%X\n", ghosts,
                        level_txt.find("speed") != std::string::npos ? "du script" : "par defaut",
                        rates.min, rates.normal, rates.max, t,
                        diverged < 0 ? "identiques" : "DIVERGENTES", rate_lo, rate_hi);
            if (dress_ticks > 0)
                std::printf("  habillage des fantomes : roues qui tournent %ld%%, au sol %ld%%, "
                            "en mouvement %ld%%, derapage %ld%%, particules vivantes %.1f par tick\n",
                            dress_rolling * 100 / dress_ticks, dress_ground * 100 / dress_ticks,
                            dress_moving * 100 / dress_ticks, dress_skid * 100 / dress_ticks,
                            static_cast<double>(dress_particles) / static_cast<double>(t));
            for (int i = 1; i < bigred::kGridRacers; ++i) {
                if (!s1.is_ghost(i)) continue;
                const bigred::RacerProgress& p = s1.race().progress(i);
                std::printf("    fantome %d (%s) : %d tour(s)%s, place %d", i,
                            cars.empty() ? "?" : cars[static_cast<size_t>(std::min<int>(
                                tracks[static_cast<size_t>(i)].vehicle_index,
                                static_cast<int>(cars.size()) - 1))].c_str(),
                            p.lap, p.finished ? " FINI" : "", p.place);
                std::printf(", checkpoint manque %d fois (depasse %d, desordre %d), progression "
                            "figee au plus %.1f s",
                            ghost_missed_past[i] + ghost_missed_order[i], ghost_missed_past[i],
                            ghost_missed_order[i], ghost_stall_max[i] * bigred::kPhysicsTickMs / 1000.0);
                if (p.finished)
                    std::printf(", en %.1f s", p.finish_tick * bigred::kGameTickMs / 1000.0);
                std::printf(", tours :");
                for (uint32_t lt : p.lap_ticks) std::printf(" %.1f", lt * bigred::kGameTickMs / 1000.0);
                std::printf("\n");
            }

            // ...and the other way round: a line NOBODY can touch - ordinal 1 moved far off
            // the map - must be reported the moment each ghost drives past where it was, once
            // a lap (event 0x28), not only at the next line.
            if (gl.size() >= 3) {
                std::vector<bigred::CheckLine> gone = gl;
                for (bigred::CheckLine& c : gone)
                    if (c.ordinal() == 1) c.x += 200000;
                bigred::SceneryState w3;
                w3.resize(scene.instances.size());
                bigred::RaceSession s3;
                s3.rules = rules;
                s3.configure(ter, &w3, gg, bigred::parse_laps(level_txt),
                             bigred::parse_countdown_ticks(level_txt), gone, rg, nitros, field,
                             origin, rates);
                int past[bigred::kGridRacers] = {}, order[bigred::kGridRacers] = {};
                for (int k = 0; k < t; ++k) {
                    s3.tick(idle);
                    for (int i = 1; i < bigred::kGridRacers; ++i) {
                        if (!s3.is_ghost(i)) continue;
                        past[i] += s3.race().progress(i).missed_window_now ? 1 : 0;
                        order[i] += s3.race().progress(i).missed_check_now ? 1 : 0;
                    }
                }
                std::printf("  ligne 1 rendue intouchable, %d ticks :", t);
                for (int i = 1; i < bigred::kGridRacers; ++i)
                    if (s3.is_ghost(i))
                        std::printf(" fantome %d depasse %d / desordre %d ;", i, past[i], order[i]);
                std::printf("\n");
            }
        }
        std::printf("\n");
    }

    // --- LIGHTS -------------------------------------------------------------------------
    // The start gantry's traffic lights. The user's report: "there is no countdown number on
    // screen; the posts holding the start banner go from red to green when the race starts."
    // `S_LIGHTS` is the shape, it sits in pairs either side of the `START` banner, and its
    // collision category is 0x35 - a category of its own. What this block asks is whether its
    // display list draws something DIFFERENT for a different variant byte, which is the only
    // mechanism the engine has for changing a decor piece's appearance.
    if (bloc("LIGHTS")) {
        int slot = -1;
        for (size_t i = 0; i < scene.shape_names_by_slot.size(); ++i)
            if (scene.shape_names_by_slot[i] == "S_LIGHTS") slot = static_cast<int>(i);
        if (slot < 0) {
            std::printf("LIGHTS   ce niveau n'a pas de S_LIGHTS\n\n");
        } else {
            auto blob = scene.lev.find("S_LIGHTS.3DW");
            std::printf("LIGHTS   S_LIGHTS = forme %d, %d instance(s)\n", slot, [&] {
                int n = 0;
                for (const bigred::SceneryInstance& in : scene.instances)
                    if (in.shape_index == slot) ++n;
                return n;
            }());
            for (const bigred::SceneryInstance& in : scene.instances) {
                if (in.shape_index != slot) continue;
                std::printf("  instance variante=%d cat=%d a (%.0f,%.0f,%.0f)\n", in.variant_index,
                            in.collision_category_byte, in.x, in.y, in.z);
            }
            if (blob != scene.lev.end()) {
                const bigred::ThreeDW m =
                    bigred::parse_3dw(blob->second.data(), blob->second.size());
                std::printf("  %zu sommet(s), %zu face(s), %zu recadrage(s) de texture\n",
                            m.vertices.size(), m.faces.size(), m.texture_variants.size());
                for (size_t c = 0; c < m.texture_variants.size(); ++c) {
                    const bigred::UVVariant& uv = m.texture_variants[c];
                    std::printf("  recadrage %zu : u %.4f..%.4f v %.4f..%.4f\n", c,
                                uv.u0, uv.u1, uv.v0, uv.v1);
                }
                for (int v = 0; v < 8; ++v) {
                    const bigred::DisplayListReplay r = bigred::replay_display_list(
                        m.branches, m.face_stream_pos, m.crop_stream_pos, m.faces.size(), v);
                    int n = 0;
                    for (bool b : r.drawn) if (b) ++n;
                    std::printf("  variante %d -> %d face(s), recadrages", v, n);
                    for (size_t f = 0; f < r.crop.size(); ++f)
                        std::printf(" %d", r.crop[f]);
                    std::printf("\n");
                }
            }
            std::printf("\n");
        }
    }

    // --- RAMP-GATE ----------------------------------------------------------------------
    // ⚠️ Not the same thing as the RAMP block further down: that one drives UP a ramp and
    // traces the take-off, this one asks whether the wedge is a floor or a wall.
    // "Taking a jump the wrong way round teleports the car to the top of it." A ramp is a
    // WEDGE, and the original resolves a point that is INSIDE the wedge by pushing it straight
    // out to the surface - a push with no upper bound. Come at the wedge from its tall end and
    // the body's front corners start the tick several hundred units under the surface, so that
    // one push is the whole height of the ramp: the car appears at the top instead of stopping
    // against the wall.
    //
    // Two measurements, because the fix has to satisfy both. The PROFILE walk asks the narrow
    // phase directly, with no integration: at every point along the object's own Z axis, what
    // does the probe say the ground is and how hard does it push? The DRIVE then does what the
    // user did, from each end in turn, and prints the biggest single-tick lift.
    if (bloc("RAMP-GATE")) {
        std::printf("RAMP-GATE  the drivable wedges, probed along their own Z axis\n");
        int shown = 0;
        std::set<std::string> ramp_shapes_done;
        for (const bigred::SimObject& o : ter.objects) {
            if (o.platform) continue;
            if (o.slope == 0) continue;     // flat: nothing to climb, nothing to fall off
            std::string shape = "?";
            if (o.instance_index >= 0 &&
                static_cast<size_t>(o.instance_index) < scene.instances.size()) {
                const int si = scene.instances[static_cast<size_t>(o.instance_index)].shape_index;
                if (si >= 0 && static_cast<size_t>(si) < scene.shape_names_by_slot.size())
                    shape = scene.shape_names_by_slot[static_cast<size_t>(si)];
            }
            if (!ramp_shapes_done.insert(shape).second) continue;   // one row per SHAPE
            if (shown++ >= 4) break;
            const int32_t half = (o.depth_low + o.depth_high) / 2;
            std::printf("      %s at (%d,%d,%d) yaw=%.0f deg  demi-profondeur=%d  hauteur=%d"
                        " pente=%d (soit %d au bout)  large=%d  ramp_gate=%d  .FT+0x32=%d\n",
                        shape.c_str(), o.x, o.y, o.z, deg16(o.yaw_bam16), half, o.height,
                        o.slope, o.slope, o.wide ? 1 : 0, o.ramp_gate ? 1 : 0, o.past_z_lift);
            // A single candidate is what a lone ramp gives the probe, and that is the case the
            // report is about: with fewer than two candidates the original SKIPS its far-Z
            // bound, so the surface carries on past the end of the object.
            bigred::ObjectCandidates cand;
            cand.items[0] = &o;
            cand.count = 1;
            const int32_t terr = ter.height_fx(o.x << 9, o.z << 9) >> 9;
            // +Y is DOWN and `ybottom` is the box's lowest corner, so a body resting on that
            // ground has its CENTRE that far above the surface.
            const int32_t body_y = terr - tuning.box.ybottom;
            std::printf("      sol autour de l'objet : %d ; un corps y est a %d\n", terr,
                        body_y);
            std::printf("      %8s %8s %8s %8s  %s\n", "lz", "test", "sol", "poussee Y", "lecture");
            for (int32_t lz = -half * 2; lz <= half * 2; lz += half / 2 ? half / 2 : 1) {
                const int32_t wx = o.x + bigred::fx::mul15(lz, bigred::fx::sin16(o.yaw_bam16));
                const int32_t wz = o.z + bigred::fx::mul15(lz, bigred::fx::cos16(o.yaw_bam16));
                const int32_t wy = body_y;
                int32_t g = 0, push[3] = {0, 0, 0};
                const int r = ter.object_probe(cand, wx << 9, wy << 9, wz << 9, g, push);
                const char* read = r == 0 ? "hors profil"
                                  : r == 1 ? "pose dessus"
                                           : "DEDANS - repousse";
                std::printf("      %8d %8d %8d %8d  %s%s\n", lz, r, r ? g : 0, push[1], read,
                            (lz < -half || lz > half) ? "  (au-dela du bout)" : "");
            }
            // And now the drive, from each end. +Z first, then -Z: one of the two is the ramp
            // taken the right way round and should climb it, the other is the wall.
            bigred::VehicleSim probe;
            for (int dir = 0; dir < 2; ++dir) {
                const int32_t approach = dir ? (o.yaw_bam16 + 0x8000) & 0xFFFF : o.yaw_bam16;
                const int32_t start = 2600;
                bigred::RespawnPoint p;
                p.x = o.x - bigred::fx::mul15(start, bigred::fx::sin16(approach));
                p.z = o.z - bigred::fx::mul15(start, bigred::fx::cos16(approach));
                p.yaw_bam16 = approach;
                probe.reset(tuning, ter, rules, p, nitros);
                settle(probe, 40);
                bigred::VehicleInput go;
                go.accel = true;
                int32_t worst_lift = 0, worst_tick = -1, speed_at = 0, speed_after = 0;
                int32_t y_prev = probe.py >> 9;
                int32_t dist_at = 0;
                // ...and, separately, what the FIRST contact with this object did to the speed.
                // The two numbers answer the two halves of the report: the lift says whether the
                // body was teleported, the speed says whether it was stopped.
                int32_t hit_speed_in = -1, hit_speed_out = -1, hit_dist = 0;
                int32_t closest = INT32_MAX;
                for (int t = 0; t < 160; ++t) {
                    const int32_t speed_before = probe.speed;
                    probe.step(go);
                    const int32_t y = probe.py >> 9;
                    const int32_t dx = (probe.px >> 9) - o.x, dz = (probe.pz >> 9) - o.z;
                    const int32_t d = static_cast<int32_t>(
                        std::llround(std::sqrt(static_cast<double>(dx) * dx +
                                               static_cast<double>(dz) * dz)));
                    if (d < closest) closest = d;
                    const int32_t lift = y_prev - y;   // +Y is DOWN, so a rise is positive here
                    if (lift > worst_lift) {
                        worst_lift = lift;
                        worst_tick = t;
                        speed_at = speed_before;
                        speed_after = probe.speed;
                        dist_at = d;
                    }
                    if (hit_speed_in < 0 && probe.touched_instance(o.instance_index)) {
                        hit_speed_in = speed_before;
                        hit_speed_out = probe.speed;
                        hit_dist = d;
                    }
                    y_prev = y;
                }
                // ⚠️ Read BOTH. A wall that works takes the speed away; a wedge that teleports
                // keeps it and moves the body instead. A run that shows neither never arrived.
                std::printf("      approche %s : plus gros bond vertical d'un seul tick = %d u"
                            " (tick %d, a %d u du centre, vitesse %d -> %d)\n",
                            dir ? "par -Z" : "par +Z", worst_lift, worst_tick, dist_at, speed_at,
                            speed_after);
                if (hit_speed_in >= 0)
                    std::printf("                     bloque a %d u du centre, vitesse %d -> %d"
                                " (approche au plus pres : %d u)\n",
                                hit_dist, hit_speed_in, hit_speed_out, closest);
                else
                    std::printf("                     jamais bloque ; approche au plus pres :"
                                " %d u\n", closest);
            }
        }
        if (shown == 0) std::printf("      ce niveau n'en a aucune\n");
        std::printf("\n");
    }

    // --- TUNNELS ------------------------------------------------------------------------
    // "The terrain above a tunnel has no physics, and neither has the tunnel's roof or its
    // sides - only its floor does, so a car driving OVER a tunnel sinks into the hill down to
    // the tunnel floor." That is one measurement: settle a body on top of the shape and compare
    // where it comes to rest with what the HEIGHTMAP says the ground is there. A tunnel is a
    // bore through a hill; its floor is the terrain, so the two must agree.
    if (bloc("TUNNELS")) {
        std::printf("TUNNELS  se poser SUR un tunnel : le corps suit-il le terrain ?\n");
        int rows = 0;
        std::set<std::string> done;
        for (const bigred::SimObject& o : ter.objects) {
            if (!o.tunnel) continue;
            std::string shape = "?";
            if (o.instance_index >= 0 &&
                static_cast<size_t>(o.instance_index) < scene.instances.size()) {
                const int si = scene.instances[static_cast<size_t>(o.instance_index)].shape_index;
                if (si >= 0 && static_cast<size_t>(si) < scene.shape_names_by_slot.size())
                    shape = scene.shape_names_by_slot[static_cast<size_t>(si)];
            }
            if (!done.insert(shape).second) continue;
            if (rows++ >= 4) break;
            // The profile, because every question about a bore is a question about its cotes:
            // `x_low..x_high` are the local-X bounds the SIDE WALLS sit on, and `depth_*` the
            // half-extent along local Z. Printed next to the body's own half-width, which is
            // what has to fit between the walls for the mouth to be passable at all.
            std::printf("      %-10s x %d..%d, demi-Z %d..%d, rayon %d, flags %02X, large=%d"
                        "  (demi-largeur du corps : %d)\n",
                        shape.c_str(), o.x_low, o.x_high, o.depth_low, o.depth_high, o.radius,
                        o.flags, o.wide ? 1 : 0,
                        std::max(std::abs(tuning.box.xmin), std::abs(tuning.box.xmax)));
            // The narrow-phase half of the `.FT`: the original's scenery pass runs a sector test
            // (0x16 a sector reach) or a box test (0x16 >= 0xfffd, then a box at 0x18..0x1e)
            // BEFORE any of
            // its cases, tunnels included - the gate that says whether the car touches the shape
            // at all.
            if (auto ft = scene.lev.find(shape + ".FT"); ft != scene.lev.end() && ft->second.size() >= 0x56) {
                const uint8_t* f = ft->second.data();
                auto s16 = [f](int off) { return static_cast<int16_t>(f[off] | (f[off + 1] << 8)); };
                std::printf("      %-10s .FT 0x16=%04X  0x18..0x1e = %d %d %d %d  secteurs:",
                            shape.c_str(), static_cast<uint16_t>(s16(0x16)), s16(0x18), s16(0x1a),
                            s16(0x1c), s16(0x1e));
                for (int k = 0; k < 32; k += 4) std::printf(" %d", s16(0x16 + 2 * k));
                int count = 0;
                for (const bigred::SimObject& o2 : ter.objects)
                    if (o2.tunnel && o2.instance_index >= 0 &&
                        scene.instances[static_cast<size_t>(o2.instance_index)].shape_index ==
                            scene.instances[static_cast<size_t>(o.instance_index)].shape_index)
                        ++count;
                std::printf("  (%d instance(s))\n", count);
            }
            // ⚠️ THE YARDSTICK IS THE TERRAIN UNDER EACH CORNER, not the heightmap at the
            // object's centre. A tunnel mouth sits in a cliff face by construction, so the
            // centre reading can be 800 units away from what the body is actually standing on -
            // which reads as a catastrophic sink when nothing is wrong at all. The question is
            // narrower and has a yes/no answer: does any corner take its ground from the SHAPE
            // instead of from the heightmap?
            bigred::RespawnPoint p;
            p.x = o.x;
            p.z = o.z;
            p.yaw_bam16 = o.yaw_bam16;
            bigred::VehicleSim probe;
            probe.reset(tuning, ter, rules, p, nitros);
            settle(probe, 120);
            // ⚠️ With a tolerance: `corner_ground` comes from the sliding terrain probe and
            // `height_fx` from a plain bilinear read, so they differ by a unit or two on a
            // slope. A tunnel deck is HUNDREDS of units away from the hilltop, so anything
            // that matters here is far outside that band.
            auto drift = [&](const bigred::VehicleSim& v) {
                int32_t w = 0;
                for (int i = 0; i < 8; ++i) {
                    const int32_t h = ter.height_fx(v.corner_x[i] << 9, v.corner_z[i] << 9) >> 9;
                    const int32_t d = v.corner_ground[i] - h;
                    if (std::abs(d) > std::abs(w)) w = d;
                }
                return w;
            };
            const int32_t at_rest = drift(probe);
            // ...and again while DRIVING across the shape, along its own local X - the path that
            // takes the body over the crown of the hill the bore runs through.
            const int32_t cross = (o.yaw_bam16 + 0x4000) & 0xFFFF;
            bigred::RespawnPoint q;
            q.x = o.x - bigred::fx::mul15(1800, bigred::fx::sin16(cross));
            q.z = o.z - bigred::fx::mul15(1800, bigred::fx::cos16(cross));
            q.yaw_bam16 = cross;
            probe.reset(tuning, ter, rules, q, nitros);
            settle(probe, 30);
            bigred::VehicleInput go;
            go.accel = true;
            int32_t driving = 0;
            for (int t = 0; t < 120; ++t) {
                probe.step(go);
                const int32_t d = drift(probe);
                if (std::abs(d) > std::abs(driving)) driving = d;
            }
            // ⚠️ THE VERDICT IS NOT A FLAT THRESHOLD, because two of the three roles legitimately
            // put the body on the deck. As in the original: a shape with NO roof (flags bit 2) is a
            // floor whenever the body is above its deck, and a body inside the bore stands on
            // the deck by definition. Only the third case - above a ROOFED shape - has to track
            // the heightmap, and that is what the driving pass crosses.
            const char* verdict;
            if ((o.flags & 0x02) != 0)      verdict = "sans voute : toujours plancher, conforme";
            else if (std::abs(driving) > 8) verdict = "LE TUNNEL FAIT PLANCHER SUR LA COLLINE";
            else                            verdict = "ok : la colline garde son relief";
            std::printf("      %-10s ecart sol/terrain : %+d u a l'arret, %+d u en roulant"
                        " par-dessus  %s  [voute %d, radier %d]\n",
                        shape.c_str(), at_rest, driving, verdict,
                        o.roof_height + o.y, o.height + o.y);
            // And the other half of the report: the bore's SIDES. Put the body inside the tube,
            // STRADDLING the footprint's local-X bound - centre inside, outer corners 30 units
            // past it - and run a few ticks. A wall that works pushes it back toward the axis;
            // no wall at all leaves it where it was.
            //
            // ⚠️ STRADDLING, not wholly outside. The walls sit behind the scenery pass's narrow
            // phase, the `.FT` box, and a body entirely past the bound does not touch that box -
            // in the original no more than here. A car only ever meets a wall from the inside,
            // i.e. overlapping it, which is the case measured. (Placed wholly outside, 60 units
            // past, this row read "no wall" on half the shapes the day the box gate went in.)
            {
                const int32_t axis = o.yaw_bam16;                 // local +Z, along the bore
                const int32_t side = (o.yaw_bam16 + 0x4000) & 0xFFFF;  // local +X, across it
                const int32_t half_w = std::max(std::abs(tuning.box.xmin), std::abs(tuning.box.xmax));
                const int32_t out_by = o.x_high + 30 - half_w;
                bigred::RespawnPoint r;
                r.x = o.x + bigred::fx::mul15(out_by, bigred::fx::sin16(side));
                r.z = o.z + bigred::fx::mul15(out_by, bigred::fx::cos16(side));
                r.yaw_bam16 = axis;
                bigred::VehicleSim w;
                w.reset(tuning, ter, rules, r, nitros);
                // ⚠️ HOLD IT INSIDE THE BORE, and give it a tick to fill its corner ring first.
                // `hit_tunnel` reads the corners the PREVIOUS tick left - the original's own
                // one-tick lag - so a single step straight after a reset tests nothing, which
                // is what made this row report a wall on some shapes and not on others.
                const int32_t bore_y = ((o.roof_height + o.height) / 2 + o.y) << 9;
                w.py = bore_y;
                w.vy = 0;
                w.step(bigred::VehicleInput{});
                w.py = bore_y;
                w.vy = 0;
                const int32_t bx = w.px, bz = w.pz;
                for (int k = 0; k < 4; ++k) {
                    w.step(bigred::VehicleInput{});
                    w.py = bore_y;      // stay in the tube; the floor is not what is measured
                    w.vy = 0;
                }
                const int32_t moved_x = (w.px - bx) >> 9, moved_z = (w.pz - bz) >> 9;
                const int32_t back = -(bigred::fx::mul15(moved_x, bigred::fx::sin16(side)) +
                                       bigred::fx::mul15(moved_z, bigred::fx::cos16(side)));
                // A shape whose flags remove the +X wall (bit 8) has nothing to push with there.
                const bool no_wall = (o.flags & 0x08) != 0;
                std::printf("      %-10s a cheval sur le bord lateral (coins %d u au-dela) : repousse"
                            " de %d u vers l'axe  %s\n", shape.c_str(), 30, back,
                            no_wall ? "pas de paroi +X dans ce .FT (bit 8), conforme"
                                    : back > 10 ? "ok" : "PAS DE PAROI");
            }
            // ⚠️ AND THE THIRD CASE, the one the first attempt at this broke: can you DRIVE
            // THROUGH? Inside the bore the deck IS the roadway - the heightmap is not dug out,
            // it is solid hill - so a tunnel that stops being a floor plugs its own mouth.
            // Start just inside one mouth, on the deck, and drive down the axis.
            {
                const int32_t axis = o.yaw_bam16;                 // local +Z, along the bore
                const int32_t half = (o.depth_low + o.depth_high) / 2;
                const int32_t from = -(half - 120);
                bigred::RespawnPoint r;
                r.x = o.x + bigred::fx::mul15(from, bigred::fx::sin16(axis));
                r.z = o.z + bigred::fx::mul15(from, bigred::fx::cos16(axis));
                r.yaw_bam16 = axis;
                bigred::VehicleSim t;
                t.reset(tuning, ter, rules, r, nitros);
                t.py = ((o.height + o.y) << 9);   // on the deck, not wherever the hill is
                t.vy = 0;
                settle(t, 20);
                bigred::VehicleInput go;
                go.accel = true;
                int in_bore = 0;
                for (int k = 0; k < 200; ++k) { t.step(go); if (t.in_tunnel) ++in_bore; }
                const int32_t ex = (t.px >> 9) - o.x, ez = (t.pz >> 9) - o.z;
                const int32_t along = bigred::fx::mul15(ex, bigred::fx::sin16(axis)) +
                                      bigred::fx::mul15(ez, bigred::fx::cos16(axis));
                std::printf("      %-10s lance a lz=%d, arrive a lz=%d apres 200 ticks"
                            " (%d tick(s) dans le tube)  %s\n",
                            shape.c_str(), from, along, in_bore,
                            (along - from) > 600 ? "ok, ca traverse" : "BOUCHE");
            }
            // ...and the CAMERA through it. Start well back on the approach so the camera has
            // settled behind the car before the mouth, drive the axis, and report the largest
            // vertical step the camera takes in one tick. The user's report of 2026-09-23 - "elle
            // monte d'un coup a la sortie de tunnel" - is exactly that number spiking on the tick
            // the car leaves the bore while the camera is still in it, under the hill.
            {
                const int32_t axis = o.yaw_bam16;
                const int32_t half = (o.depth_low + o.depth_high) / 2;
                const int32_t from = -(half + 1500);
                bigred::RespawnPoint r;
                r.x = o.x + bigred::fx::mul15(from, bigred::fx::sin16(axis));
                r.z = o.z + bigred::fx::mul15(from, bigred::fx::cos16(axis));
                r.yaw_bam16 = axis;
                bigred::VehicleSim t;
                t.reset(tuning, ter, rules, r, nitros);
                settle(t, 20);
                bigred::ChaseCamera cam;
                cam.reset(t, ter);
                bigred::VehicleInput go;
                go.accel = true;
                int32_t worst = 0, worst_at = -1, below = 0, prev_y = cam.now.y;
                bool was_in = false;
                int exit_tick = -1;
                for (int k = 0; k < 300; ++k) {
                    t.step(go);
                    cam.step(t, ter);
                    if (was_in && !t.in_tunnel && exit_tick < 0) exit_tick = k;
                    was_in = t.in_tunnel;
                    // Skip the opening pull-in: it is a planned move, not a jump.
                    const int32_t dy = cam.now.y - prev_y;
                    prev_y = cam.now.y;
                    if (k < 40) continue;
                    if ((dy < 0 ? -dy : dy) > (worst < 0 ? -worst : worst)) { worst = dy; worst_at = k; }
                    const int32_t ground = ter.height_fx(cam.now.x << 9, cam.now.z << 9) >> 9;
                    if (cam.now.y > ground && !cam.under_roof()) ++below;   // +Y down
                }
                std::printf("      %-10s camera : plus grand pas vertical %+d u au tick %d"
                            " (sortie du tube au tick %d), %d tick(s) sous le relief hors tube\n",
                            shape.c_str(), -worst, worst_at, exit_tick, below);
            }
        }
        if (rows == 0) std::printf("      ce niveau n'en a aucun\n");
        std::printf("\n");
    }

    // --- PROBE -------------------------------------------------------------------------
    // BENCH_PROBE=x,z : drop the car at one world point and trace it tick by tick - height,
    // the heightmap under it, the lowest corner ground, and which objects are candidates. The
    // tool for "why does the car sink HERE", which a scan can only point at.
    if (const char* pe = std::getenv("BENCH_PROBE"); bloc("PROBE") && pe) {
        int px_w = 0, pz_w = 0;
        if (std::sscanf(pe, "%d,%d", &px_w, &pz_w) == 2) {
            bigred::RespawnPoint r;
            r.x = px_w;
            r.z = pz_w;
            r.yaw_bam16 = 0;
            bigred::VehicleSim v;
            v.reset(tuning, ter, rules, r, nitros);
            std::printf("PROBE    (%d,%d) terrain %d\n", px_w, pz_w,
                        ter.height_fx(px_w << 9, pz_w << 9) >> 9);
            for (int t = 0; t < 20; ++t) {
                const bigred::ObjectCandidates c = ter.gather_candidates(
                    v.px, v.py, v.pz, tuning.box.radius, &tuning.box, v.yaw >> 8);
                std::printf("  t%-2d y %d  terrain %d  sol des coins", t, v.py >> 9,
                            ter.height_fx(v.px, v.pz) >> 9);
                for (int i = 0; i < 8; i += 2) std::printf(" %d", v.corner_ground[i]);
                std::printf("  tunnel %d  candidats", v.in_tunnel ? 1 : 0);
                for (int i = 0; i < c.count; ++i)
                    std::printf(" #%d(flags %02X)", c.items[i]->instance_index, c.items[i]->flags);
                std::printf("\n");
                v.step(bigred::VehicleInput{});
            }
        }
    }

    // --- TUNNELS2 -----------------------------------------------------------------------
    // The block above tests ONE instance per shape, along its own axis. Two reports of
    // 2026-09-23 fell outside that: on CITY4 "si on passe dans le tunnel de gauche, on est
    // téléporté dans celui de droite", and on DIRT3 "si on passe dessus, on tombe dedans". So,
    // over EVERY tunnel instance:
    //   * DRIVE THROUGH each roofed bore down its axis and record how far the body strays
    //     sideways from that axis - a push from a neighbouring bore shows up as a jump of the
    //     order of the distance between the two;
    //   * SCAN THE GROUND around it: drop the car on a 256-unit grid, let it settle, and record
    //     how far its ground sits BELOW the heightmap while it is not in a bore. Anything over a
    //     few units is a shape acting as a floor under the hill.
    if (bloc("TUNNELS2")) {
        int tubes = 0;
        for (const bigred::SimObject& o : ter.objects) if (o.tunnel) ++tubes;
        if (tubes > 0) std::printf("TUNNELS2 toutes les instances (%d)\n", tubes);
        auto name_of = [&](const bigred::SimObject& o) -> std::string {
            if (o.instance_index < 0) return "?";
            const int si = scene.instances[static_cast<size_t>(o.instance_index)].shape_index;
            return (si >= 0 && static_cast<size_t>(si) < scene.shape_names_by_slot.size())
                       ? scene.shape_names_by_slot[static_cast<size_t>(si)] : "?";
        };
        for (const bigred::SimObject& o : ter.objects) {
            if (!o.tunnel) continue;
            const int32_t axis = o.yaw_bam16 & 0xFFFF;
            const int32_t side = (o.yaw_bam16 + 0x4000) & 0xFFFF;
            auto world = [&](int32_t lx, int32_t lz, int32_t& wx, int32_t& wz) {
                wx = o.x + bigred::fx::mul15(lx, bigred::fx::sin16(side)) +
                     bigred::fx::mul15(lz, bigred::fx::sin16(axis));
                wz = o.z + bigred::fx::mul15(lx, bigred::fx::cos16(side)) +
                     bigred::fx::mul15(lz, bigred::fx::cos16(axis));
            };
            auto local = [&](int32_t wx, int32_t wz, int32_t& lx, int32_t& lz) {
                const int32_t ex = wx - o.x, ez = wz - o.z;
                lx = bigred::fx::mul15(ex, bigred::fx::sin16(side)) +
                     bigred::fx::mul15(ez, bigred::fx::cos16(side));
                lz = bigred::fx::mul15(ex, bigred::fx::sin16(axis)) +
                     bigred::fx::mul15(ez, bigred::fx::cos16(axis));
            };
            std::printf("      %-9s #%-4d (%6d,%6d) cap &%04X flags %02X :", name_of(o).c_str(),
                        o.instance_index, o.x, o.z, static_cast<unsigned>(axis), o.flags);

            // Through the bore - roofed shapes only; a roofless apron has no bore.
            if ((o.flags & 0x02) == 0) {
                const int32_t half = (o.depth_low + o.depth_high) / 2;
                bigred::RespawnPoint r;
                world(0, -(half - 120), r.x, r.z);
                r.yaw_bam16 = axis;
                bigred::VehicleSim t;
                t.reset(tuning, ter, rules, r, nitros);
                t.py = (o.height + o.y) << 9;
                t.vy = 0;
                settle(t, 20);
                bigred::VehicleInput go;
                go.accel = true;
                int32_t worst = 0;
                for (int k = 0; k < 200; ++k) {
                    t.step(go);
                    int32_t lx, lz;
                    local(t.px >> 9, t.pz >> 9, lx, lz);
                    if (lz > half + 400) break;          // out the far mouth: done
                    if (std::abs(lx) > std::abs(worst)) worst = lx;
                }
                std::printf(" ecart lateral max dans le tube %+d u %s", worst,
                            std::abs(worst) > 300 ? "<- POUSSE HORS DE SON TUBE" : "ok");
            }

            // The ground around it.
            int32_t worst_sink = 0, at_lx = 0, at_lz = 0, bad = 0, samples = 0;
            const int32_t reach_z = std::max(o.depth_low, o.depth_high) + 1536;
            for (int32_t lz = -reach_z; lz <= reach_z; lz += 256) {
                for (int32_t lx = -3072; lx <= 3072; lx += 256) {
                    bigred::RespawnPoint r;
                    world(lx, lz, r.x, r.z);
                    r.yaw_bam16 = axis;
                    bigred::VehicleSim v;
                    v.reset(tuning, ter, rules, r, nitros);
                    settle(v, 15);
                    if (v.in_tunnel) continue;
                    ++samples;
                    int32_t sink = 0;
                    for (int i = 0; i < 8; ++i) {
                        const int32_t h = ter.height_fx(v.corner_x[i] << 9, v.corner_z[i] << 9) >> 9;
                        sink = std::max(sink, v.corner_ground[i] - h);   // +Y down: + is below
                    }
                    if (sink > 60) ++bad;
                    if (sink > worst_sink) { worst_sink = sink; at_lx = lx; at_lz = lz; }
                }
            }
            std::printf("  | sol : %d/%d points sous la colline, pire %+d u a (lx %d, lz %d)%s\n",
                        bad, samples, worst_sink, at_lx, at_lz,
                        worst_sink > 200 ? "  <- ON TOMBE DANS LA COLLINE" : "");
        }
        if (tubes > 0) std::printf("\n");
    }

    // --- TERRAIN ------------------------------------------------------------------------
    // Does the terrain the renderer DRAWS agree with the terrain the simulation drives on?
    // A user report - "the banks and road edges are wrong, and only in the render: the car is in
    // the right place but we go through polygons" - comes down to one bit of geometry, the
    // diagonal each cell is split along. Every corner agrees whichever split you pick, so the
    // test has to sample INSIDE the triangles: the centroid of each emitted triangle, against
    // SimTerrain::height_fx at the same spot.
    //
    // This is a data question about the whole map, not something a screenshot can settle, so it
    // lives here. It has to stay at 0: any non-zero maximum is terrain the car drives on and the
    // player cannot see.
    if (bloc("TERRAIN")) {
        std::printf("\nTERRAIN  drawn mesh vs simulated ground, centroid of every triangle\n");
        const std::vector<bigred::TerrainVertex> mesh = bigred::build_terrain_mesh(
            scene.mph_data, scene.mpc_data,
            (scene.cht_size >= 512) ? scene.cht_data : nullptr, scene.cht_size,
            0,                        // no atlases: the flat-colour path, geometry is identical
            scene.map_w, scene.map_h,
            {}, {},                   // no palettes: only x/y/z is read below
            1.0f,                     // render scale 1 -> raw world units
            bigred::kHeightWorldScale);

        // Render space mirrors world X about (map_w - 1) * 512 and points +Y up; the simulation is
        // unmirrored with +Y down. Bring a drawn vertex back into the simulation's own frame.
        const float mirror_axis =
            static_cast<float>(scene.map_w - 1) * static_cast<float>(bigred::kTerrainTexelWorldSize);
        auto to_sim = [&](const bigred::TerrainVertex& v, double& x, double& y, double& z) {
            x = mirror_axis - static_cast<double>(v.x);
            y = -static_cast<double>(v.y);
            z = static_cast<double>(v.z);
        };

        long long worst = 0, worst_x = 0, worst_z = 0, checked = 0, nonzero = 0;
        double sum = 0.0;
        for (size_t i = 0; i + 2 < mesh.size(); i += 3) {
            double x[3], y[3], z[3];
            for (int k = 0; k < 3; ++k) to_sim(mesh[i + static_cast<size_t>(k)], x[k], y[k], z[k]);
            const double cx = (x[0] + x[1] + x[2]) / 3.0;
            const double cy = (y[0] + y[1] + y[2]) / 3.0;
            const double cz = (z[0] + z[1] + z[2]) / 3.0;
            // A centroid always lands strictly inside its own half of the split - (1/3, 2/3) and
            // (2/3, 1/3) in cell fractions - so `fx > fz` picks this very triangle, never its
            // neighbour. No tie to break.
            const int32_t xf = static_cast<int32_t>(cx * 512.0);
            const int32_t zf = static_cast<int32_t>(cz * 512.0);
            const double ground = ter.height_fx(xf, zf) / 512.0;
            const long long d = std::llround(std::abs(ground - cy));
            ++checked;
            sum += static_cast<double>(d);
            if (d) ++nonzero;
            if (d > worst) { worst = d; worst_x = static_cast<long long>(cx); worst_z = static_cast<long long>(cz); }
        }
        std::printf("      %lld triangles, %lld disagree (%.2f%%), mean %.2f u, worst %lld u",
                    checked, nonzero, checked ? 100.0 * static_cast<double>(nonzero) / static_cast<double>(checked) : 0.0,
                    checked ? sum / static_cast<double>(checked) : 0.0, worst);
        if (worst) std::printf(" at world (%lld, %lld)", worst_x, worst_z);
        std::printf("\n");

        // The size of the mistake, if the split is taken the wrong way round: at a cell's centre
        // the two splits differ by |(n00 + n11) - (n01 + n10)| * 8 world units. Printed so the
        // number the fix is worth stays visible per level.
        {
            int twist_max = 0, twisted = 0, cells = 0;
            for (int gz = 0; gz + 1 < scene.map_h; ++gz)
                for (int gx = 0; gx + 1 < scene.map_w; ++gx) {
                    auto H = [&](int a, int b) {
                        return static_cast<int>(scene.mph_data[static_cast<size_t>(b) * scene.map_w + a]);
                    };
                    const int t = std::abs((H(gx, gz) + H(gx + 1, gz + 1))
                                         - (H(gx + 1, gz) + H(gx, gz + 1)));
                    ++cells;
                    if (t) ++twisted;
                    twist_max = std::max(twist_max, t);
                }
            std::printf("      wrong split would cost: %d%% of cells, up to %d u at the centre\n",
                        cells ? twisted * 100 / cells : 0, twist_max * 8);
        }
    }

    bigred::VehicleSim car;

    // --- SCENERY ------------------------------------------------------------------------
    // "The decor has no physics, you go through the trees." The question a bench can answer is
    // not "does it look solid" but "does a car aimed at one actually stop", so: put the car a few
    // hundred units in front of a blocking object, drive it straight at the thing, and print how
    // close it got and what happened to its speed.
    //
    // Two families are measured apart because they are two different narrow phases and either
    // could regress alone: the 32-sector silhouette (trees, posts, tyres) and the oriented box
    // (fences, barriers, billboards).
    if (bloc("SCENERY")) {
        std::printf("\nSCENERY  drive straight at a blocking object, 200 ticks\n");
        std::map<std::string, std::pair<int, int>> by_kind; // name -> {box, radial}
        for (const bigred::SimObject& o : ter.blockers) {
            const char* k = o.destructible ? "destructible" : "bloc";
            if (o.use_box) ++by_kind[k].first; else ++by_kind[k].second;
        }
        for (const auto& kv : by_kind)
            std::printf("      %-13s %4d en boite, %4d en radial\n", kv.first.c_str(),
                        kv.second.first, kv.second.second);
        if (ter.blockers.empty()) {
            std::printf("      this level has none\n");
        } else {
            // The stop distance only means something next to the object's own reach along the
            // approach: stopping 55 units from the centre of a post 5 units wide is a failure,
            // stopping 340 from the centre of a 320-wide fence is exactly right.
            std::printf("\n      %-10s %-6s %-9s %7s %8s %8s %7s  %s\n", "forme", "test",
                        "role", "attendu", "arret a", "renvoi", "vit.ap", "verdict");
            std::printf("      (renvoi = de combien l'impact devie la trajectoire, en degres :"
                        " ~180 = rejete en arriere, ~90 = glisse de cote)\n");
            // --- the VOLUME, probed directly ------------------------------------------
            // "The physics volume around a sign looks smaller than the sign itself" is a
            // question about the narrow phase, and driving at the thing cannot answer it: the
            // car drifts sideways over 200 ticks and a miss reads the same as a pass-through.
            // So ask the narrow phase itself. Walk a body in from 1200 units along four
            // headings and print the distance at which contact FIRST fires, against what the
            // shape's own profile says it should be. No integration, no drift, no aim.
            {
                std::printf("      volume de collision, sonde directe (4 approches)\n");
                std::printf("      %-10s %-6s %-9s %8s %8s %8s\n", "forme", "test", "role",
                            "attendu", "mesure", "ecart");
                std::set<std::string> probed;
                int rows = 0;
                for (size_t i = 0; i < ter.blockers.size() && rows < 24; ++i) {
                    const bigred::SimObject& o = ter.blockers[i];
                    std::string nm;
                    if (o.instance_index >= 0 &&
                        static_cast<size_t>(o.instance_index) < scene.instances.size()) {
                        const int si =
                            scene.instances[static_cast<size_t>(o.instance_index)].shape_index;
                        if (si >= 0 && static_cast<size_t>(si) < scene.shape_names_by_slot.size())
                            nm = scene.shape_names_by_slot[static_cast<size_t>(si)];
                    }
                    if (!probed.insert(nm).second) continue;
                    ++rows;
                    int32_t worst_gap = 0;
                    int32_t shown_want = 0, shown_got = 0;
                    for (int dir = 0; dir < 4; ++dir) {
                        const int32_t heading = dir * 0x4000;
                        // Where the profile says contact should start, along this heading.
                        // ⚠️ The SAME sector arithmetic the contact test uses, half-sector
                        // rounding included. Three times running, the thing that was wrong in
                        // this block was the yardstick and not the physics; computing it a
                        // second, slightly different way is how that keeps happening.
                        const int32_t bearing = (heading + 0x8000) & 0xFFFF;  // body -> object
                        const int32_t car_yaw = bearing;                     // facing it
                        const int32_t h = (bearing + 0x8000 + 0x400 - o.yaw_bam16) & 0xFFFF;
                        int32_t want = tuning.box.sector_reach[static_cast<size_t>(
                            ((bearing + 0x400 - car_yaw) >> 11) & 0x1f)];
                        if (o.use_box) {
                            const int32_t sx = std::abs(bigred::fx::sin16(h));
                            const int32_t cz = std::abs(bigred::fx::cos16(h));
                            const int32_t hx = std::max(std::abs(o.box_xmin), std::abs(o.box_xmax));
                            const int32_t hz = std::max(std::abs(o.box_zmin), std::abs(o.box_zmax));
                            // ⚠️ The ray leaves the box ALREADY GROWN by the body reach, so the
                            // reach goes INSIDE each axis term. Adding it outside the min - as a
                            // separate term - is a different and larger number, and was the
                            // fourth yardstick bug in this block.
                            const int64_t tx = sx ? (static_cast<int64_t>(hx + want) << 15) / sx : INT32_MAX;
                            const int64_t tz = cz ? (static_cast<int64_t>(hz + want) << 15) / cz : INT32_MAX;
                            want = static_cast<int32_t>(std::min(tx, tz));
                        } else {
                            want += o.sector_reach[static_cast<size_t>((h >> 11) & 0x1f)];
                        }
                        // Walk in one unit at a time and find where it first reports contact.
                        int32_t got = 0, got_reach = 0;
                        for (int32_t d = 1200; d >= 1; --d) {
                            const int32_t x = o.x + ((bigred::fx::sin16(heading) * d) >> 15);
                            const int32_t z = o.z + ((bigred::fx::cos16(heading) * d) >> 15);
                            const bigred::ObjectContact c =
                                ter.blocker_contact(o, x, o.y, z, (heading + 0x8000) & 0xFFFF,
                                                    o.y, tuning.box);
                            if (d == 1200) got_reach = c.reach;
                            if (c.hit) { got = d; break; }
                        }
                        // Compare against the separation the test ITSELF used, so the column is
                        // self-consistent: for the silhouette that is the reach it summed, for
                        // the box the ray-exit distance through the grown box.
                        const int32_t ref = o.use_box ? want : got_reach;
                        if (std::abs(got - ref) > std::abs(worst_gap) || dir == 0) {
                            worst_gap = got - ref;
                            shown_want = ref;
                            shown_got = got;
                        }
                    }
                    std::printf("      %-10s %-6s %-9s %8d %8d %8d%s\n", nm.c_str(),
                                o.use_box ? "boite" : "radial",
                                o.destructible ? "destruct." : "bloc",
                                shown_want, shown_got, worst_gap,
                                shown_got == 0 ? "   AUCUN CONTACT" : "");
                }
                std::printf("\n");
            }

            // One run per SHAPE, not the first six instances of whatever the list happens to
            // start with. "The signs don't act as a wall" is a question about a shape, and six
            // rows all reading BILBORD1 cannot answer it.
            std::set<std::string> shapes_done;
            int shown = 0;
            for (size_t i = 0; i < ter.blockers.size() && shown < 24; ++i) {
                const bigred::SimObject& o = ter.blockers[i];
                std::string shape_name;
                if (o.instance_index >= 0 &&
                    static_cast<size_t>(o.instance_index) < scene.instances.size()) {
                    const int si =
                        scene.instances[static_cast<size_t>(o.instance_index)].shape_index;
                    if (si >= 0 && static_cast<size_t>(si) < scene.shape_names_by_slot.size())
                        shape_name = scene.shape_names_by_slot[static_cast<size_t>(si)];
                }
                // ⚠️ Mark the shape as done only once a run has actually been PERFORMED. Marking
                // it here, before the start-point check below, permanently drops any shape whose
                // first instance happens to sit somewhere the car cannot launch from - which is
                // how CHEVRONS and ROADSIGN went missing from this table on DIRT1, the two shapes
                // the question was about.
                if (shapes_done.count(shape_name)) continue;
                // Aim from far enough out that the car is at speed when it arrives, and along
                // +X so the approach bearing is fixed and the run is reproducible.
                const int32_t start_dist = 2600;
                bigred::RespawnPoint p;
                p.x = o.x - start_dist;
                p.z = o.z;
                p.yaw_bam16 = 0x4000; // +X in the game's frame
                car.reset(tuning, ter, rules, p, nitros);
                settle(car, 40);
                // Only keep runs that really do start clear of the object, otherwise the "stop
                // distance" measures nothing.
                const int32_t d0 = std::abs((car.px >> 9) - o.x);
                if (d0 < 600) continue;
                shapes_done.insert(shape_name);
                bigred::VehicleInput go;
                go.accel = true;
                // ⚠️ Measure the APPROACH, and stop measuring at the first contact.
                // Carrying on to the end of the run mixes two different things: how close the
                // car gets before the object stops it, and how far it then slides ALONG it. On a
                // row of identical barriers the slide takes the car past the end of the one it
                // hit and right down the flank of the next, so the whole-run minimum reads as a
                // deep penetration when nothing was penetrated at all. The distance at the tick
                // the contact first fires has no such ambiguity.
                int32_t closest = d0;
                bool touched = false;
                // ⚠️ The heading BEFORE the contact tick. Reading it after the loop breaks gets
                // the post-impact heading, so the deflection measures nothing at all and every
                // row reads 0 degrees - which is how a working rebound looked like no rebound.
                int32_t travel_before = car.travel;
                int32_t speed_before = car.speed;
                for (int t = 0; t < 200 && !touched; ++t) {
                    travel_before = car.travel;
                    speed_before = car.speed;
                    car.step(go);
                    const int32_t dx = (car.px >> 9) - o.x, dz = (car.pz >> 9) - o.z;
                    const int32_t d = static_cast<int32_t>(
                        std::llround(std::sqrt(static_cast<double>(dx) * dx +
                                               static_cast<double>(dz) * dz)));
                    if (d < closest) closest = d;
                    // Only a contact with THIS object counts - the run passes others on the way.
                    if (car.touched_instance(o.instance_index))
                        touched = true;
                }
                // What the object's own profile says the car should be held at, approaching along
                // -X: the object's widest reach plus the body's reach toward it.
                // ⚠️ It has to be the reach along THIS approach, not the widest one. A billboard
                // is 280 wide and 10 thick: hit it edge-on and stopping 60 units from its centre
                // is correct, not a failure. The run comes in along world +X, which is heading
                // 0x4000 in the game's axes (x = sin, z = cos), so in the object's own frame it
                // arrives on heading 0x4000 - yaw.
                const int32_t body_reach = tuning.box.sector_reach[8];
                const int32_t h = (0x4000 - o.yaw_bam16) & 0xFFFF;
                int32_t want = body_reach;
                if (o.use_box) {
                    // ⚠️ Where the RAY leaves the box, not the box's support function along h.
                    // For a rotated box those differ, and the support one is always the larger:
                    // using it made a correct stop read as ENFONCE on every angled BARRIER of
                    // CITY1. The ray from the centre leaves through whichever face it reaches
                    // first, so it is a min of the two axis crossings.
                    const int32_t sx = std::abs(bigred::fx::sin16(h));
                    const int32_t cz = std::abs(bigred::fx::cos16(h));
                    const int32_t hx = std::max(std::abs(o.box_xmin), std::abs(o.box_xmax));
                    const int32_t hz = std::max(std::abs(o.box_zmin), std::abs(o.box_zmax));
                    const int64_t tx = sx ? (static_cast<int64_t>(hx) << 15) / sx : INT32_MAX;
                    const int64_t tz = cz ? (static_cast<int64_t>(hz) << 15) / cz : INT32_MAX;
                    want += static_cast<int32_t>(std::min(tx, tz));
                } else {
                    want += o.sector_reach[static_cast<size_t>((h >> 11) & 0x1f)];
                }
                const std::string& name = shape_name;
                // A run that never got going says nothing about collision: the synthetic start
                // point can land in water, on a cliff or off the map. Call that out rather than
                // let it read as "drove through".
                // What the impact DID to the direction of travel - the thing the user judges.
                // The approach is along +X; a real rebound sends the car back the way it came, a
                // deflection near 180 degrees. Near 0 means it carried straight on, and near 90
                // means it was slid sideways along the obstacle - which is exactly what a
                // mirrored contact bearing used to produce.
                int32_t deflect = 0, speed_after = car.speed;
                if (touched) {
                    deflect = (car.travel - travel_before) & 0xFFFF;
                    if (deflect > 0x8000) deflect -= 0x10000;
                    if (deflect < 0) deflect = -deflect;
                }
                (void)speed_before;
                const int32_t travelled = d0 - closest;
                // Never reaching the target is only a failure if the car got close enough to
                // have touched it. Stopping 1200 units short means something ELSE stopped the
                // car on the way in - which is a working collision, not a missing one.
                const char* verdict =
                    travelled < 500           ? "n'a pas pu s'elancer"
                    : (!touched && closest > want * 3) ? "arrete plus tot par un autre"
                    : !touched                ? "TRAVERSE - jamais touche"
                    : o.destructible          ? "renverse"
                    : closest >= want * 3 / 4 ? "arrete"
                                              : "ENFONCE";
                std::printf("      %-10s %-6s %-9s %7d %8d %8.0f %7d  %s\n",
                            name.c_str(), o.use_box ? "boite" : "radial",
                            o.destructible ? "destruct." : "bloc",
                            want, closest, deg16(deflect), speed_after, verdict);
                (void)start_dist;
                ++shown;
            }
        }
    }

    // --- does ONE tick clear a penetration? -----------------------------------
    // "With a single isolated board I go slightly through it and then the physics seems
    // to catch up." (user, 2026-09-16) That is one question and it has a number: put the
    // body deliberately inside an object, run ONE tick, and see what is left.
    //
    // ⚠️ Set it up rather than drive into it. Driving at a synthetic target misses - the
    // car drifts on the terrain, is deflected by something else, or never launches - and
    // a miss reads exactly like a pass-through. Every earlier attempt in this block to
    // measure by driving produced rows that meant nothing.
    //
    // The push-out is penetration + 4, so ONE tick has to leave zero. Anything else and
    // the body crawls out over several ticks, which is what the user is seeing.
    if (bloc("SCENERY")) {
        std::printf("\n      un tick suffit-il a degager une penetration ?\n");
        std::printf("      %-10s %-6s %8s %8s %8s  %s\n", "forme", "test", "avant",
                    "apres", "normale", "verdict");
        std::set<std::string> done;
        int rows = 0;
        for (size_t i = 0; i < ter.blockers.size() && rows < 8; ++i) {
            const bigred::SimObject& o = ter.blockers[i];
            if (o.destructible) continue;   // a destructible returns no push by design
            std::string nm;
            if (o.instance_index >= 0 &&
                static_cast<size_t>(o.instance_index) < scene.instances.size()) {
                const int si =
                    scene.instances[static_cast<size_t>(o.instance_index)].shape_index;
                if (si >= 0 && static_cast<size_t>(si) < scene.shape_names_by_slot.size())
                    nm = scene.shape_names_by_slot[static_cast<size_t>(si)];
            }
            if (done.count(nm)) continue;

            // Drop the body a little off the object's centre, along its own +X, so the
            // overlap is real but not degenerate.
            bigred::RespawnPoint p;
            p.x = o.x + ((bigred::fx::sin16(o.yaw_bam16 + 0x4000) * 40) >> 15);
            p.z = o.z + ((bigred::fx::cos16(o.yaw_bam16 + 0x4000) * 40) >> 15);
            p.yaw_bam16 = o.yaw_bam16;
            car.reset(tuning, ter, rules, p, nitros);
            car.py = (o.y << 9);            // level with the object, not falling onto it
            car.vy = 0;

            auto depth_now = [&]() {
                return ter.blocker_contact(o, car.px >> 9, car.py >> 9, car.pz >> 9,
                                           car.yaw >> 8,
                                           ter.height_fx(car.px, car.pz) >> 9,
                                           tuning.box);
            };
            const bigred::ObjectContact before = depth_now();
            if (!before.hit) continue;      // not actually overlapping: nothing to test
            // ⚠️ ALONE. A shape built in pieces - BILBORD2 is half a board bolted to a
            // BILBORD1, BAY_POLE holds up a BAY_ROOF - puts a body set on its centre
            // inside TWO objects, and getting out of one is getting into the other: no
            // engine resolves that, and the row read as a push bug (2026-09-27). Another
            // instance of the same shape, standing on its own, is what gets tested.
            {
                bigred::ObjectContact cs[4];
                if (ter.blocker_probe(p.x, o.y, p.z, p.yaw_bam16,
                                      ter.height_fx(p.x << 9, p.z << 9) >> 9, tuning.box,
                                      cs, 4) > 1)
                    continue;
            }
            done.insert(nm);
            const bigred::VehicleInput idle;
            car.step(idle);
            const bigred::ObjectContact after = depth_now();
            ++rows;
            std::printf("      %-10s %-6s %8d %8d %8.0f  %s", nm.c_str(),
                        o.use_box ? "boite" : "radial", before.depth, after.depth,
                        deg16(before.normal_bam16),
                        after.hit ? "TOUJOURS DEDANS" : "degage");
            if (o.use_box)
                std::printf("  (boite x %d..%d z %d..%d, cap %d ; voiture deplacee de "
                            "%d,%d ; normale apres %.0f)",
                            o.box_xmin, o.box_xmax, o.box_zmin, o.box_zmax, o.yaw_bam16,
                            (car.px >> 9) - p.x, (car.pz >> 9) - p.z,
                            deg16(after.normal_bam16));
            std::printf("\n");
            // Every object the body overlaps at that spot, not only the one tested: a
            // body set inside two of them is pushed out of one INTO the other.
            if (after.hit) {
                bigred::VehicleSim probe;
                probe.reset(tuning, ter, rules, p, nitros);
                bigred::ObjectContact cs[8];
                const int k = ter.blocker_probe(p.x, o.y, p.z, p.yaw_bam16,
                                                ter.height_fx(p.x << 9, p.z << 9) >> 9,
                                                tuning.box, cs, 8);
                for (int q = 0; q < k; ++q) {
                    std::string on;
                    const int ii = cs[q].object ? cs[q].object->instance_index : -1;
                    if (ii >= 0 && static_cast<size_t>(ii) < scene.instances.size()) {
                        const int si = scene.instances[static_cast<size_t>(ii)].shape_index;
                        if (si >= 0 && static_cast<size_t>(si) < scene.shape_names_by_slot.size())
                            on = scene.shape_names_by_slot[static_cast<size_t>(si)];
                    }
                    std::printf("          au depart, contact %d : %-9s instance %d, "
                                "enfoncement %d, normale %.0f\n",
                                q, on.c_str(), ii, cs[q].depth, deg16(cs[q].normal_bam16));
                }
            }
        }
        if (rows == 0) std::printf("      (aucun chevauchement exploitable ici)\n");
    }

    // --- ROULER SUR UN OBJET DEJA RENVERSE ------------------------------------
    // "The impact sound plays in a LOOP when the car is on the grass." On CITY4 the
    // grass is carpeted with destructible plants, so what the car is really doing is
    // driving across props it has already flattened. That has to produce exactly ONE
    // splat and then nothing at all: no further wreck event, and - this is the part
    // that broke - no COLLISION either, because the original's destructible cases
    // never touch the collision flag.
    //
    // Counted here as the audio counts it, because that is where it was heard.
    if (bloc("SCENERY")) {
        std::printf("\n      rouler sur un objet destructible, 120 ticks\n");
        const bigred::SimObject* prop = nullptr;
        for (const bigred::SimObject& o : ter.blockers)
            if (o.destructible) { prop = &o; break; }
        if (!prop) {
            std::printf("      (ce circuit n'a pas de destructible)\n");
        } else {
            bigred::SceneryState sworld;
            sworld.resize(scene.instances.size());
            bigred::RespawnPoint p;
            p.x = prop->x - 900;
            p.z = prop->z;
            p.yaw_bam16 = 0x4000;
            car.reset(tuning, ter, rules, p, nitros);
            car.world = &sworld;
            settle(car, 20);
            bigred::VehicleInput go;
            go.accel = true;
            int wreck_events = 0, collision_ticks = 0, bang_ticks = 0, over_ticks = 0;
            for (int t = 0; t < 120; ++t) {
                car.step(go);
                // ⚠️ ONLY the target's own event. The run crosses other props on the
                // way, and counting those too turns "this one screamed twice" into a
                // number that cannot be read.
                if (car.scenery_wrecked == prop->instance_index) ++wreck_events;
                if (car.touched_instance(prop->instance_index)) ++over_ticks;
                if (car.scenery_hit) ++collision_ticks;
                // The audio's own test, verbatim - see VehicleAudio::on_tick.
                if (car.scenery_wrecked < 0 && car.scenery_hit &&
                    car.impact_magnitude > 0x5db)
                    ++bang_ticks;
            }
            car.world = nullptr;
            std::printf("      %d tick(s) en contact avec lui, %d ecrasement(s),"
                        " %d tick(s) de collision, %d bang(s) d'impact\n",
                        over_ticks, wreck_events, collision_ticks, bang_ticks);
            std::printf("      attendu : 1 ecrasement, et 0 bang tant qu'aucun mur"
                        " n'est touche\n");
        }
    }

    // --- DESTRUCT -----------------------------------------------------------------------
    // Knocking a prop over has to be VISIBLE, and "visible" is two load-time facts plus one
    // run-time one: the instance has a second baked body, the flipbook has the steps the
    // original's counter walks, and a contact actually flips the shared state. All three are
    // measurable with no window, which is the point - the screenshot only tells you whether it
    // looks right, never whether the wiring is there.
    if (bloc("DESTRUCT")) {
        std::printf("\nDESTRUCT  decor destructible, etats cuits\n");
        int destructible_instances = 0;
        for (const bigred::SimObject& o : ter.blockers)
            if (o.destructible) ++destructible_instances;
        size_t intact_v = 0, settled_v = 0;
        for (const bigred::LevelScene::WreckPiece& wp : scene.wrecks) {
            intact_v += static_cast<size_t>(wp.intact.second);
            settled_v += static_cast<size_t>(wp.steps.back().second);
        }
        std::printf("      %d instance(s) destructible(s), %zu avec un second corps\n",
                    destructible_instances, scene.wrecks.size());
        std::printf("      %zu sommets intacts -> %zu abimes, dans un tampon de %zu\n",
                    intact_v, settled_v, scene.wreck_buf.size());

        // Per shape: how many distinct baked states the flipbook really has. One means the prop
        // just lies down; ten is a destruction animation. Zero faces settled means what is left
        // of it is the sprite - the case that would read as "the object vanished" if the sprites
        // were not drawn.
        std::map<std::string, std::array<int, 5>> by_shape; // n, intact_v, settled_v, states, sprites
        for (const bigred::LevelScene::WreckPiece& wp : scene.wrecks) {
            const bigred::SceneryInstance& inst =
                scene.instances[static_cast<size_t>(wp.instance)];
            std::string nm = "?";
            if (inst.shape_index >= 0 &&
                static_cast<size_t>(inst.shape_index) < scene.shape_names_by_slot.size())
                nm = scene.shape_names_by_slot[static_cast<size_t>(inst.shape_index)];
            std::set<std::pair<int, int>> states(wp.steps.begin(), wp.steps.end());
            std::array<int, 5>& row = by_shape[nm];
            ++row[0];
            row[1] = wp.intact.second;
            row[2] = wp.steps.back().second;
            row[3] = static_cast<int>(states.size());
        }
        for (const bigred::SpritePlacement& sp : scene.sprites) {
            if (sp.wreck < 0 || static_cast<size_t>(sp.wreck) >= scene.wrecks.size()) continue;
            const bigred::SceneryInstance& inst =
                scene.instances[static_cast<size_t>(scene.wrecks[static_cast<size_t>(sp.wreck)].instance)];
            if (inst.shape_index < 0 ||
                static_cast<size_t>(inst.shape_index) >= scene.shape_names_by_slot.size()) continue;
            ++by_shape[scene.shape_names_by_slot[static_cast<size_t>(inst.shape_index)]][4];
        }
        if (by_shape.empty()) {
            std::printf("      (ce circuit n'en a aucun)\n");
        } else {
            std::printf("\n      %-10s %5s %8s %8s %7s %8s\n", "forme", "inst", "intact",
                        "abime", "etats", "sprites");
            for (const auto& kv : by_shape) {
                std::printf("      %-10s %5d %8d %8d %7d %8d\n", kv.first.c_str(), kv.second[0],
                            kv.second[1], kv.second[2], kv.second[3], kv.second[4]);
            }
        }

        // And the wiring, end to end: put the body inside a destructible, run ONE tick, and ask
        // the shared state - not the renderer, not a heuristic - whether the thing is down.
        // Set it up rather than drive at it: a miss reads exactly like a failure to trigger
        // (the method lesson).
        {
            std::printf("\n      un contact suffit-il a renverser ? (pose + 1 tick)\n");
            std::printf("      %-10s %8s %8s  %s\n", "forme", "avant", "apres", "verdict");
            bigred::SceneryState world;
            world.resize(scene.instances.size());
            car.world = &world;
            std::set<std::string> done;
            int rows = 0;
            for (const bigred::SimObject& o : ter.blockers) {
                if (!o.destructible || rows >= 8) continue;
                std::string nm = "?";
                if (o.instance_index >= 0 &&
                    static_cast<size_t>(o.instance_index) < scene.instances.size()) {
                    const int si =
                        scene.instances[static_cast<size_t>(o.instance_index)].shape_index;
                    if (si >= 0 && static_cast<size_t>(si) < scene.shape_names_by_slot.size())
                        nm = scene.shape_names_by_slot[static_cast<size_t>(si)];
                }
                if (!done.insert(nm).second) continue;

                bigred::RespawnPoint p;
                p.x = o.x + ((bigred::fx::sin16(o.yaw_bam16 + 0x4000) * 40) >> 15);
                p.z = o.z + ((bigred::fx::cos16(o.yaw_bam16 + 0x4000) * 40) >> 15);
                p.yaw_bam16 = o.yaw_bam16;
                car.reset(tuning, ter, rules, p, nitros);
                car.world = &world;
                car.py = (o.y << 9);
                car.vy = 0;
                const bool before = world.is_wrecked(o.instance_index);
                const bigred::VehicleInput idle;
                car.step(idle);
                const bool after = world.is_wrecked(o.instance_index);
                ++rows;
                std::printf("      %-10s %8s %8s  %s\n", nm.c_str(), before ? "abime" : "intact",
                            after ? "abime" : "intact",
                            after ? "renverse" : "PAS DE DECLENCHEMENT");
            }
            if (rows == 0) std::printf("      (aucun destructible a portee sur ce circuit)\n");

            // The flipbook ITSELF, as the renderer will read it: per state, which sprite
            // records are on and WHICH RECTANGLE of the sheet each one shows, in raw `.TM`
            // texels. This is the only view that tells a real filmstrip from a plausible-looking
            // one - a wrong crop draws perfectly happily, it just shows something that is not the
            // object. States that repeat the previous one are folded, so a 33-step flipbook
            // prints as the handful of pictures it really has.
            std::printf("\n      pellicule sprite, une instance par forme (rect en texels .TM)\n");
            std::map<std::string, int> first_wreck;  // shape -> a wrecks[] index to sample
            for (size_t w = 0; w < scene.wrecks.size(); ++w) {
                const bigred::SceneryInstance& wi =
                    scene.instances[static_cast<size_t>(scene.wrecks[w].instance)];
                if (wi.shape_index < 0 ||
                    static_cast<size_t>(wi.shape_index) >= scene.shape_names_by_slot.size())
                    continue;
                first_wreck.emplace(scene.shape_names_by_slot[static_cast<size_t>(wi.shape_index)],
                                    static_cast<int>(w));
            }
            int shapes_with_sprites = 0;
            for (const auto& kv : first_wreck) {
                std::vector<const bigred::SpritePlacement*> recs;
                for (const bigred::SpritePlacement& sp : scene.sprites)
                    if (sp.wreck == kv.second) recs.push_back(&sp);
                if (recs.empty()) continue;
                ++shapes_with_sprites;
                std::printf("      %s : %zu record(s), planche = couche %.0f\n", kv.first.c_str(),
                            recs.size(), recs[0]->layer);
                std::string prev;
                for (int k = 0; k <= bigred::LevelScene::kWreckStepCount; ++k) {
                    std::string line;
                    for (size_t r = 0; r < recs.size(); ++r) {
                        if (static_cast<size_t>(k) >= recs[r]->frames.size()) continue;
                        const bigred::SpritePlacement::Frame& f =
                            recs[r]->frames[static_cast<size_t>(k)];
                        if (!f.on) continue;
                        char buf[96];
                        std::snprintf(buf, sizeof buf, " #%zu c%d (%.0f,%.0f)-(%.0f,%.0f)", r,
                                      f.layer >= 0 ? f.layer : static_cast<int>(recs[r]->layer),
                                      f.u0 * bigred::kTmWidth, f.v0 * bigred::kTmHeight,
                                      f.u1 * bigred::kTmWidth, f.v1 * bigred::kTmHeight);
                        line += buf;
                    }
                    if (line == prev) continue;
                    prev = line;
                    if (k == 0)
                        std::printf("        intact  :%s\n",
                                    line.empty() ? " (aucun)" : line.c_str());
                    else
                        std::printf("        pas %-3d :%s\n", k - 1,
                                    line.empty() ? " (aucun)" : line.c_str());
                }
            }
            if (shapes_with_sprites == 0)
                std::printf("      (aucune forme a sprites sur ce circuit)\n");

            // The flipbook as a duration, not a tick count - that is how the renderer reads it.
            std::printf("      la pellicule : %d pas de %d ms, soit %d ms avant l'etat fige\n",
                        bigred::LevelScene::kWreckStepCount - 1, bigred::kGameTickMs,
                        (bigred::LevelScene::kWreckStepCount - 1) * bigred::kGameTickMs);
            car.world = nullptr;
        }
    }

    // --- DEGATS -------------------------------------------------------------------------
    // "The car is wrecked far too fast: in the original a head-on hit at full speed does barely
    // 10% damage, here it is instant." That is one number - what a single full-speed frontal
    // impact costs - and it is measurable: drive flat out into a blocking object and read the
    // gauge at the tick the contact fires.
    //
    // ⚠️ Read the MAGNITUDE column with it. `damage += magnitude >> shift` and the magnitude IS
    // the speed going into the surface, so a run that arrives at half speed reports half the
    // damage and says nothing about a full-speed crash. A row whose magnitude is well under the
    // vehicle's top speed was slowed on the way in and is not the measurement.
    if (bloc("DEGATS")) {
        // The gauge is set by reset(): read it from a fresh car, not from the previous block's.
        car.reset(tuning, ter, rules, grid, nitros);
        std::printf("\nDEGATS  un choc frontal a pleine vitesse, sur objet bloquant\n");
        std::printf("        jauge : %d (= `hnd damage` %d, option degats %d%%, defaut TRACE)\n",
                    car.damage_capacity ? car.damage_capacity : tuning.damage, tuning.damage,
                    bigred::VehicleSim::kDefaultDamageOption);
        int rows = 0;
        std::set<std::string> done;
        for (const bigred::SimObject& o : ter.blockers) {
            if (o.destructible || o.ramp_gate) continue;   // has to be a wall, not a prop
            if (rows >= 3) break;
            std::string name = "?";
            if (o.instance_index >= 0 &&
                static_cast<size_t>(o.instance_index) < scene.instances.size()) {
                const int si = scene.instances[static_cast<size_t>(o.instance_index)].shape_index;
                if (si >= 0 && static_cast<size_t>(si) < scene.shape_names_by_slot.size())
                    name = scene.shape_names_by_slot[static_cast<size_t>(si)];
            }
            if (!done.insert(name).second) continue;
            bigred::RespawnPoint p;
            p.x = o.x - 2600;
            p.z = o.z;
            p.yaw_bam16 = 0x4000;               // straight at it along +X
            car.reset(tuning, ter, rules, p, nitros);
            settle(car, 40);
            if (std::abs((car.px >> 9) - o.x) < 600) continue;
            bigred::VehicleInput go;
            go.accel = true;
            int32_t before = 0, mag = 0, speed_in = 0;
            bool touched = false;
            for (int t = 0; t < 200 && !touched; ++t) {
                before = car.damage;
                speed_in = car.speed;
                car.step(go);
                if (car.touched_instance(o.instance_index)) touched = true;
            }
            if (!touched) continue;
            mag = car.impact_magnitude;
            // The cabin head dip this contact throws (VehicleSim::head_kick, BAM16; > 0 = down).
            std::printf("        plongee de la tete : %d (%.1f deg), vfwd apres le rebond %d\n",
                        car.head_kick, car.head_kick * 360.0 / 65536.0, car.vfwd);
            const int32_t added = (car.damage < 0) ? (car.damage_capacity - before) : (car.damage - before);
            const int32_t cap = car.damage_capacity > 0 ? car.damage_capacity : 1;
            std::printf("        %-10s vitesse %6d  magnitude %6d  degats +%-4d = %3d%% de la"
                        " jauge  -> %d chocs pour detruire\n",
                        name.c_str(), speed_in, mag, added, added * 100 / cap,
                        added > 0 ? (cap + added - 1) / added : 0);
            // The body's PITCH after the first crash, which the cabin view follows at 7/8:
            // in the original the view dips and comes back in about a
            // second - the user, 2026-09-29. Degrees, every 4 ticks, throttle released.
            // The head dip as the cabin view accumulates it (RaceScreen::follow: minus each
            // tick's kick, then back by 0x40), throttle HELD against the wall - the offset from
            // rest in BAM16, > 0 = down, every 2 ticks.
            if (rows == 0) {
                bigred::VehicleSim held = car;
                int32_t dip = held.head_kick - 0x40;
                std::printf("          plongee, accel tenu (tous les 2 ticks) :");
                for (int t = 0; t < 40; ++t) {
                    if (t % 2 == 0) std::printf(" %d", dip);
                    held.step(go);
                    dip += held.head_kick;
                    dip -= std::clamp(dip, -0x40, 0x40);
                }
                std::printf("\n");
            }
            if (rows == 0) {
                std::printf("          tangage apres le choc (deg, tous les 4 ticks) :");
                const bigred::VehicleInput idle;
                for (int t = 0; t < 60; ++t) {
                    if (t % 4 == 0) std::printf(" %.1f", deg24(car.pitch));
                    car.step(idle);
                }
                std::printf("\n");
            }
            ++rows;
        }
        if (rows == 0) std::printf("        aucun objet bloquant atteignable sur ce circuit\n");
        std::printf("        (l'original ferait `magnitude >> %d`, soit %d fois plus - voir"
                    " VehicleSim::kImpactDamageShift)\n",
                    bigred::VehicleSim::kOriginalImpactDamageShift,
                    1 << (bigred::VehicleSim::kImpactDamageShift -
                          bigred::VehicleSim::kOriginalImpactDamageShift));
    }

    // --- TILTX --------------------------------------------------------------------------
    // "The `tilter x` seems to fire on almost every tick under throttle: it
    // zeroes the pitch rate, forces a dive and clears both snap flags." Counted here rather
    // than concluded: flat out from the grid for 200 ticks, how often it takes over, and what
    // the body's pitch does meanwhile (degrees against the terrain's own).
    if (bloc("TILTX")) {
        std::printf("\nTILTX  plein gaz depuis la grille, 200 ticks\n");
        std::printf("        tilter x : mode %d a=%d b=%d c=%d\n", tuning.tilter_x.mode,
                    tuning.tilter_x.a, tuning.tilter_x.b, tuning.tilter_x.c);
        car.reset(tuning, ter, rules, grid, nitros);
        settle(car, 40);
        bigred::VehicleInput go;
        go.accel = true;
        int fired = 0, grounded = 0, accelerating = 0;
        double sum_dev = 0.0, worst_dev = 0.0;
        for (int t = 0; t < 200; ++t) {
            car.step(go);
            if (car.tilter_x_fired) ++fired;
            if (car.near_ground != 0) ++grounded;
            if (car.accel_delta > 0) ++accelerating;
            const double dev = (car.pitch / 256.0 - car.terrain_pitch) * 360.0 / 65536.0;
            sum_dev += dev;
            if (std::abs(dev) > std::abs(worst_dev)) worst_dev = dev;
        }
        std::printf("        agit %d tick(s) sur 200 (au sol %d, en acceleration %d) ; tangage moyen %+.2f deg,"
                    " pire %+.2f deg par rapport au terrain\n",
                    fired, grounded, accelerating, sum_dev / 200.0, worst_dev);
    }

    // --- EPAVE --------------------------------------------------------------------------
    // "What happens to a destroyed player?" (user, 2026-09-27.) In the original a wreck takes
    // no orders at all - the original zeroes the six controls when the damage is negative - so it
    // coasts to a stop, ALT does nothing, and nothing repairs it. Destroyed on the grid at full
    // throttle here: the speed must die away, ALT must not move it, the damage must stay.
    if (bloc("EPAVE")) {
        std::printf("\nEPAVE  une voiture detruite, plein gaz et ALT\n");
        car.reset(tuning, ter, rules, grid, nitros);
        settle(car, 40);
        bigred::VehicleInput go;
        go.accel = true;
        for (int t = 0; t < 60; ++t) car.step(go);
        const int32_t speed_alive = car.speed;
        car.damage = -1;   // what take_impact_damage leaves at 100%
        car.damage_percent = 100;
        for (int t = 0; t < 150; ++t) car.step(go);
        const int32_t speed_dead = car.speed;
        const uint32_t respawns = car.respawns;
        go.recover = true;
        for (int t = 0; t < 5; ++t) car.step(go);
        const bool stays = car.respawns == respawns && car.damage < 0;
        std::printf("        vitesse vivante %d, 150 ticks plus tard %d  %s\n", speed_alive, speed_dead,
                    std::abs(speed_dead) < std::abs(speed_alive) / 4 ? "ok" : "<-- ROULE ENCORE");
        std::printf("        ALT : %s\n", stays ? "ignore, toujours detruite - ok" : "<-- REMISE EN PISTE");
        car.reset(tuning, ter, rules, grid, nitros);
        std::printf("        nouvelle course : degats %d%%  %s\n", car.damage_percent,
                    car.damage == 0 ? "ok" : "<-- PAS REPAREE");
    }

    // --- CYLINDREE ----------------------------------------------------------------------
    // The speed class, "Mario Kart CC" (user, 2026-09-30; RaceRules::speed_percent): the gear
    // table stretched along both axes. From the grid, full throttle: the speed after 40 ticks
    // (the acceleration) and the best reached in 150 (the top speed, if the road allows it)
    // must both grow with the class; the ghosts' playback rate with it. 100CC is the original.
    if (bloc("CYLINDREE")) {
        std::printf("\nCYLINDREE  plein gaz depuis la grille, par classe\n");
        for (const bigred::SpeedClass& sc : bigred::kSpeedClasses) {
            bigred::RaceRules r = rules;
            r.speed_percent = sc.percent;
            car.reset(tuning, ter, r, grid, nitros);
            settle(car, 40);
            bigred::VehicleInput go;
            go.accel = true;
            int32_t at40 = 0, best = 0;
            for (int t = 0; t < 150; ++t) {
                car.step(go);
                if (t == 39) at40 = car.vfwd;
                best = std::max(best, car.vfwd);
            }
            // The dial's own calibration, as the HUD reads it: 120 at a vfwd of 35000.
            std::printf("  %-6s %3d %%  vfwd a 40 ticks %6d, max %6d (%3d km/h)  fantomes x%.2f\n",
                        sc.name, sc.percent, at40, best,
                        static_cast<int>(static_cast<int64_t>(best) * 120 / 35000),
                        sc.percent / 100.0);
        }
        car.reset(tuning, ter, rules, grid, nitros);
    }

    // --- BANG ---------------------------------------------------------------------------
    // "The standard impact sound plays in a LOOP - against a wall, a tree, or even falling from
    // height - but only on the grass texture." A looping one-shot is always one of two things:
    // a voice that never ends, or a TRIGGER that fires every tick. This block settles which, by
    // driving into a wall from a grass cell and then printing, tick by tick, every input the
    // audio's own triggers read.
    //
    // ⚠️ The surface matters to the PHYSICS, not to `hit 0` - that entry is not indexed by the
    // ground type. So if the loop is surface-dependent, what changes with the surface is
    // whether the car SEPARATES from what it hit: `tune->bounce[land]` and `tune->damp[land]`
    // are per-surface, and a car that never separates re-collides on every tick.
    if (bloc("BANG")) {
        std::printf("\nBANG  choc depuis une case d'herbe, puis 40 ticks\n");
        // Find a blocking object standing on the grass, so the run reproduces the report
        // exactly rather than something adjacent to it.
        // ⚠️ Try candidates until one is actually REACHED. The first off-track blocker on the
        // list is usually unreachable from a synthetic start - it sits on a cliff, in water or
        // behind something else - and a run that never arrives reads exactly like a run that
        // found nothing wrong.
        const bigred::SimObject* target = nullptr;
        int32_t grass_land = -1;
        {
            bigred::VehicleSim scout;
            bigred::VehicleInput go;
            go.accel = true;
            int tried = 0;
            for (const bigred::SimObject& o : ter.blockers) {
                if (o.destructible || o.ramp_gate) continue;
                const int32_t l = ter.land_type((o.x - 900) << 9, o.z << 9);
                if (l == 0 || l == 7) continue;     // 0 is the track, 7 is AIR
                if (++tried > 120) break;
                bigred::RespawnPoint p;
                p.x = o.x - 2600;
                p.z = o.z;
                p.yaw_bam16 = 0x4000;
                scout.reset(tuning, ter, rules, p, nitros);
                settle(scout, 40);
                if (std::abs((scout.px >> 9) - o.x) < 600) continue;
                for (int t = 0; t < 200; ++t) {
                    scout.step(go);
                    if (scout.touched_instance(o.instance_index)) { target = &o; break; }
                }
                if (target) { grass_land = l; break; }
            }
        }
        if (!target) {
            std::printf("      (aucun objet bloquant hors piste atteignable sur ce circuit)\n");
        } else {
            std::string nm = "?";
            if (target->instance_index >= 0 &&
                static_cast<size_t>(target->instance_index) < scene.instances.size()) {
                const int si =
                    scene.instances[static_cast<size_t>(target->instance_index)].shape_index;
                if (si >= 0 && static_cast<size_t>(si) < scene.shape_names_by_slot.size())
                    nm = scene.shape_names_by_slot[static_cast<size_t>(si)];
            }
            std::printf("      cible %s, sol de type %d ; `bounce` = %d,%d,%d,%d\n", nm.c_str(),
                        grass_land, static_cast<int>(static_cast<int16_t>(tuning.bounce[grass_land & 7][0])),
                        static_cast<int>(static_cast<int16_t>(tuning.bounce[grass_land & 7][1])),
                        static_cast<int>(static_cast<int16_t>(tuning.bounce[grass_land & 7][2])),
                        static_cast<int>(static_cast<int16_t>(tuning.bounce[grass_land & 7][3])));
            bigred::SceneryState sworld;
            sworld.resize(scene.instances.size());
            bigred::RespawnPoint p;
            p.x = target->x - 2600;
            p.z = target->z;
            p.yaw_bam16 = 0x4000;
            car.reset(tuning, ter, rules, p, nitros);
            car.world = &sworld;
            settle(car, 40);
            bigred::VehicleInput go;
            go.accel = true;
            bool touched = false;
            for (int t = 0; t < 200 && !touched; ++t) {
                car.step(go);
                if (car.touched_instance(target->instance_index)) touched = true;
            }
            (void)touched;
            // ⚠️ AND NOW THE PART THAT MATTERS: keep driving. The report is not about the tick
            // of the crash, it is about everything after it. Steering gently keeps the car
            // moving over the same off-track ground instead of settling against one object.
            std::printf("      %5s %5s %5s %9s %10s %9s %8s  %s\n", "tick", "sol", "sol0",
                        "collision", "magnitude", "atterr.", "vitesse", "ce que l'audio jouerait");
            int bangs = 0, lands = 0, splats = 0, shown_rows = 0;
            std::map<std::string, int> bang_by_shape;
            for (int t = 0; t < 600; ++t) {
                go.right = ((t / 90) % 2) == 0;
                go.left = !go.right;
                car.step(go);
                const char* fires = nullptr;
                if (car.scenery_wrecked >= 0) { fires = "splat"; ++splats; }
                else if (car.scenery_hit && car.impact_magnitude > 0x5db) {
                    fires = "BANG (hit 0)";
                    ++bangs;
                    // WHICH object is banging is the whole question - a shape that should not
                    // be solid at all reads exactly like a contact that will not separate.
                    for (int k = 0; k < car.scenery_hit_count; ++k) {
                        const int ii = car.scenery_hit_instances[k];
                        std::string s = "?";
                        if (ii >= 0 && static_cast<size_t>(ii) < scene.instances.size()) {
                            const int si = scene.instances[static_cast<size_t>(ii)].shape_index;
                            if (si >= 0 &&
                                static_cast<size_t>(si) < scene.shape_names_by_slot.size())
                                s = scene.shape_names_by_slot[static_cast<size_t>(si)];
                        }
                        ++bang_by_shape[s];
                    }
                }
                if (car.landing_impact / 2 > 3999) { fires = "land"; ++lands; }
                if (fires && shown_rows < 20) {
                    ++shown_rows;
                    // `sol0` is the ground type under the body centre, BEFORE the per-corner
                    // rewrites (6 on a drivable object, 7 in the air) - the one the player
                    // calls "the texture".
                    std::printf("      %5d %5d %5d %9d %10d %9d %8d  %s\n", t, car.land,
                                ter.land_type(car.px, car.pz), car.scenery_hit ? 1 : 0,
                                car.impact_magnitude, car.landing_impact, car.speed, fires);
                }
            }
            std::printf("      sur 600 ticks hors piste : %d bang(s), %d ecrasement(s),"
                        " %d atterrissage(s)\n", bangs, splats, lands);
            if (!bang_by_shape.empty()) {
                std::printf("      formes qui font BANG :");
                for (const auto& kv : bang_by_shape)
                    std::printf(" %s x%d", kv.first.c_str(), kv.second);
                std::printf("\n");
            }
            car.world = nullptr;
        }
    }

    // --- REST ---------------------------------------------------------------------------
    if (bloc("REST")) {
        car.reset(tuning, ter, rules, grid);
        settle(car, 400);
        const int32_t x0 = car.px, y0 = car.py, z0 = car.pz;
        settle(car, 60);
        std::printf("REST  drift over 60 ticks  dx=%-7d dy=%-7d dz=%-7d (1/512 units)\n",
                    car.px - x0, car.py - y0, car.pz - z0);
        std::printf("      rates  pitch=%-8d roll=%-8d yaw=%-8d  speed=%d vy=%d\n",
                    car.pitch_rate, car.roll_rate, car.yaw_rate, car.speed, car.vy);
        std::printf("      con=%02X near=%02X  body pitch %.1f deg, terrain %.1f deg\n\n",
                    car.contact & 0xFF, car.near_ground & 0xFF, deg24(car.pitch),
                    deg16(car.terrain_pitch));
    }

    // --- SLOPES -------------------------------------------------------------------------
    if (bloc("SLOPES")) {
        std::printf("SLOPES  step = steepest height-byte change across the cell\n");
        std::printf("  step  body pitch  terrain pitch    gap  con   corner gaps (world units)\n");
        const int wanted[] = {0, 2, 4, 6, 8, 12, 16};
        for (const int want : wanted) {
            int bx = -1, bz = -1;
            for (int gz = 2; gz < ter.h - 2 && bx < 0; ++gz) {
                for (int gx = 2; gx < ter.w - 2; ++gx) {
                    if (cell_step(ter, gx, gz) == want) {
                        bx = gx;
                        bz = gz;
                        break;
                    }
                }
            }
            if (bx < 0) {
                std::printf("  %4d  (no such cell on this map)\n", want);
                continue;
            }
            bigred::RespawnPoint p;
            p.x = bx * 512 + 256;
            p.z = bz * 512 + 256;
            p.yaw_bam16 = 0;
            car.reset(tuning, ter, rules, p);
            settle(car, 600);
            std::printf("  %4d  %9.1f  %13.1f  %5.1f   %02X  ", want, deg24(car.pitch),
                        deg16(car.terrain_pitch), deg24(car.pitch) - deg16(car.terrain_pitch),
                        car.contact & 0xFF);
            for (int i = 0; i < 4; ++i) std::printf("%6d", car.corner_ground[i] - car.corner_y[i]);
            std::printf("\n");
        }
    }

    // --- TORQUE -------------------------------------------------------------------------
    // The decisive measurement. The righting torque (`cogs`) fires whenever one GROUP of bottom
    // corners touches the ground and the other does not, and nothing bounds it: the terrain snap
    // that would stop it needs |rate| < 0x3000, which one tick of torque already overshoots.
    // So the question is not "what does the body settle at" but "does the rate settle at all".
    //
    // Sampled over every cell steeper than a threshold, not one hand-picked cell, because a
    // single cell can be lucky.
    //
    // ⚠️ READ "settled" CAREFULLY. It counts runs that end with both rates at exactly zero, and
    // a vehicle that is quietly SLIDING DOWN a hill never reaches that - which is correct
    // behaviour, not a runaway. Since the push-back slides along the slope, this column dropped
    // from ~85% to ~57% when the slide was restored even though every other measurement
    // improved. Judge the torque by JUMP's take-off rate, which is the number that decides what
    // the body does in the air; this block is only useful for spotting a rate that GROWS.
    if (bloc("TORQUE")) {
        std::printf("\nTORQUE  drop the vehicle on every steep cell, let it settle, watch the rate\n");
        // `tilted` counts runs that END more than 3 degrees off the terrain's own angle on either
        // axis while on the ground - the "does not come back level" report of 2026-09-23 - and
        // `flipped` those that end past 90 degrees of roll or pitch.
        std::printf("  min step  cells  settled  worst peak rate  worst final rate  tilted  flipped\n");
        for (const int floor_step : {4, 8, 12, 16}) {
            int cells = 0, settled = 0, tilted = 0, flipped = 0;
            int32_t worst_peak = 0, worst_final = 0;
            for (int gz = 2; gz < ter.h - 2; gz += 7) {
                for (int gx = 2; gx < ter.w - 2; gx += 7) {
                    if (cell_step(ter, gx, gz) < floor_step) continue;
                    if (++cells > 400) break;
                    bigred::RespawnPoint p;
                    p.x = gx * 512 + 256;
                    p.z = gz * 512 + 256;
                    p.yaw_bam16 = 0;
                    car.reset(tuning, ter, rules, p);
                    int32_t peak = 0;
                    const bigred::VehicleInput idle;
                    for (int i = 0; i < 600; ++i) {
                        car.step(idle);
                        // Ignore the first 40 ticks: that is the 500-unit drop reset() starts from.
                        if (i >= 40 && std::abs(car.pitch_rate) > std::abs(peak)) peak = car.pitch_rate;
                    }
                    if (car.pitch_rate == 0 && car.roll_rate == 0) ++settled;
                    const double gp = deg24(car.pitch) - deg16(car.terrain_pitch);
                    const double gr = -deg24(car.roll) - deg16(car.terrain_roll);
                    if (std::abs(deg24(car.pitch)) > 90 || std::abs(deg24(car.roll)) > 90) ++flipped;
                    else if (car.near_ground != 0 && (std::abs(gp) > 3 || std::abs(gr) > 3)) ++tilted;
                    if (std::abs(peak) > std::abs(worst_peak)) worst_peak = peak;
                    if (std::abs(car.pitch_rate) > std::abs(worst_final)) worst_final = car.pitch_rate;
                }
            }
            std::printf("  %8d  %5d  %6d%%  %15d  %16d  %6d  %7d\n", floor_step, cells,
                        cells ? settled * 100 / cells : 0, worst_peak, worst_final, tilted, flipped);
        }
    }

    // --- JUMP ---------------------------------------------------------------------------
    // The measurement the user's report is about: "the boat somersaults the moment it leaves a
    // ramp". In the original the angular rates are FROZEN in the air, so what the body does up
    // there is decided entirely by the rate it LEFT THE GROUND with. That rate is the number.
    //
    // Many runs, from cells spread across the map and in four headings, rather than one run from
    // the grid: a single run only ever samples one bit of terrain.
    if (bloc("JUMP")) {
        std::printf("\nJUMP  flat out from %d starts x 4 headings, 900 ticks each\n", 24);
        {
            int flights = 0, somersaults = 0;
            long long rate_sum = 0;
            int32_t worst_rate = 0, worst_in_air = 0;
            double worst_turn = 0.0;
            bigred::VehicleInput go;
            go.accel = true;
            int starts = 0;
            for (int gz = 8; gz < ter.h - 8 && starts < 24; gz += 23) {
                for (int gx = 8; gx < ter.w - 8 && starts < 24; gx += 29) {
                    ++starts;
                    for (int heading = 0; heading < 4; ++heading) {
                        bigred::RespawnPoint p;
                        p.x = gx * 512 + 256;
                        p.z = gz * 512 + 256;
                        p.yaw_bam16 = heading * 16384;
                        car.reset(tuning, ter, rules, p);
                        int run = 0, grounded = 0;
                        bool was_driving = false;
                        int32_t takeoff = 0, pitch0 = 0;
                        for (int t = 0; t < 900; ++t) {
                            const uint16_t before = car.near_ground;
                            car.step(go);
                            if (before == 0 && car.near_ground == 0) {
                                if (run == 0) {
                                    takeoff = car.pitch_rate;
                                    pitch0 = car.pitch;
                                    // Was this a jump taken while DRIVING, or the tail of a crash?
                                    // Only the first is the user's scenario, and a bench that drives
                                    // blind into a hillside for 900 ticks produces plenty of the
                                    // second.
                                    was_driving = (grounded >= 30 && car.speed > 20000);
                                }
                                ++run;
                                if (std::abs(car.pitch_rate) > std::abs(worst_in_air))
                                    worst_in_air = car.pitch_rate;
                            } else if (run > 0) {
                                grounded = 0;
                                // Only count real flights, not the one-tick gaps of a bumpy road.
                                if (run >= 5 && was_driving) {
                                    ++flights;
                                    rate_sum += std::abs(takeoff);
                                    if (std::abs(takeoff) > std::abs(worst_rate)) worst_rate = takeoff;
                                    const double turn = deg24(car.pitch - pitch0);
                                    if (std::abs(turn) > std::abs(worst_turn)) worst_turn = turn;
                                    if (std::abs(turn) > 90.0) ++somersaults;
                                }
                                run = 0;
                            } else {
                                ++grounded;
                            }
                        }
                    }
                }
            }
            std::printf("      %d jumps taken while driving, %d of them turned more than 90 deg\n",
                        flights, somersaults);
            std::printf("      take-off pitch rate: mean %lld, worst %d (%.2f deg/tick)\n",
                        flights ? rate_sum / flights : 0, worst_rate, deg24(worst_rate));
            std::printf("      worst pitch rate at ANY tick in the air  %d\n", worst_in_air);
            std::printf("      biggest rotation in one flight  %.1f deg\n", worst_turn);
        }
    }
    // --- LIFT ---------------------------------------------------------------------------
    // The user's report of 2026-09-24: hit something steep - a mountainside, the side of the
    // DIRT1 bridge from the water - and the car is TELEPORTED up onto it. A penetration
    // push-back is meant to undo a few units of sinking, so any tick where it lifts the body by
    // more than 64 world units is suspect. Drive flat out from many starts, some of them
    // steering, and name the source of every such lift: terrain, drivable object, platform.
    if (bloc("LIFT")) {
        std::printf("\nLIFT  push-back lifting the body > 64 units in one tick\n");
        struct Ev {
            int32_t lift, x, y, z, speed; int src; const bigred::SimObject* o;
            int gx, gz, heading, t;
        };
        std::vector<Ev> evs;
        int count[4] = {0, 0, 0, 0};
        int starts = 0;
        for (int gz = 6; gz < ter.h - 6 && starts < 48; gz += 17) {
            for (int gx = 6; gx < ter.w - 6 && starts < 48; gx += 19) {
                ++starts;
                for (int heading = 0; heading < 8; ++heading) {
                    bigred::RespawnPoint p;
                    p.x = gx * 512 + 256;
                    p.z = gz * 512 + 256;
                    p.yaw_bam16 = heading * 8192;
                    car.reset(tuning, ter, rules, p);
                    for (int t = 0; t < 700; ++t) {
                        bigred::VehicleInput in;
                        in.accel = true;
                        in.left = (heading & 1) && (t / 90) % 3 == 1;
                        in.right = (heading & 1) && (t / 90) % 3 == 2;
                        car.step(in);
                        if (t < 40 || car.push_worst_y <= 64) continue;
                        ++count[car.push_source & 3];
                        evs.push_back({car.push_worst_y, car.px >> 9, car.py >> 9, car.pz >> 9,
                                       car.speed, car.push_source, car.push_object,
                                       gx, gz, heading, t});
                    }
                }
            }
        }
        std::printf("      %d starts x 8 headings : %d lift(s) from terrain, %d from drivable "
                    "objects, %d from platforms\n", starts, count[1], count[2], count[3]);
        std::sort(evs.begin(), evs.end(), [](const Ev& a, const Ev& b) { return a.lift > b.lift; });
        for (size_t i = 0; i < evs.size() && i < 12; ++i) {
            const Ev& e = evs[i];
            std::string what = e.src == 1 ? "terrain" : "?";
            if (e.o && e.o->instance_index >= 0 &&
                static_cast<size_t>(e.o->instance_index) < scene.instances.size()) {
                const auto& inst = scene.instances[static_cast<size_t>(e.o->instance_index)];
                const std::string name =
                    (inst.shape_index >= 0 &&
                     static_cast<size_t>(inst.shape_index) < scene.shape_names_by_slot.size())
                        ? scene.shape_names_by_slot[static_cast<size_t>(inst.shape_index)] : "?";
                char buf[160];
                std::snprintf(buf, sizeof buf, "%s %s cat %d lift=%d at (%d,%d,%d)",
                              e.src == 3 ? "platform" : "object", name.c_str(),
                              bigred::collision_category_index(inst.collision_category_byte),
                              e.o->platform_lift, e.o->x, e.o->y, e.o->z);
                what = buf;
            }
            std::printf("      +%-5d u  car (%d,%d,%d) speed %6d  <- %s\n", e.lift, e.x, e.y, e.z,
                        e.speed, what.c_str());
        }
        // Where the player actually is: from the middle of every gate of the ring, drive flat out
        // ACROSS the track, both ways, and diagonally - into the banks, the bridge flanks
        // (DIRT1's bridge is terrain, it has no shape), the hillsides lining the road.
        {
            std::vector<bigred::Gate> lg;
            if (auto cse = scene.lev.find("MAP.CSE"); cse != scene.lev.end())
                lg = bigred::parse_gates(cse->second.data(), cse->second.size());
            int runs = 0, lifted = 0;
            int32_t worst = 0, worst_gain = 0;
            for (const bigred::Gate& g : lg) {
                const bigred::RespawnPoint mid = bigred::gate_respawn(g, 0x2000);
                for (const int32_t off : {0x4000, -0x4000, 0x2000, -0x2000, 0x6000, -0x6000}) {
                    bigred::RespawnPoint p = mid;
                    p.yaw_bam16 = static_cast<uint16_t>(mid.yaw_bam16 + off);
                    car.reset(tuning, ter, rules, p);
                    for (int t = 0; t < 40; ++t) car.step(bigred::VehicleInput());
                    const int32_t y0 = car.py >> 9;
                    int32_t top = y0;
                    bigred::VehicleInput go;
                    go.accel = true;
                    for (int t = 0; t < 250; ++t) {
                        car.step(go);
                        if (car.push_worst_y > worst) worst = car.push_worst_y;
                        if (car.push_worst_y > 64) ++lifted;
                        top = std::min(top, car.py >> 9);
                    }
                    worst_gain = std::max(worst_gain, y0 - top);
                    ++runs;
                }
            }
            std::printf("      off the track from every gate: %d runs, %d tick(s) lifted > 64, "
                        "worst lift %d, highest climb %d units\n", runs, lifted, worst, worst_gain);
        }
        // Where is the bridge? The gate whose road stands highest over the ground either side of
        // it. Then ram that road from the low side, across, both ways, and trace the worst run.
        {
            std::vector<bigred::Gate> lg;
            if (auto cse = scene.lev.find("MAP.CSE"); cse != scene.lev.end())
                lg = bigred::parse_gates(cse->second.data(), cse->second.size());
            struct Cand { int gate; int32_t drop; };
            std::vector<Cand> cands;
            for (size_t gi = 0; gi < lg.size(); ++gi) {
                const bigred::RespawnPoint mid = bigred::gate_respawn(lg[gi], 0x2000);
                const int32_t road = ter.height_fx(mid.x << 9, mid.z << 9) >> 9;
                int32_t drop = 0;
                for (const int32_t side : {0x4000, -0x4000}) {
                    const int32_t a = mid.yaw_bam16 + side;
                    for (int32_t d = 600; d <= 2400; d += 300) {
                        const int32_t gx = mid.x + bigred::fx::mul15(d, bigred::fx::sin16(a));
                        const int32_t gz = mid.z + bigred::fx::mul15(d, bigred::fx::cos16(a));
                        drop = std::max(drop, (ter.height_fx(gx << 9, gz << 9) >> 9) - road);
                    }
                }
                // Water is the heightmap's floor, height 0: a road with it on both sides within
                // 1500 units is a bridge (or a causeway).
                int water_sides = 0;
                for (const int32_t side : {0x4000, -0x4000}) {
                    const int32_t a = mid.yaw_bam16 + side;
                    for (int32_t d = 300; d <= 1500; d += 150) {
                        const int32_t gx = mid.x + bigred::fx::mul15(d, bigred::fx::sin16(a));
                        const int32_t gz = mid.z + bigred::fx::mul15(d, bigred::fx::cos16(a));
                        if ((ter.height_fx(gx << 9, gz << 9) >> 9) >= -4) { ++water_sides; break; }
                    }
                }
                if (water_sides == 2) drop += 100000;   // rank the bridges first
                cands.push_back({static_cast<int>(gi), drop});
            }
            std::sort(cands.begin(), cands.end(),
                      [](const Cand& a, const Cand& b) { return a.drop > b.drop; });
            for (size_t k = 0; k < cands.size() && k < 3; ++k) {
                const bigred::RespawnPoint mid = bigred::gate_respawn(lg[static_cast<size_t>(cands[k].gate)], 0x2000);
                std::printf("      road over the ground: gate %d at (%d,%d), %d units above the "
                            "lowest ground within 2400 across\n", cands[k].gate, mid.x, mid.z,
                            cands[k].drop);
                if (k == 0) {
                    std::printf("        road height %d; what stands within 2500:\n",
                                ter.height_fx(mid.x << 9, mid.z << 9) >> 9);
                    for (const auto& inst : scene.instances) {
                        const int64_t dx = static_cast<int64_t>(inst.x) - mid.x;
                        const int64_t dz = static_cast<int64_t>(inst.z) - mid.z;
                        if (dx * dx + dz * dz > 2500LL * 2500LL) continue;
                        const std::string nm =
                            (inst.shape_index >= 0 &&
                             static_cast<size_t>(inst.shape_index) < scene.shape_names_by_slot.size())
                                ? scene.shape_names_by_slot[static_cast<size_t>(inst.shape_index)] : "?";
                        std::printf("          %-9s cat %2d at (%d,%d,%d) yaw &%04X  ground %d\n",
                                    nm.c_str(),
                                    bigred::collision_category_index(inst.collision_category_byte),
                                    static_cast<int>(inst.x), static_cast<int>(inst.y),
                                    static_cast<int>(inst.z), inst.angle_y & 0xFFFF,
                                    ter.height_fx(static_cast<int32_t>(inst.x) << 9,
                                                  static_cast<int32_t>(inst.z) << 9) >> 9);
                    }
                }
                // Ram it: start 1800 across on each side, facing the road.
                for (const int32_t side : {0x4000, -0x4000}) {
                    for (const int32_t d : {1200, 1800, 2400}) {
                    const int32_t a = mid.yaw_bam16 + side;
                    bigred::RespawnPoint p;
                    p.x = mid.x + bigred::fx::mul15(d, bigred::fx::sin16(a));
                    p.z = mid.z + bigred::fx::mul15(d, bigred::fx::cos16(a));
                    p.yaw_bam16 = static_cast<uint16_t>(a + 0x8000);
                    car.reset(tuning, ter, rules, p);
                    for (int t = 0; t < 40; ++t) car.step(bigred::VehicleInput());
                    const int32_t y0 = car.py >> 9;
                    int32_t top = y0, worst_tick = 0, worst_rise = 0, prev = y0;
                    bigred::VehicleInput go;
                    go.accel = true;
                    for (int t = 0; t < 120; ++t) {
                        car.step(go);
                        const int32_t yy = car.py >> 9;
                        if (prev - yy > worst_rise) { worst_rise = prev - yy; worst_tick = t; }
                        prev = yy;
                        top = std::min(top, yy);
                    }
                    std::printf("        from %s %d: start y %d, highest %d (climb %d), worst rise "
                                "in one tick %d at t=%d, end speed %d\n",
                                side > 0 ? "right" : "left", d, y0, top, y0 - top, worst_rise,
                                worst_tick, car.speed);
                    }
                }
            }
        }
        // The user's own scene: DIRT1's bridge rammed from the water. Its deck is TERRAIN (the
        // BR_* shapes are decor), so its flank is a steep terrain face. Start 1500 units off
        // every BR_* instance in 8 directions and drive flat out at it.
        {
            int runs = 0, lifted = 0;
            int32_t worst = 0, worst_gain = 0;
            for (const auto& inst : scene.instances) {
                if (inst.shape_index < 0 ||
                    static_cast<size_t>(inst.shape_index) >= scene.shape_names_by_slot.size())
                    continue;
                const std::string& nm = scene.shape_names_by_slot[static_cast<size_t>(inst.shape_index)];
                if (nm.find("BR") == std::string::npos) continue;
                if (runs == 0) std::printf("      (first match: %s)\n", nm.c_str());
                for (int dir = 0; dir < 8; ++dir) {
                    const int32_t a = dir * 8192;
                    bigred::RespawnPoint p;
                    p.x = static_cast<int32_t>(inst.x) - bigred::fx::mul15(1500, bigred::fx::sin16(a));
                    p.z = static_cast<int32_t>(inst.z) - bigred::fx::mul15(1500, bigred::fx::cos16(a));
                    p.yaw_bam16 = a;
                    car.reset(tuning, ter, rules, p);
                    for (int t = 0; t < 40; ++t) car.step(bigred::VehicleInput());
                    const int32_t y0 = car.py >> 9;
                    int32_t top = y0;
                    bigred::VehicleInput go;
                    go.accel = true;
                    for (int t = 0; t < 160; ++t) {
                        car.step(go);
                        if (car.push_worst_y > worst) worst = car.push_worst_y;
                        if (car.push_worst_y > 64) ++lifted;
                        top = std::min(top, car.py >> 9);
                    }
                    worst_gain = std::max(worst_gain, y0 - top);
                    ++runs;
                }
            }
            if (runs == 0 && std::getenv("OBR_SHAPES")) {
                for (const std::string& nm : scene.shape_names_by_slot) std::printf(" %s", nm.c_str());
                std::printf("\n");
            }
            if (runs > 0)
                std::printf("      bridge (*BR*) rammed: %d runs, %d tick(s) lifted > 64, worst lift %d, "
                            "highest climb %d units\n", runs, lifted, worst, worst_gain);
        }
        // The worst one, replayed tick by tick: the push vector, what the bounce did to the
        // speed, and the slope the engine saw.
        if (!evs.empty()) {
            const Ev& w = evs.front();
            bigred::RespawnPoint p;
            p.x = w.gx * 512 + 256;
            p.z = w.gz * 512 + 256;
            p.yaw_bam16 = w.heading * 8192;
            car.reset(tuning, ter, rules, p);
            std::printf("      trace of the worst, ticks %d..%d\n", w.t - 10, w.t + 2);
            for (int t = 0; t <= w.t + 2; ++t) {
                bigred::VehicleInput in;
                in.accel = true;
                in.left = (w.heading & 1) && (t / 90) % 3 == 1;
                in.right = (w.heading & 1) && (t / 90) % 3 == 2;
                const int32_t sp0 = car.speed, vy0 = car.vy;
                car.step(in);
                if (t < w.t - 10) continue;
                std::printf("        t=%d y=%d push=(%d,%d,%d) speed %d->%d vy %d->%d con=%02X "
                            "tpitch=%d pitch=%.1f land=%d\n",
                            t, car.py >> 9, car.push_x_dbg() >> 9, car.push_y_dbg() >> 9, car.push_z_dbg() >> 9,
                            sp0, car.speed, vy0, car.vy, car.contact, car.terrain_pitch,
                            deg24(car.pitch), car.land);
            }
        }
        // What stands near the worst lifts: if the original stops a car at a steep face, it
        // does it with something in MAP.DAT, and its category says which switch case.
        std::vector<size_t> near_seen;
        for (size_t i = 0; i < evs.size() && i < 12; ++i) {
            for (size_t k = 0; k < scene.instances.size(); ++k) {
                const auto& inst = scene.instances[k];
                const int64_t dx = static_cast<int64_t>(inst.x) - evs[i].x;
                const int64_t dz = static_cast<int64_t>(inst.z) - evs[i].z;
                if (dx * dx + dz * dz > 1200LL * 1200LL) continue;
                if (std::find(near_seen.begin(), near_seen.end(), k) != near_seen.end()) continue;
                near_seen.push_back(k);
                const std::string name =
                    (inst.shape_index >= 0 &&
                     static_cast<size_t>(inst.shape_index) < scene.shape_names_by_slot.size())
                        ? scene.shape_names_by_slot[static_cast<size_t>(inst.shape_index)] : "?";
                std::printf("        near: %-10s cat %2d at (%d,%d,%d)\n", name.c_str(),
                            bigred::collision_category_index(inst.collision_category_byte),
                            static_cast<int>(inst.x), static_cast<int>(inst.y),
                            static_cast<int>(inst.z));
            }
        }
    }
    // --- BRIDGE -------------------------------------------------------------------------
    // The scenery pass's cases 0x12/0x1a (RIG1's WOODBRIJ): a floor from above, a ceiling from below.
    // The user's report of 2026-09-25: from the river, driving into the bridge put the car ON
    // it. Two tests: ram each section from the water on both sides, and drive across it.
    if (bloc("BRIDGE")) {
        bool any = false;
        for (const bigred::SimObject& o : ter.objects) any = any || o.bridge;
        if (any) {
            std::printf("\nBRIDGE  decks of cases 0x12/0x1a\n");
            int rams = 0, onto = 0, rams_x = 0, onto_x = 0;
            int32_t worst_rise = 0;
            for (const bigred::SimObject& o : ter.objects) {
                if (!o.bridge) continue;
                const int32_t deck = o.y + o.height;
                for (const int32_t side : {0, 0x8000, 0x4000, 0xC000}) {
                    // Across the deck's local X (the road runs along local Z, yaw + 0/0x8000)
                    const int32_t a = (o.yaw_bam16 + 0x4000 + side) & 0xFFFF;
                    for (const int32_t d : {900, 1400, 2000}) {
                        bigred::RespawnPoint p;
                        p.x = o.x - bigred::fx::mul15(d, bigred::fx::sin16(a));
                        p.z = o.z - bigred::fx::mul15(d, bigred::fx::cos16(a));
                        p.yaw_bam16 = static_cast<uint16_t>(a);
                        // only starts in the water, well under the deck
                        if ((ter.height_fx(p.x << 9, p.z << 9) >> 9) < deck + 100) continue;
                        car.reset(tuning, ter, rules, p);
                        for (int t = 0; t < 40; ++t) car.step(bigred::VehicleInput());
                        const int32_t y0 = car.py >> 9;
                        int32_t top = y0;
                        bigred::VehicleInput go;
                        go.accel = true;
                        // Count it only if the car got up there OVER a section - climbing the
                        // bank on the far side is not the bug.
                        bool on_deck = false;
                        for (int t = 0; t < 150; ++t) {
                            car.step(go);
                            const int32_t yy = car.py >> 9;
                            if (yy + tuning.box.ybottom > deck + 20) continue;
                            for (const bigred::SimObject& s : ter.objects) {
                                if (!s.bridge) continue;
                                const int32_t ddx = (car.px >> 9) - s.x, ddz = (car.pz >> 9) - s.z;
                                if (ddx * ddx + ddz * ddz < 450 * 450) on_deck = true;
                            }
                            if (on_deck) top = std::min(top, yy);
                        }
                        ++rams;
                        worst_rise = std::max(worst_rise, y0 - top);
                        if (on_deck) ++onto;
                        if (side == 0 || side == 0x8000) {   // square to the deck: the report
                            ++rams_x;
                            if (on_deck) ++onto_x;
                        }
                    }
                }
            }
            std::printf("      rammed from the water: %d runs, %d got up onto a section, "
                        "worst rise %d units; square to the deck: %d of %d\n", rams, onto,
                        worst_rise, onto_x, rams_x);
            // Across, from the BANKS: the two outermost sections along the deck's local Z, 1500
            // beyond each, driving toward the other end. At every section passed, the body's
            // underside against that section's deck.
            {
                const bigred::SimObject* first = nullptr;
                const bigred::SimObject* last = nullptr;
                int32_t lo = 1 << 30, hi = -(1 << 30);
                for (const bigred::SimObject& o : ter.objects) {
                    if (!o.bridge) continue;
                    const int32_t along = bigred::fx::mul15(o.x, bigred::fx::sin16(o.yaw_bam16)) +
                                          bigred::fx::mul15(o.z, bigred::fx::cos16(o.yaw_bam16));
                    if (along < lo) { lo = along; first = &o; }
                    if (along > hi) { hi = along; last = &o; }
                }
                for (int way = 0; way < 2 && first; ++way) {
                    const bigred::SimObject& from = way == 0 ? *first : *last;
                    const int32_t a = (from.yaw_bam16 + (way == 0 ? 0 : 0x8000)) & 0xFFFF;
                    bigred::RespawnPoint p;
                    p.x = from.x - bigred::fx::mul15(1500, bigred::fx::sin16(a));
                    p.z = from.z - bigred::fx::mul15(1500, bigred::fx::cos16(a));
                    p.yaw_bam16 = static_cast<uint16_t>(a);
                    car.reset(tuning, ter, rules, p);
                    for (int t = 0; t < 40; ++t) car.step(bigred::VehicleInput());
                    std::printf("      across from the %s bank (ground %d):", way ? "far" : "near",
                                ter.height_fx(p.x << 9, p.z << 9) >> 9);
                    bigred::VehicleInput go;
                    go.accel = true;
                    std::vector<const bigred::SimObject*> seen;
                    for (int t = 0; t < 250; ++t) {
                        car.step(go);
                        for (const bigred::SimObject& o : ter.objects) {
                            if (!o.bridge) continue;
                            if (std::find(seen.begin(), seen.end(), &o) != seen.end()) continue;
                            const int32_t ddx = (car.px >> 9) - o.x, ddz = (car.pz >> 9) - o.z;
                            if (std::abs(ddx) + std::abs(ddz) > 250) continue;
                            seen.push_back(&o);
                            std::printf("  section %d: underside %d (deck %d)",
                                        static_cast<int>(&o - ter.objects.data()),
                                        (car.py >> 9) + tuning.box.ybottom, o.y + o.height);
                        }
                    }
                    std::printf("  [%zu sections crossed]\n", seen.size());
                }
            }
        }
    }
    // --- RAMP ---------------------------------------------------------------------------
    // The user's actual scenario, and the one every other test here MISSES: a jump taken off a
    // drivable RAMP OBJECT, not off a fold in the terrain. WATER1 carries 98 of them, and they
    // go through object_probe, a completely different code path from the terrain probe.
    //
    // Approach: a ramp's surface is tilted along its own local Z, and that axis points along
    // (sin yaw, cos yaw) in the world - the same convention `travel` uses. So driving in with
    // yaw = the object's yaw runs straight up it.
    if (bloc("RAMP") && !ter.objects.empty()) {
        std::printf("\nRAMP  drive up each ramp object and trace the take-off\n");
        int traced = 0;
        for (size_t oi = 0; oi < ter.objects.size() && traced < 3; ++oi) {
            const bigred::SimObject& o = ter.objects[oi];
            if (o.slope == 0 || o.platform) continue; // a flat deck is not a jump

            const int32_t approach = 4000; // world units before the ramp
            const int32_t sn = bigred::fx::sin16(o.yaw_bam16);
            const int32_t cs = bigred::fx::cos16(o.yaw_bam16);
            bigred::RespawnPoint p;
            p.x = o.x - bigred::fx::mul15(approach, sn);
            p.z = o.z - bigred::fx::mul15(approach, cs);
            p.yaw_bam16 = o.yaw_bam16;
            car.reset(tuning, ter, rules, p);

            bigred::VehicleInput go;
            go.accel = true;
            // Settle onto the ground first, then drive in.
            for (int i = 0; i < 40; ++i) car.step(bigred::VehicleInput());

            std::vector<std::string> lines;
            int airborne_at = -1;
            for (int t = 0; t < 400 && airborne_at < 0; ++t) {
                car.step(go);
                char buf[200];
                std::snprintf(buf, sizeof buf,
                              "   %4d near=%02X con=%02X land=%d  pitch=%7.2f prate=%9d  "
                              "terr_pitch=%7.2f  speed=%6d vy=%7d",
                              t, car.near_ground & 0xFF, car.contact & 0xFF, car.land,
                              deg24(car.pitch), car.pitch_rate, deg16(car.terrain_pitch),
                              car.speed, car.vy);
                lines.push_back(buf);
                if (car.land == 6) airborne_at = t; // touched the ramp
            }
            if (airborne_at < 0) continue; // never reached this ramp; try the next

            // Keep going through the jump itself AND the landing after it: how long the body
            // takes to come back to the terrain's angle is half of what the user judges.
            for (int t = 0; t < 160; ++t) {
                car.step(go);
                char buf[200];
                std::snprintf(buf, sizeof buf,
                              "   %4d near=%02X con=%02X land=%d  pitch=%7.2f prate=%9d  "
                              "terr_pitch=%7.2f  speed=%6d vy=%7d",
                              airborne_at + 1 + t, car.near_ground & 0xFF, car.contact & 0xFF,
                              car.land, deg24(car.pitch), car.pitch_rate,
                              deg16(car.terrain_pitch), car.speed, car.vy);
                lines.push_back(buf);
            }
            std::printf("  ramp #%zu at (%d,%d) slope=%d height=%d - reached at tick %d\n",
                        oi, o.x, o.z, o.slope, o.height, airborne_at);
            // Only the ticks around the ramp are interesting.
            const size_t from = (airborne_at > 8) ? size_t(airborne_at - 8) : 0;
            for (size_t i = from; i < lines.size(); ++i) std::printf("%s\n", lines[i].c_str());
            ++traced;
        }
        if (traced == 0) std::printf("  (no ramp was reached by driving straight at it)\n");
    }

    // --- LANDING ------------------------------------------------------------------------
    // The other half of the user's report, and the one no other block here measured: once a
    // vehicle touches down nose-high or nose-low, HOW LONG does it take to lie flat on the
    // ground again? The original rights a body hard - the torque accumulates every tick one
    // corner group hangs - so this should be a fraction of a second, and it should be FAST
    // compared with how slowly the body turns while airborne.
    //
    // Method: put the body over flat ground, 600 units up, at a given pitch, let it fall, and
    // count the ticks from first ground contact until it is within 2 degrees of the terrain.
    if (bloc("LANDING")) {
        std::printf("\nLANDING  drop from 600 units at a set pitch, over flat ground\n");
        std::printf("  start pitch   ticks airborne   ticks to flatten   ms   final pitch\n");
        // A cell whose neighbourhood is as flat as this map has.
        int bx = -1, bz = -1;
        for (int gz = 4; gz < ter.h - 4 && bx < 0; ++gz)
            for (int gx = 4; gx < ter.w - 4; ++gx)
                if (cell_step(ter, gx, gz) == 0 && cell_step(ter, gx + 1, gz) == 0 &&
                    cell_step(ter, gx, gz + 1) == 0) {
                    bx = gx;
                    bz = gz;
                    break;
                }
        for (const double want_deg : {-40.0, -20.0, 20.0, 40.0}) {
            bigred::RespawnPoint p;
            p.x = bx * 512 + 256;
            p.z = bz * 512 + 256;
            p.yaw_bam16 = 0;
            car.reset(tuning, ter, rules, p);
            settle(car, 200);
            const int32_t flat = car.pitch;
            // Lift it and set the attitude by hand: reset() drops from 500, which is not enough
            // to give a 40-degree body time to show what it does.
            car.py -= 600 << 9;
            car.pitch = flat + static_cast<int32_t>(want_deg * 16777216.0 / 360.0);
            car.pitch_rate = 0;
            car.vy = 0;
            const bigred::VehicleInput idle;
            int air = 0, flatten = -1;
            for (int t = 0; t < 400; ++t) {
                car.step(idle);
                if (car.near_ground == 0) {
                    if (flatten < 0) ++air;
                    continue;
                }
                const double gap = deg24(car.pitch) - deg16(car.terrain_pitch);
                if (flatten < 0 && std::abs(gap) < 2.0) flatten = t - air;
            }
            std::printf("  %9.0f   %14d   %16s   %4d   %9.1f\n", want_deg, air,
                        flatten < 0 ? "never" : std::to_string(flatten).c_str(),
                        flatten < 0 ? 0 : flatten * 28, deg24(car.pitch));
        }
        // And the same run traced tick by tick, because the total says nothing about WHERE the
        // time goes: the righting torque only fires while ONE corner group is down, and both
        // the half-damping and the fresh-contact inversion of the original can undo it.
        {
            bigred::RespawnPoint p;
            p.x = bx * 512 + 256;
            p.z = bz * 512 + 256;
            p.yaw_bam16 = 0;
            car.reset(tuning, ter, rules, p);
            settle(car, 200);
            const int32_t flat = car.pitch;
            car.py -= 600 << 9;
            car.pitch = flat + static_cast<int32_t>(40.0 * 16777216.0 / 360.0);
            car.pitch_rate = 0;
            car.vy = 0;
            const bigred::VehicleInput idle;
            bool touched = false;
            int shown = 0;
            std::printf("  trace of the 40 deg case, from first contact\n");
            for (int t = 0; t < 200 && shown < 30; ++t) {
                car.step(idle);
                if (car.near_ground == 0 && !touched) continue;
                touched = true;
                ++shown;
                std::printf("    %3d near=%02X con=%02X  pitch=%7.2f prate=%9d  vy=%7d\n", t,
                            car.near_ground & 0xFF, car.contact & 0xFF, deg24(car.pitch),
                            car.pitch_rate, car.vy);
            }
        }
    }

    // --- TILT ---------------------------------------------------------------------------
    // "The car does not come back level: it stays a little on two wheels - usually the RIGHT
    // ones - or a little nose-down, and takes a while to flatten" (2026-09-23, mostly DIRT1).
    // LANDING only asks about 20-40 degree drops; this asks about the SMALL leftover angles, on
    // the ground, at rest, and both ways round, because the report is an asymmetry. A body is
    // set on flat ground at a given roll or pitch with zero rates and left alone.
    if (bloc("TILT")) {
        std::printf("\nTILT  small leftover angle on flat ground, at rest, no input\n");
        std::printf("  axis   start   ticks to <0.5deg   angle after 10 / 40 / 150 ticks   "
                    "near  con\n");
        int bx = -1, bz = -1;
        for (int gz = 4; gz < ter.h - 4 && bx < 0; ++gz)
            for (int gx = 4; gx < ter.w - 4; ++gx)
                if (cell_step(ter, gx, gz) == 0 && cell_step(ter, gx + 1, gz) == 0 &&
                    cell_step(ter, gx, gz + 1) == 0 && cell_step(ter, gx - 1, gz) == 0 &&
                    cell_step(ter, gx, gz - 1) == 0) {
                    bx = gx;
                    bz = gz;
                    break;
                }
        for (int axis = 0; axis < 2; ++axis) {
            for (const double want : {-8.0, -4.0, -2.0, 2.0, 4.0, 8.0}) {
                bigred::RespawnPoint p;
                p.x = bx * 512 + 256;
                p.z = bz * 512 + 256;
                p.yaw_bam16 = 0;
                car.reset(tuning, ter, rules, p);
                settle(car, 200);
                int32_t& ang = axis == 0 ? car.roll : car.pitch;
                const int32_t flat = ang;
                ang = flat + static_cast<int32_t>(want * 16777216.0 / 360.0);
                car.pitch_rate = car.roll_rate = 0;
                const bigred::VehicleInput idle;
                int done = -1;
                double at[3] = {};
                for (int t = 1; t <= 150; ++t) {
                    car.step(idle);
                    const double g = deg24((axis == 0 ? car.roll : car.pitch) - flat);
                    if (done < 0 && std::abs(g) < 0.5) done = t;
                    if (t == 10) at[0] = g;
                    if (t == 40) at[1] = g;
                    if (t == 150) at[2] = g;
                }
                std::printf("  %-5s  %5.0f   %16s   %8.2f %8.2f %8.2f        %02X   %02X\n",
                            axis == 0 ? "roll" : "pitch", want,
                            done < 0 ? "never" : std::to_string(done).c_str(), at[0], at[1],
                            at[2], car.near_ground & 0xFF, car.contact & 0xFF);
            }
        }
        // The +4 degree roll, tick by tick: which corners are near, which penetrate, and how far
        // each bottom corner is off the ground (+ = above it).
        {
            bigred::RespawnPoint p;
            p.x = bx * 512 + 256;
            p.z = bz * 512 + 256;
            p.yaw_bam16 = 0;
            car.reset(tuning, ter, rules, p);
            settle(car, 200);
            car.roll += static_cast<int32_t>(4.0 * 16777216.0 / 360.0);
            car.pitch_rate = car.roll_rate = 0;
            std::printf("  trace, roll +4 : near con   roll   roll_rate   vy   corner gaps 0..3\n");
            for (int t = 1; t <= 8; ++t) {
                car.step(bigred::VehicleInput{});
                std::printf("    %2d  %02X  %02X  %6.2f  %9d  %5d  ", t, car.near_ground & 0xFF,
                            car.contact & 0xFF, deg24(car.roll), car.roll_rate, car.vy);
                for (int c = 0; c < 4; ++c)
                    std::printf(" %4d", car.corner_ground[c] - car.corner_y[c]);
                std::printf("\n");
            }
        }
    }

    // --- BUMP ---------------------------------------------------------------------------
    // The small scenery the user feels: WATER1's roots are `platform` objects that raise the
    // ground 32 units over a 70-unit radius. Driving over one should kick the boat.
    if (bloc("BUMP")) {
        const bigred::SimObject* root = nullptr;
        for (const bigred::SimObject& o : ter.objects)
            if (o.platform) { root = &o; break; }
        if (root) {
            std::printf("\nBUMP  drive straight over a %d-unit platform of radius %d\n",
                        root->platform_lift, root->radius);
            // Come in along +X so the approach is independent of the object's own yaw.
            bigred::RespawnPoint p;
            p.x = root->x - 3000;
            p.z = root->z;
            p.yaw_bam16 = 16384; // +X
            car.reset(tuning, ter, rules, p);
            bigred::VehicleInput go;
            go.accel = true;
            for (int i = 0; i < 40; ++i) car.step(bigred::VehicleInput());
            // Trace, not aggregates: the question is what ONE root does as the boat crosses it,
            // and a count over 400 ticks hides that completely.
            std::vector<std::string> lines;
            int crossing = -1;
            for (int t = 0; t < 400; ++t) {
                const int32_t vy_before = car.vy, py_before = car.py;
                car.step(go);
                char buf[200];
                std::snprintf(buf, sizeof buf,
                              "   %4d near=%02X con=%02X land=%d  y=%7d dy=%6d vy=%7d dvy=%7d "
                              "prate=%9d speed=%6d",
                              t, car.near_ground & 0xFF, car.contact & 0xFF, car.land,
                              car.py >> 9, (car.py - py_before) >> 9, car.vy,
                              car.vy - vy_before, car.pitch_rate, car.speed);
                lines.push_back(buf);
                if (car.land == 6 && crossing < 0) crossing = t;
            }
            if (crossing < 0) {
                std::printf("      the boat never touched the platform - test line missed it\n");
            } else {
                const size_t from = (crossing > 4) ? size_t(crossing - 4) : 0;
                const size_t to = std::min(lines.size(), size_t(crossing + 10));
                for (size_t i = from; i < to; ++i) std::printf("%s\n", lines[i].c_str());
            }
        }
    }

    // --- NEAR ---------------------------------------------------------------------------
    // BENCH_NEAR=x,z,r: every decor instance within r world units of (x, z), with its shape,
    // height and collision category - to find what the user saw at a place on the track.
    if (const char* nv = std::getenv("BENCH_NEAR"); bloc("NEAR") && nv) {
        int nx = 0, nz = 0, nr = 4000;
        std::sscanf(nv, "%d,%d,%d", &nx, &nz, &nr);
        std::printf("\nNEAR  decor within %d of (%d,%d)\n", nr, nx, nz);
        for (size_t i = 0; i < scene.instances.size(); ++i) {
            const auto& in = scene.instances[i];
            const int64_t dx = static_cast<int64_t>(in.x) - nx, dz = static_cast<int64_t>(in.z) - nz;
            if (dx * dx + dz * dz > static_cast<int64_t>(nr) * nr) continue;
            const std::string nm =
                (in.shape_index >= 0 && static_cast<size_t>(in.shape_index) < scene.shape_names_by_slot.size())
                    ? scene.shape_names_by_slot[static_cast<size_t>(in.shape_index)] : "?";
            std::printf("      #%-4zu %-10s (%6.0f,%6.0f,%6.0f) cat %02X  var %d\n", i, nm.c_str(), in.x,
                        in.y, in.z, in.collision_category_byte, in.variant_index);
        }
    }

    // --- PIVOT -------------------------------------------------------------------------
    // Where the body turns: the vehicle's ORIGIN, the point the simulation rotates its contact
    // box about and the renderer its mesh about (the same matrix). Measured on the box, +Y down:
    // how far above the bottom of the box (the tyres) and below its top the origin sits.
    if (bloc("PIVOT")) {
        const auto& b = tuning.box;
        const int h = b.ybottom - b.ytop;
        std::printf("\nPIVOT  box y %d (bottom) .. %d (top), height %d: origin %d above the bottom = %d%% of the height;"
                    " width %d, length %d\n", b.ybottom, b.ytop, h, b.ybottom,
                    h ? b.ybottom * 100 / h : 0, b.xmax - b.xmin, b.zmax - b.zmin);
    }

    // --- GRIP ---------------------------------------------------------------------------
    // Two user reports live here: "it slides LESS than the original when off-track", and "the
    // inertia is too great, it does not pick its direction back up in a corner". Both are the
    // same line of code - `svel`, the decay of the lateral velocity component - and both depend
    // on the `mode` flag, which picks the LOW or the HIGH value of every skid pair.
    if (bloc("GRIP")) {
        std::printf("\nGRIP  hold full lock at speed, per surface type\n");
        std::printf("      %-4s %-8s %-8s  %s\n", "land", "svel[0]", "svel[1]", "(low/high pair)");
        for (int l = 0; l < 8; ++l)
            std::printf("      %-4d %-8d %-8d   bounce %6d %6d %6d %6d\n", l, tuning.svel[l].low, tuning.svel[l].high,
                        tuning.bounce[l][0], tuning.bounce[l][1], tuning.bounce[l][2], tuning.bounce[l][3]);
        std::printf("      tilter z: mode %d a=%d b=%d c=%d   tilter x: mode %d a=%d b=%d c=%d\n",
                    tuning.tilter_z.mode, tuning.tilter_z.a, tuning.tilter_z.b, tuning.tilter_z.c,
                    tuning.tilter_x.mode, tuning.tilter_x.a, tuning.tilter_x.b, tuning.tilter_x.c);

        // Which surfaces the LEVEL actually declares, and how much of the map is each one. If
        // this collapses to a single type, every per-surface table above is dead weight and
        // off-track grip cannot differ from on-track grip.
        {
            const std::vector<uint8_t> lt = bigred::parse_land_table(level_txt);
            std::printf("\n      level `land` line:");
            for (size_t i = 0; i < lt.size(); ++i) std::printf(" %d", lt[i] & 7);
            std::printf("\n");
            int hist[8] = {};
            for (int gz = 0; gz < ter.h; gz += 2)
                for (int gx = 0; gx < ter.w; gx += 2)
                    ++hist[ter.land_type(static_cast<int32_t>(gx) << 18,
                                         static_cast<int32_t>(gz) << 18) & 7];
            int total = 0;
            for (int l = 0; l < 8; ++l) total += hist[l];
            std::printf("      map coverage by surface:");
            for (int l = 0; l < 8; ++l)
                if (hist[l]) std::printf("  %d=%d%%", l, total ? hist[l] * 100 / total : 0);
            std::printf("\n");
        }

        std::printf("\n      run              land seen   mode on   slip angle   speed\n");
        struct Run { const char* name; bool brake; };
        const Run runs[] = {{"full lock", false}, {"lock + brake", true}};
        for (const Run& r : runs) {
            car.reset(tuning, ter, rules, grid);
            bigred::VehicleInput in;
            in.accel = !r.brake;
            in.brake = r.brake;
            // Straight-line launch first, then hold the turn.
            bigred::VehicleInput go;
            go.accel = true;
            for (int t = 0; t < 120; ++t) car.step(go);
            in.right = true;
            int mode_on = 0, n = 0;
            long long slip_sum = 0, speed_sum = 0;
            int land_mask = 0;
            for (int t = 0; t < 200; ++t) {
                car.step(in);
                ++n;
                if (car.mode) ++mode_on;
                land_mask |= 1 << (car.land & 7);
                int32_t slip = (car.travel - (car.yaw >> 8)) & 0xFFFF;
                if (slip > 0x8000) slip -= 0x10000;
                slip_sum += std::abs(slip);
                speed_sum += car.speed;
            }
            char lands[32] = {0};
            size_t k = 0;
            for (int l = 0; l < 8; ++l)
                if (land_mask & (1 << l)) k += static_cast<size_t>(
                    std::snprintf(lands + k, sizeof(lands) - k, "%d", l));
            std::printf("      %-16s %-11s %5d%%   %7.2f deg   %5lld\n", r.name, lands,
                        n ? mode_on * 100 / n : 0, deg16(static_cast<int32_t>(slip_sum / n)),
                        speed_sum / n);
        }
    }

    // --- NITRO --------------------------------------------------------------------------
    // "Besides the visual effect, is there actually an effect in the game?" - the user's
    // question, and it is a measurement, not a reading of the code. The boost is a real term in
    // the engine (the original's `input_bits & 8` branch): it ADDS thrust, it holds the
    // accelerator down and locks out the brake for its whole duration, and it raises the speed
    // ceiling from whatever the gearbox tops out at to `nitro[4]`.
    //
    // So: run the same launch twice, once clean and once with the key pressed, and print what
    // the difference actually is.
    if (bloc("NITRO")) {
        const int32_t* N = tuning.nitro;
        std::printf("\nNITRO  same launch, with and without the boost\n");
        std::printf("      %s `nitros %d,%d`\n", level.c_str(), nitros.count,
                    nitros.full_scale_ticks);
        std::printf("      vehicle `hnd nitro`: mode %d, %d ticks (%d ms), thrust %d,"
                    " fades from %d, ceiling %d\n",
                    N[0], N[1], N[1] * bigred::kPhysicsTickMs, N[2], N[3], N[4]);
        {
            // Which of the level's two numbers this vehicle's mode actually spends, and what that
            // buys in seconds - the one figure that makes modes 1 and 2 comparable.
            car.reset(tuning, ter, rules, grid, nitros);
            const int32_t reserve = car.nitro_fuel;
            std::printf("      reserve given to this vehicle: %d (mode %d takes the %s `nitros`"
                        " value) = %d ticks of boost, %.1f s\n",
                        reserve, N[0], N[0] == 1 ? "first" : (N[0] == 2 ? "second" : "no"),
                        reserve * N[1], reserve * N[1] * bigred::kPhysicsTickMs / 1000.0);
        }
        if (N[0] == 3) {
            std::printf("      mode 3 - this is an aircraft: `nitro` is the COLLECTIVE, not a"
                        " boost, and there is no reserve to spend.\n");
        } else {
            // Long enough for the clean run to reach its own ceiling, and for the boost to run
            // out and the car to settle back down: 200 ticks of boost + room either side.
            const int kTicks = 700;
            struct Run { const char* name; bool boost; };
            int32_t peak[2] = {0, 0}, at_end[2] = {0, 0};
            int boost_ticks = 0, first_fire = -1, peak_tick[2] = {0, 0};
            std::printf("\n      run          peak speed   reached at   speed at %d ticks\n",
                        kTicks);
            const Run runs[] = {{"throttle", false}, {"throttle+nitro", true}};
            for (int r = 0; r < 2; ++r) {
                car.reset(tuning, ter, rules, grid, nitros);
                settle(car, 60);
                bigred::VehicleInput in;
                in.accel = true;
                for (int t = 0; t < kTicks; ++t) {
                    // Mode 1 fires on the key's edge: hold it and you get exactly one boost, which
                    // is what a player pressing and holding gets too.
                    in.nitro = runs[r].boost;
                    car.step(in);
                    if (runs[r].boost && car.boost_active) {
                        ++boost_ticks;
                        if (first_fire < 0) first_fire = t;
                    }
                    if (car.speed > peak[r]) { peak[r] = car.speed; peak_tick[r] = t; }
                }
                at_end[r] = car.speed;
                std::printf("      %-14s %10d %10d   %14d\n", runs[r].name, peak[r], peak_tick[r],
                            at_end[r]);
            }
            std::printf("\n      the boost ran %d ticks (%d ms) from tick %d\n", boost_ticks,
                        boost_ticks * bigred::kPhysicsTickMs, first_fire);
            std::printf("      peak speed %+d (%+.1f%%), and the gearbox alone tops out at %d"
                        " against a nitro ceiling of %d\n",
                        peak[1] - peak[0],
                        peak[0] ? 100.0 * (peak[1] - peak[0]) / static_cast<double>(peak[0]) : 0.0,
                        peak[0], N[4]);

            // What a player can actually feel besides the top speed: the boost holds the throttle
            // and refuses the brake for its whole duration. Brake through a boost and see.
            {
                car.reset(tuning, ter, rules, grid, nitros);
                settle(car, 60);
                bigred::VehicleInput go;
                go.accel = true;
                for (int t = 0; t < 200; ++t) car.step(go);
                const int32_t before = car.speed;
                bigred::VehicleInput brake_only;
                brake_only.brake = true;
                int32_t plain = 0;
                {
                    bigred::VehicleSim copy = car;
                    for (int t = 0; t < 40; ++t) copy.step(brake_only);
                    plain = copy.speed;
                }
                bigred::VehicleInput brake_boost = brake_only;
                brake_boost.nitro = true;
                for (int t = 0; t < 40; ++t) car.step(brake_boost);
                std::printf("      braking from %d for 40 ticks: %d without nitro, %d with"
                            " (the boost locks the brake out)\n", before, plain, car.speed);
            }

            // And the reserve is finite: fire it until it is empty.
            {
                car.reset(tuning, ter, rules, grid, nitros);
                settle(car, 60);
                bigred::VehicleInput in;
                in.accel = true;
                int fired = 0, total_on = 0;
                int32_t prev_fuel = car.nitro_fuel;
                for (int t = 0; t < 40000; ++t) {
                    // Tap the key: mode 1 needs an edge, mode 2 does not care.
                    in.nitro = (t % 4) < 2;
                    car.step(in);
                    // Count SPENDING, not the flame. A tap landing on the exact tick the timer
                    // runs out fires the next boost seamlessly, so the flame never goes out and
                    // counting its edges reports one long boost instead of four. That chaining is
                    // the original's own behaviour (it fires whenever the timer is at 0
                    // and the reserve is not), not an artefact of the bench.
                    if (car.nitro_fuel < prev_fuel) ++fired;
                    prev_fuel = car.nitro_fuel;
                    if (car.boost_active) ++total_on;
                }
                std::printf("      reserve: %d boost(s) fired for %d ticks total, then nothing"
                            " (fuel %d)\n", fired, total_on, car.nitro_fuel);
            }
        }
    }

    // --- SOUND --------------------------------------------------------------------------
    // Every `hnd fx` line this vehicle declares, resolved to a file. Nothing here opens an audio
    // device, and that is the point: "I hear nothing" has three quite different causes - the
    // line is not declared, the sample is not on disk, or the trigger never fires - and only the
    // first two are visible without speakers.
    if (bloc("SOUND")) {
        std::printf("\nSOUND  %s, les lignes `hnd fx` et leurs echantillons\n",
                    vehicle_name.c_str());
        bigred::SoundBank bank;
        bank.configure(game_dir, vehicle_name, level, bigred::env_name_for_level(level));

        static const char* kCat[] = {"engine", "nitro", "ground", "skid", "land", "hit", "check"};
        // What the INDEX means changes with the category - that is the design, not an accident.
        static const char* kIndexMeaning[] = {"-", "0=turbo 1=vide", "type de sol", "type de sol",
                                              "type de sol", "type de choc", "evenement"};
        int declared = 0, resolved = 0, missing = 0;
        // ⚠️ `n3` and `n4` are the third and fourth numbers of the line. They are printed
        // because the third one is the difference between a rolling sound that SUSTAINS and one
        // that is a single crunch: `GEN/DRV_ROCK.WAV` is byte-for-byte `GEN/BANG3.WAV`, the
        // standard impact, so looping it turns driving on that surface into a crash every 0.7 s.
        std::printf("      %-7s %-3s %-9s %-6s %8s %8s %3s %3s %6s %7s  %s\n", "cat", "idx",
                    "echantillon", "dossier", "boucle", "fin", "n3", "n4", "freq", "freq max",
                    "fichier");
        for (int c = 0; c < bigred::kFxCategoryCount; ++c) {
            for (int i = 0; i < bigred::kFxIndexCount; ++i) {
                const bigred::FxSound& e = tuning.fx[c][i];
                if (!e.declared()) continue;
                ++declared;
                const std::string path = bank.resolve(e.dir, e.name);
                if (path.empty()) ++missing; else ++resolved;
                // Only the tail of the path is interesting; the install prefix is the same for all.
                std::string tail = path;
                const size_t cut = tail.find_last_of('\\');
                if (cut != std::string::npos && cut > 0) {
                    const size_t cut2 = tail.find_last_of('\\', cut - 1);
                    if (cut2 != std::string::npos) tail = tail.substr(cut2 + 1);
                }
                std::printf("      %-7s %-3d %-9s %-6s %8d %8d %3d %3d %6d %7d  %s\n", kCat[c], i,
                            e.name.c_str(), e.dir.empty() ? "gen" : e.dir.c_str(),
                            e.loop_start, e.loop_end, e.one_shot ? 1 : 0, e.p4, e.freq,
                            e.freq_full,
                            path.empty() ? "ABSENT" : tail.c_str());
            }
        }
        std::printf("      %d ligne(s), %d resolue(s), %d sans fichier\n", declared, resolved,
                    missing);
        for (int c = 0; c < bigred::kFxCategoryCount; ++c) {
            int n = 0;
            for (int i = 0; i < bigred::kFxIndexCount; ++i) if (tuning.fx[c][i].declared()) ++n;
            if (n == 0) std::printf("      (aucune ligne `%s` : ce vehicule est muet la-dessus - "
                                    "%s)\n", kCat[c], kIndexMeaning[c]);
        }

        // And now the TRIGGERS, which is the third cause. Drive from the grid with no audio
        // device at all - VehicleAudio degrades to bookkeeping - and count what each tick asks
        // for. A voice that is never requested is a wiring bug; a voice requested every tick on
        // a sample that does not exist is a data gap. The two read differently here.
        bigred::Mixer silent;            // never opened: every call is a no-op
        bigred::VehicleAudio va;
        va.configure(&tuning, &bank, &silent);
        bigred::SceneryState sworld;
        sworld.resize(scene.instances.size());
        car.reset(tuning, ter, rules, grid, nitros);
        car.world = &sworld;
        bigred::VehicleInput go;
        go.accel = true;
        int want[4] = {0, 0, 0, 0};      // engine, ground, skid, nitro
        int have[4] = {0, 0, 0, 0};      // ...and a `hnd fx` line existed for that surface
        int oneshots = 0;
        std::map<int, int> silent_surface;  // surfaces driven on with no `ground` line
        for (int t = 0; t < 300; ++t) {
            go.right = t > 120;          // straight, then hard over, as the SPOT block does
            go.nitro = (t % 97) == 0;    // and ask for a boost now and then
            car.step(go);
            va.on_tick(car, /*distance=*/600, /*pan=*/128);
            const bigred::VehicleAudio::Debug& d = va.debug();
            const bigred::VehicleAudio::SlotDebug* slots[4] = {&d.engine, &d.ground, &d.skid,
                                                               &d.nitro};
            for (int k = 0; k < 4; ++k) {
                if (slots[k]->want) ++want[k];
                if (slots[k]->want && slots[k]->declared) ++have[k];
            }
            if (d.ground.want && !d.ground.declared) ++silent_surface[d.land];
            oneshots += d.one_shots;
        }
        // `voulu` is the simulation asking; `servi` is a `hnd fx` line existing for that
        // surface. The GAP between them is data, not a bug: a JEEP has no rolling sound on the
        // track, so it rolls in silence there and only the tyres on grass or water are heard.
        std::printf("\n      300 ticks depuis la grille (sans peripherique audio) :\n");
        static const char* kSlot[4] = {"moteur", "roulage", "derapage", "nitro"};
        std::printf("        %-9s %8s %8s %8s\n", "voix", "voulu", "servi", "demarr.");
        {
            const bigred::VehicleAudio::Debug& d = va.debug();
            const bigred::VehicleAudio::SlotDebug* sl[4] = {&d.engine, &d.ground, &d.skid,
                                                           &d.nitro};
            for (int k = 0; k < 4; ++k)
                std::printf("        %-9s %5d/300 %5d/300 %8d\n", kSlot[k], want[k], have[k],
                            sl[k]->starts);
        }
        // ⚠️ `demarr.` is only readable for a ONE-SHOT entry here. With no device every `play`
        // fails, so a LOOPING voice is re-attempted on every tick and its count is the number
        // of ticks it was wanted, not the number of times it would really have started. A
        // one-shot is not re-attempted, by design - which is the whole point of the column:
        // `roulage` on a surface whose line is a one-shot must read 1 per crossing, not 300.
        // `GEN/DRV_ROCK.WAV` IS `GEN/BANG3.WAV`, so 300 there is the crash sound on loop.
        {
            // Which one-shots fired, and how loud the loudest was. Volume is capped at 0x7f =
            // 127 by the mixer (the original's own clamp), so a peak at 127 means a real crash
            // and a peak around 30 means the run only ever brushed things.
            const bigred::VehicleAudio::Debug& d = va.debug();
            std::printf("        %d son(s) ponctuel(s) : %d atterrissage(s), %d choc(s), "
                        "%d ecrasement(s), %d nitro refuse(s), %d explosion(s)\n",
                        oneshots, d.n_land, d.n_hit, d.n_splat, d.n_notturb, d.n_explode);
            std::printf("        volume ponctuel le plus fort demande : %d (plafond 127)\n",
                        d.peak_one_shot);
        }
        // The engine is gated on the vehicle being ALIVE (the original clears its alive flag when
        // damage goes negative), so a run that ends wrecked reports fewer than 300 -
        // correctly. Printing the damage is what makes that number readable instead of
        // mysterious.
        std::printf("        jauge de degats : %d (= `hnd damage` %d, degats subis %d%%)\n",
                    car.damage_capacity, tuning.damage, car.damage_option);
        std::printf("        degats en fin de run : %d%% (%s)\n", car.damage_percent,
                    car.damage < 0 ? "DETRUIT, moteur coupe - conforme" : "en etat");
        if (want[0] == 0)
            std::printf("        ATTENTION le moteur n'est jamais demande\n");
        for (const auto& kv : silent_surface)
            std::printf("        (sol %d : %d tick(s) roules sans ligne `ground` - silencieux "
                        "par conception)\n", kv.first, kv.second);
    }

    // --- SPOT ---------------------------------------------------------------------------
    // The ground effects. Three questions, and the screenshot answers none of them: does the
    // environment declare anything for THIS level, did the flipbooks bake, and does driving
    // actually fill the pool? The emission is doubly gated - on the surface type under the
    // wheels matching the `spot` line, and on the per-corner ground mask - so an effect can be
    // perfectly loaded and never fire because the car is on a surface no line covers.
    if (bloc("SPOT")) {
        std::printf("\nSPOT  effets de sol, %s\n", level.c_str());
        const std::string env_name = bigred::env_name_for_level(level);
        bigred::Archive env;
        try {
            env = *bigred::game_archive(game_dir, env_name + ".ENV");
        } catch (const std::exception& e) {
            std::printf("      %s.ENV illisible: %s\n", env_name.c_str(), e.what());
        }
        std::vector<bigred::SpotEffect> spots;
        auto txt = env.find(env_name + ".TXT");
        if (txt != env.end())
            spots = bigred::parse_spot_effects(std::string(txt->second.begin(), txt->second.end()));
        std::printf("      %s.ENV -> %zu ligne(s) spot\n", env_name.c_str(), spots.size());

        const bigred::ParticleVisuals vis = bigred::build_particle_visuals(spots, env, scene);
        std::printf("\n      %-4s %-9s %-5s %7s %6s %9s %8s\n", "type", "forme", "sol",
                    "images", "ticks", "duree", "sommets");
        for (size_t i = 0; i < spots.size(); ++i) {
            const bigred::SpotEffect& sp = spots[i];
            size_t verts = 0, sprites = 0;
            if (i < vis.kinds.size()) {
                for (const std::vector<bigred::TerrainVertex>& v : vis.kinds[i].by_frame)
                    verts += v.size();
                for (const std::vector<bigred::ParticleSprite>& v : vis.kinds[i].sprites_by_frame)
                    sprites += v.size();
            }
            // Lifetime in wall clock: one frame is held `speed` TIMER ticks of 14 ms, and the
            // particle runs frames 0..frame_count inclusive.
            const int life_ms = (sp.frame_count + 1) * sp.speed * bigred::kGameTickMs;
            const char* kind = sp.type == 1 ? "derap" : (sp.type == 2 ? "roule" : "choc");
            // A sprite-only effect (SPARK_FX) has no faces at all: everything it shows is
            // camera-facing billboards, so a zero in the vertex column is correct there.
            static char spr_note[32];
            auto sprite_note = [](size_t n) {
                std::snprintf(spr_note, sizeof(spr_note), "  +%zu billboards", n);
                return static_cast<const char*>(spr_note);
            };
            std::printf("      %-4s %-9s %5d %7d %6d %6d ms %8zu%s\n", kind, sp.shape.c_str(),
                        sp.land, sp.frame_count, sp.speed, life_ms, verts,
                        sprites ? sprite_note(sprites) : (verts ? "" : "  RIEN"));
        }

        // WHICH IMPACT SOUND each destructible shape of this circuit makes on this vehicle.
        // The index comes from the prop's own `var` (MAP.DAT offset 21) through the original's
        // rule, so this table is the one that shows a sheep and a cone are not the same sound -
        // they were, when the index was hard-coded (user, 2026-09-21).
        {
            std::map<std::string, int> var_of;
            for (const bigred::SceneryInstance& in : scene.instances) {
                const int cat = bigred::collision_category_index(in.collision_category_byte);
                if (!bigred::category_is_destructible(cat)) continue;
                if (in.shape_index < 0 ||
                    static_cast<size_t>(in.shape_index) >= scene.shape_names_by_slot.size()) continue;
                var_of[scene.shape_names_by_slot[static_cast<size_t>(in.shape_index)]] =
                    in.aux_byte & 0xff;
            }
            if (!var_of.empty()) {
                std::printf("\n      ecrasements : quel son pour quel objet\n");
                std::printf("      %-11s %4s %4s  %s\n", "forme", "var", "hit", "echantillon");
                for (const auto& kv : var_of) {
                    const int v = kv.second;
                    const int idx = (v >= 1 && v <= 3) ? (3 + v) : 1;
                    const bigred::FxSound& e = tuning.fx[static_cast<int>(bigred::FxCategory::Hit)][idx];
                    std::printf("      %-11s %4d %4d  %s\n", kv.first.c_str(), v, idx,
                                e.declared() ? e.name.c_str() : "(non declare - muet)");
                }
            }
        }

        // And now drive, because the table above says nothing about whether anything ever
        // FIRES. Two runs from the grid: straight, then hard over. The second one is the
        // interesting one - on most environments the only `spot 1` lines are the dust, so a car
        // that never slides raises nothing at all, and that is correct behaviour, not a defect.
        struct Run { const char* name; bool steer; };
        const Run runs[2] = {{"tout droit", false}, {"braquage a fond", true}};
        for (const Run& r : runs) {
            bigred::ParticleField field;
            field.configure(spots);
            bigred::SceneryState fxworld;
            fxworld.resize(scene.instances.size());
            car.reset(tuning, ter, rules, grid, nitros);
            car.world = &fxworld;
            bigred::VehicleInput go;
            go.accel = true;
            go.right = r.steer;
            int peak = 0, first_tick = -1, total = 0, sliding = 0, grounded = 0, turning = 0;
            std::map<int, int> land_ticks;
            std::map<std::string, int> born;
            std::vector<int> was_live(bigred::kParticleSlots, 0);
            const int ticks = 300;
            for (int t = 0; t < ticks; ++t) {
                car.step(go);
                if (car.skidding) ++sliding;  // the skid flag, |vlat| > 5000 - the real thing
                if (car.mode) ++turning;      // the yaw-rate limiter - what this WRONGLY used at first
                if ((car.near_ground & 0xf) != 0) ++grounded;
                ++land_ticks[car.land];
                field.emit(car, static_cast<uint32_t>(t));
                // Births, counted by comparing the slot table across the emit: a slot that was
                // free and is now live is one particle, whoever it overwrote.
                for (int i = 0; i < bigred::kParticleSlots; ++i) {
                    const bigred::Particle& pt = field.slots()[static_cast<size_t>(i)];
                    if (pt.alive() && was_live[static_cast<size_t>(i)] == 0) {
                        ++total;
                        if (first_tick < 0) first_tick = t;
                        if (pt.kind >= 0 && static_cast<size_t>(pt.kind) < spots.size())
                            ++born[spots[static_cast<size_t>(pt.kind)].shape];
                    }
                }
                field.tick();
                field.tick();
                for (int i = 0; i < bigred::kParticleSlots; ++i)
                    was_live[static_cast<size_t>(i)] =
                        field.slots()[static_cast<size_t>(i)].alive() ? 1 : 0;
                peak = std::max(peak, field.live_count());
            }
            // `mode` is printed next to the real flag on purpose: the GAP between the two
            // columns is the defect the user saw. Dust follows `glisse`, never `mode`.
            std::printf("\n      %d ticks, %s : %d au sol, %d en glisse, %d en virage (mode),"
                        " sols vus", ticks, r.name, grounded, sliding, turning);
            for (const auto& kv : land_ticks) std::printf(" %d(x%d)", kv.first, kv.second);
            std::printf("\n");
            if (total == 0) {
                std::printf("        aucune particule - aucune ligne spot ne couvre ces sols, "
                            "ou rien ne touchait\n");
            } else {
                std::printf("        %d nee(s), 1re au tick %d, pic %d/%d vivantes :",
                            total, first_tick, peak, bigred::kParticleSlots);
                for (const auto& kv : born) std::printf("  %s x%d", kv.first.c_str(), kv.second);
                std::printf("\n");
            }
        }
    }

    // --- PARTS --------------------------------------------------------------------------
    // Every `wheel` line of the exterior section, and what the renderer made of it. Two defects
    // lived here and neither showed in a vertex total: the SNOWCAT drove with no tracks (`&FF`
    // anchors were skipped outright) and the BUGGY with its three right wheels missing (their
    // `&8B` index runs past the `wheelsh` lines, because every name that owns the team variants
    // is deployed into 7 slots). An anchor that resolves to nothing
    // prints NO SHAPE here, which is the whole point of the block.
    if (bloc("PARTS")) {
        std::printf("\nPARTS  %s, the `wheel` anchors of the exterior section\n",
                    vehicle_name.c_str());
        std::unordered_map<std::string, bigred::Blob> no_ingame;
        const bigred::VehicleModel pm =
            bigred::build_vehicle_model(*vhc, no_ingame, tuning, 1, scene);
        size_t resolved = 0, expected = 0;
        if (!tuning.sections.empty()) {
            const bigred::VehicleSection& sec = tuning.sections[0];
            std::printf("      shape table: %zu name(s) deployed into %zu slot(s)\n",
                        sec.wheelsh.size(), sec.shape_table.size());
            for (const bigred::WheelAttach& w : sec.wheels) {
                if (w.is_effect()) continue; // emitters are the FX block's job
                ++expected;
                const char* role = w.is_wheel() ? (w.is_steered() ? "wheel/steered" : "wheel")
                                  : w.is_powered() ? "rotor/powered"
                                  : "decor &FF";
                const bool have_mesh = !w.shape.empty() && vhc->count(w.shape + ".3DW") != 0;
                if (have_mesh) ++resolved;
                const char* note = have_mesh ? ""
                                   : w.is_global_shape()
                                       ? "- the player's character, never in the .VHC"
                                       : "- MISSING";
                std::printf("      &%02X sel &%02X  %-13s %-9s %s\n",
                            static_cast<unsigned>(w.flags) & 0xFFu,
                            static_cast<unsigned>(w.shape_sel) & 0xFFu, role,
                            w.shape.empty() ? (w.is_global_shape() ? "(global)" : "NO SHAPE")
                                            : w.shape.c_str(),
                            note);
            }
        }
        size_t parts_v = 0;
        for (const bigred::VehiclePart& pt : pm.parts) parts_v += pt.local.size();
        std::printf("      %zu of %zu anchor(s) resolved, %zu part(s) built, %zu vertices\n",
                    resolved, expected, pm.parts.size(), parts_v);

        // WHICH WAY EACH WHEEL TURNS, measured instead of eyeballed. `wheel_spin` grows with
        // forward speed (sim/vehicle.cpp), the body's +Z is its front, so the material point
        // sitting at the bottom of a rolling wheel has to travel REARWARD. Until 2026-09-21 it
        // did on one side only: the two sides of a vehicle are yawed half a turn apart, and the
        // roll was applied in the mesh's own frame, ahead of that yaw. The user saw it the day
        // the BUGGY's right wheels appeared - "they turn as if the vehicle were reversing".
        {
            bigred::VehiclePose still;             // zeroed: no body rotation, at the origin
            bigred::VehiclePose rolled = still;
            rolled.wheel_spin = 2048.0f;           // an eighth of a turn
            const std::vector<bigred::TerrainVertex> va =
                bigred::place_vehicle(pm, still, scene.map_w, 1.0f);
            const std::vector<bigred::TerrainVertex> vb =
                bigred::place_vehicle(pm, rolled, scene.map_w, 1.0f);
            size_t off = pm.body.size(), wrong = 0, rolling = 0;
            for (const bigred::VehiclePart& pt : pm.parts) {
                const size_t n = pt.local.size();
                if (!pt.rolls || n == 0 || off + n > va.size() || off + n > vb.size()) {
                    off += n;
                    continue;
                }
                ++rolling;
                size_t bottom = off;               // render space is +Y up: lowest = smallest y
                for (size_t i = off; i < off + n; ++i)
                    if (va[i].y < va[bottom].y) bottom = i;
                const float dz = vb[bottom].z - va[bottom].z;
                if (dz > 0.0f) ++wrong;
                std::printf("      wheel at x=%+5d  contact point moves %+7.1f in Z  %s\n",
                            pt.pos[0], dz,
                            dz < 0.0f ? "rearward - rolls forward"
                                      : "FORWARD - TURNS AS IF REVERSING");
                off += n;
            }
            std::printf("      %zu rolling wheel(s), %zu turning the wrong way\n",
                        rolling, wrong);

            // AND THE ROLL MUST STAY ON THE AXLE WHILE THE WHEEL IS STEERED. A wheel turns
            // about its own axle, full stop: no vertex may travel ALONG that axle when only
            // `wheel_spin` changes. Roll it about anything else - the body's X axis, say - and a
            // steered wheel cones about a third axis, by an amount proportional to the steering.
            // The user saw exactly that on 2026-09-21, and no amount of staring at a wheel
            // standing straight ahead would have shown it, which is why the test steers first.
            bigred::VehiclePose locked = still;
            locked.steer = 256.0f;                 // full lock
            bigred::VehiclePose locked_rolled = locked;
            locked_rolled.wheel_spin = 2048.0f;
            const std::vector<bigred::TerrainVertex> sa =
                bigred::place_vehicle(pm, locked, scene.map_w, 1.0f);
            const std::vector<bigred::TerrainVertex> sb =
                bigred::place_vehicle(pm, locked_rolled, scene.map_w, 1.0f);
            const float kBamToRad = 6.283185307179586f / 65536.0f;

            float worst = 0.0f;
            off = pm.body.size();
            for (const bigred::VehiclePart& pt : pm.parts) {
                const size_t n = pt.local.size();
                if (!pt.rolls || n == 0 || off + n > sa.size() || off + n > sb.size()) {
                    off += n;
                    continue;
                }
                // The axle, in RENDER space: the mesh's local Z after the part's yaw, and world
                // X is mirrored (place_vehicle), so its X component changes sign.
                const float ang = (static_cast<float>(pt.rot_bam16[1]) +
                                   locked.steer * static_cast<float>(pt.steer_gain)) * kBamToRad;
                const float ax = -std::sin(ang), az = std::cos(ang);
                for (size_t i = off; i < off + n; ++i) {
                    const float drift = (sb[i].x - sa[i].x) * ax + (sb[i].z - sa[i].z) * az;
                    worst = std::max(worst, std::fabs(drift));
                }
                off += n;
            }
            std::printf("      at full lock, worst slide of a vertex ALONG its axle: %.3f"
                        " units (0 = the roll is on the axle)\n", worst);

            // HOW EACH POWERED PART IS AIMED. Rotors, fans and rudders are authored like a
            // wheel - the disc in the mesh's own XY plane, the axle along its local Z - and it
            // is the anchor's THREE angles that aim them: a main rotor pitches a quarter turn
            // to lie flat, a tail rotor yaws a quarter turn to stand across the tail. Keeping
            // only the yaw, as the renderer did until 2026-09-21, laid every main rotor on its
            // side. The thin axis of the placed box is the axle, so it says which happened.
            // WHAT THE STEERING MOVES. The gain per part is the original's: 15 BAM per
            // unit for a steered wheel, MINUS its `size` field for a `&80` rudder. A part with
            // no gain must not budge between `still` and `locked` - that is how the six-wheelers
            // read, since the original steers none of their wheels.
            off = pm.body.size();
            for (const bigred::VehiclePart& pt : pm.parts) {
                const size_t n = pt.local.size();
                if (n == 0 || off + n > va.size() || off + n > sa.size()) {
                    off += n;
                    continue;
                }
                float swing = 0.0f;
                for (size_t i = off; i < off + n; ++i) {
                    const float dx = sa[i].x - va[i].x, dy = sa[i].y - va[i].y,
                                dz = sa[i].z - va[i].z;
                    swing = std::max(swing, std::sqrt(dx * dx + dy * dy + dz * dz));
                }
                if (pt.steer_gain != 0 || swing > 0.01f)
                    std::printf("      &%02X  steer gain %+4d BAM/unit -> swings %6.1f units at"
                                " full lock\n", static_cast<unsigned>(pt.flags) & 0xFFu,
                                pt.steer_gain, swing);
                off += n;
            }

            bigred::VehiclePose turned = still;
            turned.rotor_spin = static_cast<float>(bigred::kRotorSpinPerTick);
            const std::vector<bigred::TerrainVertex> rb =
                bigred::place_vehicle(pm, turned, scene.map_w, 1.0f);
            off = pm.body.size();
            for (const bigred::VehiclePart& pt : pm.parts) {
                const size_t n = pt.local.size();
                if ((pt.flags != 0x80 && pt.flags != 0x81 && pt.flags != 0x83) || n == 0 ||
                    off + n > va.size() || off + n > rb.size()) {
                    off += n;
                    continue;
                }
                float lo[3] = {va[off].x, va[off].y, va[off].z};
                float hi[3] = {va[off].x, va[off].y, va[off].z};
                for (size_t i = off; i < off + n; ++i) {
                    const float v[3] = {va[i].x, va[i].y, va[i].z};
                    for (int k = 0; k < 3; ++k) {
                        lo[k] = std::min(lo[k], v[k]);
                        hi[k] = std::max(hi[k], v[k]);
                    }
                }
                const float d[3] = {hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]};
                const int thin = (d[0] <= d[1] && d[0] <= d[2]) ? 0 : (d[1] <= d[2] ? 1 : 2);
                static const char* kAxis[3] = {"X (stands across the vehicle)",
                                               "Y (lies flat, main rotor)",
                                               "Z (stands fore-and-aft)"};
                // AND WHICH WAY IT TURNS. One tick of rotor spin, then follow the blade tip:
                // the cross product of its before and after radius vectors points along the
                // axle, and its sign is the sense of rotation. The UFO's two discs are the
                // same shape pitched +90 and -90, so their signs MUST come out opposite - that
                // is what makes them counter-rotating, and it is the one thing here that the
                // data itself asserts rather than us.
                const float c[3] = {(lo[0] + hi[0]) * 0.5f, (lo[1] + hi[1]) * 0.5f,
                                    (lo[2] + hi[2]) * 0.5f};
                size_t tip = off;
                float best = -1.0f;
                for (size_t i = off; i < off + n; ++i) {
                    const float r[3] = {va[i].x - c[0], va[i].y - c[1], va[i].z - c[2]};
                    const float q = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
                    if (q > best) { best = q; tip = i; }
                }
                const float r0[3] = {va[tip].x - c[0], va[tip].y - c[1], va[tip].z - c[2]};
                const float r1[3] = {rb[tip].x - c[0], rb[tip].y - c[1], rb[tip].z - c[2]};
                const float cr[3] = {r0[1] * r1[2] - r0[2] * r1[1],
                                     r0[2] * r1[0] - r0[0] * r1[2],
                                     r0[0] * r1[1] - r0[1] * r1[0]};
                const float moved = std::sqrt((r1[0] - r0[0]) * (r1[0] - r0[0]) +
                                              (r1[1] - r0[1]) * (r1[1] - r0[1]) +
                                              (r1[2] - r0[2]) * (r1[2] - r0[2]));
                int big = (std::fabs(cr[0]) >= std::fabs(cr[1]) &&
                           std::fabs(cr[0]) >= std::fabs(cr[2])) ? 0
                          : (std::fabs(cr[1]) >= std::fabs(cr[2]) ? 1 : 2);
                std::printf("      powered &%02X  box %5.0f x %5.0f x %5.0f  axle on %-28s"
                            "  tip moves %6.1f in one tick, sense %c%c\n",
                            static_cast<unsigned>(pt.flags) & 0xFFu, d[0], d[1], d[2],
                            kAxis[thin], moved,
                            moved < 0.01f ? ' ' : (cr[big] < 0.0f ? '-' : '+'),
                            moved < 0.01f ? '-' : static_cast<char>('X' + big));
                off += n;
            }
        }
    }

    // --- FX -----------------------------------------------------------------------------
    // The boost flame is a FLIPBOOK, resolved at load: NIT_FX's display list is a branch
    // tree over the animation counter whose leaves are crop records into SPOTFX3.TM's
    // sprite sheet. Whether a phase produced geometry is a
    // load-time data question, so it belongs here rather than in a screenshot - and the
    // game's own stdout cannot be read, because a windowed build never flushes it.
    if (bloc("FX")) {
        std::printf("\nFX  nitro flame, baked per animation phase\n");
        bigred::Archive ingame;
        try {
            ingame = *bigred::game_archive(game_dir, "INGAME.WAP");
        } catch (const std::exception& e) {
            std::printf("      INGAME.WAP unreadable: %s\n", e.what());
        }
        const bigred::VehicleModel vm =
            bigred::build_vehicle_model(*vhc, ingame, tuning, 1, scene);
        // How much geometry the car actually IS, body and wheels. The display-list replay decides
        // this (which branches are taken picks which faces are reached), so it is the number that
        // moves when the replay's reading of a branch record changes - and the only cheap way to
        // tell a replay change apart from a rendering change on a vehicle, where map_preview's
        // per-level vertex total says nothing.
        {
            size_t parts_v = 0;
            for (const bigred::VehiclePart& p : vm.parts) parts_v += p.local.size();
            std::printf("      geometry: body %zu vertices, %zu part(s) %zu vertices\n",
                        vm.body.size(), vm.parts.size(), parts_v);
        }
        std::printf("      %zu emitter(s) (&87/&88)\n", vm.nitro_fx.size());
        int lo = -1, hi = -1, frames = 0;
        size_t tris = 0;
        for (int i = 0; i < bigred::kFlamePhaseCount; ++i) {
            size_t n = 0;
            for (const bigred::VehicleFx& f : vm.nitro_fx)
                if (static_cast<size_t>(i) < f.by_phase.size())
                    n += f.by_phase[static_cast<size_t>(i)].size();
            if (n == 0) continue;
            ++frames;
            tris += n / 3;
            if (lo < 0) lo = i;
            hi = i;
        }
        std::printf("      %d phase(s) with geometry, %d..%d, %zu triangles in all\n",
                    frames, lo, hi, tris);
        // The simulation only ever shows 0 (off) and 0x11..0x20, so those are the ones that
        // have to be there. A gap inside that range is a hole in the animation.
        int missing = 0;
        for (int i = 0x11; i <= 0x20; ++i) {
            size_t n = 0;
            for (const bigred::VehicleFx& f : vm.nitro_fx)
                if (static_cast<size_t>(i) < f.by_phase.size())
                    n += f.by_phase[static_cast<size_t>(i)].size();
            if (n == 0) ++missing;
        }
        std::printf("      of the 16 phases the boost actually uses (0x11..0x20): %d empty\n",
                    missing);
        // And which crop each of those phases landed on, as a texel rect - if they are all
        // the same rect, the branch tree is not being replayed and the flame will not move.
        std::set<std::string> rects;
        for (int i = 0x11; i <= 0x20; ++i) {
            for (const bigred::VehicleFx& f : vm.nitro_fx) {
                if (static_cast<size_t>(i) >= f.by_phase.size()) continue;
                const std::vector<bigred::TerrainVertex>& m = f.by_phase[static_cast<size_t>(i)];
                if (m.empty()) continue;
                float u0 = 2, v0 = 2, u1 = -1, v1 = -1;
                for (const bigred::TerrainVertex& v : m) {
                    u0 = std::min(u0, v.u); u1 = std::max(u1, v.u);
                    v0 = std::min(v0, v.v); v1 = std::max(v1, v.v);
                }
                char buf[80];
                std::snprintf(buf, sizeof buf, "%.0f,%.0f..%.0f,%.0f",
                              u0 * bigred::kTmWidth, v0 * bigred::kTmHeight,
                              u1 * bigred::kTmWidth, v1 * bigred::kTmHeight);
                rects.insert(buf);
            }
        }
        std::printf("      %zu distinct texel crop(s) over those phases:", rects.size());
        for (const std::string& r : rects) std::printf("  [%s]", r.c_str());
        std::printf("\n");
        // The damage effects (&89 FLAMES, &8A..&8C SMOKE, &8D BOOM) and the wreck: per emitter,
        // how many of its eight frames came out with geometry, and how many vertices the
        // biggest has. A frame at 0 is a hole in the animation; an emitter at 0/8 draws nothing.
        std::printf("      damage effects: %zu emitter(s), wreck body %zu vertices, %zu part angle(s)\n",
                    vm.damage_fx.size(), vm.wreck_body.size(), vm.wreck_rot.size());
        for (const bigred::VehicleDamageFx& d : vm.damage_fx) {
            int filled = 0;
            size_t most = 0;
            for (const std::vector<bigred::TerrainVertex>& f : d.by_phase) {
                if (!f.empty()) ++filled;
                most = std::max(most, f.size());
            }
            for (const std::vector<bigred::ParticleSprite>& f : d.sprites_by_phase)
                if (!f.empty() && d.by_phase.size() == d.sprites_by_phase.size()) {
                    const size_t k = static_cast<size_t>(&f - d.sprites_by_phase.data());
                    if (d.by_phase[k].empty()) ++filled;
                }
            std::printf("        &%02X at %d,%d,%d: %d/%d frame(s), up to %zu vertices", d.flags,
                        d.pos[0], d.pos[1], d.pos[2], filled, bigred::kDamageFxPhases, most);
            // Its extent round the anchor, render space (+Y up), frame 0.
            if (!d.by_phase.empty() && !d.by_phase[0].empty()) {
                float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
                for (const bigred::TerrainVertex& v : d.by_phase[0]) {
                    const float p[3] = {v.x, v.y, v.z};
                    for (int k = 0; k < 3; ++k) { lo[k] = std::min(lo[k], p[k]); hi[k] = std::max(hi[k], p[k]); }
                }
                std::printf("  x %.0f..%.0f  y(up) %.0f..%.0f  z %.0f..%.0f", lo[0], hi[0], lo[1],
                            hi[1], lo[2], hi[2]);
            }
            std::printf("\n");
        }
        // Each effect shape over EVERY variant byte: which values draw anything. The frames
        // above assume 0..7; a shape whose tree branches on other bits shows it here.
        for (const char* name : {"FLAMES", "SMOKE", "BOOM"}) {
            const auto it = ingame.find(std::string(name) + ".3DW");
            if (it == ingame.end()) { std::printf("        %s.3DW: absent\n", name); continue; }
            try {
                const bigred::ThreeDW m = bigred::parse_3dw(it->second.data(), it->second.size());
                std::printf("        %s.3DW: %zu face(s), %zu crop(s), texture '%s'; variants drawing:",
                            name, m.faces.size(), m.texture_variants.size(),
                            m.embedded_texture_name.c_str());
                int shown = 0;
                for (int ph = 0; ph < 256; ++ph)
                    if (!bigred::bake_fx_phase(m, 0, ph).empty() ||
                        !bigred::bake_fx_sprites(m, 0, ph).empty()) {
                        if (shown++ < 24) std::printf(" %d", ph);
                    }
                std::printf("  (%d of 256)\n", shown);
            } catch (const std::exception& e) {
                std::printf("        %s.3DW: %s\n", name, e.what());
            }
        }
    }

    // --- FORMES -------------------------------------------------------------------------
    // Every .3DW of every archive of the game through parse_3dw: how many parse, how many throw,
    // and which parse to NO face at all - the census behind the renderer's "shapes that do not
    // parse" line.
    if (bloc("FORMES")) {
        std::printf("\nFORMES  tous les .3DW du jeu, par le parseur\n");
        static const char* kArchives[] = {
            "ALWAYS.WAP", "INGAME.WAP", "FEJEEP.WAP", "CHOPPER1.LEV", "CHOPPER2.LEV",
            "CHOPPER3.LEV", "CITY1.LEV", "CITY2.LEV", "CITY3.LEV", "CITY4.LEV", "DIRT1.LEV",
            "DIRT2.LEV", "DIRT3.LEV", "MOON1.LEV", "MOON2.LEV", "MOON3.LEV", "RIG1.LEV",
            "RIG3.LEV", "RIG4.LEV", "SNOW1.LEV", "SNOW2.LEV", "SNOW3.LEV", "WATER1.LEV",
            "WATER2.LEV", "WATER3.LEV", "ATV.VHC", "BEETLE.VHC", "BUGGY.VHC", "CABBET.VHC",
            "CABMINI.VHC", "CAT.VHC", "COPTER.VHC", "FEHELI.VHC", "HELI.VHC", "HOVCAR.VHC",
            "INFLATE.VHC", "JCB.VHC", "JEEP.VHC", "JETCAR.VHC", "MINI.VHC", "MONS.VHC",
            "PLOUGH.VHC", "SNOWCAT.VHC", "TRUCK.VHC", "UFO.VHC", "CHARSCN.SCN", "GLOBESCN.SCN"};
        std::set<std::string> seen, thrown, empty;
        int total = 0;
        for (const char* a : kArchives) {
            const bigred::Archive& arc = *bigred::game_archive_or_empty(game_dir, a);
            for (const auto& [name, blob] : arc) {
                if (name.size() < 4 || name.compare(name.size() - 4, 4, ".3DW") != 0) continue;
                ++total;
                const std::string key = std::string(a) + "/" + name;
                try {
                    const bigred::ThreeDW m = bigred::parse_3dw(blob.data(), blob.size());
                    if (m.faces.empty() && m.sprites.empty()) empty.insert(key);
                } catch (const std::exception& e) {
                    thrown.insert(key + " (" + e.what() + ")");
                }
                seen.insert(name);
            }
        }
        std::printf("      %d fichier(s), %zu nom(s) distincts ; %zu echec(s), %zu sans face ni sprite\n",
                    total, seen.size(), thrown.size(), empty.size());
        for (const std::string& t : thrown) std::printf("      ECHEC  %s\n", t.c_str());
        for (const std::string& t : empty) std::printf("      VIDE   %s\n", t.c_str());
    }

    // --- GLYPHES ------------------------------------------------------------------------
    // The interface alphabets drawn as sheets, ten glyphs a row, glyph i at row i/10, column
    // i%10 - `glyphs_<FONT>.bmp` in the working directory. What the glyphs past `Z` are (the
    // accented capitals, the key caps) is read off these pictures.
    if (bloc("GLYPHES")) {
        std::printf("\nGLYPHES  les alphabets de l'interface, en planches\n");
        const bigred::Archive& always = *bigred::game_archive_or_empty(game_dir, "ALWAYS.WAP");
        std::vector<bigred::RGB> pal;
        if (const bigred::Blob* col = bigred::find_member(always, "DEFAULT.COL"))
            pal = bigred::load_palette_rgb(col->data(), col->size());
        for (const char* font : {"WIBS1", "SML_ALPH"}) {
            const bigred::Blob* spr = bigred::find_member(always, std::string(font) + ".SPR");
            if (!spr || pal.size() < 256) continue;
            const std::vector<bigred::SpriteGlyph> g = bigred::decode_spr(spr->data(), spr->size());
            int cw = 1, ch = 1;
            for (const bigred::SpriteGlyph& s : g) { cw = std::max(cw, s.width); ch = std::max(ch, s.height); }
            cw += 4; ch += 4;
            const int cols = 10, rows = (static_cast<int>(g.size()) + cols - 1) / cols;
            const int w = cols * cw, h = rows * ch;
            std::vector<uint8_t> rgb(static_cast<size_t>(w) * h * 3, 60);
            for (size_t i = 0; i < g.size(); ++i) {
                const bigred::SpriteGlyph& s = g[i];
                if (!s.valid()) continue;
                const int ox = static_cast<int>(i % cols) * cw + 2, oy = static_cast<int>(i / cols) * ch + 2;
                for (int y = 0; y < s.height; ++y)
                    for (int x = 0; x < s.width; ++x) {
                        const uint8_t p = s.pixels[static_cast<size_t>(y) * s.width + x];
                        if (!p) continue;
                        const size_t o = (static_cast<size_t>(oy + y) * w + (ox + x)) * 3;
                        rgb[o] = pal[p].r; rgb[o + 1] = pal[p].g; rgb[o + 2] = pal[p].b;
                    }
            }
            const std::string path = std::string("glyphs_") + font + ".bmp";
            write_bmp(path, w, h, rgb);
            std::printf("      %-8s %zu glyphe(s), cellule %dx%d -> %s\n", font, g.size(), cw, ch,
                        path.c_str());
        }
    }

    // --- DASH ---------------------------------------------------------------------------
    // The cabin view's dashboard: which vehicles declare one and with
    // which gauges, then this vehicle's sheet drawn at rest, at half and at full scale, written
    // as `dash_<VEHICLE>_<n>.bmp` in the working directory - the needles are judged by eye.
    if (bloc("DASH")) {
        std::printf("\nDASH  tableaux de bord (`dash` + `ndial` de la section interieure)\n");
        static const char* kAll[] = {"ATV", "BEETLE", "BUGGY", "CABBET", "CABMINI", "CAT",
                                     "COPTER", "FEHELI", "HELI", "HOVCAR", "INFLATE", "JCB",
                                     "JEEP", "JETCAR", "MINI", "MONS", "PLOUGH", "SNOWCAT",
                                     "TRUCK", "UFO"};
        static const char* kType[] = {"copy", "point", "pill", "block", "harea", "varea"};
        for (const char* v : kAll) {
            bigred::VehicleTuning vt;
            bigred::ArchivePtr arc;
            if (!bigred::load_vehicle(game_dir, v, vt, &arc)) continue;
            bigred::Dashboard dash;
            const bool ok = dash.load(*arc, vt);
            std::printf("      %-8s dash %-6s %s", v, vt.dash.empty() ? "-" : vt.dash.c_str(),
                        ok ? "" : "(pas de feuille) ");
            if (vt.sections.size() > 1) {
                const auto& in = vt.sections[1];
                std::printf("interne %s (%d,%d,%d, %d) :", in.base.c_str(), in.internal[0],
                            in.internal[1], in.internal[2], in.internal[3]);
                for (const bigred::DashDial& d : in.dials)
                    std::printf(" %s%s", kType[d.type],
                                d.type == bigred::DashDial::Copy ? ""
                                    : ("(" + std::to_string(d.r[1]) + ")").c_str());
            }
            std::printf("\n");
        }
        // Where the cabin sits against the body: both meshes' extents in LOCAL render space
        // (+Y up, true units), the eye (`internal`, game axes +Y down) and each cabin part.
        {
            const auto extent = [](const std::vector<bigred::TerrainVertex>& vs, const char* what) {
                if (vs.empty()) { std::printf("      %-10s (vide)\n", what); return; }
                float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
                for (const bigred::TerrainVertex& v : vs) {
                    const float p[3] = {v.x, v.y, v.z};
                    for (int k = 0; k < 3; ++k) {
                        lo[k] = std::min(lo[k], p[k]);
                        hi[k] = std::max(hi[k], p[k]);
                    }
                }
                std::printf("      %-10s x %7.1f..%7.1f  y %7.1f..%7.1f  z %7.1f..%7.1f  (%zu)\n",
                            what, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2], vs.size());
            };
            for (int sec = 0; sec < 2 && sec < static_cast<int>(tuning.sections.size()); ++sec) {
                const bigred::VehicleModel m = bigred::build_vehicle_model(
                    *vhc, tuning, 1, scene.palette, nullptr, nullptr, nullptr, nullptr, sec);
                extent(m.body, sec == 0 ? "exterieur" : "cabine");
                for (const bigred::VehiclePart& p : m.parts)
                    std::printf("        piece &%02X en %d,%d,%d  rot %d,%d,%d\n", p.flags, p.pos[0],
                                p.pos[1], p.pos[2], p.rot_bam16[0], p.rot_bam16[1], p.rot_bam16[2]);
            }
            // The cabin body's textured faces, as the display-list walk sees them at variant 0:
            // a face it never reaches is not drawn (vehicle_render.cpp, shape_to_local).
            if (tuning.sections.size() > 1) {
                const std::string body = tuning.sections[1].base + "1.3DW";
                if (const auto it = vhc->find(body); it != vhc->end()) {
                    const bigred::ThreeDW m = bigred::parse_3dw(it->second.data(), it->second.size());
                    const bigred::DisplayListReplay r = bigred::replay_display_list(
                        m.branches, m.face_stream_pos, m.crop_stream_pos, m.faces.size(), 0);
                    std::printf("      %s : %zu faces, %zu recadrages, %zu branches\n", body.c_str(),
                                m.faces.size(), m.texture_variants.size(), m.branches.size());
                    for (size_t fi = 0; fi < m.faces.size(); ++fi) {
                        const bigred::Face& f = m.faces[fi];
                        const bool tex = f.textured_variant ||
                                         (!m.texture_variants.empty() && !f.color.has_value());
                        if (!tex) continue;
                        std::printf("        face %2zu  uv propre %d  recadrage %d  atteinte %d  sommets"
                                    " (x,y,z vrais, +Y bas) :", fi,
                                    f.uv.size() == f.indices.size() ? 1 : 0,
                                    fi < r.crop.size() ? r.crop[fi] : -9,
                                    fi < r.drawn.size() ? static_cast<int>(r.drawn[fi]) : -9);
                        for (const int32_t idx : f.indices)
                            if (static_cast<size_t>(idx) < m.vertices.size()) {
                                const bigred::Vec3i& p = m.vertices[static_cast<size_t>(idx)];
                                std::printf(" (%d,%d,%d)", p.x / 2, -p.y / 2, p.z / 2);
                            }
                        std::printf("\n");
                    }
                }
            }
            if (tuning.sections.size() > 1) {
                const auto& in = tuning.sections[1];
                std::printf("      oeil (internal) %d,%d,%d  tangage %d\n", in.internal[0],
                            in.internal[1], in.internal[2], in.internal[3]);
            }
        }
        bigred::Dashboard dash;
        if (dash.load(*vhc, tuning) && scene.palette.size() >= 256) {
            const auto write_bmp = [&](const std::string& path) {
                const int w = bigred::kTmWidth, h = bigred::kTmHeight;
                std::vector<uint8_t> f(54 + static_cast<size_t>(w) * h * 3, 0);
                const auto put32 = [&](size_t at, uint32_t x) {
                    for (int k = 0; k < 4; ++k) f[at + k] = static_cast<uint8_t>(x >> (8 * k));
                };
                f[0] = 'B'; f[1] = 'M';
                put32(2, static_cast<uint32_t>(f.size())); put32(10, 54); put32(14, 40);
                put32(18, static_cast<uint32_t>(w)); put32(22, static_cast<uint32_t>(h));
                f[26] = 1; f[28] = 24;
                for (int y = 0; y < h; ++y)
                    for (int x = 0; x < w; ++x) {
                        const bigred::RGB& c =
                            scene.palette[dash.pixels()[static_cast<size_t>(y) * w + x]];
                        const size_t o = 54 + (static_cast<size_t>(h - 1 - y) * w + x) * 3;
                        f[o] = c.b; f[o + 1] = c.g; f[o + 2] = c.r;
                    }
                bigred::write_file_utf8(path, f);
            };
            const std::vector<uint8_t> before = dash.pixels();
            for (int n = 0; n < 3; ++n) {
                bigred::DashValues dv;
                dv.by_source = {35000 * n / 2, 255 * n / 2, 2 * n, n, 0};
                dash.draw(dv);
                int changed = 0;
                for (size_t k = 0; k < before.size(); ++k) changed += dash.pixels()[k] != before[k];
                const std::string path = "dash_" + vehicle_name + "_" + std::to_string(n) + ".bmp";
                write_bmp(path);
                std::printf("      %s : %d texel(s) differents de la feuille d'origine -> %s\n",
                            vehicle_name.c_str(), changed, path.c_str());
            }
        }
    }

    // --- CAMERA -------------------------------------------------------------------------
    // The camera runs on the simulation tick, so it can be measured
    // here with no window at all - which is the only way to compare two framings honestly.
    if (bloc("CAMERA")) {
        std::printf("\nCAMERA  chase camera, preset %d of 3\n", 0);
        const bigred::CameraPreset& cp = bigred::kCameraPresets[0];
        std::printf("      aim %d over the car, distance %d..%d units, never under %d\n",
                    cp.aim_height, cp.dist_base / 4, (cp.dist_base + cp.dist_factor) / 4,
                    cp.dist_min / 4);

        bigred::ChaseCamera cam;

        // FOLLOW: accelerate from rest and read the distance back off at four speeds. The
        // distance is the thing the user sees first, and it is NOT a constant - it grows with
        // speed and saturates well before the vehicle does.
        std::printf("\n      after   speed   back   over car   over ground   pitch down\n");
        car.reset(tuning, ter, rules, grid);
        cam.reset(car, ter);
        settle(car, 40);
        cam.reset(car, ter);
        bigred::VehicleInput go;
        go.accel = true;
        const int marks[] = {1, 30, 90, 200, 400};
        int at = 0;
        for (int t = 0; t <= 400; ++t) {
            if (t > 0) { car.step(go); cam.step(car, ter); }
            bool report = false;
            for (int m : marks) if (m == t) report = true;
            if (!report) continue;
            ++at;
            const int32_t dx = car.world_x() - cam.now.x, dz = car.world_z() - cam.now.z;
            const int32_t ground = ter.height_fx(cam.now.x << 9, cam.now.z << 9) >> 9;
            std::printf("      %4d t  %6d  %5d   %8d   %11d   %8.1f deg\n", t, car.speed,
                        bigred::fx::hypot32(dx, dz), car.world_y() - cam.now.y, ground - cam.now.y,
                        deg16(static_cast<int16_t>(cam.now.pitch)));
        }

        // LAG: hold full lock and watch how far the camera falls behind the car's heading.
        // This IS the "camera lag" the user asked for, and there is no constant behind it: the
        // rope never moves sideways, so the whole of this number comes out of the car driving
        // away from where the camera happens to be standing.
        // A single average over a doughnut says nothing: the rope has no equilibrium against a
        // sustained turn, so the number is whatever the run length happened to be. What matters
        // is the SHAPE - how far behind the camera is after a corner of a realistic length, and
        // how fast the lag then unwinds once the car straightens up.
        std::printf("\n      held lock for   28ms ticks   lag when it ends   lag 1 s later   roulis maxi\n");
        const int corners[] = {8, 18, 36, 72, 144};
        for (int hold : corners) {
            car.reset(tuning, ter, rules, grid);
            cam.reset(car, ter);
            for (int t = 0; t < 120; ++t) { car.step(go); cam.step(car, ter); }
            bigred::VehicleInput in;
            in.accel = true;
            in.right = true;
            // The original's camera roll, NOT ported: 4 x the view's yaw change per 14 ms timer tick,
            // i.e. 2 x its change per 28 ms camera tick. Measured here to say whether it shows.
            int32_t roll_max = 0;
            for (int t = 0; t < hold; ++t) {
                const int32_t before = cam.now.yaw;
                car.step(in);
                cam.step(car, ter);
                const int32_t d = static_cast<int16_t>(static_cast<uint16_t>(cam.now.yaw - before));
                roll_max = std::max(roll_max, 2 * (d < 0 ? -d : d));
            }
            auto lag = [&]() {
                int32_t d = static_cast<int16_t>(
                    static_cast<uint16_t>(car.yaw_bam16() - cam.now.yaw));
                return deg16((d < 0) ? -d : d);
            };
            const double at_end = lag();
            for (int t = 0; t < 36; ++t) { car.step(go); cam.step(car, ter); }
            std::printf("      %10d ms %11d %15.1f deg %13.1f deg %11.1f deg\n",
                        hold * bigred::kPhysicsTickMs, hold, at_end, lag(), deg16(roll_max));
        }

        // OPENING: the mode-4 entry plants the camera 2500 units back and the closing limiter
        // walks it in at 37.5 units a tick. This is the pull-in a race starts with, and the
        // number below is how long it lasts - if it ever reads 1 tick, the limiter is broken.
        {
            car.reset(tuning, ter, rules, grid);
            settle(car, 40);
            cam.reset(car, ter);
            const int32_t start = bigred::fx::hypot32(car.world_x() - cam.now.x,
                                                      car.world_z() - cam.now.z);
            int ticks = 0;
            for (; ticks < 400; ++ticks) {
                cam.step(car, ter);
                car.step(bigred::VehicleInput());
                const int32_t d = bigred::fx::hypot32(car.world_x() - cam.now.x,
                                                      car.world_z() - cam.now.z);
                if (d <= cp.dist_min / 4 + 5) break;
            }
            std::printf("\n      opening pull-in: %d units -> %d, %d ticks (%d ms)\n", start,
                        cp.dist_min / 4, ticks, ticks * bigred::kPhysicsTickMs);
        }

        // TUNNEL: the car's tunnel flag drives three of the camera rules at once, so a level with
        // tunnels has to light it up and a level without them must not.
        {
            car.reset(tuning, ter, rules, grid);
            cam.reset(car, ter);
            int inside = 0;
            for (int t = 0; t < 600; ++t) {
                car.step(go);
                cam.step(car, ter);
                if (car.in_tunnel) ++inside;
            }
            std::printf("      tunnel flag set on %d of 600 ticks driving straight ahead\n",
                        inside);
        }
    }

    if (bloc("COMFORT_CAMERA")) {
        // Synthetic geometry: the executable's ordinary level load is shared with
        // the bench, but no authored collision placement determines these results.
        std::vector<uint8_t> flat(64*64,0);
        bigred::SimTerrain world;
        world.w=world.h=64;
        world.mph=flat.data(); world.mpc=flat.data();
        world.build_blocker_grid();
        using bigred::CameraPoint;
        const CameraPoint focus{2000,-200,2000}, eye{2000,-200,600};
        auto require=[](bool ok,const char* label) {
            if (!ok) std::fprintf(stderr,"COMFORT_CAMERA: %s\n",label);
            return ok;
        };
        if (!require(bigred::camera_arm_fraction(world,focus,eye)==1,"clear boom retracts")) return 1;
        const CameraPoint buried{2000,200,600};
        const double ground_fraction=bigred::camera_arm_fraction(world,focus,buried);
        if (!require(ground_fraction>0 && ground_fraction<1 &&
                     focus.y+(buried.y-focus.y)*ground_fraction+16<=0,
                     "ground collision leaves eye underground")) return 1;
        bigred::SimObject wall;
        wall.blocker=wall.hard=wall.use_box=true;
        wall.x=2000; wall.z=1300; wall.y=0; wall.radius=1000;
        wall.box_xmin=-500; wall.box_xmax=500;
        wall.box_zmin=-20; wall.box_zmax=20;
        wall.y_top=-1000; wall.y_bottom=1000;
        world.blockers.push_back(wall);
        world.build_blocker_grid();
        const double wall_fraction=bigred::camera_arm_fraction(world,focus,eye);
        if (!require(wall_fraction>.1 && wall_fraction<.5 &&
                     focus.z+(eye.z-focus.z)*wall_fraction>1336,
                     "boom passes through wall")) return 1;
        if (!require(bigred::camera_arm_fraction(world,{2000,-1200,2000},{2000,-1200,600})==1,
                     "camera above wall is falsely blocked")) return 1;
        // Below a floating obstacle: vehicle contacts' vertical gate alone lacks this test.
        world.blockers[0].y_top=-1000; world.blockers[0].y_bottom=-500;
        if (!require(bigred::camera_arm_fraction(world,focus,eye)==1,
                     "camera below obstacle is falsely blocked")) return 1;
        world.blockers[0].hard=false;
        world.blockers[0].y_bottom=1000;
        if (!require(bigred::camera_arm_fraction(world,focus,eye)==1,
                     "soft prop blocks camera")) return 1;
        std::puts("COMFORT_CAMERA: ground, hard wall, vertical bounds and soft props passed");
    }

    for (const std::string& name : g_blocs_wanted) {
        if (g_blocs_known.count(name) == 0) {
            std::fprintf(stderr, "--bloc: no block named %s\n", name.c_str());
            return 2;
        }
    }
    return 0;
}
