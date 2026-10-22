#include "race_screen.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>

#include "app/app.h"
#include "formats/shading.h"
#include "formats/tourn.h"
#include "formats/sprite.h"
#include "formats/terrain.h"
#include "formats/texture.h" // kTmWidth/kTmHeight, the atlas layer dimensions
#include "game/competition.h"
#include "game/level_catalog.h"
#include "game/results_screen.h"
#include "game/split_layout.h"
#include "net/net_game.h"
#include "scene/character_model.h"
#include "scene/camera_obstruction.h"
#include "sim/lockstep.h"
#include "sim/snapshot.h"
#include "ui/team_logos.h"
#include "render/shader.h"
#include "render/world_shaders.h"

namespace bigred {
namespace {

// --- TRAKSCN's "VIEW", the scripted helicopter pass -----------------------------------------
// World units per second along the checkpoint ring, and how high over the ground it flies. Both
// are choices, not measurements: the original's own fly-by is in the executable and has not
// been studied. 9 000 u/s is about what a fast vehicle does, and 2 600 units clears the tallest
// scenery of the shipped levels without losing the road.
constexpr double kFlyBySpeed = 9000.0;
constexpr int32_t kFlyByHeight = 2600;
// Pitched down a sixteenth of a turn - 22.5 degrees - so the track fills the lower half of the
// frame and the horizon stays in it. BAM16, and NEGATIVE is down: +Y is down in this frame.
constexpr int32_t kFlyByPitch = -4096;

// The cabin's own projection (WorldFrame::cabin), world units. The eye sits 10 to 40 units from
// the dashboard - inside the world's near plane of 10 - and the interior never reaches further
// than a vehicle is long.
constexpr float kCabinNearWorld = 1.0f;
constexpr float kCabinFarWorld = 1000.0f;


// The simulation rate, in seconds. NOT a placeholder and NOT tied to the display: the original's
// timer is a 14 ms multimedia timer, but the VEHICLE PHYSICS runs on every OTHER tick of it -
// 28 ms. See level_script.h's kPhysicsTickMs for the instruction-level evidence. Lockstep needs
// every client to step the same count over the same span.
constexpr double kSimStep = static_cast<double>(kPhysicsTickMs) / 1000.0;

// Each local player's mini-map dot, as an offset from game_spr::kDot0: GAMESVGA's six dots are
// the six team colours in their palette order - red, green, olive, blue, orange, violet. Orange
// (4) is kept for everyone else.
constexpr int kLocalDot[kMaxLocalPlayers] = {0, 3, 1, 5};


} // namespace

bool RaceScreen::enter(App& app) {
    mixer_ = &app.mixer();   // borrowed, not owned - see the header
    keys_ = app.profile().keys;
    start_music(app);
    if (!load_world(app)) return false;
    if (!build_gl()) return false;
    board_.build(app.game_dir(), *ingame_, scene_);

    map_span_ = static_cast<float>(scene_.map_w * kTerrainTexelWorldSize) * kRenderScale;
    for (LocalView& v : views_) {
        v.camera.reset(session_.car(v.racer), level_.terrain);
        v.cam_prev = v.camera.now;
        v.comfort_camera = ComfortCamera{};
        v.comfort_cut_pending = true;
        if (setup_.start_in_cabin) {
            v.camera.preset = 2;   // where `C` leaves it on the way into the cabin
            v.cabin = true;
        }
    }
    pose_prev_.assign(static_cast<size_t>(session_.racer_count()), VehiclePose());
    for (int i = 0; i < session_.racer_count(); ++i)
        pose_prev_[static_cast<size_t>(i)] = pose_of(session_.car(i));
    pose_cur_ = pose_prev_;

    std::printf("\n=== OpenBigRed ===\n");
    std::printf("Level %s, vehicle %s (%s).\n", setup_.level.c_str(), vehicle_name_.c_str(),
                tuning().is_aircraft() ? "aircraft"
                                      : (tuning().needs_drive_contact() ? "wheeled" : "sliding"));
    std::printf("Contact box x %d..%d, y %d..%d, z %d..%d - gravity %d/tick - %zu gear steps.\n",
                tuning().box.xmin, tuning().box.xmax, tuning().box.ytop, tuning().box.ybottom,
                tuning().box.zmin, tuning().box.zmax, tuning().gravity(), tuning().gear_f.size());
    std::printf("%zu checkpoint gate(s), %d lap(s), countdown %d ticks (%.1f s).\n",
                race().gates().size(), race().laps(), race().countdown_ticks_left(),
                race().countdown_ticks_left() * kGameTickMs / 1000.0);
    std::printf("ARROWS drive - CTRL boost - ALT back on track - C viewpoint - ESC quit.\n");
    std::printf("Simulation: %d ms fixed tick (one timer tick in two), integer only.\n\n",
                kPhysicsTickMs);

    if (setup_.fly_by) {
        if (!fly_heli_) build_flyby_path();
        return true;   // nothing is about to start, so no starter's sound
    }
    // The original fires it once, from the race set-up itself,
    // before the countdown starts winding down - and only with COMMENTARY on.
    audio_.on_race_start();
    // Everything is built: the host starts the race once every machine has said so. ⚠️ NOT
    // SAID FROM HERE: see draw(), which says it once the race has really been drawn.
    if (net_) net_->take_start();
    return true;
}

// The rail. The checkpoint ring already describes the track as an ordered chain of slabs lying
// across it (sim/race.h), so the middle of each slab, in order, IS the racing line at the
// resolution a camera needs - 14 to 69 points a lap. Nothing new has to be decoded for this.
void RaceScreen::build_flyby_path() {
    fly_path_.clear();
    fly_distance_ = 0.0;
    fly_total_ = 0.0;
    const std::vector<Gate>& gates = race().gates();
    if (gates.empty()) return;
    fly_path_.reserve(gates.size() + 1);
    for (const Gate& g : gates) {
        int32_t x = 0, z = 0;
        gate_point(g, g.length / 2, 0, x, z);
        FlyPoint p;
        p.x = x;
        p.z = z;
        // ⚠️ +Y IS DOWN here, so "above the ground" is a SMALLER number. The height is the
        // terrain's, raised by the helicopter's own altitude.
        p.y = (level_.terrain.valid() ? (level_.terrain.height_fx(x << 9, z << 9) >> 9) : 0) -
              kFlyByHeight;
        fly_path_.push_back(p);
    }
    fly_path_.push_back(fly_path_.front());   // closed, so the lap comes back round
    for (size_t i = 1; i < fly_path_.size(); ++i) {
        const double dx = fly_path_[i].x - fly_path_[i - 1].x;
        const double dz = fly_path_[i].z - fly_path_[i - 1].z;
        fly_total_ += std::sqrt(dx * dx + dz * dz);
    }
}

// One frame of the fly-by. It writes straight into the CameraState the ordinary path reads, so
// `camera_view` - and with it the whole world transform, the mirror in X included - stays the
// same code the race uses, and the two can never disagree about where the track is.
void RaceScreen::step_flyby(double dt_ms) {
    if (fly_path_.size() < 2 || fly_total_ <= 0.0) return;
    fly_distance_ += kFlyBySpeed * dt_ms / 1000.0;
    while (fly_distance_ >= fly_total_) fly_distance_ -= fly_total_;

    // Walk the chain to the segment the distance falls in. A linear scan over at most seventy
    // points, once a frame, is not worth an index.
    double travelled = 0.0, seg_len = 0.0;
    size_t seg = 0;
    for (; seg + 2 < fly_path_.size(); ++seg) {
        const double dx = fly_path_[seg + 1].x - fly_path_[seg].x;
        const double dz = fly_path_[seg + 1].z - fly_path_[seg].z;
        seg_len = std::sqrt(dx * dx + dz * dz);
        if (travelled + seg_len > fly_distance_) break;
        travelled += seg_len;
    }
    if (seg + 1 >= fly_path_.size()) return;
    const double t = seg_len > 0.0 ? (fly_distance_ - travelled) / seg_len : 0.0;
    const FlyPoint& a = fly_path_[seg];
    const FlyPoint& b = fly_path_[seg + 1];

    views_[0].camera.now.x = static_cast<int32_t>(a.x + (b.x - a.x) * t);
    views_[0].camera.now.y = static_cast<int32_t>(a.y + (b.y - a.y) * t);
    views_[0].camera.now.z = static_cast<int32_t>(a.z + (b.z - a.z) * t);
    // Facing the way it flies, in the game's own convention: x = sin(yaw), z = cos(yaw).
    const double dx = b.x - a.x, dz = b.z - a.z;
    if (dx != 0.0 || dz != 0.0) {
        const double turns = std::atan2(dx, dz) * 65536.0 / 6.283185307179586;
        const int32_t want = static_cast<int32_t>(std::lround(turns)) & 0xFFFF;
        // ⚠️ EASED, NOT SNAPPED: the rail is a chain of straight segments, and a camera that
        // took each segment's bearing outright would flick at every gate. This is a first-order
        // filter towards the segment's heading, which is what makes it read as a helicopter.
        const int32_t d = static_cast<int16_t>(static_cast<uint16_t>(want - views_[0].camera.now.yaw));
        views_[0].camera.now.yaw = (views_[0].camera.now.yaw + static_cast<int32_t>(d * dt_ms / 260.0)) & 0xFFFF;
    }
    views_[0].camera.now.pitch = kFlyByPitch;
    // It IS the frame rather than a tick, so there is nothing to interpolate between: both
    // states are this one.
    views_[0].cam_prev = views_[0].camera.now;
    alpha_ = 0.0f;
}

bool RaceScreen::load_world(App& app) {
    const std::string& game_dir = app.game_dir();

    // A NETWORK race takes everything the simulation reads from the START the host sent -
    // every machine must build the same race - and only what is drawn from this one.
    const NetRaceStart* start = nullptr;
    if (setup_.network) {
        net_ = &app.net();
        start = &net_->race();
        const int me = net_->local_racer();
        if (me < 0 || static_cast<size_t>(me) >= start->racers.size()) return false;
        setup_.level = start->settings.level;
        setup_.vehicle = start->racers[static_cast<size_t>(me)].vehicle;
        setup_.damage_option = start->racers[static_cast<size_t>(me)].look.damage;
        setup_.opponents = start->settings.ghosts;
        setup_.fly_by = false;
    }

    scene_.load_shade_table(game_dir);
    {
        // From wherever the circuit comes - a shipped .LEV or an added one (game/level_catalog.h).
        int w = 0, h = 0;
        level_map_size(game_dir, setup_.level, w, h);
        if (!scene_.load_files(setup_.level, *level_files(game_dir, setup_.level), w, h,
                               game_path(game_dir, setup_.level + ".LEV")))
            return false;
    }

    // The level as the simulation sees it: the terrain and its objects, the checkpoint ring,
    // the grid, the laps, the countdown and the ghosts' rates - built by the same function the
    // benches use (scene/race_level.h). Everything the handling needs comes out of the data;
    // nothing here is tuned by ear.
    build_race_level(scene_, setup_.level, level_, game_dir);
    // The world's destruction state - shared, and part of the simulation rather than of the
    // renderer, because a prop knocked over has to be down for every player at once.
    world_.resize(scene_.instances.size());
    if (!level_.terrain.objects.empty() || !level_.terrain.blockers.empty())
        std::printf("%zu drivable object(s), %zu blocking one(s).\n",
                    level_.terrain.objects.size(), level_.terrain.blockers.size());

    // THE FIELD. The humans at their grid blocks and, with opponents, the level's recorded
    // ghost `N.REC` on every other block N (sim/replay.h). Alone, the player takes the block
    // the level's `spos` line names, else a DRAWN one (`rand() % 6`, as the original), and the
    // ghost recorded for that block is not raced; with no ghost at all the
    // player keeps the first block (user, 2026-09-23). The draw happens HERE, before the
    // session exists, so it is part of the race's setup and never of the simulation.
    std::vector<FieldHuman> humans;
    if (start) {
        for (const NetRacer& r : start->racers)
            humans.push_back(FieldHuman{r.grid, r.vehicle, r.look.damage});
    } else {
        int me = setup_.grid_place;
        if (me < 0 || me >= kGridRacers) {
            const StartPlaces spos = parse_start_places(level_.level_txt);
            if (spos.present) {
                me = spos.single;
            } else {
                std::random_device rd;
                me = static_cast<int>(rd() % kGridRacers);
            }
        }
        me = ((me % kGridRacers) + kGridRacers) % kGridRacers;
        // The fly-by: the level's own `6.REC` names the helicopter (its header's vehicle
        // index into the `car` list), and that is what the "player" drives.
        std::string vehicle = setup_.vehicle;
        fly_heli_ = false;
        if (setup_.fly_by) {
            auto rec = scene_.lev.find("6.REC");
            if (rec != scene_.lev.end()) {
                fly_track_ = parse_replay(rec->second);
                if (fly_track_.valid() && fly_track_.vehicle_index >= 0 &&
                    static_cast<size_t>(fly_track_.vehicle_index) < level_.cars.size()) {
                    vehicle = level_.cars[static_cast<size_t>(fly_track_.vehicle_index)];
                    fly_heli_ = true;
                    me = 0;
                }
            }
        }
        humans.push_back(FieldHuman{me, vehicle, setup_.damage_option});
        // The split screen's other players, on the blocks after the first player's: the
        // ghosts recorded for those blocks stay out, as for any human.
        for (size_t g = 0; g < setup_.guests.size() && g + 1 < static_cast<size_t>(kMaxLocalPlayers); ++g)
            humans.push_back(FieldHuman{(me + 1 + static_cast<int>(g)) % kGridRacers,
                                        setup_.guests[g].vehicle, setup_.guests[g].damage_option});
    }
    const bool ghosts = setup_.opponents && !setup_.fly_by;
    if (!build_race_field(game_dir, scene_, level_, humans, ghosts, field_)) return false;
    player_ = field_.session_of[static_cast<size_t>(start ? net_->local_racer() : 0)];
    if (fly_heli_) {
        field_.racers[static_cast<size_t>(player_)].ghost = &fly_track_;
        const HeliPoint hp = parse_heli(level_.level_txt);
        if (hp.present) {
            fly_start_ = RespawnPoint{hp.x, hp.z, hp.yaw_bam16};
            field_.racers[static_cast<size_t>(player_)].start = &fly_start_;
        }
        std::printf("Fly-by: %s played back from 6.REC, from %s.\n",
                    field_.vehicle[static_cast<size_t>(player_)].c_str(),
                    hp.present ? "the level's heli point" : "the grid");
    }
    views_.assign(1, LocalView());
    views_[0].racer = player_;
    views_[0].controls = setup_.controls.empty() ? Controls::Everything : setup_.controls[0];
    if (!start) {
        for (size_t h = 1; h < field_.session_of.size(); ++h) {
            LocalView v;
            v.racer = field_.session_of[h];
            v.controls = h < setup_.controls.size() ? setup_.controls[h] : Controls::Everything;
            views_.push_back(std::move(v));
        }
    }
    vehicle_name_ = field_.vehicle[static_cast<size_t>(player_)];
    if (field_.ghosts > 0 || start)
        std::printf("%zu racer(s) on the grid (%d ghost(s)), the player on block %d.\n",
                    humans.size() + static_cast<size_t>(field_.ghosts), field_.ghosts, player_ + 1);
    std::fflush(stdout);   // a killed process loses a full buffer

    // The session puts every car ON its grid block (VehicleSim::place_on_grid), hands it the
    // shared destruction state, and links the field so the cars collide with one another.
    const int laps = (start && start->settings.laps) ? start->settings.laps : level_.laps;
    // The rules: the host's in a network race, this machine's otherwise (sim/race_rules.h).
    session_.rules = start ? start->rules : setup_.rules;
    {
        const RaceRules& r = session_.rules;
        const RaceRules d = start ? RaceRules::network() : RaceRules();
        if (r.air_damp_shift != d.air_damp_shift) {
            if (r.air_damp_shift == 0) std::printf("Airborne damping OFF (-AIRDAMP=0).\n");
            else std::printf("Airborne damping: rates lose 1/%d a tick (-AIRDAMP).\n", 1 << r.air_damp_shift);
        }
        if (r.air_rate_cap_value != d.air_rate_cap_value)
            std::printf("Deviation 3: airborne cap at %d (-AIRCAP).\n", r.air_rate_cap_value);
        if (r.righting_tolerance)
            std::printf("Retired deviation 2 ON: righting torque on the tolerance mask (-DEV2).\n");
        if (!r.air_rate_cap)
            std::printf("Deviation 3 OFF: body rates not capped in the air (-NOAIRCAP).\n");
    }
    session_.configure(level_.terrain, &world_, level_.gates, laps, level_.countdown_ticks,
                       level_.lines, level_.grid, level_.nitros, field_.racers, level_.origin,
                       level_.rates);
    // No elastic on a fly-by: it paces the ghosts against a human, and there is none.
    if (fly_heli_) session_.rules.rubber_band = false;
    inputs_.assign(static_cast<size_t>(session_.racer_count()), VehicleInput());
    start_ = session_.start(player_);
    if (setup_.start_damage >= 0 && !net_) {
        VehicleSim& me = session_.car(player_);
        const int32_t cap = me.damage_capacity > 0 ? me.damage_capacity : 1;
        if (setup_.start_damage >= 100) {
            me.damage = -1;
            me.damage_percent = 100;
        } else {
            me.damage = cap * setup_.start_damage / 100;
            me.damage_percent = setup_.start_damage;
        }
        std::printf("Starting %d%% damaged (-DAMAGE).\n", me.damage_percent);
    }
    spectator_.load(level_.level_txt, level_.terrain);
    sun_direction(parse_light(level_.level_txt), sun_);
    if (start) net_race_.begin(*net_, start->delay, start->racers.size(), setup_.autodrive);
    // The series' points as they stand before this race (push_results adds this race's): read
    // now, because the host's record of it may reach this machine before or after the results.
    if (start) {
        for (const NetPlayer& p : net_->lobby().players) net_points_before_[p.id] = p.points;
        net_series_done_before_ = net_->lobby().series_done;
    }
    if (setup_.trace) {
        std::printf("[race] %s : %zu portes, %d tour(s), %zu ligne(s) CHECKP :",
                    setup_.level.c_str(), race().gates().size(), race().laps(),
                    race().check_lines().size());
        for (const CheckLine& c : race().check_lines())
            std::printf(" %d%s(porte %d%s)", c.ordinal(), c.last() ? "*" : "", c.gate,
                        c.in_gate ? "" : "~");
        std::printf("\n[race] `map_preview - %s -DEBUG` dessine l'anneau et les lignes.\n",
                    setup_.level.c_str());
    }

    // The effect shapes (NIT_FX, FLAMES, SMOKE, BOOM) are shared by every vehicle and live in
    // INGAME.WAP, never in a `.VHC`. Missing it is not fatal - you just get no flame.
    ingame_ = game_archive_or_empty(game_dir, "INGAME.WAP");

    // The GROUND EFFECTS. Their shapes, their sheets and the `spot` lines that declare them all
    // live in the level's ENVIRONMENT archive - DIRT1 reads DIRT.ENV - which is why every dirt
    // track kicks up the same dust and only the water tracks splash. See sim/particles.h.
    {
        const std::string env_name = env_name_for_level(setup_.level);
        env_ = game_archive_or_empty(game_dir, env_name + ".ENV");
        if (const Blob* txt = find_member(*env_, env_name + ".TXT"))
            spots_ = parse_spot_effects(std::string(txt->begin(), txt->end()));
        particles_.configure(spots_);
    }

    // ⚠️ Before the atlas is uploaded: these APPEND their sprite sheets to it.
    // The player's colour and emblem, straight from the setup the menus filled in. Both change
    // what is DRAWN and nothing the simulation reads, so neither can desynchronise a race.
    VehicleLogo logo;
    logo.rgba = setup_.logo.empty() ? nullptr : setup_.logo.data();
    logo.w = logo.h = kLogoCell;
    // The drivers' heads, out of CHARSCN.SCN, read once for the whole field. A character's
    // colour is its own ramp, as independent of the vehicle's as the original's data made it.
    const std::vector<ColorPreset>& char_presets = color_presets();
    struct Driver {
        CharacterShapes shapes;
        VehicleTint tint;
        VehicleDriver ref;
    };
    const auto driver_for = [&](int character, int colour, bool tinted, Driver& d) -> const VehicleDriver* {
        if (character < 0 || !load_character_shapes(game_dir, character, d.shapes))
            return nullptr;
        d.ref.head = &d.shapes.head;
        d.ref.mouth_tm = &d.shapes.mouth_tm;
        if (tinted && !char_presets.empty()) {
            const ColorPreset& p = char_presets[static_cast<size_t>(colour) % char_presets.size()];
            d.tint = VehicleTint{p.r, p.g, p.b};
            d.ref.tint = &d.tint;
        }
        return &d.ref;
    };
    // How each racer is announced when wrecked: the named colour nearest its paint, or, for
    // an unpainted car, its place in the field - which is all the original goes by.
    wreck_colour_.assign(static_cast<size_t>(session_.racer_count()), 0);
    for (int i = 0; i < session_.racer_count(); ++i) wreck_colour_[static_cast<size_t>(i)] = i % 6;
    const auto colour_word = [](const VehicleTint& t) {
        static constexpr uint8_t kNamed[6][3] = {{200, 30, 30},  {30, 160, 40},  {230, 210, 40},
                                                  {40, 70, 200},  {240, 130, 20}, {140, 40, 170}};
        int best = 0;
        long best_d = -1;
        for (int c = 0; c < 6; ++c) {
            const long dr = static_cast<long>(t.r) - kNamed[c][0];
            const long dg = static_cast<long>(t.g) - kNamed[c][1];
            const long db = static_cast<long>(t.b) - kNamed[c][2];
            const long d = dr * dr + dg * dg + db * db;
            if (best_d < 0 || d < best_d) { best_d = d; best = c; }
        }
        return best;
    };
    if (setup_.tinted) wreck_colour_[static_cast<size_t>(player_)] = colour_word(setup_.tint);
    Driver me;
    car_model_ = build_vehicle_model(*field_.vhc[static_cast<size_t>(player_)], *ingame_, tuning(),
                                     1, scene_, setup_.tinted ? &setup_.tint : nullptr,
                                     logo.valid() ? &logo : nullptr,
                                     driver_for(setup_.character, setup_.character_colour, true, me));
    // Every other racer's body. A ghost in its recording's team repaint (header byte 31,
    // sim/replay.h); another human in the colour and emblem they announced in the lobby.
    // Before the atlas upload, like the player's.
    TeamLogos team_logos;
    const bool have_logos = start && team_logos.load(game_dir);
    ghost_models_.assign(static_cast<size_t>(session_.racer_count()), VehicleModel());
    for (int i = 0; i < session_.racer_count(); ++i) {
        const size_t si = static_cast<size_t>(i);
        if (i == player_ || !field_.racers[si].tuning) continue;
        const int h = field_.human[si];
        if (h >= 0 && start) {
            const NetLook& look = start->racers[static_cast<size_t>(h)].look;
            const std::vector<ColorPreset>& presets = color_presets();
            VehicleTint tint{};
            if (!presets.empty()) {
                const ColorPreset& p = presets[look.colour % presets.size()];
                tint = VehicleTint{p.r, p.g, p.b};
            }
            std::vector<uint8_t> emblem;
            if (have_logos) {
                int lw = 0, lh = 0;
                // Their own logo is painted from the bytes they sent, never from ours: index
                // 13 is "the player's own" on every machine, and ours is not theirs.
                if (look.logo >= kTeamLogoCount) {
                    if (decode_logo_png(look.custom_logo, emblem)) lw = lh = kCustomLogoSide;
                } else {
                    emblem = team_logos.logo_rgba(look.logo, lw, lh);
                }
                if (lw != kLogoCell || lh != kLogoCell) emblem.clear();
            }
            VehicleLogo theirs;
            theirs.rgba = emblem.empty() ? nullptr : emblem.data();
            theirs.w = theirs.h = kLogoCell;
            if (!presets.empty()) wreck_colour_[si] = colour_word(tint);
            Driver them;
            ghost_models_[si] = build_vehicle_model(
                *field_.vhc[si], *ingame_, field_.tunings[si], 1, scene_,
                presets.empty() ? nullptr : &tint, theirs.valid() ? &theirs : nullptr,
                driver_for(look.character, look.character_colour, true, them));
        } else if (h > 0 && !start && static_cast<size_t>(h - 1) < setup_.guests.size()) {
            // A split-screen guest: the colour, emblem and driver of their seat.
            const Setup::Guest& gs = setup_.guests[static_cast<size_t>(h - 1)];
            if (gs.tinted) wreck_colour_[si] = colour_word(gs.tint);
            VehicleLogo theirs;
            theirs.rgba = gs.logo.empty() ? nullptr : gs.logo.data();
            theirs.w = theirs.h = kLogoCell;
            Driver them;
            ghost_models_[si] = build_vehicle_model(
                *field_.vhc[si], *ingame_, field_.tunings[si], 1, scene_,
                gs.tinted ? &gs.tint : nullptr, theirs.valid() ? &theirs : nullptr,
                driver_for(gs.character, gs.character_colour, true, them));
        } else {
            // A recorded opponent: its head is the recording's person byte. Keep the head's
            // authored colours; the team/body colour is carried separately through field_.colour.
            const int character = field_.tracks[si].valid()
                                      ? field_.tracks[si].character
                                      : i % kCharacterCount;
            Driver them;
            ghost_models_[si] = build_vehicle_model(
                *field_.vhc[si], *ingame_, field_.tunings[si], field_.colour[si] + 1, scene_,
                nullptr, nullptr, driver_for(character, 0, false, them));
        }
    }
    // THE CABINS, one per local player, in their car's colour: the interior (`vehicle IN_...`,
    // section 1) and the dashboard its faces show. Before the atlas upload, like every model.
    cabins_.assign(static_cast<size_t>(session_.racer_count()), Cabin());
    for (const LocalView& lv : views_) {
        const size_t si = static_cast<size_t>(lv.racer);
        if (si >= cabins_.size() || !field_.racers[si].tuning) continue;
        const VehicleTint* tint = nullptr;
        if (lv.racer == player_) {
            tint = setup_.tinted ? &setup_.tint : nullptr;
        } else if (const int h = field_.human[si];
                   h > 0 && static_cast<size_t>(h - 1) < setup_.guests.size()) {
            const Setup::Guest& gs = setup_.guests[static_cast<size_t>(h - 1)];
            tint = gs.tinted ? &gs.tint : nullptr;
        }
        Cabin& c = cabins_[si];
        c.model = build_vehicle_model(*field_.vhc[si], *ingame_, field_.tunings[si], 1, scene_,
                                      tint, nullptr, nullptr, /*section=*/1);
        if (c.model.valid() && c.dash.load(*field_.vhc[si], field_.tunings[si])) {
            const auto it = c.model.sheet_layers.find(c.dash.sheet_name());
            c.layer = it == c.model.sheet_layers.end() ? -1 : it->second;
        }
    }
    particle_visuals_ = build_particle_visuals(spots_, *env_, scene_);

    // THE SOUND, once the field is built and every car has its colour (race_audio.h).
    {
        RaceAudio::Config ac;
        ac.game_dir = game_dir;
        ac.level = setup_.level;
        ac.level_txt = level_.level_txt;
        ac.field = &field_;
        ac.session = &session_;
        ac.player = player_;
        ac.seats.assign(static_cast<size_t>(session_.racer_count()), -1);
        for (int i = 0; i < session_.racer_count(); ++i) ac.seats[static_cast<size_t>(i)] = local_seat_of(i);
        ac.wreck_colours = wreck_colour_;
        ac.commentary = app.profile().commentary;
        ac.alone = !net_ && !split();
        ac.trace = setup_.trace;
        audio_.configure(mixer_, ac);
    }
    return true;
}

bool RaceScreen::build_gl() {
    // The whole world, uploaded once: terrain, decor, animation steps, destruction states, start
    // lights and the atlas - by the renderer map_preview uses too (scene/world_renderer.h).
    if (!renderer_.create()) return false;
    renderer_.upload(scene_);
    return true;
}

void RaceScreen::exit(App&) {
    audio_.stop();
    // ⚠️ The two figures must agree to within a tick or two. They are printed together rather
    // than just the count because the count alone cannot show a drift: extracting this screen
    // out of main() changed WHERE the frame delta comes from, and "does the simulation still
    // run at 28 ms" is exactly the regression that would otherwise be invisible.
    std::printf("%llu simulation ticks in %.1f s (expected %.0f at %d ms).\n", sim_ticks_,
                wall_ms_ / 1000.0, wall_ms_ / kPhysicsTickMs, kPhysicsTickMs);

    renderer_.release();
}

bool RaceScreen::handle_event(App& app, const SDL_Event& e) {
    // ⚠️ A RACE WITH OTHER PEOPLE IN IT ASKS BEFORE IT IS LEFT (user, 2026-09-26): one
    // Escape pressed by mistake threw a whole network game, or everyone's split screen, away.
    // Escape (or the pad's START) asks; Enter / the bottom pad button confirms; Escape again /
    // the right pad button carries on. A network race does not stop while the question is up -
    // there is no pause in lockstep - but a split screen does.
    const bool ask = (net_ && net_->active()) || split();
    const bool esc = e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE;
    const bool pad_start =
        e.type == SDL_CONTROLLERBUTTONDOWN && e.cbutton.button == SDL_CONTROLLER_BUTTON_START;
    if (confirm_quit_) {
        const bool yes = (e.type == SDL_KEYDOWN && !e.key.repeat &&
                          (e.key.keysym.sym == SDLK_RETURN || e.key.keysym.sym == SDLK_KP_ENTER)) ||
                         (e.type == SDL_CONTROLLERBUTTONDOWN &&
                          e.cbutton.button == SDL_CONTROLLER_BUTTON_A);
        const bool no = esc || pad_start ||
                        (e.type == SDL_CONTROLLERBUTTONDOWN &&
                         e.cbutton.button == SDL_CONTROLLER_BUTTON_B);
        if (yes) {
            confirm_quit_ = false;
            leave_race(app);
            return true;
        }
        if (no) {
            confirm_quit_ = false;
            return true;
        }
        // Everything else still drives: a network race goes on under the question.
        if (e.type == SDL_KEYDOWN || e.type == SDL_CONTROLLERBUTTONDOWN) return !net_;
    }
    if (esc || (pad_start && ask)) {
        if (ask && !(esc && e.key.repeat)) confirm_quit_ = true;
        else if (!ask) leave_race(app);
        return true;
    }
    // The original's own three viewpoints, cycled by whoever pressed: on a split
    // screen, `C` is the left half of the keyboard's, `End` the arrows', and each pad its own.
    // ⚠️ THE VIEW IS AN EVENT, NOT A POLL, unlike the six driving actions: it is a one-shot, and
    // a poll would cycle the camera once per frame for as long as the button was held. `X` is
    // SDL's name for the LEFT face button - positions, not letters (app/app.h).
    const auto cycle_for = [&](const auto& matches) {
        bool any = false;
        for (LocalView& v : views_)
            if (matches(v.controls)) { cycle_view(v); any = true; }
        return any;
    };
    if (e.type == SDL_KEYDOWN && !e.key.repeat) {
        const SDL_Keycode k = e.key.keysym.sym;
        if (k == SDLK_c)
            return cycle_for([](Controls c) {
                return c == Controls::Everything || c == Controls::KeyboardLeft;
            });
        if (k == SDLK_END)
            return cycle_for([](Controls c) { return c == Controls::KeyboardRight; });
    }
    if (e.type == SDL_CONTROLLERBUTTONDOWN && e.cbutton.button == SDL_CONTROLLER_BUTTON_X) {
        const int pad = app.pad_index_of(e.cbutton.which);
        return cycle_for([pad](Controls c) {
            return c == Controls::Everything || (pad >= 0 && controls_pad(c) == pad);
        });
    }
    return false;
}

// Pops rather than quits. With nothing under it the App ends, which is what it did before; once
// there is a menu under it, this is the way back to it.
// ⚠️ IN A NETWORK RACE, LEAVING THE RACE LEAVES THE GAME. There is no pause in lockstep: the
// others cannot wait for a player who is in a menu. The host fills in this racer's inputs from
// here on, and the lobby underneath sees the game is gone.
void RaceScreen::leave_race(App& app) {
    if (net_) net_->leave();
    app.pop();
}

// The circuit's own music, from its line of `TOURN.TXT`: the intro once, then the music for as
// long as the race lasts. See formats/tourn.h.
//
// ⚠️ IT IS ASKED FOR BEFORE THE LEVEL LOADS, on purpose. Loading blocks this thread for seconds
// and the decoder is on its own; asking first means the new piece is already playing while the
// world is still being built, instead of the menu's carrying on under a loading screen.
void RaceScreen::start_music(App& app) {
    if (!app.music().ready()) return;
    // A fly-by changes nothing: the presentation screen it was launched from is still playing
    // the circuit's intro, and that is what the player should keep hearing over it.
    if (setup_.fly_by) return;
    // No catalogue, or no track for this circuit: whatever is playing keeps playing.
    const TournCatalog tourn = load_tourn_catalog(app.game_dir());
    const TournTrack* t = tourn.find(setup_.level);
    if (!t || t->music_track <= 0) return;
    // ⚠️ COMING FROM THE PRESENTATION SCREEN, THE INTRO IS ALREADY PLAYING AND MUST BE CUT.
    // That screen is where the user wants to hear it (2026-09-23), and RACE interrupts it there
    // and then - so the race asks for the looping piece outright rather than starting the
    // jingle a second time.
    if (setup_.skip_intro) {
        app.music().play_cd_track(t->music_track);
        return;
    }
    // ⚠️ THE INTRO IS PLAYED ONLY IF THE FOLDER REALLY HAS ITS FILE, and is never drawn
    // for (user, 2026-09-22). On the original disc the intros are short jingles among 38
    // tracks; a folder holding just the music has no file for them, and drawing a song in
    // place of a jingle would push the circuit's own music minutes into the race.
    const bool have_intro = t->intro_track > 0 && app.music().has_cd_track(t->intro_track);
    app.music().play_cd_track(have_intro ? t->intro_track : t->music_track, t->music_track);
}


void RaceScreen::note_wrecks() {
    if (!setup_.trace) return;
    for (int i = 0; i < session_.racer_count(); ++i)
        if (session_.wrecked_for(i) == 1) {
            std::printf("[race] racer %d wrecked at tick %u\n", i, session_.ticks());
            std::fflush(stdout);
        }
}

// The race's own rule, the session's (sim/race_session.h): a wreck is out once
// `rules.wreck_out_ticks` have passed, which is shorter in a network race.
bool RaceScreen::out_of_race(int racer) const { return session_.out_of_race(racer); }

void RaceScreen::update(App& app, double dt_ms) {
    // Fixed-timestep simulation: the vehicle and the camera, and everything else that will ever
    // move, run an exact whole number of times per second of wall clock, identically on every
    // machine. The DISPLAY then interpolates between the last two ticks, which is what makes
    // 30 fps and 240 fps show the same motion rather than the same code at two speeds. (The
    // optional comfort rig is presentation only: it follows the drawn poses on the render clock.)
    wall_ms_ += dt_ms;

    // ⚠️ A FLY-BY RUNS NO SIMULATION AT ALL. There is no vehicle to step, no input to read
    // and no result to push - only the camera moves, and it moves on the wall clock like every
    // other presentation thing. Taking this branch before the
    // accumulator also means the deterministic tick count is not polluted by a screen nobody
    // is playing.
    // The end of the fly-by: once the helicopter's recording has run out, a fade to black and
    // back to the screen the fly-by was launched from.
    if (fly_heli_) {
        if (fly_fade_from_ms_ < 0.0 && session_.ghost_finished(player_)) fly_fade_from_ms_ = wall_ms_;
        if (fly_fade_from_ms_ >= 0.0) {
            fade_ = static_cast<float>(std::min(1.0, (wall_ms_ - fly_fade_from_ms_) / kFadeMs));
            if (fade_ >= 1.0f) {
                leave_race(app);
                return;
            }
        }
    }
    if (setup_.fly_by && !fly_heli_) {
        step_flyby(dt_ms);
        views_[0].place(views_[0].cam_prev, views_[0].camera.now, 0.0f, scene_.map_w);
        return;
    }

    accumulator_ += dt_ms / 1000.0;
    if (accumulator_ > 0.25) accumulator_ = 0.25;

    // The six logical actions, and nothing else, is the whole control surface of a vehicle.
    // Read once per frame, applied to every tick of this frame.
    //
    // ⚠️ KEYBOARD AND PAD ARE OR'ED, NOT CHOSEN BETWEEN. Both are live at once and neither
    // takes the other over: there is no key-binding screen yet, so a player must be able to
    // pick up whichever is to hand mid-race. The day bindings are configurable,
    // this is where the profile's table replaces the constants.
    //
    // The pad, as the user asked for it: right trigger accelerates, left trigger brakes and
    // reverses, the BOTTOM face button is the nitro, the TOP one puts the car back on the
    // track. Steering is the left stick or the d-pad - without it the pad could not drive at
    // all. The LEFT face button cycles the view and is an event, not a poll: see
    // handle_event(). All of them are POSITIONS on the pad, never letters (app/app.h).
    for (LocalView& v : views_) v.input = read_controls(app, v.controls);

    if (setup_.autodrive) {
        // A pure function of (racer, tick): the throttle held, the steering in bursts.
        uint32_t s = 2654435761u * static_cast<uint32_t>(player_ + 1) +
                     (session_.ticks() / 20) * 40503u;
        s ^= s >> 13;
        s *= 2246822519u;
        s ^= s >> 16;
        VehicleInput& in = views_[0].input;
        in = VehicleInput();
        in.accel = true;
        in.left = (s % 3) == 0;
        in.right = (s % 3) == 1;
        in.nitro = (s % 17) == 0;
    }
    if (net_) {
        const uint32_t stop_at =
            setup_.quit_after && setup_.end_after ? std::min(setup_.quit_after, setup_.end_after)
                                                  : std::max(setup_.quit_after, setup_.end_after);
        NetRaceDriver::Ticker t;
        t.run = [this](uint32_t tick, const std::vector<VehicleInput>& per) {
            for (size_t h = 0; h < per.size(); ++h)
                inputs_[static_cast<size_t>(field_.session_of[h])] = per[h];
            step_simulation();
            if (setup_.desync_at && tick == setup_.desync_at) {
                session_.car(player_).px += 200 << 9;
                pose_cur_[static_cast<size_t>(player_)] = pose_of(car());
                std::printf("[autotest] tick %u: own car nudged 200 units (-DESYNCAT)\n", tick);
                std::fflush(stdout);
            }
        };
        t.resynced = [this] {
            for (int i = 0; i < session_.racer_count(); ++i)
                pose_prev_[static_cast<size_t>(i)] = pose_cur_[static_cast<size_t>(i)] =
                    pose_of(session_.car(i));
        };
        net_race_.frame(session_, accumulator_, kSimStep, dt_ms, views_[0].input, stop_at, t);
    } else if (confirm_quit_) {
        // A split screen holds still while it asks whether to stop.
        accumulator_ = 0.0;
    } else {
        while (accumulator_ >= kSimStep &&
               (!setup_.quit_after || session_.ticks() < setup_.quit_after)) {
            accumulator_ -= kSimStep;
            for (const LocalView& v : views_) inputs_[static_cast<size_t>(v.racer)] = v.input;
            step_simulation();
        }
    }
    // Pushed, not swapped: the results sit ON TOP of the frozen race, the same shape as the
    // presentation screen pushing THIS one. Alone, that is the moment the player crosses the
    // line on the last lap. ⚠️ IN A NETWORK RACE, NOT BEFORE EVERY HUMAN HAS FINISHED: the
    // race must keep ticking - and this machine keep sending its inputs - for as long as
    // anyone is still driving, or the others would wait for it for ever. Every machine sees
    // the last human cross on the same tick, so they all leave together.
    if (setup_.quit_after && session_.ticks() >= setup_.quit_after) {
        // Two seconds more before leaving, still servicing the network: the last inputs and
        // checksums have to reach the others, or a test would stop them one tick short.
        if (quit_reached_ms_ < 0.0) quit_reached_ms_ = wall_ms_;
        if (wall_ms_ - quit_reached_ms_ < 2000.0) return;
        std::printf("[autotest] tick %u checksum %08x desync %u\n", session_.ticks(),
                    session_.checksum(), net_ ? net_->desync_tick() : 0u);
        std::fflush(stdout);
        if (net_) net_->leave();
        app.quit();
        return;
    }
    // A fly-by never ends in a result: Esc takes the player back to the track's screen.
    // ⚠️ "Home" is `out_of_race`, not `finished`: a wreck never crosses the line. Alone, the
    // race ends 4.9 s after the player's car died, as the original has it; with others, a wreck
    // simply stops being waited for.
    const bool over = !setup_.fly_by && (net_ ? (race().race_over() || humans_finished() ||
                               (session_.finish_deadline() && session_.ticks() >= session_.finish_deadline()))
                            : (race().race_over() ||
                               (out_of_race(player_) &&
                                std::all_of(views_.begin(), views_.end(), [&](const LocalView& v) {
                                    return out_of_race(v.racer);
                                })))) ||
                      (setup_.end_after && session_.ticks() >= setup_.end_after);
    // ⚠️ NOT THE MOMENT THE LINE IS CROSSED: five seconds later, counted down on screen (user,
    // 2026-09-27 - "la fin d'une course est abrupte"). The race goes on ticking meanwhile, the
    // car rolling past the line under the FINISHED banner. Counted in SIMULATION TICKS, not wall time: every machine
    // of a network race then leaves on the same tick, as it already does when the race ends,
    // and none of them stops sending inputs while another still needs them.
    if (over && over_tick_ == 0) over_tick_ = session_.ticks() + 1;
    const bool results_due =
        over && (session_.ticks() + 1 - over_tick_ >= kResultsDelayTicks ||
                 (setup_.end_after && session_.ticks() >= setup_.end_after));
    // The fade to black over the last ticks before the results - a display thing, read off the
    // tick count, so it ends exactly when the results come up.
    if (over && over_tick_ != 0 && !results_pushed_) {
        const double into = static_cast<double>(session_.ticks() + 1 - over_tick_) +
                            accumulator_ / kSimStep;
        const double from = static_cast<double>(kResultsDelayTicks - kFadeTicks);
        fade_ = static_cast<float>(std::clamp((into - from) / kFadeTicks, 0.0, 1.0));
    }
    if (results_due && !results_pushed_) {
        results_pushed_ = true;
        push_results(app);
    }
    // How far the wall clock has travelled into the tick that has not happened yet.
    alpha_ = static_cast<float>(accumulator_ / kSimStep);

    // Camera: whatever the last two ticks left, read at the fraction of a tick we are at.
    views_[0].place(view_prev(), view_now(), alpha_, scene_.map_w);
    for (size_t i = 1; i < views_.size(); ++i)
        views_[i].place(views_[i].cam_prev, views_[i].camera.now, alpha_, scene_.map_w);
    // -CAMERA=COMFORT replaces the chase views (not the trackside one) with the comfort rig,
    // which follows exactly the pose being drawn, network visual correction included.
    if (app.comfort_camera() && !setup_.fly_by) {
        for (size_t i = 0; i < views_.size(); ++i) {
            if (i == 0 && spectator_.active()) continue;
            LocalView& v = views_[i];
            const int watched = v.watch >= 0 ? v.watch : v.racer;
            // Use exactly the pose being drawn, including network visual correction.
            const VehiclePose pose = shown_pose(watched);
            bool confined = session_.car(watched).in_tunnel;
            if (v.comfort_camera.ready() && !v.comfort_cut_pending) {
                const CameraSample& eye = v.comfort_camera.current();
                confined = confined || camera_under_roof(level_.terrain, {eye.x, eye.y, eye.z});
            }
            const ComfortTarget target{{pose.x, pose.y, pose.z}, pose.yaw,
                                       v.camera.preset, confined};
            const CameraSample sample = v.comfort_camera.update(target, dt_ms / 1000.0,
                v.comfort_cut_pending, [this](CameraPoint focus, CameraPoint eye) {
                    return camera_arm_fraction(level_.terrain, focus, eye);
                });
            v.comfort_cut_pending = false;
            v.place(camera_view(sample, scene_.map_w, kRenderScale));
        }
    }
    for (size_t i = 0; i < views_.size(); ++i) place_cabin(views_[i], i == 0 && spectator_.active());
}

VehicleLighting RaceScreen::light_for(const LocalView& v, const VehiclePose& pose) const {
    VehicleLighting l;
    for (int k = 0; k < 3; ++k) l.sun[k] = sun_[k];
    // The camera back in game axes: render X is mirrored about (map_w - 1) * 512.
    const float mirror_axis =
        static_cast<float>(scene_.map_w - 1) * static_cast<float>(kTerrainTexelWorldSize);
    const float cx = mirror_axis - v.cam_pos.x / kRenderScale;
    const float cy = -v.cam_pos.y / kRenderScale;
    const float cz = v.cam_pos.z / kRenderScale;
    const float dx = pose.x - cx, dy = pose.y - cy, dz = pose.z - cz;
    l.intensity = light_intensity(std::sqrt(dx * dx + dy * dy + dz * dz));
    return l;
}

void RaceScreen::cycle_view(LocalView& v) {
    const size_t si = static_cast<size_t>(v.racer);
    const bool has_cabin = si < cabins_.size() && cabins_[si].model.valid();
    if (v.cabin) {
        v.cabin = false;
        v.camera.cycle_preset();   // from the third height back to the first
    } else if (v.camera.preset == 2 && has_cabin && !setup_.fly_by) {
        v.cabin = true;            // the chase camera keeps running underneath
    } else {
        v.camera.cycle_preset();
    }
}

void RaceScreen::place_cabin(LocalView& v, bool spectating) {
    v.in_cabin = false;
    const size_t si = static_cast<size_t>(v.racer);
    if (!v.cabin || spectating || setup_.fly_by || v.watch != v.racer || si >= cabins_.size() ||
        !cabins_[si].model.valid())
        return;
    // A wreck leaves the cabin for good (as in the original: damage at 100 % -> the chase camera).
    const VehicleSim& car = session_.car(v.racer);
    if (car.damage_percent >= 100) {
        v.cabin = false;
        return;
    }
    const std::vector<VehicleSection>& sections = field_.tunings[si].sections;
    if (sections.size() < 2) return;
    int32_t internal[4] = {sections[1].internal[0], sections[1].internal[1],
                           sections[1].internal[2], sections[1].internal[3]};
    if (v.cabin_look_set)
        internal[3] = v.cabin_look_prev +
                      static_cast<int32_t>((v.cabin_look - v.cabin_look_prev) * alpha_);
    const CameraView cv = cabin_view(shown_pose(v.racer), internal, scene_.map_w, kRenderScale);
    v.cam_pos = {cv.eye[0], cv.eye[1], cv.eye[2]};
    v.look = {cv.forward[0], cv.forward[1], cv.forward[2]};
    v.right = {cv.right[0], cv.right[1], cv.right[2]};
    v.sky_up = {cv.up[0], cv.up[1], cv.up[2]};
    v.in_cabin = true;
}

// The six actions of one set of controls (app/local_players.h).
//
// The pad, as the user asked for it: right trigger accelerates, left trigger brakes and
// reverses, the BOTTOM face button is the nitro, the TOP one puts the car back on the track.
// Steering is the left stick or the d-pad. All of them are POSITIONS on the pad (app/app.h).
//
// ⚠️ ALONE, KEYBOARD AND EVERY PAD ARE OR'ED, NOT CHOSEN BETWEEN (`Everything`): a player must
// be able to pick up whichever is to hand mid-race. On a split screen each player has their own
// half of the keyboard or their own pad, and reads nothing else.
VehicleInput RaceScreen::read_controls(App& app, Controls c) const {
    VehicleInput in;
    const Uint8* k = SDL_GetKeyboardState(nullptr);
    const auto pad_into = [&](int pad) {   // -1: every pad at once
        const auto axis = [&](PadAxis a) { return pad < 0 ? app.pad_axis(a) : app.pad_axis(a, pad); };
        const auto down = [&](PadButton b) { return pad < 0 ? app.pad_down(b) : app.pad_down(b, pad); };
        const int stick = axis(PadAxis::LeftX);
        in.accel = in.accel || axis(PadAxis::TriggerRight) > App::kPadTriggerOn;
        in.brake = in.brake || axis(PadAxis::TriggerLeft) > App::kPadTriggerOn;
        in.left = in.left || down(PadButton::DpadLeft) || stick < -App::kPadStickOn;
        in.right = in.right || down(PadButton::DpadRight) || stick > App::kPadStickOn;
        in.nitro = in.nitro || down(PadButton::South);
        in.recover = in.recover || down(PadButton::North);
    };
    switch (c) {
    case Controls::Everything: {
        // The player's own keys (Options -> Controls). A modifier stands for both of its sides,
        // which is what the arrows-Ctrl-Alt default always did.
        const auto held = [&](int action) {
            const int sc = keys_[static_cast<size_t>(action)];
            if (sc <= SDL_SCANCODE_UNKNOWN || sc >= SDL_NUM_SCANCODES) return false;
            if (k[sc]) return true;
            if (sc >= SDL_SCANCODE_LCTRL && sc <= SDL_SCANCODE_LGUI) return k[sc + 4] != 0;
            if (sc >= SDL_SCANCODE_RCTRL && sc <= SDL_SCANCODE_RGUI) return k[sc - 4] != 0;
            return false;
        };
        in.accel = held(kKeyAccel);
        in.brake = held(kKeyBrake);
        in.left = held(kKeyLeft);
        in.right = held(kKeyRight);
        in.nitro = held(kKeyNitro);
        in.recover = held(kKeyRecover);
        pad_into(-1);
        break;
    }
    case Controls::KeyboardRight:
        in.accel = k[SDL_SCANCODE_UP] != 0;
        in.brake = k[SDL_SCANCODE_DOWN] != 0;
        in.left = k[SDL_SCANCODE_LEFT] != 0;
        in.right = k[SDL_SCANCODE_RIGHT] != 0;
        in.nitro = k[SDL_SCANCODE_RCTRL] != 0;
        in.recover = k[SDL_SCANCODE_RSHIFT] != 0;
        break;
    case Controls::KeyboardLeft:
        in.accel = k[SDL_SCANCODE_W] != 0;
        in.brake = k[SDL_SCANCODE_S] != 0;
        in.left = k[SDL_SCANCODE_A] != 0;
        in.right = k[SDL_SCANCODE_D] != 0;
        in.nitro = k[SDL_SCANCODE_LSHIFT] != 0;
        in.recover = k[SDL_SCANCODE_LCTRL] != 0;
        break;
    default:
        pad_into(controls_pad(c));
        break;
    }
    return in;
}

int RaceScreen::local_seat_of(int racer) const {
    for (size_t v = 0; v < views_.size(); ++v)
        if (views_[v].racer == racer) return static_cast<int>(v);
    return -1;
}

void RaceScreen::LocalView::place(const CameraState& prev, const CameraState& now, float alpha,
                                  int map_w) {
    place(camera_view(prev, now, alpha, map_w, kRenderScale));
}

void RaceScreen::LocalView::place(const CameraView& v) {
    cam_pos = {v.eye[0], v.eye[1], v.eye[2]};
    look = {v.forward[0], v.forward[1], v.forward[2]};
    right = {v.right[0], v.right[1], v.right[2]};
    sky_up = {v.up[0], v.up[1], v.up[2]};
}

int RaceScreen::watch_for(int own) const {
    if (net_ || !split() || !out_of_race(own)) return own;
    for (const LocalView& v : views_)
        if (const int r = v.racer; r != own && !out_of_race(r) && session_.car(r).active) return r;
    for (int i = 0; i < session_.racer_count(); ++i)
        if (i != own && field_.racers[static_cast<size_t>(i)].tuning && !out_of_race(i) &&
            session_.car(i).active)
            return i;
    return own;
}

void RaceScreen::follow(LocalView& v) {
    int& watch = v.watch;
    ChaseCamera& cam = v.camera;
    const int want = watch_for(v.racer);
    if (want != watch) {
        // The first call lands here too, and the camera is already behind its own car.
        if (watch >= 0) {
            cam.reset(session_.car(want), level_.terrain);
            v.comfort_cut_pending = true;
        }
        watch = want;
    } else if (session_.respawned(want)) {
        // A respawn re-enters camera mode 4 in the original, which re-plants the camera behind
        // the vehicle instead of letting the rope drag it across the whole map.
        cam.reset(session_.car(want), level_.terrain);
        v.comfort_cut_pending = true;
    }
    cam.step(session_.car(want), level_.terrain);   // on the tick too - never on the frame

    // The cabin's head dip (LocalView::cabin_look): only while this view is in its own cabin.
    const size_t si = static_cast<size_t>(v.racer);
    if (!v.cabin || want != v.racer || si >= field_.tunings.size() ||
        field_.tunings[si].sections.size() < 2) {
        v.cabin_look_set = false;
        return;
    }
    const int32_t rest = field_.tunings[si].sections[1].internal[3];
    if (!v.cabin_look_set) {
        v.cabin_look = rest;
        v.cabin_look_set = true;
    }
    v.cabin_look_prev = v.cabin_look;
    const VehicleSim& car = session_.car(want);
    v.cabin_look -= car.head_kick;   // NOT car.vfwd, which is already past the rebound
    constexpr int32_t kCabinLookReturn = 0x40;   // the original's step, per tick
    v.cabin_look += std::clamp(rest - v.cabin_look, -kCabinLookReturn, kCabinLookReturn);
}

void RaceScreen::step_simulation() {
    ++sim_ticks_;
    // The race and every car, in the original's order - see RaceSession::tick. One input per
    // racer, filled in by the caller: the local player's alone, or everyone's off the network.
    // The poses are kept BEFORE stepping, so the pair always holds two consecutive ticks even
    // when a slow frame runs several of them.
    pose_prev_ = pose_cur_;
    session_.tick(inputs_.data());
    for (int i = 0; i < session_.racer_count(); ++i)
        pose_cur_[static_cast<size_t>(i)] = pose_of(session_.car(i));
    for (LocalView& v : views_) {
        v.cam_prev = v.camera.now;
        follow(v);
    }
    // The finish countdown is the session's (RaceSession::finish_deadline); said once, here.
    if (session_.finish_deadline() != 0 && !finish_announced_) {
        finish_announced_ = true;
        std::printf("[race] more than half the humans home at tick %u: the race ends at tick %u\n",
                    session_.ticks(), session_.finish_deadline());
        std::fflush(stdout);
    }
    update_spectator();
    // The ground effects, in the original's own order: the emitter runs right after the physics
    // (the same tick routine calls both), and the field is aged at the end of the tick.
    //
    // ⚠️ TWICE. The field ages on the 14 ms TIMER tick while the physics runs on every other
    // one, so one 28 ms step of ours is two of its ticks. Ageing it once would run every puff of
    // dust at half speed and double its life.
    // EVERY racer raises its own dust, the ghosts included (their ground contact is derived
    // from the tape, VehicleSim::dress_replayed), in grid order.
    for (int i = 0; i < session_.racer_count(); ++i)
        if (session_.car(i).active)
            particles_.emit(session_.car(i), static_cast<uint32_t>(sim_ticks_));
    particles_.tick();
    particles_.tick();

    // -DEBUG: the race, to the console. One line per crossing, and it says both what the ring
    // did and whether the sound that goes with it had anywhere to come from - the two halves of
    // "the checkpoint beep is almost never heard".
    if (setup_.trace) {
        const RacerProgress& me = race().progress(player_);
        const bool was_respawning = session_.respawned(player_);
        if (me.gate_crossed || me.lap_crossed || me.checkpoint_crossed || me.missed_check_now) {
            const uint32_t ms = race().race_ticks() * kGameTickMs;
            const bool sounding = me.reached >= 0 &&
                                  static_cast<size_t>(me.reached) < race().gates().size() &&
                                  race().gates()[static_cast<size_t>(me.reached)].checkpoint;
            std::printf("[race] %u:%02u.%02u  porte %d/%zu%s%s%s%s  "
                        "(tour %d, dernier checkpoint %d%s, reste %d)\n",
                        ms / 60000u, (ms / 1000u) % 60u, (ms / 10u) % 100u, me.reached,
                        race().gates().size(), sounding ? "*" : " ",
                        me.checkpoint_crossed ? "  CHECKPOINT (bip)" : "",
                        me.lap_crossed ? "  TOUR BOUCLE" : "",
                        me.missed_check_now ? "  CHECKPOINT RATE" : "",
                        me.lap, me.last_check == 0x80 ? -1 : (me.last_check & 0x7f),
                        (me.last_check & 0x80) && me.last_check != 0x80 ? "*" : "",
                        me.checks_missing);
        }
        if (me.wrong_way_now) std::printf("[race] SENS INTERDIT\n");
        if (me.gate_crossed && was_respawning)
            std::printf("[race] remis en piste porte %d\n", me.reached);
    }
    note_wrecks();
    // The sound, on the tick and not on the frame: an impact or a landing lasts one tick.
    audio_.on_tick(view_now());
}

// THE SPLIT SCREEN's layout. The viewport the App set up is the whole target - the window, or
// the low-resolution buffer of the `+`/`-` ladder - and every view is a share of it; the layout
// itself is in window pixels, which is what the board is drawn in. `each(i, rect)` is called
// for view `i` with the viewport and the scissor on its share; then, with three players, once
// more with i = -1 for the fourth corner.
template <typename F> void RaceScreen::for_each_split_view(App& app, F each) {
    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    const int ww = app.width(), wh = app.height();
    const SplitLayout layout = split_layout(static_cast<int>(views_.size()), ww, wh, 2,
                                             setup_.split_side_by_side);
    const auto to_gl = [&](const SplitRect& r) {
        const GLint x0 = vp[0] + r.x * vp[2] / ww, x1 = vp[0] + (r.x + r.w) * vp[2] / ww;
        const GLint y0 = vp[1] + (wh - r.y - r.h) * vp[3] / wh, y1 = vp[1] + (wh - r.y) * vp[3] / wh;
        glViewport(x0, y0, x1 - x0, y1 - y0);
        glScissor(x0, y0, x1 - x0, y1 - y0);
    };
    glEnable(GL_SCISSOR_TEST);
    for (size_t i = 0; i < layout.views.size(); ++i) {
        to_gl(layout.views[i]);
        each(static_cast<int>(i), layout.views[i]);
    }
    if (layout.has_map) {
        to_gl(layout.map);
        each(-1, layout.map);
    }
    glDisable(GL_SCISSOR_TEST);
    glViewport(vp[0], vp[1], vp[2], vp[3]);
}

void RaceScreen::draw(App& app) {
    // "Loaded" is said once the race has been drawn - see NetRaceDriver::frame_drawn.
    if (net_) net_race_.frame_drawn();
    // Smoothed textures at the window's own resolution only; the low rungs sample nearest.
    renderer_.set_smooth_textures(app.presentation().lines() == 0);
    if (!split()) {
        draw_view(views_[0], app.width(), app.height());
        return;
    }
    // The lines between the views are whatever this leaves.
    glDisable(GL_SCISSOR_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    for_each_split_view(app, [&](int i, const SplitRect& r) {
        if (i >= 0) draw_view(views_[static_cast<size_t>(i)], r.w, r.h);
    });
}

// ⚠️ THE BOARD IS DRAWN HERE, AT THE WINDOW'S OWN RESOLUTION, not in draw(). draw() may be
// rendering into the 200-line buffer of the `+`/`-` ladder, and the board used to go in with
// it: laid out in window pixels and squeezed into 200 lines, an 8-row glyph came out on four
// and the words could not be read (user, 2026-09-28). The rung is the world's, as in the menus.
// The same split views as draw(), in the same order, over the stretched world.
void RaceScreen::draw_overlay(App& app) {
    const int ww = app.width(), wh = app.height();
    if (!split()) {
        draw_board(views_[0], ww, wh);
    } else {
        for_each_split_view(app, [&](int i, const SplitRect& r) {
            if (i >= 0) draw_board(views_[static_cast<size_t>(i)], r.w, r.h);
            else board_.draw_race_map(session_, board_dots(-1), r.w, r.h);
        });
    }
    if (confirm_quit_) board_.draw_quit_confirm(ww, wh, net_ != nullptr);
    board_.draw_fade(ww, wh, fade_);
}

VehiclePose RaceScreen::shown_pose(int racer) const {
    const size_t r = static_cast<size_t>(racer);
    return drawn_pose(racer, lerp_pose(pose_prev_[r], pose_cur_[r], alpha_));
}

// The current view's camera, for a `width` x `height` view - the world's and the name tags'.
Mat4 RaceScreen::view_mvp(const LocalView& v, int width, int height) const {
    const Mat4::Vec3 target{v.cam_pos.x + v.look.x, v.cam_pos.y + v.look.y, v.cam_pos.z + v.look.z};
    // The chase camera never rolls; the cabin does, with the car (cabin_view).
    const Mat4::Vec3 up = v.in_cabin ? v.sky_up : Mat4::Vec3{0.0f, 1.0f, 0.0f};
    Mat4 view = Mat4::lookAt(v.cam_pos, target, up);
    // Far at twice the map's width: its diagonal is 1.41 of it, and every factor spent past what
    // can be seen is depth precision taken from the near field, which kNearPlaneWorld now needs.
    Mat4 proj = Mat4::perspective(60.0f * 3.14159265f / 180.0f,
                                  static_cast<float>(width) / static_cast<float>(height),
                                  kNearPlaneWorld * kRenderScale, map_span_ * 2.0f);
    return proj * view * Mat4::identity();
}

void RaceScreen::draw_view(const LocalView& v, int width, int height) {
    // What is not the world - the cars and the effects - built for this frame, then the whole
    // picture drawn by the shared renderer (scene/world_renderer.h).
    car_scratch_.clear();
    // The vehicle. Streamed rather than baked: it is one small mesh and it moves every frame.
    // Same shader, same vertex layout, same world transform as the decor.
    if (car_model_.valid() && (!setup_.fly_by || fly_heli_)) {
        // The first player's car, whichever view this is - on a split screen the view being
        // drawn may be a guest's, and the guest's own car is one of the "others" below.
        // From its own cabin a car's body is not drawn (the original skips it too).
        if (!(v.in_cabin && v.racer == player_)) {
            const VehiclePose pose = shown_pose(player_);
            const VehicleLighting light = light_for(v, pose);
            car_scratch_ = place_vehicle(car_model_, pose, scene_.map_w, kRenderScale, &light);
        }
        // The opponents, into the same stream and the same draw call.
        for (size_t i = 0; i < ghost_models_.size(); ++i) {
            if (static_cast<int>(i) == player_) continue;
            if (v.in_cabin && static_cast<int>(i) == v.racer) continue;
            if (!ghost_models_[i].valid() || i >= pose_cur_.size()) continue;
            const VehiclePose pose = shown_pose(static_cast<int>(i));
            const VehicleLighting light = light_for(v, pose);
            const std::vector<TerrainVertex> g =
                place_vehicle(ghost_models_[i], pose, scene_.map_w, kRenderScale, &light);
            car_scratch_.insert(car_scratch_.end(), g.begin(), g.end());
        }
    }

    // The ground effects. Streamed like the vehicle: at most 100 live particles of a few quads
    // each, rebuilt from the field every frame.
    {
        const float r3[3] = {v.right.x, v.right.y, v.right.z};
        const float u3[3] = {v.sky_up.x, v.sky_up.y, v.sky_up.z};
        fx_scratch_.clear();
        if (particle_visuals_.any())
            fx_scratch_ = place_particles(particle_visuals_, particles_, r3, u3, scene_.map_w,
                                          kRenderScale);
        // The damage effects of every car - smoke, engine fire, explosion - into the same
        // unculled stream (scene/vehicle_render.h, place_vehicle_effects).
        if (!setup_.fly_by || fly_heli_) {
            const float c3[3] = {v.cam_pos.x, v.cam_pos.y, v.cam_pos.z};
            const auto add = [&](const VehicleModel& m, const VehiclePose& pose) {
                const std::vector<TerrainVertex> e =
                    place_vehicle_effects(m, pose, c3, r3, u3, scene_.map_w, kRenderScale);
                fx_scratch_.insert(fx_scratch_.end(), e.begin(), e.end());
            };
            if (car_model_.valid() && !(v.in_cabin && v.racer == player_))
                add(car_model_, shown_pose(player_));
            for (size_t i = 0; i < ghost_models_.size(); ++i) {
                if (static_cast<int>(i) == player_) continue;
                if (!ghost_models_[i].valid() || i >= pose_cur_.size()) continue;
                add(ghost_models_[i],
                    shown_pose(static_cast<int>(i)));
            }
        }
    }

    WorldFrame f;
    f.view.mvp = view_mvp(v, width, height);
    f.view.eye = v.cam_pos;
    f.view.forward = v.look;
    f.view.right = v.right;
    f.view.up = v.sky_up;
    f.view.aspect = static_cast<float>(width) / static_cast<float>(height);
    f.wall_ms = SDL_GetTicks();
    f.world = &world_;
    f.sim_tick = car().tick;
    // The gantry shows the countdown and nothing else: the three beeps are the three
    // red/amber steps.
    {
        using SL = LevelScene::StartLight;
        const int n = race().countdown_number();
        f.start_light = (n >= 3) ? SL::Red : (n == 2) ? SL::RedAmber : (n == 1) ? SL::Amber : SL::Green;
    }
    f.vehicles = &car_scratch_;
    f.effects = &fx_scratch_;

    // THE CABIN: the dashboard redrawn from this car's own readings, then the interior over
    // everything with a near plane of its own (WorldFrame::cabin).
    cabin_scratch_.clear();
    if (v.in_cabin) {
        Cabin& c = cabins_[static_cast<size_t>(v.racer)];
        const VehicleSim& car = session_.car(v.racer);
        if (c.dash.valid() && c.layer >= 0) {
            DashValues dv;
            dv.by_source = {car.vfwd, car.rev, car.nitro_fuel, car.gear,
                            (level_.terrain.height_fx(car.px, car.pz) - car.py) >> 9};
            c.dash.draw(dv);
            renderer_.update_layer(c.layer, decode_tm_rgba(c.dash.pixels().data(), scene_.palette));
        }
        const VehiclePose cabin_pose = shown_pose(v.racer);
        const VehicleLighting cabin_light = light_for(v, cabin_pose);
        cabin_scratch_ =
            place_vehicle(c.model, cabin_pose, scene_.map_w, kRenderScale, &cabin_light);
        // place_vehicle emits the body first, then the parts: split there, so the steering wheel
        // can be painted over the dashboard (WorldFrame::cabin_parts).
        const size_t body = std::min(c.model.body.size(), cabin_scratch_.size());
        cabin_parts_scratch_.assign(cabin_scratch_.begin() + static_cast<std::ptrdiff_t>(body),
                                    cabin_scratch_.end());
        cabin_scratch_.resize(body);
        f.cabin_parts = &cabin_parts_scratch_;
        const Mat4::Vec3 target{v.cam_pos.x + v.look.x, v.cam_pos.y + v.look.y,
                                v.cam_pos.z + v.look.z};
        f.cabin_mvp = Mat4::perspective(60.0f * 3.14159265f / 180.0f,
                                        static_cast<float>(width) / static_cast<float>(height),
                                        kCabinNearWorld * kRenderScale,
                                        kCabinFarWorld * kRenderScale) *
                      Mat4::lookAt(v.cam_pos, target, v.sky_up);
        f.cabin = &cabin_scratch_;
    }
    renderer_.draw(scene_, f);
}

// --- the board --------------------------------------------------------------------------------
// What the board shows is decided here, how it is laid out in game/race_board.cpp. Drawn last,
// over everything, and read ONLY from the race and the cars - never from a frame counter.
// ⚠️ NOT ON A FLY-BY: a board over a presentation shot is the one thing the original never does.
void RaceScreen::draw_board(const LocalView& v, int width, int height) {
    if (setup_.fly_by) return;
    BoardFrame f;
    f.width = width;
    f.height = height;
    f.mvp = view_mvp(v, width, height);
    f.session = &session_;
    f.player = v.racer;
    f.tuning = &field_.tunings[static_cast<size_t>(v.racer)];
    f.nitros = &level_.nitros;
    f.show_minimap = views_.size() != 3;   // three players: the race map has the corner
    f.dots = board_dots(v.racer);

    // NAME TAGS over the other vehicles, in a network race or on a split screen: who is who
    // (user, 2026-09-25). A human carries the name of their profile, a ghost just "IA" - the
    // user's word. Not in the original, which had no names to show. Placed with the world's own
    // transform, so a tag sits on its car wherever the camera is.
    if (net_ || split()) {
        // The camera the distances are taken from: the first view's may be the spectator's.
        const CameraState& eye = &v == &views_[0] ? view_now() : v.camera.now;
        const float mirror_axis =
            static_cast<float>(scene_.map_w - 1) * static_cast<float>(kTerrainTexelWorldSize);
        for (int i = 0; i < session_.racer_count(); ++i) {
            if ((i == v.racer && !spectator_.active()) || !session_.car(i).active) continue;
            if (static_cast<size_t>(i) >= pose_cur_.size()) continue;
            const VehiclePose p = shown_pose(i);
            // Just over the roof: the top of the vehicle's own contact box (40 to 80 units over
            // its origin), plus a margin. It used to be a flat 450, which put the tag of every
            // car near the camera off the top of the screen - so no tag was ever seen close up.
            // Game +Y is down, hence the minus; X is the world's mirror (CLAUDE.md).
            const float lift =
                60.0f - static_cast<float>(field_.tunings[static_cast<size_t>(i)].box.ytop);
            BoardTag t;
            t.x = (mirror_axis - p.x) * kRenderScale;
            t.y = -(p.y - lift) * kRenderScale;
            t.z = p.z * kRenderScale;
            const float dx = p.x - eye.x, dz = p.z - eye.z;
            t.distance = std::sqrt(dx * dx + dz * dz);
            const int h = field_.human[static_cast<size_t>(i)];
            t.text = "IA";
            if (net_) {
                t.human = h >= 0 && static_cast<size_t>(h) < net_->race().racers.size();
                if (t.human) t.text = net_->race().racers[static_cast<size_t>(h)].look.name;
            } else if (const int seat = local_seat_of(i); seat >= 0) {
                t.human = true;
                t.text = seat == 0 ? setup_.player_name
                                   : setup_.guests[static_cast<size_t>(seat - 1)].name;
            }
            f.tags.push_back(std::move(t));
        }
    }

    // Split screen: this quarter follows someone else now (watch_for).
    if (v.watch >= 0 && v.watch != v.racer) {
        const int seat = local_seat_of(v.watch);
        const std::string who = seat == 0  ? setup_.player_name
                                : seat > 0 ? setup_.guests[static_cast<size_t>(seat - 1)].name
                                           : "GHOST " + field_.vehicle[static_cast<size_t>(v.watch)];
        f.watching = "WATCHING " + who;
        for (char& ch : f.watching) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    }

    // The count to the results (user, 2026-09-27): whole seconds, rounded up, so it reads
    // 5 4 3 2 1 and the table comes on the tick the 1 would have run out.
    if (over_tick_ != 0 && !results_pushed_) {
        const uint32_t gone = session_.ticks() + 1 - over_tick_;
        if (gone < kResultsDelayTicks) {
            const uint32_t left_ms = (kResultsDelayTicks - gone) * kPhysicsTickMs;
            f.results_in = static_cast<int>((left_ms + 999) / 1000);
        }
    }
    if (net_) network_status(f.status, f.status_rgb);
    board_.draw(f);
}

// Every racer's dot on the mini-map, the view's own car LAST so it is never hidden under an
// opponent's. ⚠️ NOT THE ORIGINAL'S COLOURING, which gave each racer its own dot (GAMESVGA
// 14 + colour): at the user's request (2026-09-25, after a network test) the player is RED and
// every opponent the same ORANGE - with up to twelve racers and six colours, "which one is me"
// mattered more than "which one is who". On a split screen every player keeps one colour on
// every map - player 1 red, 2 blue, 3 green, 4 violet. `own` = -1: the race map of a three-way
// split, where every player is drawn in their colour and nobody is "the view's own".
std::vector<BoardDot> RaceScreen::board_dots(int own) const {
    namespace g = game_spr;
    std::vector<BoardDot> dots;
    constexpr int kDotOrange = g::kDot0 + 4;
    for (int i = 0; i < session_.racer_count(); ++i)
        if (local_seat_of(i) < 0 && session_.car(i).active) dots.push_back({i, kDotOrange});
    for (int i = 0; i < session_.racer_count(); ++i)
        if (const int seat = local_seat_of(i); seat >= 0 && i != own)
            dots.push_back({i, g::kDot0 + kLocalDot[seat]});
    if (own >= 0) dots.push_back({own, g::kDot0 + kLocalDot[std::max(0, local_seat_of(own))]});
    return dots;
}

// --- the network race -------------------------------------------------------------------------

void RaceScreen::update_spectator() {
    const bool want = ((net_ && out_of_race(player_)) || setup_.spectate) &&
                      !results_pushed_ && race().started();
    std::vector<int> left;
    if (want) {
        // Who is still racing: the humans first, the ghosts only if no human is left (user,
        // 2026-09-25). ⚠️ A human who has LEFT the game is not one: the host keeps their car on
        // the track with nothing pressed, and following a parked car is watching nobody.
        for (size_t h = 0; h < field_.session_of.size(); ++h) {
            const int s = field_.session_of[h];
            if (net_ && !net_->racer_present(static_cast<int>(h))) continue;
            if (s != player_ && !out_of_race(s) && session_.car(s).active) left.push_back(s);
        }
        if (left.empty())
            for (int i = 0; i < session_.racer_count(); ++i)
                if (i != player_ && !out_of_race(i) && session_.car(i).active) left.push_back(i);
    }
    spectator_.tick(session_, level_.terrain, want, left);
}

bool RaceScreen::humans_finished() const {
    for (size_t h = 0; h < field_.session_of.size(); ++h) {
        if (net_ && !net_->racer_present(static_cast<int>(h))) continue;
        if (!out_of_race(field_.session_of[h])) return false;
    }
    return true;
}

// One line under the board: why the race is not moving, or that it has gone wrong. Only ever
// drawn in a network race.
void RaceScreen::network_status(std::string& line, float rgb[3]) const {
    float r = 1.0f, g = 1.0f, b = 0.4f;
    if (!net_->active()) {
        line = (net_->error().empty() ? std::string("THE GAME IS OVER") : net_->error()) +
               " - ESC";
        g = 0.5f;
        b = 0.5f;
    } else if (!net_->race_go()) {
        line = "WAITING FOR THE OTHER PLAYERS";
    } else if (net_->desync_tick() != 0) {
        line = "DESYNCHRONISED AT TICK " + std::to_string(net_->desync_tick()) +
               (net_->resync_tick() ? " - RESYNCHRONISING" : "");
        g = 0.5f;
        b = 0.5f;
    } else if (net_race_.resync_banner()) {
        line = "RESYNCHRONISED";
        r = 0.5f;
        b = 0.5f;
    } else if (session_.finish_deadline() && session_.ticks() < session_.finish_deadline()) {
        const uint32_t left_ms = (session_.finish_deadline() - session_.ticks()) * kPhysicsTickMs;
        line = "THE RACE ENDS IN " + std::to_string((left_ms + 999) / 1000);
        if (spectator_.active() && spectator_.target() >= 0) {
            const int h = field_.human[static_cast<size_t>(spectator_.target())];
            line += " - WATCHING " + (h >= 0 ? net_->race().racers[static_cast<size_t>(h)].look.name
                                             : field_.vehicle[static_cast<size_t>(spectator_.target())]);
        }
    } else if (out_of_race(player_) && !humans_finished()) {
        line = "WAITING FOR THE OTHERS TO FINISH";
    } else if (net_race_.waiting() && net_race_.stall_ms() >= 300.0 && race().started()) {
        line = net_race_.stalled_on_someone() ? "WAITING FOR " + net_race_.stall_name()
                                   : std::string("WAITING FOR THE NETWORK");
        b = 1.0f;
    }
    rgb[0] = r;
    rgb[1] = g;
    rgb[2] = b;
}

void RaceScreen::push_results(App& app) {
    const RacerProgress& me = race().progress(player_);
    ResultsScreen::Setup rs;
    rs.level_title = setup_.level_title.empty() ? setup_.level : setup_.level_title;
    rs.laps = race().laps();
    rs.place = me.place;
    rs.finish_ticks = me.finish_tick;
    rs.lap_ticks = me.lap_ticks;
    // EVERYONE, as data: the screen lays it out. A human by the
    // name they chose, a ghost as "GHOST" and what it drives.
    Competition::LocalSeries& ser = app.competition().series();
    const bool local_series = !net_ && split() && ser.circuits.size() > 1;
    if (local_series && ser.points.size() < app.locals().seats.size())
        ser.points.resize(app.locals().seats.size(), 0);
    if (local_series) ser.last_race.assign(ser.points.size(), 0);
    for (int i = 0; i < session_.racer_count(); ++i) {
        if (!field_.racers[static_cast<size_t>(i)].tuning) continue;
        using Row = ResultsScreen::Setup::Row;
        Row r;
        const RacerProgress& p = race().progress(i);
        r.place = p.place;
        r.finish_ticks = p.finish_tick;
        r.status = p.finished ? Row::Status::Finished
                 : session_.car(i).damage < 0 ? Row::Status::Wrecked
                                              : Row::Status::Running;
        const int seat = local_seat_of(i);
        r.local = seat >= 0;
        const int h = field_.human[static_cast<size_t>(i)];
        if (net_) {
            const NetRaceStart& start = net_->race();
            r.local = i == player_;
            if (h >= 0) {
                r.name = start.racers[static_cast<size_t>(h)].look.name;
                if (!net_->racer_present(h)) r.status = Row::Status::Left;
                // The series: the lobby's total when this race started, plus this race's -
                // every machine works it out; the host records it just below.
                r.race_points = net_series_points(r.place);
                const auto before = net_points_before_.find(start.racers[static_cast<size_t>(h)].player_id);
                r.total_points = r.race_points + (before != net_points_before_.end() ? before->second : 0);
            }
        } else if (seat >= 0) {
            r.name = split() ? (seat == 0 ? setup_.player_name
                                          : setup_.guests[static_cast<size_t>(seat - 1)].name)
                             : app.profile().pseudo;
            if (local_series) {
                std::vector<int>& pts = ser.points;
                r.race_points = net_series_points(r.place);
                pts[static_cast<size_t>(seat)] += r.race_points;
                ser.last_race[static_cast<size_t>(seat)] = r.race_points;
                r.total_points = pts[static_cast<size_t>(seat)];
            }
        }
        if (r.name.empty()) r.name = "GHOST " + field_.vehicle[static_cast<size_t>(i)];
        for (char& c : r.name) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        rs.rows.push_back(std::move(r));
    }
    if (local_series) {
        rs.heading = (ser.next + 1 >= ser.circuits.size() ? "SERIES OVER - RACE " : "RACE ") +
                     std::to_string(ser.next + 1) + " OF " + std::to_string(ser.circuits.size());
    }
    if (net_ && net_->lobby().settings.races > 1) {
        const NetLobbyState& lobby = net_->lobby();
        const int n = std::min<int>(net_series_done_before_ + 1, lobby.settings.races);
        rs.heading = (n >= lobby.settings.races ? std::string("SERIES OVER - RACE ") : std::string("RACE ")) +
                     std::to_string(n) + " OF " + std::to_string(lobby.settings.races);
    }
    // Written out too, so a test with no hands can read what the screen was given.
    if (!rs.heading.empty()) std::printf("[results] %s\n", rs.heading.c_str());
    for (const ResultsScreen::Setup::Row& r : rs.rows)
        std::printf("[results] %d. %s%s  status %d  ticks %u  points +%d = %d\n", r.place,
                    r.name.c_str(), r.local ? " (here)" : "", static_cast<int>(r.status),
                    r.finish_ticks, r.race_points, r.total_points);
    if (setup_.autodrive) rs.auto_continue_ms = 2000.0;
    // A race of the session's competition (a championship) counts the moment it is over, and
    // the results say where the competition stands after it (game/competition.h).
    if (!net_ && !split() && !setup_.fly_by) {
        app.competition().race_over(setup_.level, me.place);
        const std::string line = app.competition().results_heading();
        if (!line.empty()) rs.heading = line;
    }
    // The series: the host counts the points, once per race.
    if (net_ && net_->is_host() && net_->active()) {
        const NetRaceStart& start = net_->race();
        std::vector<std::pair<uint8_t, int>> places;
        for (size_t h = 0; h < start.racers.size() && h < field_.session_of.size(); ++h)
            places.emplace_back(start.racers[h].player_id,
                                race().progress(field_.session_of[h]).place);
        net_->record_race(places);
    }
    // AFTER THE RACE (user 2026-09-28): RACE AGAIN for a race on
    // its own and a series, never in a championship, where every race counts; QUIT for a
    // competition. A network race goes back to the lobby, whose host decides what comes next.
    if (!net_ && !setup_.fly_by) {
        const bool champ = app.competition().kind() == Competition::Kind::Championship;
        if (!champ) {
            Setup again = setup_;
            again.skip_intro = true;
            again.replays = std::max(0, setup_.replays - 1);
            rs.auto_replay = setup_.replays > 0;
            rs.replay = [again]() -> std::unique_ptr<Screen> {
                return std::make_unique<RaceScreen>(again);
            };
        }
        if (champ && !app.competition().championship().over()) rs.quit_label = "QUIT CHAMPIONSHIP";
        if (local_series && !ser.over()) rs.quit_label = "QUIT SERIES";
        rs.ok_leaves = local_series && ser.over();
    }
    std::printf("[race] results pushed at tick %u, place %d\n", session_.ticks(), me.place);
    std::fflush(stdout);
    app.push(std::make_unique<ResultsScreen>(std::move(rs)));
}

} // namespace bigred
