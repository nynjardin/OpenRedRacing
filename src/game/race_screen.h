// The race, as a Screen: one drivable vehicle on the fixed-timestep loop.
//
// This is what `openbigred.cpp`'s main() used to be. Every member below was a local of that
// function; moving them here is what lets a race be STARTED and ENDED inside one process
// instead of being the process. See app/screen.h for why that had to change.
//
// ⚠️ `uses_canvas()` is FALSE. The world and the board render at the player's own resolution,
// full frame - only menus go through the 320x200 canvas.
//
// ⚠️ THE SIMULATION LOOP IS THIS SCREEN'S, not the App's. The App hands out a wall-clock delta
// and nothing more, because only the race has a deterministic tick to keep: 28 ms, an exact
// whole number of times per second of wall clock, identically on every machine, with the
// display interpolating between the last two. Lockstep networking depends on it, and
// retrofitting determinism is how you never get it.
#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "app/local_players.h"
#include "app/player_profile.h"
#include "app/screen.h"
#include "audio/vehicle_audio.h"
#include "game/race_audio.h"
#include "formats/game_files.h"
#include "formats/level_script.h"
#include "formats/sprite.h"
#include "formats/vehicle_script.h"
#include "render/gl_loader.h"
#include "game/net_race_driver.h"
#include "game/race_board.h"
#include "game/race_spectator.h"
#include "render/mat4.h"
#include "render/sky.h"
#include "scene/chase_camera.h"
#include "scene/comfort_camera.h"
#include "scene/dashboard.h"
#include "scene/level_scene.h"
#include "scene/particle_render.h"
#include "scene/race_level.h"
#include "scene/vehicle_render.h"
#include "scene/world_renderer.h"
#include "sim/particles.h"
#include "sim/race.h"
#include "sim/race_session.h"
#include "sim/vehicle.h"

namespace bigred {

class NetGame;

class RaceScreen : public Screen {
public:
    struct Setup {
        std::string level = "DIRT1";
        std::string level_title;   // "JAPAN" - what the results screen shows, not the file name
        std::string vehicle;   // empty = the level's own first `car`
        bool trace = false;    // -DEBUG: the race, to the console
        // The player's own choice (app/player_profile.h), applied after VehicleSim::reset() - reset()
        // re-defaults it, see race_screen.cpp. 0 (SOLIDE) .. 100 (FRAGILE).
        int damage_option = VehicleSim::kDefaultDamageOption;

        // The player's team colour, the one the vehicle screen shows them. False means "the
        // shipped repaint", which is what `openbigred <LEVEL>` gets when no menu was crossed.
        //
        // ⚠️ IT CHANGES WHAT IS DRAWN AND NOTHING ELSE. The simulation never reads it, so it
        // cannot make two machines in a lockstep race disagree.
        bool tinted = false;
        VehicleTint tint;

        // The driver at the wheel (scene/character_model.h), and the colour of their clothes -
        // the head the vehicle carries at its `&40` anchor. -1: nobody known, no head. Drawn
        // only, like the tint.
        int character = -1;
        int character_colour = 0;

        // The player's chosen emblem, `kLogoCell` x `kLogoCell` RGBA, painted into the vehicle
        // sheet's logo cell at load. Empty = the vehicle's own authored emblem. Like the tint,
        // it changes what is DRAWN and nothing the simulation reads.
        std::vector<uint8_t> logo;

        // TRAKSCN's "VIEW": the same level, the same world, NOBODY DRIVING. The camera is flown
        // round the checkpoint ring on rails, there is no board and no vehicle, and Escape
        // comes straight back to the presentation screen. See game/track_screen.h.
        //
        // ⚠️ IT IS THIS SCREEN AND NOT A SECOND ONE, on purpose: a fly-by has to load, build
        // and draw a level EXACTLY as the race does or it is showing the player a different
        // circuit from the one they are about to drive. A copy would drift the first time one
        // of the two was touched.
        bool fly_by = false;

        // The presentation screen has already started the circuit's intro jingle, and pressing
        // RACE has to CUT it rather than restart it (user, 2026-09-23). True means "go straight
        // to the looping music" - see start_music().
        bool skip_intro = false;

        // The OPPONENTS: the level's recorded ghosts (`N.REC`, sim/replay.h). On by default;
        // `-SOLO` on the command line races alone, on the first block.
        bool opponents = true;
        // With them, the grid block the player takes, 0..5, and the ghost recorded for that
        // block stays out. -1 = the original's rule: the level's `spos` line, else a draw
        // (sim/replay.h, StartPlaces). The draw happens HERE, before the session exists, so
        // it is part of the race's setup and never of the simulation.
        int grid_place = -1;
        // Starts every local view in the cabin (-CABIN): what `C` pressed three times gives, for
        // a capture with no hands on the keyboard.
        bool start_in_cabin = false;

        // A NETWORK race: App::net()'s current one. The level, every racer's place and vehicle,
        // the laps and the ghosts all come from the START the host sent (net/net_game.h), and
        // override what is above; only what is DRAWN for the local player - tint, emblem - is
        // still taken from here. The race waits for the host's GO and runs in lockstep.
        bool network = false;

        // TESTS WITHOUT A HAND ON THE KEYBOARD (openbigred's -AUTODRIVE / -QUITAFTER=N): the
        // controls are a fixed pattern of the tick - throttle, steering in bursts - and the
        // game quits after that many ticks, printing the race's checksum. Two instances in a
        // network race print the same line or the race desynchronised.
        bool autodrive = false;
        // -SPECTATE: watch the race through the trackside cameras from the start, as a player
        // who has finished does - to judge the cameras without finishing a race first.
        bool spectate = false;
        uint32_t quit_after = 0;
        // -ENDAFTER=T: the race is declared over at tick T, as if everyone had finished - the
        // whole results-and-back loop tested without having to finish three laps. The results
        // then continue by themselves.
        uint32_t end_after = 0;
        // -REPLAYS=N: with -AUTODRIVE, the results press RACE AGAIN that many times before OK -
        // the after-the-race loop tested without hands.
        int replays = 0;
        // -DESYNCAT=T: THIS machine's own car is pushed 200 units sideways after tick T, which
        // no other machine does - a deliberate desync, to watch the resync repair it.
        uint32_t desync_at = 0;
        // The rules of a race played on this machine (sim/race_rules.h) - the command line's
        // -NOAIRCAP, -DEV2, -AIRCAP, -AIRDAMP. A network race plays by the host's, which come
        // in its start message, and ignores these.
        RaceRules rules;
        // -DAMAGE=N: the player starts N % damaged, 100 = already a wreck. Alone only, a test
        // aid for the smoke, the fire, the explosion and the wreck. -1 = unset.
        int start_damage = -1;

        // SPLIT SCREEN: the other people at this machine, players 2 to 4 (app/local_players.h).
        // Each has a car on the grid next to the first player's, a view of their own and the
        // same kind of look the first player has. Empty: one player, the whole window.
        struct Guest {
            std::string name;
            std::string vehicle;
            int damage_option = VehicleSim::kDefaultDamageOption;
            bool tinted = false;
            VehicleTint tint;
            int character = -1;
            int character_colour = 0;
            std::vector<uint8_t> logo;
        };
        std::vector<Guest> guests;
        // Two players: side by side instead of one above the other (game/split_layout.h).
        bool split_side_by_side = false;
        // What each local player drives with, the first player first. Empty: the first player
        // takes everything (keyboard and every pad), which is the one-player rule.
        std::vector<Controls> controls;
        std::string player_name;   // the first player's, for the split-screen results
    };

    explicit RaceScreen(Setup setup) : setup_(std::move(setup)) {}

    bool enter(App& app) override;
    void exit(App& app) override;
    bool handle_event(App& app, const SDL_Event& e) override;
    void update(App& app, double dt_ms) override;
    void draw(App& app) override;
    void draw_overlay(App& app) override;
    bool uses_canvas() const override { return false; }
    const char* name() const override { return "race"; }

private:
    bool load_world(App& app);
    bool build_gl();
    void start_music(App& app);
    void step_simulation();
    void push_results(App& app);

    // --- the local players' views (Setup::guests) ---------------------------------------------
    // One view per local player, the first player's FIRST: alone there is one, on a split screen
    // one per quarter. Each carries its racer, its controls, its camera and what the display
    // made of it this frame, and is handed to draw_view()/draw_board() for its share of the
    // window - nothing is swapped in and out of the screen's own members.
    struct LocalView {
        int racer = 0;
        Controls controls = Controls::Everything;
        ChaseCamera camera;
        CameraState cam_prev{};
        ComfortCamera comfort_camera;
        bool comfort_cut_pending = true;
        VehicleInput input{};
        Mat4::Vec3 cam_pos{0.0f, 0.0f, 0.0f};
        Mat4::Vec3 look{0.0f, 0.0f, 1.0f};
        Mat4::Vec3 right{1.0f, 0.0f, 0.0f};
        Mat4::Vec3 sky_up{0.0f, 1.0f, 0.0f};
        int watch = -1;   // the racer this view's camera follows - see watch_for()
        // THE CABIN VIEW: `C` steps the three chase heights, then the
        // cabin, then back. `cabin` is the choice; `in_cabin` whether it holds this frame - not
        // when the view is watching someone else, nor once the car is a wreck.
        bool cabin = false;
        bool in_cabin = false;
        // The cabin's LOOK PITCH (BAM16), where the head is at rest `internal[3]`
        // and a crash throws it down: the original's contact code takes `vfwd >> 4` off it on
        // every blocking contact of the car it watches in the cabin, and its camera update brings
        // it back by at most 0x40 a tick - the dip "that comes back up in about a second" (user, 2026-09-29).
        // Stepped on the tick, the pair interpolated on the frame.
        int32_t cabin_look = 0, cabin_look_prev = 0;
        bool cabin_look_set = false;
        // The eye, `alpha` of the way from camera `prev` to camera `now`.
        void place(const CameraState& prev, const CameraState& now, float alpha, int map_w);
        void place(const CameraView& view);
    };
    std::vector<LocalView> views_;
    bool split() const { return views_.size() > 1; }
    // SPLIT SCREEN: whom a view follows. Its own car while it races; once it is home or
    // wrecked, another local player still racing, else any car still racing - the quarter
    // would otherwise sit on a parked car until the last one finishes.
    int watch_for(int own) const;
    // Steps one view's camera onto whatever it follows, re-planting it on a change of target.
    void follow(LocalView& v);
    // Everything one set of controls presses now (app/local_players.h).
    VehicleInput read_controls(App& app, Controls c) const;
    // The first player's keys, from the profile (Options -> Controls).
    std::array<int, kDriveKeyCount> keys_ = default_drive_keys();
    // 0 for the first player, 1..3 for a guest, -1 for anyone else.
    int local_seat_of(int racer) const;
    // One view: the world, in whatever viewport is current, `width` x `height` window pixels.
    void draw_view(const LocalView& v, int width, int height);
    // One view's HUD, over the world - see draw_overlay. Fills a BoardFrame and hands it to
    // board_ (game/race_board.h).
    void draw_board(const LocalView& v, int width, int height);
    // Every racer's mini-map dot, in drawing order - see there. `own`: the view's racer, drawn
    // last; -1 for the race map of a three-way split, where nobody is "the view's own".
    std::vector<BoardDot> board_dots(int own) const;
    Mat4 view_mvp(const LocalView& v, int width, int height) const;
    // Every split view's share of the window, the viewport and the scissor set in turn for
    // `each(view_index, rect)`; the viewport is restored after.
    template <typename F> void for_each_split_view(App& app, F each);
    // The scripted camera of `fly_by`: the checkpoint ring, flown at a constant speed at a
    // fixed height over the ground. Built once in enter(), advanced in update().
    void build_flyby_path();
    void step_flyby(double dt_ms);

    Setup setup_;
    // ⚠️ ONE-SHOT: race().race_over() STAYS true once every racer has finished, so update() must
    // not push a second results screen every frame after the first.
    bool results_pushed_ = false;
    // The tick after the one the race was seen to be over, 0 before; the results wait
    // kResultsDelayTicks from there.
    uint32_t over_tick_ = 0;
    // THE FADE TO BLACK before a screen changes, instead of a cut (user, 2026-09-27). At the
    // end of a race it is counted in TICKS over the last kFadeTicks of the results delay, so
    // the results still come up on the same tick on every machine of a network game; at the
    // end of a fly-by, which is never networked, on the wall clock.
    static constexpr uint32_t kFadeTicks = 600 / kPhysicsTickMs;
    static constexpr double kFadeMs = 600.0;
    double fly_fade_from_ms_ = -1.0;   // wall_ms_ when the fly-by's fade started, -1 = none
    float fade_ = 0.0f;                // 0 = the picture, 1 = black; drawn over everything
    static constexpr uint32_t kResultsDelayTicks = 5000 / kPhysicsTickMs;
    std::string vehicle_name_;

    // --- the world and the simulation ------------------------------------------------------
    LevelScene scene_;
    ArchivePtr ingame_, env_;   // never null once the world is loaded
    // The level as the simulation sees it, and the racers - built by the functions the benches
    // use (scene/race_level.h), so the game and the network game build the same race.
    RaceLevel level_;
    RaceField field_;
    const VehicleTuning& tuning() const { return field_.tunings[static_cast<size_t>(player_)]; }
    SceneryState world_;
    // The simulated race: every car, the ring and the laps - see sim/race_session.h.
    RaceSession session_;
    // The LOCAL player's racer. 0 when racing alone; the last grid block with opponents.
    int player_ = 0;
    VehicleSim& car() { return session_.car(player_); }
    const VehicleSim& car() const { return session_.car(player_); }
    RaceState& race() { return session_.race(); }
    const RaceState& race() const { return session_.race(); }
    RespawnPoint start_{};
    ParticleField particles_;
    std::vector<SpotEffect> spots_;

    // --- sound: every car, the race's own events and the commentator (game/race_audio.h) ------
    // ⚠️ THE MIXER IS THE APP'S, borrowed for the length of the race. It used to be a member
    // here and it was opened and closed with the screen, which meant the device - and with it
    // the music - died between two races.
    Mixer* mixer_ = nullptr;
    RaceAudio audio_;

    // --- wrecks ---------------------------------------------------------------------------
    // Which colour a wreck is announced by (RED GREEN YELLOW BLUE ORANGE PURPLE, 0..5).
    std::vector<int> wreck_colour_;
    void note_wrecks();
    // Out of the race: over the line, or wrecked long enough ago. What every "is everyone
    // home?" of this screen asks - a wreck will never cross the line and must not be waited for.
    bool out_of_race(int racer) const;

    // --- what gets drawn --------------------------------------------------------------------
    VehicleModel car_model_;
    std::vector<VehicleModel> ghost_models_;     // per racer; empty for the player's slot
    ParticleVisuals particle_visuals_;
    // The board and the window's overlays (game/race_board.h).
    RaceBoard board_;
    float map_span_ = 0.0f;

    // --- GL: the world is drawn by the renderer every executable shares (scene/world_renderer.h),
    // created in build_gl() and released in exit(). This screen only hands it the cars and the
    // effects of each frame.
    WorldRenderer renderer_;
    // Per-frame scratch, kept as members so the allocation is made once and not every frame.
    std::vector<TerrainVertex> fx_scratch_, car_scratch_, cabin_scratch_, cabin_parts_scratch_;
    // Each local player's cabin: the interior model, its dashboard and the atlas layer the
    // dashboard is uploaded to. By racer; empty for everyone who is not a local player.
    struct Cabin {
        VehicleModel model;
        Dashboard dash;
        int layer = -1;
    };
    std::vector<Cabin> cabins_;
    // The level's sun (`light` line), towards it, game axes - what the vehicles are lit by.
    float sun_[3] = {0.0f, -1.0f, 0.0f};
    // The light one vehicle is drawn under from this view: the sun, at the intensity its
    // distance from the camera gives it.
    VehicleLighting light_for(const LocalView& v, const VehiclePose& pose) const;
    // `C`: the next of the three chase heights, or the cabin after the last one.
    void cycle_view(LocalView& v);
    // Puts a view in the cabin for this frame, or leaves it on the chase camera.
    void place_cabin(LocalView& v, bool spectating);

    // --- the tick, and what the display interpolates between --------------------------------
    // The fly-by's rail: the middle of every gate of the ring, in game world units, closed
    // back onto its first point. Empty in an ordinary race.
    struct FlyPoint {
        int32_t x = 0, y = 0, z = 0;   // +Y DOWN, like everything in the simulation frame
    };
    std::vector<FlyPoint> fly_path_;
    // THE ORIGINAL'S FLY-BY, when the level has one (every shipped level does): the player IS
    // racer 6, the presentation helicopter, played back from `6.REC` from the `heli` point
    // (the original makes racer 6 the player, sets its fly-by flag, then keeps racer 6
    // alone). The ordinary chase camera follows it. The rail above is only the fallback.
    bool fly_heli_ = false;
    ReplayTrack fly_track_;
    RespawnPoint fly_start_;
    double fly_distance_ = 0.0;        // how far along the rail, in world units
    double fly_total_ = 0.0;           // the rail's whole length, so one lap can end it


    // --- the end of a network race -----------------------------------------------------------
    // The FINISH COUNTDOWN is the session's (RaceSession::finish_deadline); this only makes
    // sure it is written to the console once.
    bool finish_announced_ = false;
    // SPECTATING: a player out of the race watches the others (game/race_spectator.h).
    RaceSpectator spectator_;
    // Whether this view spectates now, and whom it may follow - a race rule, so decided here.
    void update_spectator();
    // The first view's camera pair - the spectator's while there is one.
    const CameraState& view_now() const {
        return spectator_.active() ? spectator_.now() : views_[0].camera.now;
    }
    const CameraState& view_prev() const {
        return spectator_.active() ? spectator_.prev() : views_[0].cam_prev;
    }
    // Every car's pose at the last two ticks, for the display to interpolate between; the
    // player's own car included, at player_.
    std::vector<VehiclePose> pose_prev_, pose_cur_;
    // Where racer `racer` is drawn this frame: between its last two ticks at alpha_, plus what
    // is left of a resync's offset.
    VehiclePose shown_pose(int racer) const;
    std::vector<VehicleInput> inputs_;           // one per racer, the player's at player_
    unsigned long long sim_ticks_ = 0;
    double accumulator_ = 0.0;
    // Wall-clock milliseconds the loop has been handed, so leaving the race can say whether the
    // tick rate actually held. A fixed timestep that silently runs fast is a game running fast,
    // and it is not visible from inside a frame.
    double wall_ms_ = 0.0;
    float alpha_ = 0.0f;   // how far into the tick that has not happened yet

    // --- the network race (Setup::network) --------------------------------------------------
    // Borrowed from the App for the length of the race; null in a local one.
    NetGame* net_ = nullptr;
    // The lockstep loop and the resync (game/net_race_driver.h).
    NetRaceDriver net_race_;
    // The series as it stood when this race started (load_world): points by player id, races done.
    std::map<uint8_t, int> net_points_before_;
    int net_series_done_before_ = 0;
    // Where a car is drawn: its pose plus what is left of a resync's offset (none in a local race).
    VehiclePose drawn_pose(int racer, const VehiclePose& p) const {
        return net_race_.drawn_pose(racer, p);
    }
    double quit_reached_ms_ = -1.0;       // -QUITAFTER: when the last tick was reached
    // Every human still in the race has finished (a departed one no longer counts).
    bool humans_finished() const;
    // A network race's state, as one line under the board and its colour; empty when all is well.
    void network_status(std::string& line, float rgb[3]) const;
    // Leaving a race with others in it is asked first (handle_event); the question is drawn by
    // the board, and this is the answer.
    void leave_race(App& app);
    bool confirm_quit_ = false;
};

} // namespace bigred
