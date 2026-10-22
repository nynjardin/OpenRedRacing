// openbigred - the game.
//
// STATE: one drivable vehicle, validated against the original by the user. The level loads, a
// car from its own `car` list spawns on its `grid` slot, and the confirmed handling model runs
// on a 28 ms fixed-timestep loop. Ramps, bridges and tunnel
// floors are solid. No opponents and no laps yet.
//
// Controls: ARROWS to drive, CTRL for the boost, ALT to get put back on the track, ESC to quit.
//
// What separates it from `map_preview`, which renders the same world:
//   - no free camera, no level switching, no live calibration keys. A game has a viewpoint the
//     simulation decides, not one the operator flies around.
//   - a FIXED-TIMESTEP simulation loop, decoupled from the render rate. The simulation core has
//     to stay deterministic for lockstep networking, which rules
//     out advancing it by a variable frame delta. It is in place from the start rather than
//     retrofitted, because retrofitting determinism is how you never get it.
//   - no placeholder markers for unresolved shapes: the game shows the world, not diagnostics.
//
// Use `map_preview` to look at maps. Use this to play - once there is something to play.

// ⚠️ NEEDED EVEN THOUGH THIS FILE CALLS NOTHING IN SDL. SDL2main provides the real Windows
// entry point and calls `SDL_main`; `SDL.h` is what `#define main SDL_main` comes from. Drop
// this include and the link fails with "undefined reference to SDL_main", which says nothing
// about the cause.
#include <SDL.h>

#include "app/app.h"
#include "app/canvas_check.h"
#include "game/menu_shots.h"
#include "game/split_screen.h"
#include "audio/mixer.h"
#include "audio/music.h"
#include "formats/bigred_archive.h"
#include "formats/level_script.h"
#include "formats/tourn.h"
#include "game/level_catalog.h"
#include "game/lobby_screen.h"
#include "game/main_menu_screen.h"
#include "game/multiplayer_screen.h"
#include "net/net_game.h"
#include "game/profile_screen.h"
#include "game/race_screen.h"

#include <algorithm>
#include <charconv>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr int kMinWindowWidth = 320;
constexpr int kMinWindowHeight = 200;
constexpr int kMaxWindowWidth = 16384;
constexpr int kMaxWindowHeight = 16384;

// `-SIZE=WIDTHxHEIGHT`, already upper-cased by main(). Keep the parse local and strict so a
// launcher mistake fails before SDL creates a window or a game-data scan begins.
bool parse_window_size(const std::string& arg, int& width, int& height) {
    if (arg.rfind("-SIZE=", 0) != 0) return false;
    const char* first = arg.data() + 6;
    const char* last = arg.data() + arg.size();
    const char* cross = std::find(first, last, 'X');
    if (cross == first || cross == last || cross == last - 1) return false;

    int parsed_width = 0;
    int parsed_height = 0;
    auto w = std::from_chars(first, cross, parsed_width);
    auto h = std::from_chars(cross + 1, last, parsed_height);
    if (w.ec != std::errc() || w.ptr != cross ||
        h.ec != std::errc() || h.ptr != last ||
        parsed_width < kMinWindowWidth || parsed_width > kMaxWindowWidth ||
        parsed_height < kMinWindowHeight || parsed_height > kMaxWindowHeight) {
        return false;
    }
    width = parsed_width;
    height = parsed_height;
    return true;
}

// `--list`: print every playable circuit and the cars it allows, one per line, as
//
//     LEVEL<TAB>CAR1,CAR2,...<TAB>PLACE<TAB>WHERE ON THE GLOBE
//
// It exists for the launcher. The scan itself is in game/level_catalog.h, because the MENU
// needs exactly the same answer and the roster must never be written down twice.
//
// ⚠️ EVERY FIELD AFTER THE SECOND IS APPENDED, NEVER INSERTED: the third is what a
// player should read ("JAPAN", not "DIRT1" - see game/level_catalog.h), the fourth is where
// the globe screen puts it. Growing the line only at the end keeps every reader that wants
// the first two fields working, the shipped launcher included - and the fourth field is the
// only windowless way to check that `TOURN.TXT`'s coordinates were read right.
int list_levels(const std::string& game_dir) {
    const std::vector<bigred::LevelEntry> levels = bigred::scan_levels(game_dir);
    for (const bigred::LevelEntry& lvl : levels) {
        std::printf("%s\t", lvl.name.c_str());
        for (size_t i = 0; i < lvl.cars.size(); ++i)
            std::printf("%s%s", i ? "," : "", lvl.cars[i].c_str());
        std::printf("\t%s\t", lvl.title.c_str());
        if (lvl.planet != 0) std::printf("PLANET %d\n", lvl.planet);
        else std::printf("&%04X,&%04X\n", lvl.globe_x, lvl.globe_y);
    }
    return levels.empty() ? 1 : 0;
}

// `--music [game_dir] [seconds]`: the playlist, played with NO WINDOW. A NEGATIVE duration
// means "one second of each track in turn" - the sweep that says every file in the folder
// really decodes.
//
// It exists because "is there music" cannot be answered by a screenshot and should not need the
// whole game to be launched and listened to. It prints what the scan found - which is the half
// that goes wrong, since the naming of a rip is the player's and not ours - and then the frames
// that actually reached the device, which is the difference between "the decoder believes it is
// playing" and "there is sound".
int play_music(const std::string& game_dir, double seconds) {
    bigred::Mixer mixer;
    if (!mixer.open()) return 1;
    bigred::MusicPlayer music;
    if (!music.open(game_dir, mixer.out_rate(), mixer.device())) {
        std::printf("No music: %s\\MUSIC holds no numbered .ogg file.\n", game_dir.c_str());
        return 1;
    }
    mixer.set_music(&music);
    std::printf("%d track(s):\n", music.track_count());
    for (const bigred::MusicTrack& t : music.playlist())
        std::printf("  %2d  %s\n", t.number, t.path.c_str());

    // What the GAME will ask for, and what answers. The numbers are the original CD's, out of
    // TOURN.TXT (formats/tourn.h); a folder that does not carry them gets the fallback, and
    // this is where that shows. It is also the answer to "which file should I name 31.ogg?".
    {
        const bigred::TournCatalog tourn = bigred::load_tourn_catalog(game_dir);
        if (!tourn.empty()) {
            // ⚠️ `has_cd_track`, NOT `resolve_cd_track`: resolving a number the folder does not
            // carry DRAWS one, so printing that would be one sample of a die - a table saying
            // "RIG1 plays 06" when the next race plays 11. What a reader needs to know is
            // whether the folder answers at all.
            const auto answer = [&music](int cd) {
                if (music.track_count() == 0) return std::string("-");
                if (!music.has_cd_track(cd)) return std::string("drawn at random");
                char buf[64];
                std::snprintf(buf, sizeof buf, "%02d", cd);
                return std::string(buf);
            };
            std::printf("\nWhat each screen asks the CD for:\n");
            std::printf("  %-10s front end   track %2d -> %s\n", "(menus)",
                        bigred::kFrontEndMusicTrack, answer(bigred::kFrontEndMusicTrack).c_str());
            for (const bigred::TournTrack& t : tourn.tracks)
                std::printf("  %-10s intro %2d %-9s  music %2d -> %s\n", t.level.c_str(),
                            t.intro_track,
                            music.has_cd_track(t.intro_track) ? "(played)" : "(skipped)",
                            t.music_track, answer(t.music_track).c_str());
            std::printf("\n");
        }
    }

    int failed = 0;
    if (seconds < 0.0) {
        // The sweep. A track that decodes delivers about a second of frames; one that does not
        // delivers nothing at all, and says so by name instead of just going quiet.
        const double kEach = 1.0;
        for (int i = 0; i < music.track_count(); ++i) {
            const uint64_t before = music.frames_delivered();
            music.play(i);
            SDL_Delay(static_cast<uint32_t>(kEach * 1000.0));
            const uint64_t got = music.frames_delivered() - before;
            const double ratio = static_cast<double>(got) / (kEach * mixer.out_rate());
            // Half a second of the previous track is still in the ring when the switch lands,
            // so the bar is "most of a second", not "exactly a second".
            const bool ok = ratio > 0.5;
            if (!ok) ++failed;
            std::printf("  %-2d %-10s %6llu frames (%.0f%%)\n", music.playlist()[i].number,
                        ok ? "decodes" : "SILENT", static_cast<unsigned long long>(got),
                        ratio * 100.0);
        }
        std::printf("%d track(s) silent.\n", failed);
    } else {
        music.play(0);
        const uint32_t end = SDL_GetTicks() + static_cast<uint32_t>(seconds * 1000.0);
        int last = -2;
        while (SDL_GetTicks() < end) {
            const int now = music.current();
            if (now != last) {
                last = now;
                std::printf("  -> playing #%d (%s)\n", now,
                            now >= 0 ? music.playlist()[now].path.c_str() : "stopped");
            }
            SDL_Delay(50);
        }
        // Anything far below the output rate means the decoder is starving the callback, which
        // is the failure a listener would describe as "it stutters".
        const uint64_t delivered = music.frames_delivered();
        std::printf("%llu frames delivered in %.1f s (%.0f/s, device runs at %d).\n",
                    static_cast<unsigned long long>(delivered), seconds,
                    static_cast<double>(delivered) / seconds, mixer.out_rate());
    }
    music.close();
    mixer.set_music(nullptr);
    mixer.close();
    return failed == 0 ? 0 : 1;
}

} // namespace

namespace {

// -ADDLEVEL=SRC:NAME[:PAD] (the part after the `=`, upper case): SRC's files declared, in memory,
// as a circuit NAME with no .LEV of its own; PAD pads the map to 256 x 320. See main().
void declare_added_level(const std::string& game_dir, const std::string& arg) {
    const size_t c1 = arg.find(':');
    const std::string src = arg.substr(0, c1);
    const std::string rest = c1 == std::string::npos ? src + "X" : arg.substr(c1 + 1);
    const size_t c2 = rest.find(':');
    const std::string name = rest.substr(0, c2);
    const bool pad = c2 != std::string::npos && rest.substr(c2 + 1) == "PAD";
    bigred::LevelFiles f;
    f.files = *bigred::level_files(game_dir, src);
    if (auto t = f.files.find(src + ".TXT"); t != f.files.end()) {
        f.files[name + ".TXT"] = t->second;
        f.files.erase(src + ".TXT");
    }
    if (pad) {
        auto mph = f.files.find("MAP.MPH"), mpc = f.files.find("MAP.MPC");
        if (mph != f.files.end() && mpc != f.files.end()) {
            const int w = static_cast<int>(std::sqrt(static_cast<double>(mph->second.size())));
            mph->second.resize(mph->second.size() + static_cast<size_t>(w) * 64, 0);
            mpc->second.resize(mpc->second.size() + static_cast<size_t>(w) * 64, 0);
            f.map_w = w;
            f.map_h = w + 64;
        }
    }
    bigred::LevelEntry e;
    for (const bigred::LevelEntry& l : bigred::scan_levels(game_dir))
        if (l.name == src) e = l;
    e.name = name;
    e.title = name;
    std::printf("-ADDLEVEL: %s declared from %s's files%s.\n", name.c_str(), src.c_str(),
                pad ? ", map padded to 256 x 320" : "");
    bigred::add_level(game_dir, std::move(e), std::move(f));
}

// Where the game's own files are when no directory is named: bigred::find_game_dir(), which
// looks first around the executable - the game is shared as openbigred.exe and SDL2.dll dropped
// into an original install. When no folder holds ALWAYS.WAP, say so in a
// box listing where it looked and stop: carrying on gave a menu without text and the
// development path in the console, which tells a player on another PC nothing.
std::string default_game_dir() {
    std::string tried;
    std::string dir = bigred::find_game_dir(&tried);
    if (!dir.empty()) return dir;
    const std::string msg =
        "Big Red Racing's files were not found (no ALWAYS.WAP).\n\n"
        "Put openbigred.exe and SDL2.dll in the game's folder (the one holding ALWAYS.WAP), "
        "or give that folder as the first argument.\n\nFolders searched:\n" + tried;
    std::fprintf(stderr, "%s", msg.c_str());
    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "OpenBigRed", msg.c_str(), nullptr);
    std::exit(1);
}

} // namespace

int main(int argc, char* argv[]) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    // `--list [game_dir]` prints the circuits and their cars, then exits. Handled before
    // anything else opens a window.
    if (argc > 1 && std::string(argv[1]) == "--list") {
        const std::string dir = (argc > 2 && std::string(argv[2]) != "-")
                                    ? argv[2]
                                    : default_game_dir();
        return list_levels(dir);
    }

    // `--music [game_dir] [seconds]` plays the music folder with no window - see above.
    if (argc > 1 && std::string(argv[1]) == "--music") {
        const std::string dir = (argc > 2 && std::string(argv[2]) != "-")
                                    ? argv[2]
                                    : default_game_dir();
        const double secs = (argc > 3) ? std::atof(argv[3]) : 10.0;
        return play_music(dir, secs);
    }

    // `--canvas [SCREEN] [game_dir]` shows one original menu background through the whole menu
    // path - 320x200 canvas, presented in a centred 4:3 box - so the presentation decision can
    // be judged by eye. See app/canvas_check.h. It loads no level and touches nothing the race
    // uses; it is here rather than in a fourth binary because it needs exactly the game's own
    // window and GL setup, which is now `App`.
    if (argc > 1 && std::string(argv[1]) == "--canvas") {
        const std::string screen = (argc > 2) ? argv[2] : "CONFSCN";
        const std::string dir = (argc > 3 && std::string(argv[3]) != "-")
                                    ? argv[3]
                                    : default_game_dir();
        bigred::App app;
        app.set_game_dir(dir);
        bigred::App::Options opts;
        opts.title = "OpenBigRed - canvas check";
        if (!app.init(opts)) return 1;
        return app.run(std::make_unique<bigred::CanvasCheck>(screen));
    }

    // `--shots [out_dir] [game_dir]` shoots every menu screen at several window sizes, in a
    // hidden window, and reports what overlaps, leaves the canvas or wraps. See
    // game/menu_shots.h. Default out_dir: `menu_shots` in the working directory.
    if (argc > 1 && std::string(argv[1]) == "--shots") {
        const std::string out = (argc > 2 && std::string(argv[2]) != "-") ? argv[2] : "menu_shots";
        const std::string dir = (argc > 3 && std::string(argv[3]) != "-")
                                    ? argv[3]
                                    : default_game_dir();
        bigred::App app;
        app.set_game_dir(dir);
        bigred::App::Options opts;
        opts.title = "OpenBigRed - menu shots";
        opts.mute = true;
        opts.hidden = true;
        if (!app.init(opts)) return 1;
        // `-RES=320` / `-RES=640` after the others: the 3D models of the menus at that rung
        // (app/presentation.h, draw_model). Set after init, which applies the profile's own.
        for (int i = 2; i < argc; ++i) {
            const std::string a = argv[i];
            if (a == "-RES=320") app.presentation().set_mode(0);
            else if (a == "-RES=640") app.presentation().set_mode(1);
            else if (a.rfind("-ADDLEVEL=", 0) == 0) declare_added_level(dir, a.substr(10));
            else if (a.rfind("-LANG=", 0) == 0) bigred::set_language_override(a.substr(6));
        }
        return app.run(std::make_unique<bigred::MenuShots>(out));
    }

    // -DEBUG traces the race to the CONSOLE: every gate crossed, every checkpoint, every lap,
    // and whether the sound that goes with it found its sample. Nothing is drawn - the game
    // shows the world, not its own bookkeeping, and the checkpoint ring is drawn by
    // `map_preview` (see scene/gate_overlay.h). It exists to answer "did the beep not fire, or
    // did it fire and I did not hear it", which is not a question a picture can settle.
    // Accepted anywhere among the arguments, in either case, so the positional convention
    // below is untouched.
    bool trace_race = false;
    // -SOLO races with no opponents: the player alone on the first grid block.
    bool solo = false;
    // -PLACE=N (1..6) puts the player on grid block N instead of the original's draw - for a
    // test that has to start from the same block every time.
    int grid_place = -1;
    // -CABIN starts the race in the cabin view, as `C` three times would.
    bool start_in_cabin = false;
    // Which rung of the resolution ladder to START on. The ladder itself is the player's, on
    // the + and - keys (app/presentation.h); this only picks the first rung, so a launcher can
    // remember a preference and a test does not need a hand on the keyboard.
    int display_mode = bigred::kDefaultDisplayMode;
    bool display_mode_given = false;
    // THE NETWORK GAME WITHOUT A HAND ON THE KEYBOARD, for testing it on one machine:
    //   -NETHOST=N        host, and start once N players are in and ready
    //   -NETJOIN[=ADDR]   join ADDR, or the first game found on the network
    //   -AUTODRIVE        the race drives itself (a fixed pattern of the tick)
    //   -QUITAFTER=T      leave after T ticks, printing the race's checksum
    //   -NAME=X           this instance's pseudo, not saved
    int net_host = 0;
    bool net_join = false;
    // -ADDLEVEL=SRC:NAME[:PAD]: declares SRC's files, in memory, as a circuit NAME that has no
    // .LEV of its own - the way a generated circuit enters (game/level_catalog.h, add_level).
    // PAD adds 64 empty rows to the map, which makes it 256 x 320: a rectangular map. A test
    // of that door, nothing more.
    std::string add_level_arg;
    // -HIDDEN: no window on screen, and muted - what a test run by CTest wants (src/CMakeLists.txt).
    bool hidden = false;
    bool borderless = false;
    int window_width = 1280;
    int window_height = 720;
    std::string bad_window_size;
    std::string net_join_addr;
    bool autodrive = false;
    bool spectate = false;    // -SPECTATE: the trackside cameras from the start
    bool fly_by = false;      // -FLYBY: the track's presentation fly-by (the VIEW button)
    bool mute = false;        // -MUTE: no sound, no music (a -SHOT run always is)
    bool comfort_camera = false; // -CAMERA=COMFORT: the damped rig; CLASSIC (default) the original
    uint32_t quit_after = 0;
    std::string name_override;
    bool net_local = false;   // -NETLOCAL: 127.0.0.1 only, no firewall question
    bool net_ghosts = false;  // -NETGHOSTS: the host fills the grid with the recorded ghosts
    uint32_t end_after = 0;   // -ENDAFTER=T: the race ends by itself at tick T
    bool no_air_cap = false;  // -NOAIRCAP: deliberate deviation 3 off (alone, or hosting)
    bool dev2 = false;        // -DEV2: the retired deviation 2 back on (alone, or hosting)
    int32_t air_cap = 0;      // -AIRCAP=N: deviation 3's cap at N (alone, or hosting)
    int speed_pct = 0;        // -SPEED=PCT: the speed class, 100 = the original (alone, or hosting)
    int air_damp = -1;        // -AIRDAMP=N: airborne damping 1/2^N, 0 = off (alone, or hosting)
    int start_damage = -1;    // -DAMAGE=N: the player starts N % damaged (100 = a wreck), alone only
    uint32_t desync_at = 0;   // -DESYNCAT=T: this machine's own car is nudged at tick T (resync test)
    int races = 1;            // -RACES=N: that many races, then quit
    int replays = 0;          // -REPLAYS=N: the results press RACE AGAIN N times (with -AUTODRIVE)
    int net_loss = 0;         // -NETLOSS=N: this machine drops N % of its input packets (test)
    int players = 1;          // -PLAYERS=N: a split screen of N (2..4), straight into the race
    bool side_by_side = false; // -SIDEBYSIDE: two players side by side (not saved to the profile)
    std::string shot;         // -SHOT=FILE: a PNG of the window after 6 s, then quit
    int shot_s = 6;           // -SHOTAT=S: ... after S seconds instead
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        std::string upper = a;
        for (char& ch : upper) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
        if (upper == "-DEBUG" || upper == "--DEBUG") trace_race = true;
        else if (upper.rfind("-LANG=", 0) == 0) bigred::set_language_override(upper.substr(6));
        else if (upper == "-SOLO" || upper == "--SOLO") solo = true;
        else if (upper.rfind("-PLACE=", 0) == 0) grid_place = std::atoi(upper.c_str() + 7) - 1;
        else if (upper == "-CABIN") start_in_cabin = true;
        else if (upper.rfind("-CAMERA=", 0) == 0) {
            const std::string mode = upper.substr(8);
            if (mode != "COMFORT" && mode != "CLASSIC") {
                std::fprintf(stderr, "Camera must be COMFORT or CLASSIC.\n");
                return 1;
            }
            comfort_camera = mode == "COMFORT";
        }
        else if (upper == "-RES=320" || upper == "-RES=320X200") display_mode = 0, display_mode_given = true;
        else if (upper == "-RES=640" || upper == "-RES=640X480") display_mode = 1, display_mode_given = true;
        else if (upper == "-RES=NATIVE") display_mode = bigred::kDefaultDisplayMode, display_mode_given = true;
        else if (upper.rfind("-NETHOST=", 0) == 0) net_host = std::max(1, std::atoi(upper.c_str() + 9));
        else if (upper == "-NETJOIN") net_join = true;
        else if (upper.rfind("-NETJOIN=", 0) == 0) { net_join = true; net_join_addr = a.substr(9); }
        else if (upper == "-AUTODRIVE") autodrive = true;
        else if (upper == "-NOAIRCAP") no_air_cap = true;
        else if (upper == "-DEV2") dev2 = true;
        else if (upper.rfind("-AIRCAP=", 0) == 0) air_cap = std::atoi(upper.c_str() + 8);
        else if (upper.rfind("-SPEED=", 0) == 0) speed_pct = std::clamp(std::atoi(upper.c_str() + 7), 25, 400);
        else if (upper.rfind("-AIRDAMP=", 0) == 0) air_damp = std::clamp(std::atoi(upper.c_str() + 9), 0, 15);
        else if (upper.rfind("-DAMAGE=", 0) == 0) start_damage = std::clamp(std::atoi(upper.c_str() + 8), 0, 100);
        else if (upper == "-SPECTATE") spectate = true;
        else if (upper == "-FLYBY") fly_by = true;
        else if (upper == "-MUTE") mute = true;
        else if (upper == "-HIDDEN") hidden = true;
        else if (upper == "-BORDERLESS") borderless = true;
        else if (upper == "-WINDOWED") borderless = false;
        else if (upper.rfind("-SIZE=", 0) == 0) {
            if (!parse_window_size(upper, window_width, window_height)) bad_window_size = a;
        }
        else if (upper == "-NETLOCAL") net_local = true;
        else if (upper == "-NETGHOSTS") net_ghosts = true;
        else if (upper.rfind("-ENDAFTER=", 0) == 0) end_after = static_cast<uint32_t>(std::atoi(upper.c_str() + 10));
        else if (upper.rfind("-REPLAYS=", 0) == 0) replays = std::atoi(upper.c_str() + 9);
        else if (upper.rfind("-DESYNCAT=", 0) == 0) desync_at = static_cast<uint32_t>(std::atoi(upper.c_str() + 10));
        else if (upper.rfind("-NETLOSS=", 0) == 0) net_loss = std::clamp(std::atoi(upper.c_str() + 9), 0, 90);
        else if (upper.rfind("-RACES=", 0) == 0) races = std::max(1, std::atoi(upper.c_str() + 7));
        else if (upper.rfind("-QUITAFTER=", 0) == 0) quit_after = static_cast<uint32_t>(std::atoi(upper.c_str() + 11));
        else if (upper.rfind("-NAME=", 0) == 0) name_override = upper.substr(6);
        else if (upper == "-SIDEBYSIDE") side_by_side = true;
        else if (upper.rfind("-PLAYERS=", 0) == 0) players = std::atoi(upper.c_str() + 9);
        else if (upper.rfind("-SHOT=", 0) == 0) shot = a.substr(6);
        else if (upper.rfind("-SHOTAT=", 0) == 0) shot_s = std::max(1, std::atoi(upper.c_str() + 8));
        else if (upper.rfind("-ADDLEVEL=", 0) == 0) add_level_arg = upper.substr(10);
        else args.push_back(std::move(a));
    }
    if (!bad_window_size.empty()) {
        std::fprintf(stderr,
                     "Invalid %s: use -SIZE=WIDTHxHEIGHT with width 320..16384 and "
                     "height 200..16384.\n",
                     bad_window_size.c_str());
        return 2;
    }
    // `-` means "the default install", so the later arguments can be reached without
    // retyping the accented path - the convention physics_bench already uses, and the
    // reason it has it: a shell mangles the accent and the level never loads.
    std::string game_dir = (!args.empty() && args[0] != "-") ? args[0]
                                      : default_game_dir();
    // ⚠️ NO LEVEL NAMED = THE MENU. Naming one still starts that race straight away, which is
    // what the launcher does and what every test in this project does; it is also the only way
    // back to a single-command repro of a handling bug.
    std::string level = (args.size() > 1) ? args[1] : "";
    std::string vehicle_name = (args.size() > 2) ? args[2] : "";
    if (!add_level_arg.empty()) declare_added_level(game_dir, add_level_arg);

    bigred::App app;
    app.set_game_dir(game_dir);
    bigred::App::Options opts;
    opts.width = window_width;
    opts.height = window_height;
    opts.borderless = borderless;
    opts.comfort_camera = comfort_camera;
    opts.display_mode = display_mode;
    opts.hidden = !shot.empty() || hidden;
    opts.mute = mute || !shot.empty() || hidden;
    if (!app.init(opts)) return 1;
    // A -RES named on the command line wins over the profile's rung for this run, as it does
    // for --shots; init() applies the profile's, and nothing here saves it back.
    if (display_mode_given) app.presentation().set_mode(display_mode);
    // -SHOT: the window as it was drawn, six seconds in - the countdown over, everyone on the
    // move - read from the back buffer before it is shown, then the game quits.
    if (!shot.empty()) {
        const uint32_t at = SDL_GetTicks() + static_cast<uint32_t>(shot_s) * 1000;
        app.set_frame_hook([shot, at](bigred::App& a) {
            if (SDL_GetTicks() < at) return;
            std::vector<uint8_t> raw(static_cast<size_t>(a.width()) * a.height() * 4);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glReadPixels(0, 0, a.width(), a.height(), GL_RGBA, GL_UNSIGNED_BYTE, raw.data());
            std::vector<uint8_t> rgba(raw.size());
            const size_t row = static_cast<size_t>(a.width()) * 4;
            for (int y = 0; y < a.height(); ++y)
                std::copy_n(raw.data() + static_cast<size_t>(a.height() - 1 - y) * row, row,
                            rgba.data() + static_cast<size_t>(y) * row);
            bigred::write_png(shot, rgba, a.width(), a.height());
            std::printf("Shot written to %s\n", shot.c_str());
            a.set_frame_hook(nullptr);
            a.quit();
        });
    }

    // ⚠️ NO PROFILE = THE CREATE SCREEN, NOT THE MENU: the original's very first question,
    // piecemeal across NAMESCN/CHARSCN, is here a single pseudo entry before anything else.
    // App::init() already tried to load one; profile().valid() says whether that succeeded.
    if (!name_override.empty()) app.profile().pseudo = name_override;   // never saved
    if (net_loss > 0) app.net().set_input_loss(net_loss);   // menus or -NETHOST/-NETJOIN alike
    // The command line's rule switches, for a race alone and for the races this machine hosts
    // (sim/race_rules.h). A client plays by its host's and ignores its own.
    const auto dev_rules = [&](bigred::RaceRules r) {
        if (no_air_cap) r.air_rate_cap = false;
        if (dev2) r.righting_tolerance = true;
        if (air_cap > 0) r.air_rate_cap_value = air_cap;
        if (air_damp >= 0) r.air_damp_shift = air_damp;
        if (speed_pct > 0) r.speed_percent = speed_pct;
        return r;
    };
    if (net_host > 0 || net_join) {
        bigred::LobbyScreen::Auto au;
        au.on = true;
        au.start_at = net_host > 0 ? net_host : 2;
        au.quit_after = quit_after;
        au.end_after = end_after;
        au.desync_at = desync_at;
        au.races = races;
        (void)autodrive;   // an automatic lobby always drives itself
        if (net_host > 0) {
            bigred::NetSettings s;
            s.level = level.empty() ? std::string("DIRT1") : level;
            s.ghosts = net_ghosts;
            // -RACES=N is also the series the host runs, up to the lobby's own limit.
            s.races = static_cast<uint8_t>(std::clamp(races, 1, bigred::kNetMaxSeries));
            if (!app.net().host(bigred::look_from_profile(app), s, net_local)) return 1;
            app.net().set_rules(dev_rules(bigred::RaceRules::network()));
            return app.run(std::make_unique<bigred::LobbyScreen>(au));
        }
        return app.run(std::make_unique<bigred::JoinScreen>(net_join_addr, au, net_local));
    }
    if (level.empty()) {
        if (!app.profile().valid())
            return app.run(std::make_unique<bigred::ProfileScreen>(bigred::ProfileScreen::Mode::Create));
        return app.run(std::make_unique<bigred::MainMenuScreen>());
    }

    bigred::RaceScreen::Setup setup;
    setup.level = level;
    setup.vehicle = vehicle_name;
    setup.trace = trace_race;
    setup.opponents = !solo;
    setup.grid_place = grid_place;
    setup.start_in_cabin = start_in_cabin;
    setup.autodrive = autodrive;
    setup.spectate = spectate;
    setup.fly_by = fly_by;
    setup.quit_after = quit_after;
    setup.end_after = end_after;
    setup.replays = replays;
    setup.rules = dev_rules(bigred::RaceRules());
    setup.start_damage = start_damage;
    if (players > 1) {
        // Straight into a split-screen race, with the seats the menu would start from.
        app.locals() = bigred::default_local_players(app, std::min(players, bigred::kMaxLocalPlayers));
        for (const bigred::LevelEntry& e : bigred::scan_levels(game_dir)) {
            if (e.name != level || e.cars.empty()) continue;
            int main_vehicle = 0;
            for (size_t i = 0; i < e.cars.size(); ++i)
                if (e.cars[i] == vehicle_name) main_vehicle = static_cast<int>(i);
            bigred::fill_split_setup(app, e, main_vehicle, setup);
        }
        if (side_by_side) setup.split_side_by_side = true;
    }
    return app.run(std::make_unique<bigred::RaceScreen>(std::move(setup)));
}
