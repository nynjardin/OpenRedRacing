// map_preview - the standalone level viewer.
//
// Loads a real `.LEV` and shows it: terrain, decor, backdrop, distance lighting, with a free
// camera and the live calibration keys. This is a TOOL, not the game - it deliberately keeps the
// debugging affordances (level switching, height-scale tuning, camera files, placeholder markers
// for unresolved shapes) that the game executable must not have. The game is `openbigred`.
//
// Everything about the world itself lives in scene/level_scene.{h,cpp}, shared with the game, so
// the two can never drift apart visually.
//
// The original game's assets are never committed to this repo (see README.md) - this reads them
// from the original install directory, overridable via argv[1]. argv[2] picks which level to load
// (default CHOPPER1). --verbose/-v (any position) turns on the per-instance "[diag]" traces.
// -DEBUG (any position) starts with the CHECKPOINT RING drawn over the world; C toggles it
// while running. That ring is MAP.CSE, the table the game measures every lap and every placing
// against, and it is invisible in the game itself - see scene/gate_overlay.h.

#include <SDL.h>
#include <SDL_opengl.h>

#include "formats/bigred_archive.h"
#include "formats/level_script.h"
#include "formats/scenery.h"
#include "formats/shading.h"
#include "formats/terrain.h"
#include "formats/threedw.h"
#include "render/gl_loader.h"
#include "render/mat4.h"
#include "scene/gate_overlay.h"
#include "scene/level_scene.h"
#include "scene/world_renderer.h"
#include "sim/race.h"
#include "sim/vehicle.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

using bigred::g_verbose_logs;
using bigred::kRenderScale;

int main(int argc, char* argv[]) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0); // line-buffered so console/log output (level
                                               // load stats, live +/- and 'Y' key feedback)
                                               // shows up immediately, not just at exit
    // --verbose/-v may appear anywhere among the arguments; everything else is positional
    // (game_dir, then LEVEL_NAME), same as before this flag existed.
    // --diag=NAME dumps the local and world bounding box of every instance whose MAP.SHL name
    // contains NAME. This is how you settle "do these two instances overlap?" from the real data
    // instead of guessing at a screenshot - as the starting-grid z-fighting was settled.
    std::vector<std::string> positional_args;
    std::string diag_shape_filter;
    // -DEBUG draws the checkpoint ring from the first frame. Spelled the way the user asks for
    // it on the command line, and accepted in either case and with either dash count.
    bool show_gates = false;
    // -SHOT=FILE.bmp: the window after -SHOTAT=S seconds (2 by default), then quit - to look at
    // the viewer's picture without a hand on it (debug_camera.txt places the camera).
    std::string shot;
    int shot_s = 2;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        std::string upper = a;
        for (char& ch : upper) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
        if (a == "--verbose" || a == "-v") {
            g_verbose_logs = true;
        } else if (upper == "-DEBUG" || upper == "--DEBUG") {
            show_gates = true;
        } else if (upper.rfind("-SHOT=", 0) == 0) {
            shot = a.substr(6);
        } else if (upper.rfind("-SHOTAT=", 0) == 0) {
            shot_s = std::max(1, std::atoi(a.c_str() + 8));
        } else if (a.rfind("--diag=", 0) == 0) {
            diag_shape_filter = a.substr(7);
        } else {
            positional_args.push_back(std::move(a));
        }
    }
    // `-` means "the default install", the same convention openbigred and physics_bench
    // use: it is how the LEVEL argument is reached without retyping a path whose accent
    // the shell mangles.
    std::string game_dir = (positional_args.size() > 0 && positional_args[0] != "-")
                               ? positional_args[0]
                               : bigred::find_game_dir();

    std::string level = (positional_args.size() > 1) ? positional_args[1] : "CHOPPER1";
    // How far the world is drawn, in terrain cells - the original's view distance. See shading.h
    // for why this is a choice here and was a moving target there.
    int shade_view_cells = bigred::kShadeViewDistanceCellsDefault;
    // How much of the ramp to actually walk, and how dark a colour has to be to be walked at
    // all. Both are OUR values, not the original's - it walks every index all the way. See the
    // fragment shader for why.
    float shade_strength = bigred::kShadeStrengthDefault;
    float shade_dark_ceil = bigred::kShadeDarkCeilDefault;
    // All `.LEV` files in the install directory, sorted - lets the live level-switch key (see
    // PageUp/PageDown in the render loop) cycle through every level without needing them
    // hardcoded here. Falls back to just the requested level if the directory scan finds
    // nothing (e.g. an unusual install layout), so startup never depends on this succeeding.
    std::vector<std::string> available_levels;
    try {
        for (const auto& entry : std::filesystem::directory_iterator(game_dir)) {
            if (!entry.is_regular_file()) continue;
            std::filesystem::path p = entry.path();
            if (p.extension() == ".LEV" || p.extension() == ".lev") {
                available_levels.push_back(p.stem().string());
            }
        }
        std::sort(available_levels.begin(), available_levels.end());
    } catch (const std::exception&) {
        available_levels.clear();
    }
    if (available_levels.empty()) available_levels.push_back(level);
    int level_index = 0;
    for (size_t i = 0; i < available_levels.size(); ++i) {
        if (available_levels[i] == level) { level_index = static_cast<int>(i); break; }
    }

    // The level itself. Everything that used to be two dozen locals and four lambdas here now
    // lives in LevelScene, shared with the game executable. The references below keep the render
    // loop reading the same way it always has.
    bigred::LevelScene scene;
    scene.load_shade_table(game_dir);
    bigred::g_diag_shape_filter = diag_shape_filter;

    auto& vbuf = scene.vbuf;
    auto& map_w = scene.map_w;
    auto& map_h = scene.map_h;
    auto& height_scale = scene.height_scale;
    auto& decor_absolute_y = scene.decor_absolute_y;
    const bigred::Blob& shade_table = scene.shade_table;

    auto load_level = [&](const std::string& name) { return scene.load(game_dir, name); };
    auto rebuild_vbuf = [&](float h) { return scene.build_world_mesh(h); };
    auto rebuild_anim_bufs = [&]() { scene.rebuild_anim_bufs(); };


    if (!load_level(level)) return 1;

    if (!shade_table.empty()) {
        std::printf("[shading] radius %d cells (%.0f world units) | strength %.1f | applies "
                    "below luminance %.2f   (F/G radius, H/J strength, B/N threshold)\n",
                    shade_view_cells, bigred::shade_radius_world(shade_view_cells),
                    shade_strength, shade_dark_ceil);
    }

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    SDL_Window* window = SDL_CreateWindow(
        "OpenBigRed",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1280, 720,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE
    );
    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_GLContext glContext = SDL_GL_CreateContext(window);
    if (!glContext) {
        std::fprintf(stderr, "SDL_GL_CreateContext failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    SDL_GL_SetSwapInterval(1);

    if (!gl::load()) {
        std::fprintf(stderr, "Failed to load required OpenGL 3.3 functions.\n");
        SDL_GL_DeleteContext(glContext);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // THE WORLD, drawn by the renderer the game uses too (scene/world_renderer.h): the same
    // buffers, the same order, the same start lights and horizon objects. This file keeps only
    // what is the viewer's own - the free camera, the calibration keys and the checkpoint ring.
    bigred::WorldRenderer renderer;
    if (!renderer.create()) {
        SDL_GL_DeleteContext(glContext);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    renderer.upload(scene);

    // The CHECKPOINT RING (-DEBUG / C). `MAP.CSE`'s gates are what the engine measures a lap,
    // a placing and a respawn against, and the game draws nothing of them - so when a
    // checkpoint does not beep where it should, there is nothing to look at. This is that
    // something: GL_LINES, rebuilt only on a level change, handed to the renderer each frame.
    // See scene/gate_overlay.h for what the colours mean.
    std::vector<bigred::TerrainVertex> gate_lines;
    auto upload_gate_overlay = [&]() {
        gate_lines.clear();
        std::vector<bigred::Gate> gates;
        auto cse = scene.lev.find("MAP.CSE");
        if (cse != scene.lev.end())
            gates = bigred::parse_gates(cse->second.data(), cse->second.size());
        if (gates.empty()) {
            std::printf("[gates] %s.LEV has no MAP.CSE - no ring to draw\n", level.c_str());
            return;
        }
        std::string level_txt;
        auto txt = scene.lev.find(level + ".TXT");
        if (txt != scene.lev.end()) level_txt.assign(txt->second.begin(), txt->second.end());
        bigred::mark_unsafe_gates(gates, bigred::parse_unsafe_gates(level_txt));
        // The `CHECKP` lines - what actually fires a checkpoint - and the gates they are
        // attributed to.
        std::vector<bigred::CheckLine> check_lines = bigred::build_check_lines(
            scene.instances, scene.shape_names_by_slot,
            [&](const std::string& shape) -> const bigred::Blob* {
                auto it = scene.lev.find(shape + ".FT");
                return it == scene.lev.end() ? nullptr : &it->second;
            });
        bigred::mark_checkpoint_gates(gates, check_lines);

        bigred::SimTerrain ter;   // heights only - that is all the overlay asks of it
        ter.mph = scene.mph_data;
        ter.mpc = scene.mpc_data;
        ter.cht = (scene.cht_size >= 512) ? scene.cht_data : nullptr;
        ter.w = map_w;
        ter.h = map_h;

        gate_lines = bigred::build_gate_overlay(gates, ter, map_w, kRenderScale);
        const std::vector<bigred::TerrainVertex> masts =
            bigred::build_check_line_overlay(check_lines, ter, map_w, kRenderScale);
        gate_lines.insert(gate_lines.end(), masts.begin(), masts.end());
        int sounding = 0;
        for (const bigred::Gate& g : gates) if (g.checkpoint) ++sounding;
        std::printf("[gates] %s : %zu porte(s), %zu ligne(s) CHECKP, %d porte(s) de reprise "
                    "(vert = ligne d'arrivee, orange = porte d'une ligne, violet = unsafe, "
                    "bleu = ordinaire, jaune = LIGNE CHECKP, une barre par numero d'ordre)\n",
                    level.c_str(), gates.size(), check_lines.size(), sounding);
        std::fflush(stdout);
    };
    upload_gate_overlay();

    glEnable(GL_DEPTH_TEST);

    // Free-fly camera. Right-click-drag
    // (relative mouse mode) to look around, arrow keys to move, mouse wheel to change speed.
    // Mouse is only captured while the right button is held; released, it's free in the host
    // desktop as normal.
    float center_x = (map_w * bigred::kTerrainTexelWorldSize * 0.5f) * kRenderScale;
    float center_z = (map_h * bigred::kTerrainTexelWorldSize * 0.5f) * kRenderScale;
    float map_span = (map_w * bigred::kTerrainTexelWorldSize) * kRenderScale;
    float start_radius = map_span * 0.35f;
    float start_height = map_span * 0.25f;

    Mat4::Vec3 cam_pos{0, 0, 0};
    float cam_pitch = 0, cam_yaw = 0, move_speed = map_span * 0.15f;
    const float kMinSpeed = map_span * 0.01f, kMaxSpeed = map_span * 2.0f;
    const float kMouseSensitivity = 0.0025f;
    bool mouselook = false;

    // Resets the camera to the same default orbit-start view used at startup - called again
    // after a live level switch (PageUp/PageDown below), since the old position/facing made
    // sense for the previous map's layout but not necessarily this one.
    auto reset_camera = [&]() {
        cam_pos = {center_x + start_radius, start_height, center_z};
        Mat4::Vec3 start_dir = Mat4::normalize({-start_radius, -start_height, 0.0f});
        cam_pitch = std::asin(start_dir.y);
        cam_yaw = std::atan2(start_dir.x, start_dir.z);
        move_speed = map_span * 0.15f;
    };
    reset_camera();

    // Debug camera override (dev tool, user-requested) - avoids a rebuild for every camera
    // position tried while visually verifying decor placement. Looks for a plain text file
    // next to the exe, one line, one of:
    //   RAW <raw_x> <raw_y> <raw_z> [distance]
    //     Aims at a raw MAP.DAT-space point (the exact numbers a quick data-analysis script
    //     prints), converted through the same mirror+scale transform place_scenery uses, then
    //     backs the camera off along (1,1,1) by `distance` render units (default 30) looking
    //     back at the target - matches the manual "compute wx/wy/wz by hand" recipe used
    //     throughout this project's screenshot-based verification, just without the arithmetic.
    //   RENDER <camX> <camY> <camZ> <dirX> <dirY> <dirZ>
    //     Sets the camera directly in render-space (dir need not be normalized) - for
    //     replicating an exact previously-good shot.
    // Silently does nothing if the file is missing or malformed (normal case for regular play -
    // this is not a shipped feature, just a local dev convenience).
    {
        std::ifstream cam_file("debug_camera.txt");
        std::string mode;
        if (cam_file >> mode) {
            if (mode == "RAW") {
                float rx, ry, rz, dist = 30.0f;
                if (cam_file >> rx >> ry >> rz) {
                    cam_file >> dist; // optional, keeps default 30 if absent/fails
                    float tx = (static_cast<float>(map_w - 1) * bigred::kTerrainTexelWorldSize - rx) * kRenderScale;
                    float ty = -ry * kRenderScale;
                    float tz = rz * kRenderScale;
                    float off = dist / std::sqrt(3.0f);
                    cam_pos = {tx + off, ty + off, tz + off};
                    Mat4::Vec3 d = Mat4::normalize({-off, -off, -off});
                    cam_pitch = std::asin(d.y);
                    cam_yaw = std::atan2(d.x, d.z);
                    std::printf("[debug_camera.txt] RAW %.0f %.0f %.0f dist=%.0f -> render target (%.1f,%.1f,%.1f)\n",
                                rx, ry, rz, dist, tx, ty, tz);
                }
            } else if (mode == "RENDER") {
                float cx, cy, cz, dx, dy, dz;
                if (cam_file >> cx >> cy >> cz >> dx >> dy >> dz) {
                    cam_pos = {cx, cy, cz};
                    Mat4::Vec3 d = Mat4::normalize({dx, dy, dz});
                    cam_pitch = std::asin(d.y);
                    cam_yaw = std::atan2(d.x, d.z);
                    std::printf("[debug_camera.txt] RENDER cam=(%.1f,%.1f,%.1f) dir=(%.2f,%.2f,%.2f)\n",
                                cx, cy, cz, d.x, d.y, d.z);
                }
            }
            std::fflush(stdout);
        }
    }

    int width = 1280, height = 720;
    bool running = true;
    Uint32 last_ticks = SDL_GetTicks();
    const Uint32 shot_at = last_ticks + static_cast<Uint32>(shot_s) * 1000u;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_RESIZED) {
                width = event.window.data1;
                height = event.window.data2;
                glViewport(0, 0, width, height);
            } else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) {
                running = false;
            } else if (event.type == SDL_KEYDOWN &&
                       (event.key.keysym.sym == SDLK_h || event.key.keysym.sym == SDLK_j ||
                        event.key.keysym.sym == SDLK_b || event.key.keysym.sym == SDLK_n)) {
                // The two taste knobs of the distance lighting - neither exists in the original,
                // both are here because the faithful version washes the near field out. H/J is
                // how far up the ramp a colour is allowed to travel (0 = lighting off entirely);
                // B/N is how dark it has to be to travel at all. See shading.h.
                if (event.key.keysym.sym == SDLK_h) shade_strength -= 0.1f;
                if (event.key.keysym.sym == SDLK_j) shade_strength += 0.1f;
                if (event.key.keysym.sym == SDLK_b) shade_dark_ceil -= 0.02f;
                if (event.key.keysym.sym == SDLK_n) shade_dark_ceil += 0.02f;
                shade_strength = std::max(0.0f, std::min(1.0f, shade_strength));
                shade_dark_ceil = std::max(0.0f, std::min(1.0f, shade_dark_ceil));
                std::printf("[shading] strength = %.1f, applies below luminance %.2f\n",
                            shade_strength, shade_dark_ceil);
                std::fflush(stdout);
            } else if (event.type == SDL_KEYDOWN &&
                       (event.key.keysym.sym == SDLK_f || event.key.keysym.sym == SDLK_g)) {
                // Live view-distance / lighting-range calibration. In the original this number
                // was retuned every frame against the frame time (10..22 cells),
                // so there is no single authored value to recover - it has to be judged on
                // screen. F shortens the range (everything falls back to its raw palette colour
                // sooner), G lengthens it. See shading.h.
                shade_view_cells += (event.key.keysym.sym == SDLK_g) ? 1 : -1;
                shade_view_cells = std::max(3, std::min(40, shade_view_cells));
                std::printf("[shading] radius = %d cells (%.0f world units)\n",
                            shade_view_cells, bigred::shade_radius_world(shade_view_cells));
                std::fflush(stdout);
            } else if (event.type == SDL_KEYDOWN &&
                       (event.key.keysym.sym == SDLK_EQUALS || event.key.keysym.sym == SDLK_KP_PLUS ||
                        event.key.keysym.sym == SDLK_MINUS || event.key.keysym.sym == SDLK_KP_MINUS)) {
                // Live kHeightWorldScale calibration - see terrain.h's header comment, the real
                // value is still unconfirmed (entangled in a per-frame camera-relative rasterizer
                // setup in the original, not a plain constant).
                bool increase = (event.key.keysym.sym == SDLK_EQUALS || event.key.keysym.sym == SDLK_KP_PLUS);
                height_scale += increase ? 2.0f : -2.0f;
                height_scale = std::max(0.0f, std::min(200.0f, height_scale));
                vbuf = rebuild_vbuf(height_scale);
                renderer.upload_meshes(scene);
                std::printf("height_scale = %.1f\n", height_scale);
                std::fflush(stdout);
            } else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_c) {
                // The checkpoint ring on/off - the same view -DEBUG starts with.
                show_gates = !show_gates;
                std::printf("checkpoint ring %s\n", show_gates ? "ON" : "off");
                std::fflush(stdout);
            } else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_y) {
                // Live A/B test (session 22, user-requested): does decor ground-snap onto
                // MAP.MPH at render time, or sit at MAP.DAT's own fixed (x,-y,z) like plain
                // config data? See place_scenery's comment for the full context.
                decor_absolute_y = !decor_absolute_y;
                vbuf = rebuild_vbuf(height_scale);
                rebuild_anim_bufs();
                renderer.upload_meshes(scene);
                std::printf("decor_absolute_y = %s\n", decor_absolute_y ? "true (MAP.DAT fixed Y)" : "false (MAP.MPH ground-snap)");
                std::fflush(stdout);
            } else if (event.type == SDL_KEYDOWN &&
                       (event.key.keysym.sym == SDLK_PAGEUP || event.key.keysym.sym == SDLK_PAGEDOWN)) {
                // Live level switch (user-requested), cycling through every `.LEV` found in
                // game_dir at startup (see available_levels above) - lets you compare multiple
                // maps (e.g. MOON1/2/3) without relaunching the exe. Wraps at both ends (modulo
                // below) - and, since an incomplete install can lack data that
                // `load_level` needs, keeps stepping
                // in the requested direction past any level that fails to load instead of
                // getting stuck retrying it forever. Capped at one full lap so a
                // completely-broken install can't spin forever.
                int step = (event.key.keysym.sym == SDLK_PAGEUP) ? 1 : -1;
                int n = static_cast<int>(available_levels.size());
                bool switched = false;
                for (int tries = 0; tries < n; ++tries) {
                    int next_index = (level_index + step * (tries + 1) + n * n) % n;
                    const std::string& next_level = available_levels[static_cast<size_t>(next_index)];
                    if (load_level(next_level)) {
                        level_index = next_index;
                        level = next_level;
                        switched = true;
                        break;
                    }
                    std::fprintf(stderr, "  (skipping %s.LEV - failed to load)\n", next_level.c_str());
                }
                if (switched) {
                    vbuf = rebuild_vbuf(height_scale);
                    renderer.upload(scene);
                    upload_gate_overlay();
                    reset_camera();
                    std::printf("Switched to %s.LEV (%zu/%zu, PageUp/PageDown to cycle)\n",
                                level.c_str(), static_cast<size_t>(level_index) + 1, available_levels.size());
                } else {
                    std::fprintf(stderr, "No loadable level found in either direction - staying on %s.LEV\n",
                                 level.c_str());
                    load_level(level); // restore state (was cleared by the failed attempts above)
                }
                std::fflush(stdout);
            } else if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_RIGHT) {
                mouselook = true;
                SDL_SetRelativeMouseMode(SDL_TRUE);
            } else if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_RIGHT) {
                mouselook = false;
                SDL_SetRelativeMouseMode(SDL_FALSE);
            } else if (event.type == SDL_MOUSEMOTION && mouselook) {
                cam_yaw -= static_cast<float>(event.motion.xrel) * kMouseSensitivity;
                cam_pitch -= static_cast<float>(event.motion.yrel) * kMouseSensitivity;
                cam_pitch = std::max(-1.55f, std::min(1.55f, cam_pitch));
            } else if (event.type == SDL_MOUSEWHEEL) {
                move_speed *= std::pow(1.15f, static_cast<float>(event.wheel.y));
                move_speed = std::max(kMinSpeed, std::min(kMaxSpeed, move_speed));
            }
        }

        Uint32 now_ticks = SDL_GetTicks();
        float dt = static_cast<float>(now_ticks - last_ticks) / 1000.0f;
        last_ticks = now_ticks;

        const Uint8* keys = SDL_GetKeyboardState(nullptr);
        // Up/Down move along the camera's actual look axis (pitch included); Left/Right strafe
        // sideways, perpendicular to that axis (same cross-product convention as Mat4::lookAt's
        // own screen-right vector, so it matches what's visually left/right on screen).
        Mat4::Vec3 look_dir{std::sin(cam_yaw) * std::cos(cam_pitch), std::sin(cam_pitch),
                             std::cos(cam_yaw) * std::cos(cam_pitch)};
        Mat4::Vec3 right_dir = Mat4::normalize(Mat4::cross(look_dir, {0.0f, 1.0f, 0.0f}));
        if (keys[SDL_SCANCODE_UP]) {
            cam_pos.x += look_dir.x * move_speed * dt;
            cam_pos.y += look_dir.y * move_speed * dt;
            cam_pos.z += look_dir.z * move_speed * dt;
        }
        if (keys[SDL_SCANCODE_DOWN]) {
            cam_pos.x -= look_dir.x * move_speed * dt;
            cam_pos.y -= look_dir.y * move_speed * dt;
            cam_pos.z -= look_dir.z * move_speed * dt;
        }
        if (keys[SDL_SCANCODE_RIGHT]) {
            cam_pos.x += right_dir.x * move_speed * dt;
            cam_pos.z += right_dir.z * move_speed * dt;
        }
        if (keys[SDL_SCANCODE_LEFT]) {
            cam_pos.x -= right_dir.x * move_speed * dt;
            cam_pos.z -= right_dir.z * move_speed * dt;
        }

        Mat4 model_m = Mat4::identity();

        Mat4::Vec3 target{cam_pos.x + look_dir.x, cam_pos.y + look_dir.y, cam_pos.z + look_dir.z};
        Mat4 view = Mat4::lookAt(cam_pos, target, {0.0f, 1.0f, 0.0f});
        Mat4 proj = Mat4::perspective(60.0f * 3.14159265f / 180.0f,
                                       static_cast<float>(width) / static_cast<float>(height),
                                       bigred::kNearPlaneWorld * bigred::kRenderScale, map_span * 4.0f);
        Mat4 mvp = proj * view * model_m;

        // The world, as the game draws it. The viewer has no simulation: the destruction state
        // is none (everything intact) and the start gantry shows its red state, as it does on
        // the grid before the countdown.
        bigred::WorldFrame frame;
        frame.view.mvp = mvp;
        frame.view.eye = cam_pos;
        frame.view.forward = look_dir;
        frame.view.right = right_dir;
        frame.view.up = Mat4::normalize(Mat4::cross(right_dir, look_dir));
        frame.view.aspect = static_cast<float>(width) / static_cast<float>(height);
        frame.wall_ms = SDL_GetTicks();
        frame.shade_view_cells = shade_view_cells;
        frame.shade_strength = shade_strength;
        frame.shade_dark_ceil = shade_dark_ceil;
        if (show_gates) frame.lines = &gate_lines;
        renderer.draw(scene, frame);

        if (!shot.empty() && SDL_GetTicks() >= shot_at) {
            // GL rows run bottom-up; a BMP surface top-down.
            std::vector<uint8_t> px(static_cast<size_t>(width) * height * 4);
            glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
            SDL_Surface* s = SDL_CreateRGBSurfaceWithFormat(0, width, height, 32, SDL_PIXELFORMAT_ABGR8888);
            if (s) {
                for (int y = 0; y < height; ++y)
                    std::memcpy(static_cast<uint8_t*>(s->pixels) + static_cast<size_t>(y) * s->pitch,
                                px.data() + static_cast<size_t>(height - 1 - y) * width * 4,
                                static_cast<size_t>(width) * 4);
                const bool ok = SDL_SaveBMP(s, shot.c_str()) == 0;
                SDL_FreeSurface(s);
                std::printf("%s %s\n", ok ? "Shot written to" : "Shot FAILED:", shot.c_str());
            }
            running = false;
        }

        SDL_GL_SwapWindow(window);
    }

    // Save the final camera position/facing on exit, user-requested ("enregistrer la position
    // de la camera lorsque openbigred est ferme pour que tu puisse ouvrir le meme point de vue")
    // - so a screenshot the user shares from their own free-cam navigation can be reproduced
    // exactly by copying this file to debug_camera.txt (same RENDER format it already reads)
    // instead of guessing coordinates. Always overwrites; harmless if never looked at.
    {
        Mat4::Vec3 look_dir{std::sin(cam_yaw) * std::cos(cam_pitch), std::sin(cam_pitch),
                             std::cos(cam_yaw) * std::cos(cam_pitch)};
        std::ofstream last_cam("last_camera.txt");
        if (last_cam) {
            last_cam << "RENDER " << cam_pos.x << " " << cam_pos.y << " " << cam_pos.z << " "
                      << look_dir.x << " " << look_dir.y << " " << look_dir.z << "\n";
        }
    }

    renderer.release();
    SDL_GL_DeleteContext(glContext);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
