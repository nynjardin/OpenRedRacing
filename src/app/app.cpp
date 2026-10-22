#include "app.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <cstdio>
#include <cstdlib>

#include "formats/tourn.h"   // kFrontEndMusicTrack
#include "game/competition.h"
#include "net/net_game.h"
#include "render/gl_loader.h"
#include "ui/team_logos.h"

namespace bigred {

App::App() = default;
App::~App() { shutdown(); }

NetGame& App::net() {
    if (!net_) net_ = std::make_unique<NetGame>();
    return *net_;
}

Competition& App::competition() {
    if (!competition_) competition_ = std::make_unique<Competition>();
    return *competition_;
}

bool App::init(const Options& opts) {
    comfort_camera_ = opts.comfort_camera;
    // ⚠️ GAMECONTROLLER, NOT JOYSTICK: SDL only raises SDL_CONTROLLERBUTTONDOWN/AXISMOTION for a
    // device opened as a game controller, and only a device SDL recognises as one at all - and
    // every menu must work with the pad as well as with the mouse. Its absence is not fatal: a
    // machine with no pad plugged in is the common case, and every screen using the RmlUiScreen
    // base already works with mouse and keyboard alone.
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }
    // Pads already plugged in at launch do not raise SDL_CONTROLLERDEVICEADDED - that event is
    // only for a HOTPLUG. Open what is already there now; run() takes it from here.
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        if (SDL_IsGameController(i)) {
            if (SDL_GameController* c = SDL_GameControllerOpen(i)) controllers_.push_back(c);
        }
    }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    width_ = opts.width;
    height_ = opts.height;
    hidden_ = opts.hidden;
    window_ = SDL_CreateWindow(opts.title.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                               width_, height_,
                               SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE |
                                   (opts.hidden ? SDL_WINDOW_HIDDEN : 0));
    if (!window_) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return false;
    }
    SDL_SetWindowMinimumSize(window_, 320, 200);
    SDL_GetWindowSize(window_, &windowed_width_, &windowed_height_);
    SDL_GetWindowPosition(window_, &windowed_x_, &windowed_y_);
    have_windowed_bounds_ = true;
    // Hidden captures must keep the exact canvas size requested by their tooling. A visible
    // game can open directly into the monitor's desktop, which also covers borderless startup.
    if (opts.borderless && !opts.hidden) set_borderless(true);
    gl_context_ = SDL_GL_CreateContext(window_);
    if (!gl_context_ || !gl::load()) {
        std::fprintf(stderr, "Failed to create a GL 3.3 context.\n");
        shutdown();
        return false;
    }
    SDL_GL_SetSwapInterval(opts.vsync ? 1 : 0);
    // The menu canvas. A failure here is not fatal: a screen that renders at window resolution
    // (the race) does not touch it, and the reason is already on stderr.
    presentation_.init();
    open_audio();
    cursor_.load(game_dir_);
    front_.configure(game_dir_, std::string(), std::string(), std::string());

    // ⚠️ THE PROFILE'S OWN SETTINGS OVERRIDE `opts.display_mode`, not the other way round: a
    // caller-supplied rung only matters the very first time, before a profile exists to
    // remember one (see game/profile_screen.h and app/player_profile.h). A profile that was
    // never saved is a default-constructed one whose fields already equal opts.display_mode's
    // usual default and each system's own built-in volume, so applying it unconditionally below
    // changes nothing on a first run.
    profile_ = profile_exists() ? load_profile() : PlayerProfile();
    // The player's own logo (ui/team_logos.h): one PNG, one place, taken if it is there.
    set_custom_logo_path(profile_dir() + "logo.png");
    presentation_.set_mode(profile_.valid() ? profile_.display_mode : opts.display_mode);
    music_.set_volume(opts.mute ? 0 : profile_.music_volume);
    mixer_.set_master(opts.mute ? 0 : profile_.fx_volume);
    return true;
}

// ⚠️ NEEDS `game_dir_` ALREADY SET - the music lives under it. Both entry points call
// set_game_dir() before init(), which is why this is here and not in the constructor.
//
// Nothing in here is fatal. No device, no `MUSIC/` folder, or an unreadable track: the game
// runs, quieter. That is the decision of 2026-09-21, and it is why there is no message when
// the folder is simply absent.
void App::open_audio() {
    if (!mixer_.open()) return;
    if (music_.open(game_dir_, mixer_.out_rate(), mixer_.device())) {
        mixer_.set_music(&music_);
        std::printf("Music: %d track(s) in MUSIC/.\n", music_.track_count());
        // The front end's own piece, from the first frame - the original plays it before any
        // menu is up (with ten retries). A screen that wants another one asks.
        music_.play_cd_track(kFrontEndMusicTrack);
    }
}

void App::menu_click() {
    // The original menu-click routine's own table (nine bytes a name): every one of them is in
    // SAMPLES/.
    static const char* const kSilly[36] = {
        "laugh", "ooh", "extra6", "baby", "bed", "burp1", "chic", "comeon", "dirty",
        "extra", "extra2", "extra3", "extra4", "extra5", "fart1", "fart2", "fart3", "floor",
        "richard", "spit", "wet", "wus", "xmas", "foot", "gear", "hey", "hurl1",
        "hurl2", "laugh2", "led", "off", "on", "parrot", "pedal", "piss", "pop"};
    if (!mixer_.ready()) return;
    const char* name = "BEEP2";
    if (profile_.commentary) name = kSilly[static_cast<size_t>(std::rand()) % 36];
    const Sample* sample = front_.get("front", name);
    if (!sample) return;
    VoiceParams vp;
    vp.rate = sample->rate;
    vp.volume = 256;
    mixer_.play(*sample, vp, false, 0, -1, false);
}

void App::shutdown() {
    // Screens first, and through exit(), while the context is still current. A GL object freed
    // after SDL_GL_DeleteContext is a silent leak at best.
    while (!stack_.empty()) {
        stack_.back()->exit(*this);
        stack_.pop_back();
    }
    pending_push_.clear();
    pending_pops_ = 0;
    // A network game still open is LEFT, so the others hear it at once instead of after a
    // timeout.
    net_.reset();
    for (SDL_GameController* c : controllers_) SDL_GameControllerClose(c);
    controllers_.clear();
    presentation_.release();
    cursor_.release();
    // The interface runtime, before the context dies: shutting RmlUi down releases every
    // document, and each release calls back into our renderer to free GL objects.
    ui_.shutdown();
    // The decoder thread first, then the pointer it is reached through, then the device: the
    // player takes the device lock from its own thread, so it must be joined while the device
    // still exists.
    music_.close();
    mixer_.set_music(nullptr);
    mixer_.close();
    if (gl_context_) {
        SDL_GL_DeleteContext(gl_context_);
        gl_context_ = nullptr;
    }
    if (window_) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
    SDL_Quit();
}

void App::push(std::unique_ptr<Screen> s) {
    if (s) pending_push_.push_back(std::move(s));
}

void App::pop() { ++pending_pops_; }

// ⚠️ ROUNDS, NOT ONE PASS: a screen's resumed() or enter() may itself ask for a pop and a push -
// "replace me", as the presentation screen does between the races of a split-screen series.
// Those are applied in the next round, pops first, exactly as if they had been asked for from
// update(). One pass used to push the new screen at once and leave the pop for the next frame,
// which then popped the NEW screen: the user was sent back to the first circuit (2026-09-30).
void App::apply_pending() {
    for (int round = 0; round < 16 && (pending_pops_ > 0 || !pending_push_.empty()); ++round) {
        const bool popped = pending_pops_ > 0;
        while (pending_pops_ > 0 && !stack_.empty()) {
            --pending_pops_;
            stack_.back()->exit(*this);
            stack_.pop_back();
        }
        pending_pops_ = 0;
        std::vector<std::unique_ptr<Screen>> pushes = std::move(pending_push_);
        pending_push_.clear();
        // Whatever is underneath is the top again, and it never left - so it gets resumed(),
        // not enter(). See screen.h.
        if (popped && pushes.empty() && !stack_.empty()) stack_.back()->resumed(*this);
        for (auto& s : pushes) {
            const char* n = s->name();
            if (!s->enter(*this)) {
                std::fprintf(stderr, "Screen '%s' failed to enter.\n", n);
                s->exit(*this);
                continue;
            }
            stack_.push_back(std::move(s));
        }
    }
}

void App::handle_controller_device_event(const SDL_Event& e) {
    if (e.type == SDL_CONTROLLERDEVICEADDED) {
        // `which` is a JOYSTICK device INDEX here (it becomes an instance id only after open) -
        // that distinction is exactly what SDL's own docs warn trips people up.
        if (SDL_GameController* c = SDL_GameControllerOpen(e.cdevice.which)) controllers_.push_back(c);
    } else if (e.type == SDL_CONTROLLERDEVICEREMOVED) {
        // Here `which` IS an instance id, unlike the ADDED case above.
        for (size_t i = 0; i < controllers_.size(); ++i) {
            SDL_Joystick* j = SDL_GameControllerGetJoystick(controllers_[i]);
            if (j && SDL_JoystickInstanceID(j) == e.cdevice.which) {
                SDL_GameControllerClose(controllers_[i]);
                controllers_.erase(controllers_.begin() + static_cast<long>(i));
                break;
            }
        }
    }
}

void App::remember_windowed_bounds() {
    if (!window_ || borderless_ || hidden_ ||
        (SDL_GetWindowFlags(window_) & (SDL_WINDOW_MINIMIZED | SDL_WINDOW_FULLSCREEN |
                                        SDL_WINDOW_FULLSCREEN_DESKTOP))) {
        return;
    }
    SDL_GetWindowSize(window_, &windowed_width_, &windowed_height_);
    SDL_GetWindowPosition(window_, &windowed_x_, &windowed_y_);
    have_windowed_bounds_ = true;
}

bool App::set_borderless(bool borderless) {
    if (!window_ || hidden_ || borderless_ == borderless) return true;

    if (borderless) {
        remember_windowed_bounds();
        if (SDL_SetWindowFullscreen(window_, SDL_WINDOW_FULLSCREEN_DESKTOP) != 0) {
            std::fprintf(stderr, "SDL_SetWindowFullscreen failed: %s\n", SDL_GetError());
            return false;
        }
        borderless_ = true;
    } else {
        if (SDL_SetWindowFullscreen(window_, 0) != 0) {
            std::fprintf(stderr, "Leaving fullscreen failed: %s\n", SDL_GetError());
            return false;
        }
        borderless_ = false;
        if (have_windowed_bounds_) {
            SDL_SetWindowSize(window_, windowed_width_, windowed_height_);
            SDL_SetWindowPosition(window_, windowed_x_, windowed_y_);
        }
    }
    SDL_GetWindowSize(window_, &width_, &height_);
    if (gl_context_) {
        SDL_GL_MakeCurrent(window_, gl_context_);
        glViewport(0, 0, width_, height_);
    }
    std::printf("Window mode: %s\n", borderless ? "borderless fullscreen" : "windowed");
    std::fflush(stdout);
    return true;
}

namespace {

SDL_GameControllerButton sdl_button(PadButton b) {
    switch (b) {
    case PadButton::South: return SDL_CONTROLLER_BUTTON_A;
    case PadButton::East: return SDL_CONTROLLER_BUTTON_B;
    case PadButton::West: return SDL_CONTROLLER_BUTTON_X;
    case PadButton::North: return SDL_CONTROLLER_BUTTON_Y;
    case PadButton::DpadLeft: return SDL_CONTROLLER_BUTTON_DPAD_LEFT;
    case PadButton::DpadRight: return SDL_CONTROLLER_BUTTON_DPAD_RIGHT;
    case PadButton::DpadUp: return SDL_CONTROLLER_BUTTON_DPAD_UP;
    case PadButton::DpadDown: return SDL_CONTROLLER_BUTTON_DPAD_DOWN;
    }
    return SDL_CONTROLLER_BUTTON_INVALID;
}

SDL_GameControllerAxis sdl_axis(PadAxis a) {
    switch (a) {
    case PadAxis::LeftX: return SDL_CONTROLLER_AXIS_LEFTX;
    case PadAxis::LeftY: return SDL_CONTROLLER_AXIS_LEFTY;
    case PadAxis::TriggerLeft: return SDL_CONTROLLER_AXIS_TRIGGERLEFT;
    case PadAxis::TriggerRight: return SDL_CONTROLLER_AXIS_TRIGGERRIGHT;
    }
    return SDL_CONTROLLER_AXIS_INVALID;
}

} // namespace

bool App::pad_down(PadButton b) const {
    const SDL_GameControllerButton sb = sdl_button(b);
    for (SDL_GameController* c : controllers_)
        if (c && SDL_GameControllerGetButton(c, sb)) return true;
    return false;
}

int App::pad_axis(PadAxis a) const {
    const SDL_GameControllerAxis sa = sdl_axis(a);
    int best = 0;
    for (SDL_GameController* c : controllers_) {
        if (!c) continue;
        const int v = SDL_GameControllerGetAxis(c, sa);
        if (std::abs(v) > std::abs(best)) best = v;
    }
    return best;
}

void App::resize_window(int w, int h) {
    if (window_) SDL_SetWindowSize(window_, w, h);
    width_ = w;
    height_ = h;
}

bool App::pad_down(PadButton b, int pad) const {
    if (pad < 0 || pad >= pad_count()) return false;
    SDL_GameController* c = controllers_[static_cast<size_t>(pad)];
    return c && SDL_GameControllerGetButton(c, sdl_button(b));
}

int App::pad_axis(PadAxis a, int pad) const {
    if (pad < 0 || pad >= pad_count()) return 0;
    SDL_GameController* c = controllers_[static_cast<size_t>(pad)];
    return c ? SDL_GameControllerGetAxis(c, sdl_axis(a)) : 0;
}

int App::pad_index_of(int instance_id) const {
    for (size_t i = 0; i < controllers_.size(); ++i) {
        SDL_Joystick* j = SDL_GameControllerGetJoystick(controllers_[i]);
        if (j && SDL_JoystickInstanceID(j) == instance_id) return static_cast<int>(i);
    }
    return -1;
}

bool App::mouse_canvas(int mx, int my, float& cx, float& cy) const {
    return Presentation::to_canvas(width_, height_, mx, my, cx, cy);
}

int App::run(std::unique_ptr<Screen> first) {
    push(std::move(first));
    apply_pending();
    if (stack_.empty()) return 1;

    running_ = true;
    uint64_t last = SDL_GetPerformanceCounter();
    const double freq = static_cast<double>(SDL_GetPerformanceFrequency());

    while (running_ && !stack_.empty()) {
        Screen* screen = stack_.back().get();

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                running_ = false;
                break;
            }
            if (e.type == SDL_WINDOWEVENT) {
                if (e.window.event == SDL_WINDOWEVENT_RESIZED ||
                    e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                    width_ = e.window.data1;
                    height_ = e.window.data2;
                    glViewport(0, 0, width_, height_);
                    remember_windowed_bounds();
                } else if (e.window.event == SDL_WINDOWEVENT_MOVED) {
                    remember_windowed_bounds();
                }
            }
            if (e.type == SDL_CONTROLLERDEVICEADDED || e.type == SDL_CONTROLLERDEVICEREMOVED) {
                handle_controller_device_event(e);
                continue;   // process-wide, not a screen's concern
            }
            // ⚠️ THE RESOLUTION KEYS ARE THE APP'S, not any screen's, and they are handled
            // BEFORE the screen sees them: the original switched between 320x200 and 640x480
            // with + and -, and a player who presses them expects it to work wherever they
            // are - in a menu, on the grid, mid-race. Both the main row and the keypad, and
            // SDLK_EQUALS because that is the unshifted key `+` sits on.
            if (e.type == SDL_KEYDOWN && !e.key.repeat) {
                const SDL_Keycode k = e.key.keysym.sym;
                if ((k == SDLK_RETURN || k == SDLK_KP_ENTER) &&
                    (e.key.keysym.mod & KMOD_ALT)) {
                    set_borderless(!borderless_);
                    continue;   // Alt+Enter belongs to the window, not the current screen
                }
                int delta = 0;
                if (k == SDLK_PLUS || k == SDLK_EQUALS || k == SDLK_KP_PLUS) delta = 1;
                if (k == SDLK_MINUS || k == SDLK_KP_MINUS || k == SDLK_UNDERSCORE) delta = -1;
                if (delta != 0) {
                    const int before = presentation_.mode();
                    presentation_.step_mode(delta);
                    if (presentation_.mode() != before) {
                        std::printf("Resolution: %s\n", presentation_.mode_name());
                        std::fflush(stdout);
                    }
                    continue;   // consumed: a screen must not also act on it
                }
            }
            screen->handle_event(*this, e);
            // A screen may have asked to be popped; stop feeding it events this frame.
            if (pending_pops_ > 0 || !pending_push_.empty()) break;
        }
        if (!running_) break;
        if (net_) net_->service(SDL_GetTicks());

        const uint64_t now = SDL_GetPerformanceCounter();
        double dt_ms = static_cast<double>(now - last) * 1000.0 / freq;
        last = now;
        // A breakpoint, a level load or a dragged window can produce an enormous delta. Clamp
        // it: a screen that accumulates a fixed timestep would otherwise try to catch up with
        // hundreds of ticks at once and appear to freeze.
        if (dt_ms > 250.0) dt_ms = 250.0;

        screen->update(*this, dt_ms);
        cursor_.update(screen->uses_canvas(), height_, SDL_GetTicks());

        if (screen->uses_canvas() && presentation_.ready()) {
            // A menu: laid out at the real resolution of its 4:3 box. The chosen rung of the
            // ladder only touches the 3D models it shows (Presentation::draw_model).
            presentation_.begin_canvas(width_, height_);
            screen->draw(*this);
            presentation_.present(width_, height_);
        } else if (presentation_.begin_world(width_, height_)) {
            // The world, really rendered at that many lines and stretched over the window.
            // False at the native rung, which is the branch below - the usual one.
            screen->draw(*this);
            presentation_.present_world(width_, height_);
            screen->draw_overlay(*this);
        } else {
            glViewport(0, 0, width_, height_);
            screen->draw(*this);
            screen->draw_overlay(*this);
        }

        if (frame_hook_) frame_hook_(*this);
        SDL_GL_SwapWindow(window_);
        apply_pending();
    }

    shutdown();
    return 0;
}

} // namespace bigred
