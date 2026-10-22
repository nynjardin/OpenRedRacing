// The application shell: the window, the GL context, the event pump, and the screen stack.
//
// It owns everything that must outlive a level - which is the whole reason it exists. See
// screen.h for the history. In the layering, `app/` is above `scene/` + `render/` and below
// nothing. It knows about screens; it knows nothing about racing.
//
// The stack, not a single current screen, because the menus of the original are a graph that
// goes BACK: the track screen is entered from the globe and returns to it, and `mbox 0` with
// its invisible 2x2 rectangle in the corner is that return. A stack gives that for free.
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "audio/mixer.h"
#include "audio/music.h"
#include "audio/sound_bank.h"
#include "local_players.h"
#include "menu_cursor.h"
#include "player_profile.h"
#include "ui/rml_ui.h"
#include "presentation.h"
#include "screen.h"

struct SDL_Window;
using SDL_GLContext = void*;
struct _SDL_GameController;
using SDL_GameController = _SDL_GameController;

namespace bigred {

class Competition;
class NetGame;

// ⚠️ NAMED FOR WHERE THE BUTTON IS, NOT FOR WHAT IS WRITTEN ON IT. The bottom face button
// is "A" on an Xbox pad, "cross" on a PlayStation one and "B" on a Nintendo one; SDL's
// GameController layer already normalises every pad to the Xbox POSITIONS, so the game speaks
// positions too and no screen has to know which pad is plugged in. South validates, East goes
// back - the arrangement every console has used for twenty years.
enum class PadButton { South, East, West, North, DpadLeft, DpadRight, DpadUp, DpadDown };

// The analogue inputs a screen asks for. Sticks read -32768..32767, triggers 0..32767.
enum class PadAxis { LeftX, LeftY, TriggerLeft, TriggerRight };

class App {
public:
    struct Options {
        int width = 1280;
        int height = 720;
        bool vsync = true;
        std::string title = "OpenBigRed";
        // The rung of the resolution ladder to open on. The player moves it with + and -
        // afterwards; this is only the starting point. See presentation.h.
        int display_mode = kDefaultDisplayMode;
        // A window nobody sees - `openbigred --shots`, which reads the menus back from their
        // offscreen canvas and never needs the window itself (game/menu_shots.h).
        bool hidden = false;
        // Use the monitor's full desktop rather than a decorated window at width x height.
        // App still remembers the windowed size and position so Alt+Enter can restore it.
        bool borderless = false;
        // No sound and no music, for this run only - the profile's volumes are not touched.
        // Every hidden test run (-SHOT) is silent: it plays behind whatever the user is doing.
        bool mute = false;
        // -CAMERA=COMFORT: the damped comfort rig instead of the original chase camera.
        bool comfort_camera = false;
    };

    // Both out of line: `net_` holds a type only app.cpp sees in full.
    App();
    ~App();

    // Opens the window and creates the GL context. ⚠️ NOTHING IS LOADED HERE: a screen loads
    // what it needs in enter(). That is the invariant the old main() broke.
    bool init(const Options& opts);

    // Runs until the stack empties or quit() is called. Takes ownership of the first screen.
    int run(std::unique_ptr<Screen> first);

    // --- for screens ----------------------------------------------------------------------
    void push(std::unique_ptr<Screen> s);
    // Pops the current screen at the end of the frame - never mid-draw, so a screen may call
    // this from its own handle_event without destroying itself under its feet.
    void pop();
    void quit() { running_ = false; }

    // --- the audio device, opened once for the whole process -------------------------------
    // ⚠️ IT IS THE APP'S, NOT THE RACE'S, and that is the point: the music plays in the menus,
    // keeps playing while a level loads, and does not restart between two races. The race used
    // to own the Mixer and it died with the screen. A machine with no sound card gets a Mixer
    // that answers `ready() == false` to everything, which every caller already handles.
    Mixer& mixer() { return mixer_; }
    MusicPlayer& music() { return music_; }

    // The interface runtime. Owned here and initialised lazily by the first menu, for the same
    // reason the audio device is: it must outlive any one screen. See ui/rml_ui.h.
    RmlUi& ui() { return ui_; }

    int width() const { return width_; }
    bool comfort_camera() const { return comfort_camera_; }
    int height() const { return height_; }
    SDL_Window* window() const { return window_; }
    Presentation& presentation() { return presentation_; }

    // The mouse in CANVAS coordinates (320x200), or false when it is in the letterbox. Only
    // meaningful for a screen with uses_canvas(). See presentation.h.
    bool mouse_canvas(int mx, int my, float& cx, float& cy) const;

    // The game directory every screen loads from. Held here because it is process-wide and a
    // screen should not have to be handed it through three constructors.
    const std::string& game_dir() const { return game_dir_; }
    void set_game_dir(std::string dir) { game_dir_ = std::move(dir); }

    // True once at least one gamepad has produced an SDL_CONTROLLERDEVICEADDED - i.e. there is
    // one to navigate with. RmlUiScreen uses this only to decide whether to print a hint; the
    // input handling itself does not need to ask, since a closed controller produces no events.
    bool has_gamepad() const { return !controllers_.empty(); }

    // ⚠️ OWNED HERE, LIKE THE AUDIO DEVICE: several screens outlive one another and all need the
    // same pseudo/colours/logo/settings (game/profile_screen.h edits it, game/options_screen.h
    // edits its display/volume fields, a future multiplayer screen reads pseudo/colour/logo to
    // announce the player). Loaded once in init() - see there for what "loaded" applies.
    PlayerProfile& profile() { return profile_; }
    // THE PEOPLE AT THIS MACHINE, for the split screen (app/local_players.h). Empty - or one
    // seat - is an ordinary one-player game; the main menu empties it on its way back.
    LocalPlayers& locals() { return locals_; }
    // --- the gamepad, polled ---------------------------------------------------------------
    // EVERY OPENED PAD MERGED: a button is down if it is down on any of them, and an axis is
    // whichever pad pushes it furthest. A player with two pads plugged in can drive with
    // either, and no screen has to care which one moved - the same reason hotplug lives here.
    //
    // These are for state a screen samples every frame (a trigger held down). A one-shot press
    // is better read as an SDL_CONTROLLERBUTTONDOWN event, which cannot be missed between two
    // frames the way a poll can.
    bool pad_down(PadButton b) const;
    int pad_axis(PadAxis a) const;
    // ONE pad, by its rank among the pads plugged in (0 = the first). The split screen gives
    // each player their own; out of range reads as a pad nobody touches.
    int pad_count() const { return static_cast<int>(controllers_.size()); }
    bool pad_down(PadButton b, int pad) const;
    int pad_axis(PadAxis a, int pad) const;
    // The rank of the pad an SDL controller event came from (`cbutton.which`), or -1.
    int pad_index_of(int instance_id) const;
    // Past the point where a trigger or a stick counts as pressed. One place, so the race and
    // the menus agree on what "pushed" means.
    static constexpr int kPadTriggerOn = 8000;    // of 32767 - a light pull already counts
    static constexpr int kPadStickOn = 12000;     // a deliberate push, not a resting thumb

    // Writes the CURRENT in-memory profile to disk. A screen calls this after changing a field
    // it owns rather than the whole profile being saved on a timer or at exit.
    bool save_profile_now() { return save_profile(profile_); }

    // --- the network game ------------------------------------------------------------------
    // ⚠️ OWNED HERE, AND SERVICED BY THE LOOP EVERY FRAME, whatever screen is up. A network game
    // outlives the lobby that opened it: the vehicle screen, the race and the results come and
    // go on top, and the connection must stay alive under all of them. Created on first use;
    // `net().active()` says whether a game is open. See net/net_game.h.
    NetGame& net();
    bool has_net() const { return net_ != nullptr; }

    // --- what is being played -----------------------------------------------------------------
    // The competition of this session - a race on its own, or a championship - and what comes
    // next (game/competition.h). Created on first use; lives until the process ends.
    Competition& competition();

    // --- tools ----------------------------------------------------------------------------
    // Called once per frame, after the top screen has drawn and before the pushes and pops of
    // the frame are applied - so the hook sees the finished picture and may push/pop itself.
    // Only `openbigred --shots` sets one (game/menu_shots.h).
    void set_frame_hook(std::function<void(App&)> hook) { frame_hook_ = std::move(hook); }
    // Resizes the window and takes the new size at once, without waiting for the resize event
    // a hidden window may never deliver.
    void resize_window(int w, int h);
    // THE SOUND OF A MENU CHOICE, as the original plays it: `BEEP2`, or - with the COMMENTARY
    // setting on - one of 36 silly samples of SAMPLES/ drawn at random.
    void menu_click();
    // The screen on top, or null.
    Screen* top() const { return stack_.empty() ? nullptr : stack_.back().get(); }

private:
    void shutdown();
    void open_audio();
    void apply_pending();
    // SDL only raises SDL_CONTROLLERBUTTONDOWN/AXISMOTION for an OPENED controller - a
    // connected-but-unopened one is silent. Handled by the App, not any screen: hotplug is
    // process-wide, and a screen should not have to care which of several pads moved.
    void handle_controller_device_event(const SDL_Event& e);
    bool set_borderless(bool borderless);
    void remember_windowed_bounds();

    SDL_Window* window_ = nullptr;
    SDL_GLContext gl_context_ = nullptr;
    Presentation presentation_;
    MenuCursor cursor_;   // the game's own animated pointer over the menus

    Mixer mixer_;
    MusicPlayer music_;
    SoundBank front_;   // SAMPLES/, for the menus' own sounds
    RmlUi ui_;
    std::vector<SDL_GameController*> controllers_;
    PlayerProfile profile_;
    LocalPlayers locals_;
    std::unique_ptr<NetGame> net_;
    std::unique_ptr<Competition> competition_;

    std::vector<std::unique_ptr<Screen>> stack_;
    std::vector<std::unique_ptr<Screen>> pending_push_;
    int pending_pops_ = 0;

    int width_ = 0, height_ = 0;
    bool comfort_camera_ = false;
    int windowed_x_ = 0, windowed_y_ = 0;
    int windowed_width_ = 0, windowed_height_ = 0;
    bool have_windowed_bounds_ = false;
    bool borderless_ = false;
    bool hidden_ = false;
    bool running_ = false;
    std::function<void(App&)> frame_hook_;
    std::string game_dir_;
};

} // namespace bigred
