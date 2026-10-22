# OpenRedRacing

A clean-room reimplementation of *Big Red Racing* (Big Red Software / Domark, 1996), in the spirit of OpenRCT2 / OpenLoco / OpenRA. A new portable C++ engine (CMake, SDL2, OpenGL 3.3) that plays the game from **your own original files**, unmodified.

This repository contains **no game assets and no original executables**.

## Features

- **All 24 circuits**: terrain, scenery, sky, distance lighting, animated decor, billboards
- **Original handling**: the vehicle physics, ported to a deterministic 28 ms fixed-step integer simulation
- **Full races**: checkpoints, laps, lap times, ranking, respawn, wrong-way / missed-checkpoint warnings
- **Opponents**: the original recorded ghosts (`.REC`), with rubber-banding and a random grid slot
- **Collisions**: drivable ramps, bridges and tunnels, ~8,000 solid scenery objects, car-to-car impacts
- **Destructible scenery** (1,393 objects) and **particles** (dust, snow, water spray, sparks)
- **Sound and music**: engine, skids, impacts, race cues, the OGG soundtrack with per-screen tracks
- **Menus and HUD**: circuit globe, vehicle choice with a 3D preview, team colour, logo and driver, minimap
- **Multiplayer**: local split-screen up to 4 players, LAN up to 6 (lockstep, tested across two PCs)
- **Damage effects**: smoke, fire, explosion and wreck
- **Spoken commentary**
- **Free resolution**, with the original `+` / `-` keys kept (320x200 / 640x480 / window); frame-rate independent
- **Gamepad support**
- **Three chase-camera views** and the **cockpit view** with its dials

## Not finished yet

- **Championship**: the seven rounds are implemented, not yet fully tried in-game
- **Handling defects**: pitch recovery, cornering inertia, the `tilter`
- **Rendering polish**: near-field lighting, tunnel interiors, a few textures that should be transparent
- **Menus**: layout of the translations, trembling letters (`WIBS`)

## (Maybe) later

Depending on the success of the project.

- **Multiplayer up to 12** players
- A **legacy render mode**: pixel-accurate, palette-based, like the 1996 original
- **Driving AI** for single-player opponents
- **Rollback** netcode to hide latency
- A **circuit editor**
- A **WebAssembly demo** running on the official demo's files
- **Procedural circuits**, generated from a theme and a seed
- Other game mode
- Export/import for car and map
- Modding

## Building and running

### What you need

- **Windows 10 or 11**, 64-bit. Windows is the only platform built and tested so far.
- **A MinGW-w64 GCC toolchain** with C++20 support (tested with GCC 16 from
  [WinLibs](https://winlibs.com/), UCRT, POSIX threads), plus **CMake 3.21+** and **Ninja**.
  WinLibs ships all three: `winget install BrechtSanders.WinLibs.POSIX.UCRT`. MSVC is not supported.
- **Your own copy of Big Red Racing**: the installed game folder, the one holding `ALWAYS.WAP` and
  the `.LEV` / `.VHC` files. For the music, put the soundtrack as `.ogg` files (`02.ogg`, `03.ogg`...)
  in a `MUSIC` folder inside it.
- An internet connection for the first configure, which downloads SDL2 2.30.9, RmlUi 6.3 and
  ENet 1.3.18 (pinned and hash-checked) into the build folder.

### Build

```
git clone https://github.com/nynjardin/OpenRedRacing.git
cd OpenRedRacing
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

The programs land in `build/src/`, with `SDL2.dll` next to them. To build offline, point
`SDL2_PREBUILT_DIR`, `RMLUI_DIR` and `ENET_DIR` at unpacked copies of those releases.

`cmake --build build --target dist` gathers in `build/dist/` what to copy into the game folder to
play: `openbigred.exe` and `SDL2.dll`.

### Where the game files are found

Every program takes the game folder as its first argument. Without one, it looks for `ALWAYS.WAP`
next to the executable and in its parent folders, in the working directory and its parents, in a
`BIGRED` subfolder of each of those, and in `C:\BIGRED`. You can also bake your own folder into the
build with `-DOPENBIGRED_GAME_DIR="D:/Games/BIGRED"`. A game folder argument of `-` means "look
for it", which lets you pass the later arguments alone.

### The game: `openbigred`

```
openbigred                          # the menus, as in the original
openbigred C:\BIGRED                # the same, with the game folder named
openbigred - DIRT1 JEEP             # straight into one race: a level and a vehicle
openbigred - DIRT1 JEEP -PLAYERS=2  # the same race in split screen
openbigred - -BORDERLESS -SIZE=1920x1080  # desktop-fullscreen; restores to a 1920x1080 window
```

Arrow keys to drive, `Ctrl` and `Alt` for the vehicle's actions; gamepads work too. `+` / `-`
change the resolution (320x200, 640x480, the window's own), `-RES=320|640|native` picks the
starting one, and `-MUTE` silences everything. `Alt+Enter` switches between the desktop-sized
borderless view and a resizable window. `-BORDERLESS` and `-WINDOWED` choose the startup mode;
`-SIZE=WIDTHxHEIGHT` chooses the window size. Sizes from 320x200 through 16384x16384 are
accepted. Windowed mode keeps any aspect ratio the display supports.

The chase view is the original game's camera. `-CAMERA=COMFORT` swaps in an independent
comfort rig instead: damped horizontal and vertical follow, a stable horizon, gentle
travel-heading alignment and separate object framing, so small hover movements and turning
on the spot do not drag the eye around, and reversing keeps the current orbit side rather
than swinging through 180 degrees. `C` still cycles the three chase distances and the cabin
view; cabin, trackside and scripted fly-by views keep their own camera behaviour.

### The level viewer: `map_preview`

```
map_preview [game_folder] [LEVEL]   # default CHOPPER1
map_preview - MOON2
```

A free camera over any circuit: hold the **right mouse button** to look around, **arrow keys** to
move, the **mouse wheel** to change speed. **Page Up / Page Down** step through every level,
**C** shows the checkpoint ring, **Esc** quits. `-SHOT=picture.bmp` saves a picture after two
seconds and quits.

### The vehicle viewer: `vehicle_preview`

```
vehicle_preview [game_folder] [VEHICLE] [MODEL]   # default JEEP JEEP1
vehicle_preview C:\BIGRED BUGGY BUGGY1
vehicle_preview --dump C:\BIGRED JEEP             # no window: what every face of a .VHC is made of
```

### Checks

`ctest --test-dir build` runs the yes/no checks (they need the game files).

On Windows x64, `tools/check_window_modes.ps1` checks native window bounds, borderless
startup and an Alt+Enter round trip without global keyboard injection:

```powershell
./tools/check_window_modes.ps1 -Executable ./build/src/openbigred.exe -GameDirectory C:/BIGRED -OutputDirectory ./build/window-probe -IncludeMenu
```

It briefly shows its own test windows and writes results under the output directory.

## Documentation

Coming Soon™

## License

The OpenBigRed source code is released under the [MIT License](LICENSE). *Big Red Racing* itself
and its game files remain the property of their owners and are not included.

## Credits

- [jacobgelling/red-archive](https://github.com/jacobgelling/red-archive): `.ENV` archive format and compression
- [jacobgelling/red-image](https://github.com/jacobgelling/red-image): `.COL` / `.MPH` / `.RAW` / `.TM` image formats
- [stb_vorbis](https://github.com/nothings/stb), [SDL2](https://www.libsdl.org/), [RmlUi](https://github.com/mikke89/RmlUi), [ENet](http://enet.bespin.org/)
