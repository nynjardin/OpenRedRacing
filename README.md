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

Coming Soon

## Documentation

Coming Soon™

## Credits

- [jacobgelling/red-archive](https://github.com/jacobgelling/red-archive): `.ENV` archive format and compression
- [jacobgelling/red-image](https://github.com/jacobgelling/red-image): `.COL` / `.MPH` / `.RAW` / `.TM` image formats
- [stb_vorbis](https://github.com/nothings/stb), [SDL2](https://www.libsdl.org/), [RmlUi](https://github.com/mikke89/RmlUi), [ENet](http://enet.bespin.org/)

