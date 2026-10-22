# Three binaries, one shared world.
#
#   openbigred      - the game. One drivable vehicle on a 28 ms fixed-timestep loop.
#   map_preview     - the standalone level viewer: free camera, live calibration keys.
#   vehicle_preview - single-model debug viewer.
#
# Everything about the world itself lives in scene/level_scene.{h,cpp} + render/world_shaders.h,
# so the game and the viewer can never drift apart visually.

# A reference initialised with itself is an error, not a silent read of garbage: on 2026-09-28
# a scripted rename turned RaceBoard's `car = f.session->car(...)` into `car = car`, and the
# board showed a frozen 297 km/h (and a wrong damage, boost and wreck state) with no warning,
# since no warning is on in this project.
add_compile_options(-Werror=init-self -Wuninitialized)

# The application shell: window, GL context, screen stack, and the 4:3 canvas the menus draw
# into. Only the game links it - map_preview and vehicle_preview are single-purpose tools that
# own their own window, and physics_bench has none at all.
set(OPENBIGRED_APP_SOURCES
    app/app.cpp
    # The game's own animated mouse pointer over the menus. See app/menu_cursor.h.
    app/menu_cursor.cpp
    app/local_players.cpp
    app/canvas_check.cpp
    app/player_profile.cpp
    app/presentation.cpp
    app/rml_screen.cpp
    game/car_screen.cpp
    game/championship_screen.cpp
    game/character_screen.cpp
    game/globe_screen.cpp
    game/level_catalog.cpp
    game/lobby_screen.cpp
    game/main_menu_screen.cpp
    # `openbigred --shots`: every menu, read back and checked for overlaps. See its header.
    game/menu_shots.cpp
    # The multiplayer fork and the network menus: create, join, the list of LAN games.
    game/multiplayer_screen.cpp
    game/options_screen.cpp
    game/net_race_driver.cpp
    game/competition.cpp
    game/placeholder_screen.cpp
    game/profile_screen.cpp
    game/race_audio.cpp
    game/race_board.cpp
    game/race_screen.cpp
    game/race_spectator.cpp
    game/results_screen.cpp
    # MULTIPLAYER > SPLIT SCREEN, and the guests it hands the race. See its header.
    game/split_screen.cpp
    game/track_screen.cpp
    # ⚠️ menu_screen.{h,cpp} became quick_race_screen.{h,cpp} on 2026-09-23 and was RETIRED the
    # same day: the flat list of circuits it drew is now the globe (globe_screen) plus the
    # presentation screen (track_screen) plus the vehicle screen (car_screen), which is the
    # split the original itself has.
    ui/background.cpp
    # The globe of GLOBESCN: a real textured sphere. It belongs with the interface rather than
    # with the world - only a menu ever shows it. See ui/globe.h.
    ui/globe.cpp
    # The vehicle turning on the car screen. Same rule as the globe: only a menu ever shows it,
    # and it draws the RACE's own model rather than a second one. See ui/vehicle_display.h.
    # The game's own thirteen team logos, out of LOGOSCN.SCN. Shared by whatever screen offers
    # the choice - today the vehicle screen. See ui/team_logos.h.
    ui/team_logos.cpp
    # PNG decoding for the player's own logo (third_party/README.md).
    third_party/lodepng.cpp
    ui/vehicle_display.cpp
    # The interface layer: RmlUi turns documents into triangles, these draw them with the
    # game's own GL and the game's own alphabet.
    ui/rml_font_engine.cpp
    ui/rml_render_interface.cpp
    ui/rml_system_interface.cpp
    ui/rml_ui.cpp
    # The original's menu screens rebuilt on RmlUi: picture, sprites, wibbly text at the places
    # its .SCN and TOURN.<LANG> give them. See ui/scn_kit.h.
    ui/scn_kit.cpp
)

set(OPENBIGRED_WORLD_SOURCES
    audio/mixer.cpp
    # The music. In the WORLD list rather than the APP one because the mixer itself calls into
    # it, and map_preview/physics_bench link the mixer. ⚠️ It compiles stb_vorbis (see
    # third_party/README.md), so it is the one slow file of the set.
    audio/music.cpp
    audio/sound_bank.cpp
    audio/vehicle_audio.cpp
    formats/bigred_archive.cpp
    formats/game_files.cpp
    formats/terrain_members.cpp
    formats/threedw.cpp
    formats/terrain.cpp
    formats/texture.cpp
    formats/shading.cpp
    formats/sprite.cpp
    formats/scenery.cpp
    formats/level_script.cpp
    formats/localised_text.cpp
    formats/tourn.cpp
    # The championship's rule, apart from its screen so physics_bench can play it.
    game/championship.cpp
    formats/vehicle_script.cpp
    # The six drivers of CHARSCN.SCN: the menu shows one, a vehicle carries its head.
    scene/character_model.cpp
    scene/chase_camera.cpp
    scene/comfort_camera.cpp
    scene/camera_obstruction.cpp
    # The cabin view's dashboard: the `ndial` gauges drawn into the `dash` sheet.
    scene/dashboard.cpp
    scene/gate_overlay.cpp
    scene/level_scene.cpp
    scene/particle_render.cpp
    # The level as the SIMULATION sees it, and the field of a race - shared by the game, the
    # network game and both benches so that they all build the same race. See race_level.h.
    scene/race_level.cpp
    scene/vehicle_render.cpp
    # The world, drawn - shared by the game and map_preview (code review A1, 2026-09-28).
    scene/world_renderer.cpp
    sim/fixed.cpp
    sim/particles.cpp
    sim/lockstep.cpp
    sim/race.cpp
    sim/race_session.cpp
    sim/replay.cpp
    sim/snapshot.cpp
    sim/vehicle.cpp
    render/gl_loader.cpp
    render/hud.cpp
    render/shader.cpp
)

# The network game: the wire format, the LAN discovery, the lobby and the lockstep race over
# ENet. The game and net_bench link it; nothing else opens a socket.
set(OPENBIGRED_NET_SOURCES
    net/discovery.cpp
    net/net_game.cpp
    net/net_protocol.cpp
)

add_executable(openbigred
    openbigred.cpp
    ${OPENBIGRED_NET_SOURCES}
    ${OPENBIGRED_WORLD_SOURCES}
    ${OPENBIGRED_APP_SOURCES}
)
# ⚠️ RMLUI_STATIC_LIB must be defined by any target that links RmlUi statically, or its
# headers declare the symbols dllimport and the link fails. Only the game links it: the two
# viewers and the bench have no interface.
target_link_libraries(openbigred PRIVATE RmlUi::Core enet)
target_compile_definitions(openbigred PRIVATE RMLUI_STATIC_LIB)

add_executable(map_preview
    map_preview.cpp
    ${OPENBIGRED_WORLD_SOURCES}
)

# The circuit generator's bench, with a window: a THEME and a SEED, drawn by the world
# renderer the game uses. Nothing generated reaches the game until it is judged here.
add_executable(track_gen
    track_gen.cpp
    gen/track_generator.cpp
    gen/ground_tiles.cpp
    gen/autotile.cpp
    gen/hill_paint.cpp
    gen/track_layers.cpp
    third_party/lodepng.cpp
    ${OPENBIGRED_WORLD_SOURCES}
)

add_executable(vehicle_preview
    vehicle_preview.cpp
    formats/bigred_archive.cpp
    formats/game_files.cpp
    formats/terrain_members.cpp
    formats/localised_text.cpp   # the HUD's alphabet: interface_glyph
    formats/threedw.cpp
    render/gl_loader.cpp
    render/hud.cpp
    render/shader.cpp
)

# The RmlUi compatibility probe. NOT in ALL: it drags in the whole RmlUi build, which nothing
# else needs yet, and it exists to be run deliberately when RmlUi or the compiler moves.
#
#   cmake --build build --target rmlui_probe && build/src/rmlui_probe.exe
#
# ⚠️ RMLUI_STATIC_LIB must be defined by any target that links RmlUi statically, or its headers
# declare the symbols dllimport and the link fails.
add_executable(rmlui_probe EXCLUDE_FROM_ALL tools/rmlui_probe.cpp)
target_link_libraries(rmlui_probe PRIVATE RmlUi::Core)
target_compile_definitions(rmlui_probe PRIVATE RMLUI_STATIC_LIB)
# Outside the foreach below on purpose: it links no SDL2 (so SDL2main must not redefine its
# main()) and needs no game include path.
if(MINGW)
    target_link_options(rmlui_probe PRIVATE -static-libgcc -static-libstdc++ -static)
endif()

# The windowless measurement bench for the simulation. It opens no window and touches no GL; it
# is a target rather than a scratch file so the measurements behind a handling decision can be
# REPEATED instead of rewritten from scratch every session.
add_executable(physics_bench
    physics_bench.cpp
    game/level_catalog.cpp
    ${OPENBIGRED_WORLD_SOURCES}
)

# The windowless NETWORK bench: a host and two clients in one process over 127.0.0.1, or one
# machine of a multi-process test. See net_bench.cpp for what it checks.
add_executable(net_bench
    net_bench.cpp
    ${OPENBIGRED_NET_SOURCES}
    ${OPENBIGRED_WORLD_SOURCES}
)
target_link_libraries(net_bench PRIVATE enet)

# `cmake --build build --target dist` - what to COPY INTO AN ORIGINAL INSTALL to play (and to
# share the game with a friend for a network race): the game, SDL2.dll, and a short note.
# Lands in build/dist/.
add_custom_target(dist
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/dist"
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:openbigred> "${CMAKE_BINARY_DIR}/dist/"
    # Without its debug symbols: a fifth of the size, and nothing a player needs.
    COMMAND ${CMAKE_STRIP} "${CMAKE_BINARY_DIR}/dist/openbigred.exe"
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:SDL2::SDL2> "${CMAKE_BINARY_DIR}/dist/"
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_SOURCE_DIR}/tools/dist/LISEZMOI-OpenBigRed.txt"
            "${CMAKE_BINARY_DIR}/dist/"
    DEPENDS openbigred
    COMMENT "build/dist: openbigred.exe + SDL2.dll + LISEZMOI, to copy into the game folder")

foreach(tgt openbigred map_preview vehicle_preview physics_bench net_bench track_gen)
    target_include_directories(${tgt} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    # Link order matters for static archives: SDL2main.a needs symbols SDL2.dll.a provides, so
    # it must be listed first (GNU ld doesn't revisit an earlier archive once past it).
    # physics_bench is a CONSOLE program with a plain main(); SDL2main redefines main() to look
    # for SDL_main, so it must not be linked into it.
    if(tgt STREQUAL "physics_bench" OR tgt STREQUAL "net_bench")
        target_link_libraries(${tgt} PRIVATE SDL2::SDL2 opengl32)
    else()
        target_link_libraries(${tgt} PRIVATE SDL2::SDL2main SDL2::SDL2 opengl32)
    endif()
    # Statically link the MinGW runtime (libstdc++/libgcc/libwinpthread) so the exe runs
    # standalone on a machine without this toolchain's bin/ on PATH - confirmed needed (user hit
    # "libstdc++-6.dll est introuvable" / "libgcc_s_seh-1.dll est introuvable" launching the exe
    # outside the dev shell, where MinGW's bin dir happens to already be on PATH). `-static`
    # alone was tried first and also pulled in libwinpthread-1.dll's dependency (confirmed via
    # `objdump -p`, not mentioned in the user's report but would have failed the same way) -
    # `-static-libgcc -static-libstdc++` plus the blanket `-static` together fully static-link
    # the MinGW runtime while leaving SDL2 correctly dynamic (its import lib is passed as an
    # explicit path, not found via `-lSDL2`, so `-static` doesn't affect it). Verified end-to-end
    # by launching with PATH reduced to just `C:\Windows\System32;C:\Windows` (no MinGW
    # whatsoever) - runs and renders correctly.
    if(MINGW)
        target_link_options(${tgt} PRIVATE -static-libgcc -static-libstdc++ -static)
    endif()
    # Ship SDL2.dll next to the exe (dynamic SDL2, see root CMakeLists.txt) so the binary runs
    # without needing SDL2 on PATH.
    add_custom_command(TARGET ${tgt} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL2::SDL2>
            $<TARGET_FILE_DIR:${tgt}>
    )
endforeach()

add_executable(comfort_camera_test comfort_camera_test.cpp scene/comfort_camera.cpp)
target_include_directories(comfort_camera_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
if(MINGW)
    target_link_options(comfort_camera_test PRIVATE -static-libgcc -static-libstdc++ -static)
endif()
add_test(NAME comfort_camera COMMAND comfort_camera_test)
add_test(NAME comfort_camera_obstruction COMMAND physics_bench - CHOPPER1 --bloc=COMFORT_CAMERA)
set_tests_properties(comfort_camera_obstruction PROPERTIES TIMEOUT 30)

add_executable(replay_metadata_test replay_metadata_test.cpp sim/replay.cpp sim/fixed.cpp)
target_include_directories(replay_metadata_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
if(MINGW)
    target_link_options(replay_metadata_test PRIVATE -static-libgcc -static-libstdc++ -static)
endif()
add_test(NAME replay_metadata COMMAND replay_metadata_test)

# --- A6: checks that answer yes or no ----------------------------------------------------------

# Container decoding checks need no game files, SDL window or renderer.
add_executable(archive_validation_test archive_validation_test.cpp formats/bigred_archive.cpp)
target_include_directories(archive_validation_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
if(MINGW)
    target_link_options(archive_validation_test PRIVATE -static-libgcc -static-libstdc++ -static)
endif()
add_test(NAME archive_validation COMMAND archive_validation_test)

# `ctest --test-dir build` (or `-R name` for one). They read the original game from the default
# install, like every tool here (`-` = that install), and each one states what must come out.
# ⚠️ A REFERENCE CHECKSUM CHANGES WITH ANY CHANGE TO THE SIMULATION. When one is meant to, the
# new value goes here AND in the project dashboard's reference checks, in the same commit.
set(OBR_TEST_DIR "${CMAKE_CURRENT_BINARY_DIR}")
# The reference race: DIRT1, the JEEP on block 3, driven by -AUTODRIVE, checked at tick 600.
add_test(NAME race_checksum
         COMMAND openbigred - DIRT1 JEEP -PLACE=2 -HIDDEN -AUTODRIVE -QUITAFTER=600
         WORKING_DIRECTORY ${OBR_TEST_DIR})
set_tests_properties(race_checksum PROPERTIES TIMEOUT 120
                     PASS_REGULAR_EXPRESSION "tick 600 checksum 7d8581ca desync 0")
# The same race on a circuit declared in memory, with its map padded to 256 x 320: an added
# circuit and a rectangular map must race exactly like the shipped one (code review, A4).
add_test(NAME added_level_checksum
         COMMAND openbigred - GEN2 JEEP -ADDLEVEL=DIRT1:GEN2:PAD -PLACE=2 -HIDDEN -AUTODRIVE
                 -QUITAFTER=600
         WORKING_DIRECTORY ${OBR_TEST_DIR})
set_tests_properties(added_level_checksum PROPERTIES TIMEOUT 120
                     PASS_REGULAR_EXPRESSION "tick 600 checksum 7d8581ca desync 0")
# A resync snapshot saved, loaded back, and the race going on bit for bit (physics_bench's
# INSTANTANE block alone: under a second, where the whole bench takes ten).
add_test(NAME snapshot_roundtrip COMMAND physics_bench - WATER1 --bloc=INSTANTANE
         WORKING_DIRECTORY ${OBR_TEST_DIR})
set_tests_properties(snapshot_roundtrip PROPERTIES TIMEOUT 120
                     PASS_REGULAR_EXPRESSION "charge : oui ; relu a l'identique : oui ; 1500 ticks ensuite : IDENTIQUES")
# The whole network on 127.0.0.1: lobby, lockstep, desync and resync, departures, eight players.
add_test(NAME net_bench COMMAND net_bench - WORKING_DIRECTORY ${OBR_TEST_DIR})
set_tests_properties(net_bench PROPERTIES TIMEOUT 300)
# The catalogue: every circuit's script read, the unplayable ones left out.
add_test(NAME level_catalogue COMMAND openbigred --list WORKING_DIRECTORY ${OBR_TEST_DIR})
set_tests_properties(level_catalogue PROPERTIES TIMEOUT 60
                     PASS_REGULAR_EXPRESSION "DIRT1\t[A-Z0-9,]*JEEP")

add_executable(terrain_members_test terrain_members_test.cpp
    formats/terrain_members.cpp formats/bigred_archive.cpp)
target_include_directories(terrain_members_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
if(MINGW)
    target_link_options(terrain_members_test PRIVATE -static-libgcc -static-libstdc++ -static)
endif()
add_test(NAME terrain_member_selection COMMAND terrain_members_test)
if(OPENBIGRED_GAME_DIR)
    add_test(NAME terrain_members_retail COMMAND terrain_members_test -)
endif()
add_test(NAME championship_rules COMMAND physics_bench - DIRT1 --bloc=CHAMPIONNAT
         WORKING_DIRECTORY ${OBR_TEST_DIR})
set_tests_properties(championship_rules PROPERTIES TIMEOUT 120)
# A generated circuit can be driven: an autopilot laps a DIRT and a SNOW circuit in every vehicle
# of the theme, and the lap is the RACE's own - the generated CHECKP lines fired in order.
add_test(NAME generated_lap
         COMMAND track_gen - DIRT 1 -DRIVE -NOWINDOW WORKING_DIRECTORY ${OBR_TEST_DIR})
set_tests_properties(generated_lap PROPERTIES TIMEOUT 120
                     PASS_REGULAR_EXPRESSION "ATV +tour boucle"
                     FAIL_REGULAR_EXPRESSION "PAS de tour")
add_test(NAME generated_lap_snow
         COMMAND track_gen - SNOW 2 -DRIVE -NOWINDOW WORKING_DIRECTORY ${OBR_TEST_DIR})
set_tests_properties(generated_lap_snow PROPERTIES TIMEOUT 120
                     PASS_REGULAR_EXPRESSION "SNOWCAT +tour boucle"
                     FAIL_REGULAR_EXPRESSION "PAS de tour")
# The autotiler understands how the maps lay their ground: DIRT2 laid again from the material of
# each vertex alone comes back 99.5 % identical or more (it is 99.7 %; gen/autotile.h).
add_test(NAME autotile_round_trip
         COMMAND track_gen - --autotile=DIRT2 -MIN=995 WORKING_DIRECTORY ${OBR_TEST_DIR})
set_tests_properties(autotile_round_trip PROPERTIES TIMEOUT 60)
