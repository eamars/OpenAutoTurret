cmake_minimum_required(VERSION 3.16)
project(open_auto_turret
        VERSION 0.1.0
        DESCRIPTION "OpenAutoTurret vision tracking station (control stack)"
        LANGUAGES CXX)

# The whole production stack builds from Firmware/ (see PROGRESS.md location
# rule). C++20 per architecture doc §5.2/§51.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(OTA_BUILD_TESTS "Build unit tests" ON)
if(OTA_BUILD_TESTS)
  enable_testing()
endif()

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

add_subdirectory(control)

# Commissioning / probe CLI (architecture §50 tools/).
add_executable(turret-can tools/turret_can.cpp)
target_link_libraries(turret-can PRIVATE ota_core)
add_executable(cybergear-diagnostics tools/cybergear_diagnostics.cpp)
target_link_libraries(cybergear-diagnostics PRIVATE ota_core)
add_executable(probe-commissioning-watchdog tools/probe_commissioning_watchdog.cpp)
target_link_libraries(probe-commissioning-watchdog PRIVATE ota_core)
add_test(NAME commissioning_watchdog_probe COMMAND probe-commissioning-watchdog ${CMAKE_CURRENT_SOURCE_DIR}/config/turret.yaml)
add_executable(probe-yousee-backpressure tools/probe_yousee_backpressure.cpp)
target_link_libraries(probe-yousee-backpressure PRIVATE ota_core)
add_test(NAME yousee_backpressure_probe COMMAND probe-yousee-backpressure)
set_tests_properties(commissioning_watchdog_probe yousee_backpressure_probe PROPERTIES TIMEOUT 10)
add_executable(probe-reference-settling tools/probe_reference_settling.cpp)
target_link_libraries(probe-reference-settling PRIVATE ota_core)
add_test(NAME reference_settling_probe COMMAND probe-reference-settling)

# Phase 9: payload profiling / verification commissioning CLI (§28.5/§31/§44).
add_executable(turret-payload tools/turret_payload.cpp)
target_link_libraries(turret-payload PRIVATE ota_core)

# §81: replay a recorded session through the real controller. Registered as a test with
# --expect, so the tool's own behaviour is checked by the same `ctest` run as everything
# else: a tool that no one runs is a tool that broke three months ago.
# §94/§46: what a control cycle costs, per mode. Run as a ctest entry with a gate that is
# deliberately about orders of magnitude: the point is to notice a factor of ten (an
# allocation, a write, a socket on the control path), not 40 us of laptop jitter — the
# station's own timing budget is measured on the station, and this says so in its header.
add_executable(cycle-cost tools/cycle_cost.cpp)
target_link_libraries(cycle-cost PRIVATE ota_core)
add_test(NAME cycle_cost_gate
         COMMAND cycle-cost --cycles 2000 --assert-p99-us 4000)

add_executable(replay-session tools/replay_session.cpp)
add_executable(inspect-perception-wire tools/inspect_perception_wire.cpp)
target_link_libraries(inspect-perception-wire PRIVATE ota_core)
add_executable(probe-estimator-boundary tools/probe_estimator_boundary.cpp)
add_executable(probe-tracking-boundaries tools/probe_tracking_boundaries.cpp)
add_executable(probe-alignment tools/probe_alignment.cpp)
target_link_libraries(probe-alignment PRIVATE ota_core)
add_test(NAME alignment_geometry_probe COMMAND probe-alignment)
set_tests_properties(alignment_geometry_probe PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(probe-tracking-boundaries PRIVATE ota_core)
add_executable(probe-closed-loop tools/probe_closed_loop.cpp)
target_link_libraries(probe-closed-loop PRIVATE ota_core)
add_executable(replay-guidance tools/replay_guidance.cpp)
target_link_libraries(replay-guidance PRIVATE ota_core)
# Standalone design comparison: simulated successes have failed live validation.
# Keep its acceptance report separate from the regression suite.
target_link_libraries(probe-estimator-boundary PRIVATE ota_core)
add_test(NAME estimator_boundary_probe COMMAND probe-estimator-boundary)
add_executable(probe-command-cache tools/probe_command_cache.cpp)
add_executable(probe-mode-transition tools/probe_mode_transition.cpp)
target_link_libraries(probe-mode-transition PRIVATE ota_core)
target_link_libraries(probe-command-cache PRIVATE ota_core)
add_test(NAME command_cache_probe COMMAND probe-command-cache)
add_executable(probe-register-boundary tools/probe_register_boundary.cpp)
target_link_libraries(probe-register-boundary PRIVATE ota_core)
add_test(NAME register_boundary_probe COMMAND probe-register-boundary)
target_link_libraries(replay-session PRIVATE ota_core)
# The same session against the station's own configuration file. Two entries, not one
# parametrised loop, because the interesting difference between them is the station: if the
# YAML-driven run ever stops agreeing with the default one, somebody changed a commissioning
# value that changes what the operator may do — which is worth a failing test.
add_test(NAME replay_session_station_config
         COMMAND replay-session
                 --config ${CMAKE_CURRENT_SOURCE_DIR}/config/turret.yaml
                 --expect "phase=TRACKING sel=1"
                 --expect "phase=WAIT_TARGET sel=0 intent=hold"
                 ${CMAKE_CURRENT_SOURCE_DIR}/tools/example_sessions/select_and_lose.txt)
add_test(NAME replay_session_cli
         COMMAND replay-session
                 --expect "phase=TRACKING sel=1"
                 --expect "cmd=select_target accepted"
                 --expect "phase=WAIT_TARGET sel=0 intent=hold"
                 ${CMAKE_CURRENT_SOURCE_DIR}/tools/example_sessions/select_and_lose.txt)
