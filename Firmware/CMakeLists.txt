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

# Phase 9: payload profiling / verification commissioning CLI (§28.5/§31/§44).
add_executable(turret-payload tools/turret_payload.cpp)
target_link_libraries(turret-payload PRIVATE ota_core)
