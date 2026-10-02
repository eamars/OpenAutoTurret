cmake_minimum_required(VERSION 3.16)
project(tracking_core LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# ADR-003 tracking core (no yaml): the target estimator, Level 1 and the tracker that joins
# them through the existing camera/kinematics/LOS-joint geometry (control/src/geometry).
# Linked by the independent 3a host (commissiond), production controld and the simulator.
add_library(tracking_core STATIC estimator.cpp level1.cpp tracker.cpp)
target_include_directories(tracking_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../control/src)
target_compile_options(tracking_core PRIVATE -Wall -Wextra -Werror)
# Parameter asset loader and the closed-loop simulator (yaml-cpp, the ADR-002.2 plant models).
add_library(tracking_tools STATIC tracking_config.cpp tracking_sim.cpp)
target_link_libraries(tracking_tools PUBLIC tracking_core servo_tools yaml-cpp)
target_compile_options(tracking_tools PRIVATE -Wall -Wextra -Werror)
add_executable(tracking-sim tracking_sim_main.cpp)
target_link_libraries(tracking-sim PRIVATE tracking_tools)
add_executable(test-tracking tests/test_tracking.cpp)
target_link_libraries(test-tracking PRIVATE tracking_tools)
add_test(NAME tracking_core COMMAND test-tracking)
