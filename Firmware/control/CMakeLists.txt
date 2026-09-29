# ota_core: shared control-stack library (CAN, control, calibration,
# geometry, tracking, telemetry). Kept separate from the executables so
# tools and tests link the exact same code the daemon runs.
file(GLOB_RECURSE OTA_CORE_SRCS CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(FILTER OTA_CORE_SRCS EXCLUDE REGEX "/main\\.cpp$")
# Keep the mixed production-profile parser in the shared library even if the
# source collection is changed from a recursive glob in a future build cleanup.
set(MIXED_HARDWARE_PROFILE_SOURCE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config/mixed_hardware_profile.cpp)
if(NOT MIXED_HARDWARE_PROFILE_SOURCE IN_LIST OTA_CORE_SRCS)
  list(APPEND OTA_CORE_SRCS ${MIXED_HARDWARE_PROFILE_SOURCE})
endif()

add_library(ota_core STATIC ${OTA_CORE_SRCS})
target_include_directories(ota_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_options(ota_core PRIVATE -Wall -Wextra)
target_link_libraries(ota_core PUBLIC Threads::Threads yaml-cpp spdlog::spdlog axis_control_core)

# The control daemon.
add_executable(controld ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(controld PRIVATE ota_core)

if(OTA_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  file(GLOB OTA_TEST_SRCS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
  foreach(test_src ${OTA_TEST_SRCS})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} PRIVATE ota_core GTest::gtest_main)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
