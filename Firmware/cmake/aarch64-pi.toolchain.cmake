# Cross-compiling for the station: Raspberry Pi 5, Debian 13 (trixie) on aarch64.
#
# Why this file exists: the station used to compile every release itself, which cost about four
# minutes of the deploy and wore on its storage. The container this is built in runs the *same*
# Debian release as the station (measured 2026-09-28: container Debian 13 trixie, station Debian
# 13 trixie, both g++ 14.2.0), so the arm64 development packages installed here are the station's
# own library versions -- not an approximation of them. That is what makes a cross build honest
# here rather than a gamble on ABI drift.
#
# The pinned *_DIR hints are passed by tools/cross_build.py rather than set here, because they
# depend on where the locally built GTest landed. They are not decoration: without them
# find_package can find an amd64 config file and then fail to link, with an error that reads
# like a missing library instead of "wrong architecture".

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Debian multiarch: the arm64 libraries live in /usr/lib/aarch64-linux-gnu, and naming the
# architecture is what makes find_library look there instead of in the host directory.
set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)

set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu /usr)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)   # host tools (cmake generators) still run here
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)    # the pinned hints below are absolute paths

# The cross linker does NOT search the multiarch directory by default. Verified the blunt way,
# with a two-line program and -lyaml-cpp: it said "cannot find -lyaml-cpp" until told. The
# rpath-link is for the transitive dependencies (spdlog -> fmt) that are not named on the line.
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "-L/usr/lib/aarch64-linux-gnu -Wl,-rpath-link,/usr/lib/aarch64-linux-gnu")

set(CMAKE_PREFIX_PATH "/usr/lib/aarch64-linux-gnu")
