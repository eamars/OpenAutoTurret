# Cross-compiling for the station against an extracted Debian 13 arm64 sysroot that carries its
# own cross compiler and binutils (on the Windows workstation: run/adr0022-debian13/root, used from
# WSL). Same compiler (Debian GCC 14.2.0) and the same library packages as the station, so nothing
# has to be installed on the host -- the counterpart of aarch64-pi.toolchain.cmake, which needs the
# host's own arm64 multiarch packages.
#
# No CMAKE_CROSSCOMPILING_EMULATOR: the tree this builds is shipped and its suite runs on the
# station, where an emulator path would be baked into every registered CTest command.
#
#   cmake -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-sysroot.toolchain.cmake -DOTA_SYSROOT=/abs/root ...
# (tools/cross_build.py --sysroot does this). The sysroot's binutils link against libraries that
# live beside it; put that directory on LD_LIBRARY_PATH (cross_build.py --host-lib).

if(NOT OTA_SYSROOT)
  set(OTA_SYSROOT "$ENV{OTA_SYSROOT}")
endif()
if(NOT IS_DIRECTORY "${OTA_SYSROOT}/usr/lib/aarch64-linux-gnu")
  message(FATAL_ERROR "OTA_SYSROOT must name a Debian arm64 sysroot (got '${OTA_SYSROOT}')")
endif()
# try_compile projects re-read this file without the main project's cache.
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES OTA_SYSROOT)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER "${OTA_SYSROOT}/usr/bin/aarch64-linux-gnu-gcc-14")
set(CMAKE_CXX_COMPILER "${OTA_SYSROOT}/usr/bin/aarch64-linux-gnu-g++-14")
set(CMAKE_SYSROOT "${OTA_SYSROOT}")
# The compiler driver looks for as/ld under the host's prefix unless told where the sysroot's are.
set(CMAKE_C_FLAGS_INIT "-B${OTA_SYSROOT}/usr/aarch64-linux-gnu/bin")
set(CMAKE_CXX_FLAGS_INIT "-B${OTA_SYSROOT}/usr/aarch64-linux-gnu/bin")

set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)
set(CMAKE_FIND_ROOT_PATH "${OTA_SYSROOT}" "${OTA_SYSROOT}/usr/aarch64-linux-gnu")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# As in aarch64-pi.toolchain.cmake: the cross linker does not search the multiarch directory by
# default, and rpath-link covers transitive dependencies (spdlog -> fmt).
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "-L${OTA_SYSROOT}/usr/lib/aarch64-linux-gnu -Wl,-rpath-link,${OTA_SYSROOT}/usr/lib/aarch64-linux-gnu")
set(CMAKE_SKIP_RPATH ON)
