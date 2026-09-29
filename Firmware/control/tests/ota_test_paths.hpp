#pragma once
// Where a test finds the shipped config. __FILE__ is the *build machine's* source tree, and it
// stopped being an answer the day the binaries were cross-compiled (2026-09-28): the station has
// no /workspace, so four config-reading tests failed there over a path, not over behaviour.
// OTA_FIRMWARE_ROOT names the release the binary is running from -- set by the launcher whenever
// the binaries were built elsewhere. The compiled-in path stays as the fallback for a native,
// in-tree run, which is how these tests were written and how they still run in the container.
#include <cstdlib>
#include <filesystem>
#include <string>

inline std::filesystem::path ota_test_firmware_dir(const std::filesystem::path& compiled_in) {
  const char* root = std::getenv("OTA_FIRMWARE_ROOT");
  if (root != nullptr && *root != '\0') return std::filesystem::path(root);
  return compiled_in;
}

inline std::string ota_test_config(const char* relative, const std::string& compiled_in) {
  const char* root = std::getenv("OTA_FIRMWARE_ROOT");
  if (root != nullptr && *root != '\0')
    return (std::filesystem::path(root) / relative).string();
  return compiled_in;
}
