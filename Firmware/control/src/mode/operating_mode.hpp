// OpenAutoTurret — the operating-mode vocabulary.
//
// Extracted from mode_manager.hpp for the same reason the phase vocabulary was
// extracted: the per-cycle trace has to say which mode owned the axis when the row
// was written, and telemetry cannot include the manager that includes telemetry.
// Keeping one copy is the whole point -- a second list of these three names drifts
// the first time a mode is added.
#pragma once

#include <cstdint>

namespace ota {

// The three primary operator modes (§2). HOLD / JOG / TRACKING / COASTING /
// LOST_HOLD / SWEEP ... are phases *inside* these, not peers.
enum class OperatingMode : uint8_t {
  Manual,
  AutoTrack,
  AutoRoam,
};

inline const char* operating_mode_name(OperatingMode m) {
  switch (m) {
    case OperatingMode::Manual:    return "MANUAL";
    case OperatingMode::AutoTrack: return "AUTO_TRACK";
    case OperatingMode::AutoRoam:  return "AUTO_ROAM";
  }
  return "?";
}

// Parse a mode name from a web command. Case-insensitive on the documented
// spellings. Returns false (rather than guessing) for anything else — a mode
// command that silently fell back to MANUAL would be a safety surprise.
inline bool operating_mode_from_name(const char* name, OperatingMode& out) {
  auto eq = [](const char* a, const char* b) {
    int i = 0;
    for (;; ++i) {
      char ca = a[i], cb = b[i];
      if (ca >= 'a' && ca <= 'z') ca -= 32;
      if (cb >= 'a' && cb <= 'z') cb -= 32;
      if (ca != cb) return false;
      if (ca == '\0') return true;
    }
  };
  if (name == nullptr) return false;
  if (eq(name, "MANUAL") || eq(name, "MAN")) { out = OperatingMode::Manual; return true; }
  if (eq(name, "AUTO_TRACK") || eq(name, "AUTOTRACK") || eq(name, "TRACK")) {
    out = OperatingMode::AutoTrack; return true;
  }
  if (eq(name, "AUTO_ROAM") || eq(name, "AUTOROAM") || eq(name, "ROAM")) {
    out = OperatingMode::AutoRoam; return true;
  }
  return false;
}

}  // namespace ota
