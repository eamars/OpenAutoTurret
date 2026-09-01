// OpenAutoTurret — payload mismatch detection + automatic derating
// (architecture §31.3, §31.4, §59.19, §42.1).
//
// After homing (optionally at boot, §27, or on the developer command
// `start_payload_verification`, §42.2) the station runs a SMALL move in a
// safe central region with conservative limits, measures the response, and
// compares it against the stored payload profile baseline:
//
//   * match within tolerance      -> status OK
//   * material deviation          -> status MISMATCH
//   * no stored profile           -> status NO_PROFILE
//   * check could not be measured -> status ERROR
//
// On MISMATCH the station automatically selects the conservative path
// (§31.3): it DERATES its own motion limits by the configured derate factor.
// It never runs an unconstrained aggressive online PID self-tune at startup
// (frozen decision §59.19) — re-profiling with the commissioning tool
// (turret-payload) is the way to clear a mismatch.
//
// Pure logic: no CAN, no motor, no I/O.
#pragma once

#include <string>
#include <vector>

#include "common/types.hpp"
#include "payload/payload_profile.hpp"
#include "payload/response_metrics.hpp"

namespace ota {
namespace payload {

enum class PayloadStatus : uint8_t {
  NoProfile = 0,  // no stored profile for the active payload
  Ok = 1,         // measured response matches the stored baseline
  Mismatch = 2,   // material deviation -> conservative profile / derate
  Error = 3,      // the check could not be measured (timeout, no feedback)
};

inline const char* payload_status_name(PayloadStatus s) {
  switch (s) {
    case PayloadStatus::NoProfile: return "no_profile";
    case PayloadStatus::Ok:        return "ok";
    case PayloadStatus::Mismatch:  return "mismatch";
    case PayloadStatus::Error:     return "error";
  }
  return "?";
}

// Tolerances for comparing a measured step against the stored baseline
// (§31.3 "compare against the stored profile"). Ratios are measured/base.
struct VerifyTolerances {
  double rise_ratio_min = 0.4;     // measured rise time vs baseline
  double rise_ratio_max = 2.5;
  double settle_ratio_min = 0.4;
  double settle_ratio_max = 2.5;
  double overshoot_abs_max = 0.02;  // measured overshoot may exceed baseline by
  double peak_effort_ratio_max = 2.0;  // (payload mass shifts the gravity term)
  double peak_effort_abs_max_nm = 0.1;
  double tracking_rms_ratio_max = 2.0;
  double tracking_rms_abs_max_rad = 0.002;
};

// Per-axis verification outcome.
struct AxisVerifyResult {
  bool measured = false;              // the check motion completed
  StepMetrics step_pos;               // measured (small positive move)
  StepMetrics step_neg;               // measured (small negative move)
  bool ok = false;
  std::vector<std::string> violations;  // human-readable (events/logs)
};

// Overall verification outcome.
struct VerifyResult {
  PayloadStatus status = PayloadStatus::NoProfile;
  AxisVerifyResult axes[kAxisCount];  // [pitch, yaw]
  std::string detail;                 // summary for telemetry / event log
};

// Compare the measured steps of one axis against the stored baseline
// (§31.3). Worst of the +/− step is compared; a missing baseline or an
// unmeasured step counts against the axis (fail safe).
AxisVerifyResult compare_axis(const AxisPayloadProfile& base,
                              const StepMetrics& step_pos,
                              const StepMetrics& step_neg,
                              const VerifyTolerances& tol);

// Fold the per-axis outcomes into the overall status:
//   any unmeasured axis           -> Error
//   no profile (empty baselines)  -> NoProfile
//   any axis in violation         -> Mismatch
//   otherwise                     -> Ok
PayloadStatus overall_status(const AxisVerifyResult (&axes)[kAxisCount],
                             bool profile_loaded);

// §31.3/§59.19: what the station does about the status. On Mismatch the
// motion limits are derated by `derate_factor` (the conservative profile is
// the active one scaled down); nothing else changes — no online PID retune.
struct DerateDecision {
  bool derate = false;
  double factor = 1.0;  // multiplier for the profile v/a/j limits
  std::string action;   // "ok" | "derate <f>" | "no_profile" | "error"
};

DerateDecision decide(const VerifyResult& r, double derate_factor);

}  // namespace payload
}  // namespace ota
