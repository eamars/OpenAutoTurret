// OpenAutoTurret — the §44 payload profiling utility (slow commissioning).
//
// Runs the response-test battery against a MotorBackend:
//   * low-amplitude step response (+ and -, per axis)
//   * low-amplitude triangle (tracking RMS)
//   * braking from incrementally increasing speeds (feeds a_brake / soft
//     margins, §44)
//   * holding effort at a check angle (pitch gravity term)
// and extracts the measured response baseline (§44 "Measure") plus the safe
// v/a/j limits (§28.5).
//
// This is a STANDALONE slow path (like turret-can): it owns the axis directly
// while the operator runs commissioning (controld is NOT running — the
// single-owner-of-motors rule, §4.1). It is transport-agnostic and unit-
// testable against the SimMotorBackend with no CAN (§54).
//
// Determinism hooks: the tick clock (TimeSource) and the pacer are injectable,
// so tests run on a synthetic clock with no real-time sleep.
#pragma once

#include <array>
#include <functional>
#include <string>

#include "common/types.hpp"
#include "control/motor_backend.hpp"
#include "control/safety_envelope.hpp"
#include "payload/payload_profile.hpp"
#include "payload/response_metrics.hpp"

namespace ota {
namespace payload {

struct ProfilerConfig {
  // §44: low-amplitude, conservative limits, safe central region.
  double step_amplitude_rad = 3.0 * kDeg2Rad;
  double triangle_amplitude_rad = 2.0 * kDeg2Rad;
  double speed_rad_s = 10.0 * kDeg2Rad;        // conservative test speed
  std::array<double, 4> brake_speeds_rad_s = {
      2.0 * kDeg2Rad, 5.0 * kDeg2Rad, 10.0 * kDeg2Rad, 20.0 * kDeg2Rad};
  double hold_offset_rad = 5.0 * kDeg2Rad;     // holding-effort check angle
  double region_center_rad = 0.0;
  double region_half_span_rad = 20.0 * kDeg2Rad;
  // Safe envelope from the station config: the profiled limits can never
  // exceed it.
  double env_v_max_rad_s = 30.0 * kDeg2Rad;
  double env_a_max_rad_s2 = 60.0 * kDeg2Rad;
  // Timing (one tick = one snapshot + one command).
  int tick_ns = 5'000'000;         // 200 Hz
  int max_move_ticks = 15 * 200;   // 15 s per sub-test
  double settle_band_frac = 0.02;
  double at_rest_vel_rad_s = 0.05;
  int settle_ticks = 60;           // 0.3 s dwell
  int hold_ticks = 200;            // 1.0 s holding capture
};

class PayloadProfiler {
 public:
  using TimeSource = std::function<TimeNs()>;
  using Pacer = std::function<void()>;

  // Default TimeSource = monotonic now; default Pacer = real sleep for one
  // tick. Tests inject a synthetic clock + no-op pacer.
  PayloadProfiler(MotorBackend& backend, ProfilerConfig cfg,
                  TimeSource ts = nullptr, Pacer pacer = nullptr);

  // Profile one axis starting from q_start (must lie inside the safe central
  // region). All motion is clamped to region ∩ `lim` soft limits.
  AxisProfileMetrics profile_axis(AxisId a, double q_start_rad,
                                  const AxisLimits& lim, std::string& err);

  // §28.5: derive the safe motion limits from the measured baseline
  // (never above the configured envelope).
  static void derive_limits(const AxisProfileMetrics& m,
                            const ProfilerConfig& cfg,
                            AxisPayloadProfile& out);

 private:
  double clamp_target(AxisId a, double q, const AxisLimits& lim) const;

  // Command (q_ref, lim) every tick, sampling each tick, until stop() is true
  // or max_ticks. Appends samples. Returns false on timeout.
  bool run(AxisId a, double q_ref, double lim, int max_ticks,
           const std::function<bool(const AxisSnapshot&)>& stop,
           std::vector<ResponseSample>* samples, std::string& err);
  // Move to q and settle (bounded).
  bool settle_at(AxisId a, double q, const AxisLimits& lim, std::string& err);

  MotorBackend& backend_;
  ProfilerConfig cfg_;
  TimeSource ts_;
  Pacer pacer_;
};

}  // namespace payload
}  // namespace ota
