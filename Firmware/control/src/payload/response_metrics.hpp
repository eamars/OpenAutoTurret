// OpenAutoTurret — payload response metrics (architecture §44 "Measure").
//
// Pure time-series analysis: no CAN, no motor, no I/O. Given a sampled
// trajectory (monotonic timestamps, position, velocity, effort), extract the
// response metrics the payload profiling uses:
//   * step response: rise time (10-90 %), overshoot, settling time, tracking
//     RMS error, peak effort
//   * braking: stop distance and stop time from a constant-velocity approach
//
// All quantities are SI (s, rad, rad/s, rad/s^2, Nm). The analysis is
// deliberately conservative about degenerate input: short or non-converging
// trajectories still yield a VALID result with worst-case (largest) values,
// so a comparison against a stored baseline fails safe (§31.3).
#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

#include "common/types.hpp"

namespace ota {
namespace payload {

// One sampled point of a moving axis (monotonic-clock timestamps).
struct ResponseSample {
  TimeNs t_ns = 0;
  double q_rad = 0.0;
  double v_rad_s = 0.0;
  double effort_nm = 0.0;
};

// Step-response metrics for one move (q0 -> q1), §44: rise time, overshoot,
// settling, tracking error, peak effort.
struct StepMetrics {
  bool valid = false;            // enough data; the move made >= 50 % progress
  double amplitude_rad = 0.0;    // |q1 - q0|
  double rise_time_s = 0.0;      // 10 % -> 90 % of the move
  double overshoot = 0.0;        // excursion beyond q1 / amplitude, >= 0
  double settling_time_s = 0.0;  // first time inside the band that stays there
  double settle_band_rad = 0.0;  // the band used (band_frac * amplitude)
  double tracking_rms_rad = 0.0; // RMS(q - q1) over the settled tail
  double peak_effort_nm = 0.0;
};

// Braking metrics: approach at ~v0, brake command, travel to rest (§44).
struct BrakeMetrics {
  bool valid = false;
  double v0_rad_s = 0.0;          // approach speed at the brake command
  double stop_distance_rad = 0.0; // net travel from brake command to at-rest
  double stop_time_s = 0.0;       // brake command to at-rest
};

// Full per-axis profiling result; the "response baseline" stored in a payload
// profile (§28.5).
struct AxisProfileMetrics {
  StepMetrics step_pos;              // small positive step
  StepMetrics step_neg;              // small negative step
  double triangle_rms_rad = 0.0;     // tracking RMS over a low-amplitude triangle
  BrakeMetrics brake;                // braking from the (highest) test speed
  double hold_effort_nm = 0.0;       // holding effort at the check angle
  double max_verified_speed_rad_s = 0.0;  // highest brake-test speed that stopped
  bool valid = false;                // all sub-tests that ran produced data
};

// ---------------------------------------------------------------------------
// Step analysis.
//
// `samples` starts at (or near) q0 and ends at (or near) q1. Crossing times
// are linearly interpolated between samples. If the move never reaches 90 %
// (or never settles within the capture), rise/settling report the full
// capture duration — a worst-case value that trips the baseline comparison.
// ---------------------------------------------------------------------------
inline StepMetrics analyze_step(const std::vector<ResponseSample>& samples,
                                double q0, double q1,
                                double band_frac = 0.02) {
  StepMetrics m;
  m.amplitude_rad = std::fabs(q1 - q0);
  m.settle_band_rad = band_frac * m.amplitude_rad;
  if (samples.size() < 5 || m.amplitude_rad < 1e-6) return m;

  const double span = q1 - q0;
  const double t_first = samples.front().t_ns / 1e9;
  const double t_last = samples.back().t_ns / 1e9;
  const double dur = std::max(1e-9, t_last - t_first);

  // Linear-interpolated crossing time of a level (first crossing).
  auto crossing = [&](double level) -> double {
    for (std::size_t i = 1; i < samples.size(); ++i) {
      const double a = samples[i - 1].q_rad;
      const double b = samples[i].q_rad;
      if ((a - level) * (b - level) <= 0.0 && a != b) {
        const double f = (level - a) / (b - a);
        return samples[i - 1].t_ns / 1e9 +
               f * (samples[i].t_ns - samples[i - 1].t_ns) / 1e9;
      }
    }
    return -1.0;  // not crossed
  };

  double max_progress = 0.0;
  for (const auto& s : samples)
    max_progress = std::max(max_progress, (s.q_rad - q0) / span);
  m.valid = max_progress >= 0.5;  // the move actually happened

  const double t10 = crossing(q0 + 0.10 * span);
  const double t90 = crossing(q0 + 0.90 * span);
  m.rise_time_s = (t10 < 0 || t90 < 0) ? dur : std::max(0.0, t90 - t10);

  // Overshoot: maximum excursion beyond q1 in the direction of the move.
  double beyond = 0.0;
  for (const auto& s : samples) {
    const double e = (s.q_rad - q1) / span;  // signed, normalized
    if (e > beyond) beyond = e;
  }
  m.overshoot = beyond;

  // Settling: the first time after which the trajectory stays inside the
  // band around q1.
  auto inside_band = [&](std::size_t i) {
    return std::fabs(samples[i].q_rad - q1) <= m.settle_band_rad;
  };
  double settle = -1.0;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (!inside_band(i)) continue;
    bool stays = true;
    for (std::size_t j = i + 1; j < samples.size() && stays; ++j)
      stays = inside_band(j);
    if (stays) {
      settle = samples[i].t_ns / 1e9 - t_first;
      break;
    }
  }
  if (settle < 0) settle = dur;  // never settled within the capture

  // Tracking RMS over the settled tail (last 25 % of the capture).
  const std::size_t tail_begin =
      samples.size() - std::max<std::size_t>(4, samples.size() / 4);
  double ss = 0.0;
  int n = 0;
  for (std::size_t i = tail_begin; i < samples.size(); ++i) {
    const double e = samples[i].q_rad - q1;
    ss += e * e;
    ++n;
  }
  m.tracking_rms_rad = n > 0 ? std::sqrt(ss / n) : 0.0;

  m.settling_time_s = std::min(settle, dur);
  for (const auto& s : samples)
    m.peak_effort_nm = std::max(m.peak_effort_nm, std::fabs(s.effort_nm));
  return m;
}

// ---------------------------------------------------------------------------
// Braking analysis. `brake_idx` is the sample where the brake command was
// issued (approach at ~v0 before it, deceleration after). Stop distance is
// the net travel from the brake command to the first at-rest sample; v0 is
// the speed at the brake command.
// ---------------------------------------------------------------------------
inline BrakeMetrics analyze_brake(const std::vector<ResponseSample>& samples,
                                  std::size_t brake_idx,
                                  double at_rest_vel_rad_s = 0.05) {
  BrakeMetrics m;
  if (samples.size() < 3) return m;  // need the brake sample + at least one after
  brake_idx = std::min(brake_idx, samples.size() - 1);
  m.v0_rad_s = std::fabs(samples[brake_idx].v_rad_s);

  // First sample after the command where the axis is at rest.
  std::size_t rest_idx = samples.size();  // not reached
  for (std::size_t i = brake_idx + 1; i < samples.size(); ++i) {
    if (std::fabs(samples[i].v_rad_s) < at_rest_vel_rad_s) {
      rest_idx = i;
      break;
    }
  }
  if (rest_idx == samples.size()) return m;  // still moving: invalid
  const auto& b = samples[brake_idx];
  const auto& r = samples[rest_idx];
  m.stop_distance_rad = std::fabs(r.q_rad - b.q_rad);
  m.stop_time_s = std::max(0.0, (r.t_ns - b.t_ns) / 1e9);
  m.valid = m.stop_time_s > 0.0;
  return m;
}

// RMS tracking error of a trajectory against a fixed target (rad).
inline double rms_tracking_error(const std::vector<ResponseSample>& samples,
                                 double q_target_rad) {
  if (samples.empty()) return 0.0;
  double ss = 0.0;
  for (const auto& s : samples) {
    const double e = s.q_rad - q_target_rad;
    ss += e * e;
  }
  return std::sqrt(ss / samples.size());
}

}  // namespace payload
}  // namespace ota
