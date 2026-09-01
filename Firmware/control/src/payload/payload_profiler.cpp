// OpenAutoTurret — §44 payload profiling utility implementation.
#include "payload/payload_profiler.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "common/time.hpp"

namespace ota {
namespace payload {

PayloadProfiler::PayloadProfiler(MotorBackend& backend, ProfilerConfig cfg,
                                 TimeSource ts, Pacer pacer)
    : backend_(backend), cfg_(std::move(cfg)), ts_(std::move(ts)),
      pacer_(std::move(pacer)) {
  if (!ts_) ts_ = []() { return now_monotonic_ns(); };
  if (!pacer_) {
    const auto tick = std::chrono::nanoseconds(cfg_.tick_ns);
    pacer_ = [tick]() { std::this_thread::sleep_for(tick); };
  }
}

double PayloadProfiler::clamp_target(AxisId, double q,
                                     const AxisLimits& lim) const {
  const double lo = cfg_.region_center_rad - cfg_.region_half_span_rad;
  const double hi = cfg_.region_center_rad + cfg_.region_half_span_rad;
  double t = std::max(lo, std::min(hi, q));
  if (lim.valid) {
    // Keep a 2 deg margin from the soft limits on each side. If the usable
    // band is narrower than 2 * margin, the [min+margin, max-margin] range
    // inverts; in that degenerate case all targets collapse to the band
    // midpoint so the battery cannot measure a real response (the profile
    // is reported invalid). Fails safe: never commands outside the band.
    const double m = 2.0 * kDeg2Rad;
    const double a = lim.q_soft_min_rad + m;
    const double b = lim.q_soft_max_rad - m;
    t = a <= b ? std::max(a, std::min(b, t)) : 0.5 * (a + b);
  }
  return t;
}

bool PayloadProfiler::run(AxisId a, double q_ref, double lim, int max_ticks,
                          const std::function<bool(const AxisSnapshot&)>& stop,
                          std::vector<ResponseSample>* samples,
                          std::string& err) {
  for (int i = 0; i < max_ticks; ++i) {
    const TimeNs t = ts_();
    AxisSnapshot s = backend_.snapshot(a, t);
    if (samples) samples->push_back({t, s.q_rad, s.v_rad_s, s.torque_nm});
    backend_.command(a, q_ref, lim);
    pacer_();
    if (stop(s)) return true;
  }
  err = "profiler: sub-test timeout";
  return false;
}

bool PayloadProfiler::settle_at(AxisId a, double q, const AxisLimits& lim,
                                std::string& err) {
  const double band = 0.5 * kDeg2Rad;
  int in_band = 0;
  std::vector<ResponseSample> scratch;
  bool ok = run(a, q, cfg_.speed_rad_s, cfg_.max_move_ticks,
                [&](const AxisSnapshot& s) {
                  const bool in = std::fabs(s.q_rad - q) <= band &&
                                  std::fabs(s.v_rad_s) <= cfg_.at_rest_vel_rad_s;
                  in_band = in ? in_band + 1 : 0;
                  return in_band >= cfg_.settle_ticks;
                },
                &scratch, err);
  return ok;
}

AxisProfileMetrics PayloadProfiler::profile_axis(AxisId a, double q_start_rad,
                                                 const AxisLimits& lim,
                                                 std::string& err) {
  AxisProfileMetrics m;
  const double A = std::min(cfg_.step_amplitude_rad,
                            cfg_.region_half_span_rad - 3.0 * kDeg2Rad);
  const double T = std::min(cfg_.triangle_amplitude_rad, A);
  const double lo = cfg_.region_center_rad - cfg_.region_half_span_rad;
  const double hi = cfg_.region_center_rad + cfg_.region_half_span_rad;
  if (q_start_rad < lo || q_start_rad > hi) {
    err = "start pose outside the safe central region";
    return m;
  }

  // 0. Settle at the start pose.
  if (!settle_at(a, clamp_target(a, q_start_rad, lim), lim, err)) return m;

  // 1. Low-amplitude step, +.
  {
    std::vector<ResponseSample> s;
    const double t1 = clamp_target(a, q_start_rad + A, lim);
    if (!run(a, t1, cfg_.speed_rad_s, cfg_.max_move_ticks,
             [&](const AxisSnapshot& x) {
               return std::fabs(x.q_rad - t1) <=
                          cfg_.settle_band_frac * A &&
                      std::fabs(x.v_rad_s) <= cfg_.at_rest_vel_rad_s;
             },
             &s, err))
      return m;
    m.step_pos = analyze_step(s, q_start_rad, t1, cfg_.settle_band_frac);
    if (!settle_at(a, clamp_target(a, q_start_rad, lim), lim, err)) return m;
  }

  // 2. Low-amplitude step, -.
  {
    std::vector<ResponseSample> s;
    const double t1 = clamp_target(a, q_start_rad - A, lim);
    if (!run(a, t1, cfg_.speed_rad_s, cfg_.max_move_ticks,
             [&](const AxisSnapshot& x) {
               return std::fabs(x.q_rad - t1) <=
                          cfg_.settle_band_frac * A &&
                      std::fabs(x.v_rad_s) <= cfg_.at_rest_vel_rad_s;
             },
             &s, err))
      return m;
    m.step_neg = analyze_step(s, q_start_rad, t1, cfg_.settle_band_frac);
    if (!settle_at(a, clamp_target(a, q_start_rad, lim), lim, err)) return m;
  }

  // 3. Low-amplitude triangle (tracking RMS per leg).
  {
    double rms = 0.0;
    for (int leg = 0; leg < 2; ++leg) {
      const double t1 = clamp_target(a, q_start_rad + (leg == 0 ? T : -T), lim);
      std::vector<ResponseSample> s;
      if (!run(a, t1, cfg_.speed_rad_s, cfg_.max_move_ticks,
               [&](const AxisSnapshot& x) {
                 return std::fabs(x.q_rad - t1) <=
                            cfg_.settle_band_frac * T &&
                        std::fabs(x.v_rad_s) <= cfg_.at_rest_vel_rad_s;
               },
               &s, err))
        return m;
      rms = std::max(rms, rms_tracking_error(s, t1));
    }
    m.triangle_rms_rad = rms;
    if (!settle_at(a, clamp_target(a, q_start_rad, lim), lim, err)) return m;
  }

  // 4. Braking from incrementally increasing speeds (§44). Keep the highest
  //    speed that stopped cleanly.
  double max_verified_v = 0.0;
  for (double v : cfg_.brake_speeds_rad_s) {
    if (!settle_at(a, clamp_target(a, q_start_rad, lim), lim, err)) return m;
    const double D = std::min(v * 1.0,
                              cfg_.region_half_span_rad - 3.0 * kDeg2Rad);
    std::vector<ResponseSample> s;
    // Cruise up to the speed.
    if (!run(a, clamp_target(a, q_start_rad + D, lim), v,
             cfg_.max_move_ticks / 2,
             [&](const AxisSnapshot& x) {
               return std::fabs(x.v_rad_s) >= 0.8 * v;
             },
             &s, err)) {
      continue;  // could not reach cruise: skip this speed
    }
    // At-rest threshold strictly below the cruise speed (a 2 deg/s test
    // speed is below the generic 0.05 rad/s at-rest threshold).
    const double rest_vel = std::min(cfg_.at_rest_vel_rad_s, 0.4 * v);
    const std::size_t brake_idx = s.size();
    // Brake: pin the reference to the current position, keep the speed limit
    // (the drive decelerates under its own dynamics).
    const double q_brake = s.back().q_rad;
    bool stopped = run(a, q_brake, v, cfg_.max_move_ticks / 2,
                       [&](const AxisSnapshot& x) {
                         return std::fabs(x.v_rad_s) < rest_vel;
                       },
                       &s, err);
    if (!stopped) continue;
    BrakeMetrics br = analyze_brake(s, brake_idx, rest_vel);
    if (br.valid) {
      m.brake = br;
      max_verified_v = std::max(max_verified_v, br.v0_rad_s);
    }
  }

  // 5. Holding effort at the check angle.
  {
    const double t1 = clamp_target(a, q_start_rad + cfg_.hold_offset_rad, lim);
    if (settle_at(a, t1, lim, err)) {
      std::vector<ResponseSample> s;
      std::string hold_err;  // the capture runs the full hold (no stop)
      run(a, t1, 0.0, cfg_.hold_ticks,
          [](const AxisSnapshot&) { return false; }, &s, hold_err);
      // Mean |effort| over the last half of the capture (steady hold).
      double sum = 0.0;
      int n = 0;
      for (std::size_t i = s.size() / 2; i < s.size(); ++i) {
        sum += std::fabs(s[i].effort_nm);
        ++n;
      }
      m.hold_effort_nm = n > 0 ? sum / n : 0.0;
      if (!settle_at(a, clamp_target(a, q_start_rad, lim), lim, err))
        return m;
    }
  }

  m.valid = m.step_pos.valid && m.step_neg.valid;
  m.max_verified_speed_rad_s = max_verified_v;
  return m;
}

void PayloadProfiler::derive_limits(const AxisProfileMetrics& m,
                                    const ProfilerConfig& cfg,
                                    AxisPayloadProfile& out) {
  // v: the highest speed the brake test stopped cleanly from, never above the
  // configured envelope (or the conservative test speed if no brake ran).
  out.v_max_rad_s = std::min(
      cfg.env_v_max_rad_s,
      m.brake.valid ? std::max(m.brake.v0_rad_s, cfg.speed_rad_s)
                    : cfg.speed_rad_s);
  // a: the deceleration the drive actually demonstrated in the brake test
  // (v0 / stop time), with a small floor so an empty profile stays usable.
  double a = 1.0 * kDeg2Rad;  // floor
  if (m.brake.valid && m.brake.stop_time_s > 0.01)
    a = m.brake.v0_rad_s / m.brake.stop_time_s;
  out.a_max_rad_s2 = std::min(cfg.env_a_max_rad_s2, std::max(a, 1.0 * kDeg2Rad));
  // j: conservative default (jerk measurement needs higher-rate data than the
  // 200 Hz feedback provides).
  out.j_max_rad_s3 = 3.0 * out.a_max_rad_s2;
  out.baseline = m;
}

}  // namespace payload
}  // namespace ota
