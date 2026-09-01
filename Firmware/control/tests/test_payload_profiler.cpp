// Tests for the standalone payload profiler battery (§44) driving the
// SimMotorBackend with a synthetic 5 ms clock and a no-op pacer — no CAN,
// no motor, no real-time waits.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "control/safety_envelope.hpp"
#include "payload/payload_profiler.hpp"
#include "sim/sim_motor_backend.hpp"

namespace {

using ota::AxisId;
using ota::AxisLimits;
using ota::TimeNs;
using ota::payload::AxisPayloadProfile;
using ota::payload::AxisProfileMetrics;
using ota::payload::PayloadProfiler;
using ota::payload::ProfilerConfig;
using ota::sim::SimMotorBackend;
constexpr double kDeg = M_PI / 180.0;

// A profiler rig: sim plant + fake clock + no-op pacer.
class ProfilerRig {
 public:
  ProfilerRig() : sim_(0.005), t_(0), prof_(sim_, cfg_, ts(), pacer()) {}
  void energize_all() {
    std::string err;
    sim_.enter_position_mode(AxisId::Pitch, 0.5, err);
    sim_.enter_position_mode(AxisId::Yaw, 0.5, err);
  }
  AxisProfileMetrics profile(AxisId a, double q0, const AxisLimits& lim,
                             std::string& err) {
    return prof_.profile_axis(a, q0, lim, err);
  }
  SimMotorBackend& sim() { return sim_; }
  ProfilerConfig& cfg() { return cfg_; }

 private:
  SimMotorBackend sim_;
  TimeNs t_ = 0;
  ProfilerConfig cfg_;
  PayloadProfiler::TimeSource ts() {
    return [this]() { return t_ += 5'000'000; };
  }
  PayloadProfiler::Pacer pacer() { return [](void) {}; }
  PayloadProfiler prof_;
};

}  // namespace

TEST(PayloadProfiler, FullBatteryProducesValidMetrics) {
  ProfilerRig rig;
  rig.energize_all();
  std::string err;
  AxisLimits lim;  // invalid: rely on the safe central region
  const AxisProfileMetrics m = rig.profile(AxisId::Pitch, 0.0, lim, err);
  ASSERT_TRUE(m.valid) << err;
  ASSERT_TRUE(m.step_pos.valid);
  ASSERT_TRUE(m.step_neg.valid);

  // The sim is first-order with a 5 deg/s... no: the conservative test speed
  // is 10 deg/s, so a 3 deg step is speed-limited: rise ~ A/v + tau*ln(9)
  // territory, bounded well inside [0.15 s, 0.5 s].
  EXPECT_GT(m.step_pos.rise_time_s, 0.15);
  EXPECT_LT(m.step_pos.rise_time_s, 0.5);
  EXPECT_LT(m.step_pos.overshoot, 1e-6);
  EXPECT_GT(m.step_pos.peak_effort_nm, 0.0);

  // Braking from the highest test speed (20 deg/s): v0 within 20 %, and a
  // small but non-zero stop distance (the sim decays in one tick when the
  // reference is pinned, so keep the upper bound generous).
  ASSERT_TRUE(m.brake.valid);
  EXPECT_NEAR(m.brake.v0_rad_s, 20.0 * kDeg, 0.2 * 20.0 * kDeg);
  EXPECT_GT(m.brake.stop_distance_rad, 0.0);
  EXPECT_LT(m.brake.stop_distance_rad, 0.05);
  EXPECT_GT(m.brake.stop_time_s, 0.0);

  // Triangle RMS: the legs include the travel transient (up to 2 deg), so
  // the RMS over the whole leg is small but non-negligible.
  EXPECT_GT(m.triangle_rms_rad, 0.0);
  EXPECT_LT(m.triangle_rms_rad, 0.05);
  EXPECT_GE(m.hold_effort_nm, 0.0);
  EXPECT_NEAR(m.max_verified_speed_rad_s, m.brake.v0_rad_s, 1e-9);

  // The axis returns to its start position (the battery is pose-neutral).
  EXPECT_NEAR(rig.sim().position(AxisId::Pitch), 0.0, 0.5 * kDeg);
}

TEST(PayloadProfiler, DerivedLimitsRespectTheEnvelope) {
  ProfilerRig rig;
  rig.energize_all();
  rig.cfg().env_v_max_rad_s = 15.0 * kDeg;  // tighter than the 20 deg/s test
  rig.cfg().env_a_max_rad_s2 = 5.0 * kDeg;
  std::string err;
  AxisLimits lim;
  const AxisProfileMetrics m = rig.profile(AxisId::Yaw, 0.0, lim, err);
  ASSERT_TRUE(m.valid) << err;
  AxisPayloadProfile out;
  PayloadProfiler::derive_limits(m, rig.cfg(), out);
  EXPECT_LE(out.v_max_rad_s, 15.0 * kDeg + 1e-9);
  EXPECT_GT(out.v_max_rad_s, 0.0);
  EXPECT_LE(out.a_max_rad_s2, 5.0 * kDeg + 1e-9);
  EXPECT_GT(out.a_max_rad_s2, 0.0);
  EXPECT_GT(out.j_max_rad_s3, 0.0);
}

TEST(PayloadProfiler, SoftLimitsClampTheBattery) {
  ProfilerRig rig;
  rig.energize_all();
  // A usable band narrower than the full battery: targets clamp to the
  // band (a 2 deg margin inside the soft limits) and the battery runs the
  // clamped amplitudes.
  AxisLimits lim;
  lim.set_from_endpoints(-6.0 * kDeg, 6.0 * kDeg, 1.0 * kDeg);  // soft +/-5 deg
  std::string err;
  const AxisProfileMetrics m = rig.profile(AxisId::Pitch, 0.0, lim, err);
  ASSERT_TRUE(m.valid) << err;
  // The axis never left the soft band (and in fact stayed inside the
  // clamped +/-3 deg working range).
  EXPECT_LE(std::fabs(rig.sim().position(AxisId::Pitch)), 3.5 * kDeg);
}

TEST(PayloadProfiler, DegenerateLimitsFailGracefully) {
  // A band so narrow that the 2 deg safety margin leaves no room to move:
  // every target clamps to the same point, the steps measure nothing, and
  // the profiler reports an INVALID profile. The unreachable cruise
  // sub-tests time out (that is the err string), but the axis never leaves
  // the band.
  ProfilerRig rig;
  rig.energize_all();
  AxisLimits lim;
  lim.set_from_endpoints(-2.0 * kDeg, 2.0 * kDeg, 0.5 * kDeg);  // soft +/-1.5 deg
  std::string err;
  const AxisProfileMetrics m = rig.profile(AxisId::Pitch, 0.0, lim, err);
  EXPECT_FALSE(m.valid);
  EXPECT_EQ(err, "profiler: sub-test timeout");
  EXPECT_LE(std::fabs(rig.sim().position(AxisId::Pitch)), 1.5 * kDeg);
}

TEST(PayloadProfiler, PayloadShiftChangesTheMeasuredResponse) {
  // Emulate a heavier payload: a larger position time constant must show up
  // as a slower measured response (this is exactly what §31.3 compares).
  ProfilerRig rig;
  rig.energize_all();
  std::string err;
  AxisLimits lim;
  const AxisProfileMetrics light = rig.profile(AxisId::Pitch, 0.0, lim, err);
  ASSERT_TRUE(light.valid) << err;
  // Re-energize the axis at its (returned-to) start pose.
  err.clear();
  rig.sim().enter_position_mode(AxisId::Pitch, 0.5, err);
  rig.sim().set_response_tau(AxisId::Pitch, 0.15);  // 3x inertia
  const AxisProfileMetrics heavy = rig.profile(AxisId::Pitch, 0.0, lim, err);
  ASSERT_TRUE(heavy.valid) << err;
  EXPECT_GT(heavy.step_pos.rise_time_s, light.step_pos.rise_time_s);
  EXPECT_GT(heavy.step_pos.settling_time_s, light.step_pos.settling_time_s);
}

TEST(PayloadProfiler, MissingFeedbackFailsTheAxis) {
  ProfilerRig rig;
  std::string err;
  AxisLimits lim;
  // Never energized: no position mode, no meaningful feedback motion.
  const AxisProfileMetrics m = rig.profile(AxisId::Pitch, 0.0, lim, err);
  EXPECT_FALSE(m.valid);
  EXPECT_FALSE(err.empty());
}
