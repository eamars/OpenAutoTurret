// Unit tests for the search planner (architecture §36, §49). Verifies the yaw
// sweep between bounds, the dwell + reverse at each bound, and the reduced
// speed limit. Pure planning.
#include <gtest/gtest.h>

#include "control/search_planner.hpp"

namespace {
using ota::SearchPlanner;
using ota::SearchPlannerConfig;

constexpr int64_t kMs = 1000 * 1000;
constexpr double kDeg = 3.14159265358979323846 / 180.0;

SearchPlannerConfig test_cfg() {
  SearchPlannerConfig c;
  c.yaw_low_rad = -45.0 * kDeg;
  c.yaw_high_rad = 45.0 * kDeg;
  c.pitch_rad = 0.0;
  c.v_max_rad_s = 10.0 * kDeg;
  c.dwell_s = 0.25;
  return c;
}

TEST(SearchPlanner, TargetsFartherBoundFirst) {
  SearchPlanner sp(test_cfg());
  // Start near the low bound: the first target should be the (farther) high
  // bound.
  auto out = sp.step(0, -40.0 * kDeg);
  EXPECT_NEAR(out.q_yaw_rad, 45.0 * kDeg, 1e-9);
  EXPECT_NEAR(out.q_pitch_rad, 0.0, 1e-9);
  EXPECT_NEAR(out.v_max_rad_s, 10.0 * kDeg, 1e-9);
}

TEST(SearchPlanner, ReversesAtBoundWithDwell) {
  SearchPlanner sp(test_cfg());
  sp.start(0.0);
  // Drive toward the high bound and reach it.
  int64_t t = 0;
  bool reached_high = false;
  for (int i = 0; i < 1000 && !reached_high; ++i) {
    t += kMs;  // 1 ms per step
    auto out = sp.step(t, 44.9 * kDeg);  // at the high bound
    if (out.dwelling) reached_high = true;
  }
  ASSERT_TRUE(reached_high) << "should dwell at the high bound";
  // After the dwell, the target should have reversed to the low bound.
  auto after = sp.step(t + static_cast<int64_t>(0.3 * 1e9), 44.9 * kDeg);
  EXPECT_NEAR(after.q_yaw_rad, -45.0 * kDeg, 1e-9);
}

TEST(SearchPlanner, SustainedSweepAlternatesBounds) {
  SearchPlanner sp(test_cfg());
  sp.start(0.0);
  // Simulate the yaw actually following the commanded target (fast enough to
  // reach the bounds), and record the distinct target bounds visited.
  double yaw = 0.0;
  bool saw_high = false, saw_low = false;
  int64_t t = 0;
  for (int i = 0; i < 5000; ++i) {
    t += 2 * kMs;  // 2 ms per step
    auto out = sp.step(t, yaw);
    if (std::fabs(out.q_yaw_rad - 45.0 * kDeg) < 1e-6) saw_high = true;
    if (std::fabs(out.q_yaw_rad + 45.0 * kDeg) < 1e-6) saw_low = true;
    // The yaw tracks the commanded target quickly (the trajectory generator
    // would do this with an S-curve).
    const double err = out.q_yaw_rad - yaw;
    yaw += std::copysign(std::min(0.1, std::fabs(err)), err);
  }
  EXPECT_TRUE(saw_high) << "should sweep to the high bound";
  EXPECT_TRUE(saw_low) << "should sweep to the low bound";
}

TEST(SearchPlanner, ReducedSpeedBelowTracking) {
  SearchPlanner sp(test_cfg());
  sp.start(0.0);
  auto out = sp.step(0, 0.0);
  EXPECT_LT(out.v_max_rad_s, 30.0 * kDeg);  // reduced vs tracking
}

}  // namespace
