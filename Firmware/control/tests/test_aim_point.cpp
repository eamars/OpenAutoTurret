// The head-aim rule is the operator's acceptance rule, stated in the revision and in the
// operator's own words: the reticle belongs on the target's head, within a third of the box
// height. These tests exist because "centred" meant "box centroid" for the whole of v3, which
// for a standing person is a torso. They were written against a stub that returned the anchor
// unchanged, and failed - which is the only reason to believe they can detect the absence of the
// behaviour rather than merely restate the implementation.
#include "tracking/aim_point.hpp"

#include <gtest/gtest.h>

namespace {

using ota::geo::CameraIntrinsics;
using ota::tracking::AimOptions;
using ota::tracking::aim_point_px;

// The station's measured intrinsics (calibration/camera_intrinsics.yaml, 1920x1080 stream).
CameraIntrinsics station() {
  CameraIntrinsics in;
  in.fx = 1389.0;
  in.fy = 1467.0;
  in.cx = 960.0;
  in.cy = 540.0;
  in.width = 1920;
  in.height = 1080;
  return in;
}

// A standing person's box, and the centroid anchor a detector would report for it.
constexpr double kXmin = 0.45, kYmin = 0.40, kXmax = 0.55, kYmax = 0.76;
constexpr double kAnchorU = 960.0;
constexpr double kAnchorV = 626.4;  // (0.40+0.76)/2 * 1080

TEST(AimPoint, CentroidFollowingIsUnchangedWhenHeadAimingIsOff) {
  AimOptions off;
  off.aim_at_head = false;
  const auto a = aim_point_px(kAnchorU, kAnchorV, kXmin, kYmin, kXmax, kYmax, station(), off);
  EXPECT_DOUBLE_EQ(a.u_px, kAnchorU);
  EXPECT_DOUBLE_EQ(a.v_px, kAnchorV);
  EXPECT_FALSE(a.head_applied) << "the cue must not claim head aiming while following the anchor";
}

TEST(AimPoint, HeadAimingMovesThePointIntoTheHeadNotTheTorso) {
  AimOptions on;
  on.aim_at_head = true;
  on.head_fraction_from_top = 0.22;
  const auto a = aim_point_px(kAnchorU, kAnchorV, kXmin, kYmin, kXmax, kYmax, station(), on);
  EXPECT_TRUE(a.head_applied);
  EXPECT_NEAR(a.v_px, (kYmin + 0.22 * (kYmax - kYmin)) * 1080.0, 1e-9);
  EXPECT_NEAR(a.u_px, 0.5 * (kXmin + kXmax) * 1920.0, 1e-9);
  // v grows downward, so head-ward means smaller.
  EXPECT_LT(a.v_px, kAnchorV);
  // The whole point of the rule: the head point is more than a tenth of the box height away from
  // the centroid, so a controller that followed the centroid could not have been satisfying it.
  const double box_h_px = (kYmax - kYmin) * 1080.0;
  EXPECT_GT(kAnchorV - a.v_px, 0.1 * box_h_px);
}

TEST(AimPoint, NoUseableBoxMeansNoHeadAndSaysSo) {
  AimOptions on;
  on.aim_at_head = true;

  const auto none = aim_point_px(700, 300, 0, 0, 0, 0, station(), on);
  EXPECT_FALSE(none.head_applied);
  EXPECT_DOUBLE_EQ(none.u_px, 700);
  EXPECT_DOUBLE_EQ(none.v_px, 300);

  const auto flat = aim_point_px(700, 300, 0.4, 0.5, 0.6, 0.5, station(), on);
  EXPECT_FALSE(flat.head_applied);

  // Uncalibrated intrinsics must not produce invented pixels.
  CameraIntrinsics bad;
  bad.fx = 0.0;
  const auto uncal = aim_point_px(700, 300, kXmin, kYmin, kXmax, kYmax, bad, on);
  EXPECT_FALSE(uncal.head_applied);
  EXPECT_DOUBLE_EQ(uncal.u_px, 700);
}

TEST(AimPoint, ANonsenseFractionCannotAimOutsideTheTargetsOwnBox) {
  AimOptions on;
  on.aim_at_head = true;
  on.head_fraction_from_top = 1.7;  // 170 % below the top edge: outside the box
  const auto a = aim_point_px(kAnchorU, kAnchorV, kXmin, kYmin, kXmax, kYmax, station(), on);
  EXPECT_TRUE(a.head_applied);
  EXPECT_LE(a.v_px, kYmax * 1080.0 + 1e-9);
  EXPECT_GE(a.v_px, kYmin * 1080.0 - 1e-9);

  on.head_fraction_from_top = -0.5;
  const auto b = aim_point_px(kAnchorU, kAnchorV, kXmin, kYmin, kXmax, kYmax, station(), on);
  EXPECT_GE(b.v_px, kYmin * 1080.0 - 1e-9);
}

}  // namespace
