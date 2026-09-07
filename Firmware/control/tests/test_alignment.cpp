#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <limits>
#include <unistd.h>
#include "config/tracking_setup.hpp"
#include "control/reference_manager.hpp"
#include "web/web_server.hpp"

using namespace ota;
namespace {
const auto firmware = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path();

std::string source_config() {
  std::ifstream in(firmware / "config/turret.yaml");
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

config::LoadResult load_text(const std::string& text) {
  char name[] = "/tmp/ota-alignment-XXXXXX";
  int fd = mkstemp(name);
  if (fd < 0) throw std::runtime_error("cannot create test config");
  close(fd);
  { std::ofstream out(name); out << text; }
  auto result = config::load_turret_config(name);
  std::filesystem::remove(name);
  result.config.camera.intrinsics_file = (firmware / "calibration/camera_intrinsics.yaml").string();
  result.config.camera.extrinsics_file = (firmware / "calibration/camera_extrinsics.yaml").string();
  return result;
}

void replace(std::string& text, const std::string& old, const std::string& value) {
  const auto p = text.find(old);
  if (p == std::string::npos) throw std::runtime_error("test fixture changed: " + old);
  text.replace(p, old.size(), value);
}
}

TEST(AlignmentConfig, ShippedConfigurationReachesProductionSetup) {
  auto parsed = load_text(source_config());
  ASSERT_TRUE(parsed.ok);
  const auto cfg = config::make_tracking_config(parsed.config);
  EXPECT_EQ(cfg.aim.mode, tracking::AimMode::BoxFraction);
  EXPECT_DOUBLE_EQ(cfg.aim.y_fraction, .22);
  EXPECT_FALSE(cfg.alignment.enabled);
  EXPECT_DOUBLE_EQ(cfg.alignment.camera_right_mm, 75);
  EXPECT_DOUBLE_EQ(cfg.alignment.camera_up_mm, 75);
}

TEST(AlignmentConfig, ManualDepthProducesExpectedReticle) {
  auto text = source_config();
  replace(text, "mode: off", "mode: manual_depth");
  auto parsed = load_text(text);
  ASSERT_TRUE(parsed.ok);
  auto cfg = config::make_tracking_config(parsed.config);
  const auto a = geo::laser_alignment(cfg.alignment, cfg.intrinsics);
  ASSERT_TRUE(a.valid);
  EXPECT_NEAR(a.u_norm*1920, 949.5825, 1e-9);
  EXPECT_NEAR(a.v_norm*1080, 551.0025, 1e-9);
  parsed.config.camera.intrinsics_file += ".missing";
  EXPECT_THROW(config::make_tracking_config(parsed.config), std::invalid_argument);
  parsed = load_text(text);
  parsed.config.camera.extrinsics_file += ".missing";
  EXPECT_THROW(config::make_tracking_config(parsed.config), std::invalid_argument);
}

TEST(AlignmentConfig, RejectsBadValuesInsteadOfDefaulting) {
  for (const auto& [old, value] : std::initializer_list<std::pair<std::string, std::string>>{
      {"y_fraction: 0.22", "y_fraction: .nan"}, {"y_fraction: 0.22", "y_fraction: true"},
      {"y_fraction: 0.22", "y_fraction: 1.1"}, {"mode: box_fraction", "mode: typo"},
      {"x_fraction: 0.50", "x_fracton: 0.50"}, {"mode: off", "mode: measured"},
      {"assumed_depth_m: 10", "assumed_depth_m: 0"}, {"right: 75", "right: .inf"},
      {"laser_axis_deg: {right: 0, up: 0}", "laser_axis_deg: {right: 90, up: 0}"},
      {"camera_from_laser_mm: {right: 75, up: 75, forward: 0}", "camera_from_laser_mm: null"}}) {
    SCOPED_TRACE(value);
    auto text = source_config(); replace(text, old, value);
    EXPECT_FALSE(load_text(text).ok);
  }
  auto parsed = load_text(source_config());
  parsed.config.alignment.enabled = true;
  parsed.config.alignment.camera_right_mm = 1e9;
  EXPECT_THROW(config::make_tracking_config(parsed.config), std::invalid_argument);
}

TEST(AlignmentConfig, MissingNewBlocksPreservesLegacyAndExplicitModeWins) {
  auto text = source_config();
  const auto begin = text.find("  aim_point:");
  const auto end = text.find("  search_enabled_by_default:", begin);
  text.replace(begin, end-begin, "  aim_at_head: true\n  head_fraction_from_top: 0.3\n");
  const auto alignment = text.find("alignment:\n");
  text.erase(alignment, text.find("shutdown:", alignment)-alignment);
  auto parsed = load_text(text);
  ASSERT_TRUE(parsed.ok);
  auto cfg = config::make_tracking_config(parsed.config);
  EXPECT_EQ(cfg.aim.mode, tracking::AimMode::Legacy);
  EXPECT_TRUE(cfg.aim.aim_at_head);
  EXPECT_DOUBLE_EQ(cfg.aim.head_fraction_from_top, .3);
  EXPECT_FALSE(cfg.alignment.enabled);
  auto point = tracking::aim_point_px(900, 500, .2, .1, .8, .9, cfg.intrinsics, cfg.aim, true);
  EXPECT_DOUBLE_EQ(point.v_px, 500);
  text = source_config();
  replace(text, "  aim_point:", "  aim_at_head: false\n  head_fraction_from_top: .9\n  aim_point:");
  parsed = load_text(text);
  ASSERT_TRUE(parsed.ok);
  cfg = config::make_tracking_config(parsed.config);
  point = tracking::aim_point_px(900, 500, .2, .1, .8, .9, cfg.intrinsics, cfg.aim, true);
  EXPECT_NEAR(point.v_px, .276*1080, 1e-9);
  EXPECT_STREQ(point.source, "box_fraction");
}

TEST(AlignmentGeometry, SignsDepthAndAngles) {
  geo::LaserAlignmentConfig c;
  c.enabled = true; c.camera_right_mm = c.camera_up_mm = 75;
  const geo::CameraIntrinsics in;
  auto a = geo::laser_alignment(c, in);
  c.assumed_depth_m = 20;
  auto far = geo::laser_alignment(c, in);
  EXPECT_NEAR((a.u_norm-.5)/2, far.u_norm-.5, 1e-12);
  c.camera_right_mm = -75; c.camera_up_mm = -75;
  a = geo::laser_alignment(c, in);
  EXPECT_GT(a.u_norm, .5); EXPECT_LT(a.v_norm, .5);
  c.camera_right_mm = c.camera_up_mm = 0;
  c.laser_right_deg = 1; c.laser_up_deg = 2;
  a = geo::laser_alignment(c, in);
  EXPECT_GT(a.u_norm, .5); EXPECT_LT(a.v_norm, .5);
  c.camera_forward_mm = -30000;
  EXPECT_FALSE(geo::laser_alignment(c, in).valid);
}

TEST(AlignmentGeometry, BoundedAndIterativeSolversUseSameSight) {
  auto kin = geo::TurretKinematics::aligned();
  const geo::Vec3 sight{-.04, .03, 1};
  geo::LosJointSolver solver(kin, sight);
  const auto target = kin.ray_to_base(sight, .1, -.2);
  double az, el, yaw, pitch;
  geo::TurretKinematics::base_ray_to_los(target, az, el);
  ASSERT_TRUE(solver.solve_within_limits(az, el, 0, 0, -.5, .5, -.5, .5, yaw, pitch));
  EXPECT_NEAR(yaw, .1, 1e-9); EXPECT_NEAR(pitch, -.2, 1e-9);
  ASSERT_TRUE(solver.solve_from_pose(az, el, 0, 0, yaw, pitch));
  EXPECT_NEAR(yaw, .1, 1e-7); EXPECT_NEAR(pitch, -.2, 1e-7);
  EXPECT_FALSE(solver.solve_within_limits(az, el, 0, 0, -.05, .05, -.05, .05, yaw, pitch));
  geo::LosJointSolver bad(kin, {0, 0, 0});
  EXPECT_FALSE(bad.solve_from_pose(az, el, 0, 0, yaw, pitch));
}

TEST(AlignmentGeometry, SightIsLimitedToAutoTrackIntents) {
  ReferenceManager manager{geo::LosJointSolver(geo::TurretKinematics::aligned())};
  MotionIntent in;
  in.type = IntentType::LosDirection; in.has_los = true;
  in.sight_camera = {-.04, .03, 1};
  ReferenceManager::IntentLimits limits;
  in.source = MotionSource::AutoTrack;
  const auto aligned = manager.resolve(in, limits);
  EXPECT_TRUE(aligned.is_tracking_reference);
  EXPECT_GT(std::hypot(aligned.q_yaw_rad, aligned.q_pitch_rad), .04);
  in.source = MotionSource::AutoRoam;
  const auto optical = manager.resolve(in, limits);
  EXPECT_DOUBLE_EQ(optical.q_yaw_rad, 0); EXPECT_DOUBLE_EQ(optical.q_pitch_rad, 0);
  in.source = MotionSource::AutoTrack; in.sight_camera = {0, 0, -1};
  EXPECT_TRUE(manager.resolve(in, limits).target_unreachable);
}

TEST(AlignmentGeometry, InvalidBoxFallsBackAndClippedBoxIsDeclared) {
  tracking::AimOptions opt;
  opt.mode = tracking::AimMode::BoxFraction;
  auto p = tracking::aim_point_px(900, 500, .8, .2, .2, .9, {}, opt, true);
  EXPECT_DOUBLE_EQ(p.v_px, 500); EXPECT_STREQ(p.source, "invalid_box_anchor_fallback");
  p = tracking::aim_point_px(900, 500, .2, 0, .8, 1, {}, opt, true);
  EXPECT_TRUE(p.box_clipped);
  opt.y_fraction = std::numeric_limits<double>::quiet_NaN();
  TrackingController::Config cfg; cfg.aim = opt;
  EXPECT_THROW(TrackingController{cfg}, std::invalid_argument);
}
