#include "ota_test_paths.hpp"
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
const auto firmware = ota_test_firmware_dir(std::filesystem::path(__FILE__).parent_path().parent_path().parent_path());

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
  EXPECT_DOUBLE_EQ(cfg.alignment.wide.right_mm, 75);
  EXPECT_DOUBLE_EQ(cfg.alignment.wide.up_mm, 75);
  EXPECT_DOUBLE_EQ(cfg.alignment.detail.right_mm, 75);
}

TEST(AlignmentConfig, ManualDepthProducesExpectedReticle) {
  auto text = source_config();
  replace(text, "mode: off", "mode: manual_depth");
  auto parsed = load_text(text);
  ASSERT_TRUE(parsed.ok);
  auto cfg = config::make_tracking_config(parsed.config);
  const auto a = geo::bore_alignment(cfg.alignment, geo::CameraRole::Wide, cfg.intrinsics);
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
      {"assumed_depth_m: 10", "assumed_depth_m: 0"}, {"right_mm: 75", "right_mm: .inf"},
      // Single-line patterns: a release's YAML has CRLF line ends (Windows checkout, git archive).
      {"wide:   {right_mm: 75, up_mm: 75, forward_mm: 0, right_deg: 0,",
       "wide:   {right_mm: 75, up_mm: 75, forward_mm: 0, right_deg: 90,"},
      {"detail: {right_mm: 75", "detail: {right_mm: 75, typo: 1"},
      {"detail: {right_mm: 75, up_mm: 75, forward_mm: 0, right_deg: 0, up_deg: 0}", "detail_missing: 1"},
      {"wide:   {right_mm: 75, up_mm: 75, forward_mm: 0, right_deg: 0, up_deg: 0}", "wide: null"},
      {"camera_offset:", "camera_from_bore_mm:"}}) {
    SCOPED_TRACE(value);
    auto text = source_config(); replace(text, old, value);
    EXPECT_FALSE(load_text(text).ok);
  }
  auto parsed = load_text(source_config());
  parsed.config.alignment.enabled = true;
  parsed.config.alignment.wide.right_mm = 1e9;
  EXPECT_THROW(config::make_tracking_config(parsed.config), std::invalid_argument);
}

TEST(AlignmentConfig, MissingNewBlocksPreservesLegacyAndExplicitModeWins) {
  auto text = source_config();
  const auto begin = text.find("  aim_point:");
  const auto end = text.find("  search_enabled_by_default:", begin);
  text.replace(begin, end-begin, "  aim_at_head: true\n  head_fraction_from_top: 0.3\n");
  const auto alignment = text.find("alignment:");
  const auto shutdown = text.find("shutdown:", alignment);
  ASSERT_NE(alignment, std::string::npos);
  ASSERT_NE(shutdown, std::string::npos);
  text.erase(alignment, shutdown-alignment);
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
  constexpr auto W = geo::CameraRole::Wide;
  geo::BoreAlignmentConfig c;
  c.enabled = true; c.wide.right_mm = c.wide.up_mm = 75;
  const geo::CameraIntrinsics in;
  auto a = geo::bore_alignment(c, W, in);
  c.assumed_depth_m = 20;
  auto far = geo::bore_alignment(c, W, in);
  EXPECT_NEAR((a.u_norm-.5)/2, far.u_norm-.5, 1e-12);
  // A camera right of and above the bore sees it to the left and below; the other way round, the
  // other way.
  EXPECT_LT(a.u_norm, .5); EXPECT_GT(a.v_norm, .5);
  c.wide.right_mm = -75; c.wide.up_mm = -75;
  a = geo::bore_alignment(c, W, in);
  EXPECT_GT(a.u_norm, .5); EXPECT_LT(a.v_norm, .5);
  // A camera that looks right of and up from the bore sees it to the left and below.
  c.wide.right_mm = c.wide.up_mm = 0;
  c.wide.right_deg = 1; c.wide.up_deg = 2;
  a = geo::bore_alignment(c, W, in);
  EXPECT_LT(a.u_norm, .5); EXPECT_GT(a.v_norm, .5);
  c.wide.forward_mm = -30000;
  EXPECT_FALSE(geo::bore_alignment(c, W, in).valid);
}

TEST(AlignmentGeometry, EachCameraHasItsOwnOffset) {
  geo::BoreAlignmentConfig c;
  c.enabled = true;
  c.wide.right_mm = 75;
  c.detail.right_mm = -40;   // the detail camera on the bore's other side
  const geo::CameraIntrinsics in;
  const auto wide = geo::bore_alignment(c, geo::CameraRole::Wide, in);
  const auto detail = geo::bore_alignment(c, geo::CameraRole::Detail, in);
  ASSERT_TRUE(wide.valid && detail.valid);
  EXPECT_LT(wide.u_norm, .5);
  EXPECT_GT(detail.u_norm, .5);
}

TEST(AlignmentGeometry, PointingIsAnExactRotationNotATangentSum) {
  // A camera looking 10 deg right and 10 deg up of the bore. The bore's direction seen from the
  // camera is that rotation undone: turning it by Ry(right) Rx(up) must give the bore axis back.
  geo::BoreAlignmentConfig c;
  c.enabled = true;
  c.assumed_depth_m = 1e6;
  c.wide.right_deg = 10; c.wide.up_deg = 10;
  geo::CameraIntrinsics in;
  in.fx = in.fy = 1000; in.cx = 960; in.cy = 540; in.width = 1920; in.height = 1080;
  const auto a = geo::bore_alignment(c, geo::CameraRole::Wide, in);
  ASSERT_TRUE(a.valid);
  const double r = 10*M_PI/180;
  const auto& d = a.direction_camera;
  const double y1 = std::cos(r)*d.y - std::sin(r)*d.z, z1 = std::sin(r)*d.y + std::cos(r)*d.z;
  const double x2 = std::cos(r)*d.x + std::sin(r)*z1, z2 = -std::sin(r)*d.x + std::cos(r)*z1;
  EXPECT_NEAR(x2, 0, 1e-12); EXPECT_NEAR(y1, 0, 1e-12); EXPECT_NEAR(z2, 1, 1e-12);
  // The two-tangent shortcut the old model used puts the mark tan(r) (1/cos(u) - 1) off
  // horizontally: 2.7 mrad here (2.7 px), negligible at 1 deg x 1 deg (3 urad).
  const double tangent_u = in.cx - in.fx*std::tan(r);
  EXPECT_NEAR(a.u_norm*in.width - tangent_u, -in.fx*std::tan(r)*(1/std::cos(r) - 1), 1e-6);
}

namespace {
// The station's wide camera (calibration/camera_intrinsics.yaml, 2026-09-04).
geo::CameraIntrinsics station_wide() {
  geo::CameraIntrinsics in;
  in.fx = 1389; in.fy = 1467; in.cx = 960; in.cy = 540; in.width = 1920; in.height = 1080;
  return in;
}
geo::BoreAlignment mark_at(geo::CameraOffset o, double depth_m) {
  geo::BoreAlignmentConfig c;
  c.enabled = true; c.assumed_depth_m = depth_m; c.wide = o;
  return geo::bore_alignment(c, geo::CameraRole::Wide, station_wide());
}
}

TEST(CameraZero, ATapGivesThePointingThatPutsTheMarkExactlyThere) {
  // ADR-006 D1: the solver is bore_alignment's inverse, at any installation offset, distance and
  // pointing a camera on this turret can have.
  const auto in = station_wide();
  for (const geo::CameraOffset installed : {geo::CameraOffset{155, -20, 0}, geo::CameraOffset{145, 40, 0},
                                            geo::CameraOffset{-80, 120, 35}, geo::CameraOffset{}})
    for (double depth : {3.0, 10.0, 37.5, 250.0, 1500.0})
      for (const auto& [right, up] : std::initializer_list<std::pair<double, double>>{
               {0, 0}, {1.2, -0.8}, {-2.5, 1.7}, {6, 4}}) {
        SCOPED_TRACE(testing::Message() << installed.right_mm << "," << installed.up_mm << " mm at " << depth
                                        << " m, pointing " << right << "," << up);
        geo::CameraOffset truth = installed;
        truth.right_deg = right; truth.up_deg = up;
        const auto tap = mark_at(truth, depth);
        ASSERT_TRUE(tap.valid) << tap.reason;
        const auto z = geo::solve_camera_pointing(installed, depth, tap.u_norm, tap.v_norm, in);
        ASSERT_TRUE(z.valid) << z.reason;
        EXPECT_NEAR(z.right_deg, right, 1e-7);
        EXPECT_NEAR(z.up_deg, up, 1e-7);
      }
}

TEST(CameraZero, OneZeroWithTheRightOffsetsHoldsAtEveryDistance) {
  // The owner's offsets (2026-10-24, bore frame): wide 155 mm right and 20 mm below the payload's aim
  // axis, pointing slightly off. Zeroed on a target at 25 m, the mark at any other distance is where
  // the true camera sees the aim axis: only the parallax changes, and the model carries it.
  const geo::CameraOffset installed{155, -20, 0};
  geo::CameraOffset truth = installed;
  truth.right_deg = 0.9; truth.up_deg = -0.4;
  const auto tap = mark_at(truth, 25);
  const auto z = geo::solve_camera_pointing(installed, 25, tap.u_norm, tap.v_norm, station_wide());
  ASSERT_TRUE(z.valid);
  geo::CameraOffset zeroed = installed;
  zeroed.right_deg = z.right_deg; zeroed.up_deg = z.up_deg;
  for (double depth : {5.0, 10.0, 100.0, 1500.0}) {
    const auto want = mark_at(truth, depth), got = mark_at(zeroed, depth);
    EXPECT_NEAR(got.u_norm*1920, want.u_norm*1920, 1e-6) << depth;
    EXPECT_NEAR(got.v_norm*1080, want.v_norm*1080, 1e-6) << depth;
  }
  // The parallax is real: 155 mm is 15.5 mrad at 10 m and 0.1 at 1500 m, so the mark moves ~21 px
  // across the wide picture between them.
  EXPECT_NEAR((mark_at(zeroed, 1500).u_norm - mark_at(zeroed, 10).u_norm)*1920, 1389*(0.155/10 - 0.155/1500), 0.2);
}

TEST(CameraZero, AnOffsetErrorCostsItsParallaxAwayFromTheZeroDistance) {
  // ADR-006 risk 1: the zero is exact at its distance whatever the offsets; 5 mm wrong costs
  // 5 mm x (1/D - 1/D_zero) elsewhere -- 0.45 mrad at 10 m after a zero at 100 m.
  geo::CameraOffset truth{155, -20, 0, 0.5, 0.3};
  const auto tap = mark_at(truth, 100);
  const geo::CameraOffset believed{150, -20, 0};
  const auto z = geo::solve_camera_pointing(believed, 100, tap.u_norm, tap.v_norm, station_wide());
  ASSERT_TRUE(z.valid);
  geo::CameraOffset zeroed = believed;
  zeroed.right_deg = z.right_deg; zeroed.up_deg = z.up_deg;
  EXPECT_NEAR(mark_at(zeroed, 100).u_norm, tap.u_norm, 1e-12) << "exact at the zero's distance";
  const double err_mrad = (mark_at(zeroed, 10).u_norm - mark_at(truth, 10).u_norm)*1920/1389*1000;
  EXPECT_NEAR(std::abs(err_mrad), 5e-3*(1/10.0 - 1/100.0)*1000, 0.02);
}

TEST(CameraZero, RefusesWhatCannotBeAZero) {
  const auto in = station_wide();
  const geo::CameraOffset o{155, -20, 0};
  EXPECT_FALSE(geo::solve_camera_pointing(o, 0, .5, .5, in).valid);
  EXPECT_FALSE(geo::solve_camera_pointing(o, -3, .5, .5, in).valid);
  EXPECT_FALSE(geo::solve_camera_pointing(o, 10, 1.2, .5, in).valid);
  EXPECT_FALSE(geo::solve_camera_pointing(o, 10, .5, .5, geo::CameraIntrinsics{0, 0, 0, 0, 0, 0}).valid);
  // A tap at the picture's edge from a camera 2 cm from the bore at 1 m is 34 deg off: a zero, but
  // nothing like this turret; the 45 deg rule (bore_alignment's) is what refuses a wild one.
  EXPECT_TRUE(geo::solve_camera_pointing(geo::CameraOffset{20, 0, 0}, 1, 0.0, .5, in).valid);
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
