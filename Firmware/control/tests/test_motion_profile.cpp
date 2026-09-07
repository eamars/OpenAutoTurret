#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include "config/turret_config.hpp"
#include "config/station_wiring.hpp"
#include "control/speed_servo.hpp"
#include "control/boundary_governor.hpp"
#include "control/tracking_reference.hpp"

using namespace ota;
namespace {
std::string station_config() {
  return (std::filesystem::path(__FILE__).parent_path().parent_path().parent_path()/"config/turret.yaml").string();
}
config::LoadResult load(YAML::Node root) {
  char path[] = "/tmp/ota_motion_XXXXXX";
  const int fd = mkstemp(path);
  if (fd < 0) throw std::runtime_error("mkstemp");
  close(fd);
  { std::ofstream f(path); f << root; }
  auto result = config::load_turret_config(path);
  std::remove(path);
  return result;
}
}

TEST(MotionConfig, StationTargetsUseServiceMaximumAndKeepOtherModesIndependent) {
  const auto loaded = config::load_turret_config(station_config());
  ASSERT_TRUE(loaded.ok);
  const auto c = wire::make_control_cfg(loaded.config);
  ASSERT_TRUE(c.motion.configured);
  for (int axis=0; axis<2; ++axis) {
    const auto& track = c.motion.modes[1][axis];
    EXPECT_DOUBLE_EQ(track.target.speed,20*kDeg2Rad);
    EXPECT_DOUBLE_EQ(track.target.acceleration,30*kDeg2Rad);
    EXPECT_DOUBLE_EQ(track.maximum.speed,track.target.speed);
    EXPECT_DOUBLE_EQ(track.maximum.acceleration,track.target.acceleration);
    EXPECT_DOUBLE_EQ(c.motion.modes[2][axis].target.speed,10*kDeg2Rad);
    EXPECT_DOUBLE_EQ(c.motion.modes[2][axis].target.acceleration,15*kDeg2Rad);
  }
  EXPECT_DOUBLE_EQ(loaded.config.homing.contact.coarse_speed_deg_s,5);
  EXPECT_DOUBLE_EQ(loaded.config.homing.contact.fine_speed_deg_s,3);
  EXPECT_DOUBLE_EQ(loaded.config.shutdown.speed_deg_s,3);
  EXPECT_DOUBLE_EQ(loaded.config.shutdown.verify_speed_deg_s,2);
}

TEST(MotionConfig, RejectsMissingPairsTypoesNonfiniteAndExcessiveRates) {
  for (int scenario=0; scenario<11; ++scenario) {
    SCOPED_TRACE(scenario);
    auto root = YAML::LoadFile(station_config());
    auto mode = root["motion"]["modes"]["auto_track"];
    if (scenario==0) mode.remove("target");
    if (scenario==1) mode["target"].remove("acceleration_deg_s2");
    if (scenario==2) mode["target"]["speed_deg_s"] = ".nan";
    if (scenario==3) mode["maximum"]["acceleration_deg_s2"] = 31;
    if (scenario==4) mode["target"]["speed_deg_s"] = 21;
    if (scenario==5) mode["target"]["acceleration_deg_s2"] = -1;
    if (scenario==6) mode["target"]["speed_degs"] = 10;
    if (scenario==7) root["motion"]["modes"].remove("manual");
    if (scenario==8) root["tracking"]["hold_speed_deg_s"] = 20;
    if (scenario==9) root["axes"]["pitch"]["max_acceleration_deg_s2"] = ".inf";
    if (scenario==10) root["v3"]["service_speed_control"] = false;
    EXPECT_FALSE(load(root).ok);
  }
}

TEST(MotionConfig, AxisOverridesResolveWithoutMutatingSiblingOrOtherModes) {
  auto root = YAML::LoadFile(station_config());
  auto p = root["motion"]["modes"]["auto_track"]["axes"]["pitch"];
  p["maximum"] = YAML::Load("{speed_deg_s: 8, acceleration_deg_s2: 12}");
  p["target"] = YAML::Load("{speed_deg_s: 6, acceleration_deg_s2: 10}");
  auto loaded = load(root);
  ASSERT_TRUE(loaded.ok);
  EXPECT_DOUBLE_EQ(loaded.config.motion.modes[1][0].target.speed,6*kDeg2Rad);
  EXPECT_DOUBLE_EQ(loaded.config.motion.modes[1][1].target.speed,20*kDeg2Rad);
  EXPECT_DOUBLE_EQ(loaded.config.motion.modes[2][0].target.speed,10*kDeg2Rad);
}

TEST(MotionProfile, CapsBothPairsByAxisAndPayloadAndPreservesBrakingWhenDerated) {
  control::MotionProfile p;
  p.target = p.maximum;
  const auto e = control::resolve_motion(p,{12*kDeg2Rad,18*kDeg2Rad,80*kDeg2Rad},
      {10*kDeg2Rad,16*kDeg2Rad,70*kDeg2Rad},.5);
  EXPECT_DOUBLE_EQ(e.maximum.speed,5*kDeg2Rad);
  EXPECT_DOUBLE_EQ(e.maximum.acceleration,16*kDeg2Rad);
  EXPECT_DOUBLE_EQ(e.target.speed,5*kDeg2Rad);
  EXPECT_DOUBLE_EQ(e.target.acceleration,8*kDeg2Rad);
  EXPECT_DOUBLE_EQ(e.target.jerk,70*kDeg2Rad);
}

TEST(MotionProfile, ServoDeceleratesIntoALowerModeCeilingWithoutSpeedJump) {
  control::SpeedServo servo;
  servo.velocity = 20*kDeg2Rad;
  for (int i=0; i<400; ++i) {
    const double prior = servo.velocity;
    const double v = servo.step(1,5*kDeg2Rad,0,5*kDeg2Rad,.005,
                               30*kDeg2Rad,120*kDeg2Rad,1,1,true);
    ASSERT_LE(std::abs(v-prior)/.005,30*kDeg2Rad+1e-9);
  }
  EXPECT_NEAR(servo.velocity,5*kDeg2Rad,1e-8);
}

TEST(MotionProfile, MaximumTrackingKeepsBoundaryReserveWithLagInBothDirections) {
  const auto loaded = config::load_turret_config(station_config());
  ASSERT_TRUE(loaded.ok);
  const auto p = loaded.config.motion.modes[1][0];
  const control::BoundaryGovernor governor{p.maximum.acceleration,p.maximum.jerk,.20,.05};
  AxisLimits limits;
  limits.valid=true; limits.q_soft_min_rad=-45*kDeg2Rad; limits.q_soft_max_rad=45*kDeg2Rad;
  for (double lag : {.05,.12,.20}) for (double sign : {-1.,1.}) {
    SCOPED_TRACE(::testing::Message()<<"lag="<<lag<<" sign="<<sign);
    control::ReferenceLimiter reference;
    control::SpeedServo servo;
    reference.reset_at(0);
    double q=0,v=0;
    for (int i=0; i<8000; ++i) {
      const auto b=governor.at(q,limits,p.maximum.speed,servo.acceleration,v);
      const double cap=sign>0 ? b.positive_speed : b.negative_speed;
      control::track_reference(reference,sign*60*kDeg2Rad,sign*p.target.speed,.005,
          std::min(cap,p.target.speed),p.target.acceleration,p.target.jerk);
      double cmd=servo.step(reference.q_rad,reference.v_rad_s,q,p.maximum.speed,.005,
          p.maximum.acceleration,p.maximum.jerk,b.negative_acceleration_scale,b.positive_acceleration_scale,true);
      cmd=std::clamp(cmd,-b.negative_speed,b.positive_speed);
      servo.velocity=cmd;
      v+=(cmd-v)*(1-std::exp(-.005/lag));
      q+=v*.005;
      ASSERT_GE(limits.distance_to_soft(q),governor.margin-1e-5);
    }
    EXPECT_NEAR(limits.distance_to_soft(q),governor.margin,1e-4);
  }
}
