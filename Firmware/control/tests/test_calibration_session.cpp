// ADR-005 D7: a commissioning session run by controld -- commissiond's pitch servo trial without its I/O,
// writing commissiond's journal. The plant here is the drive's speed loop at its simplest: the axis
// moves at the commanded speed.
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "calibration/calibration_session.hpp"
#include "can/gm6020_protocol.hpp"
#include "plant.hpp"
#include "calibration_fixture.hpp"

using namespace ota;
using namespace ota::calibration;
namespace fs = std::filesystem;

namespace {

struct Dir {
  fs::path path = fs::temp_directory_path() / ("ota-session-" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
                                               "-" + ::testing::UnitTest::GetInstance()->current_test_info()->name());
  Dir() { fs::remove_all(path); fs::create_directories(path); }
  // OTA_SESSION_JOURNALS keeps what the sessions wrote: tools/servo_commission/tests reads them with
  // journal.py, the parser commission.py uses.
  ~Dir() {
    if (const char* keep = std::getenv("OTA_SESSION_JOURNALS"); keep && *keep) {
      std::error_code ec;
      for (const auto& e : fs::directory_iterator(path, ec))
        if (e.path().extension() == ".jsonl") fs::copy_file(e.path(), fs::path(keep) / e.path().filename(),
                                                            fs::copy_options::overwrite_existing, ec);
    }
    fs::remove_all(path);
  }
};

using ota_cal_fixture::pitch_manifest;
std::string manifest(const std::string& extra = "", double step = 0.02) { return pitch_manifest(extra, step); }

std::map<std::string, int> kinds(const fs::path& journal) {
  std::map<std::string, int> out;
  std::ifstream in(journal);
  for (std::string line; std::getline(in, line);) ++out[YAML::Load(line)["kind"].as<std::string>()];
  return out;
}

// Steps a session against an axis that moves at the commanded speed, until it holds (or `limit_s`).
struct Plant {
  double pose;
  bool stuck = false;
  TimeNs now = 1'000'000'000;
  double run(PitchSession& s, const PitchWindow& w, double limit_s = 30) {
    for (int k = 0; k < limit_s * 1000 && s.state() == SessionState::Running; ++k) {
      const double cmd = s.step(now, pose, now - 300'000, 0.1, w);
      if (!stuck) pose += cmd * 0.001;
      now += 1'000'000;
    }
    return pose;
  }
};

}  // namespace

TEST(PitchSession, RunsTheScriptAboutTheWindowCentreAndWritesCommissiondsJournal) {
  Dir d;
  std::string err;
  auto s = PitchSession::make(manifest(), (d.path / "sensorless-homing.jsonl").string(), "t-identify", err);
  ASSERT_TRUE(s) << err;
  const PitchWindow w{-0.5, 0.3};   // production's soft envelope: centre -0.1
  Plant p{0.1};
  p.run(*s, w);
  ASSERT_EQ(s->state(), SessionState::Holding);
  EXPECT_TRUE(s->complete()) << s->detail();
  EXPECT_NEAR(p.pose, -0.1 + 0.02, 2e-3) << "the script is relative to the window centre";
  EXPECT_NEAR(s->expected_s(), 2.0 + 0.5 + 0.2 / 0.08 * 1.5, 1e-9) << "script, hold and the approach";
  // Holding: it keeps the final reference, energised (owner: never released mid-run).
  for (int k = 0; k < 500; ++k) { p.pose += s->step(p.now, p.pose, p.now, 0, w) * 0.001; p.now += 1'000'000; }
  EXPECT_NEAR(p.pose, -0.08, 2e-3);
  ASSERT_TRUE(s->finish("{\"kind\":\"idle_axis\",\"axis\":\"yaw\",\"motion_rad\":0.0003}"));
  const auto k = kinds(d.path / "sensorless-homing.jsonl");
  EXPECT_EQ(k.at("header"), 1);
  EXPECT_EQ(k.at("pitch_trial_window"), 1);
  EXPECT_EQ(k.at("pitch_trial_approach"), 1);
  EXPECT_EQ(k.at("pitch_trial_end"), 1);
  EXPECT_EQ(k.at("idle_axis"), 1);
  EXPECT_EQ(k.at("footer"), 1);
  EXPECT_GT(k.at("pitch_trial"), 5000) << "one row a millisecond, approach included";
  std::ifstream in(d.path / "sensorless-homing.jsonl");
  std::string header;
  std::getline(in, header);
  // journal.py reads the manifest back out of the header.
  const auto m = YAML::Load(YAML::Load(header)["manifest_yaml"].as<std::string>());
  EXPECT_EQ(m["servo_trial"]["kp_per_s"].as<double>(), 20);
  std::string line, last;
  while (std::getline(in, line)) {
    if (line.find("pitch_trial_window") != std::string::npos) {
      const auto row = YAML::Load(line);
      EXPECT_DOUBLE_EQ(row["center_rad"].as<double>(), -0.1);
      EXPECT_DOUBLE_EQ(row["endpoint_low_rad"].as<double>(), -0.5 - 0.14);
    }
    last = line;
  }
  EXPECT_EQ(YAML::Load(last)["status"].as<std::string>(), "COMPLETE");
}

TEST(PitchSession, AnObstructionEndsItHoldingWhereTheAxisIs) {
  Dir d;
  std::string err;
  auto s = PitchSession::make(manifest("", 0.2), (d.path / "j.jsonl").string(), "t", err);
  ASSERT_TRUE(s) << err;
  Plant p{-0.1};
  p.stuck = true;   // the step of 0.2 rad cannot be followed
  p.run(*s, {-0.5, 0.3});
  ASSERT_EQ(s->state(), SessionState::Holding);
  EXPECT_FALSE(s->complete());
  EXPECT_EQ(s->detail(), "HARD_ABORT: pitch servo following error");
  EXPECT_NEAR(s->step(p.now, p.pose, p.now, 0, {-0.5, 0.3}), 0.0, 1e-9) << "it holds the axis where it stopped";
  ASSERT_TRUE(s->finish(""));
}

TEST(PitchSession, StopEndsItAtTheNextStep) {
  Dir d;
  std::string err;
  auto s = PitchSession::make(manifest(), (d.path / "j.jsonl").string(), "t", err);
  ASSERT_TRUE(s) << err;
  Plant p{-0.1};
  p.run(*s, {-0.5, 0.3}, 0.2);
  ASSERT_EQ(s->state(), SessionState::Running);
  s->stop("the operator pressed STOP");
  p.run(*s, {-0.5, 0.3}, 0.01);
  EXPECT_EQ(s->state(), SessionState::Holding);
  EXPECT_EQ(s->detail(), "stopped: the operator pressed STOP");
}

TEST(PitchSession, ItNeedsTheWindowFromHomingAndRefusesAManifestThatHomesAgain) {
  Dir d;
  std::string err;
  EXPECT_FALSE(PitchSession::make(manifest(",\"home_first\":true"), (d.path / "a.jsonl").string(), "t", err));
  EXPECT_NE(err.find("home_first"), std::string::npos) << err;
  EXPECT_FALSE(PitchSession::make("{\"schema\":\"adr0022.yaw-control/1\"}", (d.path / "b.jsonl").string(), "t", err));
  auto s = PitchSession::make(manifest(), (d.path / "c.jsonl").string(), "t", err);
  ASSERT_TRUE(s) << err;
  Plant p{0.0};
  p.run(*s, {}, 0.01);
  EXPECT_EQ(s->detail(), "MEASUREMENT_LIMITED: no pitch window from homing");
}

namespace {
// A yaw session on the identified plant of the shipped asset (servo-sim's model): the asset's own servo
// parameters, a hold, a 0.2 rad ramp and a hold.
struct YawRig {
  YAML::Node asset = YAML::LoadFile((ota_cal_fixture::firmware() / "config/servo/yaw_servo.json").string());
  axis::YawPlant plant{axis::yaw_plant_from_yaml(asset["plant"]), 1.0};
  double delay = asset["plant"]["encoder_delay_s"].as<double>();
  double t = 0;   // seconds of plant time; frame k is sampled at k ms

  std::string manifest(double ramp_rad = 0.2) {
    YAML::Node m;
    m["schema"] = "adr0022.yaw-control/1";
    m["purpose"] = "yaw_shared_core_3a";
    m["yaw_current_bound_A"] = asset["servo_parameters"]["current_cap"];
    m["servo_parameters"] = asset["servo_parameters"];
    m["servo_motor_temperature_limit_C"] = 55;
    m["servo_speed_limit_rad_s"] = 1.75;
    m["servo_hold_after_s"] = 0.5;
    m["servo_oscillation_limit_A"] = 0.3;
    for (int k = 0; k <= 175; ++k) {   // 50 Hz: hold 0.5 s, ramp 2 s, hold 1 s
      const double ts = k * 0.02, x = std::clamp((ts - 0.5) / 2.0, 0.0, 1.0);
      YAML::Node r;
      r["time_s"] = ts;
      r["position_rad"] = ramp_rad * x;
      r["velocity_rad_s"] = ts > 0.5 && ts < 2.5 ? ramp_rad / 2.0 : 0.0;
      r["acceleration_rad_s2"] = 0.0;
      m["reference_samples"].push_back(r);
    }
    YAML::Emitter e;
    e << YAML::Flow << m;
    return e.c_str();
  }
  // Frames at 1 kHz until the session holds (or `limit_s`); what it sends drives the plant.
  void run(YawSession& s, double limit_s = 10) {
    const double quantum = 2 * M_PI / 8192;
    for (int k = 0; k < limit_s * 1000 && s.state() == SessionState::Running; ++k) step(s, quantum);
  }
  double step(YawSession& s, double quantum) {
    t += 0.001;
    plant.advance(t);
    const double sampled = plant.position(), receipt = t + delay;
    plant.advance(receipt);
    const double reading = plant.reading(sampled, receipt);
    YawFrame f;
    f.rx_ns = static_cast<TimeNs>(receipt * 1e9);
    f.dequeue_ns = f.rx_ns + 50'000;
    f.angle_count = static_cast<uint16_t>(((std::llround(reading / quantum) % 8192) + 8192) % 8192);
    f.temperature_raw = 35;
    const TimeNs now = f.rx_ns + 200'000;
    const double u = s.control(now, f);
    if (std::isfinite(u)) {
      const int raw = gm6020::current_raw_uncapped(u);
      plant.command(now * 1e-9, raw * gm6020::kAmpsPerRaw);
      s.sent(true, raw * gm6020::kAmpsPerRaw, raw, now, now + 30'000);
      return u;
    }
    return std::numeric_limits<double>::quiet_NaN();
  }
};
}  // namespace

TEST(YawSession, FollowsTheScriptOnTheIdentifiedPlantAndWritesCommissiondsJournal) {
  Dir d;
  YawRig rig;
  std::string err;
  auto s = YawSession::make(rig.manifest(), (d.path / "yaw-control.jsonl").string(), "t-probe", err);
  ASSERT_TRUE(s) << err;
  rig.run(*s);
  ASSERT_EQ(s->state(), SessionState::Holding);
  EXPECT_TRUE(s->complete()) << s->detail();
  EXPECT_NEAR(rig.plant.position(), 1.2, 0.004) << "the script is relative to where control began";
  EXPECT_NEAR(s->expected_s(), 3.5 + 0.5, 1e-9);
  const double quantum = 2 * M_PI / 8192;
  EXPECT_EQ(rig.step(*s, quantum), 0.0) << "zero current until production takes the axis back";
  ASSERT_TRUE(s->finish("{\"kind\":\"idle_axis\",\"axis\":\"pitch\",\"motion_rad\":0.0002,\"low_rad\":-0.3001,\"high_rad\":-0.2999}"));
  const auto k = kinds(d.path / "yaw-control.jsonl");
  for (const char* kind : {"header", "yaw_control_begin", "controller_parameters_readback", "servo_learned",
                           "idle_axis", "footer"})
    EXPECT_EQ(k.count(kind) ? k.at(kind) : 0, 1) << kind;
  EXPECT_GT(k.at("yaw_feedback"), 3900);
  EXPECT_GT(k.at("servo_cycle"), 3900);
  EXPECT_EQ(k.at("yaw_current_tx"), k.at("servo_cycle")) << "every step sends, and says what it sent";
}

TEST(YawSession, StopAndARunawayEndItAtZeroCurrent) {
  Dir d;
  {
    YawRig rig;
    std::string err;
    auto s = YawSession::make(rig.manifest(), (d.path / "a.jsonl").string(), "t", err);
    ASSERT_TRUE(s) << err;
    rig.run(*s, 1.0);
    s->stop("the operator pressed STOP");
    rig.run(*s, 0.01);
    EXPECT_EQ(s->state(), SessionState::Holding);
    EXPECT_EQ(s->detail(), "stopped: the operator pressed STOP");
  }
  {
    // A ramp of 6 rad in 2 s asks for 3 rad/s, above the 1.75 rad/s session limit.
    YawRig rig;
    std::string err;
    auto s = YawSession::make(rig.manifest(6.0), (d.path / "b.jsonl").string(), "t", err);
    ASSERT_TRUE(s) << err;
    rig.run(*s);
    EXPECT_FALSE(s->complete());
    EXPECT_NE(s->detail().find("HARD_ABORT: yaw speed limit"), std::string::npos) << s->detail();
  }
}

TEST(YawSession, ARefusedManifestSaysWhy) {
  Dir d;
  YawRig rig;
  std::string err;
  auto m = YAML::Load(rig.manifest());
  m["yaw_current_bound_A"] = 3.5;
  YAML::Emitter e;
  e << YAML::Flow << m;
  EXPECT_FALSE(YawSession::make(e.c_str(), (d.path / "a.jsonl").string(), "t", err));
  EXPECT_EQ(err, "HARD_ABORT: yaw current authority");
}
