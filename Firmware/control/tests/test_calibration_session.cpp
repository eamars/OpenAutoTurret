// ADR-005 D7: a commissioning session run by controld -- commissiond's pitch servo trial without its I/O,
// writing commissiond's journal. The plant here is the drive's speed loop at its simplest: the axis
// moves at the commanded speed.
#include <filesystem>
#include <fstream>
#include <map>
#include <string>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "calibration/calibration_session.hpp"
#include "calibration_fixture.hpp"

using namespace ota;
using namespace ota::calibration;
namespace fs = std::filesystem;

namespace {

struct Dir {
  fs::path path = fs::temp_directory_path() / ("ota-session-" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
                                               "-" + ::testing::UnitTest::GetInstance()->current_test_info()->name());
  Dir() { fs::remove_all(path); fs::create_directories(path); }
  ~Dir() { fs::remove_all(path); }
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
