// §72: the configuration an operator is allowed to name, and the refusals that keep the
// naming safe. Written as load-time checks on purpose — a value that moves a gimbal
// should fail with a sentence at startup rather than with a motion nobody expected.
#include <cstdio>
#include <fstream>
#include <string>

#include "config/turret_config.hpp"
#include "gtest/gtest.h"

namespace ota {
namespace {

struct TempYaml {
  std::string path;
  explicit TempYaml(const std::string& body) {
    path = "/tmp/ota_v3_config_test.yaml";
    std::ofstream f(path, std::ios::trunc);
    f << "schema_version: 1\n"
         "axes:\n"
         "  pitch:\n"
         "    expected_travel_deg: {min: -60.0, max: 30.0}\n"
         "    soft_margin_deg: 2.0\n"
         "  yaw:\n"
         "    expected_travel_deg: {min: 60.0, max: 250.0}\n"
         "    soft_margin_deg: 2.0\n"
      << body;
  }
  ~TempYaml() { std::remove(path.c_str()); }
};

bool any_error_contains(const config::LoadResult& r, const std::string& needle) {
  for (const auto& e : r.errors)
    if (e.find(needle) != std::string::npos) return true;
  return false;
}

TEST(V3Config, AbsentMeansTodayBehaviourExactly) {
  TempYaml y("control_loop_hz: 200\n");
  auto r = config::load_turret_config(y.path);
  EXPECT_FALSE(r.config.v3.has_roam_region);
  EXPECT_FALSE(r.config.v3.has_roam_pitch);
  EXPECT_EQ(r.config.v3.default_mode, "MANUAL");
  EXPECT_TRUE(r.config.v3.step_sizes_deg.empty());
  EXPECT_EQ(r.config.v3.jog_lease_ms, 0);
}

TEST(V3Config, ARegionInsideTheLimitsIsTakenAtItsWord) {
  TempYaml y(R"(v3:
  auto_roam:
    yaw_min_deg: 100.0
    yaw_max_deg: 190.0
    pitch_deg: -39.6
    velocity_deg_s: 8.0
)");
  auto r = config::load_turret_config(y.path);
  EXPECT_TRUE(r.config.v3.has_roam_region);
  EXPECT_DOUBLE_EQ(r.config.v3.roam_yaw_min_deg, 100.0);
  EXPECT_DOUBLE_EQ(r.config.v3.roam_yaw_max_deg, 190.0);
  EXPECT_TRUE(any_error_contains(r, "v3") == false)
      << "a region well inside the reported travel was refused anyway: "
      << (r.errors.empty() ? "" : r.errors[0]);
}

TEST(V3Config, ARegionOutsideTheSafeTravelIsRefusedWithNumbers) {
  // The yaw travel here is 60..250 with a 2 degree margin, so 255 is outside it. The
  // message has to carry the numbers: "out of range" in a log at commissioning tells the
  // operator nothing about which bound they crossed or what the bound was.
  TempYaml y(R"(v3:
  auto_roam:
    yaw_min_deg: 100.0
    yaw_max_deg: 255.0
)");
  auto r = config::load_turret_config(y.path);
  ASSERT_TRUE(any_error_contains(r, "v3.auto_roam")) << "no roam refusal at all";
  EXPECT_TRUE(any_error_contains(r, "255.00")) << "the refused value is not named";
  EXPECT_TRUE(any_error_contains(r, "248.00"))
      << "the limit it crossed is not named, so the operator has to go and re-derive it";
}

TEST(V3Config, BootingIntoAnAutomaticModeIsRefusedNotClamped) {
  TempYaml y(R"(v3:
  default_mode: AUTO_ROAM
)");
  auto r = config::load_turret_config(y.path);
  EXPECT_TRUE(any_error_contains(r, "MANUAL"))
      << "a config file can make the station sweep at power-up, and nothing said so";
}

TEST(V3Config, StepSizesMayNarrowTheSanctionedSetButNotWidenIt) {
  TempYaml ok(R"(v3:
  manual:
    step_sizes_deg: [1.0]
)");
  EXPECT_FALSE(any_error_contains(config::load_turret_config(ok.path), "step_sizes_deg"));

  TempYaml bad(R"(v3:
  manual:
    step_sizes_deg: [90.0]
)");
  auto r = config::load_turret_config(bad.path);
  EXPECT_TRUE(any_error_contains(r, "step_sizes_deg"))
      << "90 degrees was accepted as a step; §41 lists 0.5, 1 and 5 for a reason";
}

TEST(V3Config, ALeaseThatCannotHoldThreeRenewalsIsRefused) {
  // The same ratio the dashboard asserts across the wire, checked where the numbers are
  // written down. A lease shorter than three keepalives does not fail safely in the
  // abstract: the operator holds a button, the turret stops, and the reason is in a file
  // they have never opened.
  TempYaml y(R"(v3:
  manual:
    jog_keepalive_ms: 100
    jog_lease_timeout_ms: 200
)");
  auto r = config::load_turret_config(y.path);
  EXPECT_TRUE(any_error_contains(r, "renew")) << "no complaint about the lease ratio";
}

}  // namespace
}  // namespace ota

