// Unit tests for payload profile storage (architecture §28.5, §41).
// Filesystem round-trips in a private temp directory — no CAN, no motor.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>

#include "payload/payload_profile.hpp"

namespace {

using ota::payload::AxisPayloadProfile;
using ota::payload::PayloadProfile;
using ota::payload::PayloadProfileStore;
using ota::payload::kPayloadProfileSchemaVersion;

std::string temp_dir() {
  char tmpl[] = "/tmp/ota_payload_profiles_XXXXXX";
  const char* d = mkdtemp(tmpl);
  EXPECT_NE(d, nullptr);
  return d ? d : "/tmp/ota_payload_profiles_fallback";
}

PayloadProfile make_profile(const std::string& name) {
  PayloadProfile p;
  p.name = name;
  p.created_ns = 123456789;
  p.config_revision = "turret.yaml rev 3";
  p.hardware = "can0";
  p.notes = "test payload";
  for (int i = 0; i < 2; ++i) {
    auto& ax = p.axis(static_cast<ota::AxisId>(i));
    ax.v_max_rad_s = 0.5 + 0.1 * i;
    ax.a_max_rad_s2 = 1.5 + 0.1 * i;
    ax.j_max_rad_s3 = 4.5 + 0.1 * i;
    ax.gain_kp = 2.0 * i;
    ax.gain_ki = 0.5 * i;
    ax.gain_kd = 0.1 * i;
    ax.baseline.step_pos.valid = true;
    ax.baseline.step_pos.rise_time_s = 0.11 + 0.01 * i;
    ax.baseline.step_pos.settling_time_s = 0.20 + 0.01 * i;
    ax.baseline.step_pos.overshoot = 0.001 * i;
    ax.baseline.step_pos.peak_effort_nm = 0.7 + 0.1 * i;
    ax.baseline.step_pos.tracking_rms_rad = 0.001;
    ax.baseline.step_neg = ax.baseline.step_pos;
    ax.baseline.triangle_rms_rad = 0.0005;
    ax.baseline.brake.valid = true;
    ax.baseline.brake.v0_rad_s = 0.35;
    ax.baseline.brake.stop_distance_rad = 0.02;
    ax.baseline.brake.stop_time_s = 0.12;
    ax.baseline.hold_effort_nm = 0.2;
    ax.baseline.max_verified_speed_rad_s = 0.35;
    ax.baseline.valid = true;
  }
  return p;
}

bool file_exists(const std::string& p) {
  struct stat st{};
  return ::stat(p.c_str(), &st) == 0;
}

std::string file_text(const std::string& p) {
  std::ifstream f(p);
  std::string s((std::istreambuf_iterator<char>(f)),
                std::istreambuf_iterator<char>());
  return s;
}

}  // namespace

TEST(PayloadProfile, SaveLoadRoundTrip) {
  const std::string dir = temp_dir();
  PayloadProfileStore store(dir);
  std::string err;
  const auto p = make_profile("roundtrip");
  ASSERT_TRUE(store.save(p, err)) << err;
  PayloadProfile out;
  ASSERT_TRUE(store.load("roundtrip", out, err)) << err;
  EXPECT_EQ(out.schema_version, kPayloadProfileSchemaVersion);
  EXPECT_EQ(out.name, "roundtrip");
  EXPECT_EQ(out.created_ns, p.created_ns);
  EXPECT_EQ(out.config_revision, "turret.yaml rev 3");
  EXPECT_EQ(out.hardware, "can0");
  EXPECT_EQ(out.notes, "test payload");
  for (int i = 0; i < 2; ++i) {
    const auto& a = p.axis(static_cast<ota::AxisId>(i));
    const auto& b = out.axis(static_cast<ota::AxisId>(i));
    EXPECT_DOUBLE_EQ(b.v_max_rad_s, a.v_max_rad_s);
    EXPECT_DOUBLE_EQ(b.a_max_rad_s2, a.a_max_rad_s2);
    EXPECT_DOUBLE_EQ(b.j_max_rad_s3, a.j_max_rad_s3);
    EXPECT_DOUBLE_EQ(b.gain_kp, a.gain_kp);
    EXPECT_DOUBLE_EQ(b.gain_ki, a.gain_ki);
    EXPECT_DOUBLE_EQ(b.gain_kd, a.gain_kd);
    EXPECT_EQ(b.baseline.step_pos.valid, a.baseline.step_pos.valid);
    EXPECT_DOUBLE_EQ(b.baseline.step_pos.rise_time_s,
                     a.baseline.step_pos.rise_time_s);
    EXPECT_DOUBLE_EQ(b.baseline.step_pos.peak_effort_nm,
                     a.baseline.step_pos.peak_effort_nm);
    EXPECT_DOUBLE_EQ(b.baseline.triangle_rms_rad,
                     a.baseline.triangle_rms_rad);
    EXPECT_DOUBLE_EQ(b.baseline.brake.stop_distance_rad,
                     a.baseline.brake.stop_distance_rad);
    EXPECT_DOUBLE_EQ(b.baseline.hold_effort_nm, a.baseline.hold_effort_nm);
    EXPECT_DOUBLE_EQ(b.baseline.max_verified_speed_rad_s,
                     a.baseline.max_verified_speed_rad_s);
    EXPECT_EQ(b.baseline.valid, a.baseline.valid);
  }
}

TEST(PayloadProfile, AtomicSaveLeavesNoTmp) {
  const std::string dir = temp_dir();
  PayloadProfileStore store(dir);
  std::string err;
  ASSERT_TRUE(store.save(make_profile("atomic"), err)) << err;
  EXPECT_FALSE(file_exists(dir + "/atomic.yaml.tmp"));
  EXPECT_TRUE(file_exists(dir + "/atomic.yaml"));
}

TEST(PayloadProfile, PreviousGoodKeptAsDotPrev) {
  const std::string dir = temp_dir();
  PayloadProfileStore store(dir);
  std::string err;
  auto p1 = make_profile("prev");
  p1.notes = "first";
  ASSERT_TRUE(store.save(p1, err)) << err;
  auto p2 = p1;
  p2.notes = "second";
  p2.created_ns += 1;
  ASSERT_TRUE(store.save(p2, err)) << err;

  EXPECT_TRUE(file_exists(dir + "/prev.yaml.prev"));
  const auto prev = file_text(dir + "/prev.yaml.prev");
  EXPECT_NE(prev.find("first"), std::string::npos);
  const auto cur = file_text(dir + "/prev.yaml");
  EXPECT_NE(cur.find("second"), std::string::npos);
  EXPECT_EQ(cur.find("first"), std::string::npos);
}

TEST(PayloadProfile, ListReturnsSavedNames) {
  const std::string dir = temp_dir();
  PayloadProfileStore store(dir);
  std::string err;
  ASSERT_TRUE(store.save(make_profile("alpha"), err)) << err;
  ASSERT_TRUE(store.save(make_profile("beta"), err)) << err;
  const auto names = store.list();
  EXPECT_EQ(names.size(), 2u);
  EXPECT_NE(std::find(names.begin(), names.end(), "alpha"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "beta"), names.end());
}

TEST(PayloadProfile, MissingProfileIsAnError) {
  const std::string dir = temp_dir();
  PayloadProfileStore store(dir);
  PayloadProfile out;
  std::string err;
  EXPECT_FALSE(store.load("ghost", out, err));
  EXPECT_FALSE(err.empty());
}

TEST(PayloadProfile, CorruptYamlIsRejected) {
  const std::string dir = temp_dir();
  {
    std::ofstream f(dir + "/corrupt.yaml");
    f << "this is: [not: valid: yaml: {for: a profile";
  }
  PayloadProfileStore store(dir);
  PayloadProfile out;
  std::string err;
  EXPECT_FALSE(store.load("corrupt", out, err));
  EXPECT_FALSE(err.empty());
}

TEST(PayloadProfile, SchemaVersionMismatchRejected) {
  const std::string dir = temp_dir();
  {
    std::ofstream f(dir + "/old.yaml");
    f << "schema_version: 99\nname: old\n";
  }
  PayloadProfileStore store(dir);
  PayloadProfile out;
  std::string err;
  EXPECT_FALSE(store.load("old", out, err));
  EXPECT_NE(err.find("schema"), std::string::npos);
}

TEST(PayloadProfile, UnsafeNamesRejected) {
  const std::string dir = temp_dir();
  PayloadProfileStore store(dir);
  std::string err;
  auto evil = make_profile("../evil");
  EXPECT_FALSE(store.save(evil, err));
  evil.name = "has space";
  EXPECT_FALSE(store.save(evil, err));
  evil.name = "a/b";
  EXPECT_FALSE(store.save(evil, err));
}

TEST(PayloadProfile, CreateDirectoryOnSave) {
  const std::string base = temp_dir();
  const std::string nested = base + "/nested/payload_profiles";
  PayloadProfileStore store(nested);
  std::string err;
  ASSERT_TRUE(store.save(make_profile("createdir"), err)) << err;
  PayloadProfile out;
  ASSERT_TRUE(store.load("createdir", out, err)) << err;
  EXPECT_EQ(out.name, "createdir");
}

TEST(PayloadProfile, AxisDefaultsAreConservative) {
  AxisPayloadProfile a;
  EXPECT_EQ(a.v_max_rad_s, 0.0);
  EXPECT_EQ(a.gain_kp, 0.0);
  EXPECT_FALSE(a.valid());
  a.v_max_rad_s = 0.3;
  EXPECT_FALSE(a.valid());  // a measured baseline is required too
  a.baseline.valid = true;
  EXPECT_TRUE(a.valid());
}
