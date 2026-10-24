// ADR-005: the calibration record store -- what controld puts in force, and what it refuses.
#include "calibration_fixture.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

#include "calibration/calibration_store.hpp"

using namespace ota;
namespace fs = std::filesystem;

namespace {
using namespace ota_cal_fixture;

calibration::Store open(const State& s) {
  calibration::Store store;
  std::string err;
  EXPECT_TRUE(store.open(s.dir, err)) << err;
  return store;
}
}  // namespace

TEST(CalibrationStore, NoStoreIsAnEmptyStoreNotAnError) {
  State s;
  auto store = open(s);
  EXPECT_TRUE(store.empty());
  std::string why;
  EXPECT_EQ(store.in_force(why), nullptr);
  EXPECT_EQ(why, "no calibration records");
}

TEST(CalibrationStore, TheShippedRecordIsPutInForceWithEveryPart) {
  State s;
  make_record(s, 1, "AS SHIPPED");
  choose(s, 1);
  auto store = open(s);
  std::string why;
  const auto* r = store.in_force(why);
  ASSERT_NE(r, nullptr) << why;
  EXPECT_EQ(r->id, 1);
  EXPECT_EQ(r->name, "AS SHIPPED");
  EXPECT_EQ(r->parts.at("yaw").date, "2026-10-02");
  EXPECT_EQ(r->parts.at("extrinsic").origin, "inherited");
  EXPECT_EQ(r->parts.at("extrinsic").from, 1);
  EXPECT_EQ(r->path("yaw"), s.cal() / "records/0001/yaw_servo.json");
  EXPECT_FALSE(r->bore.enabled) << "mode off: the tracker keeps aiming the camera centre";
  EXPECT_DOUBLE_EQ(r->bore.zero_m, 10);
  const auto a = r->bore.alignment();
  EXPECT_DOUBLE_EQ(a.wide.right_mm, 75);
  EXPECT_DOUBLE_EQ(a.assumed_depth_m, 10);
}

TEST(CalibrationStore, AFileThatDoesNotParseRefusesTheRecordAndNoOtherIsPutInItsPlace) {
  State s;
  make_record(s, 1, "AS SHIPPED");
  make_record(s, 2, "BENCH TEST");
  write(s.cal() / "records/0002/yaw_servo.json", "{\"servo_parameters\": {\"kq\": ");   // truncated
  choose(s, 2);
  auto store = open(s);
  std::string why;
  EXPECT_EQ(store.in_force(why), nullptr) << "never another record instead (ADR-005 D4)";
  EXPECT_NE(why.find("record #2: yaw_servo.json does not parse"), std::string::npos) << why;
  ASSERT_NE(store.find(1), nullptr);
  EXPECT_TRUE(store.find(1)->unusable.empty());
}

TEST(CalibrationStore, AMissingPartOrABadBoreIsNamed) {
  State s;
  make_record(s, 1, "AS SHIPPED");
  make_record(s, 2, "NO PITCH");
  fs::remove(s.cal() / "records/0002/pitch_servo.json");
  make_record(s, 3, "BAD BORE", bore_yaml("sideways"));
  auto store = open(s);
  EXPECT_EQ(store.find(2)->unusable, "pitch_servo.json is missing");
  EXPECT_EQ(store.find(3)->unusable, "bore.yaml mode must be off or manual_depth");
}

TEST(CalibrationStore, AHandEditedFileNeedsNoBookkeeping) {
  // No hashes (owner, 2026-10-24): the 10-06 and 10-20 extrinsic re-references were hand edits.
  State s;
  make_record(s, 1, "AS SHIPPED");
  std::ofstream(s.cal() / "records/0001/camera_extrinsics.yaml", std::ios::app) << "# re-referenced by hand\n";
  choose(s, 1);
  auto store = open(s);
  std::string why;
  EXPECT_NE(store.in_force(why), nullptr) << why;
}

TEST(CalibrationStore, AnImportIsAdoptedUnderTheNextFreeIdAndIsNotPutInForce) {
  State s;
  make_record(s, 1, "AS SHIPPED");
  choose(s, 1);
  fs::create_directories(s.cal() / "deleted/0002");                  // a removed record keeps its id
  make_record(s, 5, "STAGED");
  fs::create_directories(s.cal() / "incoming");
  fs::rename(s.cal() / "records/0005", s.cal() / "incoming/20261024-150000-STAGED");
  fs::create_directories(s.cal() / "incoming/.staging-half-written");  // the tool's own temp dir
  auto store = open(s);
  ASSERT_NE(store.find(3), nullptr) << "1 is live and 2 is in deleted/: ids are never reused";
  EXPECT_EQ(store.find(3)->name, "STAGED");
  EXPECT_EQ(store.active_id(), 1);
  EXPECT_TRUE(fs::exists(s.cal() / "incoming/.staging-half-written")) << "a staging dir is left alone";
  ASSERT_EQ(store.adopted().size(), 1u);
  EXPECT_NE(store.adopted()[0].find("as #3"), std::string::npos);
}

TEST(CalibrationStore, NoChoiceIsReportedNotGuessed) {
  State s;
  make_record(s, 1, "AS SHIPPED");
  auto store = open(s);
  std::string why;
  EXPECT_EQ(store.in_force(why), nullptr);
  EXPECT_EQ(why, "no record is chosen (active.json is missing or unreadable)");
  choose(s, 7);
  store = open(s);
  EXPECT_EQ(store.in_force(why), nullptr);
  EXPECT_EQ(why, "record #7 named in active.json does not exist");
}

TEST(CalibrationStore, ARightTrimMovesTheBoreMarkRightAndAnUpTrimMovesItUp) {
  // Physical outcome, not a sign convention: on the picture the mark moves the way the press says.
  State s;
  make_record(s, 1, "ZERO", bore_yaml("manual_depth", ".inf"));
  make_record(s, 2, "TRIMMED", bore_yaml("manual_depth", ".inf", 1.0, 1.0));
  auto store = open(s);
  const geo::CameraIntrinsics cam{1389, 1467, 960, 540, 1920, 1080};
  const auto zero = geo::bore_alignment(store.find(1)->bore.alignment(), geo::CameraRole::Wide, cam);
  const auto trim = geo::bore_alignment(store.find(2)->bore.alignment(), geo::CameraRole::Wide, cam);
  ASSERT_TRUE(zero.valid && trim.valid) << zero.reason << " / " << trim.reason;
  EXPECT_NEAR((trim.u_norm - zero.u_norm) * 1920, 1389 * 1e-3, 0.01) << "1 mrad right = 1.4 px right";
  EXPECT_NEAR((zero.v_norm - trim.v_norm) * 1080, 1467 * 1e-3, 0.01) << "1 mrad up = 1.5 px up";
}

TEST(CalibrationStore, ZeroAtInfinityHasNoParallax) {
  State s;
  make_record(s, 1, "AT 10 M", bore_yaml("manual_depth", "10"));
  make_record(s, 2, "AT INFINITY", bore_yaml("manual_depth", ".inf"));
  auto store = open(s);
  EXPECT_TRUE(std::isinf(store.find(2)->bore.zero_m));
  const geo::CameraIntrinsics cam{1389, 1467, 960, 540, 1920, 1080};
  const auto at10 = geo::bore_alignment(store.find(1)->bore.alignment(), geo::CameraRole::Wide, cam);
  const auto inf = geo::bore_alignment(store.find(2)->bore.alignment(), geo::CameraRole::Wide, cam);
  EXPECT_NEAR(inf.u_norm * 1920, 960, 0.01) << "a bore parallel to the camera at infinity is the centre";
  EXPECT_NEAR(inf.v_norm * 1080, 540, 0.01);
  EXPECT_NEAR((inf.u_norm - at10.u_norm) * 1920, 1389 * 0.075 / 10, 0.05) << "75 mm at 10 m: 7.5 mrad";
}
