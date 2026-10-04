// SURVEILLANCE's watch point on disk (owner, 2026-10-05): what is written can be read back exactly,
// anything malformed is refused whole, and the store's thread is the only writer.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <thread>

#include "mode/watch_point.hpp"

namespace ota {
namespace {

std::filesystem::path scratch_dir() {
  std::random_device rd;
  auto dir = std::filesystem::temp_directory_path() / ("ota-watch-" + std::to_string(rd()));
  std::filesystem::create_directories(dir);
  return dir;
}

bool wait_saved(const WatchPointStore& store) {
  for (int i = 0; i < 400 && store.last_result() != 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  return store.last_result() == 1;
}

TEST(WatchPoint, ItRoundTripsExactly) {
  WatchPoint w;
  w.valid = true;
  w.yaw_absolute = true;
  w.yaw_rad = 4.123456789012345;
  w.pitch_homed_rad = 0.7071067811865476;
  w.saved_unix_s = 1790000000;
  WatchPoint r;
  std::string err;
  ASSERT_TRUE(watch_point_from_json(watch_point_to_json(w), r, err)) << err;
  EXPECT_TRUE(r.valid);
  EXPECT_TRUE(r.yaw_absolute);
  EXPECT_DOUBLE_EQ(r.yaw_rad, w.yaw_rad);
  EXPECT_DOUBLE_EQ(r.pitch_homed_rad, w.pitch_homed_rad);
  EXPECT_EQ(r.saved_unix_s, w.saved_unix_s);
  w.yaw_absolute = false;
  ASSERT_TRUE(watch_point_from_json(watch_point_to_json(w), r, err)) << err;
  EXPECT_FALSE(r.yaw_absolute);
}

TEST(WatchPoint, AnythingMalformedIsRefusedWhole) {
  WatchPoint r;
  std::string err;
  for (const char* bad : {
           "", "[]", "{\"schema\":2,\"yaw_frame\":\"homed\",\"yaw_rad\":0,\"pitch_homed_rad\":0}",
           "{\"schema\":1,\"yaw_frame\":\"session\",\"yaw_rad\":0,\"pitch_homed_rad\":0}",
           "{\"schema\":1,\"yaw_frame\":\"homed\",\"pitch_homed_rad\":0}",
           "{\"schema\":1,\"yaw_frame\":\"homed\",\"yaw_rad\":.nan,\"pitch_homed_rad\":0}",
           "{\"schema\":1,\"yaw_frame\":\"homed\",\"yaw_rad\":\"left\",\"pitch_homed_rad\":0}"}) {
    EXPECT_FALSE(watch_point_from_json(bad, r, err)) << bad;
    EXPECT_FALSE(r.valid) << bad;
    EXPECT_FALSE(err.empty()) << bad;
  }
}

TEST(WatchPoint, AMissingFileIsNoPointNotAnError) {
  WatchPoint r;
  std::string err;
  EXPECT_FALSE(load_watch_point((scratch_dir() / "absent.json").string(), r, err));
  EXPECT_FALSE(r.valid);
  EXPECT_TRUE(err.empty()) << err;
}

TEST(WatchPoint, TheStoreWritesOffTheCallersThreadAndTheNewestPointWins) {
  const auto path = (scratch_dir() / "watch_point.json").string();
  WatchPointStore store(path);
  EXPECT_TRUE(store.persistent());
  EXPECT_EQ(store.last_result(), -1);
  WatchPoint w;
  w.valid = true;
  w.yaw_absolute = true;
  for (int i = 0; i < 5; ++i) {
    w.yaw_rad = 0.1 * i;
    store.save(w);
  }
  ASSERT_TRUE(wait_saved(store));
  for (int i = 0; i < 100; ++i) {  // the last queued point may land after an earlier one
    WatchPoint r;
    std::string err;
    if (load_watch_point(path, r, err) && std::abs(r.yaw_rad - 0.4) < 1e-12) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  FAIL() << "the newest point never reached the file";
}

TEST(WatchPoint, WithNoPathNothingIsWrittenAndTheStoreSaysSo) {
  WatchPointStore store;
  EXPECT_FALSE(store.persistent());
  store.save(WatchPoint{});
  EXPECT_EQ(store.last_result(), 0);
}

TEST(WatchPoint, AnUnwritableDirectoryIsReportedNotThrown) {
  WatchPointStore store("/nonexistent-ota-dir/watch_point.json");
  WatchPoint w;
  w.valid = true;
  store.save(w);
  for (int i = 0; i < 400 && store.last_result() == -1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_EQ(store.last_result(), 0);
}

}  // namespace
}  // namespace ota
