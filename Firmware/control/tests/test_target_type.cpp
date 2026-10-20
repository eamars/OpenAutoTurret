// What the turret follows: HUMAN, HEAD, CAT or DOG (owner, 2026-10-20;
// docs/DESIGN_TARGET_TYPE_2026-10-20.md). The controller's half: the choice, its persistence, and the
// hold while perception changes model.
#include <chrono>
#include <cstring>
#include <filesystem>
#include <thread>

#include "mode/target_type.hpp"
#include "tracks/perception_wire.hpp"

#include <gtest/gtest.h>

using namespace ota;

TEST(TargetType, NamesAndCodesMatchPerception) {
  // perception/target_type.py: 1 HUMAN, 2 HEAD, 3 CAT, 4 DOG; 0 none.
  for (int code = 1; code <= 4; ++code) {
    const TargetType t = target_type_from_code(static_cast<uint8_t>(code));
    TargetType back;
    ASSERT_TRUE(target_type_from_name(target_type_name(t), back));
    EXPECT_EQ(static_cast<int>(back), code);
  }
  EXPECT_EQ(target_type_from_code(0), TargetType::None);
  EXPECT_EQ(target_type_from_code(7), TargetType::None);
  EXPECT_EQ(kDefaultTargetType, TargetType::Head) << "today's behaviour is the default";
}

TEST(TargetType, ItSurvivesARestart) {
  const auto dir = std::filesystem::temp_directory_path() / "ota-target-type-test";
  std::filesystem::create_directories(dir);
  const auto path = (dir / "target_type.json").string();
  std::filesystem::remove(path);
  TargetType t = TargetType::Head;
  std::string err;
  EXPECT_FALSE(load_target_type(path, t, err));
  EXPECT_TRUE(err.empty()) << "a missing file is the default, not an error";
  {
    TargetTypeStore store(path);
    store.save(TargetType::Dog);
    for (int i = 0; i < 200 && store.last_result() != 1; ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ASSERT_EQ(store.last_result(), 1);
  }
  ASSERT_TRUE(load_target_type(path, t, err)) << err;
  EXPECT_EQ(t, TargetType::Dog);
  TargetType u = TargetType::Head;
  EXPECT_FALSE(target_type_from_json("{\"schema\":1,\"target_type\":\"BIRD\"}", u, err));
  EXPECT_EQ(u, TargetType::Head);
}

TEST(TargetType, TheAppliedTypeRidesTheNativeHeaderFlags) {
  tracks::PerceptionHeader h{};
  std::memcpy(h.magic, "OTP1", 4);
  h.version = 1;
  h.header_size = sizeof(h);
  h.session_hi = 1;
  h.session_lo = 1;
  tracks::TrackSetWire w{};
  w.sensor_timestamp_ns = 1'000'000'000;
  w.publish_timestamp_ns = 1'001'000'000;
  w.width = 1920;
  w.height = 1080;
  std::vector<uint8_t> bytes(tracks::kPerceptionWireSize);
  auto decode = [&](uint16_t flags, tracks::TrackSet& out) {
    h.flags = flags;
    std::memcpy(bytes.data(), &h, sizeof(h));
    std::memcpy(bytes.data() + sizeof(h), &w, sizeof(w));
    return tracks::decode_perception_frame(bytes.data(), bytes.size(), out);
  };
  tracks::TrackSet out;
  ASSERT_TRUE(decode(static_cast<uint16_t>(2 | (3 << 2)), out));
  EXPECT_EQ(out.observation.target_type, 3);
  EXPECT_TRUE(out.observation.narrow_view);
  EXPECT_FALSE(decode(1u << 5, out)) << "bits above 4 are still refused";
}
