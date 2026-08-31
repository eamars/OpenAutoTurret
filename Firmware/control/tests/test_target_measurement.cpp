// Unit tests for the C++ TargetMeasurement wire format (architecture §6.2).
// This must stay in lockstep with vision/protocol.py (58-byte little-endian).
// The cross-language check (C++ encode -> Python decode, and vice-versa) is done
// in the test harness / integration check; here we verify the C++ side.
#include <gtest/gtest.h>

#include "tracking/target_measurement.hpp"

namespace {
using ota::vision::TargetMeasurement;

TEST(TargetMeasurement, WireSizeIs58) {
  EXPECT_EQ(TargetMeasurement::kWireSize, 58u);
  TargetMeasurement m;
  EXPECT_EQ(m.encode().size(), 58u);
}

TEST(TargetMeasurement, EncodeDecodeRoundTrip) {
  TargetMeasurement m;
  m.frame_sequence = 12345;
  m.sensor_timestamp_ns = 987654321ULL;
  m.valid = true;
  m.class_id = 1;
  m.confidence = 0.87f;
  m.bbox_x_min_norm = 0.10f;
  m.bbox_y_min_norm = 0.20f;
  m.bbox_x_max_norm = 0.30f;
  m.bbox_y_max_norm = 0.40f;
  m.anchor_u_px = 480.0f;
  m.anchor_v_px = 270.0f;
  m.has_track_id = true;
  m.visual_track_id = 7;

  auto bytes = m.encode();
  TargetMeasurement d = TargetMeasurement::decode(bytes);
  EXPECT_EQ(d.frame_sequence, 12345u);
  EXPECT_EQ(d.sensor_timestamp_ns, 987654321ULL);
  EXPECT_TRUE(d.valid);
  EXPECT_EQ(d.class_id, 1);
  EXPECT_NEAR(d.confidence, 0.87f, 1e-6);
  EXPECT_NEAR(d.bbox_x_min_norm, 0.10f, 1e-6);
  EXPECT_NEAR(d.bbox_y_min_norm, 0.20f, 1e-6);
  EXPECT_NEAR(d.bbox_x_max_norm, 0.30f, 1e-6);
  EXPECT_NEAR(d.bbox_y_max_norm, 0.40f, 1e-6);
  EXPECT_NEAR(d.anchor_u_px, 480.0f, 1e-4);
  EXPECT_NEAR(d.anchor_v_px, 270.0f, 1e-4);
  EXPECT_TRUE(d.has_track_id);
  EXPECT_EQ(d.visual_track_id, 7u);
}

TEST(TargetMeasurement, NoTrackIdRoundTrip) {
  TargetMeasurement m;
  m.frame_sequence = 1;
  m.sensor_timestamp_ns = 1000;
  m.valid = true;
  m.has_track_id = false;
  m.visual_track_id = 0;
  TargetMeasurement d = TargetMeasurement::decode(m.encode());
  EXPECT_FALSE(d.has_track_id);
  EXPECT_EQ(d.visual_track_id, 0u);
}

TEST(TargetMeasurement, DecodeRejectsWrongSize) {
  std::array<uint8_t, 10> small{};
  TargetMeasurement d;
  EXPECT_FALSE(ota::vision::TargetMeasurement::decode(small.data(), small.size(), d));
  std::array<uint8_t, 64> big{};
  EXPECT_FALSE(ota::vision::TargetMeasurement::decode(big.data(), big.size(), d));
}

}  // namespace
