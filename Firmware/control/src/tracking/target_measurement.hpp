#pragma once
// TargetMeasurement — the visiond -> controld message (architecture §6.2).
//
// Mirrors vision/protocol.py EXACTLY (same field order, little-endian, 58-byte
// fixed length, no padding). The optional visual_track_id is always present on
// the wire (0 when absent). The C++ test test_target_measurement encodes a
// message and the Python test decodes the same bytes (and vice-versa) to pin
// the two implementations together.
#include <array>
#include <cstdint>
#include <cstring>

namespace ota {
namespace vision {

// Packed wire layout (little-endian). Must stay in lockstep with
// vision/protocol.py::_FORMAT == "<QQbi" + "f"*7 + "bQ" (58 bytes).
struct __attribute__((packed)) TargetMeasurementWire {
  uint64_t frame_sequence;
  uint64_t sensor_timestamp_ns;
  int8_t valid;
  int32_t class_id;
  float confidence;
  float bbox_x_min_norm;
  float bbox_y_min_norm;
  float bbox_x_max_norm;
  float bbox_y_max_norm;
  float anchor_u_px;
  float anchor_v_px;
  int8_t has_track_id;
  uint64_t visual_track_id;
};
static_assert(sizeof(TargetMeasurementWire) == 58,
              "TargetMeasurement wire size must match vision/protocol.py");

struct TargetMeasurement {
  uint64_t frame_sequence = 0;
  uint64_t sensor_timestamp_ns = 0;
  bool valid = false;
  int32_t class_id = 0;
  float confidence = 0.0f;
  float bbox_x_min_norm = 0.0f;
  float bbox_y_min_norm = 0.0f;
  float bbox_x_max_norm = 0.0f;
  float bbox_y_max_norm = 0.0f;
  float anchor_u_px = 0.0f;
  float anchor_v_px = 0.0f;
  bool has_track_id = false;
  uint64_t visual_track_id = 0;

  static constexpr std::size_t kWireSize = sizeof(TargetMeasurementWire);

  std::array<uint8_t, kWireSize> encode() const {
    TargetMeasurementWire w;
    w.frame_sequence = frame_sequence;
    w.sensor_timestamp_ns = sensor_timestamp_ns;
    w.valid = valid ? 1 : 0;
    w.class_id = class_id;
    w.confidence = confidence;
    w.bbox_x_min_norm = bbox_x_min_norm;
    w.bbox_y_min_norm = bbox_y_min_norm;
    w.bbox_x_max_norm = bbox_x_max_norm;
    w.bbox_y_max_norm = bbox_y_max_norm;
    w.anchor_u_px = anchor_u_px;
    w.anchor_v_px = anchor_v_px;
    w.has_track_id = has_track_id ? 1 : 0;
    w.visual_track_id = visual_track_id;
    std::array<uint8_t, kWireSize> out;
    std::memcpy(out.data(), &w, kWireSize);
    return out;
  }

  // Returns false if the buffer is not exactly kWireSize bytes.
  static bool decode(const uint8_t* data, std::size_t size,
                     TargetMeasurement& out) {
    if (size != kWireSize || data == nullptr) return false;
    TargetMeasurementWire w;
    std::memcpy(&w, data, kWireSize);
    out.frame_sequence = w.frame_sequence;
    out.sensor_timestamp_ns = w.sensor_timestamp_ns;
    out.valid = (w.valid != 0);
    out.class_id = w.class_id;
    out.confidence = w.confidence;
    out.bbox_x_min_norm = w.bbox_x_min_norm;
    out.bbox_y_min_norm = w.bbox_y_min_norm;
    out.bbox_x_max_norm = w.bbox_x_max_norm;
    out.bbox_y_max_norm = w.bbox_y_max_norm;
    out.anchor_u_px = w.anchor_u_px;
    out.anchor_v_px = w.anchor_v_px;
    out.has_track_id = (w.has_track_id != 0);
    out.visual_track_id = w.visual_track_id;
    return true;
  }

  static TargetMeasurement decode(const std::array<uint8_t, kWireSize>& data) {
    TargetMeasurement out;
    decode(data.data(), data.size(), out);
    return out;
  }
};

}  // namespace vision
}  // namespace ota
