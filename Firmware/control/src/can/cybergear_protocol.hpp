#pragma once
// Xiaomi CyberGear CAN protocol (1 Mbit/s, CAN 2.0, 29-bit extended frames).
//
// Authoritative source: Firmware/docs/CyberGear_AI_Reference.md
// (trust-labeled; official-manual sections take precedence).
//
// Frame identifier layout (29-bit extended ID):
//   bits 28..24 : communication type (5 bits)
//   bits 23..8  : data area 2 (16 bits)  — for host->motor commands this is
//                 normally the host CAN ID (placed in bits 15..8)
//   bits 7..0   : target address / target CAN ID
//
// Endianness (do NOT mix — CyberGear_AI_Reference.md §11):
//   - COMM_TYPE_1 MIT packed u16 fields: BIG-endian (high byte first)
//   - COMM_TYPE_17/18 parameter protocol: LITTLE-endian index and scalars
//   - COMM_TYPE_2 feedback u16 fields: BIG-endian (high byte first)
#include <cstdint>
#include <cstring>

namespace ota::cybergear {

// 29-bit extended frame (no CAN_EFF_FLAG — that is a socket-layer detail).
struct CanFrame {
  uint32_t id{0};
  uint8_t dlc{8};
  uint8_t data[8]{};
};

struct ExtId {
  uint8_t comm_type{0};
  uint16_t data2{0};
  uint8_t target{0};
};

inline uint32_t pack_ext_id(uint8_t comm_type, uint16_t data2, uint8_t target) {
  return (static_cast<uint32_t>(comm_type) << 24) |
         (static_cast<uint32_t>(data2) << 8) | static_cast<uint32_t>(target);
}

inline ExtId unpack_ext_id(uint32_t id) {
  return ExtId{static_cast<uint8_t>(id >> 24),
               static_cast<uint16_t>((id >> 8) & 0xFFFF),
               static_cast<uint8_t>(id & 0xFF)};
}

// Communication types (CyberGear_AI_Reference.md §12).
enum class CommType : uint8_t {
  Discovery = 0,      // get device ID
  Mit = 1,            // motion-control / MIT-style command
  Feedback = 2,       // motor feedback
  Enable = 3,
  Stop = 4,
  SetZero = 6,
  SetCanId = 7,
  ReadReg = 17,       // 0x11
  WriteReg = 18,      // 0x12
  FaultFeedback = 21, // 0x15
  SetBitrate = 22,    // 0x16
};

// u16 range scaling. The official helper clamps then truncates.
inline uint16_t encode_u16(float x, float xmin, float xmax) {
  if (x < xmin) x = xmin;
  if (x > xmax) x = xmax;
  return static_cast<uint16_t>((x - xmin) * 65535.0f / (xmax - xmin));
}

inline float decode_u16(uint16_t u, float xmin, float xmax) {
  return xmin + static_cast<float>(u) / 65535.0f * (xmax - xmin);
}

// Ranges. kPosMin/kPosMax is the de-facto software command mapping (+/-12.5
// rad per the official C example and community drivers); the manual's table
// prints +/-4*pi — treated as a source inconsistency until firmware testing
// proves otherwise (CyberGear_AI_Reference.md §14).
struct Ranges {
  static constexpr float kTorqueMin = -12.0f;
  static constexpr float kTorqueMax = 12.0f;
  static constexpr float kPosMin = -12.5f;
  static constexpr float kPosMax = 12.5f;
  static constexpr float kVelMin = -30.0f;
  static constexpr float kVelMax = 30.0f;
  static constexpr float kKpMin = 0.0f;
  static constexpr float kKpMax = 500.0f;
  static constexpr float kKdMin = 0.0f;
  static constexpr float kKdMax = 5.0f;
  // Feedback decode range. Empirically verified against live hardware:
  // a COMM_TYPE_2 feedback angle decoded with ±12.5 rad tracks the 0x7019
  // mechPos register read to <0.001 rad, while the manual's "±4*pi nominal"
  // is off by ~0.005 rad. The firmware uses the same ±12.5 mapping as the
  // MIT position command (CyberGear_AI_Reference.md §14/§15 inconsistency).
  static constexpr float kFbAngleMin = -12.5f;
  static constexpr float kFbAngleMax = 12.5f;
};

// ---------------------------------------------------------------------------
// COMM_TYPE_0 — discovery / device ID
// ---------------------------------------------------------------------------
CanFrame make_discovery_request(uint8_t host_id, uint8_t motor_id);

struct DiscoveryResponse {
  uint8_t motor_id{0};
  uint64_t unique_id{0};
};
// Response: comm_type=0, data2 carries the motor CAN ID, low byte = 0xFE,
// data = 64-bit MCU unique identifier.
bool parse_discovery_response(const CanFrame& f, DiscoveryResponse& out);

// ---------------------------------------------------------------------------
// COMM_TYPE_2 — motor feedback
// ---------------------------------------------------------------------------
enum class FaultBits : uint16_t {
  None = 0,
  UnderVoltage = 1u << 0,   // id bit 16
  OverCurrent = 1u << 1,    // id bit 17
  OverTemperature = 1u << 2,  // id bit 18
  MagEncoderFault = 1u << 3,  // id bit 19
  HallFault = 1u << 4,        // id bit 20
  Uncalibrated = 1u << 5,     // id bit 21
};

enum class MotorMode : uint8_t {
  Reset = 0,
  Cali = 1,
  Motor = 2,
};

struct Feedback {
  uint8_t motor_id{0};
  uint8_t host_id{0};
  MotorMode mode{MotorMode::Reset};
  uint16_t faults{0};
  float angle_rad{0.0f};    // decoded from u16 (±4*pi nominal)
  float vel_rad_s{0.0f};    // ±30 rad/s
  float torque_nm{0.0f};    // ±12 N.m
  float temp_c{0.0f};       // raw / 10 (raw is degC * 10)
  uint16_t raw_angle{0};
  uint16_t raw_vel{0};
  uint16_t raw_torque{0};
  uint16_t raw_temp{0};
};
// Identifier: bits 0..7 host CAN ID, 8..15 motor CAN ID, 16..21 faults,
// 22..23 mode/state. Data: u16 big-endian fields.
bool parse_feedback(const CanFrame& f, Feedback& out);

// ---------------------------------------------------------------------------
// Simple commands (3/4/6)
// ---------------------------------------------------------------------------
CanFrame make_enable(uint8_t host_id, uint8_t motor_id);
CanFrame make_stop(uint8_t host_id, uint8_t motor_id);
CanFrame make_set_zero(uint8_t host_id, uint8_t motor_id);

// ---------------------------------------------------------------------------
// Runtime parameter registers, 0x7005..0x7020 (CyberGear_AI_Reference.md §24)
// ---------------------------------------------------------------------------
enum class Reg : uint16_t {
  RunMode = 0x7005,      // uint8: 0 motion, 1 position, 2 speed, 3 current
  IqRef = 0x7006,        // float: current command, -23..23 A
  SpdRef = 0x700A,       // float: speed command, -30..30 rad/s
  LimitTorque = 0x700B,  // float: torque limit, 0..12 N.m (manual: "imit_torque")
  CurKp = 0x7010,
  CurKi = 0x7011,
  CurFiltGain = 0x7014,
  LocRef = 0x7016,       // float: position-mode target angle, rad
  LimitSpd = 0x7017,     // float: position-mode speed limit, 0..30 rad/s
  LimitCur = 0x7018,     // float: speed/position current limit, 0..23 A
  MechPos = 0x7019,      // float R: load-side multi-turn mechanical angle, rad
  Iqf = 0x701A,          // float R: filtered Iq, -23..23 A
  MechVel = 0x701B,      // float R: load-side speed, -30..30 rad/s
  VBus = 0x701C,         // float R: bus voltage, V
  Rotation = 0x701D,     // int16 R/W: turn count
  LocKp = 0x701E,
  SpdKp = 0x701F,
  SpdKi = 0x7020,
};

struct RegInfo {
  Reg reg;
  const char* name;
  bool is_float;   // false => u8 (RunMode) or i16 (Rotation)
  bool read_only;
};

const RegInfo* reg_info(Reg r);
const char* reg_name(Reg r);

// Wire format (matches the legacy POC and official examples):
//   request data : [addr_lo, addr_hi, 0, 0, 0, 0, 0, 0]
//   response data: [addr_lo, addr_hi, ?, ?, value bytes at offset 4..]
CanFrame make_read_reg(Reg r, uint8_t host_id, uint8_t motor_id);
CanFrame make_write_reg_float(Reg r, float v, uint8_t host_id, uint8_t motor_id);
CanFrame make_write_reg_u8(Reg r, uint8_t v, uint8_t host_id, uint8_t motor_id);

// Parses a COMM_TYPE_17 response (accepts COMM_TYPE_18 write echoes too).
// value is returned as double (float registers -> float, Rotation -> i16).
bool parse_reg_response(const CanFrame& f, Reg& reg_out, double& value_out);

// ---------------------------------------------------------------------------
// COMM_TYPE_1 — MIT / motion-control command (big-endian u16 fields)
// ---------------------------------------------------------------------------
// data2 carries the torque field (u16 scaled ±12 N.m).
CanFrame make_mit_command(float torque_nm, float pos_rad, float vel_rad_s,
                          float kp, float kd, uint8_t motor_id);

// Run modes for 0x7005 (CyberGear_AI_Reference.md §24).
enum class RunMode : uint8_t {
  MotionControl = 0,
  Position = 1,
  Speed = 2,
  Current = 3,
};

}  // namespace ota::cybergear
