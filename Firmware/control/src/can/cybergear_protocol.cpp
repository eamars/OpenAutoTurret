#include "cybergear_protocol.hpp"

namespace ota::cybergear {

namespace {

uint16_t be16(const uint8_t* p) {
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

void put_be16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v >> 8);
  p[1] = static_cast<uint8_t>(v & 0xFF);
}

uint16_t le16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

void put_le16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>(v >> 8);
}

uint32_t le32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

void put_le32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

CanFrame base_frame(CommType type, uint16_t data2, uint8_t target) {
  CanFrame f;
  f.id = pack_ext_id(static_cast<uint8_t>(type), data2, target);
  f.dlc = 8;
  return f;
}

}  // namespace

// ---------------------------------------------------------------------------
// Discovery
// ---------------------------------------------------------------------------
CanFrame make_discovery_request(uint8_t host_id, uint8_t motor_id) {
  CanFrame f = base_frame(CommType::Discovery, host_id, motor_id);
  for (int i = 0; i < 8; ++i) f.data[i] = 0;
  return f;
}

bool parse_discovery_response(const CanFrame& f, DiscoveryResponse& out) {
  const ExtId e = unpack_ext_id(f.id);
  if (e.comm_type != static_cast<uint8_t>(CommType::Discovery)) return false;
  if (e.target != 0xFE) return false;  // response low byte is 0xFE
  out.motor_id = static_cast<uint8_t>(e.data2 & 0xFF);
  out.unique_id = 0;
  for (int i = 0; i < 8; ++i) {
    out.unique_id = (out.unique_id << 8) | f.data[i];
  }
  return true;
}

// ---------------------------------------------------------------------------
// Feedback
// ---------------------------------------------------------------------------
bool parse_feedback(const CanFrame& f, Feedback& out) {
  const ExtId e = unpack_ext_id(f.id);
  if (e.comm_type != static_cast<uint8_t>(CommType::Feedback)) return false;

  out.motor_id = static_cast<uint8_t>((f.id >> 8) & 0xFF);
  out.host_id = static_cast<uint8_t>(f.id & 0xFF);
  out.faults = static_cast<uint16_t>((f.id >> 16) & 0x3F);
  out.mode = static_cast<MotorMode>((f.id >> 22) & 0x3);

  out.raw_angle = be16(f.data + 0);
  out.raw_vel = be16(f.data + 2);
  out.raw_torque = be16(f.data + 4);
  out.raw_temp = be16(f.data + 6);

  out.angle_rad = decode_u16(out.raw_angle, Ranges::kFbAngleMin, Ranges::kFbAngleMax);
  out.vel_rad_s = decode_u16(out.raw_vel, Ranges::kVelMin, Ranges::kVelMax);
  out.torque_nm = decode_u16(out.raw_torque, Ranges::kTorqueMin, Ranges::kTorqueMax);
  out.temp_c = static_cast<float>(out.raw_temp) / 10.0f;
  return true;
}

// ---------------------------------------------------------------------------
// Simple commands
// ---------------------------------------------------------------------------
CanFrame make_enable(uint8_t host_id, uint8_t motor_id) {
  return base_frame(CommType::Enable, host_id, motor_id);
}

CanFrame make_stop(uint8_t host_id, uint8_t motor_id) {
  return base_frame(CommType::Stop, host_id, motor_id);
}

CanFrame make_set_zero(uint8_t host_id, uint8_t motor_id) {
  CanFrame f = base_frame(CommType::SetZero, host_id, motor_id);
  f.data[0] = 1;  // official example: 01 00 00 00 00 00 00 00
  return f;
}

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------
namespace {
const RegInfo kRegTable[] = {
    {Reg::RunMode, "run_mode", false, false},
    {Reg::IqRef, "iq_ref", true, false},
    {Reg::SpdRef, "spd_ref", true, false},
    {Reg::LimitTorque, "limit_torque", true, false},
    {Reg::CurKp, "cur_kp", true, false},
    {Reg::CurKi, "cur_ki", true, false},
    {Reg::CurFiltGain, "cur_filt_gain", true, false},
    {Reg::LocRef, "loc_ref", true, false},
    {Reg::LimitSpd, "limit_spd", true, false},
    {Reg::LimitCur, "limit_cur", true, false},
    {Reg::MechPos, "mechPos", true, true},
    {Reg::Iqf, "iqf", true, true},
    {Reg::MechVel, "mechVel", true, true},
    {Reg::VBus, "VBUS", true, true},
    {Reg::Rotation, "rotation", false, false},
    {Reg::LocKp, "loc_kp", true, false},
    {Reg::SpdKp, "spd_kp", true, false},
    {Reg::SpdKi, "spd_ki", true, false},
};
}  // namespace

const RegInfo* reg_info(Reg r) {
  for (const auto& e : kRegTable) {
    if (e.reg == r) return &e;
  }
  return nullptr;
}

const char* reg_name(Reg r) {
  const RegInfo* i = reg_info(r);
  return i ? i->name : "?";
}

CanFrame make_read_reg(Reg r, uint8_t host_id, uint8_t motor_id) {
  CanFrame f = base_frame(CommType::ReadReg, host_id, motor_id);
  put_le16(f.data + 0, static_cast<uint16_t>(r));
  return f;
}

CanFrame make_write_reg_float(Reg r, float v, uint8_t host_id, uint8_t motor_id) {
  CanFrame f = base_frame(CommType::WriteReg, host_id, motor_id);
  put_le16(f.data + 0, static_cast<uint16_t>(r));
  put_le16(f.data + 2, 0);
  uint32_t bits;
  static_assert(sizeof(bits) == sizeof(v));
  std::memcpy(&bits, &v, sizeof(bits));
  put_le32(f.data + 4, bits);
  return f;
}

CanFrame make_write_reg_u8(Reg r, uint8_t v, uint8_t host_id, uint8_t motor_id) {
  CanFrame f = base_frame(CommType::WriteReg, host_id, motor_id);
  put_le16(f.data + 0, static_cast<uint16_t>(r));
  put_le16(f.data + 2, 0);
  f.data[4] = v;  // value in low byte, remaining bytes zero
  return f;
}

bool parse_reg_response(const CanFrame& f, Reg& reg_out, double& value_out) {
  const ExtId e = unpack_ext_id(f.id);
  const uint8_t ct = e.comm_type;
  if (ct != static_cast<uint8_t>(CommType::ReadReg) &&
      ct != static_cast<uint8_t>(CommType::WriteReg)) {
    return false;
  }
  const uint16_t addr = le16(f.data + 0);
  // Find the register; unknown addresses are rejected (diagnostic: count it).
  RegInfo* match = nullptr;
  for (const auto& entry : kRegTable) {
    if (static_cast<uint16_t>(entry.reg) == addr) {
      match = const_cast<RegInfo*>(&entry);
      break;
    }
  }
  if (match == nullptr) return false;

  reg_out = match->reg;
  if (match->is_float) {
    uint32_t bits = le32(f.data + 4);
    float fv;
    std::memcpy(&fv, &bits, sizeof(fv));
    value_out = static_cast<double>(fv);
  } else if (match->reg == Reg::Rotation) {
    uint16_t u = le16(f.data + 4);
    value_out = static_cast<double>(static_cast<int16_t>(u));
  } else {
    // u8 register (run_mode): value in low byte.
    value_out = static_cast<double>(f.data[4]);
  }
  return true;
}

// ---------------------------------------------------------------------------
// MIT / motion-control command
// ---------------------------------------------------------------------------
CanFrame make_mit_command(float torque_nm, float pos_rad, float vel_rad_s,
                          float kp, float kd, uint8_t motor_id) {
  const uint16_t t = encode_u16(torque_nm, Ranges::kTorqueMin, Ranges::kTorqueMax);
  CanFrame f = base_frame(CommType::Mit, t, motor_id);
  put_be16(f.data + 0, encode_u16(pos_rad, Ranges::kPosMin, Ranges::kPosMax));
  put_be16(f.data + 2, encode_u16(vel_rad_s, Ranges::kVelMin, Ranges::kVelMax));
  put_be16(f.data + 4, encode_u16(kp, Ranges::kKpMin, Ranges::kKpMax));
  put_be16(f.data + 6, encode_u16(kd, Ranges::kKdMin, Ranges::kKdMax));
  return f;
}

}  // namespace ota::cybergear
