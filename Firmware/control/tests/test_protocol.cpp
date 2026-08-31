// CyberGear protocol encode/decode tests (architecture §54.1).
#include <cmath>

#include <gtest/gtest.h>

#include "can/cybergear_protocol.hpp"

using namespace ota::cybergear;

TEST(Protocol, ExtIdPackUnpackRoundTrip) {
  const uint32_t id = pack_ext_id(2, 0x0102, 0x64);
  const ExtId e = unpack_ext_id(id);
  EXPECT_EQ(e.comm_type, 2);
  EXPECT_EQ(e.data2, 0x0102);
  EXPECT_EQ(e.target, 0x64);
}

TEST(Protocol, U16ScaleRoundTrip) {
  for (float x : {-12.0f, -1.234f, 0.0f, 0.7f, 12.0f}) {
    const uint16_t u = encode_u16(x, Ranges::kTorqueMin, Ranges::kTorqueMax);
    const float back = decode_u16(u, Ranges::kTorqueMin, Ranges::kTorqueMax);
    EXPECT_NEAR(back, x, 0.05f);
  }
}

TEST(Protocol, U16ScaleClamps) {
  EXPECT_EQ(encode_u16(-99.0f, Ranges::kTorqueMin, Ranges::kTorqueMax), 0);
  EXPECT_EQ(encode_u16(99.0f, Ranges::kTorqueMin, Ranges::kTorqueMax), 65535);
}

TEST(Protocol, DiscoveryRequestLayout) {
  auto f = make_discovery_request(0, 100);
  EXPECT_EQ(f.dlc, 8);
  const ExtId e = unpack_ext_id(f.id);
  EXPECT_EQ(e.comm_type, static_cast<uint8_t>(CommType::Discovery));
  EXPECT_EQ(e.data2, 0);  // host id in bits 15..8
  EXPECT_EQ(e.target, 100);
  for (int i = 0; i < 8; ++i) EXPECT_EQ(f.data[i], 0);
}

TEST(Protocol, DiscoveryResponseParse) {
  CanFrame f;
  // comm=0, data2 = motor id, target = 0xFE, 64-bit unique id in payload.
  f.id = pack_ext_id(0, 100, 0xFE);
  f.dlc = 8;
  for (int i = 0; i < 8; ++i) f.data[i] = static_cast<uint8_t>(0xA5 >> (i % 8));
  DiscoveryResponse r;
  ASSERT_TRUE(parse_discovery_response(f, r));
  EXPECT_EQ(r.motor_id, 100);
  EXPECT_NE(r.unique_id, 0);
}

TEST(Protocol, DiscoveryResponseRejectsWrongTarget) {
  CanFrame f;
  f.id = pack_ext_id(0, 100, 0x64);  // not 0xFE
  DiscoveryResponse r;
  EXPECT_FALSE(parse_discovery_response(f, r));
}

namespace {
// Build a feedback frame per CyberGear_AI_Reference.md §15:
// bits 8..15 = motor CAN ID, bits 0..7 = host CAN ID, bits 16..21 = faults,
// bits 22..23 = mode.
CanFrame make_feedback_frame(uint16_t angle, uint16_t vel, uint16_t torque,
                             uint16_t temp, uint8_t motor_id,
                             uint8_t host_id = 0) {
  CanFrame f;
  f.id = pack_ext_id(2, motor_id, host_id);
  // faults bits 16..21 (0), mode bits 22..23 (2 = motor)
  f.id |= (0u << 16) | (2u << 22);
  f.dlc = 8;
  f.data[0] = static_cast<uint8_t>(angle >> 8);
  f.data[1] = static_cast<uint8_t>(angle & 0xFF);
  f.data[2] = static_cast<uint8_t>(vel >> 8);
  f.data[3] = static_cast<uint8_t>(vel & 0xFF);
  f.data[4] = static_cast<uint8_t>(torque >> 8);
  f.data[5] = static_cast<uint8_t>(torque & 0xFF);
  f.data[6] = static_cast<uint8_t>(temp >> 8);
  f.data[7] = static_cast<uint8_t>(temp & 0xFF);
  return f;
}
}  // namespace

TEST(Protocol, FeedbackParseFields) {
  auto f = make_feedback_frame(0x4000, 0x8000, 0x8000, 350, 100);
  Feedback fb;
  ASSERT_TRUE(parse_feedback(f, fb));
  EXPECT_EQ(fb.motor_id, 100);
  EXPECT_EQ(fb.host_id, 0);
  EXPECT_EQ(fb.mode, MotorMode::Motor);
  EXPECT_EQ(fb.faults, 0);
  // angle: u16 0x4000 decoded in the feedback angle range (±12.5 rad, verified
  // to match the MIT position mapping on live hardware).
  EXPECT_NEAR(fb.angle_rad, decode_u16(0x4000, Ranges::kFbAngleMin, Ranges::kFbAngleMax), 1e-6);
  EXPECT_NEAR(fb.vel_rad_s, decode_u16(0x8000, Ranges::kVelMin, Ranges::kVelMax), 1e-6);
  EXPECT_NEAR(fb.torque_nm, decode_u16(0x8000, Ranges::kTorqueMin, Ranges::kTorqueMax), 1e-6);
  EXPECT_NEAR(fb.temp_c, 35.0f, 1e-4);
}

TEST(Protocol, FeedbackParseFaultAndModeBits) {
  CanFrame f = make_feedback_frame(0, 0, 0, 0, 101);
  f.id |= (1u << 18) | (1u << 16);  // over-temp + under-voltage
  f.id = (f.id & ~(3u << 22)) | (0u << 22);  // mode reset
  Feedback fb;
  ASSERT_TRUE(parse_feedback(f, fb));
  EXPECT_EQ(fb.motor_id, 101);
  EXPECT_TRUE((fb.faults & static_cast<uint16_t>(FaultBits::OverTemperature)) != 0);
  EXPECT_TRUE((fb.faults & static_cast<uint16_t>(FaultBits::UnderVoltage)) != 0);
  EXPECT_EQ(fb.mode, MotorMode::Reset);
}

TEST(Protocol, FeedbackRejectsWrongCommType) {
  auto f = make_feedback_frame(0, 0, 0, 0, 100);
  f.id = pack_ext_id(1, 0, 100);  // MIT, not feedback
  Feedback fb;
  EXPECT_FALSE(parse_feedback(f, fb));
}

TEST(Protocol, SimpleCommandIds) {
  EXPECT_EQ(unpack_ext_id(make_enable(0, 100).id).comm_type,
            static_cast<uint8_t>(CommType::Enable));
  EXPECT_EQ(unpack_ext_id(make_stop(0, 101).id).comm_type,
            static_cast<uint8_t>(CommType::Stop));
  auto z = make_set_zero(0, 100);
  EXPECT_EQ(unpack_ext_id(z.id).comm_type, static_cast<uint8_t>(CommType::SetZero));
  EXPECT_EQ(z.data[0], 1);
}

TEST(Protocol, RegRequestLayoutLittleEndian) {
  auto f = make_read_reg(Reg::LocRef, 0, 100);
  EXPECT_EQ(f.data[0], 0x16);  // 0x7016 lo
  EXPECT_EQ(f.data[1], 0x70);  // 0x7016 hi
  EXPECT_EQ(f.data[2], 0);
  EXPECT_EQ(unpack_ext_id(f.id).comm_type, static_cast<uint8_t>(CommType::ReadReg));
  EXPECT_EQ(unpack_ext_id(f.id).target, 100);
}

TEST(Protocol, RegFloatWriteLayout) {
  auto f = make_write_reg_float(Reg::SpdRef, 1.5f, 0, 100);
  EXPECT_EQ(f.data[0], 0x0A);  // 0x700A lo
  EXPECT_EQ(f.data[1], 0x70);
  uint32_t bits;
  std::memcpy(&bits, &f.data[4], 4);
  float v;
  std::memcpy(&v, &bits, 4);
  EXPECT_FLOAT_EQ(v, 1.5f);
}

TEST(Protocol, RegResponseParse) {
  // Build a read-reg response for LocRef with value 0.25.
  CanFrame f;
  f.id = pack_ext_id(17, 0, 100);
  f.dlc = 8;
  f.data[0] = 0x16;
  f.data[1] = 0x70;
  uint32_t bits;
  float v = 0.25f;
  std::memcpy(&bits, &v, 4);
  f.data[4] = bits & 0xFF;
  f.data[5] = (bits >> 8) & 0xFF;
  f.data[6] = (bits >> 16) & 0xFF;
  f.data[7] = (bits >> 24) & 0xFF;
  Reg r;
  double out = 0;
  ASSERT_TRUE(parse_reg_response(f, r, out));
  EXPECT_EQ(r, Reg::LocRef);
  EXPECT_NEAR(out, 0.25, 1e-7);
}

TEST(Protocol, RegResponseParseU8RunMode) {
  CanFrame f;
  f.id = pack_ext_id(17, 0, 100);
  f.data[0] = 0x05;
  f.data[1] = 0x70;
  f.data[4] = 2;
  Reg r;
  double out = 0;
  ASSERT_TRUE(parse_reg_response(f, r, out));
  EXPECT_EQ(r, Reg::RunMode);
  EXPECT_DOUBLE_EQ(out, 2.0);
}

TEST(Protocol, RegResponseRejectsUnknownAddr) {
  CanFrame f;
  f.id = pack_ext_id(17, 0, 100);
  f.data[0] = 0x00;
  f.data[1] = 0x20;  // 0x2000: not in runtime table
  Reg r;
  double out;
  EXPECT_FALSE(parse_reg_response(f, r, out));
}

TEST(Protocol, MitCommandBigEndianFields) {
  auto f = make_mit_command(1.0f, 0.5f, 2.0f, 10.0f, 0.5f, 100);
  const ExtId e = unpack_ext_id(f.id);
  EXPECT_EQ(e.comm_type, static_cast<uint8_t>(CommType::Mit));
  EXPECT_EQ(e.target, 100);
  EXPECT_EQ(e.data2, encode_u16(1.0f, Ranges::kTorqueMin, Ranges::kTorqueMax));
  const uint16_t pos = static_cast<uint16_t>((f.data[0] << 8) | f.data[1]);
  EXPECT_EQ(pos, encode_u16(0.5f, Ranges::kPosMin, Ranges::kPosMax));
  const uint16_t vel = static_cast<uint16_t>((f.data[2] << 8) | f.data[3]);
  EXPECT_EQ(vel, encode_u16(2.0f, Ranges::kVelMin, Ranges::kVelMax));
}

// Empirical finding (live CyberGear, 2026-08): the motor does NOT free-run
// COMM_TYPE_2 feedback by default. Instead each control command (COMM_TYPE_1
// MIT, COMM_TYPE_3 enable, COMM_TYPE_4 stop, COMM_TYPE_18 register write) is
// answered by one COMM_TYPE_2 feedback frame (CyberGear_AI_Reference.md §25.1).
// The 200 Hz loop therefore drives feedback by sending MIT references; the
// feedback angle uses the same ±12.5 rad mapping as the MIT position command
// (the manual's "±4*pi" is a source inconsistency).
TEST(Protocol, FeedbackAngleRangeMatchesMitPosition) {
  EXPECT_FLOAT_EQ(Ranges::kFbAngleMin, Ranges::kPosMin);
  EXPECT_FLOAT_EQ(Ranges::kFbAngleMax, Ranges::kPosMax);

  // Round-trip: encode a position into a MIT frame, then decode the same raw
  // u16 as a feedback angle -> recover the original angle.
  const float x = -0.87f;
  const auto mit = make_mit_command(0.0f, x, 0.0f, 50.0f, 0.0f, 100);
  const uint16_t raw = static_cast<uint16_t>((mit.data[0] << 8) | mit.data[1]);
  const auto fb_frame = make_feedback_frame(raw, 0x7FFF, 0x7FFF, 246, 100);
  Feedback fb;
  ASSERT_TRUE(parse_feedback(fb_frame, fb));
  EXPECT_NEAR(fb.angle_rad, x, 0.01f);
}

