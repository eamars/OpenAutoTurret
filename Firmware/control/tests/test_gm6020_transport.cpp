#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <filesystem>
#include <iterator>
#include <mutex>
#include <string>
#include <vector>

#include "can/gm6020_protocol.hpp"
#include "can/gm6020_velocity.hpp"
#include "can/gm6020_friction.hpp"
#include "can/socketcan_bus.hpp"
#include "can/yousee_transport.hpp"
#include "can/cybergear_protocol.hpp"

namespace {

using ota::can::RawFrame;

RawFrame feedback_frame(uint32_t id = 0x205) {
  RawFrame f{};
  f.id = id;
  f.extended = false;
  f.dlc = 8;
  f.data[0] = 0x12;
  f.data[1] = 0x34;
  f.data[2] = 0xFF;
  f.data[3] = 0xFE;
  f.data[4] = 0x80;
  f.data[5] = 0x01;
  f.data[6] = 42;
  f.rx_ns = 123456;
  return f;
}

TEST(GM6020Codec, DecodesStandardFeedbackAndRejectsWrongFrameTypes) {
  ota::gm6020::Feedback decoded{};
  const RawFrame valid = feedback_frame();
  ASSERT_TRUE(ota::gm6020::decode(valid, 1, decoded));
  EXPECT_EQ(decoded.angle_count, 0x1234);
  EXPECT_EQ(decoded.speed_rpm, -2);
  EXPECT_EQ(decoded.current_raw, -32767);
  EXPECT_EQ(decoded.temperature_raw, 42);
  EXPECT_EQ(decoded.rx_ns, 123456);
  // Amperes, by the same scale the command side uses. The accessor is a unit conversion
  // and not a clamp: this frame reports -32767, beyond the +-16384 the command side can
  // ask for, and inventing a ceiling here would hide exactly the overload a friction
  // investigation goes looking for.
  EXPECT_DOUBLE_EQ(decoded.current_a(), -32767 * (3.0 / 16384.0));
  auto at_full = valid;
  at_full.data[4] = 0x40;
  at_full.data[5] = 0x00;  // raw +16384
  ASSERT_TRUE(ota::gm6020::decode(at_full, 1, decoded));
  EXPECT_DOUBLE_EQ(decoded.current_a(), 3.0);  // the documented endpoint, exactly
  at_full.data[4] = 0xC0;
  at_full.data[5] = 0x00;  // raw -16384
  ASSERT_TRUE(ota::gm6020::decode(at_full, 1, decoded));
  EXPECT_DOUBLE_EQ(decoded.current_a(), -3.0);

  auto wrong = valid;
  wrong.extended = true;  // Same numeric ID, wrong CAN frame type.
  EXPECT_FALSE(ota::gm6020::decode(wrong, 1, decoded));
  wrong = valid;
  wrong.rtr = true;
  EXPECT_FALSE(ota::gm6020::decode(wrong, 1, decoded));
  wrong = valid;
  wrong.error = true;
  EXPECT_FALSE(ota::gm6020::decode(wrong, 1, decoded));
  wrong = valid;
  wrong.dlc = 7;
  EXPECT_FALSE(ota::gm6020::decode(wrong, 1, decoded));
  wrong.dlc = 9;
  EXPECT_FALSE(ota::gm6020::decode(wrong, 1, decoded));
  wrong = valid;
  wrong.id++;
  EXPECT_FALSE(ota::gm6020::decode(wrong, 1, decoded));
  wrong = valid;
  wrong.data[0] = 0x20;
  wrong.data[1] = 0x00;  // Encoder count outside [0, 8191].
  EXPECT_FALSE(ota::gm6020::decode(wrong, 1, decoded));
  EXPECT_FALSE(ota::gm6020::decode(valid, 0, decoded));
  EXPECT_FALSE(ota::gm6020::decode(valid, 8, decoded));
}

TEST(GM6020Codec, BuildsSingleMotorVoltageGroupAndValidatesLimits) {
  for (uint8_t motor_id = 1; motor_id <= 7; ++motor_id) {
    const auto positive = ota::gm6020::voltage_frame(motor_id, 25000);
    const uint32_t expected_group = motor_id <= 4 ? 0x1FF : 0x2FF;
    const size_t slot = static_cast<size_t>((motor_id - 1) % 4);
    ASSERT_FALSE(positive.extended);
    EXPECT_FALSE(positive.rtr);
    EXPECT_FALSE(positive.error);
    EXPECT_EQ(positive.id, expected_group);
    EXPECT_EQ(positive.dlc, 8);
    EXPECT_EQ(positive.data[slot * 2], 0x61);
    EXPECT_EQ(positive.data[slot * 2 + 1], 0xA8);  // 25000 = 0x61A8.
    for (size_t other = 0; other < 4; ++other) {
      if (other == slot) continue;
      EXPECT_EQ(positive.data[other * 2], 0);
      EXPECT_EQ(positive.data[other * 2 + 1], 0);
    }

    const auto negative = ota::gm6020::voltage_frame(motor_id, -25000);
    EXPECT_EQ(negative.data[slot * 2], 0x9E);
    EXPECT_EQ(negative.data[slot * 2 + 1], 0x58);  // -25000 as int16 = 0x9E58.
  }

  EXPECT_THROW(ota::gm6020::voltage_frame(0, 0), std::invalid_argument);
  EXPECT_THROW(ota::gm6020::voltage_frame(8, 0), std::invalid_argument);
  EXPECT_THROW(ota::gm6020::voltage_frame(1, 25001), std::invalid_argument);
  EXPECT_THROW(ota::gm6020::voltage_frame(1, -25001), std::invalid_argument);
}

TEST(GM6020Encoder, UnwrapsForwardAndReverseModuloCrossings) {
  ota::gm6020::UnwrappedEncoder encoder;
  ASSERT_TRUE(encoder.update(8190, 1'000'000'000));
  ASSERT_TRUE(encoder.update(2, 1'001'000'000));
  EXPECT_NEAR(encoder.relative_rad(), 4 * ota::gm6020::UnwrappedEncoder::kRadiansPerCount,
              1e-12);
  ASSERT_TRUE(encoder.update(8190, 1'002'000'000));
  EXPECT_NEAR(encoder.relative_rad(), 0, 1e-12);

  // Continue over multiple complete turns in both directions.
  int count = 8190;
  int64_t expected_counts = 0;
  ota::TimeNs stamp = 2'000'000'000;
  ota::gm6020::UnwrappedEncoder many_turns;
  ASSERT_TRUE(many_turns.update(static_cast<uint16_t>(count), stamp));
  for (int step = 0; step < 200; ++step) {
    count = (count + (step < 100 ? 100 : -100) + 8192) % 8192;
    expected_counts += step < 100 ? 100 : -100;
    stamp += 10'000'000;
    ASSERT_TRUE(many_turns.update(static_cast<uint16_t>(count), stamp));
  }
  EXPECT_NEAR(many_turns.relative_rad(),
              expected_counts * ota::gm6020::UnwrappedEncoder::kRadiansPerCount,
              1e-10);
}

TEST(GM6020Encoder, RejectsAmbiguousOrReorderedSamplesAndLatchesInvalidity) {
  using ota::gm6020::UnwrappedEncoder;
  UnwrappedEncoder duplicate_time;
  ASSERT_TRUE(duplicate_time.update(10, 1'000'000'000));
  EXPECT_FALSE(duplicate_time.update(10, 1'000'000'000));
  EXPECT_FALSE(duplicate_time.valid());
  EXPECT_FALSE(duplicate_time.update(11, 1'001'000'000));  // Latched until reset.

  UnwrappedEncoder reordered;
  ASSERT_TRUE(reordered.update(10, 2'000'000'000));
  EXPECT_FALSE(reordered.update(11, 1'999'000'000));
  EXPECT_FALSE(reordered.valid());

  UnwrappedEncoder stale_gap;
  ASSERT_TRUE(stale_gap.update(10, 3'000'000'000));
  EXPECT_FALSE(stale_gap.update(10, 3'081'000'000));
  EXPECT_FALSE(stale_gap.valid());

  UnwrappedEncoder scheduled_gap;
  ASSERT_TRUE(scheduled_gap.update(10, 3'000'000'000));
  EXPECT_TRUE(scheduled_gap.update(10, 3'066'000'000));
  EXPECT_TRUE(scheduled_gap.valid());

  UnwrappedEncoder impossible_delta;
  ASSERT_TRUE(impossible_delta.update(10, 4'000'000'000));
  EXPECT_FALSE(impossible_delta.update(1010, 4'010'000'000));
  EXPECT_FALSE(impossible_delta.valid());

  UnwrappedEncoder bad_count;
  EXPECT_FALSE(bad_count.update(8192, 5'000'000'000));
  EXPECT_FALSE(bad_count.valid());
  bad_count.reset();
  EXPECT_TRUE(bad_count.update(0, 6'000'000'000));
  EXPECT_TRUE(bad_count.valid());
}

// Station, 2026-10-02 20:02:50: two frames received 3 us apart (bunched in the receive queue; the
// motor samples every 1 ms), 3 counts apart at 14 rpm, latched the old unwrap invalid and faulted
// the station for good. Production's Recover policy: the device period bounds the time between
// readings, a doubtful reading is skipped, a run of them is believed, and gaps never latch.
TEST(GM6020Encoder, RecoverPolicyNeverLatchesOnBunchedFramesGapsOrGlitches) {
  using ota::gm6020::UnwrappedEncoder;
  constexpr double k = UnwrappedEncoder::kRadiansPerCount;
  UnwrappedEncoder e(UnwrappedEncoder::Policy::Recover);
  ASSERT_TRUE(e.update(838, 1'000'000'000));
  EXPECT_TRUE(e.update(841, 1'000'003'000)) << "the station's 3 us pair";
  EXPECT_TRUE(e.update(841, 1'000'003'000)) << "a duplicate stamp";
  EXPECT_TRUE(e.update(843, 1'000'002'000)) << "a reordered stamp";
  EXPECT_NEAR(e.relative_rad(), 5 * k, 1e-12);
  // A corrupt reading half a turn away is skipped, and the next good one continues.
  EXPECT_FALSE(e.update(4900, 1'001'000'000));
  EXPECT_TRUE(e.valid());
  EXPECT_TRUE(e.update(845, 1'002'000'000));
  EXPECT_NEAR(e.relative_rad(), 7 * k, 1e-12);
  // A 400 ms gap (the old unwrap latched beyond 80 ms) re-establishes the turn by nearest count.
  EXPECT_TRUE(e.update(900, 1'402'000'000));
  EXPECT_NEAR(e.relative_rad(), 62 * k, 1e-12);
  EXPECT_EQ(e.long_gaps(), 1u);
  // A real jump the unwrap did not see coming (readings consistently elsewhere) is believed after a run.
  int accepted_at = -1;
  for (int i = 0; i < 30 && accepted_at < 0; ++i)
    if (e.update(3000, 1'403'000'000 + i * 1'000)) accepted_at = i;
  EXPECT_GE(accepted_at, 1);
  EXPECT_LT(accepted_at, UnwrappedEncoder::kBelieveAfter);
  EXPECT_TRUE(e.valid());
  // Malformed frames are skipped, never latched.
  EXPECT_FALSE(e.update(8192, 1'500'000'000));
  EXPECT_FALSE(e.update(3000, 0));
  EXPECT_TRUE(e.valid());
  EXPECT_TRUE(e.update(3001, 1'501'000'000));
}

TEST(YouseeTransport, RefusesTypedStandardFrameWithoutOpeningAdapter) {
  ota::can::YouseeTransport::Options options;
  ota::can::YouseeTransport transport(options);
  RawFrame frame{};
  frame.id = 0x205;
  frame.extended = false;
  std::string error;
  EXPECT_FALSE(transport.send_frame(frame, &error));
  EXPECT_NE(error.find("standard CAN transmit is unsupported"), std::string::npos);
  EXPECT_EQ(transport.stats().tx_failed, 1u);
}

TEST(CyberGearProtocol, RejectsCapturedUnsupportedRegisterReplyAndAcceptsValidReply) {
  ota::cybergear::CanFrame unsupported{};
  unsupported.id = 0x11017F00;
  unsupported.dlc = 8;
  const uint8_t stale_reply[8] = {0x19, 0x70, 0x00, 0x00,
                                  0x30, 0x33, 0x31, 0x05};
  std::copy(std::begin(stale_reply), std::end(stale_reply), unsupported.data);
  ota::cybergear::Reg reg = ota::cybergear::Reg::MechPos;
  double value = 123.0;
  EXPECT_FALSE(ota::cybergear::parse_reg_response(unsupported, reg, value));
  EXPECT_DOUBLE_EQ(value, 123.0);  // Refusal must not publish stale payload data.

  ota::cybergear::CanFrame valid{};
  valid.id = 0x11007F00;
  valid.dlc = 8;
  // 0x7018 (LimitCur), successful response header, little-endian float 27.0.
  const uint8_t successful_reply[8] = {0x18, 0x70, 0x00, 0x00,
                                       0x00, 0x00, 0xD8, 0x41};
  std::copy(std::begin(successful_reply), std::end(successful_reply), valid.data);
  ASSERT_TRUE(ota::cybergear::parse_reg_response(valid, reg, value));
  EXPECT_EQ(reg, ota::cybergear::Reg::LimitCur);
  EXPECT_DOUBLE_EQ(value, 27.0);
}

// Optional actual SocketCAN routing probe. It only uses a Linux virtual CAN
// interface with the exact vcan0 name and virtual sysfs backing; it never
// brings an interface up, configures it, or opens a physical CAN interface.
TEST(SocketCanTypedFrames, VcanRoutesSffEffAndRtrWithoutLosingFlags) {
  const char* enabled = std::getenv("OTA_TEST_VCAN");
  if (!enabled || std::string(enabled) != "1")
    GTEST_SKIP() << "set OTA_TEST_VCAN=1 to run the isolated vcan probe";

  std::error_code ec;
  const auto sys_path = std::filesystem::canonical("/sys/class/net/vcan0", ec);
  if (ec || sys_path.generic_string().find("/devices/virtual/net/vcan0") == std::string::npos)
    GTEST_SKIP() << "vcan0 is absent or is not backed by Linux virtual-net sysfs";

  // Callback targets are declared before buses so early ASSERT/GTEST_SKIP
  // exits destroy and join the RX owner before destroying captured state.
  std::mutex mutex;
  std::condition_variable changed;
  std::vector<RawFrame> received;
  ota::can::SocketCanBus tx, rx;
  ota::can::SocketCanBus::Options options;
  options.iface = "vcan0";
  options.install_filters = false;
  options.receive_error_frames = false;
  std::string error;
  ASSERT_TRUE(tx.open(options, error)) << error;
  ASSERT_TRUE(rx.open(options, error)) << error;
  if (!tx.is_up() || !rx.is_up()) GTEST_SKIP() << "vcan0 must already be up";

  rx.set_frame_callback([&](const RawFrame& frame) {
    {
      std::lock_guard lock(mutex);
      received.push_back(frame);
    }
    changed.notify_one();
  });
  ASSERT_TRUE(rx.start_rx(error)) << error;

  auto round_trip = [&](RawFrame sent, RawFrame& got) {
    size_t previous = 0;
    {
      std::lock_guard lock(mutex);
      previous = received.size();
    }
    if (!tx.send_frame(sent, &error)) return false;
    std::unique_lock lock(mutex);
    if (!changed.wait_for(lock, std::chrono::seconds(1), [&] {
      return received.size() > previous;
    })) {
      error = "timed out waiting for vcan loopback";
      return false;
    }
    got = received.back();
    return true;
  };

  RawFrame standard{};
  standard.id = 0x205;
  standard.extended = false;
  standard.dlc = 3;
  standard.data[0] = 0xA1;
  standard.data[1] = 0xB2;
  standard.data[2] = 0xC3;
  RawFrame got{};
  ASSERT_TRUE(round_trip(standard, got)) << error;
  EXPECT_EQ(got.id, standard.id);
  EXPECT_FALSE(got.extended);
  EXPECT_FALSE(got.rtr);
  EXPECT_EQ(got.dlc, 3);
  EXPECT_EQ(got.data[0], 0xA1);
  EXPECT_EQ(got.data[1], 0xB2);
  EXPECT_EQ(got.data[2], 0xC3);

  RawFrame extended = standard;
  extended.extended = true;  // Same numeric ID, different SocketCAN type.
  ASSERT_TRUE(round_trip(extended, got)) << error;
  EXPECT_EQ(got.id, extended.id);
  EXPECT_TRUE(got.extended);

  RawFrame remote{};
  remote.id = 0x205;
  remote.extended = false;
  remote.rtr = true;
  remote.dlc = 5;
  ASSERT_TRUE(round_trip(remote, got)) << error;
  EXPECT_EQ(got.id, remote.id);
  EXPECT_FALSE(got.extended);
  EXPECT_TRUE(got.rtr);
  EXPECT_EQ(got.dlc, remote.dlc);

  RawFrame invalid{};
  invalid.id = 0x205;
  invalid.extended = false;
  invalid.error = true;
  EXPECT_FALSE(tx.send_frame(invalid, &error));
  invalid.error = false;
  invalid.dlc = 9;
  EXPECT_FALSE(tx.send_frame(invalid, &error));

  // The legacy overload retains its documented EFF, eight-byte behavior.
  const uint8_t payload[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  size_t previous = 0;
  {
    std::lock_guard lock(mutex);
    previous = received.size();
  }
  ASSERT_TRUE(tx.send(0x205, payload, &error)) << error;
  std::unique_lock lock(mutex);
  ASSERT_TRUE(changed.wait_for(lock, std::chrono::seconds(1), [&] {
    return received.size() > previous;
  })) << "timed out waiting for legacy EFF loopback";
  EXPECT_TRUE(received.back().extended);
  EXPECT_EQ(received.back().dlc, 8);
}

}  // namespace


// Torque-current command path (migration from voltage). Encoding is checked against the
// documented endpoints, the clamp is checked in amperes, and the unused slots are checked to be
// zero because a leftover byte there would command a motor this turret does not have.
TEST(Gm6020Current, Id1FrameIsStandardDlc8On0x1FEWithZeroedUnusedSlots) {
  const auto f = ota::gm6020::current_frame(1, 0.8, 0.8);
  EXPECT_EQ(f.id, 0x1feu);
  EXPECT_FALSE(f.extended);
  EXPECT_EQ(f.dlc, 8);
  EXPECT_EQ(f.data[0], 0x11);   // 0.8 A -> 4369 raw -> 0x1111, big-endian
  EXPECT_EQ(f.data[1], 0x11);
  for (int i = 2; i < 8; ++i) EXPECT_EQ(f.data[i], 0) << "unused slot " << i;
}

TEST(Gm6020Current, NegativeCurrentIsTwoSComplementBigEndian) {
  const auto f = ota::gm6020::current_frame(1, -0.5, 0.8);
  EXPECT_EQ(f.data[0], 0xF5);   // -2731 -> 0xF555
  EXPECT_EQ(f.data[1], 0x55);
}

TEST(Gm6020Current, ScaleMatchesTheDocumentedEndpoints) {
  EXPECT_EQ(ota::gm6020::current_raw_uncapped(0.0), 0);
  EXPECT_EQ(ota::gm6020::current_raw_uncapped(3.0), 16384);
  EXPECT_EQ(ota::gm6020::current_raw_uncapped(-3.0), -16384);
}

TEST(Gm6020Current, HostLimitClampsBeforeEncoding) {
  // 2.0 A demanded against a 0.8 A ceiling must arrive at the motor as 0.8 A, not as 2.0 A and
  // not as a saturated surprise -- and not as 0 either.
  EXPECT_EQ(ota::gm6020::current_raw_from_amps(2.0, 0.8), ota::gm6020::current_raw_uncapped(0.8));
  EXPECT_EQ(ota::gm6020::current_raw_from_amps(-2.0, 0.8), ota::gm6020::current_raw_uncapped(-0.8));
}

TEST(Gm6020Current, UnreasonableCommandsAndLimitsFailClosed) {
  EXPECT_THROW(ota::gm6020::current_frame(1, NAN, 0.8), std::invalid_argument);
  EXPECT_THROW(ota::gm6020::current_frame(1, 0.5, 0.0), std::invalid_argument);
  EXPECT_THROW(ota::gm6020::current_frame(1, 0.5, -1.0), std::invalid_argument);
  // Above the continuous rating the software refuses rather than quietly agreeing.
  EXPECT_THROW(ota::gm6020::current_frame(1, 0.5, 3.0), std::invalid_argument);
  // Only ID 1 is qualified; a second yaw motor is an assumption, not a fact.
  EXPECT_THROW(ota::gm6020::current_frame(2, 0.5, 0.8), std::invalid_argument);
  EXPECT_THROW(ota::gm6020::current_raw_uncapped(4.0), std::invalid_argument);
}

// The zero frame is what every stop path sends, including the ones that run while something is
// already wrong. It may not throw (a throwing fault path is a std::terminate waiting to happen),
// and it may not leave a stale byte in another slot.
TEST(Gm6020Current, ZeroFrameIsAnEmptyPayloadOnTheCurrentId) {
  const auto f = ota::gm6020::current_zero_frame(1);
  EXPECT_EQ(f.id, 0x1feu);
  EXPECT_EQ(f.dlc, 8);
  EXPECT_FALSE(f.extended);
  EXPECT_FALSE(f.rtr);
  for (const auto byte : f.data) EXPECT_EQ(byte, 0);
  // 0x2FF/0x1FF cover IDs 5-7; the current frame does not, and pretending otherwise would put a
  // motor on a frame that cannot carry it.
  EXPECT_THROW(ota::gm6020::current_zero_frame(5), std::invalid_argument);
}

// The velocity loop's amperes output. Three separate guarantees: the number is in amperes (so it
// is bounded by an ampere ceiling, not by 25000 counts), it flips sign with the demand, and a
// broken input yields zero effort instead of a stale one.
TEST(Gm6020CurrentLoop, OutputIsAmperesAndCannotOutrunTheCeiling) {
  constexpr double kCeiling = 0.8;  // axes.yaw.host_current_limit_a
  ota::gm6020::VelocityLoop loop;
  loop.reset(0.0, 1'000'000);
  // 5 ms later, holding position, asked for 0.3 rad/s with kp 1.0 A per rad/s.
  const double first = loop.update_amps(0.3, 0.0, 6'000'000, 0.524, kCeiling, 1.0, 0.6);
  EXPECT_TRUE(loop.valid());
  EXPECT_GT(first, 0.25);
  EXPECT_LE(first, kCeiling);

  // A gain inherited from voltage mode would sail past the envelope; the ceiling is what catches
  // it, and it must land exactly on the limit rather than wrapping or growing.
  loop.reset(0.0, 1'000'000);
  const double saturated = loop.update_amps(0.5, 0.0, 6'000'000, 0.524, kCeiling, 20000.0, 0.6);
  EXPECT_EQ(std::abs(saturated), kCeiling);

  // Reversing the demand reverses the torque, or the axis can never be centred.
  loop.reset(0.0, 1'000'000);
  const double back = loop.update_amps(-0.3, 0.0, 6'000'000, 0.524, kCeiling, 1.0, 0.6);
  EXPECT_LT(back, 0.0);
}

TEST(Gm6020CurrentLoop, BadInputsProduceZeroEffortAndLatchInvalid) {
  ota::gm6020::VelocityLoop loop;
  loop.reset(0.0, 1'000'000);
  EXPECT_DOUBLE_EQ(loop.update_amps(NAN, 0.0, 6'000'000, 0.524, 0.8, 1.0, 0.6), 0.0);
  EXPECT_FALSE(loop.valid());  // latched: the caller trips, it does not keep driving blind

  ota::gm6020::VelocityLoop reference_too_big;
  reference_too_big.reset(0.0, 1'000'000);
  EXPECT_DOUBLE_EQ(reference_too_big.update_amps(0.9, 0.0, 6'000'000, 0.524, 0.8, 1.0, 0.6), 0.0);
  EXPECT_FALSE(reference_too_big.valid());

  // A ceiling that is not a positive finite number is a missing envelope, not "no limit".
  ota::gm6020::VelocityLoop no_ceiling;
  no_ceiling.reset(0.0, 1'000'000);
  EXPECT_DOUBLE_EQ(no_ceiling.update_amps(0.3, 0.0, 6'000'000, 0.524, 0.0, 1.0, 0.6), 0.0);
  EXPECT_FALSE(no_ceiling.valid());
  ota::gm6020::VelocityLoop nan_ceiling;
  nan_ceiling.reset(0.0, 1'000'000);
  EXPECT_DOUBLE_EQ(nan_ceiling.update_amps(0.3, 0.0, 6'000'000, 0.524, NAN, 1.0, 0.6), 0.0);
  EXPECT_FALSE(nan_ceiling.valid());
}

// The voltage path keeps its own bound, which the shared PI no longer knows about: 25000 counts is
// a fact about the voltage frame, and it must not be what limits an ampere request.
TEST(Gm6020CurrentLoop, VoltageWrapperKeepsItsOwnFrameBound) {
  ota::gm6020::VelocityLoop loop;
  loop.reset(0.0, 1'000'000);
  EXPECT_EQ(loop.update(0.3, 0.0, 6'000'000, 0.524, 25001.0, 1000.0, 10.0), 0);
  EXPECT_FALSE(loop.valid());
  loop.reset(0.0, 1'000'000);
  EXPECT_GT(loop.update(0.3, 0.0, 6'000'000, 0.524, 15000.0, 1000.0, 10.0), 0);
  EXPECT_TRUE(loop.valid());
}

TEST(Gm6020CurrentLoop, FreshLateCycleFreezesIntegralAndRecovers) {
  ota::gm6020::VelocityLoop loop;
  loop.reset(0, 1'000'000);
  const auto first = loop.update_amps(.1, 0, 6'000'000, .524, .8, 1, .6);
  const auto integral = loop.integral();
  EXPECT_DOUBLE_EQ(first, loop.update_amps(.1, 0, 31'000'000, .524, .8, 1, .6));
  EXPECT_TRUE(loop.valid());
  EXPECT_TRUE(loop.late_cycle());
  EXPECT_DOUBLE_EQ(integral, loop.integral());
  EXPECT_GT(loop.update_amps(.1, 0, 36'000'000, .524, .8, 1, .6), first);
  EXPECT_FALSE(loop.late_cycle());
  EXPECT_DOUBLE_EQ(0, loop.update_amps(.1, 0, 35'000'000, .524, .8, 1, .6));
  EXPECT_FALSE(loop.valid());
  loop.reset(0, 1'000'000);
  EXPECT_DOUBLE_EQ(0, loop.update_amps(.1, 0, 102'000'000, .524, .8, 1, .6));
  EXPECT_FALSE(loop.valid());
}

TEST(Gm6020Friction, OneAttemptPerLeaseAndMovingHandoff) {
  ota::gm6020::FrictionConfig cfg{true, .30, .35, .10, .12, .10, .01, .005, 2, 1.0};
  ota::gm6020::YawFrictionCompensation friction;
  auto out = friction.update(cfg, true, 1, 0, 0, 1, .005, .8);
  EXPECT_EQ(out.state, ota::gm6020::FrictionState::Breakaway);
  EXPECT_DOUBLE_EQ(out.feedforward_target_a, .30);
  EXPECT_TRUE(out.new_attempt);
  // Renewing the same intent does not reinitialize the attempt; unique RX samples
  // must establish displacement before the helper hands control back to PI.
  out = friction.update(cfg, true, 1, -.011, 1, 2, .005, .8);
  EXPECT_EQ(out.state, ota::gm6020::FrictionState::Breakaway);
  EXPECT_FALSE(out.new_attempt);
  out = friction.update(cfg, true, 1, .011, 1, 3, .005, .8);
  EXPECT_EQ(out.state, ota::gm6020::FrictionState::Breakaway);
  out = friction.update(cfg, true, 1, .012, 1, 4, .005, .8);
  EXPECT_EQ(out.state, ota::gm6020::FrictionState::Moving);
  EXPECT_TRUE(out.integral_handoff);
  EXPECT_DOUBLE_EQ(out.feedforward_target_a, .10);
  EXPECT_DOUBLE_EQ(out.previous_feedforward_a, .30);
  EXPECT_DOUBLE_EQ(out.next_feedforward_a, .10);
  out = friction.update(cfg, false, 0, .012, 1, 5, .005, .8);
  EXPECT_TRUE(out.integral_handoff);
  EXPECT_DOUBLE_EQ(out.previous_feedforward_a, .10);
  EXPECT_DOUBLE_EQ(out.next_feedforward_a, 0);
}

TEST(Gm6020Friction, QuietHoldExhaustionAndReverseWaitAreBounded) {
  ota::gm6020::FrictionConfig cfg{true, .3, .3, .1, .1, .010, .02, .005, 2, 2.0};
  ota::gm6020::YawFrictionCompensation friction;
  auto out = friction.update(cfg, false, 0, 0, 0, 1, .005, .8);
  EXPECT_EQ(out.feedforward_target_a, 0);
  out = friction.update(cfg, true, 1, 0, 0, 2, .005, .8);
  for (int i = 0; i < 3; ++i) out = friction.update(cfg, true, 1, 0, 0, 3 + i, .005, .8);
  EXPECT_TRUE(out.attempt_exhausted);
  EXPECT_FALSE(out.attempt_active);
  cfg.timeout_s = .1; // observe the reverse gate before its next attempt expires
  out = friction.update(cfg, true, -1, 0, 0, 6, .005, .8);
  EXPECT_TRUE(out.waiting_for_stationary);
  out = friction.update(cfg, true, -1, 0, 0, 7, .005, .8);
  out = friction.update(cfg, true, -1, 0, 0, 8, .005, .8);
  EXPECT_FALSE(out.waiting_for_stationary);
  EXPECT_EQ(out.state, ota::gm6020::FrictionState::Breakaway);
}

TEST(Gm6020Friction, RejectsUnboundedOrNonFiniteCalibration) {
  ota::gm6020::FrictionConfig cfg{true, NAN, .3, .1, .1, .1, .01, .005, 2, 1.0};
  EXPECT_FALSE(cfg.valid(.8));
  cfg.positive_breakaway_a = .9;
  EXPECT_FALSE(cfg.valid(.8));
}

TEST(Gm6020Friction, OpposingVelocityWaitsAndZeroDirectionDoesNotRearm) {
  ota::gm6020::FrictionConfig cfg{true, .3, .3, .1, .1, .1, .02, .005, 2, 2.0};
  ota::gm6020::YawFrictionCompensation friction;
  auto out = friction.update(cfg, true, 1, 0, -.05, 1, .005, .8);
  EXPECT_TRUE(out.waiting_for_stationary);
  EXPECT_FALSE(out.new_attempt);
  out = friction.update(cfg, true, 1, 0, 0, 2, .005, .8);
  EXPECT_TRUE(out.waiting_for_stationary);
  out = friction.update(cfg, true, 1, 0, 0, 3, .005, .8);
  EXPECT_TRUE(out.new_attempt);
  out = friction.update(cfg, true, 1, 0, 0, 4, .005, .8);
  EXPECT_FALSE(out.new_attempt);
  out = friction.update(cfg, true, 0, 0, 0, 5, .005, .8);
  EXPECT_TRUE(out.integral_handoff);
  EXPECT_EQ(out.feedforward_target_a, 0);
  out = friction.update(cfg, true, 1, 0, 0, 6, .005, .8);
  EXPECT_FALSE(out.new_attempt);
}
