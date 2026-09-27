#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iterator>
#include <mutex>
#include <string>
#include <vector>

#include "can/gm6020_protocol.hpp"
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
