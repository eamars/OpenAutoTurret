#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "can/cybergear_system.hpp"
#include "can/cybergear_protocol.hpp"
#include "control/can_motor_backend.hpp"

namespace {

class RecordingTransport final : public ota::can::CanTransport {
 public:
  bool start(std::string&) override { up_ = true; return true; }
  void stop() override { up_ = false; }
  void set_frame_callback(FrameCallback cb) override { callback_ = std::move(cb); }
  bool send(uint32_t id, const uint8_t data[8], std::string*) override {
    Frame frame{};
    frame.id = id;
    std::copy(data, data + 8, frame.data.begin());
    sent.push_back(frame);
    return true;
  }
  ota::can::BusStats stats() const override { return {}; }
  bool is_up() const override { return up_; }
  ota::can::CanIfState can_state() const override { return ota::can::CanIfState::Unknown; }
  const char* kind() const override { return "test"; }
  std::string device() const override { return "recording"; }

  struct Frame { uint32_t id{}; std::array<uint8_t, 8> data{}; };
  std::vector<Frame> sent;

 private:
  bool up_{false};
  FrameCallback callback_;
};

struct OpenSystem {
  ota::can::CyberGearSystem system;
  RecordingTransport* transport{nullptr};

  OpenSystem() {
    auto bus = std::make_unique<RecordingTransport>();
    transport = bus.get();
    ota::can::CyberGearSystemConfig cfg;
    std::string err;
    EXPECT_TRUE(system.open(cfg, err, std::move(bus))) << err;
  }
};

}  // namespace

TEST(PitchCurrentSafety, RawPitchCommandsCannotBypassSafeSetupOrFiveAmpCeiling) {
  OpenSystem fixture;
  const auto pitch = fixture.system.motor_id(ota::AxisId::Pitch);
  const auto yaw = fixture.system.motor_id(ota::AxisId::Yaw);
  std::string err;

  EXPECT_FALSE(fixture.system.send_enable(ota::AxisId::Pitch, &err));
  EXPECT_TRUE(fixture.transport->sent.empty());

  auto mit = ota::cybergear::make_mit_command(1.0f, 0.0f, 0.0f, 1.0f, 0.1f,
                                             pitch);
  EXPECT_FALSE(fixture.system.send(mit.id, mit.data, &err));

  auto iq = ota::cybergear::make_write_reg_float(ota::cybergear::Reg::IqRef,
                                                  1.0f, 0, pitch);
  EXPECT_FALSE(fixture.system.send(iq.id, iq.data, &err));

  auto too_high = ota::cybergear::make_write_reg_float(
      ota::cybergear::Reg::LimitCur, 5.01f, 0, pitch);
  EXPECT_FALSE(fixture.system.send(too_high.id, too_high.data, &err));
  auto nan_limit = ota::cybergear::make_write_reg_float(
      ota::cybergear::Reg::LimitCur,
      std::numeric_limits<float>::quiet_NaN(), 0, pitch);
  EXPECT_FALSE(fixture.system.send(nan_limit.id, nan_limit.data, &err));

  auto safe_limit = ota::cybergear::make_write_reg_float(
      ota::cybergear::Reg::LimitCur, 5.0f, 0, pitch);
  EXPECT_TRUE(fixture.system.send(safe_limit.id, safe_limit.data, &err)) << err;
  EXPECT_FALSE(fixture.system.send_enable(ota::AxisId::Pitch, &err));

  // The station-specific cap does not alter legacy yaw drive behavior.
  auto yaw_mit = ota::cybergear::make_mit_command(1.0f, 0.0f, 0.0f, 1.0f,
                                                  0.1f, yaw);
  EXPECT_TRUE(fixture.system.send(yaw_mit.id, yaw_mit.data, &err)) << err;
  auto yaw_limit = ota::cybergear::make_write_reg_float(
      ota::cybergear::Reg::LimitCur, 8.0f, 0, yaw);
  EXPECT_TRUE(fixture.system.send(yaw_limit.id, yaw_limit.data, &err)) << err;
}

TEST(PitchCurrentSafety, BackendRejectsAdaptiveRequestAboveFiveAndStops) {
  OpenSystem fixture;
  ota::CanMotorBackend backend(fixture.system);
  backend.set_current_limit(ota::AxisId::Pitch, 5.01);
  ASSERT_EQ(fixture.transport->sent.size(), 1u);
  EXPECT_EQ(ota::cybergear::unpack_ext_id(fixture.transport->sent[0].id).comm_type,
            static_cast<uint8_t>(ota::cybergear::CommType::Stop));

  fixture.transport->sent.clear();
  backend.set_current_limit(ota::AxisId::Pitch,
                            std::numeric_limits<double>::infinity());
  ASSERT_EQ(fixture.transport->sent.size(), 1u);
  EXPECT_EQ(ota::cybergear::unpack_ext_id(fixture.transport->sent[0].id).comm_type,
            static_cast<uint8_t>(ota::cybergear::CommType::Stop));

  fixture.transport->sent.clear();
  std::string err;
  EXPECT_FALSE(backend.enter_speed_mode(ota::AxisId::Pitch, 5.01, err));
  ASSERT_EQ(fixture.transport->sent.size(), 1u);
  EXPECT_EQ(ota::cybergear::unpack_ext_id(fixture.transport->sent[0].id).comm_type,
            static_cast<uint8_t>(ota::cybergear::CommType::Stop));
}
