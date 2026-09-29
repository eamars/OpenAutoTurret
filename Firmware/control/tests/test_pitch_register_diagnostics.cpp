#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "can/cybergear_system.hpp"
#include "can/cybergear_protocol.hpp"
#include "control/can_motor_backend.hpp"

namespace {

using namespace ota;

class RegisterTransport final : public can::CanTransport {
 public:
  FrameCallback callback;
  bool auto_reply = true;
  double spd_ki_readback_override = -1.0;
  std::vector<cybergear::CanFrame> sent;
  std::array<double, 0x7021> registers{};

  RegisterTransport() {
    registers[static_cast<uint16_t>(cybergear::Reg::RunMode)] = 2;
    registers[static_cast<uint16_t>(cybergear::Reg::LimitCur)] = 5;
    registers[static_cast<uint16_t>(cybergear::Reg::LimitSpd)] = 10;
    registers[static_cast<uint16_t>(cybergear::Reg::SpdKp)] = 2;
    registers[static_cast<uint16_t>(cybergear::Reg::SpdKi)] = .01;
    registers[static_cast<uint16_t>(cybergear::Reg::Iqf)] = .2;
    registers[static_cast<uint16_t>(cybergear::Reg::VBus)] = 24;
  }

  bool start(std::string&) override { return true; }
  void stop() override {}
  void set_frame_callback(FrameCallback cb) override { callback = std::move(cb); }
  bool send(uint32_t id, const uint8_t data[8], std::string*) override {
    cybergear::CanFrame request{};
    request.id = id;
    std::copy(data, data + 8, request.data);
    sent.push_back(request);
    const auto e = cybergear::unpack_ext_id(id);
    const auto comm = static_cast<cybergear::CommType>(e.comm_type);
    const uint16_t address = static_cast<uint16_t>(data[0]) |
                             (static_cast<uint16_t>(data[1]) << 8);
    if (comm == cybergear::CommType::WriteReg && address < registers.size()) {
      if (address == static_cast<uint16_t>(cybergear::Reg::RunMode))
        registers[address] = data[4];
      else {
        float value = 0;
        std::memcpy(&value, data + 4, sizeof(value));
        registers[address] = value;
      }
    }
    if (comm == cybergear::CommType::ReadReg && auto_reply) {
      cybergear::CanFrame reply{};
      reply.id = cybergear::pack_ext_id(17, e.target, 0);
      reply.data[0] = data[0]; reply.data[1] = data[1];
      double value = address < registers.size() ? registers[address] : 0;
      if (address == static_cast<uint16_t>(cybergear::Reg::SpdKi) &&
          spd_ki_readback_override >= 0)
        value = spd_ki_readback_override;
      if (address == static_cast<uint16_t>(cybergear::Reg::RunMode)) {
        reply.data[4] = static_cast<uint8_t>(value);
      } else {
        const float f = static_cast<float>(value);
        std::memcpy(reply.data + 4, &f, sizeof(f));
      }
      can::RawFrame raw{};
      raw.id = reply.id; raw.dlc = reply.dlc;
      std::copy(reply.data, reply.data + 8, raw.data);
      raw.rx_ns = now_monotonic_ns();
      callback(raw);
    }
    return true;
  }
  can::BusStats stats() const override { return {}; }
  bool is_up() const override { return true; }
  can::CanIfState can_state() const override { return can::CanIfState::Unknown; }
  const char* kind() const override { return "test"; }
  std::string device() const override { return "offline"; }

  void emit_running_feedback() {
    can::RawFrame f{};
    f.id = cybergear::pack_ext_id(2, 100, 0) | (2u << 22);
    f.data[0] = 0x80; f.data[1] = 0x00;  // angle 0
    f.data[2] = 0x80; f.data[3] = 0x00;  // velocity 0
    f.data[4] = 0x80; f.data[5] = 0x00;  // torque 0
    f.data[6] = 0x00; f.data[7] = 250;  // temperature 25 C
    f.rx_ns = now_monotonic_ns();
    callback(f);
  }
};

struct Fixture {
  can::CyberGearSystem system;
  CanMotorBackend backend{system};
  RegisterTransport* transport = nullptr;

  Fixture() {
    auto bus = std::make_unique<RegisterTransport>();
    transport = bus.get();
    can::CyberGearSystemConfig config;
    config.pitch_motor_id = 100;
    std::string error;
    EXPECT_TRUE(system.open(config, error, std::move(bus))) << error;
  }
};

}  // namespace

TEST(PitchRegisterDiagnostics, SamplesSixRegistersWithRequestAndReceiveTimes) {
  Fixture f;
  TimeNs tick = now_monotonic_ns();
  for (size_t i = 0; i < 6; ++i) {
    f.backend.poll_pitch_register_diagnostics(tick);
    f.backend.poll_pitch_register_diagnostics(tick + 1);
    const auto sample = f.backend.pitch_register_diagnostics().registers[i];
    EXPECT_TRUE(sample.valid);
    EXPECT_EQ(sample.status, 1);
    EXPECT_GT(sample.request_ns, 0);
    EXPECT_GE(sample.rx_ns, sample.request_ns);
    tick += 250'000'001;
  }
}

TEST(PitchRegisterDiagnostics, CancelsItsOutstandingReadBeforeModeSetup) {
  Fixture f;
  f.transport->auto_reply = false;
  const TimeNs now = now_monotonic_ns();
  f.backend.poll_pitch_register_diagnostics(now);
  EXPECT_EQ(f.backend.pitch_register_diagnostics().registers[0].status, 0);
  std::string error;
  EXPECT_EQ(f.backend.transition_mode(AxisId::Pitch, false, 5.0, now + 1, error),
            MotorBackend::Transition::Failed);  // No feedback; fails preflight safely.
  const auto sample = f.backend.pitch_register_diagnostics().registers[0];
  EXPECT_EQ(sample.status, -2);
  EXPECT_FALSE(sample.valid);
  f.transport->auto_reply = true;
  EXPECT_TRUE(f.system.begin_register_read(AxisId::Pitch, cybergear::Reg::Iqf, error));
  f.system.cancel_register_read();
}

TEST(PitchRegisterDiagnostics, GainReadbackMismatchCannotReportComplete) {
  Fixture f;
  f.transport->emit_running_feedback();
  std::string error;
  ASSERT_TRUE(f.backend.adopt_running_mode(AxisId::Pitch, false, error)) << error;
  TimeNs tick = now_monotonic_ns();
  for (size_t i = 0; i < 6; ++i) {
    f.backend.poll_pitch_register_diagnostics(tick);
    f.backend.poll_pitch_register_diagnostics(tick + 1);
    tick += 250'000'001;
  }
  f.transport->spd_ki_readback_override = .02;
  ASSERT_EQ(f.backend.begin_pitch_speed_loop_gain_update(2.2, .015, error),
            MotorBackend::Transition::Pending);
  bool completed = false;
  auto result = MotorBackend::Transition::Pending;
  for (int n = 0; n < 8 && result == MotorBackend::Transition::Pending; ++n) {
    result = f.backend.poll_pitch_speed_loop_gain_update(tick + n * 2, error);
  }
  completed = result == MotorBackend::Transition::Complete;
  EXPECT_FALSE(completed);
  EXPECT_EQ(result, MotorBackend::Transition::Failed);
  EXPECT_NE(error.find("readback"), std::string::npos);
}
