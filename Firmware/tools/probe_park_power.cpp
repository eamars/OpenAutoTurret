// Production control loop + CAN backend/protocol with a simulated loaded plant.
// Counts actual STOP/enable/mode-write frames. No physical transport is opened.
#include <cstring>
#include <iostream>
#include <map>
#include <thread>
#include <spdlog/spdlog.h>
#include "control/can_motor_backend.hpp"
#include "control/control_loop.hpp"

using namespace ota;
class ParkPlant final : public can::CanTransport {
 public:
  struct Motor {
    double q = 0;
    bool enabled = true;
    std::map<uint16_t, double> regs{
      {uint16_t(cybergear::Reg::RunMode), 2},
      {uint16_t(cybergear::Reg::LimitCur), 5},
      {uint16_t(cybergear::Reg::SpdKp), 4},
      {uint16_t(cybergear::Reg::SpdKi), .05},
      {uint16_t(cybergear::Reg::SpdRef), 0}};
  } motors[2];
  FrameCallback callback;
  int stops = 0, enables = 0, mode_writes = 0;
  bool early_stop = false;
  bool recoil = false;
  double maximum_travel[2]{};
  const double targets[2] = {-2*kDeg2Rad, 2*kDeg2Rad};
  bool start(std::string&) override { return true; }
  void stop() override {}
  void set_frame_callback(FrameCallback cb) override { callback = std::move(cb); }
  void feedback(int i) {
    can::RawFrame f{};
    f.id = cybergear::pack_ext_id(2, 100+i, 0) | (motors[i].enabled ? 2u<<22 : 0);
    f.dlc = 8; f.rx_ns = now_monotonic_ns();
    const auto u = cybergear::encode_u16(motors[i].q, -12.5, 12.5);
    f.data[0] = u>>8; f.data[1] = u&255;
    f.data[2] = f.data[4] = 0x80; f.data[7] = 250;
    callback(f);
  }
  void advance(double dt) {
    for (int i=0; i<2; ++i) {
      auto& m = motors[i];
      if (m.enabled) m.q += m.regs[uint16_t(cybergear::Reg::SpdRef)] * dt;
      maximum_travel[i] = std::max(maximum_travel[i], std::abs(m.q));
      feedback(i);
    }
  }
  bool send(uint32_t id, const uint8_t data[8], std::string*) override {
    const auto e = cybergear::unpack_ext_id(id);
    if (e.target != 100 && e.target != 101) return true;
    const int i = e.target - 100;
    auto& m = motors[i];
    const uint16_t reg = uint16_t(data[0]) | (uint16_t(data[1])<<8);
    if (e.comm_type == 4) {
      if (stops == 0)
        for (int j=0; j<2; ++j)
          early_stop |= std::abs(motors[j].q - targets[j]) >= .25*kDeg2Rad;
      ++stops;
      // Reproduce the real defect: removing torque away from the park pose
      // lets a loaded axis move, then the old mode recipe faults on that drift.
      if (early_stop && m.enabled) m.q += .4*kDeg2Rad;
      if (recoil && m.enabled) m.q += .3*kDeg2Rad;
      m.enabled = false;
    }
    if (e.comm_type == 3) { ++enables; m.enabled = true; }
    if (e.comm_type == 18) {
      float value; std::memcpy(&value, data+4, sizeof(value));
      m.regs[reg] = reg == uint16_t(cybergear::Reg::RunMode) ? data[4] : value;
      if (reg == uint16_t(cybergear::Reg::RunMode)) ++mode_writes;
    }
    feedback(i);
    if (e.comm_type == 17) {
      can::RawFrame f{};
      f.id = cybergear::pack_ext_id(17, 100+i, 0);
      f.dlc = 8; f.rx_ns = now_monotonic_ns();
      std::memcpy(f.data, data, 8);
      const float value = reg == uint16_t(cybergear::Reg::MechPos) ? m.q : m.regs[reg];
      std::memcpy(f.data+4, &value, sizeof(value));
      if (reg == uint16_t(cybergear::Reg::RunMode)) f.data[4] = uint8_t(m.regs[reg]);
      callback(f);
    }
    return true;
  }
  can::BusStats stats() const override { return {}; }
  bool is_up() const override { return true; }
  can::CanIfState can_state() const override { return can::CanIfState::Unknown; }
  const char* kind() const override { return "simulated-loaded-park"; }
  std::string device() const override { return "no-hardware"; }
};

class ParkBackend final : public CanMotorBackend {
 public:
  ParkBackend(can::CyberGearSystem& s, ParkPlant& p, bool evidence)
      : CanMotorBackend(s), plant(p), independent(evidence) {}
  ParkPositionEvidence park_position_evidence(AxisId a, TimeNs now) const override {
    if (!independent) return {};  // Installed hardware has no independent sensor.
    ParkPositionEvidence e;
    e.trusted = e.simulated = true; e.sampled_ns = now;
    e.q_raw_rad = plant.motors[static_cast<int>(a)].q;
    return e;
  }
  ParkPlant& plant;
  bool independent;
};

bool run(bool independent, bool approved = false, bool already_parked = false,
         bool residual = false, bool recoil = false) {
  auto transport = std::make_unique<ParkPlant>(); auto* plant = transport.get();
  plant->recoil = recoil;
  can::CyberGearSystem system; std::string error;
  if (!system.open({}, error, std::move(transport))) return false;
  if (already_parked)
    for (int i=0; i<2; ++i) plant->motors[i].q = plant->targets[i];
  plant->advance(0);
  ControlLoop::Config cfg;
  cfg.service_speed_control = true; cfg.service_speed_kp = 4; cfg.service_speed_ki = .05;
  cfg.park.park_logical_deg = {-2, 2}; cfg.park.dwell_ms = 150;
  cfg.park.require_independent_position = !approved;
  ControlLoop loop(cfg, std::make_unique<ParkBackend>(system, *plant, independent));
  std::array<AxisLogicalModel,2> models;
  std::array<AxisLimits,2> limits;
  for (int i=0; i<2; ++i) {
    models[i].set_reference(0,0);
    limits[i].set_from_endpoints(-90*kDeg2Rad,90*kDeg2Rad,5*kDeg2Rad);
  }
  if (!loop.restore_retained_homing(models,limits,error)) {
    std::cerr << error << '\n'; return false;
  }
  auto now = now_monotonic_ns();
  loop.step(now,5'000'000);
  if (!loop.start_parking(error)) { std::cerr << error << '\n'; return false; }
  const auto began = now;
  bool injected = false;
  while (loop.phase() == Phase::Parking && now-began < 12'000'000'000LL) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    const auto next = now_monotonic_ns();
    plant->advance(double(next-now)/1e9);
    if (residual && !injected && plant->motors[0].q < plant->targets[0] + .1*kDeg2Rad) {
      plant->motors[1].q += .35*kDeg2Rad;
      plant->feedback(1);
      injected = true;
    }
    now = now_monotonic_ns();  // Feedback must precede this control sample.
    loop.step(now,5'000'000);
  }
  const bool moved = plant->maximum_travel[0] > 1.75*kDeg2Rad &&
                     plant->maximum_travel[1] > 1.75*kDeg2Rad;
  const bool ok = moved && !plant->early_stop && plant->enables == 0 && plant->mode_writes == 0 &&
    (independent || approved ? loop.phase() == Phase::Parked && plant->stops == 2
                 : loop.phase() == Phase::Fault && plant->stops == 0 &&
                   loop.fault_reason().find("independent physical") != std::string::npos);
  std::cout << "independent=" << independent << " approved=" << approved
    << " already_parked=" << already_parked << " residual_injected=" << injected
    << " release_recoil=" << recoil
    << " both_axes_at_target=" << moved
    << " stop_frames=" << plant->stops << " early_stop=" << plant->early_stop
    << " enable_frames=" << plant->enables << " mode_writes=" << plant->mode_writes
    << " fault='" << loop.fault_reason() << "' pass=" << ok << '\n';
  return ok;
}

int main() {
  spdlog::set_level(spdlog::level::err);
  const bool without_sensor = run(false);
  const bool with_simulated_sensor = run(true);
  const bool approved_pose = run(false, true);
  const bool already_parked = run(false, true, true);
  const bool settles_residual = run(false, true, false, true);
  const bool accepts_recoil = run(false, true, false, false, true);
  return without_sensor && with_simulated_sensor && approved_pose && already_parked &&
         settles_residual && accepts_recoil ? 0 : 1;
}
