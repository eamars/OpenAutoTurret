#include "can/cybergear_system.hpp"

#include <chrono>

namespace ota::can {

namespace {
// Discovery responses carry the motor ID in data2 and 0xFE in the target
// byte (CyberGear_AI_Reference.md §13).
constexpr uint8_t kDiscoveryResponseTarget = 0xFE;
}  // namespace

bool CyberGearSystem::open(const CyberGearSystemConfig& cfg, std::string& err) {
  close();
  cfg_ = cfg;

  SocketCanBus::Options bopts{};
  bopts.iface = cfg_.iface;
  bopts.bitrate = cfg_.bitrate;
  bopts.bring_up_if_down = cfg_.bring_up_if_down;
  bopts.install_filters = true;
  if (!bus_.open(bopts, err)) return false;

  // Size the per-axis history rings before the RX thread starts writing.
  for (auto& ax : axes_) ax.reset_history(cfg_.history_capacity);

  bus_.set_frame_callback([this](const RawFrame& f) { on_frame(f); });
  std::string rerr;
  if (!bus_.start_rx(rerr)) {
    bus_.close();
    err = "start_rx: " + rerr;
    return false;
  }
  return true;
}

void CyberGearSystem::close() { bus_.close(); }

void CyberGearSystem::on_frame(const RawFrame& f) {
  cybergear::CanFrame cf;
  cf.id = f.id;
  cf.dlc = f.dlc;
  std::memcpy(cf.data, f.data, sizeof(cf.data));
  const cybergear::ExtId e = cybergear::unpack_ext_id(f.id);

  // Per-axis feedback fan-out (RX thread is the sole writer of axis state).
  // Feedback ID layout (§15): motor id in bits 8..15, host id in bits 0..7.
  if (e.comm_type == static_cast<uint8_t>(cybergear::CommType::Feedback)) {
    const uint8_t motor = static_cast<uint8_t>((f.id >> 8) & 0xFF);
    if (motor == cfg_.pitch_motor_id) {
      cybergear::Feedback fb;
      if (cybergear::parse_feedback(cf, fb)) {
        axis(AxisId::Pitch).on_feedback(fb, f.rx_ns);
      }
    } else if (motor == cfg_.yaw_motor_id) {
      cybergear::Feedback fb;
      if (cybergear::parse_feedback(cf, fb)) {
        axis(AxisId::Yaw).on_feedback(fb, f.rx_ns);
      }
    }
  }

  // Synchronous setup-path request/response. Single-flight: only one
  // pending request exists at a time, so matching on comm type (plus the
  // discovery target byte) is unambiguous.
  {
    std::lock_guard lk(pend_mtx_);
    if (!pending_.active) return;
    if (e.comm_type != pending_.comm) return;
    if (pending_.match_target != 0 && e.target != pending_.match_target) return;
    pending_.frame = cf;
    pending_.active = false;
  }
  pend_cv_.notify_all();
}

bool CyberGearSystem::wait_response(uint8_t comm_type, uint8_t match_target,
                                    cybergear::CanFrame& out, int timeout_ms,
                                    std::string* err) {
  {
    std::lock_guard lk(pend_mtx_);
    if (pending_.active) {
      if (err) *err = "another synchronous request already in flight";
      return false;
    }
    pending_.active = true;
    pending_.motor = 0;  // reserved
    pending_.comm = comm_type;
    pending_.match_target = match_target;
    pending_.frame = cybergear::CanFrame{};
  }
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::unique_lock lk(pend_mtx_);
  const bool ok =
      pend_cv_.wait_until(lk, deadline, [this] { return !pending_.active; });
  if (!ok) {
    pending_.active = false;
    if (err) *err = "timeout waiting for CAN response";
    return false;
  }
  pending_.active = false;
  out = pending_.frame;
  return true;
}

bool CyberGearSystem::discover(AxisId axis, uint64_t& unique_id, int timeout_ms,
                               std::string* err) {
  const uint8_t motor = motor_id(axis);
  auto f = cybergear::make_discovery_request(cfg_.host_can_id, motor);
  if (!send(f.id, f.data, err)) return false;

  cybergear::CanFrame resp;
  if (!wait_response(static_cast<uint8_t>(cybergear::CommType::Discovery),
                     kDiscoveryResponseTarget, resp, timeout_ms, err)) {
    return false;
  }
  cybergear::DiscoveryResponse dr;
  if (!cybergear::parse_discovery_response(resp, dr)) {
    if (err) *err = "malformed discovery response";
    return false;
  }
  if (dr.motor_id != motor) {
    if (err) {
      *err = "discovery response motor id mismatch";
    }
    return false;
  }
  unique_id = dr.unique_id;
  return true;
}

bool CyberGearSystem::read_register(AxisId axis, cybergear::Reg reg, double& value,
                                    int timeout_ms, std::string* err) {
  const uint8_t motor = motor_id(axis);
  auto f = cybergear::make_read_reg(reg, cfg_.host_can_id, motor);
  if (!send(f.id, f.data, err)) return false;

  cybergear::CanFrame resp;
  if (!wait_response(static_cast<uint8_t>(cybergear::CommType::ReadReg), 0, resp,
                     timeout_ms, err)) {
    return false;
  }
  cybergear::Reg r = cybergear::Reg::RunMode;
  double v = 0.0;
  if (!cybergear::parse_reg_response(resp, r, v)) {
    if (err) *err = "malformed register response";
    return false;
  }
  if (r != reg) {
    if (err) *err = "register response address mismatch";
    return false;
  }
  value = v;
  return true;
}

bool CyberGearSystem::send(uint32_t ext_id, const uint8_t data[8], std::string* err) {
  return bus_.send(ext_id, data, err);
}

bool CyberGearSystem::send_enable(AxisId axis, std::string* err) {
  auto f = cybergear::make_enable(cfg_.host_can_id, motor_id(axis));
  return send(f.id, f.data, err);
}

bool CyberGearSystem::send_stop(AxisId axis, std::string* err) {
  auto f = cybergear::make_stop(cfg_.host_can_id, motor_id(axis));
  return send(f.id, f.data, err);
}

bool CyberGearSystem::send_set_zero(AxisId axis, std::string* err) {
  auto f = cybergear::make_set_zero(cfg_.host_can_id, motor_id(axis));
  return send(f.id, f.data, err);
}

}  // namespace ota::can
