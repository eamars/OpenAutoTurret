#include "can/cybergear_system.hpp"

#include <chrono>
#include <spdlog/spdlog.h>

#include "can/socketcan_bus.hpp"
#include "can/yousee_transport.hpp"

namespace ota::can {

namespace {
// Discovery responses carry the motor ID in data2 and 0xFE in the target
// byte (CyberGear_AI_Reference.md §13).
constexpr uint8_t kDiscoveryResponseTarget = 0xFE;
}  // namespace

bool CyberGearSystem::open(const CyberGearSystemConfig& cfg, std::string& err,
                           std::unique_ptr<CanTransport> transport) {
  close();
  cfg_ = cfg;

  // PHY factory: everything below this point is transport-agnostic.
  if (transport) {
    bus_ = std::move(transport);
  } else if (cfg_.transport == "yousee") {
    YouseeTransport::Options yo{};
    yo.port = cfg_.iface;
    yo.uart_baud = cfg_.uart_baud;
    yo.can_bitrate = cfg_.bitrate;
    bus_ = std::make_unique<YouseeTransport>(yo);
  } else if (cfg_.transport == "socketcan" || cfg_.transport.empty()) {
    SocketCanBus::Options bo{};
    bo.iface = cfg_.iface;
    bo.bitrate = cfg_.bitrate;
    bo.bring_up_if_down = cfg_.bring_up_if_down;
    bo.install_filters = true;
    bus_ = std::make_unique<SocketCanBus>(bo);
  } else {
    err = "unknown can transport '" + cfg_.transport +
          "' (expected socketcan|yousee)";
    return false;
  }

  // Size the per-axis history rings before the RX thread starts writing.
  for (auto& ax : axes_) ax.reset_history(cfg_.history_capacity);

  bus_->set_frame_callback([this](const RawFrame& f) { on_frame(f); });
  if (!bus_->start(err)) {
    bus_.reset();
    return false;
  }
  return true;
}

void CyberGearSystem::close() {
  watchdog_stop_.store(true);
  if (watchdog_.joinable()) watchdog_.join();
  if (bus_) {
    bus_->stop();
    bus_.reset();
  }
}

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

  // Delayed or unrelated responses must not verify another motor's write.
  {
    std::lock_guard lk(pend_mtx_);
    if (!pending_.active || pending_.received || cf.dlc != 8) return;
    if (e.comm_type != pending_.comm) return;
    if (e.target != pending_.match_target) return;
    if ((e.data2 & 0xff) != pending_.motor) return;
    if (pending_.comm == static_cast<uint8_t>(cybergear::CommType::ReadReg) &&
        (uint16_t(cf.data[0]) | (uint16_t(cf.data[1]) << 8)) != pending_.address) return;
    pending_.frame = cf;
    pending_.received = true;
  }
  pend_cv_.notify_all();
}

bool CyberGearSystem::transact(const cybergear::CanFrame& request, uint8_t reply_target,
                               cybergear::CanFrame& out, int timeout_ms,
                               std::string* err) {
  {
    std::lock_guard lk(pend_mtx_);
    if (pending_.active) {
      if (err) *err = "another synchronous request already in flight";
      return false;
    }
    pending_.active = true;
    pending_.asynchronous = false;
    pending_.received = false;
    const auto id = cybergear::unpack_ext_id(request.id);
    pending_.motor = id.target;
    pending_.comm = id.comm_type;
    pending_.match_target = reply_target;
    pending_.address = uint16_t(request.data[0]) | (uint16_t(request.data[1]) << 8);
    pending_.frame = cybergear::CanFrame{};
  }
  // Arm before TX: an immediate response can arrive inside send().
  if (!send(request.id, request.data, err)) {
    std::lock_guard lk(pend_mtx_);
    pending_.active = false;
    return false;
  }
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::unique_lock lk(pend_mtx_);
  const bool ok =
      pend_cv_.wait_until(lk, deadline, [this] { return pending_.received; });
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
  cybergear::CanFrame resp;
  if (!transact(f, kDiscoveryResponseTarget, resp, timeout_ms, err)) {
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
  cybergear::CanFrame resp;
  if (!transact(f, cfg_.host_can_id, resp, timeout_ms, err)) {
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

void CyberGearSystem::start_watchdog() {
  if (watchdog_.joinable()) return;
  watchdog_stop_.store(false);
  heartbeat();
  watchdog_ = std::thread([this] {
    while (!watchdog_stop_.load()) {
      const auto now = now_monotonic_ns();
      const auto heartbeat_age = now - heartbeat_ns_.load();
      bool trip = heartbeat_age > 100000000LL;
      std::array<AxisLatest, 2> observed{};
      for (auto axis_id : {AxisId::Pitch, AxisId::Yaw}) {
        auto& s = observed[static_cast<int>(axis_id)];
        if (axis(axis_id).latest(s) && s.has_feedback && s.mode == 2 &&
            (now - s.rx_ns > 100000000LL || s.faults || s.temp_c > 75.0)) trip = true;
      }
      bool first_trip = false;
      if (trip) {
        std::lock_guard lock(command_mutex_);
        first_trip = !motion_inhibited_.exchange(true);
      }
      if (motion_inhibited_.load()) {
        send_stop(AxisId::Pitch);
        send_stop(AxisId::Yaw);
      }
      if (first_trip) {
        // Record the evidence after issuing both stops. A generic fault label
        // cannot distinguish a stalled host from lost feedback or a drive fault.
        for (int i = 0; i < 2; ++i) {
          const auto& s = observed[i];
          spdlog::error("motor watchdog: heartbeat_age_ms={:.3f} axis={} feedback_age_ms={:.3f} mode={} faults={} temp_c={:.1f}",
              heartbeat_age/1e6, i, s.has_feedback ? (now-s.rx_ns)/1e6 : -1.0,
              s.mode, s.faults, s.temp_c);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  });
}

bool CyberGearSystem::begin_register_read(AxisId axis, cybergear::Reg reg, std::string& err) {
  const auto f = cybergear::make_read_reg(reg, cfg_.host_can_id, motor_id(axis));
  {
    std::lock_guard lk(pend_mtx_);
    if (pending_.active) { err = "register request already active"; return false; }
    pending_ = Pending{};
    pending_.active = pending_.asynchronous = true;
    pending_.motor = motor_id(axis);
    pending_.comm = static_cast<uint8_t>(cybergear::CommType::ReadReg);
    pending_.match_target = cfg_.host_can_id;
    pending_.address = static_cast<uint16_t>(reg);
  }
  if (send(f.id, f.data, &err)) return true;
  cancel_register_read();
  return false;
}

int CyberGearSystem::poll_register_read(double& value, std::string& err) {
  std::lock_guard lk(pend_mtx_);
  if (!pending_.active || !pending_.asynchronous) { err = "no asynchronous read"; return -1; }
  if (!pending_.received) return 0;
  pending_.active = false;
  cybergear::Reg reg{};
  if (!cybergear::parse_reg_response(pending_.frame, reg, value) ||
      static_cast<uint16_t>(reg) != pending_.address) {
    err = "malformed register response"; return -1;
  }
  return 1;
}

void CyberGearSystem::cancel_register_read() {
  std::lock_guard lk(pend_mtx_);
  if (pending_.asynchronous) pending_.active = false;
}

bool CyberGearSystem::read_parameter_raw(AxisId axis, uint16_t address,
                                         std::array<uint8_t, 4>& value,
                                         int timeout_ms, std::string* err) {
  const auto request = cybergear::make_read_reg(static_cast<cybergear::Reg>(address),
                                               cfg_.host_can_id, motor_id(axis));
  cybergear::CanFrame response;
  if (!transact(request, cfg_.host_can_id, response, timeout_ms, err)) return false;
  std::memcpy(value.data(), response.data + 4, value.size());
  return true;
}

bool CyberGearSystem::send(uint32_t ext_id, const uint8_t data[8], std::string* err) {
  std::lock_guard lock(command_mutex_);
  const auto comm = cybergear::unpack_ext_id(ext_id).comm_type;
  // Type 4 is STOP; type 0 discovery and type 17 reads remain available.
  // The latch and TX share this gate so a delayed enable cannot follow a trip.
  if (motion_inhibited_.load() && comm != 4 && comm != 0 && comm != 17) {
    if (err) *err = "independent motor watchdog inhibited motion";
    return false;
  }
  if (!bus_) {
    if (err) *err = "can transport closed";
    return false;
  }
  return bus_->send(ext_id, data, err);
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
