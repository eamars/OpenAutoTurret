#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "can/cybergear_system.hpp"
#include "can/gm6020_protocol.hpp"
#include "can/gm6020_velocity.hpp"
#include "can/gm6020_rx_velocity.hpp"
#include "can/socketcan_bus.hpp"
#include "config/mixed_hardware_profile.hpp"
#include "control/can_motor_backend.hpp"

namespace ota {

// Production adapter for GM6020 continuous yaw on can0 and CyberGear pitch
// on can1. The GM6020 has session-relative feedback and no CyberGear-style
// UID/register/fault/disable interface; what we command it with -- torque
// current on 0x1FE, or voltage on 0x1FF -- is one configuration decision
// (`axes.yaw.control_mode`), and every frame this file sends is chosen by it.
//
// The one yaw output decision that must be readable from the outside: given this profile, what
// does "make it stop" put on the wire? One function, so a startup zero, a hold, a fault zero and
// a shutdown zero cannot drift apart into four subtly different claims. Free (not a member)
// because a guarantee that needs an open CAN socket to be tested is a claim, not a guarantee.
inline can::RawFrame yaw_zero_frame(const config::mixed::Axis& yaw) {
  const auto id = yaw.motor_id ? yaw.motor_id : uint8_t{1};
  if (yaw.control_mode == config::mixed::ControlMode::Current)
    return gm6020::current_zero_frame(id);  // 0x1FE, payload all zero
  return gm6020::voltage_frame(id, 0);      // 0x1FF, this motor's slot zero
}

// The one yaw speed ceiling, applied to the ask: `commanded = min(ceiling, requested)`.
// It used to be a 15 deg/s clamp here AND a 25 deg/s trip on the MEASURED reading, so a
// heavy axis that momentarily overshot cut power to the payload -- and an unpowered
// unbalanced payload drops onto a hard stop, which costs more than never cutting it
// (owner ruling, 2026-09-28). One number, one job. Exposed as a free function so the
// behaviour is testable without a CAN bus: a ceiling that only exists inside a private
// member is a claim, not a guarantee.
// Manual jog felt "明显偏慢" beside pitch for the same reason yaw felt slow before its
// velocity ceiling was fixed: this ramp limit was picked small in the codex era and never
// revisited, while the pitch drive runs its own profile at 60 deg/s^2. The owner's ruling of
// 2026-09-28 is that the axes accelerate together in manual, so this is the declared
// axes.yaw.max_acceleration_deg_s2 and test_mixed_station_config fails if the two drift.
inline constexpr double kYawMaxAccelerationRadS2 = 60.0 * 3.14159265358979323846 / 180.0;

inline constexpr double kYawSpeedCeilingDegS = 30.0;  // matched to pitch, see the ruling above
inline constexpr double kYawSpeedCeilingRadS =
    kYawSpeedCeilingDegS * 3.14159265358979323846 / 180.0;
// `no_progress` means "a demand is in front of me and the axis will not move". It says
// nothing about a caller that stopped asking -- and a demand left sitting in a field from
// the last accepted cycle looks exactly like one. So freshness is part of the condition,
// not an afterthought: 2026-09-28 read five trips before the difference between "commanded
// and stuck" and "nobody is commanding any more" was written down anywhere.
inline constexpr int64_t kNoCommandLimitNs = 50'000'000;  // ten cycles of a 200 Hz loop
// What the guard may DO, in the order the owner set on 2026-09-28: keep running beats
// holding, holding beats faulting, and Fault is reserved for three things -- the motor is
// not under our control, the motor reports it is cooking, or something is as dangerous as
// those two. The payload is an unbalanced load: faulting drops it onto a hard stop, and
// that collision costs more than any of the conditions below. Everything short of those
// three is a thing to keep driving through and say out loud.
enum class GuardResponse { Run, Hold, Fault };
struct MotionEpisodeCounter {
  int count = 0;
  bool active = false;
  void observe(bool limited) {
    if (limited && !active) ++count;
    active = limited;
  }
};
// Performance episodes are observations, never independent zero-current writers.
// Zero current is neither dynamic braking nor position hold.
// What is worth SAYING while we keep driving. A refused command only matters if somebody
// actually wanted to move: `command_not_sent` with a zero demand is the loop saying "hold",
// which is the normal state of a turret with nothing to do -- the first version of this
// line logged one a second while the station was simply parked, and called it a limp.
inline bool yaw_guard_doubt(const MotorBackend::TripInputs& in, bool wants_motion) {
  if (in.can_counters_bad || in.bus_unhealthy || in.speed_not_finite) return true;
  if (in.no_progress) return true;
  return in.command_not_sent && wants_motion;
}

inline GuardResponse yaw_guard_response(const MotorBackend::TripInputs& in, int /*episodes*/) {
  if (in.feedback_unsafe || in.can_down || in.can_state_wrong || in.heartbeat_stale)
    return GuardResponse::Fault;  // cannot see it, cannot reach it, or it stopped answering
  if (in.temp_raw_over) return GuardResponse::Fault;  // the motor's own report: heat
  return GuardResponse::Run;  // CAN hiccup, our own NaN, a stale demand: drive on, say so
}

inline bool yaw_command_is_stale(int64_t now_ns, int64_t last_command_ns) {
  return last_command_ns == 0 || now_ns - last_command_ns > kNoCommandLimitNs;
}

inline double apply_yaw_speed_ceiling(double requested_rad_s) {
  return requested_rad_s > kYawSpeedCeilingRadS ? kYawSpeedCeilingRadS
         : requested_rad_s < -kYawSpeedCeilingRadS ? -kYawSpeedCeilingRadS
                                                   : requested_rad_s;
}

class MixedCanMotorBackend final : public MotorBackend {
 public:
  MixedCanMotorBackend();
  ~MixedCanMotorBackend() override;

  // Drive authority for the yaw velocity loop WHILE IN VOLTAGE MODE, in raw GM6020 voltage counts
  // (the controller accepts up to 25000). Public, and written exactly once, by whoever builds this
  // backend: the backend has no config access of its own, which is why the number used to be a
  // constant here -- and why the axis stalled at 9000 with the output pinned and no operator way to
  // raise it. The default keeps the behaviour that shipped on 2026-09-28.
  // Current mode ignores it entirely: there the envelope is axes.yaw.host_current_limit_a, in
  // amperes. A counts ceiling says nothing about amperes, so it is not applied "converted".
  void set_yaw_voltage_ceiling(double counts) { yaw_voltage_ceiling_ = counts; }
  double yaw_voltage_ceiling_ = 15000.0;
  MixedCanMotorBackend(const MixedCanMotorBackend&) = delete;
  MixedCanMotorBackend& operator=(const MixedCanMotorBackend&) = delete;

  // Opens already-UP interfaces without changing link state, validates their
  // topology and health, verifies the pitch UID, and establishes a stationary
  // session-relative yaw reference. It does not enable, home, or zero either
  // drive. The GM startup stop is a zero-output request (zero current in current
  // mode), not disable proof: this drive reports no enable bit to prove otherwise.
  bool open(const config::mixed::Profile& profile, std::string& err);
  void close();
  bool yaw_reference_valid() const { return yaw_reference_valid_.load(); }
  std::vector<CanHealth> can_health_all() const override;
  CanHealth can_health() const override;
  bool buses_healthy() const;
  OutputEvidence output_evidence(AxisId axis) const override;
  void start_watchdog();

  bool supports_continuous_yaw() const override { return true; }
  bool yaw_feedback_registerless() const override { return true; }
  bool requires_disable_confirmation(AxisId axis) const override {
    return axis != AxisId::Yaw;
  }
  void heartbeat() override;
  bool watchdog_fault() const override;
  bool watchdog_fault_axis(AxisId axis) const override {
    return axis == AxisId::Yaw ? yaw_trip_.load() :
        (pitch_opened_.load() && pitch_backend_.watchdog_fault());
  }
  // The guard fills this under yaw_trip_detail_mutex_ and then publishes yaw_trip_.
  // A reader must hold that mutex too: a flag check makes the value visible, not a
  // struct copy atomic. Nesting order is always yaw_mutex_ → yaw_trip_detail_mutex_.
  TripDetail watchdog_trip_detail() const override;
  bool recovery_before_homing() const override { return false; }
  bool discover(AxisId axis, uint64_t& unique_id, std::string& err) override;
  bool read_register(AxisId axis, cybergear::Reg reg, double& value,
                     int timeout_ms, std::string& err) override;
  bool enter_position_mode(AxisId axis, double limit_spd_rad_s,
                           std::string& err) override;
  bool enter_speed_mode(AxisId axis, double limit_cur_a,
                        std::string& err) override;
  Transition transition_mode(AxisId axis, bool position, double limit,
                             TimeNs now_ns, std::string& err,
                             double speed_ki = -1, double speed_kp = 1,
                             bool check_displacement = true) override;
  void deenergize(AxisId axis) override;
  AxisSnapshot snapshot(AxisId axis, TimeNs now_ns) override;
  void command(AxisId axis, double q_ref_rad, double limit_spd_rad_s) override;
  void command_velocity(AxisId axis, double velocity_rad_s) override;
  void set_motion_intent(AxisId axis, bool moving) override;
  void keepalive(AxisId axis) override;
  void set_current_limit(AxisId axis, double limit_cur_a) override;
  void set_speed_loop_gains(AxisId axis, double spd_kp, double spd_ki) override;

 private:
  friend struct MixedBackendTestAccess;
  // Narrow transport seam for exercising the real command/guard mutex and
  // inhibition path without opening a second physical CAN owner.
  std::function<bool(const can::RawFrame&)> yaw_test_send_;
  std::function<CanHealth()> yaw_test_health_;
  struct YawState {
    gm6020::Feedback feedback{};
    double position_rad{};  // offset to stationary open-time reference
    uint64_t count{};
    bool received{false};
    bool encoder_valid{false};
  };

  bool validate_profile(const config::mixed::Profile& profile,
                        std::string& err) const;
  bool validate_can0(std::string& err);
  bool validate_can1(std::string& err);
  bool establish_yaw_reference(std::string& err);
  void on_yaw_frame(const can::RawFrame& frame);
  void yaw_guard_loop(std::stop_token stop);
  void trip_yaw_locked(const char* condition = nullptr);
  bool send_yaw_zero_locked();
  bool send_yaw_output_locked(const can::RawFrame& frame, double output);
  bool yaw_bus_healthy() const;
  bool yaw_feedback_safe_locked(TimeNs now_ns) const;
  AxisSnapshot yaw_snapshot_locked(TimeNs now_ns) const;
  void command_yaw_velocity_locked(double velocity_rad_s, TimeNs now_ns);

  config::mixed::Profile profile_{};
  can::SocketCanBus yaw_bus_;
  can::CyberGearSystem pitch_system_;
  CanMotorBackend pitch_backend_;
  mutable std::mutex yaw_mutex_;
  gm6020::UnwrappedEncoder yaw_encoder_;
  gm6020::VelocityLoop yaw_velocity_loop_;
  gm6020::RxVelocity yaw_rx_velocity_;
  YawState yaw_state_{};
  double yaw_origin_rad_{0};
  double yaw_position_target_rad_{0};
  double yaw_speed_target_rad_s_{0};
  double yaw_shaped_speed_rad_s_{0};
  bool yaw_position_mode_{false};
  bool yaw_speed_mode_{false};
  bool yaw_moving_intent_{false};
  std::atomic<bool> opened_{false};
  std::atomic<bool> pitch_opened_{false};
  std::atomic<bool> pitch_enabled_owned_{false};
  std::atomic<bool> pitch_transition_active_{false};
  std::atomic<TimeNs> pitch_stop_ping_ns_{0};
  std::atomic<bool> yaw_opened_{false};
  std::atomic<bool> bus_health_ok_{false};
  std::atomic<bool> yaw_reference_valid_{false};
  std::atomic<bool> yaw_motion_allowed_{false};
  std::atomic<bool> yaw_trip_{false};
  TripDetail yaw_trip_detail_{};  // guarded by yaw_trip_detail_mutex_; validity rides on yaw_trip_
  mutable std::mutex yaw_trip_detail_mutex_;
  std::atomic<bool> heartbeat_seen_{false};
  std::atomic<TimeNs> heartbeat_ns_{0};
  TimeNs yaw_velocity_loop_previous_command_ns_{0};
  std::atomic<double> yaw_requested_velocity_rad_s_{0};
  // True while the newest command was refused (non-finite, motion not permitted, stale
  // heartbeat): a zero went out instead, so `requested` describes the last accepted
  // cycle and must not be quoted as this one's demand.
  std::atomic<bool> yaw_command_not_sent_{false};
  // The guard's non-latching observations: something is worth saying, nothing is worth
  // dropping the payload for. Surfaced rather than logged-only so the next person does not
  // have to ssh in to learn the axis has been limping for an hour.
  std::atomic<bool> yaw_degraded_{false};
  std::atomic<int> yaw_guard_events_{0};
  // What we last actually put on the wire, in the unit of the configured mode: raw voltage counts
  // in voltage mode, amperes in current mode (see yaw_output_is_amperes). Without this a paralysis
  // log can only say "asked for 10 deg/s, got none" and cannot distinguish pushing with zero
  // output (our bug) from pushing hard against something solid (a fact about the world). Named as
  // the missing evidence in the 2026-09-28 case file and still missing an hour later.
  std::atomic<double> yaw_last_output_{0};
  std::atomic<double> yaw_last_shaped_rad_s_{0};
  MotionEpisodeCounter yaw_stall_episodes_;
  TimeNs yaw_tx_failure_since_ns_ = 0;
  TimeNs yaw_last_successful_tx_ns_ = 0;
  uint64_t yaw_tx_seq_ = 0;
  double yaw_requested_output_ = 0;
  int yaw_output_reason_ = 0;
  int64_t last_degrade_log_ns_ = 0;
  // Public: an axis that has been limping is worth a strip indicator, and a counter is the
  // difference between "it happened once" and "it happens every sweep".
 public:
  bool yaw_degraded() const { return yaw_degraded_.load(); }
  double yaw_last_output() const { return yaw_last_output_.load(); }
  // Which unit yaw_last_output()/diag_output() are in. A bare number in a log line is how `vout=0`
  // got read as "we are not pushing" on 2026-09-28 while the mode had already changed under it.
  bool yaw_output_is_amperes() const {
    return profile_.yaw.control_mode == config::mixed::ControlMode::Current;
  }
  // The ramp's own value: what the velocity loop was told to track last cycle, as opposed
  // to what the caller asked for (accepted, then shaped) and what the axis measured.
  double diag_commanded_speed_rad_s(AxisId axis) const override {
    return axis == AxisId::Yaw ? yaw_last_shaped_rad_s_.load()
                              : std::numeric_limits<double>::quiet_NaN();
  }
  double diag_output(AxisId axis) const override {
    return axis == AxisId::Yaw ? yaw_last_output_.load()
                               : std::numeric_limits<double>::quiet_NaN();
  }
  bool diag_degraded() const override { return yaw_degraded_.load(); }
  int diag_guard_events() const override { return yaw_guard_events_.load(); }
  int yaw_guard_events() const { return yaw_guard_events_.load(); }
 private:
  // When the loop last put a demand in front of this backend (guarded by yaw_mutex_).
  // Zero until the first command ever, which is itself a stale state.
  int64_t yaw_last_command_ns_ = 0;
  std::jthread yaw_guard_;
};

}  // namespace ota
