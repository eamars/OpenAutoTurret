#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "can/cybergear_system.hpp"
#include "can/gm6020_protocol.hpp"
#include "can/gm6020_velocity.hpp"
#include "can/socketcan_bus.hpp"
#include "config/mixed_hardware_profile.hpp"
#include "control/can_motor_backend.hpp"

namespace ota {

// Production adapter for GM6020 continuous yaw on can0 and CyberGear pitch
// on can1. The GM6020 has session-relative feedback and voltage control; it
// does not expose the CyberGear UID/register/fault/disable interface.
class MixedCanMotorBackend final : public MotorBackend {
 public:
  MixedCanMotorBackend();
  ~MixedCanMotorBackend() override;
  MixedCanMotorBackend(const MixedCanMotorBackend&) = delete;
  MixedCanMotorBackend& operator=(const MixedCanMotorBackend&) = delete;

  // Opens already-UP interfaces without changing link state, validates their
  // topology and health, verifies the pitch UID, and establishes a stationary
  // session-relative yaw reference. It does not enable, home, or zero either
  // drive. The GM startup stop is a zero-voltage request, not disable proof.
  bool open(const config::mixed::Profile& profile, std::string& err);
  void close();
  bool yaw_reference_valid() const { return yaw_reference_valid_.load(); }
  std::vector<CanHealth> can_health_all() const override;
  CanHealth can_health() const override;
  bool buses_healthy() const;
  void start_watchdog();

  bool supports_continuous_yaw() const override { return true; }
  bool yaw_feedback_registerless() const override { return true; }
  bool requires_disable_confirmation(AxisId axis) const override {
    return axis != AxisId::Yaw;
  }
  void heartbeat() override;
  bool watchdog_fault() const override;
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
  void keepalive(AxisId axis) override;
  void set_current_limit(AxisId axis, double limit_cur_a) override;
  void set_speed_loop_gains(AxisId axis, double spd_kp, double spd_ki) override;

 private:
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
  void trip_yaw_locked();
  bool send_yaw_zero_locked();
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
  YawState yaw_state_{};
  double yaw_origin_rad_{0};
  double yaw_position_target_rad_{0};
  double yaw_speed_target_rad_s_{0};
  double yaw_shaped_speed_rad_s_{0};
  bool yaw_position_mode_{false};
  bool yaw_speed_mode_{false};
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
  std::jthread yaw_guard_;
};

}  // namespace ota
