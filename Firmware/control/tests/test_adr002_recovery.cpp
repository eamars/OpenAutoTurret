#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "control/control_loop.hpp"
#include "control/mixed_can_motor_backend.hpp"
#include "sim/sim_motor_backend.hpp"

#include <gtest/gtest.h>

namespace ota {

// This peer only prepares state and invokes private production operations. All
// CAN sends still travel through MixedCanMotorBackend::send_yaw_output_locked.
struct MixedBackendTestAccess {
  static void prepare_current_yaw(MixedCanMotorBackend& backend,
                                  std::function<bool(const can::RawFrame&)> sender,
                                  CanHealth health = healthy_can(0, 0)) {
    const auto now = now_monotonic_ns();
    std::lock_guard lock(backend.yaw_mutex_);
    backend.profile_.yaw.motor_id = 1;
    backend.profile_.yaw.control_mode = config::mixed::ControlMode::Current;
    backend.profile_.yaw.host_current_limit_a = 0.8;
    backend.profile_.yaw.current_kp_a_per_rad_s = 1.0;
    backend.profile_.yaw.current_ki_a_per_rad_s = 0.6;
    backend.yaw_state_.feedback = {0, 0, 0, 30, now};
    backend.yaw_state_.position_rad = 0.0;
    backend.yaw_state_.received = true;
    backend.yaw_state_.encoder_valid = true;
    backend.yaw_reference_valid_.store(true);
    backend.yaw_motion_allowed_.store(true);
    backend.yaw_trip_.store(false);
    backend.heartbeat_seen_.store(true);
    backend.heartbeat_ns_.store(now);
    backend.yaw_velocity_loop_.reset(0.0, now - 30'000'000);
    backend.yaw_velocity_loop_previous_command_ns_ = now - 30'000'000;
    backend.yaw_test_send_ = std::move(sender);
    backend.yaw_test_health_ = [health] { return health; };
  }

  static bool trip_yaw_after_proving_mutex_is_busy(
      MixedCanMotorBackend& backend,
      const std::function<void(bool)>& report_contention) {
    std::unique_lock lock(backend.yaw_mutex_, std::try_to_lock);
    const bool was_busy = !lock.owns_lock();
    report_contention(was_busy);
    if (!lock.owns_lock()) lock.lock();
    backend.trip_yaw_locked();
    return was_busy;
  }

  static void refresh_live_inputs(MixedCanMotorBackend& backend) {
    const auto now = now_monotonic_ns();
    std::lock_guard lock(backend.yaw_mutex_);
    backend.yaw_state_.feedback.rx_ns = now;
    backend.heartbeat_ns_.store(now);
    backend.yaw_velocity_loop_.reset(backend.yaw_state_.position_rad,
                                     now - 30'000'000);
    backend.yaw_velocity_loop_previous_command_ns_ = now - 30'000'000;
  }

  static void stale_feedback(MixedCanMotorBackend& backend) {
    const auto now = now_monotonic_ns();
    std::lock_guard lock(backend.yaw_mutex_);
    backend.yaw_state_.feedback.rx_ns = now - 101'000'000;
    backend.heartbeat_ns_.store(now);
  }

  static CanHealth healthy_can(uint64_t rx_errors = 0, uint64_t tx_failures = 0) {
    CanHealth health;
    health.available = true;
    health.kind = "socketcan";
    health.device = "can0";
    health.up = true;
    health.state = static_cast<int>(can::CanIfState::ErrorActive);
    health.rx_error_frames = rx_errors;
    health.tx_failed = tx_failures;
    return health;
  }
};

TEST(Adr002YawOutput, FreshStopClockDoesNotRejectFramesNewerThanTheLastTick) {
  MixedCanMotorBackend backend;
  MixedBackendTestAccess::prepare_current_yaw(backend, [](const auto&) { return true; });
  const auto fresh_clock = now_monotonic_ns();
  const auto old = backend.snapshot(AxisId::Yaw,fresh_clock-10'000'000);
  EXPECT_FALSE(old.has_feedback);
  EXPECT_FALSE(std::isfinite(old.q_rad));
  const auto current = backend.snapshot(AxisId::Yaw,fresh_clock);
  EXPECT_TRUE(current.has_feedback);
  EXPECT_TRUE(std::isfinite(current.q_rad));
}

TEST(Adr002YawOutput, SessionTrialRejectsOverCapAndStaleFeedback) {
  MixedCanMotorBackend backend;
  MixedBackendTestAccess::prepare_current_yaw(backend, [](const auto&) { return true; });
  MotorBackend::YawTrialSettings s;
  s.kp_a_per_rad_s=1; s.ki_a_per_rad=.6; s.rx_window_ms=20;
  s.friction={true,.05,.05,.05,.05,1,.0023,.0087,5,4};
  std::string error;
  ASSERT_TRUE(backend.apply_yaw_trial(s,error)) << error;
  s.friction.positive_breakaway_a=.81;
  EXPECT_FALSE(backend.apply_yaw_trial(s,error));
  s.friction.positive_breakaway_a=.05;
  s.ki_a_per_rad=NAN;
  EXPECT_FALSE(backend.apply_yaw_trial(s,error));
  s.ki_a_per_rad=.6;
  MixedBackendTestAccess::stale_feedback(backend);
  EXPECT_FALSE(backend.apply_yaw_trial(s,error));
}

namespace {

class WatchdogSimBackend final : public sim::SimMotorBackend {
 public:
  void trip_axis(AxisId axis) { axis_tripped_[static_cast<std::size_t>(axis)] = true; }
  void trip_global() { global_trip_ = true; }

  bool watchdog_fault() const override {
    return global_trip_ || axis_tripped_[0] || axis_tripped_[1];
  }
  bool watchdog_fault_axis(AxisId axis) const override {
    return global_trip_ || axis_tripped_[static_cast<std::size_t>(axis)];
  }
  bool fault_releases_axis(AxisId axis) const override { return !holds_[static_cast<std::size_t>(axis)]; }
  void holds_on_fault(AxisId axis) { holds_[static_cast<std::size_t>(axis)] = true; }

 private:
  bool global_trip_ = false;
  bool axis_tripped_[kAxisCount] = {};
  bool holds_[kAxisCount] = {};
};

std::unique_ptr<WatchdogSimBackend> powered_backend() {
  auto backend = std::make_unique<WatchdogSimBackend>();
  std::string error;
  EXPECT_TRUE(backend->enter_position_mode(AxisId::Pitch, 1.0, error)) << error;
  EXPECT_TRUE(backend->enter_position_mode(AxisId::Yaw, 1.0, error)) << error;
  EXPECT_TRUE(backend->in_position_mode(AxisId::Pitch));
  EXPECT_TRUE(backend->in_position_mode(AxisId::Yaw));
  return backend;
}

TEST(Adr002WatchdogRecovery, YawOnlyTripLeavesHealthyPitchEnergized) {
  auto backend = powered_backend();
  auto* sim = backend.get();
  sim->trip_axis(AxisId::Yaw);
  ControlLoop loop({}, std::move(backend));

  loop.step(5'000'000, 5'000'000);

  EXPECT_EQ(loop.phase(), Phase::Fault);
  EXPECT_NE(loop.fault_reason().find("independent motor watchdog"),
            std::string::npos);
  EXPECT_FALSE(sim->in_position_mode(AxisId::Yaw));
  EXPECT_TRUE(sim->in_position_mode(AxisId::Pitch));
}

// Owner ruling 2026-10-02: releasing an unbalanced axis lets it fall. A fault releases only an axis
// that cannot be held; the pitch drive holds on its own encoder unless it has itself faulted.
TEST(Adr002WatchdogRecovery, AFaultLeavesAnAxisThatCanStillBeHeldEnergized) {
  auto backend = powered_backend();
  auto* sim = backend.get();
  sim->holds_on_fault(AxisId::Pitch);
  sim->trip_global();
  ControlLoop loop({}, std::move(backend));

  loop.step(5'000'000, 5'000'000);

  EXPECT_EQ(loop.phase(), Phase::Fault);
  EXPECT_FALSE(sim->in_position_mode(AxisId::Yaw));
  EXPECT_TRUE(sim->in_position_mode(AxisId::Pitch)) << "pitch was released and would fall";
}

TEST(Adr002WatchdogRecovery, GlobalTripDeenergizesBothAxes) {
  auto backend = powered_backend();
  auto* sim = backend.get();
  sim->trip_global();
  ControlLoop loop({}, std::move(backend));

  loop.step(5'000'000, 5'000'000);

  EXPECT_EQ(loop.phase(), Phase::Fault);
  EXPECT_FALSE(sim->in_position_mode(AxisId::Yaw));
  EXPECT_FALSE(sim->in_position_mode(AxisId::Pitch));
}

bool frame_is_zero(const can::RawFrame& frame) {
  return std::all_of(std::begin(frame.data), std::end(frame.data),
                     [](uint8_t value) { return value == 0; });
}

TEST(Adr002YawOutput, EmergencyTripWaitsForInflightCommandThenInhibitsLaterCommands) {
  MixedCanMotorBackend backend;
  std::mutex mutex;
  std::condition_variable changed;
  bool first_command_entered = false;
  bool release_first_command = false;
  std::vector<can::RawFrame> frames;
  MixedBackendTestAccess::prepare_current_yaw(
      backend, [&](const can::RawFrame& frame) {
        std::unique_lock lock(mutex);
        frames.push_back(frame);
        if (!frame_is_zero(frame) && !first_command_entered) {
          first_command_entered = true;
          changed.notify_all();
          changed.wait(lock, [&] { return release_first_command; });
        }
        return true;
      });

  std::thread command([&] { backend.command_velocity(AxisId::Yaw, 0.2); });
  bool entered = false;
  {
    std::unique_lock lock(mutex);
    entered = changed.wait_for(lock, std::chrono::seconds(2),
                               [&] { return first_command_entered; });
  }
  if (!entered) {
    {
      std::lock_guard lock(mutex);
      release_first_command = true;
    }
    changed.notify_all();
    command.join();
    FAIL() << "no non-zero frame reached the injected production sender";
    return;
  }

  std::mutex trip_mutex;
  std::condition_variable trip_changed;
  bool trip_checked = false;
  bool trip_contended = false;
  std::thread trip([&] {
    const bool contended =
        MixedBackendTestAccess::trip_yaw_after_proving_mutex_is_busy(
            backend, [&](bool busy) {
              {
                std::lock_guard lock(trip_mutex);
                trip_contended = busy;
                trip_checked = true;
              }
              trip_changed.notify_one();
            });
    EXPECT_TRUE(contended);
  });
  bool trip_blocked_by_command = false;
  {
    std::unique_lock lock(trip_mutex);
    trip_blocked_by_command = trip_changed.wait_for(
        lock, std::chrono::seconds(2), [&] { return trip_checked; }) &&
        trip_contended;
  }
  {
    std::lock_guard lock(mutex);
    release_first_command = true;
  }
  changed.notify_all();
  command.join();
  trip.join();

  ASSERT_TRUE(trip_blocked_by_command)
      << "trip did not observe the in-flight command holding yaw_mutex_";
  backend.command_velocity(AxisId::Yaw, 0.2);
  ASSERT_EQ(frames.size(), 3u);
  EXPECT_FALSE(frame_is_zero(frames[0]));
  EXPECT_TRUE(frame_is_zero(frames[1]));
  EXPECT_TRUE(frame_is_zero(frames[2]));
  EXPECT_TRUE(backend.watchdog_fault());
}

TEST(Adr002YawOutput, HistoricalCanCountersDoNotBlockFreshFeedbackCommand) {
  MixedCanMotorBackend backend;
  std::vector<can::RawFrame> frames;
  const auto health = MixedBackendTestAccess::healthy_can(7, 4);
  MixedBackendTestAccess::prepare_current_yaw(
      backend, [&](const can::RawFrame& frame) {
        frames.push_back(frame);
        return true;
      }, health);

  backend.command_velocity(AxisId::Yaw, 0.2);

  ASSERT_EQ(frames.size(), 1u);
  EXPECT_FALSE(frame_is_zero(frames.front()));
  EXPECT_FALSE(backend.watchdog_fault());
}

TEST(Adr002YawOutput, OneTransmitFailureRecoversOnNextCommand) {
  MixedCanMotorBackend backend;
  int attempts = 0;
  std::vector<can::RawFrame> frames;
  MixedBackendTestAccess::prepare_current_yaw(
      backend, [&](const can::RawFrame& frame) {
        frames.push_back(frame);
        return ++attempts != 1;
      });

  backend.command_velocity(AxisId::Yaw, 0.2);
  EXPECT_FALSE(backend.watchdog_fault());
  backend.command_velocity(AxisId::Yaw, 0.2);

  EXPECT_EQ(attempts, 2);
  EXPECT_FALSE(backend.watchdog_fault());
  EXPECT_EQ(backend.output_evidence(AxisId::Yaw).tx_seq, 1u);
}

TEST(Adr002YawOutput, SustainedTransmitFailureTripsAfterTwentyMilliseconds) {
  MixedCanMotorBackend backend;
  std::vector<can::RawFrame> frames;
  MixedBackendTestAccess::prepare_current_yaw(
      backend, [&](const can::RawFrame& frame) {
        frames.push_back(frame);
        return false;
      });

  backend.command_velocity(AxisId::Yaw, 0.2);
  EXPECT_FALSE(backend.watchdog_fault());
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  MixedBackendTestAccess::refresh_live_inputs(backend);
  backend.command_velocity(AxisId::Yaw, 0.2);

  EXPECT_TRUE(backend.watchdog_fault());
  ASSERT_GE(frames.size(), 3u);  // first, second, and emergency zero attempts
  EXPECT_TRUE(frame_is_zero(frames.back()));
}

TEST(Adr002YawOutput, BusOffRejectsMotionAndTrips) {
  MixedCanMotorBackend backend;
  auto health = MixedBackendTestAccess::healthy_can();
  health.state = static_cast<int>(can::CanIfState::BusOff);
  std::vector<can::RawFrame> frames;
  MixedBackendTestAccess::prepare_current_yaw(
      backend, [&](const can::RawFrame& frame) {
        frames.push_back(frame);
        return true;
      }, health);

  backend.command_velocity(AxisId::Yaw, 0.2);

  EXPECT_TRUE(backend.watchdog_fault());
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_TRUE(frame_is_zero(frames.front()));
}

TEST(Adr002YawOutput, StaleFeedbackRejectsMotionAndTrips) {
  MixedCanMotorBackend backend;
  std::vector<can::RawFrame> frames;
  MixedBackendTestAccess::prepare_current_yaw(
      backend, [&](const can::RawFrame& frame) {
        frames.push_back(frame);
        return true;
      });
  MixedBackendTestAccess::stale_feedback(backend);

  backend.command_velocity(AxisId::Yaw, 0.2);

  EXPECT_TRUE(backend.watchdog_fault());
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_TRUE(frame_is_zero(frames.front()));
}

TEST(Adr002YawOutput, ZeroCurrentRequestDoesNotClaimDisableOrLoadSupport) {
  MixedCanMotorBackend backend;
  std::vector<can::RawFrame> frames;
  MixedBackendTestAccess::prepare_current_yaw(
      backend, [&](const can::RawFrame& frame) {
        frames.push_back(frame);
        return true;
      });

  backend.deenergize(AxisId::Yaw);
  const auto snapshot = backend.snapshot(AxisId::Yaw, now_monotonic_ns());

  ASSERT_EQ(frames.size(), 1u);
  EXPECT_TRUE(frame_is_zero(frames.front()));
  EXPECT_FALSE(snapshot.disabled_known);
  EXPECT_FALSE(snapshot.disabled);
  EXPECT_FALSE(snapshot.in_position_mode);
  EXPECT_FALSE(snapshot.in_speed_mode);
  // A zero-current frame is observable; physical holding/support is not.
}

}  // namespace
}  // namespace ota
