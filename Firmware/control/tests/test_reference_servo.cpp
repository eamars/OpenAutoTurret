// ADR-003 3b: the ADR-002.2 yaw servo inside the production mixed backend. The servo steps on
// every GM6020 frame against the control loop's reference segment, through the production
// guards and output path; a legacy command releases it; a stale segment is held; the
// commissioning trips stop it. The plant is the identified model of the commissioned asset
// (axis::YawPlant: LuGre friction, the encoder's current crosstalk, actuation delay), fed by the
// current the backend actually put on the (test) wire.
#include <algorithm>
#include <chrono>
#include <functional>
#include <cmath>
#include <filesystem>
#include <memory>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "ota_test_paths.hpp"

#include "can/gm6020_protocol.hpp"
#include "common/time.hpp"
#include "control/mixed_can_motor_backend.hpp"
#include "plant.hpp"

#include <gtest/gtest.h>

namespace ota {

struct MixedBackendTestAccess {
  static std::filesystem::path firmware() {
    return ota_test_firmware_dir(std::filesystem::path(__FILE__).parent_path().parent_path().parent_path());
  }
  static constexpr uint16_t kFirstCount = 1000;
  static void prepare(MixedCanMotorBackend& b, std::function<bool(const can::RawFrame&)> sender) {
    const auto now = now_monotonic_ns();
    std::lock_guard lock(b.yaw_mutex_);
    b.profile_.yaw.motor_id = 1;
    b.profile_.yaw.control_mode = config::mixed::ControlMode::Current;
    b.profile_.yaw.host_current_limit_a = 0.8;
    b.profile_.yaw.current_kp_a_per_rad_s = 1.0;
    b.profile_.yaw.current_ki_a_per_rad_s = 0.6;
    config::mixed::Servo servo;
    servo.yaw_asset = (firmware() / "config/servo/yaw_servo.json").string();
    servo.pitch_asset = (firmware() / "config/servo/pitch_servo.json").string();
    servo.yaw_current_limit_a = 3.0;
    servo.yaw_rms_limit_a = 1.62;
    servo.yaw_temperature_limit_raw = 55;
    servo.oscillation_limit_a = 0.3;
    servo.speed_limit_rad_s = 10.47;
    servo.pitch_guard_rad = 0.035;
    servo.pitch_stop_acceleration_rad_s2 = 1.047;
    b.profile_.servo = servo;
    b.yaw_state_.feedback = {kFirstCount, 0, 0, 30, now};
    b.yaw_state_.position_rad = 0.0;
    b.yaw_state_.received = true;
    b.yaw_state_.encoder_valid = true;
    b.yaw_encoder_.update(kFirstCount, now);
    b.yaw_reference_valid_.store(true);
    b.yaw_motion_allowed_.store(true);
    b.yaw_trip_.store(false);
    b.heartbeat_seen_.store(true);
    b.heartbeat_ns_.store(now);
    b.yaw_test_send_ = std::move(sender);
    b.yaw_test_health_ = [] { CanHealth h; h.available = true; h.up = true; h.state = 0; return h; };
    b.bus_health_ok_.store(true);
  }
  static bool load(MixedCanMotorBackend& b, std::string& err) { return b.load_servos(err); }
  // One GM6020 frame carrying an absolute angle reading.
  static void frame(MixedCanMotorBackend& b, double reading_abs, double v) {
    b.heartbeat_ns_.store(now_monotonic_ns());
    const long counts = std::lround(reading_abs / gm6020::UnwrappedEncoder::kRadiansPerCount);
    const auto angle = static_cast<uint16_t>(((counts % 8192) + 8192) % 8192);
    can::RawFrame f;
    f.id = 0x205; f.extended = false; f.dlc = 8;
    f.data[0] = angle >> 8; f.data[1] = angle & 0xff;
    const auto rpm = static_cast<int16_t>(std::lround(v * 60 / (2 * M_PI)));
    f.data[2] = static_cast<uint16_t>(rpm) >> 8; f.data[3] = static_cast<uint16_t>(rpm) & 0xff;
    f.data[6] = 30;
    f.rx_ns = now_monotonic_ns();
    b.on_yaw_frame(f);
  }
  static bool engaged(MixedCanMotorBackend& b) { std::lock_guard l(b.yaw_mutex_); return b.yaw_servo_active_; }
  static bool tripped(MixedCanMotorBackend& b) { return b.yaw_trip_.load(); }
};

namespace {
using Access = MixedBackendTestAccess;
constexpr double kStart = Access::kFirstCount * gm6020::UnwrappedEncoder::kRadiansPerCount;

// The commissioned asset's identified plant, advanced in real time (the backend reads the clock).
struct Plant {
  std::unique_ptr<axis::YawPlant> model;
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  Plant() {
    const auto asset = YAML::LoadFile((Access::firmware() / "config/servo/yaw_servo.json").string());
    model = std::make_unique<axis::YawPlant>(axis::yaw_plant_from_yaml(asset["plant"]), kStart);
  }
  double now() const { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); }
  void command(double amps) { model->command(now(), amps); }
  double q() const { return model->position() - kStart; }  // session-relative, as the backend reports it
};

double amps_of(const can::RawFrame& f) {
  return static_cast<int16_t>((f.data[0] << 8) | f.data[1]) * gm6020::kAmpsPerRaw;
}

MotorBackend::ServoReference hold_at(double q, double valid_s = 10.0) {
  MotorBackend::ServoReference r;
  r.t_ns = now_monotonic_ns(); r.q = q; r.valid_s = valid_s;
  return r;
}

// A smooth move as the control loop publishes it: one segment per 5 ms tick of a
// minimum-jerk profile from 0 to `distance` over `duration` (the servo never sees a step).
MotorBackend::ServoReference smooth(double t, double distance, double duration) {
  const double s = std::clamp(t / duration, 0.0, 1.0);
  MotorBackend::ServoReference r;
  r.t_ns = now_monotonic_ns();
  r.q = distance * (10 * std::pow(s, 3) - 15 * std::pow(s, 4) + 6 * std::pow(s, 5));
  r.v = t < duration ? distance / duration * (30 * s * s - 60 * std::pow(s, 3) + 30 * std::pow(s, 4)) : 0.0;
  r.a = t < duration ? distance / (duration * duration) * (60 * s - 180 * s * s + 120 * std::pow(s, 3)) : 0.0;
  r.valid_s = 0.02;
  return r;
}

void run(MixedCanMotorBackend& b, Plant& plant, double seconds,
         const std::function<void(double)>& tick = nullptr) {
  const auto start = std::chrono::steady_clock::now();
  const auto end = start + std::chrono::duration<double>(seconds);
  double next_tick = 0;
  while (std::chrono::steady_clock::now() < end) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (tick && elapsed >= next_tick) { tick(elapsed); next_tick += 0.005; }
    const double t = plant.now();
    plant.model->advance(t);
    const double sampled = plant.model->position();
    Access::frame(b, plant.model->reading(sampled, t), plant.model->velocity());
  }
}
}  // namespace

TEST(ReferenceServo, FollowsTheReferenceThroughTheProductionOutputPath) {
  MixedCanMotorBackend b;
  Plant plant;
  int frames = 0;
  Access::prepare(b, [&](const can::RawFrame& f) { ++frames; EXPECT_EQ(f.id, 0x1feu); plant.command(amps_of(f)); return true; });
  std::string err;
  ASSERT_TRUE(Access::load(b, err)) << err;
  ASSERT_TRUE(b.servo_available(AxisId::Yaw));
  ASSERT_TRUE(b.command_reference(AxisId::Yaw, smooth(0, 0.05, 0.6)));
  EXPECT_TRUE(Access::engaged(b));
  run(b, plant, 1.5, [&](double t) { ASSERT_TRUE(b.command_reference(AxisId::Yaw, smooth(t, 0.05, 0.6))); });
  EXPECT_FALSE(Access::tripped(b));
  EXPECT_GT(frames, 500);  // one current frame per encoder frame
  // The asset's hold band is 0.004 rad and its stall recovery acts beyond 0.0025 rad: at rest
  // the axis sits within the band of the reference.
  EXPECT_NEAR(plant.q(), 0.05, 0.006);
}

TEST(ReferenceServo, LegacyCommandReleasesIt) {
  MixedCanMotorBackend b;
  Plant plant;
  Access::prepare(b, [&](const can::RawFrame& f) { plant.command(amps_of(f)); return true; });
  std::string err;
  ASSERT_TRUE(Access::load(b, err)) << err;
  ASSERT_TRUE(b.command_reference(AxisId::Yaw, hold_at(0.02)));
  run(b, plant, 0.1);
  b.command_velocity(AxisId::Yaw, 0.0);
  EXPECT_FALSE(Access::engaged(b)) << "a legacy speed command hands the axis back to the legacy loop";
}

TEST(ReferenceServo, StaleSegmentIsHeldNotExtrapolated) {
  MixedCanMotorBackend b;
  Plant plant;
  Access::prepare(b, [&](const can::RawFrame& f) { plant.command(amps_of(f)); return true; });
  std::string err;
  ASSERT_TRUE(Access::load(b, err)) << err;
  auto r = hold_at(0.0, 0.02);
  r.v = 0.2;  // a segment moving at 0.2 rad/s that is never renewed
  ASSERT_TRUE(b.command_reference(AxisId::Yaw, r));
  run(b, plant, 0.8);
  EXPECT_FALSE(Access::tripped(b));
  // Held at its 20 ms end (0.004 rad), not followed for 0.8 s (0.16 rad).
  EXPECT_LT(std::abs(plant.q()), 0.015);
}

TEST(ReferenceServo, FollowingErrorTrips) {
  MixedCanMotorBackend b;
  Plant plant;
  Access::prepare(b, [&](const can::RawFrame& f) { plant.command(amps_of(f)); return true; });
  std::string err;
  ASSERT_TRUE(Access::load(b, err)) << err;
  ASSERT_TRUE(b.command_reference(AxisId::Yaw, hold_at(1.0)));  // 57 deg away: beyond the 15 deg limit
  run(b, plant, 0.05);
  EXPECT_TRUE(Access::tripped(b));
  EXPECT_FALSE(Access::engaged(b));
}

}  // namespace ota
