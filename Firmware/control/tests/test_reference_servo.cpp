// ADR-003 3b: the ADR-002.2 yaw servo inside the production mixed backend. The servo steps on
// every GM6020 frame against the control loop's reference segment, through the production
// guards and output path; a legacy command releases it; a stale segment is held; the
// commissioning trips stop it. The plant is the identified model of the commissioned asset
// (axis::YawPlant: LuGre friction, the encoder's current crosstalk, actuation delay), fed by the
// current the backend actually put on the (test) wire.
#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <cmath>
#include <filesystem>
#include <memory>
#include <thread>
#include <vector>

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
  static double oscillation_rms(MixedCanMotorBackend& b) { std::lock_guard l(b.yaw_mutex_); return b.yaw_oscillation_.rms(); }
  static bool tripped(MixedCanMotorBackend& b) { return b.yaw_trip_.load(); }
};

namespace {
using Access = MixedBackendTestAccess;
constexpr double kStart = Access::kFirstCount * gm6020::UnwrappedEncoder::kRadiansPerCount;

// The commissioned asset's identified plant, advanced in real time (the backend reads the clock).
// `crosstalk_shift` (rad/A) moves the encoder's true current sensitivity off the identified table.
struct Plant {
  std::unique_ptr<axis::YawPlant> model;
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  explicit Plant(double crosstalk_shift = 0.0) {
    const auto asset = YAML::LoadFile((Access::firmware() / "config/servo/yaw_servo.json").string());
    auto p = axis::yaw_plant_from_yaml(asset["plant"]);
    for (auto& g : p.crosstalk_map) g += crosstalk_shift;
    model = std::make_unique<axis::YawPlant>(p, kStart);
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

// A Level-1-shaped move (ADR-003 tracking limits: jerk-limited to a_max) from rest to `speed`,
// a cruise, and a stop as hard as Level 1 brakes; sampled from a 1 ms table.
struct HardMove {
  std::vector<std::array<double, 3>> qva;
  HardMove(double speed, double cruise_s, double a_max = 24.56, double j_max = 2456.0) {
    double q = 0, v = 0, a = 0;
    auto reach = [&](double target) {
      for (int k = 0; k < 20000; ++k) {
        const double dv = target - v, s = dv > 0 ? 1.0 : -1.0, ramp = a * std::abs(a) / (2 * j_max);
        if (std::abs(dv) < 1e-9 && std::abs(a) < 1e-9) break;
        double j = (dv - ramp) * s > 0 ? (std::abs(a) < a_max ? s * j_max : 0.0) : (std::abs(a) > 1e-9 ? -std::copysign(j_max, a) : 0.0);
        a = std::clamp(a + j * 1e-3, -a_max, a_max);
        if (std::abs(dv) < std::abs(a) * 1e-3 && std::abs(a) < j_max * 1.5e-3) { a = 0; v = target; }
        v += a * 1e-3; q += v * 1e-3;
        qva.push_back({q, v, a});
      }
    };
    reach(speed);
    for (int k = 0; k < cruise_s * 1000; ++k) { q += v * 1e-3; qva.push_back({q, v, 0.0}); }
    reach(0.0);
  }
  double duration() const { return qva.size() * 1e-3; }
  double distance() const { return qva.back()[0]; }
  MotorBackend::ServoReference at(double t) const {
    const auto& s = qva[std::min<std::size_t>(qva.size() - 1, static_cast<std::size_t>(t * 1000))];
    MotorBackend::ServoReference r;
    r.t_ns = now_monotonic_ns(); r.q = s[0]; r.v = t < duration() ? s[1] : 0.0; r.a = t < duration() ? s[2] : 0.0;
    r.valid_s = 0.02;
    return r;
  }
};

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
  // Station, 2026-10-03 01:43:33: Park sent this zero while the servo held yaw. The release reset the
  // legacy loop at a clock reading later than the command's own, its first step saw time run
  // backwards (velocity_loop_invalid) and the station faulted. The hand-over must not trip.
  EXPECT_FALSE(Access::tripped(b));
  ASSERT_TRUE(b.command_reference(AxisId::Yaw, hold_at(0.02)));
  run(b, plant, 0.1);
  b.command(AxisId::Yaw, 0.02, 0.1);
  EXPECT_FALSE(Access::engaged(b));
  EXPECT_FALSE(Access::tripped(b)) << "a legacy position command hands over the same way";
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

TEST(ReferenceServo, PitchEngagesOnlyInsideItsEnvelope) {
  // The station on 2026-10-02: the first reference after homing carried no envelope (0, 0) with
  // the axis parked at -0.81 rad. That reference must be refused, not engaged into a guard trip.
  MotorBackend::ServoReference r;
  r.q = -0.8146;
  EXPECT_FALSE(pitch_reference_has_envelope(r));
  EXPECT_FALSE(pitch_servo_may_engage(r, -0.8146, 0.035));
  r.q_min = -1.4236; r.q_max = -0.2032;   // the soft limits homing established that day
  EXPECT_TRUE(pitch_servo_may_engage(r, -0.8146, 0.035));
  EXPECT_TRUE(pitch_servo_may_engage(r, -1.4236 - 0.015, 0.035)) << "inside the guard band's inner half";
  // Station 2026-10-03 22:26: engaged at -1.45819, one count inside the trip line at -1.4586, and
  // tripped on the next count. Engaging and tripping need room between them.
  EXPECT_FALSE(pitch_servo_may_engage(r, -1.45819, 0.035)) << "one count from the trip line";
  EXPECT_FALSE(pitch_servo_may_engage(r, -1.4236 - 0.03, 0.035)) << "outer half: the legacy path brings it in";
  EXPECT_FALSE(pitch_servo_may_engage(r, -1.4236 - 0.04, 0.035)) << "beyond it: would only trip";
  EXPECT_FALSE(pitch_servo_may_engage(r, -0.2032 + 0.04, 0.035));
  r.q_max = std::nan("");
  EXPECT_FALSE(pitch_reference_has_envelope(r));
}

// Station 2026-10-02 21:15: after hard stops at absolute 100 and 326 deg the yaw buzzed at standstill
// (34 Hz, +-1.2 A) for over 10 s: its encoder's current sensitivity had drifted about -3 mrad/A off
// the morning's table, and the commissioned gains (wn 44.7) left -2 mrad/A of margin. The design now
// survives a crosstalk error as large as the table itself (design.py). And the oscillation monitor
// counts only the current the servo did not plan (output minus feedforward): a hard stop's own
// deceleration current is not an oscillation.
TEST(ReferenceServo, AHardStopWithDriftedCrosstalkSettlesQuietly) {
  MixedCanMotorBackend b;
  Plant plant(-0.004);
  Access::prepare(b, [&](const can::RawFrame& f) { plant.command(amps_of(f)); return true; });
  std::string err;
  ASSERT_TRUE(Access::load(b, err)) << err;
  // From 44 deg absolute at 80 deg/s to a stop near 88 deg (a simulated onset angle of the old gains).
  const HardMove move(80 * M_PI / 180, 0.4);
  ASSERT_GT(move.distance(), 0.6);
  ASSERT_TRUE(b.command_reference(AxisId::Yaw, move.at(0)));
  double worst_rms = 0, hold_lo = 1e9, hold_hi = -1e9;
  run(b, plant, move.duration() + 1.5, [&](double t) {
    ASSERT_TRUE(b.command_reference(AxisId::Yaw, move.at(t)));
    worst_rms = std::max(worst_rms, Access::oscillation_rms(b));
    if (t > move.duration() + 0.5) { hold_lo = std::min(hold_lo, plant.q()); hold_hi = std::max(hold_hi, plant.q()); }
  });
  EXPECT_FALSE(Access::tripped(b));
  EXPECT_EQ(b.servo_hold_reason(), nullptr);
  EXPECT_LT(worst_rms, 0.5 * 0.3) << "the oscillation monitor must not see a planned stop, nor a buzz";
  EXPECT_NEAR(plant.q(), move.distance(), 0.006) << "settled within the hold band";
  EXPECT_LT(hold_hi - hold_lo, 0.004) << "still at rest";
}

// The camera measurement pairs each frame with the yaw at its optical time (tracking history). The
// raw GM6020 reading carries the current crosstalk -- up to ~0.36 deg/A -- which the history did not
// remove (station, 2026-10-05). q_true_rad removes it with the IDENTIFIED table: the servo's own table
// carries a -3 mrad/A stabilising bias, and with it this test measured the angle WORSE than raw
// (1.74 vs 1.21 mrad RMS against the plant, whose encoder follows the identified table).
TEST(ReferenceServo, SnapshotCarriesTheCrosstalkCorrectedAngle) {
  MixedCanMotorBackend b;
  Plant plant;
  Access::prepare(b, [&](const can::RawFrame& f) { plant.command(amps_of(f)); return true; });
  std::string err;
  ASSERT_TRUE(Access::load(b, err)) << err;
  EXPECT_FALSE(std::isfinite(b.snapshot(AxisId::Yaw, now_monotonic_ns()).q_true_rad)) << "not driving: no correction";
  const HardMove move(80 * M_PI / 180, 0.4);
  ASSERT_TRUE(b.command_reference(AxisId::Yaw, move.at(0)));
  double raw2 = 0, true2 = 0;
  int n = 0, missing = 0;
  run(b, plant, move.duration(), [&](double t) {
    ASSERT_TRUE(b.command_reference(AxisId::Yaw, move.at(t)));
    const auto s = b.snapshot(AxisId::Yaw, now_monotonic_ns());
    if (!s.has_feedback || t < 0.02) return;
    if (!std::isfinite(s.q_true_rad)) { ++missing; return; }
    // plant.q() has not advanced since the frame this snapshot holds (run() ticks before it advances).
    raw2 += std::pow(s.q_rad - plant.q(), 2);
    true2 += std::pow(s.q_true_rad - plant.q(), 2);
    ++n;
  });
  ASSERT_GT(n, 100);
  EXPECT_LT(missing, n / 50) << "the corrected angle is there whenever the servo drives";
  const double raw_rms = std::sqrt(raw2 / n), true_rms = std::sqrt(true2 / n);
  EXPECT_GT(raw_rms, 1e-3) << "the move must load the encoder with crosstalk to test anything";
  EXPECT_LT(true_rms, 0.5 * raw_rms) << "raw " << raw_rms << " rad, corrected " << true_rms << " rad";
}

// Owner ruling 2026-10-02 (STATION_OPERATIONS.md "Fault, hold, degrade"): a following error is
// usually an obstruction, not a hazard. The servo lets go and asks for a HOLD; it does not trip, and
// it does not re-engage onto the far reference.
TEST(ReferenceServo, FollowingErrorLetsGoAndHoldsWithoutTripping) {
  MixedCanMotorBackend b;
  Plant plant;
  Access::prepare(b, [&](const can::RawFrame& f) { plant.command(amps_of(f)); return true; });
  std::string err;
  ASSERT_TRUE(Access::load(b, err)) << err;
  ASSERT_TRUE(b.command_reference(AxisId::Yaw, hold_at(1.0)));  // 57 deg away: beyond the 15 deg limit
  run(b, plant, 0.05);
  EXPECT_FALSE(Access::tripped(b));
  EXPECT_FALSE(Access::engaged(b));
  ASSERT_NE(b.servo_hold_reason(), nullptr);
  EXPECT_STREQ(b.servo_hold_reason(), "yaw servo following error");
  EXPECT_FALSE(b.command_reference(AxisId::Yaw, hold_at(1.0))) << "re-engaged onto the far reference";
}

}  // namespace ota
