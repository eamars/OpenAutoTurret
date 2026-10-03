// The web's Park, Shutdown and Home (owner ruling 2026-10-03), end to end on the simulated plant.
//
// Park: yaw to 0, pitch onto its rest end stop, held there energised, and any mode leaves it.
// Shutdown: that park, then both motors off; only Home leaves it. Home: recover whatever latched,
// then home. The station menu offered four supervisory actions before this and none of them got
// the turret anywhere: Park faulted it in 40 ms, and a fault then refused both Recover and Home.
#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <memory>

#include "control/control_loop.hpp"
#include "sim/sim_motor_backend.hpp"

using namespace ota;

namespace {
constexpr int64_t kDtNs = 5'000'000;

HomingPlan two_axis_plan() {
  HomingPlanConfig hcfg;
  hcfg.homing.coarse_speed_rad_s = 20.0 * kDeg2Rad;
  hcfg.homing.fine_speed_rad_s = 2.0 * kDeg2Rad;
  hcfg.homing.settle_time_s = 0.3;
  hcfg.travel_bands[0] = TravelBand{0.0, 115.0};
  hcfg.travel_bands[1] = TravelBand{0.0, 115.0};
  std::vector<HomingAction> actions{{.type = HomingActionType::HomeFullRange, .axis = AxisId::Pitch},
                                    {.type = HomingActionType::HomeFullRange, .axis = AxisId::Yaw}};
  return HomingPlan(std::move(actions), hcfg);
}

ControlLoop::Config rest_cfg() {
  ControlLoop::Config cfg;
  cfg.control_hz = 200;
  cfg.hold_speed_rad_s = 30.0 * kDeg2Rad;
  cfg.emergency_speed_rad_s = 10.0 * kDeg2Rad;
  cfg.soft_margin_rad = 5.0 * kDeg2Rad;   // the station's pitch margin
  cfg.rest_park = true;
  cfg.rest_park_pitch_low = true;          // the station's camera-up stop is the raw minimum
  cfg.rest_park_touch_speed_rad_s = 3.0 * kDeg2Rad;
  return cfg;
}

struct Station {
  sim::SimMotorBackend* sim = nullptr;
  std::unique_ptr<ControlLoop> loop;
  int64_t t = kDtNs;
  bool saw_fault = false;

  explicit Station(std::unique_ptr<sim::SimMotorBackend> plant = nullptr,
                   ControlLoop::Config cfg = rest_cfg()) {
    if (!plant) {
      plant = std::make_unique<sim::SimMotorBackend>(0.005);
      plant->set_stops(AxisId::Pitch, -1.0, 1.0);
      plant->set_stops(AxisId::Yaw, -1.0, 1.0);
    }
    sim = plant.get();
    loop = std::make_unique<ControlLoop>(cfg, std::move(plant));
    loop->set_homing_factory([] { return two_axis_plan(); });
  }
  void step(int n = 1) {
    for (int i = 0; i < n; ++i) {
      loop->step(t, kDtNs);
      t += kDtNs;
      saw_fault |= loop->phase() == Phase::Fault;
    }
  }
  void run(const char* name, const char* arg = "") {
    loop->submit_command(name, arg);
    step(3);
  }
  bool until(const std::function<bool()>& done, int max_steps = 40'000) {
    for (int i = 0; i < max_steps; ++i) {
      if (done()) return true;
      step();
    }
    return done();
  }
  bool home() {
    std::string err;
    if (!loop->start_homing(two_axis_plan(), err)) return false;
    return until([&] { return loop->phase() == Phase::Fault || (loop->homed() && loop->at_ready()); }) &&
           loop->phase() == Phase::Hold;
  }
  std::string stage() const { return loop->rest_park_stage(); }
  ControlLoop::CommandAck ack() const { return loop->last_command_ack(); }
};

// The station as the boundary sees it (2026-10-03): speed-mode pitch, continuous yaw, and an encoder
// that reads whole counts and flickers between neighbours while the axis holds still. Every test
// that crosses the pitch envelope runs here, because the exact-position plant above passed the
// at-rest gate that chattered BRAKE on the station.
class ContinuousPlant : public sim::SimMotorBackend {
 public:
  ContinuousPlant() : SimMotorBackend(.005) {}
  bool supports_continuous_yaw() const override { return true; }
};

ControlLoop::Config station_cfg() {
  auto cfg = rest_cfg();
  cfg.service_speed_control = true;
  cfg.allow_unknown_motor_health = true;
  cfg.continuous_yaw_sector_half_span_rad = 0;
  cfg.homing_motion_checks_abort = false;
  cfg.position_servo_kp = 4.0;   // turret_mixed.yaml
  return cfg;
}

struct StationLike : Station {
  int brakes = 0;  // supervisor BRAKE decisions seen by step()
  explicit StationLike(ControlLoop::Config cfg = station_cfg()) : Station(plant(), cfg) {}
  static std::unique_ptr<sim::SimMotorBackend> plant() {
    auto p = std::make_unique<ContinuousPlant>();
    p->set_stops(AxisId::Pitch, -1, 1);
    p->set_stops(AxisId::Yaw, -100, 100);
    p->set_position(AxisId::Pitch, .5);
    p->set_encoder(AxisId::Pitch, 4 * M_PI / 32768, 0.6);   // the CyberGear's 0.38 mrad
    p->set_encoder(AxisId::Yaw, 2 * M_PI / 8192, 0.6);      // the GM6020's
    return p;
  }
  void step(int n = 1) {
    for (int i = 0; i < n; ++i) {
      Station::step();
      brakes += loop->last_decision().action == SafetyAction::Brake;
    }
  }
  bool until(const std::function<bool()>& done, int max_steps = 40'000) {
    for (int i = 0; i < max_steps; ++i) {
      if (done()) return true;
      step();
    }
    return done();
  }
  void run(const char* name, const char* arg = "") {
    loop->submit_command(name, arg);
    step(3);
  }
  bool home() {
    HomingPlanConfig hcfg;
    hcfg.homing.coarse_speed_rad_s = 20 * kDeg2Rad;
    hcfg.homing.fine_speed_rad_s = 2 * kDeg2Rad;
    hcfg.homing.settle_time_s = .3;
    hcfg.travel_bands[0] = TravelBand{0, 115};
    std::vector<HomingAction> actions{{.type = HomingActionType::HomeFullRange, .axis = AxisId::Pitch}};
    std::string error;
    if (!loop->start_homing(HomingPlan(std::move(actions), hcfg), error)) return false;
    const bool ok = until([&] {
      return saw_fault || (loop->position_ready() && loop->at_ready() && loop->phase() == Phase::Hold);
    });
    brakes = 0;  // homing is a contact move with no envelope yet; what follows is the subject
    return ok && !saw_fault;
  }
  double pitch() const { return sim->position(AxisId::Pitch); }
  double soft_min() const { return loop->limits()[static_cast<int>(AxisId::Pitch)].q_soft_min_rad; }
  bool parked() {
    run("request_park");
    return until([&] { return stage() == "parked" || saw_fault; }) && stage() == "parked";
  }
};
}  // namespace

TEST(RestPark, ParkPutsPitchOnItsRestStopAndHoldsItEnergised) {
  Station s;
  ASSERT_TRUE(s.home()) << s.loop->fault_reason();
  s.run("request_park");
  ASSERT_TRUE(s.ack().accepted) << s.ack().reason;
  EXPECT_EQ(s.loop->operating_mode(), OperatingMode::Manual) << "the park owns motion while it runs";
  ASSERT_TRUE(s.until([&] { return s.stage() == "parked" || s.saw_fault; })) << s.stage();
  EXPECT_FALSE(s.saw_fault) << s.loop->fault_reason();
  EXPECT_EQ(s.loop->phase(), Phase::Parked);
  EXPECT_TRUE(s.loop->rest_park_touched());
  EXPECT_NEAR(s.sim->position(AxisId::Pitch), -1.0, 0.3 * kDeg2Rad) << "on the stop homing measured";
  EXPECT_NEAR(s.sim->position(AxisId::Yaw), 0.0, 0.8 * kDeg2Rad);
  // Parked is a hold, not a release: both drives stay in a running mode and the pose stays put.
  s.step(400);
  EXPECT_EQ(s.stage(), "parked");
  EXPECT_FALSE(s.sim->snapshot(AxisId::Pitch, s.t).disabled);
  EXPECT_FALSE(s.sim->snapshot(AxisId::Yaw, s.t).disabled);
  EXPECT_NEAR(s.sim->position(AxisId::Pitch), -1.0, 0.3 * kDeg2Rad);
  EXPECT_EQ(s.loop->telemetry().snapshot().rest_park, "parked");
}

TEST(RestPark, AModeLeavesTheParkWithoutHoming) {
  Station s;
  ASSERT_TRUE(s.home());
  s.run("request_park");
  ASSERT_TRUE(s.until([&] { return s.stage() == "parked"; }));
  s.run("manual_step", "yaw+1");
  EXPECT_FALSE(s.ack().accepted) << "a jog from the stop has no pose to start from: pick a mode";
  s.run("set_mode", "MANUAL");
  ASSERT_TRUE(s.ack().accepted) << s.ack().reason;
  EXPECT_EQ(s.stage(), "lifting") << "off the stop first: it lies outside the soft envelope";
  ASSERT_TRUE(s.until([&] { return s.stage().empty() || s.saw_fault; }, 2000)) << s.stage();
  EXPECT_EQ(s.loop->phase(), Phase::Hold);
  // MANUAL from the park returns to the ready pose, as after homing, and stays homed on the way.
  ASSERT_TRUE(s.until([&] { return s.loop->at_ready() || s.saw_fault; }));
  EXPECT_FALSE(s.saw_fault) << s.loop->fault_reason();
  EXPECT_TRUE(s.loop->homed());
  EXPECT_NEAR(s.sim->position(AxisId::Pitch), 0.0, 1.0 * kDeg2Rad);
  // And it is ordinary MANUAL now: the DPAD's jog is accepted (owner, 2026-10-03: the park is a
  // scripted move, not a state to be stuck in).
  s.run("manual_jog_start", "yaw+:coarse");
  EXPECT_TRUE(s.ack().accepted) << s.ack().reason;
  s.run("manual_jog_stop");
}

TEST(RestPark, AutoLeavesTheParkInOnePress) {
  Station s;
  ASSERT_TRUE(s.home());
  s.run("request_park");
  ASSERT_TRUE(s.until([&] { return s.stage() == "parked"; }));
  s.run("set_mode", "AUTO_ROAM");
  ASSERT_TRUE(s.ack().accepted) << s.ack().reason;
  ASSERT_TRUE(s.until([&] { return s.stage().empty() || s.saw_fault; }, 2000)) << s.stage();
  s.step(3);
  EXPECT_EQ(s.loop->phase(), Phase::Hold);
  EXPECT_EQ(s.loop->operating_mode(), OperatingMode::AutoRoam) << "the one press carries through the lift";
  s.step(400);
  EXPECT_FALSE(s.saw_fault) << s.loop->fault_reason();
  EXPECT_TRUE(s.loop->homed());
}

TEST(RestPark, ShutdownParksThenSwitchesBothMotorsOffAndOnlyHomeStartsAgain) {
  Station s;
  ASSERT_TRUE(s.home());
  s.run("request_shutdown");
  ASSERT_TRUE(s.ack().accepted) << s.ack().reason;
  ASSERT_TRUE(s.until([&] { return s.loop->phase() == Phase::Idle || s.saw_fault; }));
  EXPECT_FALSE(s.saw_fault) << s.loop->fault_reason();
  EXPECT_TRUE(s.sim->snapshot(AxisId::Pitch, s.t).disabled);
  EXPECT_TRUE(s.sim->snapshot(AxisId::Yaw, s.t).disabled);
  EXPECT_NEAR(s.sim->position(AxisId::Pitch), -1.0, 0.3 * kDeg2Rad) << "released on the stop, not beside it";
  EXPECT_FALSE(s.loop->position_ready());
  s.run("set_mode", "AUTO_ROAM");
  s.step(200);
  EXPECT_TRUE(s.sim->snapshot(AxisId::Pitch, s.t).disabled) << "a mode cannot wake a shut-down turret";
  s.run("request_shutdown");
  EXPECT_TRUE(s.ack().accepted) << "already off is an answer, not a refusal";
  s.run("start_homing");
  ASSERT_TRUE(s.ack().accepted) << s.ack().reason;
  ASSERT_TRUE(s.until([&] { return s.saw_fault || (s.loop->homed() && s.loop->at_ready()); }));
  EXPECT_FALSE(s.saw_fault) << s.loop->fault_reason();
}

TEST(RestPark, ABootIntoShutdownStaysOffUntilHome) {
  // Owner, 2026-10-03: two states, Homed or Shutdown, and a boot is Shutdown (like a printer's
  // firmware: up and reachable, motors off, nothing moves until Home). controld's
  // OTA_START_STATE=shutdown is this: no homing at startup, both drives told off.
  Station s;
  s.loop->deenergize_all();
  s.step(400);
  EXPECT_EQ(s.loop->phase(), Phase::Idle);
  EXPECT_FALSE(s.saw_fault) << s.loop->fault_reason();
  EXPECT_FALSE(s.loop->homed());
  EXPECT_TRUE(s.sim->snapshot(AxisId::Pitch, s.t).disabled);
  EXPECT_TRUE(s.sim->snapshot(AxisId::Yaw, s.t).disabled);
  s.run("set_mode", "AUTO_ROAM");
  s.step(200);
  EXPECT_TRUE(s.sim->snapshot(AxisId::Pitch, s.t).disabled) << "a mode cannot wake a shut-down turret";
  s.run("start_homing");
  ASSERT_TRUE(s.ack().accepted) << s.ack().reason;
  ASSERT_TRUE(s.until([&] { return s.saw_fault || (s.loop->homed() && s.loop->at_ready()); }));
  EXPECT_FALSE(s.saw_fault) << s.loop->fault_reason();
}

TEST(RestPark, AShutDownTurretThatIsMovedByHandIsNotBraked) {
  // 2026-10-03 on the station: shut down, pitch on its stop (outside the soft envelope), and the
  // supervisor logged BRAKE 'stop infeasible before soft boundary' and kept a black-box scene, with
  // both motors off. A turret nobody drives has no stop to be feasible.
  Station s;
  ASSERT_TRUE(s.home());
  s.run("request_shutdown");
  ASSERT_TRUE(s.until([&] { return s.loop->phase() == Phase::Idle || s.saw_fault; }));
  ASSERT_FALSE(s.saw_fault) << s.loop->fault_reason();
  int brakes = 0;
  double q = s.sim->position(AxisId::Pitch);
  for (int i = 0; i < 200; ++i) {   // someone leans on it: pitch creeps further out, 0.2 rad/s
    q -= 0.001;
    s.sim->set_position(AxisId::Pitch, q);
    s.step(1);
    brakes += s.loop->last_decision().action == SafetyAction::Brake;
  }
  EXPECT_EQ(brakes, 0);
  EXPECT_FALSE(s.saw_fault) << s.loop->fault_reason();
}

TEST(RestPark, ShutdownWhileParkedReleasesFromTheStop) {
  Station s;
  ASSERT_TRUE(s.home());
  s.run("request_park");
  ASSERT_TRUE(s.until([&] { return s.stage() == "parked"; }));
  s.run("request_shutdown");
  ASSERT_TRUE(s.ack().accepted) << s.ack().reason;
  ASSERT_TRUE(s.until([&] { return s.loop->phase() == Phase::Idle; }, 2000));
  EXPECT_TRUE(s.sim->snapshot(AxisId::Pitch, s.t).disabled);
}

TEST(RestPark, StopDuringTheTouchHoldsShortAndShutdownTouchesAgainBeforeReleasing) {
  Station s;
  ASSERT_TRUE(s.home());
  s.run("request_park");
  ASSERT_TRUE(s.until([&] { return s.stage() == "touching"; }));
  s.step(100);  // half a second onto the stop: still well short of it
  s.run("stop_motion");
  EXPECT_TRUE(s.ack().accepted);
  EXPECT_EQ(s.stage(), "parked");
  EXPECT_FALSE(s.loop->rest_park_touched());
  const double held = s.sim->position(AxisId::Pitch);
  s.step(200);
  EXPECT_NEAR(s.sim->position(AxisId::Pitch), held, 0.2 * kDeg2Rad) << "a stop does not keep going";
  EXPECT_GT(held, -1.0 + 0.5 * kDeg2Rad);
  s.run("request_shutdown");
  ASSERT_TRUE(s.until([&] { return s.loop->phase() == Phase::Idle || s.saw_fault; }));
  EXPECT_FALSE(s.saw_fault) << s.loop->fault_reason();
  EXPECT_NEAR(s.sim->position(AxisId::Pitch), -1.0, 0.3 * kDeg2Rad) << "on the stop before letting go";
}

TEST(RestPark, StopDuringTheMoveHoldsWhereItIsInManual) {
  Station s;
  ASSERT_TRUE(s.home());
  s.run("request_park");
  s.step(60);  // under way toward the approach pose
  ASSERT_EQ(s.stage(), "moving");
  s.run("stop_motion");
  EXPECT_EQ(s.stage(), "");
  EXPECT_EQ(s.loop->phase(), Phase::Hold);
  s.step(200);  // brake to rest
  const double q = s.sim->position(AxisId::Pitch);
  s.step(400);
  EXPECT_NEAR(s.sim->position(AxisId::Pitch), q, 0.5 * kDeg2Rad)
      << "neither on to the park nor back to where it was before";
}

TEST(RestPark, ParkAndShutdownNeedAHomedTurret) {
  Station s;
  s.step(3);
  s.run("request_park");
  EXPECT_FALSE(s.ack().accepted);
  EXPECT_NE(s.ack().reason.find("Home first"), std::string::npos) << s.ack().reason;
}

TEST(RestPark, HomeRecoversAFaultAndHomes) {
  auto cfg = rest_cfg();
  cfg.start_in_auto_roam = true;  // the station's normal startup
  Station s(nullptr, cfg);
  ASSERT_TRUE(s.home());
  s.sim->set_faults(AxisId::Pitch, 1);
  s.step(3);
  ASSERT_EQ(s.loop->phase(), Phase::Fault);
  s.sim->set_faults(AxisId::Pitch, 0);  // the drive's own fault is gone; the latch is ours
  s.saw_fault = false;
  s.run("start_homing");
  ASSERT_TRUE(s.ack().accepted) << s.ack().reason;
  EXPECT_NE(s.ack().reason.find("recovering"), std::string::npos) << s.ack().reason;
  ASSERT_TRUE(s.until([&] { return s.saw_fault || (s.loop->homed() && s.loop->at_ready()); }));
  EXPECT_FALSE(s.saw_fault) << s.loop->fault_reason();
  // Home is a fresh start: the station resumes AUTO_ROAM at the ready pose, as at power-up.
  s.step(10);
  EXPECT_EQ(s.loop->operating_mode(), OperatingMode::AutoRoam);
}

TEST(RestPark, HomeDoesNotClearAFaultTheDriveStillReports) {
  Station s;
  ASSERT_TRUE(s.home());
  s.sim->set_faults(AxisId::Pitch, 1);
  s.step(3);
  ASSERT_EQ(s.loop->phase(), Phase::Fault);
  s.run("start_homing");
  s.step(1200);
  EXPECT_EQ(s.loop->phase(), Phase::Fault);
  EXPECT_NE(s.loop->fault_reason().find("RECOVERY FAILED"), std::string::npos) << s.loop->fault_reason();
}

TEST(RestPark, ContinuousYawParksAtTheNearestWholeTurn) {
  auto plant = std::make_unique<ContinuousPlant>();
  plant->set_stops(AxisId::Pitch, -1, 1);
  plant->set_stops(AxisId::Yaw, -100, 100);
  plant->set_position(AxisId::Pitch, .5);
  auto cfg = rest_cfg();
  cfg.service_speed_control = true;
  cfg.allow_unknown_motor_health = true;
  cfg.continuous_yaw_sector_half_span_rad = 0;
  cfg.homing_motion_checks_abort = false;
  Station s(std::move(plant), cfg);
  HomingPlanConfig hcfg;
  hcfg.homing.coarse_speed_rad_s = 20 * kDeg2Rad;
  hcfg.homing.fine_speed_rad_s = 2 * kDeg2Rad;
  hcfg.homing.settle_time_s = .3;
  hcfg.travel_bands[0] = TravelBand{0, 115};
  std::vector<HomingAction> actions{{.type = HomingActionType::HomeFullRange, .axis = AxisId::Pitch}};
  std::string error;
  ASSERT_TRUE(s.loop->start_homing(HomingPlan(std::move(actions), hcfg), error)) << error;
  ASSERT_TRUE(s.until([&] {
    return s.saw_fault || (s.loop->position_ready() && s.loop->at_ready() && s.loop->phase() == Phase::Hold);
  }));
  ASSERT_FALSE(s.saw_fault) << s.loop->fault_reason();
  // The process stop asks this before parking (controld main): it must hold on a continuous-yaw
  // station, where homed() never does.
  EXPECT_FALSE(s.loop->homed());
  EXPECT_TRUE(s.loop->rest_shutdown_available());
  // Four hundred degrees round: a patrol that has been going a while.
  s.run("set_mode", "MANUAL");
  s.sim->set_position(AxisId::Yaw, 7.0);
  s.step(5);
  s.run("set_mode", "MANUAL");
  s.run("request_park");
  ASSERT_TRUE(s.ack().accepted) << s.ack().reason;
  ASSERT_TRUE(s.until([&] { return s.stage() == "parked" || s.saw_fault || s.stage().empty(); }));
  ASSERT_EQ(s.stage(), "parked") << s.loop->fault_reason();
  EXPECT_NEAR(s.sim->position(AxisId::Yaw), 2 * M_PI, 0.8 * kDeg2Rad) << "zero by the short way, not 400 back";
  EXPECT_NEAR(s.sim->position(AxisId::Pitch), -1.0, 0.3 * kDeg2Rad);
}

// --- The pitch envelope's boundary: position and direction (2026-10-03) ----------------------------
//
// The rest stop lies outside the soft envelope by design (owner ruling 2026-10-03: pitch rests on
// it). On the station, Auto from the park handed the turret to the roam with pitch still on the stop;
// every one-count flicker of the encoder read as outward motion past a boundary already behind it,
// the supervisor chattered BRAKE five times a second, and no jog or mode could bring pitch back.
// Twenty minutes later a jog that did reach the servo's engage line tripped its end-stop guard one
// count later and faulted the station. These tests hold the physical outcome, not the labels: pitch
// back inside its envelope, no BRAKE on the way, and the DPAD working afterwards.

TEST(RestParkBoundary, AutoFromTheParkLiftsPitchIntoItsEnvelopeWithoutABrake) {
  StationLike s;
  ASSERT_TRUE(s.home()) << s.loop->fault_reason();
  ASSERT_TRUE(s.parked()) << s.stage() << " " << s.loop->fault_reason();
  ASSERT_LT(s.pitch(), s.soft_min()) << "the rest stop is outside the soft envelope: the case under test";
  s.brakes = 0;
  s.run("set_mode", "AUTO_ROAM");
  ASSERT_TRUE(s.ack().accepted) << s.ack().reason;
  EXPECT_EQ(s.stage(), "lifting") << "the park lifts pitch off its stop before any mode drives it";
  ASSERT_TRUE(s.until([&] { return s.saw_fault || (s.stage().empty() &&
                                   s.loop->operating_mode() == OperatingMode::AutoRoam); }, 2000))
      << s.stage() << " " << s.loop->fault_reason();
  EXPECT_FALSE(s.saw_fault) << s.loop->fault_reason();
  EXPECT_GT(s.pitch(), s.soft_min()) << "the mode starts inside the envelope";
  s.step(400);
  EXPECT_EQ(s.brakes, 0) << s.loop->last_decision().reason;
  EXPECT_EQ(s.loop->phase(), Phase::Hold);
  EXPECT_GT(s.pitch(), s.soft_min());
}

TEST(RestParkBoundary, ManualFromTheParkReturnsToReadyAndTheDpadWorks) {
  StationLike s;
  ASSERT_TRUE(s.home());
  ASSERT_TRUE(s.parked());
  s.brakes = 0;
  s.run("set_mode", "MANUAL");
  ASSERT_TRUE(s.ack().accepted) << s.ack().reason;
  ASSERT_TRUE(s.until([&] { return s.saw_fault || (s.stage().empty() && s.loop->at_ready()); }, 4000))
      << s.stage() << " " << s.loop->fault_reason();
  EXPECT_EQ(s.loop->operating_mode(), OperatingMode::Manual);
  const double before = s.pitch();
  s.run("manual_jog_start", "pitch-:coarse");
  ASSERT_TRUE(s.ack().accepted) << s.ack().reason;
  for (int i = 0; i < 4; ++i) {  // the DPAD renews its 300 ms lease while held
    s.step(40);
    s.run("manual_jog_start", "pitch-:coarse");
  }
  s.run("manual_jog_stop");
  s.step(200);
  EXPECT_LT(s.pitch(), before - 1 * kDeg2Rad) << "the jog moved pitch";
  EXPECT_EQ(s.brakes, 0) << s.loop->last_decision().reason;
  EXPECT_FALSE(s.saw_fault) << s.loop->fault_reason();
}

TEST(RestParkBoundary, StopDuringTheLiftHoldsWhereItIsAndAModeCarriesOn) {
  StationLike s;
  ASSERT_TRUE(s.home());
  ASSERT_TRUE(s.parked());
  s.brakes = 0;
  s.run("set_mode", "AUTO_ROAM");
  ASSERT_EQ(s.stage(), "lifting");
  s.step(30);
  s.run("stop_motion");
  EXPECT_TRUE(s.ack().accepted) << s.ack().reason;
  EXPECT_EQ(s.stage(), "parked");
  EXPECT_EQ(s.loop->phase(), Phase::Parked);
  s.step(20);
  const double held = s.pitch();
  s.step(200);
  EXPECT_NEAR(s.pitch(), held, 0.3 * kDeg2Rad) << "a stop does not keep lifting";
  s.run("set_mode", "AUTO_ROAM");
  ASSERT_TRUE(s.until([&] { return s.saw_fault || s.stage().empty(); }, 2000)) << s.stage();
  s.step(3);  // the mode command runs on the cycle after the lift ends
  EXPECT_GT(s.pitch(), s.soft_min());
  EXPECT_EQ(s.loop->operating_mode(), OperatingMode::AutoRoam);
  EXPECT_EQ(s.brakes, 0) << s.loop->last_decision().reason;
}

TEST(RestParkBoundary, ShutdownDuringTheLiftTouchesTheStopAgainBeforeReleasing) {
  StationLike s;
  ASSERT_TRUE(s.home());
  ASSERT_TRUE(s.parked());
  s.run("set_mode", "AUTO_ROAM");
  ASSERT_EQ(s.stage(), "lifting");
  s.step(30);
  EXPECT_TRUE(s.loop->rest_shutdown_available()) << "a process stop during the lift parks too";
  s.run("request_shutdown");
  ASSERT_TRUE(s.ack().accepted) << s.ack().reason;
  ASSERT_TRUE(s.until([&] { return s.loop->phase() == Phase::Idle || s.saw_fault; }, 4000));
  EXPECT_FALSE(s.saw_fault) << s.loop->fault_reason();
  EXPECT_NEAR(s.pitch(), -1.0, 0.3 * kDeg2Rad) << "released on the stop, not beside it";
}

TEST(RestParkBoundary, AnAxisFoundOutsideItsEnvelopeGoesBackInWithoutABrake) {
  // Position and direction, independent of the park: whatever left pitch past its soft limit, the
  // way back in is always open and an axis holding still out there is not braked.
  StationLike s;
  ASSERT_TRUE(s.home());
  s.run("set_mode", "MANUAL");
  s.step(10);
  // Held out there by hand until the velocity estimate has forgotten the jump (that jump is a real
  // outward lunge to the estimator, and braking it is right), then let go.
  for (int i = 0; i < 200; ++i) {
    s.sim->set_position(AxisId::Pitch, s.soft_min() - 3 * kDeg2Rad);
    s.step();
  }
  s.brakes = 0;
  ASSERT_TRUE(s.until([&] { return s.saw_fault || (s.loop->at_ready() && s.pitch() > s.soft_min()); }, 4000))
      << "pitch " << s.pitch() << " soft min " << s.soft_min();
  EXPECT_EQ(s.brakes, 0) << s.loop->last_decision().reason;
  EXPECT_FALSE(s.saw_fault) << s.loop->fault_reason();
}

TEST(RestParkBoundary, OutwardMotionPastTheEnvelopeIsStillBraked) {
  // The other half of the rule: a real outward motion beyond the soft limit is braked, and the brake
  // stops it where it is rather than dragging it back in.
  StationLike s;
  ASSERT_TRUE(s.home());
  s.run("set_mode", "MANUAL");
  s.step(10);
  double q = s.soft_min() - 1 * kDeg2Rad;
  for (int i = 0; i < 200; ++i) {  // held there by hand until the velocity estimate has settled
    s.sim->set_position(AxisId::Pitch, q);
    s.step();
  }
  s.brakes = 0;
  for (int i = 0; i < 100; ++i) {  // still, outside: not braked
    s.sim->set_position(AxisId::Pitch, q);
    s.step();
  }
  EXPECT_EQ(s.brakes, 0) << "an axis holding still outside its envelope is not braked";
  int braked_at = -1;
  for (int i = 0; i < 100 && braked_at < 0; ++i) {  // pushed outward at 6 deg/s
    q -= 6 * kDeg2Rad * 0.005;
    s.sim->set_position(AxisId::Pitch, q);
    s.step();
    if (s.loop->last_decision().action == SafetyAction::Brake) braked_at = i;
  }
  ASSERT_GE(braked_at, 0) << "never braked";
  EXPECT_LE(braked_at, 12) << "braked within 60 ms of a 6 deg/s outward push";
}
