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
  EXPECT_EQ(s.loop->phase(), Phase::Hold);
  EXPECT_EQ(s.stage(), "");
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
  EXPECT_EQ(s.stage(), "");
  EXPECT_EQ(s.loop->phase(), Phase::Hold);
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
  class ContinuousPlant : public sim::SimMotorBackend {
   public:
    ContinuousPlant() : SimMotorBackend(.005) {}
    bool supports_continuous_yaw() const override { return true; }
  };
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
