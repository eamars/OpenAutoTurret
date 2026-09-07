#include <gtest/gtest.h>
#include "control/control_loop.hpp"
#include "sim/sim_motor_backend.hpp"

using namespace ota;

namespace {
// Only simulation may establish an already-referenced fixture this way.
class ReferencedPlant : public sim::SimMotorBackend {
 public:
  bool independent_available = true;
  ParkPositionEvidence park_position_evidence(AxisId a, TimeNs now) const override {
    return independent_available ? sim::SimMotorBackend::park_position_evidence(a, now)
                                 : ParkPositionEvidence{};
  }
  bool adopt_running_mode(AxisId a, bool position, std::string& err,
                          double = -1, double = 1) override {
    return position ? enter_position_mode(a, .1, err) : enter_speed_mode(a, 1, err);
  }
};
struct Parking : testing::Test {
  ReferencedPlant* plant;
  std::unique_ptr<ControlLoop> loop;
  TimeNs now = 1'000'000'000;
  void setup(double pitch_deg = 65, double yaw_deg = 330) {
    auto backend = std::make_unique<ReferencedPlant>();
    plant = backend.get();
    ControlLoop::Config cfg;
    cfg.service_speed_control = true;
    cfg.park.park_logical_deg = {40, 176};
    cfg.park.speed_deg_s = 3;
    std::array<AxisLogicalModel, 2> models;
    std::array<AxisLimits, 2> limits;
    const double travel[] = {80*kDeg2Rad, 360*kDeg2Rad};
    for (int i = 0; i < 2; ++i) {
      models[i].set_reference(0, 0);
      limits[i].set_from_endpoints(0, travel[i], 5*kDeg2Rad);
      plant->set_stops(static_cast<AxisId>(i), 0, travel[i]);
    }
    plant->set_position(AxisId::Pitch, pitch_deg*kDeg2Rad);
    plant->set_position(AxisId::Yaw, yaw_deg*kDeg2Rad);
    loop = std::make_unique<ControlLoop>(cfg, std::move(backend));
    std::string err;
    ASSERT_TRUE(loop->restore_retained_homing(models, limits, err)) << err;
    tick();
    ASSERT_TRUE(loop->start_parking(err)) << err;
  }
  void tick() { loop->step(now, 5'000'000); now += 5'000'000; }
};
TEST_F(Parking, ProductionPoseLongReturnParksWithoutBrake) {
  setup();
  for (int i=0; i<40000 && loop->phase()==Phase::Parking; ++i) {
    tick();
    ASSERT_NE(loop->last_decision().action, SafetyAction::Brake);
  }
  ASSERT_EQ(loop->phase(), Phase::Parked) << loop->fault_reason();
  for (auto a : {AxisId::Pitch, AxisId::Yaw}) {
    auto s = plant->snapshot(a, now);
    EXPECT_FALSE(s.in_speed_mode);
    EXPECT_FALSE(s.in_position_mode);
  }
  EXPECT_NEAR(plant->position(AxisId::Yaw)*kRad2Deg, 176, .5);
  EXPECT_NEAR(plant->position(AxisId::Pitch)*kRad2Deg, 40, .5);
}
TEST_F(Parking, HomeAfterShutdownIsRejectedBeforeNextPhase) {
  setup();
  loop->submit_command("request_shutdown", "");
  loop->submit_command("start_homing", "");
  tick();
  auto s = loop->telemetry().snapshot();
  EXPECT_TRUE(loop->shutdown_requested());
  EXPECT_EQ(loop->phase(), Phase::Parking);
  EXPECT_EQ(s.cmd_ack_accepted, 0);
}
TEST_F(Parking, StaleFeedbackStopsBeforeAnyParkingMovement) {
  setup();
  for (int i=0;i<10;++i) tick();
  plant->set_feedback_ok(AxisId::Pitch, false);
  const double yaw = plant->position(AxisId::Yaw);
  tick();
  EXPECT_EQ(loop->last_decision().action, SafetyAction::Brake);
  EXPECT_DOUBLE_EQ(plant->position(AxisId::Yaw), yaw);
}
TEST_F(Parking, HardFaultDisablesBothAxes) {
  setup();
  plant->set_faults(AxisId::Yaw, 1);
  tick();
  EXPECT_EQ(loop->last_decision().action, SafetyAction::Disable);
  EXPECT_EQ(loop->phase(), Phase::Fault);
  EXPECT_FALSE(plant->snapshot(AxisId::Pitch, now).in_speed_mode);
  EXPECT_FALSE(plant->snapshot(AxisId::Yaw, now).in_speed_mode);
}
TEST_F(Parking, TemperatureFaultStopsParking) {
  setup();
  plant->set_temp(AxisId::Pitch, 90);
  tick();
  EXPECT_EQ(loop->last_decision().action, SafetyAction::FaultStop);
  EXPECT_EQ(loop->phase(), Phase::Fault);
}
TEST_F(Parking, BoundaryViolationStillBrakes) {
  setup();
  plant->set_position(AxisId::Yaw, 354.9*kDeg2Rad);
  tick();
  EXPECT_EQ(loop->last_decision().action, SafetyAction::Brake);
}
TEST_F(Parking, MissingIndependentConfirmationFailsWithoutReleasingAndHomeCanRecover) {
  setup(43, 179);
  plant->independent_available = false;
  for (int i=0; i<10000 && loop->phase()==Phase::Parking; ++i) tick();
  ASSERT_EQ(loop->phase(), Phase::Fault);
  EXPECT_NE(loop->fault_reason().find("independent physical"), std::string::npos);
  EXPECT_TRUE(plant->snapshot(AxisId::Pitch, now).in_speed_mode);
  EXPECT_TRUE(plant->snapshot(AxisId::Yaw, now).in_speed_mode);
  for (int i=0; i<200; ++i) tick();
  std::string err;
  HomingPlanConfig config;
  ASSERT_TRUE(loop->start_homing(HomingPlan({}, config), err)) << err;
  EXPECT_EQ(loop->phase(), Phase::Homing);
}
TEST_F(Parking, HomeAfterParkedStartsNewCalibration) {
  setup(43, 179);
  for (int i=0; i<10000 && loop->phase()==Phase::Parking; ++i) tick();
  ASSERT_EQ(loop->phase(), Phase::Parked) << loop->fault_reason();
  EXPECT_FALSE(loop->homed());
  tick();
  std::string err;
  HomingPlanConfig config;
  ASSERT_TRUE(loop->start_homing(HomingPlan({}, config), err)) << err;
  EXPECT_EQ(loop->phase(), Phase::Homing);
}
TEST_F(Parking, UntrustedFeedbackBlocksHomeRecovery) {
  setup(43, 179);
  plant->independent_available = false;
  for (int i=0; i<10000 && loop->phase()==Phase::Parking; ++i) tick();
  ASSERT_EQ(loop->phase(), Phase::Fault);
  plant->set_feedback_ok(AxisId::Pitch, false);
  tick();
  std::string err;
  HomingPlanConfig config;
  EXPECT_FALSE(loop->start_homing(HomingPlan({}, config), err));
  EXPECT_EQ(loop->phase(), Phase::Fault);
}
}  // namespace
