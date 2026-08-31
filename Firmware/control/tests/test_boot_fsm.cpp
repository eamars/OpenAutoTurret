// OpenAutoTurret — BootFsm tests (§27 boot sequence, transport-agnostic).
#include <memory>

#include "control/boot_fsm.hpp"
#include "sim/sim_motor_backend.hpp"

#include <gtest/gtest.h>

using namespace ota;

namespace {

// Drive the boot FSM to a terminal state (Unhomed or FaultLocked).
BootState run_to_terminal(BootFsm& fsm) {
  for (int i = 0; i < 16 && !fsm.ready_to_home() && !fsm.faulted(); ++i) {
    fsm.step();
  }
  return fsm.state();
}

}  // namespace

TEST(BootFsm, Boot_Success_ReachesUnhomed) {
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  BootFsm fsm(*backend, BootConfig{});
  EXPECT_EQ(fsm.state(), BootState::PowerOn);
  EXPECT_FALSE(fsm.ready_to_home());

  EXPECT_EQ(run_to_terminal(fsm), BootState::Unhomed);
  EXPECT_TRUE(fsm.ready_to_home());
  EXPECT_FALSE(fsm.faulted());
  EXPECT_TRUE(fsm.error().empty());
  // Both motors discovered (distinct unique ids captured).
  EXPECT_NE(fsm.unique_ids()[0], fsm.unique_ids()[1]);
}

TEST(BootFsm, Boot_DiscoveryFailure_FaultLocks) {
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  backend->set_discovery_ok(AxisId::Pitch, false);  // pitch missing on the bus
  BootFsm fsm(*backend, BootConfig{});

  EXPECT_EQ(run_to_terminal(fsm), BootState::FaultLocked);
  EXPECT_TRUE(fsm.faulted());
  EXPECT_FALSE(fsm.ready_to_home());
  EXPECT_NE(fsm.error().find("pitch"), std::string::npos);
}

TEST(BootFsm, Boot_SelfTestFailure_FaultLocks) {
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  backend->set_register_ok(false);  // motors found but do not answer reads
  BootFsm fsm(*backend, BootConfig{});

  EXPECT_EQ(run_to_terminal(fsm), BootState::FaultLocked);
  EXPECT_TRUE(fsm.faulted());
  EXPECT_NE(fsm.error().find("self-test"), std::string::npos);
}
