// ADR-007: the one-shot actuator. What reaches the PWM, in what order, for each thing the operator
// does; the file is read whole or not at all; the channel is found by address; the gate's shapes.
#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "actuator/actuator.hpp"
#include "web/command_validation.hpp"

namespace ota {
namespace {

using Write = std::pair<int, bool>;  // (pulse_us, enabled); pulse is meaningless when off

class FakePwm : public PwmOutput {
 public:
  struct Shared {
    std::mutex m;
    std::vector<Write> writes;
    bool fail = false;
    int fail_pulse = -1;  // refuse only switching on at this pulse (a duty_cycle write that fails)
  };
  explicit FakePwm(std::shared_ptr<Shared> s) : s_(std::move(s)) {}
  bool set(int pulse_us, bool enabled, std::string& err) override {
    std::lock_guard lk(s_->m);
    if (s_->fail || (enabled && pulse_us == s_->fail_pulse)) { err = "EIO"; return false; }
    s_->writes.emplace_back(enabled ? pulse_us : 0, enabled);
    return true;
  }

 private:
  std::shared_ptr<Shared> s_;
};

std::filesystem::path scratch_dir(const char* tag) {
  std::random_device rd;
  auto dir = std::filesystem::temp_directory_path() / (std::string("ota-actuator-") + tag + std::to_string(rd()));
  std::filesystem::create_directories(dir);
  return dir;
}

ActuatorConfig owner_config(bool hold = false) {
  // The owner's calibration of 2026-10-30, servo off its mechanism.
  ActuatorConfig c;
  c.valid = true;
  c.gpio = 15;
  c.rest_us = 1650;
  c.active_us = 1450;
  c.dwell_ms = 80;  // shortened for the test; the owner's is 300
  c.hold_at_rest = hold;
  return c;
}

struct Rig {
  std::shared_ptr<FakePwm::Shared> pwm = std::make_shared<FakePwm::Shared>();
  std::unique_ptr<Actuator> act;
  explicit Rig(ActuatorConfig c, std::string path = "", std::string problem = "", int retry_ms = 5000) {
    act = std::make_unique<Actuator>(std::make_unique<FakePwm>(pwm), "", std::move(path), 15, c, problem,
                                     Actuator::Options{60, retry_ms});
  }
  std::vector<Write> writes() {
    std::lock_guard lk(pwm->m);
    return pwm->writes;
  }
  // The worker writes what is wanted when it runs, so a request made before it ran may replace an
  // earlier one unwritten (correctly: the servo goes where it is wanted now). Tests about what was
  // written first wait for it.
  bool wait_writes(std::size_t n, int ms = 2000) {
    for (int i = 0; i < ms / 5 && writes().size() < n; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    return writes().size() >= n;
  }
  bool wait_state(const std::string& want, int ms = 2000) {
    for (int i = 0; i < ms / 5; ++i) {
      if (act->state_name() == want) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return act->state_name() == want;
  }
};

TEST(ActuatorConfig, TheToolsFileIsReadAndHoldAtRestDefaultsToLimp) {
  // Exactly what tools/actuator_calibrate.py wrote on the station on 2026-10-30.
  const std::string tool = R"({
  "schema": 1,
  "gpio": 15,
  "period_us": 20000,
  "rest_us": 1650,
  "active_us": 1450,
  "dwell_ms": 300.0,
  "saved": "2026-10-30T21:50:54+13:00"
})";
  ActuatorConfig c;
  std::string err;
  ASSERT_TRUE(actuator_config_from_json(tool, c, err)) << err;
  EXPECT_EQ(c.rest_us, 1650);
  EXPECT_EQ(c.active_us, 1450);
  EXPECT_DOUBLE_EQ(c.dwell_ms, 300.0);
  EXPECT_FALSE(c.hold_at_rest);
  ActuatorConfig r;
  c.hold_at_rest = true;
  ASSERT_TRUE(actuator_config_from_json(actuator_config_to_json(c), r, err)) << err;
  EXPECT_TRUE(r.hold_at_rest);
  EXPECT_EQ(r.saved, c.saved);
}

TEST(ActuatorConfig, AnythingOutOfBoundsOrMalformedIsRefusedWhole) {
  ActuatorConfig c;
  std::string err;
  for (const char* bad : {
           "", "[]", R"({"schema":2,"gpio":15,"rest_us":1500,"active_us":1400,"dwell_ms":300})",
           R"({"schema":1,"gpio":4,"rest_us":1500,"active_us":1400,"dwell_ms":300})",
           R"({"schema":1,"gpio":15,"rest_us":2600,"active_us":1400,"dwell_ms":300})",
           R"({"schema":1,"gpio":15,"rest_us":1500,"active_us":400,"dwell_ms":300})",
           R"({"schema":1,"gpio":15,"rest_us":1500,"active_us":1400,"dwell_ms":6000})",
           R"({"schema":1,"gpio":15,"rest_us":1500,"active_us":1400})",
           R"({"schema":1,"gpio":15,"rest_us":"A","active_us":1400,"dwell_ms":300})"}) {
    EXPECT_FALSE(actuator_config_from_json(bad, c, err)) << bad;
    EXPECT_FALSE(c.valid) << bad;
  }
}

TEST(Actuator, AtStartItGoesToRestOnceThenGoesLimp) {
  Rig rig(owner_config());
  ASSERT_TRUE(rig.wait_state("off"));
  const auto w = rig.writes();
  ASSERT_GE(w.size(), 2u);
  EXPECT_EQ(w.front(), Write(1650, true));
  EXPECT_EQ(w.back(), Write(0, false));
}

TEST(Actuator, HoldAtRestKeepsItPowered) {
  Rig rig(owner_config(true));
  ASSERT_TRUE(rig.wait_state("rest"));
  EXPECT_EQ(rig.writes(), std::vector<Write>{Write(1650, true)});
}

TEST(Actuator, AShotIsRestActiveDwellRestThenLimp) {
  Rig rig(owner_config());
  ASSERT_TRUE(rig.wait_state("off"));
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(rig.act->fire(), "");
  EXPECT_EQ(rig.act->fire(), "shot in progress");  // never queued
  ASSERT_TRUE(rig.wait_state("returning"));
  const auto dwell = std::chrono::steady_clock::now() - t0;
  EXPECT_GE(dwell, std::chrono::milliseconds(80));
  EXPECT_NE(rig.act->fire(), "");  // still travelling back
  ASSERT_TRUE(rig.wait_state("off"));
  const auto w = rig.writes();
  ASSERT_EQ(w.size(), 5u);  // start: A, off; shot: B, A, off
  EXPECT_EQ(w[2], Write(1450, true));
  EXPECT_EQ(w[3], Write(1650, true));
  EXPECT_EQ(w[4], Write(0, false));
  EXPECT_NE(rig.act->status_json()->find("\"shots\":1"), std::string::npos);
  EXPECT_EQ(rig.act->fire(), "");  // and again once at rest
}

TEST(Actuator, StopMotionEndsAShotAtRestAtOnce) {
  ActuatorConfig c = owner_config();
  c.dwell_ms = 3000;
  Rig rig(c);
  ASSERT_TRUE(rig.wait_state("off"));
  ASSERT_EQ(rig.act->fire(), "");
  ASSERT_TRUE(rig.wait_state("active"));
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  const auto t0 = std::chrono::steady_clock::now();
  rig.act->abort();
  ASSERT_TRUE(rig.wait_state("off"));
  EXPECT_LT(std::chrono::steady_clock::now() - t0, std::chrono::milliseconds(1000));
  const auto w = rig.writes();
  EXPECT_EQ(w[w.size() - 2], Write(1650, true));
}

TEST(Actuator, UnconfiguredItIsOffRefusesToFireAndIsCalibratedFromScratch) {
  const auto dir = scratch_dir("cal");
  const std::string path = (dir / "actuator.json").string();
  Rig rig(ActuatorConfig{}, path);
  EXPECT_EQ(rig.act->state_name(), "unconfigured");
  ASSERT_TRUE(rig.wait_writes(1));
  EXPECT_NE(rig.act->fire().find("not calibrated"), std::string::npos);
  EXPECT_NE(rig.act->set_pulse(1500), "");  // no session
  ASSERT_EQ(rig.act->begin_calibration(std::nullopt), "");
  EXPECT_EQ(rig.act->set_pulse(400), "pulse 400 us is outside 500..2500 us");
  ASSERT_EQ(rig.act->set_pulse(1600), "");
  ASSERT_EQ(rig.act->test(1600, 1500, 50), "");
  EXPECT_EQ(rig.act->save(1600, 1500, 50), "the actuator's calibration is not open");  // mid-test
  ASSERT_TRUE(rig.wait_state("calibrating"));
  ASSERT_EQ(rig.act->save(1600, 1500, 250), "");
  for (int i = 0; i < 400 && rig.act->status_json()->find("\"saved\":1") == std::string::npos; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  ActuatorConfig on_disk;
  std::string err;
  ASSERT_TRUE(load_actuator_config(path, on_disk, err)) << err;
  EXPECT_EQ(on_disk.rest_us, 1600);
  EXPECT_EQ(on_disk.active_us, 1500);
  EXPECT_DOUBLE_EQ(on_disk.dwell_ms, 250);
  EXPECT_EQ(on_disk.gpio, 15);
  EXPECT_FALSE(on_disk.saved.empty());
  EXPECT_NE(rig.act->fire(), "");  // the session is still open
  ASSERT_EQ(rig.act->end_calibration(), "");
  ASSERT_TRUE(rig.wait_state("off"));
  const auto w = rig.writes();
  EXPECT_EQ(w.front(), Write(0, false));  // switched off at start: a crash may have left it running
  EXPECT_EQ(w[w.size() - 2], Write(1600, true));
  EXPECT_EQ(rig.act->fire(), "");
  std::filesystem::remove_all(dir);
}

TEST(Actuator, ClosingAnUnsavedSessionReturnsToTheSavedRest) {
  Rig rig(owner_config());
  ASSERT_TRUE(rig.wait_state("off"));
  ASSERT_EQ(rig.act->begin_calibration(1500), "");
  ASSERT_EQ(rig.act->set_pulse(1520), "");
  ASSERT_EQ(rig.act->end_calibration(), "");
  ASSERT_TRUE(rig.wait_state("off"));
  const auto w = rig.writes();
  EXPECT_EQ(w[w.size() - 2], Write(1650, true));
  EXPECT_EQ(rig.act->status_json()->find("\"rest_us\":1520"), std::string::npos);
}

TEST(Actuator, TheProcessStopEndsAShotAtRestAndSwitchesOff) {
  ActuatorConfig c = owner_config(true);
  c.dwell_ms = 3000;
  Rig rig(c);
  ASSERT_TRUE(rig.wait_state("rest"));
  ASSERT_EQ(rig.act->fire(), "");
  ASSERT_TRUE(rig.wait_writes(2));  // at B
  rig.act->shutdown();
  EXPECT_EQ(rig.act->state_name(), "stopped");
  const auto w = rig.writes();
  ASSERT_GE(w.size(), 3u);
  EXPECT_EQ(w[w.size() - 2], Write(1650, true));
  EXPECT_EQ(w.back(), Write(0, false));
  EXPECT_NE(rig.act->fire(), "");
}

TEST(Actuator, APwmThatCannotBeWrittenMakesItUnavailableNotSilentlyStuck) {
  Rig rig(owner_config(true));
  ASSERT_TRUE(rig.wait_state("rest"));
  {
    std::lock_guard lk(rig.pwm->m);
    rig.pwm->fail = true;
  }
  ASSERT_EQ(rig.act->fire(), "");
  ASSERT_TRUE(rig.wait_state("unavailable"));
  EXPECT_NE(rig.act->status_json()->find("PWM write failed"), std::string::npos);
  EXPECT_NE(rig.act->fire().find("unavailable"), std::string::npos);
}

// Review of 2026-10-30: the dwell used to be timed from when the shot was accepted, so a dwell of 0
// never wrote B at all, and every dwell lost the worker's wake-up and the write.
TEST(Actuator, BIsAlwaysWrittenAndHeldAtLeastOnePeriod) {
  ActuatorConfig c = owner_config();
  c.dwell_ms = 0;
  Rig rig(c);
  ASSERT_TRUE(rig.wait_state("off"));
  ASSERT_EQ(rig.act->fire(), "");
  ASSERT_TRUE(rig.wait_state("returning"));
  ASSERT_TRUE(rig.wait_state("off"));
  const auto w = rig.writes();
  ASSERT_EQ(w.size(), 5u);
  EXPECT_EQ(w[2], Write(1450, true));
  EXPECT_EQ(w[3], Write(1650, true));
}

// Review of 2026-10-30: a failed return to A left the output on at B while the status said "off".
TEST(Actuator, AFailedReturnToRestSwitchesOffAndSaysWhatIsOnTheWire) {
  Rig rig(owner_config(), "", "", 150);
  ASSERT_TRUE(rig.wait_state("off"));
  ASSERT_EQ(rig.act->fire(), "");
  ASSERT_TRUE(rig.wait_writes(3));  // at B
  {
    std::lock_guard lk(rig.pwm->m);
    rig.pwm->fail_pulse = 1650;  // A cannot be written
  }
  ASSERT_TRUE(rig.wait_state("unavailable"));
  const auto w = rig.writes();
  EXPECT_EQ(w.back(), Write(0, false)) << "switched off rather than left at B";
  const std::string s = *rig.act->status_json();
  EXPECT_NE(s.find("\"powered\":false"), std::string::npos) << s;
  EXPECT_NE(s.find("the output is off"), std::string::npos) << s;
  EXPECT_NE(rig.act->fire().find("unavailable"), std::string::npos);
  // A failed write is no hazard: the channel is tried again, and comes back when it works.
  {
    std::lock_guard lk(rig.pwm->m);
    rig.pwm->fail_pulse = -1;
  }
  ASSERT_TRUE(rig.wait_state("off"));
  EXPECT_EQ(rig.act->fire(), "");
}

// Review of 2026-10-30: a first calibration closed during its test shot went limp at B.
TEST(Actuator, AFirstCalibrationClosedMidTestReturnsToTheTestsRest) {
  Rig rig(ActuatorConfig{});
  ASSERT_TRUE(rig.wait_writes(1));
  ASSERT_EQ(rig.act->begin_calibration(1600), "");
  ASSERT_TRUE(rig.wait_writes(2));
  ASSERT_EQ(rig.act->test(1600, 1400, 3000), "");
  ASSERT_TRUE(rig.wait_writes(3));  // off, 1600, 1400
  ASSERT_EQ(rig.act->end_calibration(), "");
  ASSERT_TRUE(rig.wait_state("unconfigured"));
  const auto w = rig.writes();
  ASSERT_GE(w.size(), 5u);
  EXPECT_EQ(w[w.size() - 2], Write(1600, true));
  EXPECT_EQ(w.back(), Write(0, false));
}

TEST(Actuator, NoChannelIsUnavailableWithTheReason) {
  Actuator a(nullptr, "no PWM chip at 1f00098000: the pwm overlay is not loaded", "", 15, owner_config(), "");
  EXPECT_EQ(a.state_name(), "unavailable");
  EXPECT_NE(a.fire().find("overlay is not loaded"), std::string::npos);
  EXPECT_NE(a.status_json()->find("\"available\":false"), std::string::npos);
}

TEST(Actuator, AFileForAnotherPinIsNotUsed) {
  ActuatorConfig c = owner_config();
  c.gpio = 14;
  Rig rig(c);
  EXPECT_EQ(rig.act->state_name(), "unconfigured");
  EXPECT_NE(rig.act->fire().find("GPIO14"), std::string::npos);
}

// A sysfs tree as the station has it: class/pwm/pwmchipN links into the device's directory, whose path
// carries the block's address. The fan's chip (another address) must not be taken.
TEST(SysfsPwm, TheChannelIsFoundByAddressAndWrittenPeriodFirst) {
  namespace fs = std::filesystem;
  const auto root = scratch_dir("sysfs");
  const fs::path fan = root / "devices/1f0009c000.pwm/pwm/pwmchip0";
  const fs::path pwm0 = root / "devices/1f00098000.pwm/pwm/pwmchip1";
  for (const auto& d : {fan, pwm0 / "pwm3"}) fs::create_directories(d);
  for (const char* f : {"period", "duty_cycle", "enable"}) std::ofstream(pwm0 / "pwm3" / f) << "0\n";
  std::ofstream(pwm0 / "export") << "";
  fs::create_directories(root / "class/pwm");
  fs::create_directory_symlink(fan, root / "class/pwm/pwmchip0");
  fs::create_directory_symlink(pwm0, root / "class/pwm/pwmchip1");
  auto read = [&](const char* f) {
    std::ifstream in(pwm0 / "pwm3" / f);
    std::string s;
    std::getline(in, s);
    return s;
  };
  std::string err;
  auto ch = SysfsPwm::open((root / "class/pwm").string(), 15, 20000, err);
  ASSERT_TRUE(ch) << err;
  EXPECT_EQ(read("period"), "20000000");
  ASSERT_TRUE(ch->set(1650, true, err)) << err;
  EXPECT_EQ(read("duty_cycle"), "1650000");
  EXPECT_EQ(read("enable"), "1");
  ASSERT_TRUE(ch->set(0, false, err)) << err;
  EXPECT_EQ(read("enable"), "0");
  EXPECT_FALSE(SysfsPwm::open((root / "class/pwm").string(), 4, 20000, err));
  EXPECT_NE(err.find("no PWM0 channel"), std::string::npos);
  fs::remove(root / "class/pwm/pwmchip1");
  EXPECT_FALSE(SysfsPwm::open((root / "class/pwm").string(), 15, 20000, err));
  EXPECT_NE(err.find("overlay is not loaded"), std::string::npos);
  fs::remove_all(root);
}

TEST(ActuatorCommands, TheGateChecksShapesAndAllowsEverythingButAFaultUnhomed) {
  web::SystemCommandState s;  // not homed: Shutdown
  for (const auto& [cmd, arg] : std::vector<std::pair<std::string, std::string>>{
           {"actuate", ""}, {"actuator_calibration", "start"}, {"actuator_calibration", "start 1500"},
           {"actuator_calibration", "stop"}, {"actuator_pulse", "1520"}, {"actuator_test", "1650 1450 300"},
           {"set_actuator", "1650 1450 300"}})
    EXPECT_TRUE(web::validate_command(s, cmd, arg).ok) << cmd << " " << arg;
  for (const auto& [cmd, arg] : std::vector<std::pair<std::string, std::string>>{
           {"actuate", "now"}, {"actuator_calibration", ""}, {"actuator_calibration", "start x"},
           {"actuator_calibration", "stop 1"}, {"actuator_pulse", ""}, {"actuator_pulse", "1500 1"},
           {"actuator_test", "1650 1450"}, {"set_actuator", "a b c"}})
    EXPECT_FALSE(web::validate_command(s, cmd, arg).ok) << cmd << " " << arg;
  s.fault = true;
  EXPECT_FALSE(web::validate_command(s, "actuate", "").ok);
  EXPECT_FALSE(web::validate_command(s, "actuator_pulse", "1500").ok);
  EXPECT_TRUE(web::validate_command(s, "actuator_calibration", "stop").ok);
  s.fault = false;
  s.shutdown_or_parking = true;  // the process's exit stop
  EXPECT_FALSE(web::validate_command(s, "actuate", "").ok);
  // Closing is never refused (review of 2026-10-30: the exit stop and motor recovery refused it).
  EXPECT_TRUE(web::validate_command(s, "actuator_calibration", " stop ").ok);
  s.shutdown_or_parking = false;
  s.motor_recovery_active = true;
  EXPECT_TRUE(web::validate_command(s, "actuator_calibration", "stop").ok);
}

TEST(ActuatorCommands, EveryNumberIsTheWholeOfItsWordAndInRange) {
  web::SystemCommandState s;
  // Words, not stream positions: a trailing space is no different from none.
  EXPECT_TRUE(web::validate_command(s, "actuator_calibration", "start ").ok);
  EXPECT_TRUE(web::validate_command(s, "actuator_calibration", "start 1500 ").ok);
  // A number too large for an int used to pass the gate as a long and open the session at the default.
  EXPECT_FALSE(web::validate_command(s, "actuator_calibration", "start 99999999999").ok);
  EXPECT_FALSE(web::validate_command(s, "actuator_pulse", "1500us").ok);
  EXPECT_FALSE(web::validate_command(s, "actuator_pulse", "-5").ok);
  EXPECT_FALSE(web::validate_command(s, "set_actuator", "1650 1450 nan").ok);
  web::ActuatorArgs a;
  ASSERT_EQ(web::parse_actuator_args("actuator_test", "1650 1450 300.5", a), "");
  EXPECT_EQ(a.a, 1650);
  EXPECT_EQ(a.b, 1450);
  EXPECT_DOUBLE_EQ(a.dwell, 300.5);
}

}  // namespace
}  // namespace ota
