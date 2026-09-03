// OpenAutoTurret — ControlLoop tests (§46 per-cycle engine + safety).
//
// Driven against the SimMotorBackend (a first-order plant with end stops and a
// stall effort), so the whole safety path runs with no CAN:
//   * the Phase-2 deliverable: boot -> homed -> safe hold -> park cycle;
//   * stale feedback -> Brake (a recoverable safe stop, NOT a fault);
//   * motor hard fault -> Disable (de-energize, fault-locked).
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "calibration/homing_plan.hpp"
#include "control/control_loop.hpp"
#include "sim/sim_motor_backend.hpp"

#include <gtest/gtest.h>

using namespace ota;

namespace {

constexpr int64_t kDtNs = 5'000'000;  // 200 Hz
constexpr int kMaxSteps = 60000;      // 300 s of sim

// A homing plan that full-homes both axes (measures/validates both endpoints).
HomingPlan make_plan() {
  HomingPlanConfig hcfg;
  HomingParams hp;
  hp.coarse_speed_rad_s = 20.0 * kDeg2Rad;
  hp.fine_speed_rad_s = 2.0 * kDeg2Rad;
  hp.settle_time_s = 0.3;
  hcfg.homing = hp;
  hcfg.travel_bands[0] = TravelBand{0.0, 115.0};  // pitch
  hcfg.travel_bands[1] = TravelBand{0.0, 115.0};  // yaw
  std::vector<HomingAction> actions;
  actions.push_back(HomingAction{.type = HomingActionType::HomeFullRange,
                                 .axis = AxisId::Pitch});
  actions.push_back(HomingAction{.type = HomingActionType::HomeFullRange,
                                 .axis = AxisId::Yaw});
  return HomingPlan(std::move(actions), hcfg);
}

ControlLoop::Config make_cfg() {
  ControlLoop::Config cfg;
  cfg.control_hz = 200;
  cfg.hold_speed_rad_s = 30.0 * kDeg2Rad;
  cfg.emergency_speed_rad_s = 10.0 * kDeg2Rad;
  cfg.soft_margin_rad = 2.0 * kDeg2Rad;
  cfg.park.park_logical_deg = {30.0, 60.0};  // pitch 30 deg, yaw 60 deg
  cfg.park.speed_deg_s = 50.0;
  cfg.park.dwell_ms = 300;
  cfg.park.pos_tol_deg = 0.5;
  cfg.park.vel_tol_deg_s = 1.0;
  cfg.park.min_soft_margin_deg = 2.0;
  return cfg;
}

// Home both axes, then move to the safe ready pose (the §27 MOVE_TO_READY /
// HOLD_POSE transition). Returns the sim time after the ready pose is reached.
TEST(ControlLoopSim, SimulatedBackendClaimsNoCanLink) {
  // The interface default is the whole guarantee: a simulated plant must never
  // report a healthy bus, because every §55 CAN metric read from a sim run
  // would otherwise look like evidence about hardware.
  sim::SimMotorBackend backend(0.005);
  const CanHealth h = backend.can_health();
  EXPECT_FALSE(h.available);
  EXPECT_EQ(h.state, -1);
  EXPECT_EQ(h.rx_frames, 0u);
  EXPECT_TRUE(h.kind.empty());
}

bool run_to_ready(ControlLoop& loop, sim::SimMotorBackend& sim, int64_t& t_out) {
  int64_t t = 0;
  for (int i = 0; i < kMaxSteps; ++i) {
    loop.step(t, kDtNs);
    t += kDtNs;
    if (loop.phase() == Phase::Fault) return false;
    if (loop.homed() && loop.at_ready()) {
      t_out = t;
      return true;
    }
  }
  t_out = t;
  return false;
}

}  // namespace

// The Phase-2 deliverable: reliable boot -> homed -> safe hold -> park cycle.
TEST(ControlLoop, FullCycle_HomeHoldPark) {
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  sim::SimMotorBackend* sim = backend.get();
  sim->set_stops(AxisId::Pitch, -1.0, 1.0);
  sim->set_stops(AxisId::Yaw, -1.0, 1.0);
  sim->set_position(AxisId::Pitch, 0.5);
  sim->set_position(AxisId::Yaw, -0.3);

  ControlLoop loop(make_cfg(), std::move(backend));

  std::string err;
  ASSERT_TRUE(loop.start_homing(make_plan(), err)) << err;

  int64_t t = 0;
  ASSERT_TRUE(run_to_ready(loop, *sim, t))
      << "did not reach ready; phase=" << phase_name(loop.phase())
      << " fault=" << loop.fault_reason();
  EXPECT_EQ(loop.phase(), Phase::Hold);
  EXPECT_TRUE(loop.homed());
  // Both axes have valid limits after homing.
  EXPECT_TRUE(loop.limits()[static_cast<int>(AxisId::Pitch)].valid);
  EXPECT_TRUE(loop.limits()[static_cast<int>(AxisId::Yaw)].valid);

  // Park: move to the park pose, verify, dwell, de-energize pitch then yaw.
  ASSERT_TRUE(loop.start_parking(err)) << err;
  for (int i = 0; i < kMaxSteps && loop.phase() != Phase::Parked; ++i) {
    loop.step(t, kDtNs);
    t += kDtNs;
  }
  EXPECT_EQ(loop.phase(), Phase::Parked)
      << "park did not complete; fault=" << loop.fault_reason();
  // Both axes end de-energized (held by friction), at the park pose.
  EXPECT_FALSE(sim->in_position_mode(AxisId::Pitch));
  EXPECT_FALSE(sim->in_position_mode(AxisId::Yaw));
}

// Stale feedback -> Brake: a recoverable safe stop, NOT a fault. The axis is
// commanded to stop (not to keep moving) and recovers when feedback returns.
TEST(ControlLoop, StaleFeedbackTriggersBrake) {
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  sim::SimMotorBackend* sim = backend.get();
  sim->set_stops(AxisId::Pitch, -1.0, 1.0);
  sim->set_stops(AxisId::Yaw, -1.0, 1.0);
  sim->set_position(AxisId::Pitch, 0.5);
  sim->set_position(AxisId::Yaw, -0.3);

  ControlLoop loop(make_cfg(), std::move(backend));
  std::string err;
  ASSERT_TRUE(loop.start_homing(make_plan(), err)) << err;
  int64_t t = 0;
  ASSERT_TRUE(run_to_ready(loop, *sim, t)) << loop.fault_reason();
  EXPECT_EQ(loop.last_decision().action, SafetyAction::Allow);

  // Drop the pitch feedback: the supervisor must Brake (safe stop), and this is
  // recoverable (not a fault) once the feedback returns.
  sim->set_feedback_ok(AxisId::Pitch, false);
  loop.step(t, kDtNs);
  t += kDtNs;
  EXPECT_EQ(loop.last_decision().action, SafetyAction::Brake);
  EXPECT_NE(loop.phase(), Phase::Fault) << "stale feedback is not a hard fault";

  // Feedback returns: back to Allow (recovered, no manual reset needed).
  sim->set_feedback_ok(AxisId::Pitch, true);
  loop.step(t, kDtNs);
  t += kDtNs;
  EXPECT_EQ(loop.last_decision().action, SafetyAction::Allow);
  EXPECT_EQ(loop.phase(), Phase::Hold);
}

// Motor hard fault -> Disable: de-energize the axes and fault-lock (sticky).
TEST(ControlLoop, MotorFaultTriggersDisable) {
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  sim::SimMotorBackend* sim = backend.get();
  sim->set_stops(AxisId::Pitch, -1.0, 1.0);
  sim->set_stops(AxisId::Yaw, -1.0, 1.0);
  sim->set_position(AxisId::Pitch, 0.5);
  sim->set_position(AxisId::Yaw, -0.3);

  ControlLoop loop(make_cfg(), std::move(backend));
  std::string err;
  ASSERT_TRUE(loop.start_homing(make_plan(), err)) << err;
  int64_t t = 0;
  ASSERT_TRUE(run_to_ready(loop, *sim, t)) << loop.fault_reason();

  // A hard fault on yaw: the supervisor must Disable (de-energize) and the
  // loop fault-locks (sticky; it does not auto-recover).
  sim->set_faults(AxisId::Yaw, 0x1);
  loop.step(t, kDtNs);
  t += kDtNs;
  EXPECT_EQ(loop.last_decision().action, SafetyAction::Disable);
  EXPECT_EQ(loop.phase(), Phase::Fault);
  EXPECT_FALSE(sim->in_position_mode(AxisId::Yaw));
  EXPECT_FALSE(sim->in_position_mode(AxisId::Pitch));

  // Faults are sticky: even with the fault cleared, the loop stays in Fault.
  sim->set_faults(AxisId::Yaw, 0x0);
  loop.step(t, kDtNs);
  t += kDtNs;
  EXPECT_EQ(loop.phase(), Phase::Fault);
}

// --- v3 §52: every command answers, and "accepted" has to be earned -------
//
// These tests exist because v1 answered ok:true for commands it then dropped on
// the floor (enable_search was one; select_target, start_homing and
// start_installation_calibration still are, and now say so). A mode that refuses
// must say why, and the reason has to survive into the snapshot the web layer
// reads — a refusal that only reaches a log file is a refusal the operator cannot
// act on.
namespace {
struct HomedLoop {
  std::unique_ptr<sim::SimMotorBackend> sim_owner;
  sim::SimMotorBackend* sim = nullptr;
  std::unique_ptr<ControlLoop> loop;
  int64_t t = 0;
  bool ready = false;

  HomedLoop(bool home = true) {
    sim_owner = std::make_unique<sim::SimMotorBackend>(0.005);
    sim = sim_owner.get();
    sim->set_stops(AxisId::Pitch, -1.0, 1.0);
    sim->set_stops(AxisId::Yaw, -1.0, 1.0);
    loop = std::make_unique<ControlLoop>(make_cfg(), std::move(sim_owner));
    if (home) {
      std::string err;
      (void)loop->start_homing(make_plan(), err);
      ready = run_to_ready(*loop, *sim, t);
    }
  }
  void step(int n) {
    for (int i = 0; i < n; ++i) {
      loop->step(t, kDtNs);
      t += kDtNs;
    }
  }
  void run(const char* name, const char* arg = "") {
    loop->submit_command(name, arg);
    step(3);
  }
  // The published snapshot — what the web layer reads — not the publisher
  // object it is held in (§6.3). Testing the publisher would prove nothing about
  // what an operator can actually see.
  telemetry::TelemetrySnapshot snap() { return loop->telemetry().snapshot(); }
};
}  // namespace

TEST(CommandAck, NothingClaimedBeforeTheFirstCommand) {
  HomedLoop h;
  ASSERT_TRUE(h.ready) << h.loop->fault_reason();
  EXPECT_EQ(h.snap().cmd_ack_accepted, -1)
      << "no command yet must not be rendered as a success, or as a failure";
  EXPECT_TRUE(h.snap().cmd_ack_command.empty());
}

TEST(CommandAck, RejectedModeChangeCarriesTheReasonToTheSnapshot) {
  HomedLoop h(/*home=*/false);
  ASSERT_FALSE(h.loop->homed());
  h.run("set_mode", "AUTO_TRACK");
  EXPECT_EQ(h.loop->operating_mode(), OperatingMode::Manual)
      << "an unhomed station cannot enter autonomy";
  EXPECT_EQ(h.loop->last_command_ack().accepted, false);
  EXPECT_NE(h.loop->last_command_ack().reason.find("hom"), std::string::npos)
      << "the reason must say what is missing, got: "
      << h.loop->last_command_ack().reason;
  // The same text the browser shows (§52), not a different string assembled by
  // the web layer from a guess.
  EXPECT_EQ(h.snap().cmd_ack_reason, h.loop->last_command_ack().reason);
  EXPECT_EQ(h.snap().cmd_ack_accepted, 0);
}

TEST(CommandAck, AcceptedModeChangeSaysSoAndMovesTheMode) {
  HomedLoop h;
  ASSERT_TRUE(h.ready) << h.loop->fault_reason();
  h.run("set_mode", "AUTO_TRACK");
  EXPECT_EQ(h.loop->operating_mode(), OperatingMode::AutoTrack);
  EXPECT_TRUE(h.loop->last_command_ack().accepted);
  EXPECT_NE(h.loop->last_command_ack().reason.find("entered AUTO_TRACK"),
            std::string::npos)
      << h.loop->last_command_ack().reason;
  EXPECT_EQ(h.snap().operating_mode, "AUTO_TRACK");
  // The ack names the state it was decided in, so a report can be read later
  // without guessing what the turret was doing at the time.
  EXPECT_NE(h.loop->last_command_ack().controller_state.find("hold"),
            std::string::npos)
      << h.loop->last_command_ack().controller_state;
}

TEST(CommandAck, ModeNameThatDoesNotExistIsRefusedNotIgnored) {
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "SEARCH");
  EXPECT_EQ(h.loop->operating_mode(), OperatingMode::Manual);
  EXPECT_FALSE(h.loop->last_command_ack().accepted);
  EXPECT_NE(h.loop->last_command_ack().reason.find("MANUAL"), std::string::npos)
      << "the refusal must name the accepted spellings: "
      << h.loop->last_command_ack().reason;
}

TEST(CommandAck, UnknownCommandIsAnswered) {
  HomedLoop h;
  h.run("launch_the_rocket");
  EXPECT_FALSE(h.loop->last_command_ack().accepted);
  EXPECT_NE(h.loop->last_command_ack().reason.find("unknown command"),
            std::string::npos)
      << h.loop->last_command_ack().reason;
}

TEST(CommandAck, CommandsThatDoNothingSayTheyDidNothing) {
  // The three that v1 acked and dropped. These stay RED-shaped on purpose: when
  // V3-3 implements selection, this test is the place that has to change, and it
  // will change because the feature exists rather than because nobody noticed.
  HomedLoop h;
  h.run("select_target", "3");
  EXPECT_FALSE(h.loop->last_command_ack().accepted)
      << "selection has no implementation behind it yet";
  h.run("start_homing");
  EXPECT_FALSE(h.loop->last_command_ack().accepted);
  h.run("start_installation_calibration");
  EXPECT_FALSE(h.loop->last_command_ack().accepted);
}

TEST(CommandAck, StopMotionAnswersAndWorksFromAnyMode) {
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "AUTO_ROAM");
  ASSERT_EQ(h.loop->operating_mode(), OperatingMode::AutoRoam);
  h.run("stop_motion");
  EXPECT_TRUE(h.loop->last_command_ack().accepted);
  EXPECT_EQ(h.loop->operating_mode(), OperatingMode::Manual);
  EXPECT_NE(h.loop->last_command_ack().reason.find("AUTO_ROAM"), std::string::npos)
      << "it should say what it cancelled: " << h.loop->last_command_ack().reason;
}

TEST(TelemetryV3, ModeAndIntentArePublishedEveryCycle) {
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  EXPECT_EQ(h.snap().operating_mode, "MANUAL");
  // This used to assert "none", and that assertion was recording a gap rather than a
  // design: MANUAL had no controller, so no intent was produced at all and the field
  // was empty. V3-5 put a ManualController there, and the honest reading of a turret
  // that is idle in MANUAL is "manual, holding" — the mode owns motion and has decided
  // not to move. "none" would now mean something specific and different: that no mode
  // controller ran this cycle, which is a fault condition, not an idle one.
  EXPECT_EQ(h.snap().intent_source, "manual");
  EXPECT_EQ(h.snap().intent_type, "hold");
  EXPECT_STREQ(h.snap().intent_reason.c_str(), "manual hold");
  h.run("set_mode", "AUTO_TRACK");
  EXPECT_EQ(h.snap().intent_source, "auto_track");
  EXPECT_EQ(h.snap().intent_type, "hold")
      << "AUTO_TRACK with no target ever seen is a hold (§111.5), and the "
         "telemetry has to show that rather than leave it implied";
  EXPECT_EQ(h.snap().mode_phase, "WAIT_TARGET");
}

// --- v3 §59: controld decides what to follow, and v1's limits still apply --
namespace {
tracks::TrackSet make_set_with(tracks::TrackState state, int32_t class_id,
                               float confidence, float anchor_x = 0.5f) {
  tracks::TrackSet set;
  // Deliberately *not* a fixed timestamp: the caller fills these from the loop's own
  // clock (see feed()), because §11 aligns the motor pose at the capture time. A test
  // that feeds a 700 ms-old capture into a loop that has been simulating for 30 s gets
  // its measurement discarded as stale — and then every "was refused" assertion below
  // passes while proving nothing. Getting this wrong is the most comfortable way to
  // write a green test suite that tests nothing.
  set.frame_sequence = 0;
  set.sensor_timestamp_ns = 0;
  set.publish_timestamp_ns = 0;
  set.width = 1280;
  set.height = 720;
  tracks::Track t;
  t.uuid = tracks::TrackUuid{0xfeed, 3};
  t.display_index = 1;
  t.class_id = static_cast<uint16_t>(class_id);
  std::memcpy(t.class_name, "person", 6);
  t.state = state;
  t.detector_confidence = confidence;
  t.track_confidence = confidence;
  t.bbox.x_min = anchor_x - 0.05f;
  t.bbox.x_max = anchor_x + 0.05f;
  t.bbox.y_min = 0.4f;
  t.bbox.y_max = 0.7f;
  t.anchor_x = anchor_x;
  t.anchor_y = 0.55f;
  t.visible_frames = 9;
  set.add(t);
  return set;
}
}  // namespace

// Feed `n` frames of `set`, keeping every stamp inside the loop's own clock.
void feed(HomedLoop& h, tracks::TrackSet set, int n, uint64_t seq0) {
  for (int i = 0; i < n; ++i) {
    set.frame_sequence = seq0 + i;
    set.sensor_timestamp_ns = h.t;             // "this frame", in loop time
    set.publish_timestamp_ns = h.t + 1'000'000;
    h.loop->feed_track_set(set, h.t);
    h.step(1);
  }
}

TEST(FeedTrackSet, TentativeCandidatesAreNotFollowed) {
  // §8: TENTATIVE is not selectable, and "not selectable" has to mean the turret does
  // not move toward it either. The estimator latching onto a one-frame flicker is how
  // a turret acquires a shadow.
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "AUTO_TRACK");
  ASSERT_EQ(h.loop->operating_mode(), OperatingMode::AutoTrack);
  auto set = make_set_with(tracks::TrackState::Tentative, 1, 0.95f);
  feed(h, set, 5, 10);
  EXPECT_EQ(h.snap().selected_track_id, 0u)
      << "a tentative track became the thing being followed";
}

TEST(FeedTrackSet, AVisibleTargetIsSeenButOnlyASelectionIsFollowed) {
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "AUTO_TRACK");
  ASSERT_TRUE(h.loop->tracking_mode_enabled())
      << "the mode was accepted but no tracking session started: the default "
         "TrackingController::Config in this rig is not commissionable";
  auto set = make_set_with(tracks::TrackState::Confirmed, 1, 0.9f);
  feed(h, set, 4, 20);
  // §16, and the retirement of the interim rule as a source of motion. The estimator
  // IS primed — vision keeps being processed, which is what makes a selection act
  // immediately instead of waiting out a warm-up — but the turret is not following
  // anybody, and `selected_track_id` says so. An earlier version of this test asserted
  // the opposite: that the identity of the best-scoring detection was published as the
  // thing being followed. That field would have been describing a decision nobody made.
  EXPECT_TRUE(h.loop->tracking_controller().estimator_initialized())
      << "vision stopped being processed for want of a selection";
  EXPECT_EQ(h.snap().selected_track_id, 0u);
  EXPECT_EQ(h.snap().mode_phase, "WAIT_TARGET");
  EXPECT_EQ(h.snap().selected_display_index, 0);

  h.run("select_target", "1");
  ASSERT_EQ(h.snap().cmd_ack_accepted, 1) << h.snap().cmd_ack_reason;
  feed(h, set, 3, 24);
  ASSERT_EQ(h.snap().selected_track_id, 3u)
      << "§78: once the operator's choice is what the turret is acting on, the "
         "identity is published rather than argued about";
  EXPECT_EQ(h.snap().selected_display_index, 1);
}

TEST(FeedTrackSet, V1sClassAndConfidenceLimitsSurviveTheMove) {
  // §59 changes WHO selects, not HOW MUCH the station is willing to follow. v1's
  // selector refused anything but 'person' and anything under its confidence
  // threshold; if that restriction quietly widened when the decision moved into
  // controld, the turret would start acquiring cars the day visiond published every
  // class the detector knows. §72 makes both configurable; until then they are
  // carried over explicitly, and this test is what stops the carry-over being
  // "simplified" away.
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "AUTO_TRACK");
  for (auto& pair : std::vector<std::pair<int32_t, float>>{{2, 0.99f}, {1, 0.2f}}) {
    auto set = make_set_with(tracks::TrackState::Confirmed, pair.first, pair.second);
    feed(h, set, 4, 30);
    EXPECT_EQ(h.snap().selected_track_id, 0u)
        << "class " << pair.first << " at confidence " << pair.second
        << " would have been refused by v1's selector; controld must refuse it too";
  }
}

TEST(FeedTrackSet, MissingResolutionIsRefusedRatherThanGuessed) {
  // The §9 anchor is normalized (§60). Turning it back into a pixel needs the frame
  // size it was normalized against; inventing one would produce a confident, wrong
  // line of sight — the worst kind of wrong, because it moves the turret smoothly
  // somewhere that is not where the target is.
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "AUTO_TRACK");
  auto set = make_set_with(tracks::TrackState::Confirmed, 1, 0.9f);
  set.width = 0;
  set.height = 0;
  feed(h, set, 4, 40);
  EXPECT_EQ(h.snap().selected_track_id, 0u);
  EXPECT_EQ(h.loop->phase(), Phase::Hold)
      << "refusing an unreadable message must not fault the station";
}

// --- §88 through the real command API (the software half; the operator half of §88
//     — a person watching a real turret pick the right real person — stays open).
namespace {
tracks::TrackSet two_people(uint32_t seq, TimeNs sensor, uint16_t idx_a, uint16_t idx_b,
                            float conf_a = 0.9f, float conf_b = 0.7f,
                            float ax_a = 0.3f, float ax_b = 0.7f) {
  tracks::TrackSet set;
  set.frame_sequence = seq;
  set.sensor_timestamp_ns = sensor;
  set.publish_timestamp_ns = sensor + 3'000'000;
  set.width = 1280;
  set.height = 720;
  const std::pair<tracks::TrackUuid, std::pair<uint16_t, float>> who[2] = {
      {tracks::TrackUuid{0, 11}, {idx_a, conf_a}},
      {tracks::TrackUuid{0, 22}, {idx_b, conf_b}},
  };
  const float ax[2] = {ax_a, ax_b};
  for (int i = 0; i < 2; ++i) {
    tracks::Track t;
    t.uuid = who[i].first;
    t.display_index = who[i].second.first;
    t.class_id = 1;
    std::memcpy(t.class_name, "person", 6);
    t.state = tracks::TrackState::Confirmed;
    t.detector_confidence = t.track_confidence = who[i].second.second;
    t.bbox.x_min = ax[i] - 0.05f;
    t.bbox.x_max = ax[i] + 0.05f;
    t.bbox.y_min = 0.4f;
    t.bbox.y_max = 0.7f;
    t.anchor_x = ax[i];
    t.anchor_y = 0.55f;
    set.add(t);
  }
  return set;
}
}  // namespace

TEST(TargetSelectionIntegration, ChoosingTwoPeopleSelectingTheSecondFollowsThesecond) {
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  // Both must be CONFIRMED before selection is possible, which needs the set to arrive
  // more than once — but the vision side does the confirming, so controld sees them
  // confirmed from the first frame it is given.
  h.loop->feed_track_set(two_people(1, h.t, 1, 2), h.t);
  h.step(1);

  h.run("select_target", "2");
  auto snap = h.snap();
  EXPECT_EQ(snap.cmd_ack_accepted, 1) << snap.cmd_ack_reason;
  EXPECT_EQ(snap.cmd_ack_reason, "selected Person #2");
  EXPECT_EQ(snap.selected_display_index, 2);
  EXPECT_EQ(snap.selected_track_id, 0u)
      << "the selection fields say what the operator chose and the followed id says "
         "what the turret is acting on; with no tracking session running they "
         "legitimately differ, and a build that reported 22 here would be claiming "
         "motion it is not making";

  h.run("set_mode", "AUTO_TRACK");
  h.loop->feed_track_set(two_people(900, h.t, 1, 2), h.t);
  h.step(2);
  EXPECT_EQ(h.snap().selected_track_id, 22u)
      << "§78: once the mode acts on the selection, the followed identity is published "
         "rather than argued about";
  ASSERT_EQ(h.loop->operating_mode(), OperatingMode::AutoTrack);
  h.loop->feed_track_set(two_people(2, h.t, 1, 2), h.t);
  h.step(2);
  EXPECT_EQ(h.snap().selected_track_id, 22u)
      << "§88 step 5: #2 is the one being followed";

  // §88 step 6: #1 becomes the bigger, better, faster detection. v1 would have moved on
  // it without a word.
  for (int i = 0; i < 10; ++i) {
    h.loop->feed_track_set(two_people(10 + i, h.t, 1, 2, 0.99f, 0.4f,
                                      0.3f + 0.02f * i),
                           h.t);
    h.step(1);
  }
  EXPECT_EQ(h.snap().selected_track_id, 22u)
      << "the strongest detection on screen took the selection away from the one the "
         "operator chose";
  EXPECT_EQ(h.snap().selected_display_index, 2);

  // §12: MANUAL does not forget it, and AUTO_TRACK picks it straight back up.
  h.run("set_mode", "MANUAL");
  EXPECT_EQ(h.snap().selected_display_index, 2)
      << "§12: switching modes must not clear the selection";
  h.run("set_mode", "AUTO_TRACK");
  h.loop->feed_track_set(two_people(30, h.t, 1, 2), h.t);
  h.step(2);
  EXPECT_EQ(h.snap().selected_track_id, 22u);

  // And only the explicit command removes it.
  h.run("clear_target");
  EXPECT_EQ(h.snap().cmd_ack_accepted, 1) << h.snap().cmd_ack_reason;
  EXPECT_EQ(h.snap().cmd_ack_reason, "cleared Person #2");
  h.loop->feed_track_set(two_people(40, h.t, 1, 2), h.t);
  h.step(2);
  EXPECT_EQ(h.snap().selected_display_index, 0)
      << "cleared, yet controld is still following something";
}

TEST(TargetSelectionIntegration, ARefusalReachesTheOperatorWithItsReason) {
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.loop->feed_track_set(two_people(1, h.t, 1, 2), h.t);
  h.step(1);
  h.run("select_target", "7");
  const auto snap = h.snap();
  EXPECT_EQ(snap.cmd_ack_accepted, 0);
  EXPECT_NE(snap.cmd_ack_reason.find("no target # 7"), std::string::npos)
      << snap.cmd_ack_reason
      << " — a refusal the operator cannot act on is indistinguishable from a bug";
  EXPECT_EQ(snap.selected_display_index, 0)
      << "a refused selection must not half-apply";
}

TEST(TargetSelectionIntegration, SelectionSurvivesWithoutTrackingTurnedOn) {
  // The v1 gate refused this outright; §12 makes the refusal the bug. Selecting must
  // not require a tracking session, because "which one" is a question the operator can
  // ask before anything is following anything.
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  ASSERT_FALSE(h.loop->tracking_mode_enabled());
  h.loop->feed_track_set(two_people(1, h.t, 1, 2), h.t);
  h.step(1);
  h.run("select_target", "1");
  EXPECT_EQ(h.snap().cmd_ack_accepted, 1) << h.snap().cmd_ack_reason;
  EXPECT_EQ(h.snap().selected_display_index, 1);
  EXPECT_FALSE(h.loop->tracking_mode_enabled())
      << "§14: selection causes no motion and must not start a tracking session";
}

// --- §38/§52 through the real command path: the mode gate, and a jog that actually
//     moves the simulated turret and then stops by itself.
TEST(ManualMode, MotionOutsideManualIsRefusedWithTheReasonFrom52) {
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "AUTO_TRACK");
  ASSERT_EQ(h.loop->operating_mode(), OperatingMode::AutoTrack);

  h.run("manual_jog_start", "yaw+:normal");
  auto snap = h.snap();
  EXPECT_EQ(snap.cmd_ack_accepted, 0);
  EXPECT_NE(snap.cmd_ack_reason.find("only available in MANUAL"), std::string::npos)
      << snap.cmd_ack_reason;
  EXPECT_NE(snap.cmd_ack_reason.find("AUTO_TRACK"), std::string::npos)
      << "the refusal should say what mode it is refusing in";

  h.run("manual_step", "yaw+1");
  EXPECT_EQ(h.snap().cmd_ack_accepted, 0);

  // And the lease is genuinely not running: the refusal is not cosmetic.
  EXPECT_FALSE(h.snap().manual_lease_active);
}

TEST(ManualMode, ALeasedJogMovesTheTurretAndStopsWhenTheBrowserGoesQuiet) {
  HomedLoop h;
  ASSERT_TRUE(h.ready);                     // MANUAL is the mode after homing
  const double yaw0 = h.loop->last_positions()[1];

  h.run("manual_jog_start", "yaw+:normal");
  ASSERT_EQ(h.snap().cmd_ack_accepted, 1) << h.snap().cmd_ack_reason;
  ASSERT_TRUE(h.snap().manual_lease_active);

  // Keep it alive the way a browser would: every 100 ms, which at 200 Hz is every 20
  // cycles. Not more often — a test that renews every cycle would never notice a lease
  // that was shorter than it should be.
  for (int i = 0; i < 400; ++i) {
    if (i % 20 == 0) h.loop->submit_command("manual_jog_keepalive", "");
    h.step(1);
  }
  const double yaw_jogged = h.loop->last_positions()[1];
  // Two seconds at the profile's rate is a lot of motion, so the bar is high on
  // purpose. The version of this test that asked only "did it move" passed against a
  // jog that moved 2 degrees and stopped, because a broken integration still produces
  // one step of motion; the assertion was satisfied by the symptom of the bug.
  EXPECT_GT(yaw_jogged, yaw0 + 0.15)
      << "two seconds of a live jog lease moved the yaw only "
      << (yaw_jogged - yaw0) / 0.0174533 << " deg";
  EXPECT_GT(yaw_jogged - yaw0, 0.9 * 0.45 * 0.35 * 2.0 * 0.5)
      << "the jog is not sustaining motion; it moved once and parked";
  EXPECT_EQ(h.snap().intent_type, "joint_position")
      << "a jog must reach the reference as an integrated position, not a velocity the "
         "drive cannot follow (25)";

  // Silence. The tab is closed; nothing arrives.
  h.step(80);  // 400 ms, past the 300 ms lease
  EXPECT_FALSE(h.snap().manual_lease_active);
  EXPECT_EQ(h.snap().intent_source, "manual");
  EXPECT_EQ(h.snap().intent_type, "hold");
  const double yaw_after = h.loop->last_positions()[1];
  h.step(60);  // 300 ms more
  EXPECT_NEAR(h.loop->last_positions()[1], yaw_after, 2e-3)
      << "the lease expired and the turret kept going";
}

TEST(ManualMode, AStepIsAFiniteMoveThatStops) {
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  const double yaw0 = h.loop->last_positions()[1];
  h.run("manual_step", "yaw+1");
  ASSERT_EQ(h.snap().cmd_ack_accepted, 1) << h.snap().cmd_ack_reason;
  h.step(400);  // 2 s
  const double moved = h.loop->last_positions()[1] - yaw0;
  EXPECT_GT(moved, 1.0 * 0.017453292519943295 * 0.5)
      << "the step did not get close to one degree";
  EXPECT_LT(moved, 1.0 * 0.017453292519943295 * 2.0)
      << "the step overshot by more than it moved";
  h.step(200);
  EXPECT_NEAR(h.loop->last_positions()[1] - yaw0, moved, 1e-3)
      << "a finite move that keeps going is not a step";
}

TEST(ManualMode, OnlyTheSanctionedStepSizesAreAccepted) {
  // §41: 0.5 / 1 / 5 degrees, and no raw-radian move on the operator page. The reason
  // this is a refusal rather than a clamp is that a mistyped digit on an operator page
  // should fail loudly; clamping a 50 into a 5 would move the turret and hide the typo.
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("manual_step", "yaw+3");
  EXPECT_EQ(h.snap().cmd_ack_accepted, 0);
  EXPECT_NE(h.snap().cmd_ack_reason.find("0.5, 1, or 5"), std::string::npos)
      << h.snap().cmd_ack_reason;
  h.run("manual_step", "roll+1");
  EXPECT_EQ(h.snap().cmd_ack_accepted, 0);
  h.run("manual_step", "yaw+1");
  EXPECT_EQ(h.snap().cmd_ack_accepted, 1);
}

// --- §35/§86 through the loop: the sweep is the mode's, and STOP MOTION ends it.
TEST(RoamMode, AutoRoamSweepsAndReportsWhereItIsGoing) {
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  const double yaw0 = h.loop->last_positions()[1];
  h.run("set_mode", "AUTO_ROAM");
  ASSERT_EQ(h.snap().cmd_ack_accepted, 1) << h.snap().cmd_ack_reason;

  h.step(400);  // 2 s
  auto snap = h.snap();
  EXPECT_EQ(snap.operating_mode, "AUTO_ROAM");
  EXPECT_EQ(snap.intent_source, "auto_roam");
  EXPECT_TRUE(snap.mode_phase == "SWEEP" || snap.mode_phase == "MOVE_TO_SCAN_START" ||
              snap.mode_phase == "TURNAROUND")
      << "§35's states are the operator's vocabulary for what the sweep is doing; got "
      << snap.mode_phase;
  EXPECT_NE(snap.roam_sweep_direction, 0)
      << "which way it is running is the one thing a person needs in order to step out "
         "of its path with confidence";
  EXPECT_GT(std::fabs(h.loop->last_positions()[1] - yaw0), 0.05)
      << "two seconds in AUTO_ROAM and it never moved";
  // §32: the waypoint it is driving to is inside the region, never at a stop.
  EXPECT_LT(std::fabs(snap.roam_target_yaw_rad), 3.2)
      << "a waypoint at " << snap.roam_target_yaw_rad << " rad is not inside anything";
}

TEST(RoamMode, StopMotionEndsTheSweepAndLeavesItInManualHold) {
  // §35: "any operator STOP -> MANUAL/HOLD". The sweep is the most autonomous thing this
  // machine does, so this is the button whose behaviour matters most — and the part that
  // is easy to get wrong is the *planner* staying armed afterwards, so the next glance
  // says SWEEP while the turret sits still.
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "AUTO_ROAM");
  h.step(400);
  ASSERT_NE(h.snap().roam_sweep_direction, 0);

  h.run("stop_motion", "");
  EXPECT_EQ(h.snap().cmd_ack_accepted, 1);
  EXPECT_EQ(h.snap().operating_mode, "MANUAL");
  h.step(40);
  EXPECT_EQ(h.snap().roam_sweep_direction, 0)
      << "the mode says MANUAL and the planner still reports a direction";
  EXPECT_EQ(h.snap().intent_source, "manual");
  EXPECT_EQ(h.snap().intent_type, "hold");
  const double yaw = h.loop->last_positions()[1];
  h.step(200);  // a second of stopped-but-armed
  EXPECT_NEAR(h.loop->last_positions()[1], yaw, 2e-3)
      << "it stopped, and then remembered it was supposed to be sweeping";
}

TEST(RoamMode, AnUnhomedStationRefusesToRoamAndSaysWhy) {
  // §32 through §52: without homing the roam region is not *knowable* — the soft limits
  // are relative to the homed pose. The old behaviour would have been to sweep anyway
  // against limits that do not exist yet, which is how a turret learns where the stops
  // are.
  auto backend = std::make_unique<sim::SimMotorBackend>(0.005);
  ControlLoop loop(make_cfg(), std::move(backend));  // fresh: never homed
  ASSERT_FALSE(loop.homed());
  auto r = loop.request_mode(OperatingMode::AutoRoam);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(std::string(r.reason).find("homed"), std::string::npos) << r.reason;
}

// --- §21/§89: an ambiguous reacquisition must ask, not choose. The safety content of
// this is small to state and expensive to get wrong: two people step into view near
// where the selected one was last seen, and a turret that follows "whichever looks more
// like it" has just decided, on its own, to point a gimbal at a stranger.
TEST(AmbiguousReacquisition, TwoEquallyPlausiblePeopleAreNotFollowed) {
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "AUTO_TRACK");
  ASSERT_EQ(h.loop->operating_mode(), OperatingMode::AutoTrack);

  // One person, named by the operator, tracked.
  auto set = two_people(1, h.t, 1, 2, 0.90f, 0.40f, 0.30f, 0.85f);
  set.tracks[1].state = tracks::TrackState::Tentative;  // #2 is not a candidate yet
  feed(h, set, 40, 10);
  h.run("select_target", "1");
  ASSERT_EQ(h.snap().cmd_ack_accepted, 1) << h.snap().cmd_ack_reason;
  // The label is the field under test here (§78: what the operator chose). The
  // identity field additionally requires an armed tracking session, which this rig does
  // not configure, and §89 is about ambiguity rather than about that plumbing.
  ASSERT_EQ(h.snap().selected_display_index, 1) << "the operator's label did not take";
  feed(h, set, 60, 60);
  ASSERT_STREQ(h.snap().mode_phase.c_str(), "TRACKING") << h.snap().mode_phase;
  const double yaw_tracking = h.loop->last_positions()[1];

  // Vision stops entirely: 2.5 s of nothing, long past coast (300 ms) and into
  // LOST_HOLD (2 s).
  h.step(500);
  EXPECT_STREQ(h.snap().mode_phase.c_str(), "LOST_HOLD") << h.snap().mode_phase;
  const double yaw_lost = h.loop->last_positions()[1];

  // Two people step into frame near where #1 vanished, carrying *new* identities: after
  // 2.5 s the TrackManager's own association has expired the old one, so controld cannot
  // recognise the person by uuid and can only decide by geometry and confidence (§21).
  //
  // What this scene actually measures, stated because it is narrower than the heading:
  // the reacquisition score decays with the gap, so after 2.5 s both candidates fall
  // below the accept threshold. The machine holds because *nothing here is plausible*,
  // which is the outcome §111.5 asks for and the one an operator sees — but it is not
  // the tie deciding. The tie is the Events test below, where the memory is still warm;
  // the two are different refusals and they belong in different tests.
  auto both = two_people(200, h.t, 1, 2, 0.88f, 0.86f, 0.48f, 0.52f);
  both.tracks[0].uuid = tracks::TrackUuid{0, 33};  // not the person that was chosen
  both.tracks[1].uuid = tracks::TrackUuid{0, 44};
  feed(h, both, 60, 200);

  EXPECT_STREQ(h.snap().mode_phase.c_str(), "LOST_HOLD")
      << "§21: neither candidate is clearly the chosen person, so the machine holds and "
         "asks. It went to " << h.snap().mode_phase << " instead, which means it "
      << "decided to point the gimbal at a stranger on the strength of a tie.";
  EXPECT_EQ(h.snap().intent_type, "hold")
      << "§16/§21: an unresolved identity is not authority to move";
  EXPECT_NEAR(h.loop->last_positions()[1], yaw_lost, 2e-3)
      << "the turret moved " << (h.loop->last_positions()[1] - yaw_lost) / 0.0174533
      << " deg toward one of two equally plausible people";
}

TEST(AmbiguousReacquisition, OneClearCandidateIsFollowedAgainWithoutAClick) {
  // §20.3: a *confident* reacquisition needs no operator click — that is what keeps a
  // three-second occlusion from becoming a session the operator has to restart. It is
  // the same code path as the test above, and the difference between them is the whole
  // content of §21: the margin, not the visibility.
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "AUTO_TRACK");
  auto set = two_people(1, h.t, 1, 2, 0.90f, 0.40f, 0.30f, 0.85f);
  set.tracks[1].state = tracks::TrackState::Tentative;
  feed(h, set, 40, 10);
  h.run("select_target", "1");
  ASSERT_EQ(h.snap().selected_display_index, 1);
  feed(h, set, 60, 60);
  ASSERT_STREQ(h.snap().mode_phase.c_str(), "TRACKING");

  h.step(500);  // lost, into LOST_HOLD
  ASSERT_STREQ(h.snap().mode_phase.c_str(), "LOST_HOLD");

  // One person back, clearly: the same uuid, the same label, the far stronger score.
  //
  // Stated plainly, because the shape of this case is doing work: the uuid is
  // re-emitted, which is what a TrackManager that kept the track alive would send. The
  // other shape — a gap long enough for the vision side to expire it, so a confident
  // single candidate arrives under a NEW identity — is covered by the scorer's unit
  // tests, not here, and this test does not claim otherwise. The test above is the
  // new-identity case, and there the machine refuses to choose between a tie; that is
  // the asymmetry §21 asks for, and it is the reason these two sit next to each other.
  auto back = two_people(200, h.t, 1, 2, 0.95f, 0.30f, 0.48f, 0.90f);
  back.tracks[1].state = tracks::TrackState::Tentative;  // the other is not a candidate
  feed(h, back, 60, 200);

  EXPECT_FALSE(h.snap().selection_ambiguous);
  EXPECT_TRUE(h.snap().mode_phase == "REACQUIRE" || h.snap().mode_phase == "TRACKING")
      << "a confident single reacquisition should resume on its own, got "
      << h.snap().mode_phase;
  EXPECT_EQ(h.snap().selected_display_index, 1)
      << "and it must be the person that was chosen, not a new identity";
}

// --- §11/§78: the candidate list itself. A count is not a choice: the operator has to
//     be able to see who is in view, what the machine calls them, and which one is
//     already named — and the list has to be the same one the selection was validated
//     against, or the page offers labels controld will refuse.
TEST(CandidateList, BothPeopleAreListedAndNamingOneMarksIt) {
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "AUTO_TRACK");
  auto set = two_people(1, h.t, 1, 2, 0.90f, 0.70f, 0.30f, 0.70f);
  feed(h, set, 10, 10);

  auto snap = h.snap();
  ASSERT_EQ(snap.track_count, 2)
      << "two people were published and the operator's list shows "
      << snap.track_count;
  EXPECT_EQ(snap.tracks[0].display_index, 1);
  EXPECT_EQ(snap.tracks[1].display_index, 2);
  EXPECT_STREQ(snap.tracks[0].label, "Person #1");
  EXPECT_STREQ(snap.tracks[1].label, "Person #2");
  EXPECT_STREQ(snap.tracks[0].state, "CONFIRMED");
  EXPECT_TRUE(snap.tracks[0].selectable);
  EXPECT_NE(snap.tracks[0].anchor_x, 0.0f)
      << "no anchor means the overlay cannot draw a box around the thing it is listing";
  EXPECT_NE(snap.tracks[0].uuid_lo, snap.tracks[1].uuid_lo);
  EXPECT_EQ(snap.tracks[0].selected, false)
      << "nobody has chosen anybody yet";

  h.run("select_target", "2");
  ASSERT_EQ(h.snap().cmd_ack_accepted, 1) << h.snap().cmd_ack_reason;
  snap = h.snap();
  EXPECT_TRUE(snap.tracks[1].selected) << "the list does not show the operator's own choice";
  EXPECT_FALSE(snap.tracks[0].selected)
      << "the list marks the wrong person as chosen, which is worse than marking nobody";
}

TEST(CandidateList, ATentativeCandidateIsListedButNotSelectable) {
  // §8: TENTATIVE is visible but not choosable. Hiding it entirely would be the worse
  // lie — the operator can see a person standing there, and the list has to explain why
  // that person cannot be named yet rather than silently omitting them.
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "AUTO_TRACK");
  auto set = make_set_with(tracks::TrackState::Tentative, 1, 0.95f);
  feed(h, set, 3, 5);
  auto snap = h.snap();
  ASSERT_EQ(snap.track_count, 1);
  EXPECT_STREQ(snap.tracks[0].state, "TENTATIVE");
  EXPECT_FALSE(snap.tracks[0].selectable);

  // And controld agrees with the page: the label is refused, not silently accepted.
  h.run("select_target", "1");
  EXPECT_EQ(h.snap().cmd_ack_accepted, 0)
      << "the list said not-selectable and the daemon accepted the selection anyway";
}

TEST(CandidateList, VisionGoingQuietAgesTheListInsteadOfRewritingIt) {
  // The tempting behaviour is for controld to rewrite CONFIRMED to LOST after a
  // timeout, and it is wrong: association happens once per detector frame, in visiond
  // (§58). With no frames there is no new observation to report, and inventing one is
  // reporting an inference as a fact. What controld can say truthfully — and must — is
  // how long ago the newest frame in the list arrived, so the page can grey it out.
  // The selection's own staleness (§14) is the separate, already-tested mechanism that
  // decides whether the machine still believes it may follow somebody.
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "AUTO_TRACK");
  auto set = two_people(1, h.t, 1, 2);
  feed(h, set, 10, 10);
  ASSERT_EQ(h.snap().track_count, 2);
  EXPECT_GE(h.snap().track_list_age_ms, 0);

  for (int i = 0; i < 600; ++i) h.step(1);  // 3 s of silence
  auto snap = h.snap();
  EXPECT_GT(snap.track_list_age_ms, 2500)
      << "the list is published as if it were current; its age is "
      << snap.track_list_age_ms << " ms";
  bool still_confirmed = false;
  for (int i = 0; i < snap.track_count; ++i)
    still_confirmed = still_confirmed || (std::string(snap.tracks[i].state) == "CONFIRMED");
  EXPECT_TRUE(still_confirmed)
      << "controld rewrote a state it has not observed since; the age field exists so "
         "it does not have to";
  // What *is* allowed to change on controld's own clock is its belief about following.
  EXPECT_NE(snap.selection_visibility, "VISIBLE")
      << "vision has been silent for 3 s and the selection still claims to be looking "
         "at the person";
}

// --- §79: the structured event record. Not "does a log line exist" — a log line is a
//     rendering. These ask whether the *decision* is present, addressable, and says what
//     it was about, because §80's replay and an operator's memory of an afternoon are
//     both built out of these and neither can be reconstructed from prose.
namespace {
bool has_event(const telemetry::TelemetrySnapshot& s, const char* name,
               const char* must_contain = nullptr) {
  for (int i = 0; i < s.event_tail_count; ++i) {
    if (std::string(s.event_tail[i].name) != name) continue;
    if (must_contain == nullptr) return true;
    if (std::string(s.event_tail[i].detail).find(must_contain) != std::string::npos)
      return true;
  }
  return false;
}
}  // namespace

TEST(Events, DecisionsAreRecordedWithTheirSubject) {
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  const uint64_t before = h.snap().event_generation;

  h.run("set_mode", "AUTO_TRACK");
  auto set = two_people(1, h.t, 1, 2);
  feed(h, set, 6, 10);
  ASSERT_EQ(h.snap().cmd_ack_accepted, 1) << h.snap().cmd_ack_reason;
  h.run("select_target", "2");
  ASSERT_EQ(h.snap().cmd_ack_accepted, 1) << h.snap().cmd_ack_reason;
  feed(h, set, 3, 20);

  auto snap = h.snap();
  EXPECT_GT(snap.event_generation, before) << "decisions were made and nothing was recorded";
  EXPECT_TRUE(has_event(snap, "MODE_CHANGED"))
      << "the operator changed the mode and the record does not show it";
  EXPECT_TRUE(has_event(snap, "TARGET_SELECTED", "Person #2"))
      << "TARGET_SELECTED without the descriptor cannot be read back afterwards: which "
         "person, out of the two standing there?";
  EXPECT_NE(snap.event_tail_count, 0);

  h.run("stop_motion", "");
  {
    std::string seen;
    auto s2 = h.snap();
    for (int i = 0; i < s2.event_tail_count; ++i)
      seen += s2.event_tail[i].name + std::string(" ");
    EXPECT_TRUE(has_event(s2, "STOP_MOTION"))
        << "the operator's own intervention is the first thing any investigation looks "
           "for; the window held: " << seen;
  }
}

TEST(Events, AWindowIsNotAHistoryAndSaysSo) {
  // The published tail holds the newest few. What it does not hold must be visible as a
  // number rather than inferred from absence: "I see 8 events and 40 have happened" is
  // the difference between a quiet station and a feed that dropped the interesting part.
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "AUTO_TRACK");
  auto set = two_people(1, h.t, 1, 2);
  feed(h, set, 3, 10);
  h.run("select_target", "1");  // a decision, so there is a window to watch
  ASSERT_EQ(h.snap().cmd_ack_accepted, 1) << h.snap().cmd_ack_reason;
  feed(h, set, 2, 14);
  const uint64_t g0 = h.snap().event_generation;
  ASSERT_GT(g0, 0u);

  // A cycle with no events must still show the window it showed before.
  h.step(50);
  auto snap = h.snap();
  EXPECT_EQ(snap.event_generation, g0);
  EXPECT_GT(snap.event_tail_count, 0)
      << "the window emptied on a quiet cycle, so a reader sampling at 15 Hz misses "
         "everything that happened between two publishes";

  // Many events later, the generation has moved past the window.
  // Alternated rather than repeated: clearing an empty selection changes nothing, and a
  // no-op that logged an event would make the record a stream of nothing. This loop is
  // the check that only the decisions are recorded — half of these calls are no-ops.
  for (int i = 0; i < 30; ++i) {
    h.run("select_target", "1");
    feed(h, set, 1, 200 + i);
    h.run("clear_target", "");
  }
  EXPECT_GT(h.snap().event_generation, g0 + 20);
  EXPECT_LE(h.snap().event_tail_count, telemetry::TelemetrySnapshot::kEventTail);
}

TEST(Events, JogLeaseExpiryIsRecordedWithoutAnOperatorAskingForIt) {
  // The one MANUAL event that is not a deliberate act, and therefore the one that needs
  // a timestamp: it is what a dropped wifi link looks like from the turret's side.
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "MANUAL");
  h.run("manual_jog_start", "yaw+:normal");
  ASSERT_EQ(h.snap().cmd_ack_accepted, 1) << h.snap().cmd_ack_reason;
  EXPECT_TRUE(has_event(h.snap(), "MANUAL_JOG_STARTED"));

  h.step(200);  // one second of held button with no keepalive: the lease lapses
  EXPECT_TRUE(has_event(h.snap(), "MANUAL_JOG_EXPIRED"))
      << "the jog stopped by itself and the record does not say when";
}

TEST(Events, AnAmbiguousReacquisitionIsRecordedAsAskingRatherThanChoosing) {
  // §21 and §79 together. The scene is the one §89 established — same anchors, same
  // confidences, #2 tentative before the loss — because what is under test here is the
  // record, not the conditions that produce the refusal. Re-deriving those here would
  // make this test fail for reasons that have nothing to do with events, which is how a
  // suite ends up with four tests that all fail together and no way to tell which one
  // knows what.
  HomedLoop h;
  ASSERT_TRUE(h.ready);
  h.run("set_mode", "AUTO_TRACK");
  auto set = two_people(1, h.t, 1, 2, 0.90f, 0.40f, 0.30f, 0.85f);
  set.tracks[1].state = tracks::TrackState::Tentative;
  feed(h, set, 40, 10);
  h.run("select_target", "1");
  ASSERT_EQ(h.snap().cmd_ack_accepted, 1) << h.snap().cmd_ack_reason;
  feed(h, set, 60, 60);
  ASSERT_STREQ(h.snap().mode_phase.c_str(), "TRACKING") << h.snap().mode_phase;

  // A *short* gap, deliberately. The reacquisition score decays with how long the
  // target has been gone, and after a couple of seconds everything scores below the
  // threshold — the machine then holds because nothing is plausible, which is its own
  // good behaviour (§89) but is not a tie. To test the tie the memory has to still be
  // warm: the selection is gone from the set, still recent, and two candidates step into
  // roughly its footprint within a hair of each other.
  tracks::TrackSet none;  // an empty frame: the detector is alive, the person is not
  none.width = 1280;
  none.height = 720;
  feed(h, none, 30, 100);  // 150 ms of nothing in view
  auto both = two_people(200, h.t, 1, 2, 0.88f, 0.86f, 0.30f, 0.34f);
  both.tracks[0].uuid = tracks::TrackUuid{0, 33};
  both.tracks[1].uuid = tracks::TrackUuid{0, 44};
  feed(h, both, 60, 200);

  std::string seen;
  auto s2 = h.snap();
  for (int i = 0; i < s2.event_tail_count; ++i)
    seen += s2.event_tail[i].name + std::string(" | ");
  EXPECT_TRUE(has_event(s2, "TARGET_REACQUIRE_AMBIGUOUS"))
      << "the turret refused to choose between two candidates and left no record of "
         "having refused. phase=" << s2.mode_phase << " ambiguous="
      << s2.selection_ambiguous << " window: " << seen;
  EXPECT_TRUE(has_event(s2, "TARGET_REACQUIRE_AMBIGUOUS", "reselect"))
      << "a refusal that does not tell the operator to choose is a mystery, not a request";
}
