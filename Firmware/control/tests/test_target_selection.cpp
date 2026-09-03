// §83 — TargetSelectionManager unit tests, and the software half of §88.
//
// §83's list: select valid target; reject invalid track; selected target visible;
// occluded; lost; reacquired; ambiguous reacquisition; stale/retired selection;
// selection persists across mode changes; clear selection.
//
// §88 is nominally an integration test ("display two people ... verify #1 moving
// strongly does not steal selection"). Its mechanism needs no hardware, so it is here
// too: two tracks, one selected, the other one sprinting across the frame. The part
// that genuinely needs an operator — watching a real turret on a real pair of people
// and confirming the operator agrees it followed the right one — stays open and is not
// claimed by anything in this file.
#include <gtest/gtest.h>

#include <cstring>
#include <string>

#include "mode/mode_manager.hpp"
#include "tracks/target_selection_manager.hpp"

namespace ota {
namespace {

// Both namespaces have a TrackState (ota::tracking's FSM states and ota::tracks's §8
// track states), so these are spelled out rather than left to lookup: which one a test
// accidentally picked up would be a compile-then-misbehave surprise, not an error.
using tracks::TrackState;
using tracks::TrackUuid;

constexpr TimeNs kFrame = 33'333'333;  // 30 Hz

struct Builder {
  TimeNs now = 1'000'000'000;
  uint32_t seq = 0;

  struct Spec {
    TrackUuid uuid;
    uint16_t index = 1;
    uint16_t cls = 1;
    const char* name = "person";
    TrackState state = TrackState::Confirmed;
    float conf = 0.9f;
    float ax = 0.5f;
    float ay = 0.5f;
    float w = 0.1f;
    float h = 0.2f;
    float vx = 0.0f;
  };

  // Every frame carries the same width/height and advances the clock, so the
  // staleness arithmetic in the manager is driven by the same clock the selection was
  // made against. A test that mixed a wall-clock timestamp into a simulated one would
  // exercise a code path that cannot occur and hide the one that can.
  tracks::TrackSet frame(std::initializer_list<Spec> specs) {
    tracks::TrackSet set;
    set.frame_sequence = ++seq;
    now += kFrame;
    set.sensor_timestamp_ns = now;
    set.publish_timestamp_ns = now + 4'000'000;
    set.width = 1280;
    set.height = 720;
    for (const Spec& s : specs) {
      tracks::Track t;
      t.uuid = s.uuid;
      t.display_index = s.index;
      t.class_id = s.cls;
      std::strncpy(t.class_name, s.name, tracks::kClassNameLen - 1);
      t.state = s.state;
      t.detector_confidence = s.conf;
      t.track_confidence = s.conf;
      t.bbox.x_min = s.ax - s.w / 2;
      t.bbox.x_max = s.ax + s.w / 2;
      t.bbox.y_min = s.ay - s.h / 2;
      t.bbox.y_max = s.ay + s.h / 2;
      t.anchor_x = s.ax;
      t.anchor_y = s.ay;
      t.velocity_x_norm_s = s.vx;
      set.add(t);
    }
    return set;
  }
};

const TrackUuid kOne{0, 1};
const TrackUuid kTwo{0, 2};

class SelectionTest : public ::testing::Test {
 protected:
  void observe(const tracks::TrackSet& s) {
    mgr_.observe(s, b_.now);
  }
  Builder b_;
  tracks::TargetSelectionManager mgr_;
};

TEST_F(SelectionTest, SelectsAValidTarget) {
  observe(b_.frame({{kOne, 1}}));
  auto r = mgr_.select_by_display_index(1, b_.now);
  EXPECT_TRUE(r.ok) << r.reason;
  EXPECT_TRUE(r.changed);
  EXPECT_EQ(r.reason, "selected Person #1")
      << "§10: the operator is told the label they just chose, not a uuid";
  EXPECT_EQ(mgr_.selection().selected, kOne);
  EXPECT_EQ(mgr_.selection().visibility_state, tracks::Visibility::Visible);
}

TEST_F(SelectionTest, RejectsTracksThatAreNotSelectable) {
  // Four different refusals, four different reasons. Collapsing them into "invalid
  // target id" is what a first implementation does and what an operator then has to
  // debug alone, staring at a box that is plainly on the screen.
  observe(b_.frame({{kOne, 1, 1, "person", tracks::TrackState::Tentative}}));
  auto tentative = mgr_.select_by_display_index(1, b_.now);
  EXPECT_FALSE(tentative.ok);
  EXPECT_NE(tentative.reason.find("TENTATIVE"), std::string::npos) << tentative.reason;

  auto unknown = mgr_.select_by_display_index(7, b_.now);
  EXPECT_FALSE(unknown.ok);
  EXPECT_NE(unknown.reason.find("no target # 7"), std::string::npos) << unknown.reason;

  observe(b_.frame({{kTwo, 1, 2, "car"}}));
  auto wrong_class = mgr_.select_by_display_index(1, b_.now);
  EXPECT_FALSE(wrong_class.ok)
      << "§14: the class must be allowed by configuration; the default is v1's person";
  EXPECT_NE(wrong_class.reason.find("not selectable by configuration"),
            std::string::npos)
      << wrong_class.reason;

  tracks::TargetSelectionConfig cfg;
  cfg.allowed_classes = {2, 1, 0, 0, 0, 0, 0, 0};
  mgr_.set_config(cfg);
  auto now_allowed = mgr_.select_by_display_index(1, b_.now);
  EXPECT_TRUE(now_allowed.ok) << now_allowed.reason
                              << " the same refusal must be a configuration, not a wall";
}

TEST_F(SelectionTest, VisibilityWalksVisibleOccludedLost) {
  observe(b_.frame({{kOne, 1}}));
  ASSERT_TRUE(mgr_.select_by_display_index(1, b_.now).ok);
  EXPECT_EQ(mgr_.selection().visibility_state, tracks::Visibility::Visible);
  ASSERT_NE(mgr_.selected_track(), nullptr);

  observe(b_.frame({{kOne, 1, 1, "person", tracks::TrackState::Occluded}}));
  EXPECT_EQ(mgr_.selection().visibility_state, tracks::Visibility::Occluded);
  EXPECT_EQ(mgr_.selected_track(), nullptr)
      << "occluded is not something to keep driving at; §20 coasts instead, and that "
         "decision belongs to the controller, not to a stale CONFIRMED reading";

  for (int i = 0; i < 4; ++i) observe(b_.frame({}));
  EXPECT_EQ(mgr_.selection().visibility_state, tracks::Visibility::LostReacquirable);
  EXPECT_TRUE(mgr_.has_selection())
      << "§12: losing sight of it does not unselect it";
}

TEST_F(SelectionTest, ReacquiresUnambiguouslyAndRefusesWhenItCannot) {
  observe(b_.frame({{kOne, 1, 1, "person", tracks::TrackState::Confirmed, 0.9f, 0.5f,
                     0.5f, 0.1f, 0.2f, 0.02f}}));
  ASSERT_TRUE(mgr_.select_by_display_index(1, b_.now).ok);

  observe(b_.frame({}));
  observe(b_.frame({}));
  ASSERT_EQ(mgr_.selection().visibility_state, tracks::Visibility::LostReacquirable);

  // One candidate where the target was heading. §21's score must be allowed to bring
  // the selection back — this is the difference between a turret that pauses at a
  // pillar and one that has lost the day.
  observe(b_.frame({{kTwo, 1, 1, "person", tracks::TrackState::Confirmed, 0.88f, 0.53f,
                     0.5f, 0.1f, 0.2f, 0.02f}}));
  EXPECT_EQ(mgr_.selection().visibility_state, tracks::Visibility::Visible);
  EXPECT_EQ(mgr_.selection().selected, kTwo);
  EXPECT_FALSE(mgr_.selection().ambiguous_reacquisition);
}

TEST_F(SelectionTest, AmbiguousReacquisitionStopsAndAsks) {
  observe(b_.frame({{kOne, 1, 1, "person", tracks::TrackState::Confirmed, 0.9f, 0.5f,
                     0.5f}}));
  ASSERT_TRUE(mgr_.select_by_display_index(1, b_.now).ok);
  observe(b_.frame({}));
  observe(b_.frame({}));
  ASSERT_EQ(mgr_.selection().visibility_state, tracks::Visibility::LostReacquirable);

  // Two identical people, equidistant from where the target was last seen. Anything is
  // better than the alternative: pick one and the turret follows a stranger while the
  // overlay still says the original label, which is the failure mode that makes an
  // operator stop trusting the screen entirely.
  observe(b_.frame({{kTwo, 1, 1, "person", tracks::TrackState::Confirmed, 0.9f, 0.45f,
                     0.5f},
                    {TrackUuid{0, 3}, 2, 1, "person", tracks::TrackState::Confirmed,
                     0.9f, 0.55f, 0.5f}}));
  EXPECT_TRUE(mgr_.selection().ambiguous_reacquisition);
  EXPECT_EQ(mgr_.selection().visibility_state, tracks::Visibility::LostReacquirable)
      << "§21: remain LOST_HOLD and ask the operator to reselect";
  EXPECT_EQ(mgr_.selection().selected, kOne)
      << "the selection must stay pointed at the lost identity, not quietly move";

  // The operator reselects one of them, and the ambiguity clears.
  auto r = mgr_.select_by_display_index(2, b_.now);
  EXPECT_TRUE(r.ok) << r.reason;
  EXPECT_FALSE(mgr_.selection().ambiguous_reacquisition);
}

TEST_F(SelectionTest, StaleAndRetiredSelectionsAreRefusedNotHonoured) {
  observe(b_.frame({{kOne, 1}}));
  ASSERT_TRUE(mgr_.select_by_display_index(1, b_.now).ok);

  // Long enough that the reacquisition window has closed. Honouring this would drive
  // the turret toward where somebody stood a while ago.
  for (int i = 0; i < 400; ++i) observe(b_.frame({}));
  EXPECT_EQ(mgr_.selection().visibility_state, tracks::Visibility::Stale);
  EXPECT_EQ(mgr_.selected_track(), nullptr);

  // And once the memory of it ages out entirely, selecting it is refused outright.
  auto r = mgr_.select_track(kOne, b_.now + 10'000'000'000);
  EXPECT_FALSE(r.ok) << "selecting something nobody has seen for seconds must not be "
                        "possible by remembering its number";
  EXPECT_NE(r.reason.find("too long ago"), std::string::npos) << r.reason;
}

TEST_F(SelectionTest, ASilentProducerAgesTheWholeListAndNotJustTheSelectedTarget) {
  // The station's real failure, written down as a test. The detector process died, so
  // observe() was never called again — and every staleness rule in this manager lives INSIDE
  // observe(), so nothing recomputed anything. Four people stayed on the candidate list, two
  // of them still CONFIRMED, `select_target` accepted one of them, and the page reported
  // track_list_age_ms climbing past four minutes while the daemon that published it was
  // happily taking the selection. Note that the test above reaches staleness by feeding 400
  // EMPTY frames, which keeps the producer alive; that is the case that already worked.
  observe(b_.frame({{kOne, 1}}));
  ASSERT_TRUE(mgr_.select_by_display_index(1, b_.now).ok);

  const TimeNs four_seconds_later = b_.now + 4'000'000'000;  // and nothing was heard from

  auto r = mgr_.select_by_display_index(1, four_seconds_later);
  EXPECT_FALSE(r.ok) << "a pick out of a list nobody has refreshed in 4 s must be refused";
  EXPECT_NE(r.reason.find("4000 ms old"), std::string::npos)
      << "a refusal that will not say how old the list is, is a shrug: " << r.reason;

  auto r2 = mgr_.select_track(kOne, four_seconds_later);
  EXPECT_FALSE(r2.ok) << "knowing the identifier must not be a way around the age limit";

  EXPECT_EQ(mgr_.effective_visibility(four_seconds_later), tracks::Visibility::Stale)
      << "the page must not go on saying VISIBLE for something unseen since before the crash";

  // Freshen the list and the very same selection is fine again: the refusal is about the
  // silence, not about the target. Without this half the test only proves a refusal exists.
  observe(b_.frame({{kOne, 1}}));
  EXPECT_TRUE(mgr_.select_by_display_index(1, b_.now).ok)
      << mgr_.select_by_display_index(1, b_.now).reason;
}

TEST_F(SelectionTest, SelectionPersistsAcrossModeChanges) {
  // §12, the rule the whole design turns on, and the easiest thing to break by
  // accident: a ModeManager that "helpfully" clears state when the mode changes would
  // make an operator re-pick their target after every MANUAL/AUTO_TRACK round trip,
  // and every one of those re-picks is a chance to select the wrong person.
  observe(b_.frame({{kOne, 1}}));
  ASSERT_TRUE(mgr_.select_by_display_index(1, b_.now).ok);
  const tracks::TrackUuid picked = mgr_.selection().selected;

  ModeManager modes;
  ModeRequestContext ctx;
  ctx.position_valid = true;
  ctx.safety_healthy = true;
  ctx.roam_envelope_valid = true;
  for (auto mode : {OperatingMode::AutoTrack, OperatingMode::Manual,
                    OperatingMode::AutoRoam, OperatingMode::AutoTrack,
                    OperatingMode::Manual}) {
    auto res = modes.request(mode, ctx);
    ASSERT_TRUE(res.ok) << res.reason;
    EXPECT_EQ(mgr_.selection().selected, picked);
    EXPECT_TRUE(mgr_.has_selection()) << operating_mode_name(mode);
  }

  // Only the explicit command removes it (§12's "Clear target").
  auto cleared = mgr_.clear(b_.now);
  EXPECT_TRUE(cleared.ok);
  EXPECT_TRUE(cleared.changed);
  EXPECT_FALSE(mgr_.has_selection());
  EXPECT_NE(cleared.reason.find("cleared Person #1"), std::string::npos)
      << cleared.reason;
  // Clearing twice is not an error worth alarming anybody about, and it must not be
  // reported as if something had been removed.
  auto again = mgr_.clear(b_.now);
  EXPECT_TRUE(again.ok);
  EXPECT_FALSE(again.changed);
}

TEST_F(SelectionTest, SelectedLabelFollowsTheTrackersTable) {
  // §10 reuses a freed display index. A selection that froze the label at the moment
  // of selection would go on saying "Person #2" about somebody else after a retirement
  // reassigned it, so the descriptor is refreshed from the incoming table.
  observe(b_.frame({{kOne, 1}, {kTwo, 2}}));
  ASSERT_TRUE(mgr_.select_by_display_index(2, b_.now).ok);
  EXPECT_EQ(mgr_.selection().selected_display_index, 2);
  EXPECT_STREQ(mgr_.selection().selected_descriptor, "Person #2");

  observe(b_.frame({{kTwo, 1}, {TrackUuid{0, 5}, 2}}));
  EXPECT_STREQ(mgr_.selection().selected_descriptor, "Person #1")
      << "the track kept its identity; the label the operator reads moved with the "
         "tracker's table, which is the only consistent answer";
  EXPECT_EQ(mgr_.selection().selected, kTwo);
}

// §88 (the mechanism, software-only): two people, #2 selected, #1 moves hard.
TEST_F(SelectionTest, ASprintersDoesNotStealTheSelection) {
  Builder b;
  tracks::TargetSelectionManager m;
  auto a = Builder::Spec{kOne, 1};
  auto sel = Builder::Spec{kTwo, 2};
  m.observe(b.frame({a, sel}), b.now);
  ASSERT_TRUE(m.select_by_display_index(2, b.now).ok);

  for (int i = 0; i < 30; ++i) {
    a.ax += 0.012f;      // #1 crosses half the frame at speed
    a.conf = 0.99f;      // and is the better detection all the way
    sel.ax = 0.80f;      // #2 stands still, modestly visible
    sel.conf = 0.6f;
    m.observe(b.frame({a, sel}), b.now);
    EXPECT_EQ(m.selection().selected, kTwo)
        << "frame " << i << ": a stronger detection of a different person took the "
                        "selection. Selection is per-frame best-scoring in v1; in v3 "
                        "it is a decision the operator made";
    EXPECT_EQ(m.selection().visibility_state, tracks::Visibility::Visible);
  }
}

}  // namespace
}  // namespace ota
