// OpenAutoTurret v3 §82 — TrackManager unit tests.
//
// The eight required cases, named after the spec's own list so a reader can check
// §82 off line by line:
//   single target; crossing targets; temporary occlusion; detector dropout;
//   two same-class targets; abrupt bbox-size change; track creation/retirement;
//   UUID stability.
//
// All synthetic, no camera (§82: "Synthetic tests should not depend on camera
// hardware"). The frame helpers below build TrackSets directly; the association
// rules being pinned here are the ones the overlay and the selection manager are
// built on, so a change to them should make this file argue.
#include "tracks/track_manager.hpp"

#include <gtest/gtest.h>

#include <cstring>

#include "tracks/track_set.hpp"

using namespace ota::tracks;

namespace {

constexpr int64_t kFrameNs = 33'333'333;  // 30 Hz camera

Track make_det(uint16_t class_id, const char* name, float ax, float ay,
               float w = 0.10f, float h = 0.20f, float conf = 0.9f) {
  Track t;
  t.class_id = class_id;
  std::strncpy(t.class_name, name, kClassNameLen - 1);
  t.anchor_x = ax;
  t.anchor_y = ay;
  t.bbox.x_min = ax - w / 2.0f;
  t.bbox.x_max = ax + w / 2.0f;
  t.bbox.y_min = ay - h / 2.0f;
  t.bbox.y_max = ay + h / 2.0f;
  t.detector_confidence = conf;
  t.track_confidence = conf;
  return t;
}

// Feed one frame. `seq` advances the capture clock so the predictor sees a real dt.
void feed(TrackManager& m, int64_t seq, std::initializer_list<Track> dets,
          int64_t now_ns) {
  TrackSet set;
  set.frame_sequence = seq;
  set.sensor_timestamp_ns = seq * kFrameNs;
  set.publish_timestamp_ns = set.sensor_timestamp_ns + 8'000'000;  // 8 ms later
  set.width = 1280;
  set.height = 720;
  for (const Track& d : dets) set.add(d);
  m.update(set, now_ns);
}

// Hold the manager at `now_ns` while advancing frames, so state ladders that are
// measured in frames can be walked without a wall-clock dependency.
void feed_empty(TrackManager& m, int64_t& seq, int n, int64_t& now_ns) {
  for (int i = 0; i < n; ++i) {
    feed(m, ++seq, {}, now_ns += kFrameNs);
  }
}

}  // namespace

TEST(TrackManager, SingleTargetBecomesSelectable) {
  TrackManager m;
  int64_t seq = 0, now = 0;

  feed(m, ++seq, {make_det(0, "person", 0.5f, 0.5f)}, now += kFrameNs);
  const TrackUuid uuid = [&] {
    Track t{};
    m.copy_tracks(&t, 1);
    return t.uuid;
  }();
  ASSERT_TRUE(uuid.valid());
  EXPECT_FALSE(m.is_selectable(uuid))
      << "a first-frame detection is not something to point a turret at (§8)";

  for (int i = 0; i < 3; ++i)
    feed(m, ++seq, {make_det(0, "person", 0.5f, 0.5f)}, now += kFrameNs);
  EXPECT_TRUE(m.is_selectable(uuid))
      << "consistent frames must confirm it (§8 TENTATIVE -> CONFIRMED)";
  EXPECT_TRUE(m.is_trackable(uuid));
  EXPECT_EQ(m.track_count(), 1) << "one target must not become three";
}

TEST(TrackManager, CrossingTargetsKeepTheirIdentities) {
  // Two targets on a collision course, passing within a few pixels of each other.
  // The failure mode this pins: one frame of overlap makes the association swap, and
  // from then on the turret is chasing the wrong human being while the overlay labels
  // stay exactly as they were. Constant-velocity prediction plus mutual-best matching
  // is what keeps each track on its own momentum.
  TrackManager m;
  int64_t seq = 0, now = 0;

  Track a = make_det(0, "person", 0.30f, 0.5f);
  Track b = make_det(0, "person", 0.40f, 0.5f);
  for (int i = 0; i < 4; ++i) {  // establish both, moving toward each other
    a.anchor_x -= 0.005f;
    b.anchor_x += 0.005f;
    feed(m, ++seq, {a, b}, now += kFrameNs);
  }
  Track tracks[2];
  ASSERT_EQ(m.copy_tracks(tracks, 2), 2);
  const TrackUuid left = tracks[0].anchor_x < tracks[1].anchor_x ? tracks[0].uuid
                                                                : tracks[1].uuid;
  const TrackUuid right = left == tracks[0].uuid ? tracks[1].uuid : tracks[0].uuid;

  // Cross them: they meet, overlap, and pass through each other.
  for (int i = 0; i < 20; ++i) {
    a.anchor_x -= 0.005f;
    b.anchor_x += 0.005f;
    feed(m, ++seq, {a, b}, now += kFrameNs);
  }
  ASSERT_EQ(m.copy_tracks(tracks, 2), 2) << "a crossing must not delete a track";
  const TrackUuid now_left = tracks[0].anchor_x < tracks[1].anchor_x ? tracks[0].uuid
                                                                    : tracks[1].uuid;
  EXPECT_EQ(now_left, left)
      << "identities swapped when the two crossed — the turret would now follow the "
         "wrong person while the screen still says the same thing";
  EXPECT_NE(now_left, right);
}

TEST(TrackManager, TemporaryOcclusionSurvivesAndStaysTrackable) {
  TrackManager m;
  int64_t seq = 0, now = 0;
  for (int i = 0; i < 5; ++i)
    feed(m, ++seq, {make_det(0, "person", 0.5f, 0.5f)}, now += kFrameNs);
  Track t{};
  ASSERT_EQ(m.copy_tracks(&t, 1), 1);
  const TrackUuid uuid = t.uuid;
  ASSERT_TRUE(m.is_selectable(uuid));

  feed_empty(m, seq, 5, now);  // a pillar passes between camera and target
  EXPECT_NE(m.find(uuid), nullptr) << "an occluded track must still exist";
  EXPECT_EQ(m.find(uuid)->state, TrackState::Occluded) << "§8";
  EXPECT_TRUE(m.is_trackable(uuid))
      << "§20's coasting needs the selected target to still be a subject";
  EXPECT_FALSE(m.is_selectable(uuid))
      << "but an operator must not be able to newly select something the detector "
         "is not confirming right now";

  feed(m, ++seq, {make_det(0, "person", 0.5f, 0.5f)}, now += kFrameNs);
  EXPECT_EQ(m.find(uuid)->state, TrackState::Confirmed);
  EXPECT_TRUE(m.is_selectable(uuid));
}

TEST(TrackManager, DetectorDropoutWalksTheStateLadderAndRetires) {
  TrackManager m;
  int64_t seq = 0, now = 0;
  for (int i = 0; i < 5; ++i)
    feed(m, ++seq, {make_det(0, "person", 0.5f, 0.5f)}, now += kFrameNs);
  Track t{};
  ASSERT_EQ(m.copy_tracks(&t, 1), 1);
  const TrackUuid uuid = t.uuid;

  feed_empty(m, seq, m.config().lost_frames, now);
  ASSERT_NE(m.find(uuid), nullptr);
  EXPECT_EQ(m.find(uuid)->state, TrackState::Lost) << "§8";
  EXPECT_FALSE(m.is_trackable(uuid))
      << "a lost track is not something the controller keeps driving toward";

  feed_empty(m, seq, m.config().reacquire_frames + 1, now);
  EXPECT_EQ(m.find(uuid), nullptr)
      << "§22: past the reacquisition window the track retires";
  EXPECT_GT(m.stats().retired, 0u);
}

TEST(TrackManager, TwoSameClassTargetsGetDifferentLabels) {
  // §10: "Person #1 / Person #2", not raw UUIDs, and not one label shared by two
  // people. The reuse rule (below) is the other half of the same section.
  TrackManager m;
  int64_t seq = 0, now = 0;
  for (int i = 0; i < 5; ++i) {
    feed(m, ++seq,
         {make_det(0, "person", 0.3f, 0.5f), make_det(0, "person", 0.7f, 0.5f)},
         now += kFrameNs);
  }
  Track tracks[4];
  ASSERT_EQ(m.copy_tracks(tracks, 4), 2);
  EXPECT_NE(tracks[0].uuid, tracks[1].uuid);
  EXPECT_NE(tracks[0].display_index, tracks[1].display_index)
      << "two people cannot share one label on the selector";
  EXPECT_EQ(tracks[0].display_index, 1);
  EXPECT_EQ(tracks[1].display_index, 2);
  EXPECT_EQ(tracks[0].class_id, tracks[1].class_id);
}

TEST(TrackManager, DisplayIndexIsReusedOnlyAfterRetirement) {
  // §10 verbatim. This is the reason selection must carry the UUID: if an index
  // could be handed out while its previous owner still existed, the operator's
  // "#1" would silently mean a different human being.
  TrackManager m;
  int64_t seq = 0, now = 0;
  for (int i = 0; i < 5; ++i) {
    feed(m, ++seq,
         {make_det(0, "person", 0.3f, 0.5f), make_det(0, "person", 0.7f, 0.5f)},
         now += kFrameNs);
  }
  Track tracks[2];
  ASSERT_EQ(m.copy_tracks(tracks, 2), 2);
  const TrackUuid first = tracks[0].display_index == 1 ? tracks[0].uuid : tracks[1].uuid;

  // Kill the #1 track only: keep feeding a detection at the other anchor.
  TrackUuid survivor;
  survivor = first == tracks[0].uuid ? tracks[1].uuid : tracks[0].uuid;
  const float keep_x = tracks[0].display_index == 1 ? 0.7f : 0.3f;
  for (int i = 0; i < m.config().reacquire_frames + m.config().lost_frames + 2; ++i)
    feed(m, ++seq, {make_det(0, "person", keep_x, 0.5f)}, now += kFrameNs);

  ASSERT_EQ(m.find(survivor)->display_index, 2) << "the survivor keeps its label";
  EXPECT_EQ(m.find(first), nullptr) << "the retired one is gone";

  // A third person appears and takes the freed label — legitimately, because #1's
  // owner is fully retired.
  for (int i = 0; i < 5; ++i) {
    feed(m, ++seq,
         {make_det(0, "person", keep_x, 0.5f), make_det(0, "person", 0.15f, 0.8f)},
         now += kFrameNs);
  }
  Track after[3];
  const int n = m.copy_tracks(after, 3);
  bool newcomer_has_one = false;
  for (int i = 0; i < n; ++i)
    if (after[i].uuid != survivor && after[i].display_index == 1)
      newcomer_has_one = true;
  EXPECT_TRUE(newcomer_has_one)
      << "a freed index should be handed to the next track, lowest first";
}

TEST(TrackManager, AbruptBboxSizeChangeDoesNotKillTheTrack) {
  // §82's quietest requirement and the reason association is anchored on the anchor
  // point rather than on IoU. A detector that suddenly returns half the box (target
  // crouches, occluder clips the legs, model has an off frame) must not be read as
  // "this object is gone and a new one appeared here".
  TrackManager m;
  int64_t seq = 0, now = 0;
  for (int i = 0; i < 5; ++i)
    feed(m, ++seq, {make_det(0, "person", 0.5f, 0.5f, 0.30f, 0.60f)},
         now += kFrameNs);
  Track t{};
  ASSERT_EQ(m.copy_tracks(&t, 1), 1);
  const TrackUuid uuid = t.uuid;
  ASSERT_TRUE(m.is_selectable(uuid));

  for (int i = 0; i < 3; ++i)
    feed(m, ++seq, {make_det(0, "person", 0.5f, 0.5f, 0.06f, 0.10f)},
         now += kFrameNs);

  ASSERT_NE(m.find(uuid), nullptr)
      << "the box shrank to a fifth and the identity was thrown away";
  EXPECT_EQ(m.find(uuid)->state, TrackState::Confirmed);
  EXPECT_EQ(m.track_count(), 1) << "and a second track was invented in its place";
}

TEST(TrackManager, UuidStaysStableAcrossEveryTransition) {
  // §10. The selection command carries the UUID, so an identity that changes under
  // the operator's finger would either drop tracking or — worse, because it is
  // silent — retarget onto somebody else.
  TrackManager m;
  int64_t seq = 0, now = 0;
  for (int i = 0; i < 5; ++i)
    feed(m, ++seq, {make_det(0, "person", 0.5f, 0.5f)}, now += kFrameNs);
  Track t{};
  ASSERT_EQ(m.copy_tracks(&t, 1), 1);
  const TrackUuid uuid = t.uuid;
  const uint16_t label = t.display_index;

  feed_empty(m, seq, 6, now);                       // through OCCLUDED
  ASSERT_NE(m.find(uuid), nullptr);
  feed(m, ++seq, {make_det(0, "person", 0.52f, 0.5f)}, now += kFrameNs);  // back
  ASSERT_NE(m.find(uuid), nullptr);
  EXPECT_EQ(m.find(uuid)->uuid, uuid);
  EXPECT_EQ(m.find(uuid)->display_index, label);

  // A re-identification inside the reacquisition window keeps the same identity too
  // (§21), which is what lets a selection survive being briefly lost.
  feed_empty(m, seq, m.config().lost_frames + 2, now);
  ASSERT_NE(m.find(uuid), nullptr) << "still inside the reacquire window";
  feed(m, ++seq, {make_det(0, "person", 0.52f, 0.5f)}, now += kFrameNs);
  ASSERT_NE(m.find(uuid), nullptr)
      << "reacquisition must reuse the identity, not mint a new one";
  EXPECT_GT(m.stats().reacquired, 0u);
}

TEST(TrackManager, LatencyStampsAreReportedAndUnknownIsNotZero) {
  // §61. "Never received" and "0 ms old" are different claims, and a dashboard that
  // shows the second when the first is true is what makes a dead detector look like
  // a healthy one with an old frame.
  TrackManager m;
  EXPECT_LT(m.stale_ms(1'000'000'000), 0.0);
  EXPECT_LT(m.sensor_age_ms(1'000'000'000), 0.0);
  EXPECT_LT(m.publish_to_receive_ms(), 0.0);

  // Three stamps, two intervals (§61): captured at T, let go by visiond at T+8 ms,
  // received by controld at T+12 ms. The first version of this test fed a receipt
  // timestamp *earlier* than the publish stamp and failed — which was the
  // implementation refusing to publish a negative transport delay, and the test
  // inventing a frame that arrives before it is sent. The guard is the behaviour;
  // the numbers below are the ones a real pipeline could produce.
  const int64_t capture = 300 * kFrameNs;               // sensor_timestamp_ns
  const int64_t now = capture + 12'000'000;             // received 12 ms later
  feed(m, /*seq=*/300, {make_det(0, "person", 0.5f, 0.5f)}, now);
  EXPECT_DOUBLE_EQ(m.stale_ms(now), 0.0) << "this frame just arrived";
  EXPECT_NEAR(m.sensor_age_ms(now), 12.0, 0.001)
      << "how stale the observation is, measured from the camera's own clock";
  EXPECT_NEAR(m.publish_to_receive_ms(), 4.0, 0.001)
      << "the transport delay has to be visible on its own, or a slow publisher and "
         "a slow camera are indistinguishable — and they need different fixes";
}

TEST(TrackManager, CapacityPressureNeverSacrificesASelectedTrack) {
  // At kMaxTracks the manager has to drop something. Dropping a confirmed track the
  // operator may have selected, to make room for a detection nobody asked for, is the
  // one option that would look like the turret misbehaving with nothing in the log.
  TrackManager m;
  int64_t seq = 0, now = 0;
  Track dets[kMaxTracks];
  for (int i = 0; i < kMaxTracks; ++i)
    dets[i] = make_det(0, "person", 0.01f + 0.02f * i, 0.5f);
  for (int round = 0; round < 5; ++round) {
    TrackSet set;
    set.frame_sequence = seq;
    set.sensor_timestamp_ns = seq * kFrameNs;
    for (int i = 0; i < kMaxTracks; ++i) set.add(dets[i]);
    m.update(set, now += kFrameNs);
    ++seq;
  }
  ASSERT_EQ(m.track_count(), kMaxTracks);
  Track first{};
  const int n = m.copy_tracks(&first, 1);
  ASSERT_EQ(n, 1);
  const TrackUuid oldest = first.uuid;

  // One more track on top of a full table.
  for (int round = 0; round < 6; ++round) {
    TrackSet set;
    set.frame_sequence = seq;
    set.sensor_timestamp_ns = seq * kFrameNs;
    for (int i = 0; i < kMaxTracks; ++i) set.add(dets[i]);
    set.add(make_det(0, "person", 0.99f, 0.99f));
    m.update(set, now += kFrameNs);
    ++seq;
  }
  EXPECT_EQ(m.track_count(), kMaxTracks) << "the cap is a contract, not a hint";
  EXPECT_NE(m.find(oldest), nullptr)
      << "a confirmed track was dropped to make room for an unproven detection";
}
