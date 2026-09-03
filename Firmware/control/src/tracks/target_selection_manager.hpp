#pragma once
// v3 §12/§13/§14 — "which candidate is THE target", decided once, in one place.
//
// §12 is the load-bearing rule and it is easy to under-read: selection state is
// independent of operating mode. In v1 there was nothing to be independent *from* —
// visiond picked the best-scoring detection every frame, so "the target" was a
// per-frame opinion that changed whenever somebody else scored better. v3 has to be
// able to answer "follow that one" and keep answering it through an occlusion, a mode
// change and a moment when the detector prefers a different human being.
//
// Two consequences that this file is responsible for:
//   * ModeManager must never clear a selection (§12 asks for an explicit Clear target
//     command instead), and switching to MANUAL and back must not lose it.
//   * Selection changes no motor command by itself (§14). It records a decision; the
//     motion is AutoTrackController's to make, and only while the mode allows it.
//
// Threading: owned by the control thread. The web thread never touches it — it pushes
// a command, and the control thread performs the validation, which is what makes the
// answer trustworthy: the reasons below are computed against the TrackSet that is
// actually in hand, not against a snapshot the web thread happened to see.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "common/time.hpp"
#include "tracks/track_set.hpp"

namespace ota::tracks {

// §13's suggested enum, verbatim. STALE is distinct from LOST_REACQUIRABLE on purpose:
// "we could get it back" and "we have waited too long to be believed" read the same on
// a screen unless they are named differently, and the operator's next action differs.
enum class Visibility : uint8_t {
  None = 0,
  Visible = 1,
  Occluded = 2,
  LostReacquirable = 3,
  Stale = 4,
};

inline const char* visibility_name(Visibility v) {
  switch (v) {
    case Visibility::None: return "NONE";
    case Visibility::Visible: return "VISIBLE";
    case Visibility::Occluded: return "OCCLUDED";
    case Visibility::LostReacquirable: return "LOST_REACQUIRABLE";
    case Visibility::Stale: return "STALE";
  }
  return "NONE";
}

struct SelectionResult {
  bool ok = false;
  bool changed = false;
  std::string reason;  // §52: the operator reads this verbatim
};

// §13's TargetSelection. `optional` is spelled as a bool + value because this is
// published into a telemetry snapshot every cycle and a std::optional in that path
// invites a read that assumes presence.
struct TargetSelection {
  bool has_selection = false;
  TrackUuid selected{};
  uint16_t selected_class = 0;
  char selected_descriptor[24] = {};  // "Person #2" (§10), never a bare uuid
  uint16_t selected_display_index = 0;
  TimeNs selection_timestamp = 0;
  Visibility visibility_state = Visibility::None;
  float match_confidence = 0.0f;
  TimeNs last_seen_timestamp = 0;
  bool ambiguous_reacquisition = false;  // §21: two candidates, no legitimate winner
  // §78: the two numbers behind the decision. Without them a reacquisition that did
  // not happen is indistinguishable from one that was refused for a different reason,
  // and "why did it stop following" is answered by reading these rather than by
  // re-running the scene in the yard.
  float reacquisition_score = 0.0f;
  float ambiguity_margin = 0.0f;
};

// §21's reacquisition score, from what §9's wire actually carries.
//
// The document lists an appearance descriptor as one of the inputs. There is none: §9
// has no appearance field, the IMX500 bridge does not produce one, and inventing a
// weight for a signal that does not exist would make the score look more capable than
// it is. So its share goes to where-in-the-image and time-since-seen, and when an
// appearance model arrives this function gains one term and the thresholds get
// re-tuned against recorded data rather than by eye.
// Configs live at namespace scope, aliased into the classes. Not style: a nested
// Config used as a default argument inside its own enclosing class does not compile
// under this g++ (same reason TrackManagerConfig and TargetEstimatorConfig are spelled
// this way), and `Foo::Config cfg = {}` at a call site is the commonest thing that
// would then break.
struct ReacquisitionScorerConfig {
  float gate_norm = 0.25f;    // §112: how far a reacquisition candidate may be
  float threshold = 0.55f;    // §21: score >= reacquire_threshold
  float margin = 0.15f;       // §21: and sufficient margin over the next-best
};

class ReacquisitionScorer {
 public:
  using Config = ReacquisitionScorerConfig;

  explicit ReacquisitionScorer(Config cfg = Config()) : cfg_(cfg) {}

  // All inputs are in normalized image units, as published (§60). `dt_s` is the time
  // since the selected target was last seen, which is what makes the *predicted*
  // position the thing to compare against rather than the old one.
  float score(const Track& candidate, float last_x, float last_y,
              float last_vx, float last_bbox_area, float dt_s) const {
    // Predicted position: anchor advanced by the last known velocity (§9 carries
    // velocity_x_norm_s for exactly this reason).
    const float px = last_x + last_vx * dt_s;
    const float dx = candidate.anchor_x - px;
    const float dy = candidate.anchor_y - last_y;
    const float dist = std::sqrt(dx * dx + dy * dy);
    float loc = 1.0f - dist / cfg_.gate_norm;
    if (loc < 0.0f) loc = 0.0f;

    // Bbox scale: a target that comes back at a tenth of the size it vanished at is
    // probably a different object further away. log ratio, clipped, so a factor-of-e
    // mismatch costs everything and small detector jitter costs nearly nothing.
    const float area = (candidate.bbox.x_max - candidate.bbox.x_min) *
                       (candidate.bbox.y_max - candidate.bbox.y_min);
    float scale = 0.0f;
    if (last_bbox_area > 1e-6f && area > 1e-6f) {
      const float rel = std::fabs(std::log(area / last_bbox_area));
      scale = 1.0f - rel;
      if (scale < 0.0f) scale = 0.0f;
    }

    // Recency. Inside the reacquisition window this decays linearly; the caller has
    // already refused anything beyond it.
    float recency = 1.0f - dt_s;
    if (recency < 0.0f) recency = 0.0f;

    return 0.15f + 0.45f * loc + 0.20f * scale + 0.20f * recency;
  }

  float threshold() const { return cfg_.threshold; }
  float margin() const { return cfg_.margin; }
  float gate_norm() const { return cfg_.gate_norm; }

 private:
  Config cfg_;
};

struct TargetSelectionConfig {
    // §14: "class is allowed by configuration". Terminated by 0 (CLASS_NONE is never
    // a legal selection), so §72 can grow the list without touching this file. The
    // default is v1's: a person. A station that silently acquired any class the
    // detector happens to know about would not be distinguishable from a broken one.
    std::array<uint16_t, 8> allowed_classes{{1, 0, 0, 0, 0, 0, 0, 0}};
    // §14: "track exists or is recently known". How long a uuid that is no longer in
    // the incoming TrackSet still counts as knowable, and how many such records are
    // remembered. A stale selection is refused rather than honoured: selecting a
    // retired track and having the turret swing toward where it used to be is worse
    // than a refusal.
    int64_t recently_known_ms = 1500;

    // How old the candidate LIST may be and still be pointed at. This is not in the config
    // file — `recently_known_ms` above isn't either, so this follows its neighbour rather than
    // inventing a plumbing path for one value. The number is §21's reacquisition window: the
    // station already refuses to keep a *target* alive this long without measurements, so it
    // must not accept a whole *list* that is older than the thing it just gave up on.
    int64_t candidate_list_max_age_ms = 3000;
    int recently_known_slots = 16;
    // §22's reacquisition window, in the same units the vision side uses.
    int64_t reacquire_ms = 2000;
  ReacquisitionScorer::Config scorer;  // §21's thresholds, kept beside the rest
};

class TargetSelectionManager {
 public:
  using Config = TargetSelectionConfig;

  explicit TargetSelectionManager(Config cfg = Config())
      : cfg_(cfg), scorer_(cfg_.scorer) {}

  void set_config(Config cfg) {
    cfg_ = cfg;
    scorer_ = ReacquisitionScorer(cfg.scorer);
  }
  const Config& config() const { return cfg_; }

  const TargetSelection& selection() const { return sel_; }
  bool has_selection() const { return sel_.has_selection; }

  // The TrackSet the manager last observed. Kept because §14's validation
  // ("confirmed", "recently known", "class allowed") can only be answered against the
  // set that is actually in hand — and because this is the first place in controld
  // with a legitimate reason to retain a received set. Nothing else may read it to
  // make its own decisions about identity; that is the boundary
  // test_track_wire.ArrivalDoesNotReassociateAnything holds.
  const TrackSet& last_set() const { return last_set_; }
  bool have_set() const { return have_set_; }

  // Age of the most recent TrackSet, in ms, measured against `now`.
  //
  // Why this is a query and not a field: every other staleness rule in this manager is
  // evaluated inside observe(), i.e. when a new set arrives. A producer that stops publishing
  // never calls observe(), so nothing ever recomputes anything and the last list received is
  // "current" forever. That is not theoretical: on the station the detector process died, the
  // candidate list kept showing four people for four minutes, and `select_target` accepted one
  // of them. Anything whose clock only advances when it is spoken to will report the world as
  // it last heard it.
  int64_t list_age_ms(TimeNs now) const {
    if (!have_set_ || last_set_ns_ == 0) return -1;
    if (now < last_set_ns_) return 0;  // clock moved backwards; treat as fresh, never negative
    return static_cast<int64_t>((now - last_set_ns_) / 1000000);
  }

  // Empty when the list is fresh enough to point at; otherwise the refusal, with the age in
  // it — a refusal that does not carry its number is a shrug (§52).
  std::string list_too_stale(TimeNs now) const {
    const int64_t age = list_age_ms(now);
    if (age < 0 || age <= cfg_.candidate_list_max_age_ms) return {};
    return "the candidate list is " + std::to_string(age) +
           " ms old and nothing has been seen since, so there is nothing visible to select "
           "(the limit is " + std::to_string(cfg_.candidate_list_max_age_ms) +
           " ms) — start the detector again, or roam until something is seen";
  }

  // Visibility as of `now`, which is not the same as visibility as of the last frame. Without
  // this the page keeps saying VISIBLE for a target nobody has observed since the producer
  // went quiet, because the value was true once and nothing was ever asked to revisit it.
  Visibility effective_visibility(TimeNs now) const {
    if (!sel_.has_selection) return Visibility::None;
    if (!list_too_stale(now).empty()) return Visibility::Stale;
    return sel_.visibility_state;
  }

  // §14. The operator picks a *label*, because that is what §10 puts on the screen;
  // the uuid is resolved here, so the number the human reads and the identity the
  // machine holds cannot drift apart without this function noticing.
  SelectionResult select_by_display_index(uint16_t index, TimeNs now) {
    if (!have_set_) {
      return {false, false, "no vision data has reached controld yet"};
    }
    // "Is there any vision data" is the wrong question; "is it current" is the right one.
    // Answering only the first is how a list four minutes old stayed selectable.
    if (std::string why = list_too_stale(now); !why.empty()) return {false, false, why};
    const Track* found = nullptr;
    for (int i = 0; i < last_set_.count; ++i) {
      if (last_set_.tracks[i].display_index == index) {
        found = &last_set_.tracks[i];
        break;
      }
    }
    if (found == nullptr) {
      // Say which number ran out rather than "invalid id": the operator is looking at
      // a list, and "there is no Person #3" is actionable while "invalid target id"
      // sends them back to count the boxes on screen.
      return {false, false,
              "no target # " + std::to_string(index) + " in the current frame"};
    }
    return select_track(found->uuid, now);
  }

  SelectionResult select_track(TrackUuid uuid, TimeNs now) {
    if (!uuid.valid()) return {false, false, "target id must not be zero"};

    const Track* t = find_in_last(uuid);
    if (t != nullptr) {
      // The list-level age check goes on the in-list path ONLY. A uuid that has already left
      // the list has its own, more specific story ("that target was last seen too long ago"),
      // and a refusal that could have named the target but names the list instead is a worse
      // sentence about the same decision. Ordering is not decoration here: the first draft of
      // this gate ran ahead of that branch and replaced the specific reason with the general
      // one, which is the kind of thing an operator has to work around.
      if (std::string why = list_too_stale(now); !why.empty()) return {false, false, why};
    }
    if (t == nullptr) {
      const Recent* r = recent_find(uuid);
      if (r == nullptr) return {false, false, "unknown target id"};
      if (now - r->last_seen_ns > cfg_.recently_known_ms * 1000000) {
        return {false, false, "that target was last seen too long ago to select"};
      }
      if (!class_allowed(r->class_id)) return class_refusal(r->class_id);
      // Recently known but not currently confirmed. §14 allows selecting a target the
      // tracker has only just lost — the operator pointing at an empty doorway is a
      // legitimate instruction — but it must not be treated as a live track, so the
      // visibility starts where the evidence ends rather than at VISIBLE.
      const bool was_confirmed = r->was_confirmed;
      TargetSelection s;
      s.has_selection = true;
      s.selected = uuid;
      s.selected_class = r->class_id;
      s.selected_display_index = r->display_index;
      fill_descriptor(s.selected_descriptor, sizeof s.selected_descriptor, r->class_name,
                      r->display_index);
      s.selection_timestamp = now;
      s.last_seen_timestamp = r->last_seen_ns;
      s.match_confidence = r->confidence;
      s.visibility_state = was_confirmed ? Visibility::LostReacquirable : Visibility::Stale;
      const bool changed = !(sel_.has_selection && sel_.selected == uuid);
      sel_ = s;
      if (!changed) return {true, false, "already selected"};
      return {true, true,
              was_confirmed
                  ? std::string("selected ") + s.selected_descriptor +
                        " (not visible now: it will be acquired when it reappears)"
                  : std::string("selected ") + s.selected_descriptor +
                        " (identity is too old to trust: reselect when visible)"};
    }

    if (!class_allowed(t->class_id)) return class_refusal(t->class_id);
    // §14 verbatim: "track is confirmed". A TENTATIVE flicker is refused here and
    // nowhere else, which is what makes the same word in §8 mean the same thing in the
    // overlay, in this validation, and in AutoTrackController's coast logic.
    if (t->state != TrackState::Confirmed) {
      return {false, false,
              std::string(t->class_name) + " #" +
                  std::to_string(t->display_index) + " is " +
                  state_name(t->state) + ", not CONFIRMED"};
    }

    TargetSelection s;
    s.has_selection = true;
    s.selected = uuid;
    s.selected_class = t->class_id;
    s.selected_display_index = t->display_index;
    fill_descriptor(s.selected_descriptor, sizeof s.selected_descriptor, t->class_name,
                    t->display_index);
    s.selection_timestamp = now;
    s.last_seen_timestamp = last_sensor_ns_;
    s.match_confidence = t->track_confidence;
    s.visibility_state = Visibility::Visible;
    const bool changed = !(sel_.has_selection && sel_.selected == uuid);
    sel_ = s;
    if (!changed) return {true, false, "already selected"};
    return {true, true, "selected " + std::string(s.selected_descriptor)};
  }

  // §12: this is the ONLY thing that removes a selection. Mode changes go nowhere
  // near it.
  SelectionResult clear(TimeNs now) {
    (void)now;
    if (!sel_.has_selection) return {true, false, "no target was selected"};
    const std::string who = sel_.selected_descriptor;
    sel_ = {};
    return {true, true, "cleared " + who};
  }

  // Once per incoming TrackSet, on the control thread. Maintains the visibility state
  // and the recent-known memory, and applies §21 when a lost selection has to be
  // found again.
  void observe(const TrackSet& set, TimeNs now) {
    last_set_ = set;
    last_arrival_ns_ = now;
    last_sensor_ns_ = set.sensor_timestamp_ns;
    have_set_ = true;
    last_set_ns_ = now;  // see list_age_ms(): the point is that this is the LAST time it
                         // happened to be true, which is a fact only a query can use later.

    for (int i = 0; i < set.count; ++i) remember(set.tracks[i], now);

    if (!sel_.has_selection) return;

    const Track* t = find_in_last(sel_.selected);
    if (t != nullptr && t->state == TrackState::Confirmed) {
      sel_.visibility_state = Visibility::Visible;
      sel_.match_confidence = t->track_confidence;
      sel_.last_seen_timestamp = now;
      sel_.ambiguous_reacquisition = false;
      sel_.reacquisition_score = 0.0f;
      sel_.ambiguity_margin = 0.0f;
      // The label follows the tracker's table, not the moment of selection: §10 reuses
      // a freed display index, so a selection made against an old number would
      // otherwise keep naming the wrong person after a retirement.
      sel_.selected_display_index = t->display_index;
      fill_descriptor(sel_.selected_descriptor, sizeof sel_.selected_descriptor,
                      t->class_name, t->display_index);
      sel_.selected_class = t->class_id;
      return;
    }
    if (t != nullptr && t->state == TrackState::Occluded) {
      sel_.visibility_state = Visibility::Occluded;
      sel_.ambiguous_reacquisition = false;
      return;
    }

    // Not present (or only a Lost record). How long since we last saw it decides
    // whether we are still willing to look for it at all (§22).
    const int64_t gone_ms =
        (sel_.last_seen_timestamp > 0 && now >= sel_.last_seen_timestamp)
            ? (now - sel_.last_seen_timestamp) / 1000000
            : 0;
    sel_.visibility_state = gone_ms * 1000000 <= cfg_.reacquire_ms * 1000000
                                ? Visibility::LostReacquirable
                                : Visibility::Stale;
    if (sel_.visibility_state != Visibility::LostReacquirable) {
      sel_.ambiguous_reacquisition = false;
      return;
    }

    // §21: a specific selected target must not silently become another object of the
    // same class. Score every same-class candidate inside the gate, and only hand the
    // selection back when one of them is unambiguously the best.
    const Recent* r = recent_find(sel_.selected);
    if (r == nullptr) return;
    const float dt_s = static_cast<float>(gone_ms) / 1000.0f;
    float best = -1.0f, second = -1.0f;
    TrackUuid best_uuid{};
    for (int i = 0; i < set.count; ++i) {
      const Track& c = set.tracks[i];
      if (c.uuid == sel_.selected) continue;          // that would not be a reacquisition
      if (c.class_id != sel_.selected_class) continue;  // same class only (§21)
      if (c.state != TrackState::Confirmed) continue;
      const float s = scorer_.score(c, r->anchor_x, r->anchor_y, r->velocity_x_norm_s,
                                    r->bbox_area, dt_s);
      if (s > best) {
        second = best;
        best = s;
        best_uuid = c.uuid;
      } else if (s > second) {
        second = s;
      }
    }
    if (best < scorer_.threshold()) {
      sel_.ambiguous_reacquisition = false;
      sel_.reacquisition_score = best < 0.0f ? 0.0f : best;
      sel_.ambiguity_margin = second < 0.0f ? 1.0f : best - second;
      return;
    }
    if (second >= 0.0f && best - second < scorer_.margin()) {
      // Two plausible candidates and no legitimate way to choose. Taking the better
      // one anyway is how a turret ends up tracking a stranger while the overlay still
      // says "Person #1" — the failure §83 lists as "ambiguous reacquisition" and the
      // reason the answer here is to stop and ask, not to guess.
      sel_.ambiguous_reacquisition = true;
      sel_.visibility_state = Visibility::LostReacquirable;
      sel_.reacquisition_score = best;
      sel_.ambiguity_margin = best - second;
      return;
    }
    sel_.ambiguous_reacquisition = false;
    sel_.reacquisition_score = best;
    sel_.ambiguity_margin = best - second;
    sel_.selected = best_uuid;  // §21: reacquired under the same selection
    sel_.visibility_state = Visibility::Visible;
  }

  // The track to feed to the tracker, or nullptr when the selected target is not
  // something to point a turret at right now. Occluded still resolves: §20 coasts
  // toward the last prediction, and the caller decides what to do with that.
  const Track* selected_track() const {
    if (!sel_.has_selection || !have_set_) return nullptr;
    if (sel_.visibility_state != Visibility::Visible) return nullptr;
    const Track* t = find_in_last(sel_.selected);
    if (t == nullptr || t->state != TrackState::Confirmed) return nullptr;
    return t;
  }

 private:
  // §14's "recently known". Bounded and time-limited: an unbounded memory of every
  // uuid ever seen is how a station becomes able to select things nobody is looking
  // at, and 16 slots at camera rate covers far longer than the window that honours it.
  struct Recent {
    TrackUuid uuid{};
    uint16_t class_id = 0;
    uint16_t display_index = 0;
    char class_name[kClassNameLen] = {};
    float confidence = 0.0f;
    float anchor_x = 0.0f;
    float anchor_y = 0.0f;
    float velocity_x_norm_s = 0.0f;
    float bbox_area = 0.0f;
    TimeNs last_seen_ns = 0;
    bool was_confirmed = false;
  };

  bool class_allowed(uint16_t id) const {
    for (uint16_t c : cfg_.allowed_classes) {
      if (c == 0) return false;
      if (c == id) return true;
    }
    return false;
  }

  static SelectionResult class_refusal(uint16_t id) {
    return {false, false,
            "class id " + std::to_string(id) +
                " is not selectable by configuration (§14)"};
  }

  static const char* state_name(TrackState s) {
    switch (s) {
      case TrackState::Tentative: return "TENTATIVE";
      case TrackState::Confirmed: return "CONFIRMED";
      case TrackState::Occluded: return "OCCLUDED";
      case TrackState::Lost: return "LOST";
    }
    return "UNKNOWN";
  }

  static void fill_descriptor(char* out, size_t n, const char* class_name,
                              uint16_t index) {
    // §10: "Person #2", capitalized for the operator, never the raw uuid. The uuid is
    // what travels in commands; this is what a human reads, and conflating them is how
    // a label ends up being used as an identity after a retirement reuses the label.
    char cap[kClassNameLen] = {};
    std::strncpy(cap, class_name, kClassNameLen - 1);
    if (cap[0] >= 'a' && cap[0] <= 'z') cap[0] = static_cast<char>(cap[0] - 'a' + 'A');
    std::snprintf(out, n, "%s #%u", cap, static_cast<unsigned>(index));
  }

  const Track* find_in_last(TrackUuid uuid) const {
    for (int i = 0; i < last_set_.count; ++i) {
      if (last_set_.tracks[i].uuid == uuid) return &last_set_.tracks[i];
    }
    return nullptr;
  }

  const Recent* recent_find(TrackUuid uuid) const {
    for (int i = 0; i < recent_n_; ++i) {
      if (recent_[i].uuid == uuid) return &recent_[i];
    }
    return nullptr;
  }

  void remember(const Track& t, TimeNs now) {
    if (!t.uuid.valid()) return;
    for (int i = 0; i < recent_n_; ++i) {
      if (recent_[i].uuid == t.uuid) {
        touch(recent_[i], t, now);
        return;
      }
    }
    int slot = recent_n_ < cfg_.recently_known_slots ? recent_n_ : 0;
    if (recent_n_ < cfg_.recently_known_slots) ++recent_n_;
    else {
      // Full: replace the oldest sighting. Replacing by age rather than by slot order
      // matters because the table is written in arrival order, which is the order the
      // detector happened to emit — not an ordering with any meaning.
      for (int i = 1; i < recent_n_; ++i) {
        if (recent_[i].last_seen_ns < recent_[slot].last_seen_ns) slot = i;
      }
    }
    touch(recent_[slot], t, now);
  }

  static void touch(Recent& r, const Track& t, TimeNs now) {
    r.uuid = t.uuid;
    r.class_id = t.class_id;
    r.display_index = t.display_index;
    std::memcpy(r.class_name, t.class_name, kClassNameLen);
    r.confidence = t.track_confidence;
    r.anchor_x = t.anchor_x;
    r.anchor_y = t.anchor_y;
    r.velocity_x_norm_s = t.velocity_x_norm_s;
    r.bbox_area = (t.bbox.x_max - t.bbox.x_min) * (t.bbox.y_max - t.bbox.y_min);
    r.last_seen_ns = now;
    if (t.state == TrackState::Confirmed) r.was_confirmed = true;
  }

  Config cfg_;
  ReacquisitionScorer scorer_;
  TargetSelection sel_;
  TrackSet last_set_{};
  bool have_set_ = false;
  TimeNs last_set_ns_ = 0;  // when a TrackSet last actually arrived, not when the world last was
  TimeNs last_arrival_ns_ = 0;
  TimeNs last_sensor_ns_ = 0;
  Recent recent_[16]{};
  int recent_n_ = 0;
};

}  // namespace ota::tracks
