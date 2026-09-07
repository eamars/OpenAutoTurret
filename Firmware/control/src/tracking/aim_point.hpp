#pragma once
// Where inside a target the axis should actually be pointed.
//
// The operator's acceptance rule is stated against the target's HEAD with a tolerance of one third
// of the box height. Centring the box - which is what AUTO_TRACK does today, because the §9 anchor
// is the point tracking follows - puts the reticle on a standing person's torso and would still
// count as "centred" by every number the controller currently publishes. So the aim point is a
// separate concept from the anchor, computed from the detector's own box, and it is deliberately
// not wired into the wire format: this is a controller policy about a point, not a new field from
// visiond.
//
// It is expressed in the detector's normalised box (as published in the TrackSet) and turned into
// pixels through the intrinsics that were actually loaded, so it cannot disagree with the reticle
// placement in the HUD - both are the same principal point.
#include "geometry/camera_model.hpp"

namespace ota {
namespace tracking {

enum class AimMode { Legacy, PerceptionAnchor, BoxFraction };

inline const char* aim_mode_name(AimMode mode) {
  switch (mode) {
    case AimMode::PerceptionAnchor: return "perception_anchor";
    case AimMode::BoxFraction: return "box_fraction";
    default: return "legacy";
  }
}

struct AimOptions {
  // Off by default: centroid following is the behaviour that was measured and tuned, and the
  // switch to head aiming changes the reference the loop servo-ed against. It is turned on in the
  // station config, deliberately, not by a changed default in code.
  bool aim_at_head = false;
  // Fraction of the box height measured DOWN from the top edge. 0.22 is the middle of a typical
  // person box's head for the detection model in use; it is a config value rather than a constant
  // because it is an assumption about a detector, not a fact about geometry.
  double head_fraction_from_top = 0.22;
  AimMode mode = AimMode::Legacy;
  double x_fraction = 0.5;
  double y_fraction = 0.22;
  bool operator==(const AimOptions&) const = default;
};

inline bool valid_aim_options(const AimOptions& opt) {
  return opt.mode == AimMode::Legacy ||
      ((opt.mode == AimMode::PerceptionAnchor || opt.mode == AimMode::BoxFraction) &&
       std::isfinite(opt.x_fraction) && std::isfinite(opt.y_fraction) &&
       opt.x_fraction >= 0 && opt.x_fraction <= 1 && opt.y_fraction >= 0 && opt.y_fraction <= 1);
}

struct AimPoint {
  double u_px = 0.0;
  double v_px = 0.0;
  // False whenever the head rule could not be honoured, so the caller can tell "aiming at the
  // head" from "fell back to the anchor". A HUD that draws a head cue over an anchor-following
  // controller would be lying in exactly the way that matters.
  bool head_applied = false;
  const char* source = "perception_anchor";
  bool box_clipped = false;
};

inline AimPoint aim_point_px(double anchor_u_px, double anchor_v_px,
                             double x_min_norm, double y_min_norm,
                             double x_max_norm, double y_max_norm,
                             const geo::CameraIntrinsics& in,
                             const AimOptions& opt, bool authoritative_anchor = false) {
  AimPoint a;
  a.u_px = anchor_u_px;
  a.v_px = anchor_v_px;
  if (opt.mode == AimMode::PerceptionAnchor ||
      (opt.mode == AimMode::Legacy && (authoritative_anchor || !opt.aim_at_head))) return a;

  a.source = "invalid_box_anchor_fallback";

  const double w = x_max_norm - x_min_norm;
  const double h = y_max_norm - y_min_norm;
  // No usable box, or no usable intrinsics: there is no head to find, so the anchor is the
  // honest answer and head_applied says so. Silently aiming at (0,0) - what an uninitialised box
  // would give - would slew the axis at the top-left corner of the frame.
  // in.valid() rather than spot-checking one focal length: the intrinsics type already knows what
  // "usable" means, and a guard that guesses which field matters will always check the wrong one.
  if (!(w > 0.0) || !(h > 0.0) || !in.valid() ||
      !std::isfinite(x_min_norm) || !std::isfinite(y_min_norm) ||
      !std::isfinite(x_max_norm) || !std::isfinite(y_max_norm) ||
      (opt.mode == AimMode::BoxFraction &&
       !(x_min_norm >= 0 && y_min_norm >= 0 && x_max_norm <= 1 && y_max_norm <= 1))) {
    return a;
  }

  // Clamped into the box. A fraction out of range is a config error, and the failure mode of
  // honouring it literally is aiming at empty scene a whole box-height away from the target,
  // which looks exactly like a tracking failure on the HUD.
  double frac = opt.mode == AimMode::BoxFraction ? opt.y_fraction : opt.head_fraction_from_top;
  if (!(frac >= 0.0)) frac = 0.0;
  if (frac > 1.0) frac = 1.0;

  const double xf = opt.mode == AimMode::BoxFraction ? opt.x_fraction : 0.5;
  if (!std::isfinite(xf) || xf < 0 || xf > 1 || !std::isfinite(frac)) return a;
  a.u_px = (x_min_norm + xf*w) * static_cast<double>(in.width);
  a.v_px = (y_min_norm + frac * h) * static_cast<double>(in.height);
  a.head_applied = opt.mode == AimMode::Legacy;
  a.source = opt.mode == AimMode::BoxFraction ? "box_fraction" : "legacy_head_fraction";
  a.box_clipped = x_min_norm <= 0 || y_min_norm <= 0 || x_max_norm >= 1 || y_max_norm >= 1;
  return a;
}

}  // namespace tracking
}  // namespace ota
