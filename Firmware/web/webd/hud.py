"""The v3.2 Apache-HUD operator page.

`docs/open_auto_turret_v3_2_apache_hud_ui_revision.md` governs presentation and overrides the v3
dashboard, whose engineering cards are the exact "header plus cards" layout §3 forbids. The
engineering numbers are not lost: the old page stays reachable at `/dashboard` until the DIAG
drawer replaces it, and `/api/*` is untouched, so nothing downstream changes.

Two decisions worth their weight, both recorded in code rather than in chat:

**The video is `contain`, not `cover`.** Cover fills the viewport by cropping, and the one thing
this station's operator has to be able to judge is whether a target is about to LEAVE THE FRAME -
the acceptance margin for the lead requirement. Cropping would hide the true frame edge and show a
boundary that is not the camera's. So the whole frame is always visible and the letterbox bars are
the cost, taken deliberately.

**The reticle is drawn at the measured principal point, not at the viewport centre.** §7 says the
reticle is the actual camera optical axis and never the target, prediction or requested LOS. With
today's calibration `cx=960 cy=540` of 1920x1080, which happens to be the centre; the instant a real
principal-point measurement disagrees, the reticle must move and the target boxes must keep mapping
through the same numbers. If telemetry does not carry the intrinsics, the reticle says so out loud
instead of quietly guessing the centre - a HUD symbology that silently relocates itself is worse than
one that admits it is unclosed.
"""
from __future__ import annotations

# ---------------------------------------------------------------------------
# Pure geometry, kept in one string so a test can execute THESE BYTES under node
# and compare them against an independent Python computation. The page and the
# test cannot drift apart, because the test does not reimplement this.
# ---------------------------------------------------------------------------
HUD_GEOMETRY_JS = r"""
// Contain-fit layout of a natural-size image inside a viewport: scale down to fit,
// centre the remainder. Returns the scale and the top-left corner of the image
// inside the viewport, in CSS pixels.
function hudLayout(vw, vh, iw, ih) {
  if (!(vw > 0) || !(vh > 0) || !(iw > 0) || !(ih > 0)) {
    return { ok: false, s: 0, ox: 0, oy: 0, w: 0, h: 0 };
  }
  const s = Math.min(vw / iw, vh / ih);
  const w = iw * s, h = ih * s;
  return { ok: true, s: s, ox: (vw - w) / 2, oy: (vh - h) / 2, w: w, h: h };
}

// Map a normalised image coordinate (u, v share the detector's own frame, so they
// are resolution-independent) to CSS pixels in the viewport. Values outside [0,1]
// are returned unsaturated: the caller decides whether that is an off-screen cue
// or a frame-exit warning, and clamping here would hide both.
function hudProject(u, v, lay) {
  if (!lay || !lay.ok) return { ok: false, x: 0, y: 0 };
  return { ok: true, x: lay.ox + u * lay.w, y: lay.oy + v * lay.h };
}

// Where the optical axis lands, in the same normalised units. This comes from the
// camera's measured principal point, NOT from 0.5 and NOT from the target.
function hudAxisNorm(intr) {
  if (!intr || !(intr.width > 0) || !(intr.height > 0) ||
      !(intr.cx >= 0) || !(intr.cy >= 0)) {
    return null;
  }
  return { u: intr.cx / intr.width, v: intr.cy / intr.height };
}


// --- §21 state deltas and §22 safety presentation -----------------------------
//
// Both builders return data. What the operator sees is derived from it, so "what does the HUD say when
// the station is coasting" is a question with an answer that a test can check without a browser - and
// this is a page whose claims about state have already been wrong once this session for lack of exactly
// that separation.

function hudStateLabel(o) {
  // §21's five states, mapped from what the daemon publishes rather than from the raw phase string.
  // `jogging` is manual_lease_active: a published fact, so "MANUAL / JOG" is read off the station and
  // not inferred from motion that might be a roam or a homing remnant.
  o = o || {};
  const mode = String(o.mode || "").toUpperCase();
  const phase = String(o.phase || "").toUpperCase();
  const auto = mode === "AUTO_TRACK";
  const roam = mode === "AUTO_ROAM";

  if (auto && phase === "TRACK") return { line1: "AUTO TRACK", line2: "TRACKING", named: true };
  if (auto && phase === "COAST") return { line1: "AUTO TRACK", line2: "COASTING", named: true };
  // §21.3 says "TARGET LOST / HOLDING or equivalent compact state". Losing the target is the fact the
  // operator has to act on; it goes on the strong line rather than under a mode name.
  if (auto && phase === "LOST_HOLD") return { line1: "TARGET LOST", line2: "HOLDING", named: true };
  if (auto && phase === "WAIT_TARGET") return { line1: "AUTO TRACK", line2: "WAIT TARGET", named: true };
  if (roam && phase === "SWEEP") return { line1: "AUTO ROAM", line2: "SWEEP", named: true };
  if (mode === "MANUAL") {
    // §21.4 asks for SWEEP LEFT|RIGHT; the daemon publishes no sweep direction, so the direction is
    // left off rather than guessed from the sign of a rate that also moves for other reasons.
    return o.jogging ? { line1: "MANUAL", line2: "JOG", named: true }
                     : { line1: "MANUAL", line2: "HOLD", named: true };
  }

  // Anything §21 does not name keeps the daemon's own word on the second line. A label invented here
  // would sound authoritative about a state nobody specified, which is the failure mode this project
  // keeps meeting: the interface asserting more than the station said.
  return {
    line1: (auto ? "AUTO TRACK" : roam ? "AUTO ROAM" : mode || "--"),
    line2: phase || "--",
    named: false
  };
}

function hudSafetyEdge(t) {
  // §22 requires DERATE to name the relevant travel-tape edge or FOR boundary. The margin that matters
  // is between the COMMAND reference and the soft limit, because that is the quantity the limiter acts
  // on - measuring actual position instead would name an edge the operator has already moved away from.
  t = t || {};
  const d = (r) => (typeof r === "number" ? r * 57.29577951308232 : null);
  const cands = [];
  [["YAW", t.q_ref_yaw_rad, t.q_soft_min_yaw_rad, t.q_soft_max_yaw_rad],
   ["PITCH", t.q_ref_pitch_rad, t.q_soft_min_pitch_rad, t.q_soft_max_pitch_rad]]
    .forEach(function (row) {
      const axis = row[0], q = d(row[1]), lo = d(row[2]), hi = d(row[3]);
      if (q === null || lo === null || hi === null || !(hi > lo)) return;
      cands.push({ axis: axis, side: "MIN", margin_deg: q - lo });
      cands.push({ axis: axis, side: "MAX", margin_deg: hi - q });
    });
  if (!cands.length) return null;
  // Ascending margin: a limit already breached (negative margin) is the one to name, and otherwise the
  // one still ahead. Any ordering that put a far limit ahead of a breached one would point the operator
  // at the wrong edge of the tape while the machine was already past the right one.
  cands.sort((a, b) => a.margin_deg - b.margin_deg);
  const w = cands[0];
  return { axis: w.axis, side: w.side, margin_deg: w.margin_deg };
}

function hudSafetyPresentation(t) {
  // §22's four presentations, plus the two daemon states §22 gives no wording for. tier drives size and
  // placement: normal is a chip, caution a chip with the limit named, prominent is larger amber, and
  // interrupt is a red banner, because a fault stop that shares a chip with "ALLOW" is the "no large
  // banners" rule taken too far - §22 itself asks FAULT to interrupt normal operation.
  t = t || {};
  const a = String(t.safety_action || "").toUpperCase();
  const fault = String(t.fault || "").trim();

  if (fault || a === "FAULT_STOP") {
    // The reason is the point of the panel. A red box that says only FAULT makes the operator go looking
    // for the cause in a log, at the worst moment to be reading logs.
    return { label: "FAULT", reason: fault || "fault stop commanded by controld",
             tone: "red", tier: "interrupt", edge: null };
  }
  if (a === "BRAKE") return { label: "BRAKING", reason: "", tone: "amber", tier: "prominent", edge: null };
  if (a === "DERATE") {
    const e = hudSafetyEdge(t);
    return { label: "DERATE",
             reason: e ? (e.axis + " " + e.side) : "limit not published",
             tone: "amber", tier: "caution", edge: e };
  }
  // §22 names four. The daemon can also answer HOLD and DISABLE, and neither is silently folded into a
  // named one: the daemon's word is shown, so an operator reading "SAFETY HOLD" is reading the station,
  // not my summary of it.
  if (a === "HOLD") return { label: "SAFETY HOLD", reason: "motion held by safety", tone: "amber",
                             tier: "caution", edge: null };
  if (a === "DISABLE") return { label: "DISABLED", reason: "amplifiers disabled", tone: "amber",
                                tier: "prominent", edge: null };
  if (a && a !== "ALLOW" && a !== "NONE" && a !== "?") {
    // A state this file has never seen must not default to green.
    return { label: "SAFETY " + a, reason: "unrecognised safety action", tone: "amber",
             tier: "caution", edge: null };
  }
  return { label: "SAFETY ALLOW", reason: "", tone: "green", tier: "normal", edge: null };
}

// --- §13 dock and §14 drawers --------------------------------------------------
//
// The command list is built as data, not as markup with commands buried in attributes, so that "what
// will this button send to the turret" is a question a test can ask without a browser. The rendering
// is derived from that list. A drawer that looks live and sends nothing - or sends something the daemon
// will refuse - is the failure mode this page keeps meeting, and no screenshot shows it.
//
// Every command name and argument spelling here was read out of the daemon's own handlers rather than
// from prose: select_target takes the DISPLAY INDEX as a number (controld's own refusal says "the label
// on the screen"), manual_jog_start takes yaw+/yaw-/pitch+/pitch- with an optional fine|normal|fast
// profile, manual_step takes yaw+1 / pitch-0.5, set_mode takes MANUAL / AUTO_TRACK / AUTO_ROAM and
// refuses anything else instead of falling back, and STOP MOTION is `hold`.
function hudDockSpecs(o) {
  // §13's five, in the order the revision lists them. Always five, whatever the state: a control that
  // disappears when it is not applicable teaches the operator that the layout is arbitrary.
  const keys = ["TARGETS", "MODE", "MANUAL", "DIAG", "MENU"];
  const open = (o && typeof o.open === "string") ? o.open : null;
  return keys.map((k) => ({ key: k, active: (k === open) }));
}

function hudDrawerActions(name, t) {
  // What this drawer offers, as commands. kind: "act" | "current" | "gated" | "stop" | "danger".
  t = t || {};
  const tracks = Array.isArray(t.tracks) ? t.tracks : [];
  const mode = String(t.operating_mode || "").toUpperCase();
  const inManual = mode === "MANUAL";

  if (name === "TARGETS") {
    const rows = [];
    tracks.forEach((tr) => {
      // A positive display index is the whole eligibility test, because that is what the daemon
      // demands: it refuses 0 and refuses anything non-numeric. Offering such a target would put a
      // live-looking button on the screen whose only possible outcome is a refusal.
      const di = (tr && typeof tr.display_index === "number") ? tr.display_index : 0;
      if (!(di >= 1 && di <= 65535)) return;
      // The selected row carries no command, matching how the MODE drawer marks the active mode: a
      // renderer that only remembers to disable by kind would otherwise still hold a real command for a
      // row that must not send one. The data is the contract the tests read, so it says what is true.
      rows.push({ label: "#" + di + " " + String(tr.label || tr.class_name || "TRACK").toUpperCase(),
                  command: tr.selected ? null : "select_target", arg: String(di),
                  kind: tr.selected ? "current" : "act",
                  note: (typeof tr.confidence === "number" ? Math.round(tr.confidence * 100) + "%" : "") });
    });
    if (!rows.length) rows.push({ label: "NO TARGETS", command: null, kind: "current", note: "" });
    if (tracks.some((x) => x && x.selected)) {
      rows.push({ label: "CLEAR SELECTION", command: "clear_target", arg: "", kind: "act", note: "" });
    }
    return rows;
  }

  if (name === "MODE") {
    // §13: MANUAL / AUTO TRACK / AUTO ROAM. The mode already in force is rendered as selected and sends
    // nothing: re-issuing the current mode is a command with no effect, and a control that appears to
    // act while doing nothing is what the rest of this file is paranoid about.
    return [["MANUAL", "MANUAL"], ["AUTO TRACK", "AUTO_TRACK"], ["AUTO ROAM", "AUTO_ROAM"]]
      .map(function (pair) {
        const shown = pair[0], wire = pair[1], now = (wire === mode);
        return { label: shown, command: now ? null : "set_mode", arg: wire,
                 kind: now ? "current" : "act", note: now ? "ACTIVE" : "" };
      });
  }

  if (name === "MANUAL") {
    // The jogs and steps are gated by the daemon: outside MANUAL it answers ok=false with an
    // explanation. They are still listed - greyed, reason on the row - because a control that silently
    // vanishes is how an operator learns to guess at the interface. STOP is deliberately not in that
    // group: `hold` is accepted in every mode, and a stop that only works in one mode is not a stop.
    const gate = inManual ? "act" : "gated";
    const note = inManual ? "" : "MANUAL MODE ONLY";
    const rows = [
      { label: "YAW LEFT", command: "manual_jog_start", arg: "yaw-", kind: gate, note: note },
      { label: "YAW RIGHT", command: "manual_jog_start", arg: "yaw+", kind: gate, note: note },
      { label: "PITCH UP", command: "manual_jog_start", arg: "pitch+", kind: gate, note: note },
      { label: "PITCH DOWN", command: "manual_jog_start", arg: "pitch-", kind: gate, note: note },
      { label: "STEP YAW +1", command: "manual_step", arg: "yaw+1", kind: gate, note: note },
      { label: "STEP YAW -1", command: "manual_step", arg: "yaw-1", kind: gate, note: note },
      { label: "STEP PITCH +1", command: "manual_step", arg: "pitch+1", kind: gate, note: note },
      { label: "STEP PITCH -1", command: "manual_step", arg: "pitch-1", kind: gate, note: note }
    ];
    rows.push({ label: "STOP MOTION", command: "hold", arg: "", kind: "stop",
                note: inManual ? "" : "ALWAYS AVAILABLE" });
    return rows;
  }

  if (name === "MENU") {
    // Supervisory actions move the turret somewhere the operator did not just ask it to go, so they ask
    // twice. §14 reserves red for stop and fault, so the confirm state - not colour alone - is what
    // signals danger here.
    return [
      { label: "HOME", command: "start_homing", arg: "", kind: "act", note: "" },
      { label: "HOLD / PARK", command: "request_park", arg: "", kind: "danger", note: "CONFIRM TWICE" },
      { label: "SUPERVISORY SHUTDOWN", command: "request_shutdown", arg: "", kind: "danger",
        note: "CONFIRM TWICE" }
    ];
  }

  return [];   // DIAG is read-only telemetry; §14 gives it no actions.
}


function hudDiagRows(t) {
  // §13: DIAG is "engineering telemetry". Read-only, and deliberately the fields an operator is asked to
  // quote when something behaves oddly - rates and limits rather than impressions. Anything the snapshot
  // does not carry shows as "--", never as a zero that would look like a measured stillness.
  t = t || {};
  const d = (r) => (typeof r === "number" ? r * 57.29577951308232 : null);
  const num = (v, dp) => (typeof v === "number" ? d(v).toFixed(dp === undefined ? 2 : dp) : "--");
  return [
    ["MODE / PHASE", String(t.operating_mode || "--") + " / " + String(t.mode_phase || "--")],
    ["Q YAW / PITCH", num(t.q_yaw_rad) + " / " + num(t.q_pitch_rad) + " DEG"],
    ["REF YAW / PITCH", num(t.q_ref_yaw_rad) + " / " + num(t.q_ref_pitch_rad)],
    ["CMD RATE YAW", num(t.q_ref_rate_yaw_rad_s) + " DEG/S"],
    ["CMD ACCEL YAW", num(t.q_ref_accel_yaw_rad_s2) + " DEG/S2"],
    ["LIMITS YAW", num(t.q_soft_min_yaw_rad, 1) + " ... " + num(t.q_soft_max_yaw_rad, 1) + " DEG"],
    ["LIMITS PITCH", num(t.q_soft_min_pitch_rad, 1) + " ... " + num(t.q_soft_max_pitch_rad, 1) + " DEG"],
    ["TRACK RATE", (typeof t.camera_fps === "number" ? t.camera_fps.toFixed(1) : "--") + " HZ"],
    ["SELECTED CONF", (typeof t.selected_confidence === "number"
                        ? Math.round(t.selected_confidence * 100) + "%" : "--")],
    ["PREDICTION HORIZON", (typeof t.prediction_horizon_ms === "number"
                            ? String(t.prediction_horizon_ms) + " MS" : "--")],
    // Geometry staleness belongs on the engineering panel: every pixel->ray conversion the HUD draws
    // rests on this file, and "measured once, eight months ago" changes how much the centring margin
    // means. Unknown is shown as UNKNOWN, never as a fresh zero.
    // The limit, not just the effort. "CMD RATE YAW" says what the controller asked for; without the
    // ceiling in force beside it, a station held at a third of its configured tracking speed looks like a
    // sluggish controller rather than like a number on the screen. Unknown is UNKNOWN, never 0 - a zero
    // ceiling would read as "forbidden to move", which is a different claim entirely.
    // 200 Hz as the architecture measures it: not "did a cycle take longer than the period" (this host
    // runs ~198 Hz constantly and the design forgives that), but how many consecutive cycles blew the
    // grace, against the grace itself and the count that triggers a Hold. "0/5" is healthy, not empty.
    ["LOOP DEADLINE", (typeof t.control_deadline_misses === "number" &&
                       typeof t.control_deadline_miss_limit === "number"
      ? t.control_deadline_misses + "/" + t.control_deadline_miss_limit +
        "  (+" + (t.control_deadline_grace_us || 0) + "us grace)"
      : "UNKNOWN")],
    // NOT a ceiling, and the wording is the point. This is SafetyEnvelope's v_max, which the code reaches
    // only when travel limits are unknown (safety_envelope.hpp:120 - if (!lim.valid) return p_.v_max_rad_s),
    // and it is written by apply_payload_derate(), not by any mode path. With valid soft limits the speed
    // comes from the braking model against real travel, so this value is not binding. Round 36 measured the
    // tracking reference at 18.56 deg/s while this field read 10.0, so calling it a "ceiling" told the
    // operator the opposite of what the machine was doing. Unknown stays UNKNOWN, never 0.
    ["ENVELOPE V-MAX", (typeof t.envelope_v_max_deg_s === "number"
      ? t.envelope_v_max_deg_s.toFixed(1) + " DEG/S" +
        (t.soft_limits_valid === true ? "  (not in force)"
                                      : "  (FALLBACK IN FORCE: travel limits unknown)") +
        (typeof t.intent_velocity_scale === "number"
          ? "  AUTH " + Math.round(t.intent_velocity_scale * 100) + "%" : "")
      : "UNKNOWN")],
    ["GEOMETRY AGE", (t.camera && typeof t.camera.measurement_age_ms === "number")
      ? (t.camera.measurement_age_ms >= 86400000
          ? (t.camera.measurement_age_ms / 86400000).toFixed(2) + " D"
          : (t.camera.measurement_age_ms / 3600000).toFixed(1) + " H")
      : "UNKNOWN"],
    ["IMU", (t.imu && t.imu.present) ? "PRESENT" : "ABSENT"]
  ];
}

// --- §10 prediction cue ----------------------------------------------------
//
// The one element on this page whose colour is a safety statement: §10 says the prediction "must not
// be green", because green on this display means measured - something the camera sees now. A
// prediction drawn in green is an intention wearing the uniform of an observation, and the operator
// cannot tell them apart at a glance. So the colour assertions below are not decoration.
function hudPredictionBox(o) {
  // Where the dashed square goes. o = { cx, cy, w, h, box: [x0,y0,x1,y1], gap }
  //
  // "Placed near the selected target but not touching its box" is a geometric rule, so it is
  // implemented as one: the cue is centred on the predicted anchor, and if that overlaps the
  // measured box it is shoved outward along the direction the prediction is pointing - the
  // direction the operator needs to read anyway - until the gap clears. When the prediction is
  // almost exactly on the target (a still target, a settled loop) there is no meaningful direction,
  // so it goes right and up, and says so by reporting shifted.
  if (!o || typeof o.cx !== "number" || typeof o.cy !== "number" || !(o.w > 0) || !(o.h > 0)) return null;
  const box = (Array.isArray(o.box) && o.box.length === 4) ? o.box : null;
  let x = o.cx - o.w / 2, y = o.cy - o.h / 2;
  const gap = (typeof o.gap === "number") ? o.gap : 8.0;
  let shifted = false;
  if (box) {
    const bx0 = Math.min(box[0], box[2]), bx1 = Math.max(box[0], box[2]);
    const by0 = Math.min(box[1], box[3]), by1 = Math.max(box[1], box[3]);
    const bcx = (bx0 + bx1) / 2, bcy = (by0 + by1) / 2;
    let guard = 0;
    while (x < bx1 + gap && x + o.w > bx0 - gap && y < by1 + gap && y + o.h > by0 - gap && guard < 400) {
      let dx = o.cx - bcx, dy = o.cy - bcy;
      if (Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6) { dx = 1.0; dy = -0.5; }   // no direction to use
      const n = Math.hypot(dx, dy) || 1.0;
      x += (dx / n) * 4.0; y += (dy / n) * 4.0; shifted = true; ++guard;
    }
  }
  return { x: x, y: y, w: o.w, h: o.h, cx: x + o.w / 2, cy: y + o.h / 2, shifted: shifted };
}

function hudPredictionSvg(b, C, o) {
  // §10's shape: amber dashed square, small amber + at its centre, small PRED label.
  if (!b) return "";
  const amber = C.amber;
  const parts = [
    '<rect x="' + b.x + '" y="' + b.y + '" width="' + b.w + '" height="' + b.h +
      '" fill="none" stroke="' + amber + '" stroke-width="1.2" stroke-dasharray="6 4" opacity=".92"/>',
    '<line x1="' + (b.cx - 5) + '" y1="' + b.cy + '" x2="' + (b.cx + 5) + '" y2="' + b.cy +
      '" stroke="' + amber + '" stroke-width="1.2"/>',
    '<line x1="' + b.cx + '" y1="' + (b.cy - 5) + '" x2="' + b.cx + '" y2="' + (b.cy + 5) +
      '" stroke="' + amber + '" stroke-width="1.2"/>',
    '<text class="tlbl" x="' + (b.x + b.w / 2) + '" y="' + (b.y - 5) +
      '" text-anchor="middle" fill="' + amber + '">PRED</text>'
  ];
  // "A long error vector across the image is not shown by default. If a short connector is used, it
  // should be very subtle." So: only when the two are already close, and at a quarter opacity. When
  // the prediction is far away - mid-dart, which is exactly when it is most tempting to draw the
  // line - the cue stands on its own.
  if (o && o.near && o.near.length === 2) {
    const d = Math.hypot(o.near[0] - b.cx, o.near[1] - b.cy);
    if (d <= (o.nearMax || 140.0)) {
      parts.unshift('<line x1="' + o.near[0] + '" y1="' + o.near[1] + '" x2="' + b.cx + '" y2="' +
                    b.cy + '" stroke="' + amber + '" stroke-width="1" opacity=".28"/>');
    }
  }
  return parts.join("");
}

// --- §11 field-of-regard inset ------------------------------------------------
//
// Everything in this inset is in LOGICAL JOINT DEGREES, which is what §11.3 asks for and also the
// only space this station can honestly fill: the camera-to-axis boresight is not separable from the
// principal point at the spans the theodolite probe reaches, so an envelope drawn over the picture
// would inherit an offset nobody has measured. In joint degrees every number comes from the encoders,
// and the tape caveat - "JOINT TRAVEL, NOT HEADING" - applies here too: the axes are labelled by
// their own travel, and no cardinal direction appears anywhere.
//
// The scale is ONE number for both axes. Fitting yaw and pitch independently would fill the box more
// attractively and would make the white FOV rectangle a lie - the rectangle's job is to show how big
// the camera's view is next to the region the turret can point, and that comparison is only true if
// both axes share a scale. So the envelope is letterboxed instead of stretched, and the test asserts
// the aspect ratio of the rectangle rather than how nicely it fills the box.
function hudForInset(o) {
  if (!o || !Array.isArray(o.pts) || o.pts.length < 3 || !(o.hfovDeg > 0) || !(o.vfovDeg > 0) ||
      !Array.isArray(o.los)) return null;
  const vw = o.vw, vh = o.vh;
  if (!(vw > 0) || !(vh > 0)) return null;

  const w = vw * 0.26, h = vh * 0.215;             // §11.2: 25-27% wide, 20-23% tall
  const x = vw * 0.015;                            // "lower left", clear of the bezel
  const y = vh - h - vh * 0.055;                   // above the §12 status strip
  const titleH = Math.max(12, h * 0.17);
  const pad = Math.max(6, Math.min(w, h) * 0.08);
  const plot = { x: x + pad, y: y + titleH, w: w - 2 * pad, h: h - titleH - pad };
  if (!(plot.w > 0) || !(plot.h > 0)) return null;

  let minY = Infinity, maxY = -Infinity, minP = Infinity, maxP = -Infinity;
  for (const pt of o.pts) {
    // typeof checks, not isFinite: in JavaScript isFinite(null) is TRUE, because Number(null) is 0.
    // A malformed vertex would otherwise have been plotted at yaw 0 as a legitimate corner, which is
    // the worst kind of wrong on a map whose entire job is saying where the turret may point.
    if (!Array.isArray(pt) || pt.length < 2 || typeof pt[0] !== "number" ||
        typeof pt[1] !== "number" || !isFinite(pt[0]) || !isFinite(pt[1])) return null;
    minY = Math.min(minY, pt[0]); maxY = Math.max(maxY, pt[0]);
    minP = Math.min(minP, pt[1]); maxP = Math.max(maxP, pt[1]);
  }
  const spanY = Math.max(1e-6, maxY - minY), spanP = Math.max(1e-6, maxP - minP);
  const k = Math.min(plot.w / spanY, plot.h / spanP);          // the one shared scale
  const cx = plot.x + plot.w / 2, cy = plot.y + plot.h / 2;
  const midY = (minY + maxY) / 2, midP = (minP + maxP) / 2;
  const toPx = (yawDeg, pitchDeg) => [cx + (yawDeg - midY) * k, cy - (pitchDeg - midP) * k];

  const clampMark = (pt) => {
    // Off-map is information, not an error: a target the axis cannot reach is exactly what an
    // operator needs to see, and silently dropping it would make the inset read as "no target".
    // So it is pinned to the border and flagged, which is the same honesty rule as the off-screen
    // target cue in the main view.
    const raw = toPx(pt[0], pt[1]);
    const off = raw[0] < plot.x || raw[0] > plot.x + plot.w ||
                raw[1] < plot.y || raw[1] > plot.y + plot.h;
    return { x: Math.min(Math.max(raw[0], plot.x), plot.x + plot.w),
             y: Math.min(Math.max(raw[1], plot.y), plot.y + plot.h),
             off: off, yawDeg: pt[0], pitchDeg: pt[1] };
  };

  const los = clampMark(o.los);
  const fovW = o.hfovDeg * k, fovH = o.vfovDeg * k;            // §11.3: size from effective HFOV/VFOV
  const g = {
    x: x, y: y, w: w, h: h, plot: plot, scale: k,
    envPx: o.pts.map((pt) => { const q = toPx(pt[0], pt[1]); return { x: q[0], y: q[1] }; }),
    fov: { x: los.x - fovW / 2, y: los.y - fovH / 2, w: fovW, h: fovH },
    los: los,
    target: Array.isArray(o.target) ? clampMark(o.target) : null,
    pred: Array.isArray(o.pred) ? clampMark(o.pred) : null,
    titleH: titleH
  };
  return g;
}

function hudForInsetSvg(g, C) {
  if (!g) return "";
  const pts = g.envPx.map((p) => p.x.toFixed(1) + "," + p.y.toFixed(1)).join(" ");
  const parts = [
    // "It is not a full dashboard card" - one low-opacity panel, no card chrome, so the scene stays
    // dominant as §11.2 requires.
    '<rect x="' + g.x + '" y="' + g.y + '" width="' + g.w + '" height="' + g.h + '" rx="2" ' +
      'fill="' + C.black + '" fill-opacity=".28" stroke="' + C.line + '" stroke-width="1"/>',
    '<text class="tlbl" x="' + (g.x + g.w / 2) + '" y="' + (g.y + g.titleH * 0.72) +
      '" text-anchor="middle" fill="' + C.dim + '">FIELD OF REGARD</text>',
    '<polygon points="' + pts + '" fill="' + C.green + '" fill-opacity=".10" stroke="' + C.green +
      '" stroke-width="1" opacity=".85"/>',
    '<text class="tlbl" x="' + (g.plot.x + 2) + '" y="' + (g.plot.y + g.plot.h - 3) +
      '" fill="' + C.green + '" opacity=".8">SAFE ENVELOPE</text>',
    '<rect x="' + g.fov.x + '" y="' + g.fov.y + '" width="' + g.fov.w + '" height="' + g.fov.h +
      '" fill="none" stroke="' + C.white + '" stroke-width="1" opacity=".95"/>',
    '<line x1="' + (g.los.x - 4) + '" y1="' + g.los.y + '" x2="' + (g.los.x + 4) + '" y2="' +
      g.los.y + '" stroke="' + C.white + '" stroke-width="1.2"/>',
    '<line x1="' + g.los.x + '" y1="' + (g.los.y - 4) + '" x2="' + g.los.x + '" y2="' +
      (g.los.y + 4) + '" stroke="' + C.white + '" stroke-width="1.2"/>'
  ];
  if (g.target) {
    parts.push('<circle cx="' + g.target.x + '" cy="' + g.target.y + '" r="4" fill="none" stroke="' +
               C.green + '" stroke-width="1.4"' + (g.target.off ? ' opacity=".45"/>' : '/>'));
  }
  if (g.pred) {
    // Amber, and shaped differently from the target marker: two green symbols side by side would put
    // an intention and a measurement in the same uniform, which §10 already forbids in the main view.
    const d = 4.5, q = g.pred;
    parts.push('<path d="M' + q.x + ' ' + (q.y - d) + 'L' + (q.x + d) + ' ' + q.y + 'L' + q.x + ' ' +
               (q.y + d) + 'L' + (q.x - d) + ' ' + q.y + 'Z" fill="none" stroke="' + C.amber +
               '" stroke-width="1.4"' + (q.off ? ' opacity=".45"/>' : '/>'));
  }
  // §11's "short legend": three rows, no paragraphs.
  const lx = g.x + g.w - Math.max(52, g.w * 0.20), ly = g.y + g.titleH + 3;
  parts.push('<line x1="' + lx + '" y1="' + ly + '" x2="' + (lx + 9) + '" y2="' + ly +
             '" stroke="' + C.white + '" stroke-width="1"/><text class="flbl" x="' + (lx + 12) +
             '" y="' + (ly + 3) + '" fill="' + C.dim + '">FOV</text>');
  parts.push('<circle cx="' + (lx + 4) + '" cy="' + (ly + 12) + '" r="3.4" fill="none" stroke="' +
             C.green + '" stroke-width="1.2"/><text class="flbl" x="' + (lx + 12) + '" y="' +
             (ly + 15) + '" fill="' + C.dim + '">TARGET</text>');
  parts.push('<path d="M' + (lx + 4) + ' ' + (ly + 18) + 'L' + (lx + 9) + ' ' + (ly + 23) + 'L' +
             (lx + 4) + ' ' + (ly + 28) + 'L' + (lx - 1) + ' ' + (ly + 23) + 'Z" fill="none" stroke="' +
             C.amber + '" stroke-width="1.2"/><text class="flbl" x="' + (lx + 12) + '" y="' +
             (ly + 26) + '" fill="' + C.dim + '">PRED</text>');
  return parts.join("");
}

// --- §5 / §6 travel tapes --------------------------------------------------
//
// Geometry and drawing, both as pure functions, deliberately. The revision specifies these tapes
// numerically - "upper 10-15% of the viewport", "middle 55-60% of the image width", endpoints that
// "always show the software-safe travel limits" - and a claim written that specific is supposed to be
// checkable. Keeping the maths and the markup out of the render path means node can execute them
// against a real telemetry payload and assert the result, which is the closest thing to looking at
// the page that exists in this environment.
// HUD_R2D is declared above, beside the other shared constants; declaring it again here would be a
// SyntaxError at load, which in a page script means the whole HUD silently draws nothing.

function hudTickSteps(spanDeg, px) {
  // "Fine tick marks at small angular increments; coarse ticks and labels at meaningful intervals"
  // is not a number, so the number here is chosen from what the tape can actually show: the smallest
  // step that keeps labels at least 52 px apart, which is roughly the width of "+120 deg" at the
  // label size in §16. A fixed 20 deg would either collide on a narrow safe range or produce four
  // ticks across a 200 deg one.
  const steps = [5, 10, 15, 20, 30, 45, 60, 90];
  let coarse = steps[steps.length - 1];
  for (let i = 0; i < steps.length; ++i) {
    if (spanDeg > 0 && (steps[i] / spanDeg) * px >= 52.0) { coarse = steps[i]; break; }
  }
  return { coarse: coarse, fine: Math.max(1.0, coarse / 4.0) };
}

function hudTravelTape(o) {
  // One function for both tapes: the revision gives the yaw tape and the pitch tape the same
  // content and the same hierarchy, and a second near-identical implementation is how the two drift
  // apart into showing different truths about the same travel.
  //
  //   o = { horizontal, x, y, length, minDeg, maxDeg, valueDeg, valid }
  //
  // Returns null when there is no ranged travel to show. That is a real state, not an error: before
  // homing, `soft_limits_valid` is false and the bounds are unset, and drawing invented endpoints
  // would name a limit this machine was never homed to. The caller draws the refusal instead.
  if (!o || !o.valid || !(o.maxDeg > o.minDeg) || !(o.length > 0)) return null;

  const span = o.maxDeg - o.minDeg;
  const steps = hudTickSteps(span, o.length);
  const at = (deg) => o.horizontal
    ? o.x + ((deg - o.minDeg) / span) * o.length          // left = min
    : o.y + ((o.maxDeg - deg) / span) * o.length;         // top = max, so up is up (§6.2)

  const ticks = [];
  const firstIdx = Math.ceil(o.minDeg / steps.fine - 1e-9);
  const lastIdx = Math.floor(o.maxDeg / steps.fine + 1e-9);
  for (let k = firstIdx; k <= lastIdx; ++k) {
    const deg = k * steps.fine;
    const coarse = Math.abs(deg / steps.coarse - Math.round(deg / steps.coarse)) < 1e-9;
    ticks.push({ deg: deg, pos: at(deg), coarse: coarse, endpoint: false,
                 label: coarse ? hudDegLabel(deg, false) : "" });
  }
  // Endpoints are always present and always labelled (§5.2), whether or not they fall on a step.
  [{ deg: o.minDeg, pos: o.horizontal ? o.x : o.y + o.length },
   { deg: o.maxDeg, pos: o.horizontal ? o.x + o.length : o.y }].forEach((e) => {
    const dupe = ticks.some((t) => Math.abs(t.deg - e.deg) < 1e-6);
    if (dupe) { ticks.filter((t) => Math.abs(t.deg - e.deg) < 1e-6).forEach((t) => {
      t.endpoint = true; t.coarse = true; t.label = hudDegLabel(e.deg, true); }); }
    else ticks.push({ deg: e.deg, pos: e.pos, coarse: true, endpoint: true,
                      label: hudDegLabel(e.deg, true) });
  });
  ticks.sort((a, b) => a.pos - b.pos);

  // §22 asks a DERATE indication to include the relevant travel-tape edge. The tape's ends ARE the soft
  // limits - the same numbers the limiter is acting on - so the honest way to do this is to light up the
  // end being named, not to add a separate marker that could disagree with the scale it sits on.
  // Matched on the degree value rather than on index, because which tick is an endpoint depends on
  // whether the limit happened to fall on a fine step.
  if (typeof o.markDeg === "number") {
    ticks.forEach((tk) => { tk.marked = Math.abs(tk.deg - o.markDeg) < 1e-6; });
  }

  // Clamped along the tape's own axis. The first version clamped the vertical case between o.x and
  // o.x - the line's own column - because the horizontal variable was reused without being thought
  // about, and every pitch marker collapsed onto the tape's x-coordinate. Hand arithmetic caught it
  // (expected 593.7, produced 1842.0); a test now carries that arithmetic.
  const lo = o.horizontal ? o.x : o.y;
  const hi = o.horizontal ? o.x + o.length : o.y + o.length;
  const marker = Math.max(lo, Math.min(hi, at(o.valueDeg)));   // never point off the tape
  return {
    horizontal: !!o.horizontal, x: o.x, y: o.y, length: o.length,
    x1: o.horizontal ? o.x + o.length : o.x, y1: o.horizontal ? o.y : o.y + o.length,
    minDeg: o.minDeg, maxDeg: o.maxDeg, steps: steps, ticks: ticks, marker: marker,
    valueDeg: o.valueDeg,
    // §6.3: the value box is a dark translucent fill with a thin green outline. Sized for
    // "PITCH -12.3 deg" at the label size, and always placed where it cannot leave the viewport.
    box: { w: 96, h: 34, x: 0, y: 0 },
    note: ""
  };
}

function hudDegLabel(deg, withDegree) {
  const v = Math.abs(deg) < 1e-9 ? 0 : deg;
  const txt = (v > 0 ? "+" : (v < 0 ? "-" : "")) + Math.abs(v).toFixed(Math.abs(v) % 1 ? 1 : 0);
  return txt + (withDegree ? "\u00b0" : "");
}

function hudTravelTapeSvg(t, C, opts) {
  // Drawing only: every number above came from hudTravelTape, so what the test executes is what the
  // page draws rather than a description of it.
  if (!t) return "";
  const base = C.green, fine = C.dim, lbl = C.green, mark = C.white;
  const parts = [];
  const w = t.horizontal;
  parts.push('<line ' + (w ? 'x1="' + t.x + '" y1="' + t.y + '" x2="' + t.x1 + '" y2="' + t.y
                           : 'x1="' + t.x + '" y1="' + t.y + '" x2="' + t.x + '" y2="' + t.y1) +
             '" stroke="' + base + '" stroke-width="1" opacity=".85"/>');
  t.ticks.forEach((tk) => {
    const len = tk.marked ? 17 : (tk.endpoint ? 13 : (tk.coarse ? 10 : 5));
    const col = tk.marked ? C.amber : (tk.coarse ? base : fine);   // §22: caution is amber
    parts.push('<line ' + (w ? 'x1="' + tk.pos + '" y1="' + t.y + '" x2="' + tk.pos + '" y2="' + (t.y + len)
                           : 'x1="' + t.x + '" y1="' + tk.pos + '" x2="' + (t.x - len) + '" y2="' + tk.pos) +
               '" stroke="' + col + '" stroke-width="1"/>');
    if (tk.label) {
      parts.push('<text class="tlbl" ' +
        (w ? 'x="' + tk.pos + '" y="' + (t.y - 7) + '" text-anchor="middle"'
           : 'x="' + (t.x + 8) + '" y="' + (tk.pos + 4) + '" text-anchor="start"') +
        ' fill="' + (tk.marked ? C.amber : lbl) + '">' + tk.label + '</text>');
    }
  });
  // Current-position caret (§5.2) and its value box. Drawn last inside the group so it sits over the
  // ticks it overlaps.
  const mk = t.marker;
  parts.push(w
    ? '<path d="M ' + mk + ' ' + (t.y + 2) + ' L ' + (mk - 6) + ' ' + (t.y + 12) + ' L ' +
      (mk + 6) + ' ' + (t.y + 12) + ' Z" fill="' + C.green + '" stroke="' + mark +
      '" stroke-width=".8"/>'
    : '<path d="M ' + (t.x - 2) + ' ' + mk + ' L ' + (t.x - 12) + ' ' + (mk - 6) + ' L ' +
      (t.x - 12) + ' ' + (mk + 6) + ' Z" fill="' + C.green + '" stroke="' + mark +
      '" stroke-width=".8"/>');
  const bx = w ? Math.max(4, Math.min(mk - 48, (opts && opts.vw ? opts.vw - 100 : mk)))
               : Math.max(4, t.x - 84);
  const by = w ? (t.y + 16) : Math.min(t.y1 + 10, (opts && opts.vh ? opts.vh - 44 : t.y1));
  parts.push('<rect x="' + bx + '" y="' + by + '" width="' + t.box.w + '" height="' + t.box.h +
             '" fill="' + C.black + '" stroke="' + C.green + '" stroke-width="1" rx="2"/>');
  parts.push('<text class="tval" x="' + (bx + t.box.w / 2) + '" y="' + (by + 14) +
             '" text-anchor="middle" fill="' + C.green + '">' + (opts && opts.title ? opts.title : "") +
             '</text>');
  parts.push('<text class="tval" x="' + (bx + t.box.w / 2) + '" y="' + (by + 28) +
             '" text-anchor="middle" fill="' + C.white + '">' + (opts && opts.value ? opts.value : "") +
             '</text>');
  // What the scale actually is, stated on the tape that uses it. §5.3 asks for logical joint travel
  // and forbids compass letters, which the drawing honours - but on this station the joint numbers
  // are surprising enough to be misread: yaw travels -22.6 to +320.2 deg (the config says in terms:
  // "YAW IS A ~360 DEG CONTINUOUS-ROTATION AXIS") and pitch sits -74.7 to -4.9, which an operator
  // will read as elevation unless told otherwise. It is not elevation. The theodolite probe records
  // that camera-to-axis boresight is NOT separable from the principal point at the spans available
  // here, so the world elevation of this scale's zero has never been measured, and the tape says so
  // rather than borrowing an offset from somebody's recollection - mine included.
  if (opts && opts.note) {
    parts.push('<text class="tlbl" x="' + (bx + t.box.w / 2) + '" y="' + (by + t.box.h + 13) +
               '" text-anchor="middle" fill="' + C.dim + '">' + opts.note + '</text>');
  }
  return parts.join("");
}

function hudUnrangedNote(x, y, label) {
  // What replaces a tape that has no endpoints to show. Silence here would read as a target-free
  // sky rather than as an un-commissioned axis.
  return '<text class="tlbl" x="' + x + '" y="' + y + '" text-anchor="middle" fill="' +
    "#f2b329" + '">' + label + ' TAPE: TRAVEL UNRANGED (home the turret)</text>';
}

// §25: "stale telemetry stops visual interpolation and indicates stale/disconnected state".
//
// A pure function of what is known, rather than three comparisons scattered through the render path,
// for two reasons. The first is that the whole rule can then be executed and tested outside a
// browser, which is more than the previous version of this page could say about its own staleness
// logic. The second is that the rule has to be one thing: it is answered differently by three
// sources that fail differently, and a rule written three times drifts.
//
//  - transportOk: webd's health says controld is gone, or the socket closed. Announces itself.
//  - telemetryStale / telemetryAgeMs: webd's own judgement, measured from when the frame ARRIVED,
//    polled from /api/health so it stays live even when no frames are coming. This is the case a
//    socket cannot cover: a controld that hangs while holding the connection open stays
//    "connected" forever, and the reticle sits still on live video looking exactly like a target
//    that has stopped moving.
//  - msgAgeMs: the link to THIS page went quiet. Independent of everything above, because the
//    failure can be only between webd and the browser, where the server's opinion is unreachable
//    by definition.
//  - trackListAgeMs: not staleness of the whole picture but of the target list specifically, which
//    controld measures itself. Kept because losing tracks while the attitude stays live is a
//    different and very real emergency - the reticle would still be honest, the boxes would not.
function hudStale(o) {
  if (!o || o.transportOk === false) return true;
  if (o.telemetryStale === true) return true;
  if (typeof o.telemetryAgeMs === "number" && o.telemetryAgeMs > o.staleAfterMs) return true;
  if (typeof o.msgAgeMs === "number" && o.msgAgeMs > o.quietAfterMs) return true;
  if (typeof o.trackListAgeMs === "number" && o.trackListAgeMs > o.trackAfterMs) return true;
  return false;
}
"""


HUD_JS = HUD_GEOMETRY_JS + r"""
const $ = (id) => document.getElementById(id);

function fmt(v, digits, suffix) {
  if (typeof v !== "number" || !isFinite(v)) return "--";
  return v.toFixed(digits) + (suffix || "");
}

function deg(rad) { return rad * 180.0 / Math.PI; }

// §15 color tokens, verbatim from the revision.
const C = {
  green: "#95f58b", dim: "rgba(149,245,139,.56)", faint: "rgba(149,245,139,.22)",
  amber: "#f2b329", red: "#ff5d5d", white: "#edf2eb", black: "rgba(3,6,5,.80)",
  line: "rgba(230,245,230,.24)"
};

let lastTelemetry = null;
let lastTelemetryAt = 0;
let transportOk = true;      // false => show the stale/disconnected state (§25)
let healthAgeMs = null;      // webd's own age of controld's data, from /api/health (§25)

const STALE_AFTER_MS = 500;  // webd's threshold, mirrored so server and page agree
const QUIET_AFTER_MS = 1500; // silence on the link to THIS page: three times the server's own
                             // threshold, so when webd can see the problem its verdict arrives
                             // first, and under the 2 s health poll, so the page never has to wait
                             // on polling alone to notice that nothing is coming.
const TRACK_AFTER_MS = 500;  // target-list staleness, as measured and reported by controld

function chip(label, state, value) {
  // §8: small translucent chips with a status dot; near-white text; no header bar.
  const d = document.createElement("div");
  d.className = "chip";
  const dot = state === "red" ? C.red : (state === "amber" ? C.amber : C.green);
  d.innerHTML = '<span class="dot" style="background:' + dot + '"></span>' +
                '<span class="lbl">' + label + '</span>' +
                (value ? '<span class="val">' + value + '</span>' : "");
  return d;
}

function render(t) {
  const video = $("video"), svg = $("overlay");
  if (!video || !svg) return;

  const vw = window.innerWidth, vh = window.innerHeight;
  const iw = video.naturalWidth || 0, ih = video.naturalHeight || 0;
  const lay = hudLayout(vw, vh, iw, ih);
  svg.setAttribute("viewBox", "0 0 " + vw + " " + vh);
  svg.setAttribute("width", vw);
  svg.setAttribute("height", vh);

  const stale = updateStaleness(t);

  // --- what the overlay is made of, rebuilt each frame -------------------
  const layers = { cand: "", sel: "", reticle: "", pred: "", for: "", tape: "" };

  // §7 + §9: boxes are drawn from the detector's own normalised bbox; the anchor is
  // the point the controller centres, and it is drawn - never as a dot on the reticle.
  const tracks = Array.isArray(t.tracks) ? t.tracks : [];
  tracks.forEach((tr) => {
    const ax = tr.anchor_x, ay = tr.anchor_y;
    if (typeof ax !== "number" || typeof ay !== "number") return;
    const bb = Array.isArray(tr.bbox) && tr.bbox.length === 4 ? tr.bbox : null;
    const selected = !!tr.selected;
    const st = String(tr.state || "").toUpperCase();
    const inside = ax >= 0 && ax <= 1 && ay >= 0 && ay <= 1;
    const conf = (typeof tr.confidence === "number") ? Math.round(tr.confidence * 100) + "%" : "";
    const label = String(tr.label || tr.display_index || ("#" + (tr.track_id || "?"))).toUpperCase();

    if (!inside) {
      // §73's off-screen cue, kept from v3: a selected target the camera is not looking
      // at must not look like a dropped frame.
      const e = hudProject(Math.min(Math.max(ax, 0), 1), Math.min(Math.max(ay, 0), 1), lay);
      if (e.ok) layers.cand += '<circle cx="' + e.x + '" cy="' + e.y + '" r="7" fill="none" ' +
        'stroke="' + C.amber + '" stroke-width="1.5"/>';
      return;
    }
    if (!bb) return;
    const p0 = hudProject(bb[0], bb[1], lay), p1 = hudProject(bb[2], bb[3], lay);
    const a = hudProject(ax, ay, lay);
    if (!p0.ok || !p1.ok) return;
    const w = p1.x - p0.x, h = p1.y - p0.y;
    const stroke = selected ? C.green : C.dim;
    const dash = selected ? "" : ' stroke-dasharray="6 5"';
    const glow = selected ? ' filter="url(#softglow)"' : "";
    const group = (selected ? "sel" : "cand");
    layers[group] +=
      '<g' + glow + '>' +
      '<rect x="' + p0.x + '" y="' + p0.y + '" width="' + w + '" height="' + h +
      '" fill="none" stroke="' + stroke + '" stroke-width="' + (selected ? 2 : 1) + '"' +
      dash + '/>' +
      '<text x="' + p0.x + '" y="' + (p0.y - 6) + '" class="lbl" fill="' + stroke + '">' +
      label + ' ' + conf + '</text>' +
      '<line x1="' + (a.x - 7) + '" y1="' + a.y + '" x2="' + (a.x + 7) + '" y2="' + a.y +
      '" stroke="' + stroke + '" stroke-width="1"/>' +
      '<line x1="' + a.x + '" y1="' + (a.y - 7) + '" x2="' + a.x + '" y2="' + (a.y + 7) +
      '" stroke="' + stroke + '" stroke-width="1"/>' +
      '</g>';
  });

  // §7: the optical axis. Sparse brackets, verticals above and below, short bars
  // left and right, open centre - and never on the target.
  const intr = t.camera_intrinsics;
  const axis = hudAxisNorm(intr) || { u: 0.5, v: 0.5 };
  const c = hudProject(axis.u, axis.v, lay);
  if (c.ok) {
    const g = C.green, r = 26, gap = 8, len = 12;
    const corner = (sx, sy) =>
      '<path d="M ' + (c.x + sx * r) + ' ' + (c.y + sy * gap) + ' L ' + (c.x + sx * r) + ' ' +
      (c.y + sy * r) + ' L ' + (c.x + sx * gap) + ' ' + (c.y + sy * r) + '" fill="none" ' +
      'stroke="' + g + '" stroke-width="1.5"/>';
    layers.reticle =
      corner(-1, -1) + corner(1, -1) + corner(-1, 1) + corner(1, 1) +
      '<line x1="' + c.x + '" y1="' + (c.y - r - 12) + '" x2="' + c.x + '" y2="' + (c.y - gap) +
      '" stroke="' + g + '" stroke-width="1"/>' +
      '<line x1="' + c.x + '" y1="' + (c.y + gap) + '" x2="' + c.x + '" y2="' + (c.y + r + 12) +
      '" stroke="' + g + '" stroke-width="1"/>' +
      '<line x1="' + (c.x - r - 12) + '" y1="' + c.y + '" x2="' + (c.x - gap - 8) + '" y2="' + c.y +
      '" stroke="' + g + '" stroke-width="1"/>' +
      '<line x1="' + (c.x + gap + 8) + '" y1="' + c.y + '" x2="' + (c.x + r + 12) + '" y2="' + c.y +
      '" stroke="' + g + '" stroke-width="1"/>' +
      (intr ? "" : '<text x="' + (c.x + r + 18) + '" y="' + (c.y + 4) + '" class="lbl" ' +
        'fill="' + C.amber + '">RETICLE UNCALIBRATED (assumed centre)</text>');
  }

  // §10: the prediction cue, from webd's `prediction` block. Absent when invalid - the revision says
  // prediction disappears when invalid or stale (§661's rule), and an empty group is the honest
  // rendering of "the controller is not predicting anything right now".
  // §10: the prediction cue, from webd's `prediction` block. Two gates, both the controller's:
  // `valid` means it predicted anything at all, and `anchor_in_frame` means that point is on the
  // picture rather than off the edge of it. Prediction disappears when there is nothing to predict
  // (the revision's own rule at §661), and an empty group is the honest rendering of that.
  const pred = (t.prediction && typeof t.prediction === "object") ? t.prediction : null;
  layers.pred = "";
  if (pred && pred.valid === true && pred.anchor_in_frame === true &&
      Array.isArray(pred.predicted_anchor_norm) && pred.predicted_anchor_norm.length === 2 && !stale) {
    const a = hudProject(pred.predicted_anchor_norm[0], pred.predicted_anchor_norm[1], lay);
    if (a.ok) {
      // The cue is the size of the target it predicts, so the operator is comparing like with like:
      // a dashed box where the same body will be, against the solid box where it is.
      const sel = tracks.filter((x) => x && x.selected && Array.isArray(x.bbox) &&
                                      x.bbox.length === 4)[0] || null;
      let w = 48.0, h = 48.0, near = null;
      if (sel) {
        const p0 = hudProject(sel.bbox[0], sel.bbox[1], lay);
        const p1 = hudProject(sel.bbox[2], sel.bbox[3], lay);
        if (p0.ok && p1.ok) {
          w = Math.max(24.0, p1.x - p0.x);
          h = Math.max(24.0, p1.y - p0.y);
          near = [(p0.x + p1.x) / 2, (p0.y + p1.y) / 2];
        }
      }
      layers.pred = hudPredictionSvg(
        hudPredictionBox({ cx: a.x, cy: a.y, w: w, h: h, box: near ? [
          near[0] - w / 2, near[1] - h / 2, near[0] + w / 2, near[1] + h / 2] : null }), C,
        { near: near });
    }
  }

  // §5 + §6 travel tapes. Placement is taken from the revision's own numbers rather than from
  // judgement: the yaw tape sits in the upper 10-15% band (12.5%) across the middle 55-60% of the
  // width (57.5%), the pitch tape in the middle 40-45% of the height (42.5%) near the right edge.
  // Those figures are asserted in the test, because a claim this specific is only worth writing if
  // something checks it, and "visually centered" is how a tape ends up wherever the last edit left
  // it. Both tapes show LOGICAL JOINT TRAVEL (§5.3) from the encoders, never a compass heading, and
  // no cardinal letters appear anywhere.
  // §22: a DERATE indication has to include the relevant travel-tape edge. The edge is computed from
  // the same published soft limits the tape is drawn from, so the highlighted end and the drawer's
  // "DERATE YAW MAX" text cannot drift apart - which matters more than it sounds, because an amber
  // highlight pointing at the wrong end of the tape is worse than no highlight at all.
  const dEdge = String(t.safety_action || "").toUpperCase() === "DERATE" ? hudSafetyEdge(t) : null;
  const yawMin = deg(t.q_soft_min_yaw_rad), yawMax = deg(t.q_soft_max_yaw_rad);
  const yawTape = hudTravelTape({
    horizontal: true, x: vw * (1 - 0.575) / 2, y: vh * 0.125, length: vw * 0.575,
    minDeg: yawMin, maxDeg: yawMax,
    markDeg: (dEdge && dEdge.axis === "YAW") ? (dEdge.side === "MIN" ? yawMin : yawMax) : undefined,
    valueDeg: deg(t.q_yaw_rad), valid: t.soft_limits_valid === true
  });
  const pitchLen = vh * 0.425;
  const pitchTape = hudTravelTape({
    horizontal: false, x: vw - Math.max(78.0, vw * 0.055), y: vh / 2 - pitchLen / 2,
    length: pitchLen, minDeg: deg(t.q_soft_min_pitch_rad), maxDeg: deg(t.q_soft_max_pitch_rad),
    markDeg: (dEdge && dEdge.axis === "PITCH")
      ? (dEdge.side === "MIN" ? deg(t.q_soft_min_pitch_rad) : deg(t.q_soft_max_pitch_rad))
      : undefined,
    valueDeg: deg(t.q_pitch_rad), valid: t.soft_limits_valid === true
  });
  // §11: the FOR inset, drawn from the daemon's own block. The coordinate_frame check is not
  // ceremony - if the server ever starts sending a polygon in a different frame, drawing it as joint
  // travel would be quietly wrong rather than visibly wrong, and this station has already been burned
  // once by a number whose frame was assumed.
  const forB = (t.field_of_regard && typeof t.field_of_regard === "object") ? t.field_of_regard : null;
  layers.for = "";
  if (forB && forB.valid === true && forB.coordinate_frame === "joint_deg" &&
      Array.isArray(forB.safe_envelope_points) && t.effective_hfov_deg > 0 &&
      t.effective_vfov_deg > 0 && typeof t.q_yaw_rad === "number" &&
      typeof t.q_pitch_rad === "number") {
    const hasIntent = t.intent_has_joint_target === true &&
                      typeof t.intent_q_yaw_rad === "number" && typeof t.intent_q_pitch_rad === "number";
    const hasRef = typeof t.q_ref_yaw_rad === "number" && typeof t.q_ref_pitch_rad === "number";
    const gi = hudForInset({
      vw: vw, vh: vh, pts: forB.safe_envelope_points,
      hfovDeg: t.effective_hfov_deg, vfovDeg: t.effective_vfov_deg,
      los: [deg(t.q_yaw_rad), deg(t.q_pitch_rad)],
      target: hasIntent ? [deg(t.intent_q_yaw_rad), deg(t.intent_q_pitch_rad)] : null,
      pred: hasRef ? [deg(t.q_ref_yaw_rad), deg(t.q_ref_pitch_rad)] : null
    });
    layers.for = hudForInsetSvg(gi, C);
  }

  layers.tape =
    hudTravelTapeSvg(yawTape, C, { title: "YAW", vw: vw, vh: vh,
                                   value: hudDegLabel(deg(t.q_yaw_rad), true),
                                   note: "JOINT TRAVEL, NOT HEADING" }) +
    hudTravelTapeSvg(pitchTape, C, { title: "PITCH", vw: vw, vh: vh,
                                     value: hudDegLabel(deg(t.q_pitch_rad), true),
                                     note: "JOINT, NOT ELEVATION" }) +
    ((yawTape || pitchTape) ? ""
     : hudUnrangedNote(vw / 2, vh * 0.125, "YAW / PITCH"));

  $("g-candidates").innerHTML = layers.cand;
  $("g-selected").innerHTML = layers.sel;
  $("g-reticle").innerHTML = layers.reticle;
  $("g-prediction").innerHTML = layers.pred;
  $("g-for").innerHTML = layers.for;
  $("g-tapes").innerHTML = layers.tape;

  // target_aim_x/y_norm (the point inside the target the controller is driving onto the axis) is
  // deliberately NOT drawn. v3.2 mentions an aiming marker exactly once - §7's open centre, which
  // IS the optical axis - and inventing a second marker would put an unspecced symbol on the
  // operator's screen. It is also unnecessary: the controller aims the head AT the axis, so what
  // the operator sees is the reticle sitting on the head, which is the acceptance rule as stated.
  // The field stays in telemetry for measurement and for the DIAG drawer.

  // §4.1 mode block, §21's state wording. Three lines, first line strongest.
  const st = hudStateLabel({ mode: t.operating_mode, phase: t.mode_phase,
                             jogging: !!t.manual_lease_active });
  $("mode-block").innerHTML =
    '<div class="m1">' + st.line1 + '</div>' +
    '<div class="m2' + (st.named ? "" : " raw") + '">' + st.line2 + '</div>' +
    '<div class="m3">' + (t.selected_label || (t.selected_uuid_valid ? String(t.selected_uuid) : "--")) +
    '</div>';

  // §8 health chips. Anything the snapshot does not carry is shown as absent, because
  // an HUD that invents health is worse than one that admits a gap.
  const hs = $("health");
  hs.innerHTML = "";
  const connected = !!t.controld_connected;
  hs.appendChild(chip("CONNECTED", connected ? "ok" : "red"));
  hs.appendChild(chip("HOMED", t.at_ready ? "ok" : "amber"));
  const vis = (typeof t.vision_track_sets === "number" && t.vision_track_sets > 0) ? "ok" : "amber";
  hs.appendChild(chip("VISION", vis, vis === "ok" ? "" : "NO SETS"));
  hs.appendChild(chip("IMU", "amber", "ABSENT"));
  // §22. Normal is green and compact; anything heavier gets its own element, sized by tier, and the
  // FAULT case is allowed to interrupt precisely because §22 asks it to.
  const sf = hudSafetyPresentation(t);
  if (sf.tier === "normal") hs.appendChild(chip("SAFETY", "ok", "ALLOW"));
  else if (sf.tier === "caution") hs.appendChild(chip(sf.label, "amber", sf.reason));
  const banner = $("safety");
  if (sf.tier === "normal" || sf.tier === "caution") {
    banner.hidden = true;
    banner.innerHTML = "";
  } else {
    banner.hidden = false;
    banner.className = sf.tone + " " + sf.tier;
    banner.innerHTML = '<div class="s1">' + sf.label + '</div>' +
                       (sf.reason ? '<div class="s2">' + sf.reason + '</div>' : "");
  }

  // §12 bottom strip. FPS here is `camera_fps`: the inter-TrackSet cadence, which is what
  // §12's example strip quotes ("FPS 29"). The browser's preview rate is a different, separately
  // limited number and is not what this cell claims..
  const cell = (k, v, cls) => '<span class="k">' + k + '</span><span class="' + (cls || "v") + '">' + v + '</span>';
  $("strip").innerHTML =
    cell("MODE", mode, "v") + '<span class="sep">|</span>' +
    cell("STATE", String(t.track_state || "--").toUpperCase(), "v") + '<span class="sep">|</span>' +
    cell("TARGETS", String(t.track_count == null ? "--" : t.track_count), "v") + '<span class="sep">|</span>' +
    cell("FPS", fmt(t.camera_fps, 0), "v") + '<span class="sep">|</span>' +
    cell("AGE", fmt(t.vision_measurement_age_ms, 0, " ms"), stale ? "warn" : "v") + '<span class="sep">|</span>' +
    cell("SAFETY", safe && safe !== "NONE" ? safe : "ALLOW", "v");
}

function paint(t) {
  lastTelemetry = t; lastTelemetryAt = Date.now();
  transportOk = true;
  resolveAckFromTelemetry(t);
  render(t);
}

function updateStaleness(t) {
  // §25. Applies the verdict, and returns it so the caller can colour the fields it is about to
  // draw. Re-rendering is how "stops visual interpolation" is enforced rather than merely asserted:
  // the overlay is rebuilt from the payload that is on hand, and when that payload is old the whole
  // viewport desaturates and declares itself - nothing keeps drifting on numbers that died.
  const verdict = hudStale({
    transportOk: transportOk,
    telemetryStale: !!(t && t.telemetry_stale === true),
    telemetryAgeMs: healthAgeMs,
    msgAgeMs: lastTelemetryAt ? (Date.now() - lastTelemetryAt) : null,
    trackListAgeMs: (t && typeof t.track_list_age_ms === "number") ? t.track_list_age_ms : null,
    staleAfterMs: STALE_AFTER_MS,
    quietAfterMs: QUIET_AFTER_MS,
    trackAfterMs: TRACK_AFTER_MS
  });
  const vp = document.getElementById("viewport");
  if (vp) vp.classList.toggle("stale", verdict);   // a no-op when the verdict did not change
  return verdict;
}

// --- transport -----------------------------------------------------------
function connect() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  const ws = new WebSocket(proto + "//" + location.host + "/ws");
  ws.onmessage = (ev) => {
    try { paint(JSON.parse(ev.data)); } catch (e) { /* malformed frames are counted server-side */ }
  };
  ws.onclose = () => { transportOk = false; setTimeout(connect, 1000); };
  ws.onerror = () => { transportOk = false; };
}

// The preview does not autostart: /api/video answers 409 until something asks for it. That is how
// this page first shipped a dead black video panel - the symbology was drawing correctly over
// nothing at all. So the HUD asks for the stream itself, and if the camera is held by something
// else (visiond can own the IMX500 and both cannot) it says why on the notices layer, rather than
// leaving the operator to guess whether the target is gone or the picture is.
function notice(msg) {
  const n = $("notices");
  if (n) n.textContent = msg || "";
}

async function ensureVideo() {
  try {
    const r = await fetch("/api/video/start", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
    const j = await r.json();
    if (r.status >= 400 || j.ok === false || j.running === false) {
      notice("VIDEO UNAVAILABLE: " + (j.error || ("HTTP " + r.status)) +
             " - symbology below is live, the picture is not");
      return false;
    }
    notice("");
    return true;
  } catch (e) {
    notice("VIDEO UNAVAILABLE: " + e);
    return false;
  }
}

// webd keeps serving the last snapshot it received after controld dies, which has already
// fooled a test script and would equally fool an operator. §25 makes it a defect; until the
// server stops doing it, the page checks the connection itself and says so.
async function pollHealth() {
  try {
    const r = await fetch("/api/health");
    const h = await r.json();
    if (!h.controld_connected) transportOk = false; else transportOk = true;
    // webd's own measure of how stale controld's data is, live, independent of whether any frames
    // are arriving - which is the only way this page can notice a daemon that hung rather than
    // died. Absent (older webd, or no telemetry yet) stays null and simply does not vote.
    healthAgeMs = (typeof h.telemetry_age_ms === "number") ? h.telemetry_age_ms : null;
    if (lastTelemetry) render(lastTelemetry);
    // Self-heal the preview: if the stream stopped (daemon bounce, or another owner took the
    // camera and let go), ask again instead of letting a frozen frame keep looking like a live one.
    const v = await (await fetch("/api/video/state")).json();
    if (v.running === false) await ensureVideo();
  } catch (e) { transportOk = false; if (lastTelemetry) render(lastTelemetry); }
}


// --- §13 dock / §14 drawer behaviour -----------------------------------------
const dock = $("dock"), drawer = $("drawer");
let drawerOpen = null;
let lastAck = { text: "", kind: "" };
let pendingConfirm = null;    // label awaiting a second press; see the two-press rule below
let pendingAck = null;        // {command, afterSeq, at}: the published ack this command is waiting on

// Line icons, drawn rather than filled: §13.1 asks for a green line icon and explicitly rules out the
// raised solid-fill card look, which is the fastest way for an overlay to stop reading as a HUD.
function dockIcon(k) {
  const g = 'stroke="' + C.green + '" stroke-width="1.4" fill="none"';
  const inner = {
    TARGETS: '<circle cx="7" cy="7" r="4" ' + g + '/><line x1="7" y1="0.5" x2="7" y2="3.5" ' + g +
             '/><line x1="7" y1="10.5" x2="7" y2="13.5" ' + g + '/><line x1="0.5" y1="7" x2="3.5" y2="7" ' +
             g + '/><line x1="10.5" y1="7" x2="13.5" y2="7" ' + g + '/>',
    MODE: '<path d="M2 5h8L7.5 2.5M12 9H4l2.5 2.5" ' + g + '/>',
    MANUAL: '<line x1="7" y1="13" x2="7" y2="5" ' + g + '/><path d="M4 8l3-3 3 3" ' + g +
            '/><line x1="2.5" y1="13" x2="11.5" y2="13" ' + g + '/>',
    DIAG: '<polyline points="1.5,10 4,10 5.5,4 8,12 10,7 12.5,7" ' + g + '/>',
    MENU: '<line x1="2" y1="4" x2="12" y2="4" ' + g + '/><line x1="2" y1="7" x2="12" y2="7" ' + g +
          '/><line x1="2" y1="10" x2="12" y2="10" ' + g + '/>'
  }[k] || "";
  return '<svg viewBox="0 0 14 14" width="14" height="14" aria-hidden="true">' + inner + "</svg>";
}

function renderDock() {
  dock.innerHTML = hudDockSpecs({ open: drawerOpen }).map((b) =>
    '<button type="button" class="dockbtn' + (b.active ? " on" : "") + '" data-key="' + b.key +
    '" aria-pressed="' + (b.active ? "true" : "false") + '">' + dockIcon(b.key) +
    "<span>" + b.key + "</span></button>").join("");
}

// The daemon's ack is the only thing entitled to say ACCEPTED. It arrives on the next snapshot, so this
// runs from render(); a command whose ack never comes is called out after a moment rather than left
// looking accepted, because a command that quietly produced nothing is the failure the operator cannot
// see from a picture that keeps moving.
function resolveAckFromTelemetry(t) {
  if (!pendingAck || !t) return;
  const seq = (typeof t.cmd_ack_seq === "number") ? t.cmd_ack_seq : 0;
  if (t.cmd_ack_command === pendingAck.command && seq > pendingAck.afterSeq) {
    const accepted = t.cmd_ack_accepted === 1 || t.cmd_ack_accepted === true;
    const why = String(t.cmd_ack_reason || "");
    lastAck = { text: pendingAck.command + (accepted ? "  ACCEPTED"
                  : "  REFUSED: " + (why || "no reason given")), kind: accepted ? "ok" : "bad" };
    pendingAck = null;
  } else if (Date.now() - pendingAck.at > 4000) {
    lastAck = { text: pendingAck.command + "  NO ACK FROM CONTROLD", kind: "bad" };
    pendingAck = null;
  }
}

function renderDrawer() {
  if (!drawerOpen) { drawer.hidden = true; drawer.innerHTML = ""; return; }
  let rows;
  if (drawerOpen === "DIAG") {
    rows = hudDiagRows(lastTelemetry || {}).map((kv) =>
      '<div class="drow"><span class="rl">' + kv[0] + '</span><span class="rn">' + kv[1] +
      "</span></div>").join("");
  } else {
    rows = hudDrawerActions(drawerOpen, lastTelemetry || {}).map((a) => {
      const inert = a.command === null || a.kind === "current" || a.kind === "gated";
      const cls = "drow " + (a.kind === "stop" ? "stop" : a.kind === "danger" ? "danger" :
                             a.kind === "gated" ? "gated" : a.kind === "current" ? "on" : "");
      const waiting = pendingConfirm === a.label;
      return '<button type="button" class="' + cls + (waiting ? " confirm" : "") + '" data-cmd="' +
             (a.command || "") + '" data-arg="' + (a.arg || "") + '" data-kind="' + a.kind + '"' +
             (inert ? " disabled" : "") + '><span class="rl">' +
             (waiting ? "CONFIRM " + a.label : a.label) + '</span><span class="rn">' +
             (waiting ? "PRESS AGAIN" : (a.note || "")) + "</span></button>";
    }).join("");
  }
  drawer.innerHTML = '<div class="dtitle">' + drawerOpen + "</div>" + rows +
    '<div class="dack ' + lastAck.kind + '" role="status">' + (lastAck.text || "&nbsp;") + "</div>";
  drawer.hidden = false;
}

// §13.2: only one drawer at a time, and pressing the same button again closes it. There is one
// `drawerOpen` variable rather than five toggles, which makes the rule structural instead of something
// each handler has to remember to honour.
function setDrawer(key) {
  drawerOpen = (drawerOpen === key) ? null : key;
  lastAck = { text: "", kind: "" };
  pendingConfirm = null;
  renderDock();
  renderDrawer();
}

async function sendCommand(cmd, arg) {
  let j = null;
  try {
    const r = await fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command: cmd, arg: arg || "" })
    });
    j = await r.json();
  } catch (e) {
    lastAck = { text: cmd + "  NOT SENT: transport", kind: "bad" };
    renderDrawer();
    return;
  }
  // What came back over the socket is NOT the verdict. It says the command reached the daemon's
  // handler; the daemon's own decision is recorded separately and published as cmd_ack_* - and the two
  // disagree today, which was found by posting a selection the station could not honour: controld's log
  // says "select_target 9999: REFUSED (no vision data has reached controld yet)" while /api/command
  // answered ok:true. Rendering that response as ACCEPTED would have told the operator the turret had
  // picked a target that does not exist. So the response is reported as SENT, and the verdict is read
  // from the ack the daemon publishes, matched on command name and sequence.
  const seq = (lastTelemetry && typeof lastTelemetry.cmd_ack_seq === "number")
    ? lastTelemetry.cmd_ack_seq : 0;
  pendingAck = { command: cmd, afterSeq: seq, at: Date.now() };
  // `verdict` says which question the response answered, so the two cases can finally be told apart.
  // "rejected" is a decision and is shown as one, immediately - the gate refused it and nothing will
  // execute. "submitted" is a receipt for queueing and must not be dressed up as success. An older
  // daemon that sends no verdict leaves the wording honest rather than guessed.
  lastAck = (j && j.verdict === "rejected")
    ? { text: cmd + "  REFUSED: " + ((j && j.error) || "no reason given"), kind: "bad" }
    : { text: cmd + ((j && j.verdict === "submitted") ? "  SUBMITTED" : "  SENT"), kind: "" };
  pendingConfirm = null;
  renderDrawer();
}

dock.addEventListener("click", (e) => {
  const b = e.target && e.target.closest ? e.target.closest("button[data-key]") : null;
  if (b) setDrawer(b.getAttribute("data-key"));
});

drawer.addEventListener("click", (e) => {
  const b = e.target && e.target.closest ? e.target.closest("button[data-cmd]") : null;
  if (!b || b.disabled) return;
  const cmd = b.getAttribute("data-cmd");
  if (!cmd) return;
  if (b.getAttribute("data-kind") === "danger") {
    // Two presses for the actions that move the turret somewhere it was not just asked to go. The label
    // changes and the row says PRESS AGAIN, so the waiting state is on screen, not in someone's memory.
    const label = String((b.querySelector && b.querySelector(".rl")) ?
                         b.querySelector(".rl").textContent : b.textContent);
    if (pendingConfirm !== label) { pendingConfirm = label; renderDrawer(); return; }
  }
  sendCommand(cmd, b.getAttribute("data-arg") || "");
});

renderDock();

window.addEventListener("resize", () => { if (lastTelemetry) render(lastTelemetry); });
document.addEventListener("DOMContentLoaded", () => {
  $("video").addEventListener("loadedmetadata", () => { if (lastTelemetry) render(lastTelemetry); });
  // An <img> error is how a stopped stream shows up; re-ask rather than reload forever.
  $("video").addEventListener("error", () => { ensureVideo(); });
  ensureVideo();
  connect();
  pollHealth();
  setInterval(pollHealth, 2000);
  // The watchdog §25 actually needs. /api/health can be polled twice a second, and the transport
  // announces a close, but between those two a link that merely goes SILENT - no close event, webd
  // still healthy, controld still publishing to everyone else - leaves this page showing its last
  // frame indefinitely. Checking the clock costs a DOM class toggle four times a second and is the
  // only mechanism that does not assume someone will eventually tell us. It re-evaluates the verdict
  // rather than repainting the page: the overlay is rebuilt from payloads when payloads exist, and
  // rebuilding the DOM four times a second to notice that nothing arrived is a lot of work to
  // discover an absence.
  setInterval(() => { if (lastTelemetry) updateStaleness(lastTelemetry); }, 250);
});
"""

HUD_CSS = r"""
:root {
  --hud-green: #95f58b;
  --hud-green-dim: rgba(149,245,139,.56);
  --hud-green-faint: rgba(149,245,139,.22);
  --hud-amber: #f2b329;
  --hud-red: #ff5d5d;
  --hud-white: #edf2eb;
  --hud-black: rgba(3,6,5,.80);
  --hud-line: rgba(230,245,230,.24);
  /* §16's stack, declared as a token because three later rules read var(--hud-mono) inside a `font:`
     SHORTHAND. An undefined custom property makes the whole shorthand invalid at computed-value time,
     which drops the size and weight too - so an undeclared token is not "falls back to the inherited
     font", it is "the dock renders in the browser's serif default at 13px". Static review does not show
     that; a test that every var() is declared does. */
  --hud-mono: "IBM Plex Mono", "Roboto Mono", "SFMono-Regular", Consolas, monospace;
}
/* §16: narrow monospaced sensor display, not a proportional UI font. */
html, body { margin: 0; height: 100%; background: #05070a; overflow: hidden;
  /* The stack lives in one place. An earlier revision of this file carried it here as a literal AND in
     var(--hud-mono), which a test written before that change had already warned about: "the font stack
     is set once; three copies is how they drift". */
  font-family: var(--hud-mono);
  color: var(--hud-white); }
#viewport { position: fixed; inset: 0; }
/* §18 layering, exactly as specified. */
#video { position: absolute; inset: 0; width: 100%; height: 100%; object-fit: contain;
  background: #000; z-index: 0; }
#overlay { position: absolute; inset: 0; z-index: 10; pointer-events: none; }
#mode-block { position: absolute; left: 1%; top: 1.2%; z-index: 20; text-shadow: 0 0 6px rgba(0,0,0,.9); }
#mode-block .m1 { color: var(--hud-green); font-size: 20px; letter-spacing: .12em; }
#mode-block .m2 { color: var(--hud-white); font-size: 11px; letter-spacing: .12em; opacity: .86; }
#mode-block .m3 { color: var(--hud-white); font-size: 11px; letter-spacing: .12em; opacity: .62; }
#health { position: absolute; right: 1%; top: 1.2%; z-index: 20; display: flex; gap: 6px; }
.chip { display: flex; align-items: center; gap: 5px; padding: 3px 7px; font-size: 10px;
  letter-spacing: .1em; background: var(--hud-black); border: 1px solid var(--hud-line);
  border-radius: 3px; }
.chip .dot { width: 6px; height: 6px; border-radius: 50%; }
.chip .val { opacity: .7; }
#strip { position: absolute; left: 1%; bottom: 1.4%; z-index: 20; display: flex; gap: 7px;
  align-items: baseline; padding: 4px 9px; font-size: 11px; letter-spacing: .08em;
  background: var(--hud-black); border: 1px solid var(--hud-line); border-radius: 3px; }
#strip .k { color: rgba(237,242,235,.55); margin-right: 3px; }
#strip .v { color: var(--hud-green); }
#strip .warn { color: var(--hud-amber); }
#strip .sep { color: var(--hud-line); }
text.tlbl { font-size: 11px; letter-spacing: .04em; font-family: inherit; }   /* scale labels */
text.tval { font-size: 12px; letter-spacing: .06em; font-family: inherit; }   /* value boxes */
/* 16's hierarchy is a size RELATIONSHIP, not a list of names. Three of its rows used to share one 11px
   rule, which left the hierarchy true only in the document: scale labels, candidate labels and the FOR
   legend all measured the same. The sizes below are the section read out in order - 12px is the strongest
   numeric (yaw/pitch values), 11px scale labels (medium-small), 10px candidate labels (small, the same
   step the bottom strip uses, as the section asks), 9px the FOR legend at the smallest readable size. */
text.lbl { font-size: 10px; letter-spacing: .08em; font-family: inherit; }    /* candidate labels */
text.flbl { font-size: 9px; letter-spacing: .06em; font-family: inherit; }    /* FOR legend */
/* §25: stale telemetry stops visual interpolation and says so. The filter is presentation
   only - the overlay keeps drawing the last known geometry, dimmed, with the AGE cell amber. */
#viewport.stale #video { filter: grayscale(.55) brightness(.72); }
#viewport.stale #overlay { opacity: .55; }
#viewport.stale::after { content: "TELEMETRY STALE / DISCONNECTED"; position: absolute;
  left: 50%; bottom: 8%; transform: translateX(-50%); z-index: 20; font-size: 11px;
  letter-spacing: .18em; color: var(--hud-amber); background: var(--hud-black);
  border: 1px solid var(--hud-amber); padding: 3px 8px; }
#notices { position: absolute; left: 1%; top: 14%; z-index: 20; font-size: 11px;
  letter-spacing: .08em; color: var(--hud-amber); text-shadow: 0 0 6px rgba(0,0,0,.9); }

/* --- §13.1 dock, §14 drawer ------------------------------------------------- */
#dock { position:absolute; right:1.2%; bottom:8.5%; display:flex; gap:6px; z-index:30; }
.dockbtn { display:flex; flex-direction:column; align-items:center; gap:3px; width:46px;
           padding:5px 2px 4px; background:rgba(3,6,5,.62); border:1px solid rgba(230,245,230,.22);
           border-radius:2px; color:#edf2eb; font:500 8.5px/1 var(--hud-mono); letter-spacing:.06em;
           cursor:pointer; }
.dockbtn:hover { border-color:rgba(149,245,139,.55); }
.dockbtn.on { border-color:#95f58b; background:rgba(3,6,5,.78); }
.dockbtn span { color:#edf2eb; }

/* §14: translucent black body, thin green/neutral border, monospaced, compact, boxes inside boxes kept
   rare. Absolutely positioned over the video, so opening it never rescales the picture - §13.2's
   explicit requirement, and the reason this is not a flex sibling of the viewport. */
#drawer { position:absolute; right:1.2%; bottom:calc(8.5% + 58px); width:min(330px,32vw); max-height:52vh;
          overflow:auto; background:rgba(2,5,4,.86); border:1px solid rgba(149,245,139,.38);
          border-radius:2px; padding:7px 8px 6px; z-index:40; font:400 10px/1.45 var(--hud-mono);
          color:#edf2eb; }
#drawer[hidden] { display:none; }
#drawer .dtitle { color:#95f58b; font-size:9px; letter-spacing:.14em; margin:0 0 5px; }
#drawer .drow { display:flex; justify-content:space-between; gap:10px; width:100%; text-align:left;
                background:none; border:0; border-bottom:1px solid rgba(230,245,230,.09); color:#edf2eb;
                font:inherit; padding:3px 1px; cursor:pointer; }
#drawer .drow:hover:not(:disabled) { background:rgba(149,245,139,.10); }
#drawer .drow .rn { color:rgba(149,245,139,.56); white-space:nowrap; }
#drawer .drow.on .rl { color:#95f58b; }
#drawer .drow.gated { opacity:.42; cursor:not-allowed; }
#drawer .drow.gated .rn { color:#f2b329; }
#drawer .drow.stop .rl { color:#ff5d5d; }
#drawer .drow.stop:hover { background:rgba(255,93,93,.14); }
#drawer .drow.confirm { background:rgba(242,179,41,.16); }
#drawer .drow.confirm .rl { color:#f2b329; }
#drawer .dack { margin-top:6px; font-size:9px; letter-spacing:.05em; color:rgba(149,245,139,.56); }
#drawer .dack.ok { color:#95f58b; }
#drawer .dack.bad { color:var(--hud-amber); }

/* --- §22 safety presentation ------------------------------------------------ */
#safety { position:absolute; left:50%; top:16%; transform:translateX(-50%); text-align:center;
          font-family:var(--hud-mono); z-index:50; }
#safety[hidden] { display:none; }
#safety .s1 { font-size:22px; letter-spacing:.22em; }
#safety .s2 { font-size:11px; letter-spacing:.08em; margin-top:3px; opacity:.9; }
#safety.amber .s1 { color:#f2b329; }
#safety.amber .s2 { color:rgba(242,179,41,.8); }
#safety.prominent .s1 { font-size:30px; }
/* §22: FAULT is red and "prominent enough to interrupt normal operation". §14 reserves red for stop and
   fault, which is exactly the case this rule is for. */
#safety.red .s1 { color:#ff5d5d; text-shadow:0 0 12px rgba(255,93,93,.45); }
#safety.red .s2 { color:rgba(255,150,150,.92); }
#safety.interrupt .s1 { font-size:38px; letter-spacing:.3em; }
#mode-block .m2.raw { color:rgba(237,242,235,.55); }   /* a phase §21 does not name, dimmed as the
                                                          daemon's own word rather than HUD wording */
"""

HUD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>OpenAutoTurret — HUD</title>
<style>""" + HUD_CSS + """</style>
</head>
<body>
<div id="viewport">
  <!-- z=0 camera image. Whole frame always visible: the frame edge is a number the
       operator has to be able to read, so the video is contained, never cropped. -->
  <img id="video" src="/api/video" alt="camera">

  <!-- z=10 candidates, z=11 selected, z=20 reticle: separate layers, because §18 orders
       them and because "the reticle never represents the target" is easier to keep true
       when they are not the same drawable. -->
  <svg id="overlay" xmlns="http://www.w3.org/2000/svg">
    <defs>
      <filter id="softglow" x="-40%" y="-40%" width="180%" height="180%">
        <feGaussianBlur stdDeviation="2.2" result="b"/>
        <feMerge><feMergeNode in="b"/><feMergeNode in="SourceGraphic"/></feMerge>
      </filter>
    </defs>
    <!-- s18 wants candidates 10, selected 11, reticle 20. Inside one SVG that ordering is
         achieved by DOCUMENT ORDER, not by z-index: SVG has no z-index for child elements, so
         putting z-index on a <g> would be decoration that the renderer ignores while the markup
         appears compliant. The order below is the layering: candidates, then selected, then the
         reticle on top of both, which is also the only way to keep "the reticle never represents
         the target" literally true - it is painted last, over anything in a box. -->
    <g id="g-candidates"></g>
    <g id="g-selected"></g>
    <g id="g-prediction"></g>
    <g id="g-reticle"></g>
    <g id="g-for"></g>
    <g id="g-tapes"></g>
  </svg>

  <div id="mode-block"></div>
  <div id="health"></div>
  <div id="notices"></div>
  <div id="strip"></div>

  <!-- §13 dock and §14 drawer. Outside the SVG deliberately: these are the only parts of the overlay the
       operator presses, and real elements keep focus, hover and button semantics away from a hit-test on
       painted geometry. They come last in document order, which on this page IS the z-order (§18: dock
       30, drawer 40); the z-index in the CSS states it rather than relying on it. -->
  <div id="dock" role="toolbar" aria-label="context controls"></div>
  <!-- §22 safety indication. Outside the health chips, because BRAKING and FAULT are asked to be more
       prominent than a chip and a fault to interrupt normal operation. -->
  <div id="safety" hidden role="status" aria-live="assertive"></div>
  <div id="drawer" hidden role="dialog" aria-modal="false"></div>
</div>
<script>""" + HUD_JS + """</script>
</body>
</html>
"""
