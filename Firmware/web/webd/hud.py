"""The v3.2 Apache-HUD operator page.

`docs/archive/implemented/design/open_auto_turret_v3_2_apache_hud_ui_revision.md` governs presentation and overrides the v3
dashboard, whose engineering cards are the exact "header plus cards" layout §3 forbids. The
engineering numbers moved into the stats overlay, off by default and turned on from
MENU > SETTINGS; the old `/dashboard` page was removed on 2026-10-03 (owner), and `/api/*` is
untouched, so nothing downstream changes.

Two decisions worth their weight, both recorded in code rather than in chat:

**The video is `contain`, not `cover`.** Cover fills the viewport by cropping, and the one thing
this station's operator has to be able to judge is whether a target is about to LEAVE THE FRAME -
the acceptance margin for the lead requirement. Cropping would hide the true frame edge and show a
boundary that is not the camera's. So the whole frame is always visible and the letterbox bars are
the cost, taken deliberately.

The optical-axis marker uses the camera principal point. With virtual bore alignment
enabled, the main reticle uses the controller's projected bore sight and is amber,
labelled with its assumed depth; the camera centre remains a small separate marker.
The requested measurement point is a white diamond. No bore range is measured here.
"""
from __future__ import annotations

from .direction_contract import DIRECTION_JS

# ---------------------------------------------------------------------------
# Pure geometry, kept in one string so a test can execute THESE BYTES under node
# and compare them against an independent Python computation. The page and the
# test cannot drift apart, because the test does not reimplement this.
# ---------------------------------------------------------------------------
HUD_GEOMETRY_JS = DIRECTION_JS + r"""
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
  // Every published coordinate is in the wide camera's frame (perception/detection/view.py). With
  // the detail camera on the main display, the picture is a centred window `k` times smaller, so
  // the same point sits k times further from the centre of what the operator sees.
  const k = (lay.k > 1) ? lay.k : 1;
  const uu = 0.5 + (u - 0.5) * k, vv = 0.5 + (v - 0.5) * k;
  return { ok: true, x: lay.ox + uu * lay.w, y: lay.oy + vv * lay.h };
}

// Which camera is on the main display, from the station rather than from this page: only that one
// is inferred (owner, 2026-10-02), so every page and every reload must agree. An absent or stale
// inference report means "assume wide" -- every boot starts there.
function hudMainView(t) {
  const inf = (t && typeof t.inference === "object" && t.inference) ? t.inference : null;
  const mc = (inf && inf.present !== false && inf.fresh !== false && inf.main_camera &&
              typeof inf.main_camera === "object") ? inf.main_camera : null;
  const main = (mc && mc.role === "detail") ? "detail" : "wide";
  const view = (mc && mc.views && mc.views.detail) || null;
  const k = (main === "detail" && view && Number(view.scale) > 1) ? Number(view.scale) : 1;
  const available = (mc && Array.isArray(mc.available)) ? mc.available : ["wide"];
  return { main: main, pip: main === "wide" ? "detail" : "wide", k: k,
           canSwap: available.indexOf("detail") >= 0,
           generation: (mc && typeof mc.generation === "number") ? mc.generation : undefined,
           boot: (mc && mc.boot) || "" };
}

// Whether a published main-display report is news to this page. A swap's reply arrives before the
// next health beat, so a report older than it must not flip the panes back for a second; but
// generations count within one visiond, so a restarted one (new boot) is always accepted -- a page
// that saw generation 4 used to ignore the new process's generation 0.
function hudAcceptMainView(seen, view) {
  const sameBoot = !view.boot || view.boot === seen.boot;
  const generation = (typeof view.generation === "number") ? view.generation : null;
  if (sameBoot && generation !== null && generation < seen.generation) return null;
  return { boot: view.boot || seen.boot,
           generation: generation !== null ? generation : (sameBoot ? seen.generation : -1) };
}

// The field of view of the picture on the main display: the wide camera's, or the detail window's
// (a centred window k times smaller in the image plane, so tan(half-angle) shrinks by k).
function hudViewFov(fovDeg, k) {
  if (!(fovDeg > 0) || !(k > 1)) return fovDeg;
  return 2 * Math.atan(Math.tan(fovDeg * Math.PI / 360) / k) * 180 / Math.PI;
}

// One rule for both panes, so the PIP can never again recover differently from the main picture.
// `pane` remembers the stream it was pointed at (`epoch`) and when it last tried; `state` is
// /api/video/state for the pane's role. A stopped source is started; a source that restarted under
// the pane (a redeploy restarts webd, and its new stream is not the one the old <img> was reading,
// whether or not the browser ever fired an error) is re-pointed. Attempts are spaced, so a refusal
// cannot become a request storm.
function hudPaneStep(pane, state, nowMs) {
  if (!state || typeof state !== "object") return "none";
  const due = !(pane.lastAttemptMs > 0) || nowMs - pane.lastAttemptMs >= 1500;
  if (state.running === false) return due ? "start" : "wait";
  if (state.running === true && state.epoch && state.epoch !== pane.epoch) return "point";
  return "none";
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

/**
 * The reticle's cant (roll) reference line, as geometry rather than as four hand-placed strokes.
 *
 * There is ONE line: the parametric point centre + k*(ux, uy). Two visible halves are the samples
 * outside the central box, which is why the middle is missing and why the two halves cannot drift out
 * of line with each other -- they are the same expression evaluated at two intervals. Rotating by
 * `deg` in screen space: 0 = horizontal, positive = clockwise.
 *
 * Returns [darkUnderStroke, greenLine] so the caller decides the order (dark first, always).
 */
function hudReticleCantSvg(cx, cy, reach, inner, deg, colors) {
  const rad = (Number.isFinite(deg) ? deg : 0) * Math.PI / 180;
  const ux = Math.cos(rad), uy = Math.sin(rad);
  const seg = (from, to) =>
    '<line x1="' + (cx + ux * from) + '" y1="' + (cy + uy * from) + '" x2="' +
    (cx + ux * to) + '" y2="' + (cy + uy * to) + '"/>';
  const halves = seg(-reach, -inner) + seg(inner, reach);
  return [halves.replace(/\/>/g, ' stroke="' + colors.stroke + '" stroke-width="3.6"/>'),
          halves.replace(/\/>/g, ' stroke="' + colors.line + '" stroke-width="2"/>')];
}

// Controller-owned projection; the browser never reconstructs mounting geometry.
function hudBoreMark(t, stale) {
  const a = t && t.alignment;
  if (stale || !a || a.mode !== "manual_depth" || a.valid !== true ||
      a.range_source !== "manual" || a.range_measured !== false ||
      !Number.isFinite(a.assumed_depth_m) || a.assumed_depth_m <= 0 ||
      !Number.isFinite(a.x_norm) || !Number.isFinite(a.y_norm) ||
      a.x_norm < 0 || a.x_norm > 1 || a.y_norm < 0 || a.y_norm > 1) return null;
  return { u: a.x_norm, v: a.y_norm, label: "ASSUMED " + a.assumed_depth_m.toFixed(1) + " m" };
}

function hudMeasurementPointSvg(t, lay, stale, C) {
  if (stale || !t || t.target_aim_valid !== true ||
      !Number.isFinite(t.target_aim_x_norm) || !Number.isFinite(t.target_aim_y_norm) ||
      t.target_aim_x_norm < 0 || t.target_aim_x_norm > 1 ||
      t.target_aim_y_norm < 0 || t.target_aim_y_norm > 1) return "";
  const p = hudProject(t.target_aim_x_norm, t.target_aim_y_norm, lay);
  if (!p.ok) return "";
  // §8's point: the measured position needs no caption -- the green reticle *is* the caption. What
  // still earns words is the two exceptions: "this point is not a box measurement" (ANCHOR) and
  // "the box is off the edge" (CLIPPED, a degraded condition, therefore amber and not green).
  const parts = ['<g class="measurement-point"><path d="M ' + p.x + ' ' + (p.y-5) + ' L ' +
    (p.x+5) + ' ' + p.y + ' L ' + p.x + ' ' + (p.y+5) + ' L ' + (p.x-5) + ' ' + p.y +
    ' Z" fill="none" stroke="#edf2eb" stroke-width="1.5" stroke-linejoin="round"/>'];
  if (t.target_aim_source !== "box_fraction") {
    parts.push('<text x="' + (p.x+9) + '" y="' + (p.y-9) + '" class="lbl" fill="' + C.text +
               '">ANCHOR</text>');
  }
  if (t.target_aim_box_clipped) {
    parts.push('<text x="' + (p.x+9) + '" y="' + (p.y+12) + '" class="lbl" fill="' + C.amber +
               '" font-weight="500">CLIPPED</text>');
  }
  return parts.join("") + '</g>';
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
  // The station is Homed or Shutdown (owner ruling 2026-10-03), and a boot is Shutdown: the phase is
  // controld's "idle", and the line says what starts it.
  if (o.supervisory === "idle") return { line1: "SHUTDOWN", line2: "MOTORS OFF · MENU › HOME", named: true };
  // At the park pose the station is homed and holding; the line names the next steps.
  if (o.supervisory === "parked") return { line1: "PARKED", line2: "AUTO · MANUAL · SHUTDOWN", named: true };
  // A mode selected on the stop: the park lifts pitch back into its envelope first, then the mode runs.
  if (o.supervisory === "parking" && o.rest === "lifting")
    return { line1: "LEAVING PARK", line2: "PITCH OFF THE REST STOP", named: true };
  if (o.supervisory && o.supervisory !== "hold") return {
    line1: String(o.supervisory).toUpperCase(), line2: "", named: true };

  if (auto && phase === "TRACK") return { line1: "AUTO TRACK", line2: "TRACKING", named: true };
  if (auto && phase === "COAST") return { line1: "AUTO TRACK", line2: "COASTING", named: true };
  // §21.3 says "TARGET LOST / HOLDING or equivalent compact state". Losing the target is the fact the
  // operator has to act on; it goes on the strong line rather than under a mode name.
  if (auto && phase === "LOST_HOLD") return { line1: "TARGET LOST", line2: "HOLDING", named: true };
  if (auto && phase === "WAIT_TARGET") return { line1: "AUTO TRACK", line2: "WAIT TARGET", named: true };
  if (roam && phase === "SWEEP") return { line1: "AUTO ROAM", line2: "SWEEP", named: true };
  // Continuous yaw: the whole circle, one direction (owner, 2026-10-02).
  if (roam && phase === "PATROL") return { line1: "AUTO ROAM", line2: "PATROL", named: true };
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
  // §13's controls, in the order the revision lists them. DIAG left the dock on 2026-10-03 (owner):
  // its rows live in the stats overlay, which MENU > SETTINGS turns on, like a video player's
  // "stats for nerds"; an operator who does not ask for engineering numbers does not see them.
  const keys = ["TARGETS", "MODE", "MANUAL", "MENU"];
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
                  command: tr.selected || tr.selectable === false ? null :
                    (t.perception_native ? "select_uuid" : "select_target"),
                  arg: t.perception_native ? JSON.stringify({track_uuid: tr.uuid,
                    session_uuid: t.perception_session_uuid,
                    track_set_sequence_seen_by_ui: t.perception_track_set_sequence}) : String(di),
                  kind: tr.selected ? "current" : tr.selectable === false ? "gated" : "act",
                  note: (typeof tr.confidence === "number" ? Math.round(tr.confidence * 100) + "%" : "") });
    });
    if (!rows.length) rows.push({ label: "NO TARGETS", command: null, kind: "current", note: "" });
    if (t.selected_uuid_valid || tracks.some((x) => x && x.selected)) {
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
      { label: "STEP YAW +1 " + otaAxisArrow("yaw", 1), command: "manual_step", arg: "yaw+1", kind: gate, note: note },
      { label: "STEP YAW -1 " + otaAxisArrow("yaw", -1), command: "manual_step", arg: "yaw-1", kind: gate, note: note },
      { label: "STEP PITCH +1 " + otaAxisArrow("pitch", 1), command: "manual_step", arg: "pitch+1", kind: gate, note: note },
      { label: "STEP PITCH -1 " + otaAxisArrow("pitch", -1), command: "manual_step", arg: "pitch-1", kind: gate, note: note }
    ];
    rows.push({ label: "STOP MOTION", command: "hold", arg: "", kind: "stop",
                note: inManual ? "" : "ALWAYS AVAILABLE" });
    return rows;
  }

  if (name === "MENU") {
    // Owner ruling 2026-10-03: three supervisory actions, each of which works from where it is offered.
    //   HOME     recovers whatever latched (fault, drive watchdog) and runs homing; AUTO ROAM after.
    //   PARK     yaw to 0, pitch onto its rest end stop, hold there; any MODE leaves it.
    //   SHUTDOWN PARK, then both motors off; only HOME starts again.
    // They move the turret somewhere the operator did not just point it, so they ask twice. §14 reserves
    // red for stop and fault, so the confirm state - not colour alone - is what signals danger here.
    const phase = String(t.phase || "");
    const rest = String(t.rest_park || "");
    const parking = rest === "moving" || rest === "touching" || rest === "lifting";
    const busy = phase === "homing" || phase === "recovering";
    const home = busy
      ? { kind: "gated", note: phase === "recovering" ? "RECOVERING THE DRIVES…" : "HOMING…" }
      : rest === "releasing" ? { kind: "gated", note: "SWITCHING THE MOTORS OFF…" }
      : { kind: "danger", note: phase === "fault" ? "Recover the drives, then home both axes"
          : phase === "idle" ? "Start: home, then AUTO ROAM" : "Re-home both axes, then AUTO ROAM" };
    const homed = phase === "hold" || phase === "parked" || phase === "parking";
    const park = parking ? { kind: "gated", note: rest === "touching" ? "TOUCHING THE REST STOP…"
                                                : rest === "lifting" ? "LEAVING THE PARK…" : "PARKING…" }
      : rest === "parked" && t.rest_park_on_stop === true ? { kind: "current", note: "PARKED · ANY MODE LEAVES" }
      : rest === "parked" ? { kind: "danger", note: "Short of the rest stop: touch it again" }
      : phase === "hold" ? { kind: "danger", note: "Yaw to 0, pitch onto its rest stop, hold" }
      : { kind: "gated", note: phase === "idle" ? "MOTORS OFF · HOME FIRST" : "HOME FIRST" };
    const off = rest === "releasing" ? { kind: "gated", note: "SWITCHING THE MOTORS OFF…" }
      : phase === "idle" ? { kind: "gated", note: "MOTORS ARE OFF · HOME TO START" }
      : homed && !busy ? { kind: "danger", note: "Park, then switch both motors off" }
      : { kind: "gated", note: "HOME FIRST" };
    return [
      { label: "HOME", command: home.kind === "danger" ? "start_homing" : null, arg: "", kind: home.kind, note: home.note },
      { label: "PARK", command: park.kind === "danger" ? "request_park" : null, arg: "", kind: park.kind, note: park.note },
      { label: "SHUTDOWN", command: off.kind === "danger" ? "request_shutdown" : null, arg: "", kind: off.kind, note: off.note }
    ];
  }

  return [];
}

// --- MENU > SETTINGS (owner, 2026-10-03) ------------------------------------------------------
// Speeds are live: controld's set_speed, bounded by each mode's maximum, for this session only (a
// restart returns to turret_mixed.yaml, so a trial speed never becomes the deployment's speed by
// accident). The DPAD's COARSE/FINE paces are the two patrol paces, so they follow too.
const HUD_SPEED_SETTINGS = [
  { key: "patrol_wide", label: "PATROL · WIDE", value: "speed_patrol_wide_deg_s",
    base: "speed_patrol_wide_default_deg_s", max: "speed_roam_max_deg_s", step: 1 },
  { key: "patrol_detail", label: "PATROL · DETAIL", value: "speed_patrol_detail_deg_s",
    base: "speed_patrol_detail_default_deg_s", max: "speed_roam_max_deg_s", step: 0.5 },
  { key: "track", label: "TRACKING", value: "speed_track_deg_s",
    base: "speed_track_default_deg_s", max: "speed_track_max_deg_s", step: 1 }
];
const HUD_SPEED_MIN_DEG_S = 0.5;   // controld's own floor

function hudSpeedRows(t) {
  t = t || {};
  const round = (x) => Math.round(x * 10) / 10;
  return HUD_SPEED_SETTINGS.map((s) => {
    const v = t[s.value], base = t[s.base], max = t[s.max];
    const known = typeof v === "number" && v > 0;
    const hasMax = typeof max === "number" && max > 0;
    const down = known ? round(Math.max(HUD_SPEED_MIN_DEG_S, v - s.step)) : null;
    const up = known && hasMax ? round(Math.min(max, v + s.step)) : null;
    return {
      key: s.key, label: s.label,
      value: known ? v : null,
      base: typeof base === "number" && base > 0 ? base : null,
      max: hasMax ? max : null,
      // Each arrow is the exact command it sends; null when it would not change anything.
      down: down !== null && down < v - 1e-9 ? s.key + "=" + down : null,
      up: up !== null && up > v + 1e-9 ? s.key + "=" + up : null,
      reset: known && typeof base === "number" && Math.abs(v - base) > 1e-6 ? s.key + "=default" : null
    };
  });
}

// One policy exists today (perception_v1.json: AUTO_SELECT_SINGLE). The row is a placeholder for the
// choice the owner expects later, so it is shown, named and inert rather than invented.
const HUD_TARGET_POLICY = { label: "TARGET POLICY", value: "ONE PERSON, ALONE 0.5 S",
                            note: "more policies later" };

// --- the stats overlay ("stats for nerds") ----------------------------------------------------
// Off by default, per browser. It replaced both the DIAG drawer and the /dashboard page (owner,
// 2026-10-03), after a field-by-field audit: fields that are constant on this station (installation
// pose, payload verification, the GM6020's torque), legacy v1 tracking state, and anything the HUD
// already shows were left out; what remains is here and nowhere else.
function hudStatsSections(t) {
  t = t || {};
  const deg = (r, dp) => (typeof r === "number" && Number.isFinite(r)
    ? (r * 57.29577951308232).toFixed(dp === undefined ? 1 : dp) : "--");
  const n = (v, dp, unit) => (typeof v === "number" && Number.isFinite(v)
    ? v.toFixed(dp || 0) + (unit || "") : "--");
  const age = (ms) => (typeof ms === "number" && ms >= 0 ? ms + " ms" : "--");
  const control = hudDiagRows(t).filter((kv) => kv[0] !== "MODE / PHASE" && kv[0] !== "IMU");
  control.push(["FEEDBACK AGE", age(t.feedback_age_ms)]);
  control.push(["CONTROL CYCLE", n(t.control_cycle_us, 0, " us")]);
  control.push(["SUPERVISORY", String(t.supervisory_state || "--")]);

  const nf = t.inference || {};
  const streams = Array.isArray(t.video_streams) ? t.video_streams : [];
  const vision = [
    ["PUBLISHER", t.vision_connected ? "CONNECTED" : "NOT CONNECTED"],
    ["MEASUREMENTS", n(t.vision_frames) + "  (" + n(t.vision_dropped) + " dropped)"],
    ["NETWORK", nf.present ? String(nf.adapter || "?").toUpperCase() + "  " + n(nf.inference_fps, 1, " Hz") +
                             "  " + n(nf.model_inference_ms, 1, " ms") : "NO REPORT"]
  ].concat(streams.map((s) => ["STREAM " + String(s.role || "?").toUpperCase(),
    (s.width && s.height ? s.width + "x" + s.height : "--") + "  " + n(s.delivered_fps, 1, " fps") +
    (s.running ? "" : "  (stopped)")]));

  const selection = [
    ["VISIBILITY", String(t.selection_visibility || "--")],
    ["LAST SEEN", age(t.selection_last_seen_age_ms) + "  predicted " + age(t.prediction_age_ms)],
    ["AMBIGUITY", (t.selection_ambiguous ? "AMBIGUOUS" : "CLEAR") + "  reacq " +
                  n(t.reacquisition_score, 2) + "  margin " + n(t.ambiguity_margin, 2)],
    // The installation pose is identity on this station, so this is the line of sight in the base frame.
    ["TARGET LOS AZ / EL", deg(t.target_az_world_rad) + " / " + deg(t.target_el_world_rad) + " DEG (base)"],
    ["INTENT", String(t.intent_source || "--") + " / " + String(t.intent_type || "--") + "  " +
               String(t.confidence_band || "") + "  " + String(t.intent_reason || "")]
  ];

  const axes = [
    ["ASKED YAW / PITCH", t.intent_has_joint_target
      ? deg(t.intent_q_yaw_rad) + " / " + deg(hudPitch(t, t.intent_q_pitch_rad)) + " DEG" : "none"],
    ["RATE YAW / PITCH", deg(t.v_yaw_rad_s) + " / " +
      deg(typeof t.v_pitch_rad_s === "number" ? t.v_pitch_rad_s * HUD_PITCH_UP : null) + " DEG/S"],
    ["PITCH TO LIMIT", deg(t.soft_limit_distance_pitch_rad) + " DEG"],
    // Casual inspection (owner, 2026-10-03): the drives' own sensors, against the supervisor's trip.
    // The GM6020 sends a raw byte, about deg C, which the project does not call calibrated.
    ["MOTOR TEMP YAW / PITCH", (typeof t.temp_yaw_c === "number" ? n(t.temp_yaw_c, 0, "°C")
        : (typeof t.temp_raw_yaw === "number" && t.temp_raw_yaw >= 0
           ? "≈" + t.temp_raw_yaw + "°C (raw)" : "--")) + " / " + n(t.temp_pitch_c, 0, "°C") +
      (typeof t.motor_overtemp_c === "number" && t.motor_overtemp_c > 0
        ? "  (trip " + t.motor_overtemp_c.toFixed(0) + "°C)" : "")],
    ["YAW CURRENT", n(t.current_a_yaw, 2, " A")],
    ["PITCH TORQUE", n(t.effort_pitch, 2, " Nm")],
    ["PAYLOAD PROFILE", String(t.payload_profile_name || "none")]
  ];

  const roam = [
    ["PATTERN", String(t.roam_pattern || "--") + (t.roam_sweep_direction
      ? (t.roam_sweep_direction > 0 ? "  +" : "  -") : "")],
    ["PROGRESS", n(typeof t.roam_progress === "number" ? t.roam_progress * 100 : null, 0, "% of this turn")],
    ["WAYPOINT YAW", typeof t.roam_target_yaw_rad === "number"
      ? hudWrapDeg(t.roam_target_yaw_rad * 57.29577951308232).toFixed(1) + " DEG" : "--"],
    ["JOG LEASE", t.manual_lease_active ? n(t.manual_lease_remaining_ms, 0, " ms") + "  " +
                                         String(t.manual_profile || "") : "idle"]
  ];

  const buses = Array.isArray(t.can_buses) && t.can_buses.length ? t.can_buses
    : (t.can_available ? [{ device: t.can_device, up: t.can_up, state: t.can_state,
        rx_frames: t.can_rx_frames, rx_error_frames: t.can_rx_error_frames, tx_frames: t.can_tx_frames,
        tx_failed: t.can_tx_failed, last_rx_age_ms: t.can_last_rx_age_ms }] : []);
  const busState = (s) => (["ERROR-ACTIVE", "ERROR-WARNING", "ERROR-PASSIVE", "BUS-OFF", "STOPPED",
                            "SLEEPING"][s] || "UNKNOWN");
  const can = buses.length ? buses.map((b) => [String(b.device || "?").toUpperCase(),
    (b.up ? busState(b.state) : "DOWN") + "  rx " + n(b.rx_frames) + " (" + n(b.rx_error_frames) +
    " err)  tx " + n(b.tx_frames) + " (" + n(b.tx_failed) + " fail)  " + age(b.last_rx_age_ms)])
    : [["CAN", "no CAN link reported"]];

  const ack = t.cmd_ack_command ? [["LAST COMMAND", String(t.cmd_ack_command) + "  " +
    (t.cmd_ack_accepted === 1 ? "ACCEPTED" : t.cmd_ack_accepted === 0 ? "REFUSED" : "--") +
    (t.cmd_ack_reason ? "  " + String(t.cmd_ack_reason) : "")]] : [];

  const events = (Array.isArray(t.events) ? t.events.slice() : []).reverse().map((e) => [
    (typeof t.ts_ns === "number" && typeof e.t_ns === "number"
      ? "-" + Math.max(0, (t.ts_ns - e.t_ns) / 1e9).toFixed(0) + " S" : "--"),
    String(e.event || "?") + (e.detail ? "  " + String(e.detail) : "")]);

  return [
    { title: "CONTROL", rows: control },
    { title: "VISION", rows: vision },
    { title: "SELECTION", rows: selection },
    { title: "AXES", rows: axes },
    { title: "ROAM / MANUAL", rows: roam },
    { title: "CAN", rows: can },
    { title: "EVENTS", rows: ack.concat(events.length ? events : [["--", "no events yet"]]) }
  ];
}


// Display origin only. Control, calibration and API joint angles stay raw.
function hudPitchCenter(t) {
  if (!t || t.soft_limits_valid !== true || !Number.isFinite(t.q_soft_min_pitch_rad) ||
      !Number.isFinite(t.q_soft_max_pitch_rad) ||
      !(t.q_soft_max_pitch_rad > t.q_soft_min_pitch_rad)) return null;
  return .5 * (t.q_soft_min_pitch_rad + t.q_soft_max_pitch_rad);
}
// Displayed pitch is UP-POSITIVE, whatever the joint's sign (owner, 2026-10-02: negative means the
// camera points down). The joint's measured screen sign (direction_contract: positive CyberGear pitch
// points the camera down) is what turns one into the other, so a remounted drive changes one constant.
const HUD_PITCH_UP = -otaJointScreenSign.pitch;
function hudPitch(t, raw) {
  const center = hudPitchCenter(t);
  if (center === null || !Number.isFinite(raw)) return null;
  const v = (raw - center) * HUD_PITCH_UP;
  return Math.abs(v) < 1e-12 ? 0 : v;   // no "-0.00" for a camera that is level
}
// The displayed pitch range, low end first, whichever joint limit each end came from.
function hudPitchRange(t) {
  const a = hudPitch(t, t && t.q_soft_min_pitch_rad), b = hudPitch(t, t && t.q_soft_max_pitch_rad);
  return (a === null || b === null) ? null : { lo: Math.min(a, b), hi: Math.max(a, b) };
}
// Yaw on a continuous axis, as an angle: (-180, 180].
function hudWrapDeg(d) {
  if (!Number.isFinite(d)) return d;
  const w = ((d + 180) % 360 + 360) % 360 - 180;
  return w === -180 ? 180 : w;
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
    ["Q YAW / PITCH", num(t.q_yaw_rad) + " / " + num(hudPitch(t, t.q_pitch_rad)) + " DEG"],
    ["REF YAW / PITCH", num(t.q_ref_yaw_rad) + " / " + num(hudPitch(t, t.q_ref_pitch_rad))],
    ["CMD RATE YAW", num(t.q_ref_rate_yaw_rad_s) + " DEG/S"],
    ["CMD ACCEL YAW", num(t.q_ref_accel_yaw_rad_s2) + " DEG/S2"],
    ["LIMITS YAW", t.yaw_envelope === "none"
      ? "BAND " + num(t.yaw_band_min_rad, 1) + " ... " + num(t.yaw_band_max_rad, 1)
        + " DEG (no boundary)"
      : num(t.q_soft_min_yaw_rad, 1) + " ... " + num(t.q_soft_max_yaw_rad, 1) + " DEG"],
    ["LIMITS PITCH", (hudPitchRange(t) ? num(hudPitchRange(t).lo, 1) + " ... " + num(hudPitchRange(t).hi, 1) : "--") + " DEG"],
    ["TRACK RATE", (typeof t.camera_fps === "number" ? t.camera_fps.toFixed(1) : "--") + " HZ"],
    ["SELECTED CONF", (typeof t.selected_confidence === "number"
                        ? Math.round(t.selected_confidence * 100) + "%" : "--")],
    ["PREDICTION HORIZON", (typeof t.prediction_horizon_ms === "number"
                            ? String(t.prediction_horizon_ms) + " MS" : "--")],
    ["ESTIMATOR NIS", Number.isFinite(t.estimator_mahalanobis) ? t.estimator_mahalanobis.toFixed(2) : "--"],
    ["PROCESS NOISE SCALE", Number.isFinite(t.estimator_process_noise_scale) ? t.estimator_process_noise_scale.toFixed(2) : "--"],
    ["ESTIMATOR REJECTS", Number.isFinite(t.estimator_rejected) ? String(t.estimator_rejected) : "--"],
    // §10: with no cue the page used to say nothing, and "nothing" is how a broken intrinsics
    // match hid until somebody noticed the box was missing. The controller states a reason, so
    // the page shows it. LIVE rather than OK: §10 reserves unambiguous words for observations.
    ["PREDICTION", (t.prediction && t.prediction.valid === true) ? "LIVE"
                    : ((t.prediction && t.prediction.reason)
                       ? String(t.prediction.reason) : "--")],
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
    // The ceiling that actually binds. Round 40 traced the tracking reference to this value: it is
    // min(hold speed, payload profile v_max) and it is applied to the AUTO_TRACK proposal before the
    // confidence derate, so it answers "why will it not keep up" in a way no other field on this panel can.
    // It is a safety constant, not a tuning knob - which is precisely why it belongs where the operator can
    // read it and disagree with it.
    ["SPEED CEILING", (typeof t.effective_speed_ceiling_deg_s === "number"
      ? t.effective_speed_ceiling_deg_s.toFixed(1) + " DEG/S  (min of hold + payload profile)"
      : "UNKNOWN")],
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
    ["IMU", imuLabel(t.imu)]
  ];
}

// The IMU chip has four states because the two absences mean different work: a station with no
// IMU configured needs a deployment change, one with a trace and no samples needs the acquisition
// process looked at. Collapsing them into "ABSENT" would send the operator to the wrong place.
// A rate nobody measured yet says "rate n/m", never "0 hz": zero is a measurement, and this is
// not one.
function imuLabel(imu) {
  // No block at all is §20's own absence, and it keeps the word the dashboard has always used.
  if (!imu) return "ABSENT";
  // A block that says "nothing was configured here" is a deployment fact, not a dead sensor.
  if (imu.configured === false) return "NOT CONFIGURED";
  if (!imu.present) return "NO SAMPLES";
  if (!imu.fresh) return "STALE " + (typeof imu.age_ms === "number" ? imu.age_ms.toFixed(0) + "ms" : "?");
  const rate = typeof imu.rate_hz === "number" ? imu.rate_hz.toFixed(0) + "HZ" : "RATE N/M";
  const acc = typeof imu.game_rv_accuracy === "number" ? "/A" + imu.game_rv_accuracy : "";
  const gaps = (imu.stats && imu.stats.gaps) ? " G" + imu.stats.gaps : "";
  return "FRESH " + rate + acc + gaps;
}

// --- §10 prediction cue ----------------------------------------------------
//
// The one element on this page whose colour is a safety statement: §10 says the prediction "must not
// be green", because green on this display means measured - something the camera sees now. A
// prediction drawn in green is an intention wearing the uniform of an observation, and the operator
// cannot tell them apart at a glance. So the colour assertions below are not decoration.
function hudPredictionBox(o) {
  // This is a geometric prediction used by the controller. Overlap with the
  // measured target is valid; moving the cue to clear a box invents an offset.
  if (!o || !Number.isFinite(o.cx) || !Number.isFinite(o.cy) || !(o.w > 0) || !(o.h > 0)) return null;
  return { x: o.cx - o.w / 2, y: o.cy - o.h / 2, w: o.w, h: o.h,
           cx: o.cx, cy: o.cy, shifted: false };
}

function hudPredictionSvg(b, C, o) {
  // §10's shape: amber dashed square, small amber + at its centre, small PRED label.
  if (!b) return "";
  const amber = C.amber;
  const parts = [
    '<rect x="' + b.x + '" y="' + b.y + '" width="' + b.w + '" height="' + b.h +
      '" fill="none" stroke="' + amber + '" stroke-width="3" stroke-dasharray="6 4" opacity=".92"/>',
    '<line x1="' + (b.cx - 7) + '" y1="' + b.cy + '" x2="' + (b.cx + 7) + '" y2="' + b.cy +
      '" stroke="' + amber + '" stroke-width="3"/>',
    '<line x1="' + b.cx + '" y1="' + (b.cy - 7) + '" x2="' + b.cx + '" y2="' + (b.cy + 7) +
      '" stroke="' + amber + '" stroke-width="3"/>',
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

  // A continuous yaw axis: the map is the whole circle, -180..180, at the envelope's pitch range, and
  // every yaw on it is an angle rather than a count of turns (owner, 2026-10-02: the map used to be
  // the +/-90 deg reference band, so the turret at +104 deg was drawn off the map).
  const ring = !!o.continuousYaw;
  const wrapYaw = (pt) => (ring && Array.isArray(pt) ? [hudWrapDeg(pt[0]), pt[1]] : pt);
  if (ring) {
    const ps = o.pts.map((p) => p[1]);
    const lo = Math.min.apply(null, ps), hi = Math.max.apply(null, ps);
    o = Object.assign({}, o, { pts: [[-180, lo], [180, lo], [180, hi], [-180, hi]],
                               los: wrapYaw(o.los), target: wrapYaw(o.target), pred: wrapYaw(o.pred) });
  }
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
  // The inset is spatial: left/right/up/down match the camera aim and D-pad.
  // Numeric labels remain joint degrees. Both conversions use the same measured
  // signs as the D-pad and travel tapes.
  // Pitch arrives up-positive (hudPitch), so up the map is up.
  const toPx = (yawDeg, pitchDeg) => [
    cx + (yawDeg - midY) * k * otaJointScreenSign.yaw,
    cy - (pitchDeg - midP) * k
  ];

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
  const fov = { x: los.x - fovW / 2, y: los.y - fovH / 2, w: fovW, h: fovH };
  // On the ring the view across +/-180 is one view: the part past one edge is drawn at the other.
  let fovWrap = null;
  if (ring) {
    const turn = 360 * k;
    if (fov.x < plot.x) fovWrap = Object.assign({}, fov, { x: fov.x + turn });
    else if (fov.x + fov.w > plot.x + plot.w) fovWrap = Object.assign({}, fov, { x: fov.x - turn });
  }
  const g = {
    x: x, y: y, w: w, h: h, plot: plot, scale: k, ring: ring,
    envPx: o.pts.map((pt) => { const q = toPx(pt[0], pt[1]); return { x: q[0], y: q[1] }; }),
    fov: fov, fovWrap: fovWrap,
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
    // The view rectangle is clipped to the map; on a ring its overhang reappears at the other edge.
    '<clipPath id="for-plot-clip"><rect x="' + g.plot.x + '" y="' + (g.plot.y - g.plot.h) +
      '" width="' + g.plot.w + '" height="' + (3 * g.plot.h) + '"/></clipPath>',
    [g.fov].concat(g.fovWrap ? [g.fovWrap] : []).map((r) =>
      '<rect x="' + r.x + '" y="' + r.y + '" width="' + r.w + '" height="' + r.h +
      '" fill="none" stroke="' + C.white + '" stroke-width="1" opacity=".95"' +
      (g.ring ? ' clip-path="url(#for-plot-clip)"' : '') + '/>').join(""),
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
  // How much travel the window shows. The owner's question -- "why 40 deg? that is
  // neither bigger nor smaller than the FOV on purpose" -- was the whole objection: a
  // made-up window size makes the tape a decorative ruler. The window is therefore the
  // CAMERA'S OWN FIELD OF VIEW on that axis (the commissioned `effective_hfov_deg` /
  // `effective_vfov_deg` the safe-envelope polygon already uses), so the tape shows
  // exactly the arc the operator can see, the caret is the boresight at its centre, and
  // the ruler slides under it as the view sweeps past the travel. Before the camera has
  // reported a usable FOV there is nothing to inherit, and the tape says which source it
  // used rather than quietly picking a number.
  const fov = Number.isFinite(o.windowDeg) ? o.windowDeg : 0;
  const windowDeg = Math.min(span, fov > 0 ? fov : Math.max(30, span / 4));
  const windowSource = fov > 0 ? (fov >= span ? "travel" : "fov") : "fallback";
  const steps = hudTickSteps(windowDeg, o.length);
  // Yaw is drawn in joint degrees with its measured screen sign; pitch arrives already up-positive
  // (hudPitch), so up the screen is always up the scale.
  const screenSign = o.horizontal ? otaJointScreenSign.yaw : -1;

  // The MARKER never moves and the SCALE always slides (owner, 2026-09-28, second pass:
  // "在一定角度之后 marker 就会动 —— 我希望 marker 永远不动，只动条带"). The first pass clamped
  // the window inside the travel, which held the caret centred mid-travel but shoved it aside
  // near an end. He rejected that, and with it the dead region I would have had to draw past
  // the end: past the end this tape simply ROLLS OVER, which is what a cyclic axis's ruler
  // is. The window is pinned to the value and never clamped, and the degrees beyond an
  // endpoint are the degrees at the other end -- the seam is a real place on a real axis.
  const lo = o.horizontal ? o.x : o.y;
  const hi = o.horizontal ? o.x + o.length : o.y + o.length;
  const mid = (lo + hi) / 2;
  const half = windowDeg / 2;
  const slope = screenSign * o.length / windowDeg;   // px per degree, direction included
  const centreDeg = Number.isFinite(o.valueDeg) ? o.valueDeg : (o.minDeg + o.maxDeg) / 2;
  const at = (deg) => mid + (deg - centreDeg) * slope;
  // Cyclic coordinates: the declared travel is one cycle of this ruler, period `span`. yaw
  // is a continuous axis so wrapping is what the world already does; pitch is physically
  // blocked and usually never reaches the seam, but it is drawn by the same rule -- one
  // widget, one behaviour, no second design somebody has to remember.
  const wrap = (deg) => {
    const w = (deg - o.minDeg) % span;
    return o.minDeg + (w < 0 ? w + span : w);
  };
  // Ticks dissolve into the last stretch of each end rather than being cut off: that is
  // how a tape says "there is more of this" without spending an element on the idea.
  const FADE = Math.max(18, o.length * 0.09);  // proportional: an absolute px band would
  // appear or vanish depending on how wide the browser made the tape
  const opacityAt = (pos) => {
    const d = Math.min(pos - lo, hi - pos);
    if (d <= 0) return 0;
    return d >= FADE ? 1 : Math.round((0.15 + 0.85 * d / FADE) * 100) / 100;
  };

  const ticks = [];
  const onGrid = (deg, step) => Math.abs(deg / step - Math.round(deg / step)) < 1e-9;
  // A continuous axis (yaw with no envelope) has no ends: its ruler is the whole circle, and +/-180
  // is a direction like any other, so nothing is drawn or labelled as a limit there.
  const continuous = !!o.continuous;
  const push = (deg, forced) => {
    const pos = at(deg), w = wrap(deg);
    const atSeam = !continuous && (Math.abs(w - o.minDeg) < 1e-6 || Math.abs(w - o.maxDeg) < 1e-6);
    const dup = ticks.filter((t) => Math.abs(t.pos - pos) < 1e-6);   // the seam is ONE place
    if (dup.length) {
      if (forced || atSeam) dup.forEach((t) => {
        t.endpoint = true; t.coarse = true; t.label = hudDegLabel(w, true); });
      return;
    }
    const coarse = forced || atSeam || onGrid(w, steps.coarse);
    const shown = (continuous && Math.abs(Math.abs(w) - 180) < 1e-6) ? "180" : hudDegLabel(w, !!atSeam || !!forced);
    ticks.push({ deg: w, pos: pos, coarse: coarse, endpoint: !!(forced || atSeam),
                 label: coarse ? shown : "" });
  };
  // Every fine step the window shows, indexed in window coordinates, labelled in cycle ones.
  for (let k = Math.floor((centreDeg - half) / steps.fine) - 1;
       k <= Math.ceil((centreDeg + half) / steps.fine) + 1; ++k) push(k * steps.fine, false);
  // The seam itself, drawn even when it misses the grid: where the ruler rolls over is
  // information, and on a continuous axis it is the only "endpoint" there ever is.
  for (let cyc = -2; cyc <= 2 && !continuous; ++cyc) {
    [o.minDeg, o.maxDeg].forEach((lim) => {
      const cand = lim + cyc * span;
      if (cand >= centreDeg - half - steps.fine && cand <= centreDeg + half + steps.fine)
        push(cand, true);
    });
  }
  ticks.sort((a, b) => a.pos - b.pos);

  // §22 asks a DERATE indication to include the relevant travel-tape edge. The tape's ends ARE the soft
  // limits - the same numbers the limiter is acting on - so the honest way to do this is to light up the
  // end being named, not to add a separate marker that could disagree with the scale it sits on.
  // Matched on the degree value rather than on index, because which tick is an endpoint depends on
  // whether the limit happened to fall on a fine step.
  if (typeof o.markDeg === "number") {
    // §22: a DERATE indication names the tape edge it is about, and the tape lights that
    // end amber. Matching on the degree value broke the day the ruler became cyclic --
    // +100 wraps to -100, so the mark matched nothing and the amber vanished silently.
    // What survives the wrap is the PIXEL a limit maps to, plus its whole-cycle images.
    const eps = steps.fine * Math.abs(slope) / 2;
    ticks.forEach((tk) => {
      for (let cyc = -2; cyc <= 2; ++cyc) {
        if (Math.abs(tk.pos - at(o.markDeg + cyc * span)) < eps) {
          // Name the limit that was named: the tick at the seam may have been labelled from
          // the other end of the cycle, and an amber highlight about +100 that reads "-100"
          // is worse than no highlight.
          tk.marked = true; tk.deg = o.markDeg; tk.label = hudDegLabel(o.markDeg, true); break;
        }
      }
    });
  }

  // Only what the window can show is drawn -- a label past an end would land on whatever
  // HUD element lives beside the tape. Survivors carry their own opacity.
  const shown = ticks.filter((tk) => tk.pos >= lo - 0.5 && tk.pos <= hi + 0.5)
                     .map((tk) => {
                       // The fade is for the middle of the scale running out of view. An
                       // endpoint never fades to nothing: the limit you are approaching is
                       // the one label that has to stay readable, and the first version of
                       // this painted it to opacity 0 exactly where it mattered.
                       tk.opacity = tk.endpoint ? Math.max(0.6, opacityAt(tk.pos))
                                                : opacityAt(tk.pos);
                       return tk;
                     });
  const seams = shown.filter((tk) => tk.endpoint).length;

  // Clamped along the tape's own axis. The first version clamped the vertical case between o.x and
  // o.x - the line's own column - because the horizontal variable was reused without being thought
  // about, and every pitch marker collapsed onto the tape's x-coordinate. Hand arithmetic caught it
  // (expected 593.7, produced 1842.0); a test now carries that arithmetic.
  const marker = mid;   // literally always: the caret is the vehicle, the world moves
  return {
    horizontal: !!o.horizontal, x: o.x, y: o.y, length: o.length,
    x1: o.horizontal ? o.x + o.length : o.x, y1: o.horizontal ? o.y : o.y + o.length,
    minDeg: o.minDeg, maxDeg: o.maxDeg, steps: steps, ticks: shown, marker: marker,
    centreDeg: centreDeg, seams: seams, cyclic: true,
    windowDeg: windowDeg, windowSource: windowSource,
    valueDeg: o.valueDeg,
    // §6.3: the value box is a dark translucent fill with a thin green outline. Sized for
    // "PITCH -12.3 deg" at the label size, and always placed where it cannot leave the viewport.
    box: { w: 96, h: 34, x: 0, y: 0 },
    note: ""
  };
}

function hudDegLabel(deg, withDegree) {
  if (!Number.isFinite(deg)) return "--";
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
  // Every line on these scales is drawn twice: a dark line ~3px wide, then the green one on top of it.
  // That is the whole contrast story over a white curtain or a window -- and it is deliberately not a
  // blurred glow, which reads as decoration and disappears against a highlight.
  const spine = (w ? 'x1="' + t.x + '" y1="' + t.y + '" x2="' + t.x1 + '" y2="' + t.y
                 : 'x1="' + t.x + '" y1="' + t.y + '" x2="' + t.x + '" y2="' + t.y1);
  parts.push('<line ' + spine + '" stroke="' + C.stroke + '" stroke-width="3" opacity=".9"/>');
  parts.push('<line ' + spine + '" stroke="' + base + '" stroke-width="1" opacity=".85"/>');
  t.ticks.forEach((tk) => {
    const len = tk.marked ? 17 : (tk.endpoint ? 13 : (tk.coarse ? 10 : 5));
    const col = tk.marked ? C.amber : (tk.coarse ? base : fine);   // §22: caution is amber
    const geom = (w ? 'x1="' + tk.pos + '" y1="' + t.y + '" x2="' + tk.pos + '" y2="' + (t.y + len)
                  : 'x1="' + t.x + '" y1="' + tk.pos + '" x2="' + (t.x - len) + '" y2="' + tk.pos);
    // Major ticks hold the green; minor ticks keep the hue but drop back, so the scale can be read
    // at a glance instead of as a comb of equal-weight marks.
    const weight = typeof tk.opacity === "number" ? tk.opacity : (tk.coarse || tk.marked ? 1 : 0.55);
    parts.push('<line ' + geom + '" stroke="' + C.stroke + '" stroke-width="2.6" opacity=".85"/>');
    parts.push('<line ' + geom + '" stroke="' + col + '" stroke-width="1" opacity="' + weight + '"/>');
    if (tk.label) {
      parts.push('<text class="tlbl" ' +
        (w ? 'x="' + tk.pos + '" y="' + (t.y - 7) + '" text-anchor="middle"'
           : 'x="' + (t.x + 8) + '" y="' + (tk.pos + 4) + '" text-anchor="start"') +
        ' fill="' + (tk.marked ? C.amber : lbl) + '" opacity="' +
         (typeof tk.opacity === "number" ? tk.opacity : 1) + '">' + tk.label + '</text>');
    }
  });
  // Current-position caret (§5.2) and its value box. Drawn last inside the group so it sits over the
  // ticks it overlaps.
  const mk = t.marker;
  // The current-position caret is the strongest thing on the scale, and it earns that with a dark
  // outline rather than with size: the geometry the operator has learned is 12px wide either way.
  parts.push(w
    ? '<path d="M ' + mk + ' ' + (t.y + 2) + ' L ' + (mk - 6) + ' ' + (t.y + 12) + ' L ' +
      (mk + 6) + ' ' + (t.y + 12) + ' Z" fill="' + C.green + '" stroke="' + C.stroke +
      '" stroke-width="1.4" stroke-linejoin="round"/>'
    : '<path d="M ' + (t.x - 2) + ' ' + mk + ' L ' + (t.x - 12) + ' ' + (mk - 6) + ' L ' +
      (t.x - 12) + ' ' + (mk + 6) + ' Z" fill="' + C.green + '" stroke="' + C.stroke +
      '" stroke-width="1.4" stroke-linejoin="round"/>');
  // The value box sits against the caret on the picture side of its tape, for both tapes (owner,
  // 2026-10-02: the pitch label belongs where the yaw label is, not at the far end of the tape).
  const bx = w ? Math.max(4, Math.min(mk - 48, (opts && opts.vw ? opts.vw - 100 : mk)))
               : Math.max(4, t.x - 16 - t.box.w);
  const by = w ? (t.y + 16) : (mk - t.box.h / 2);
  parts.push('<rect x="' + bx + '" y="' + by + '" width="' + t.box.w + '" height="' + t.box.h +
             '" fill="' + C.black + '" stroke="' + C.green + '" stroke-width="1" rx="2"/>');
  parts.push('<text class="tval" x="' + (bx + t.box.w / 2) + '" y="' + (by + 14) +
             '" text-anchor="middle" fill="' + C.green + '">' + (opts && opts.title ? opts.title : "") +
             '</text>');
  // The dark box stays (it is the one element that was already right), and the number inside it is
  // green: yaw and pitch are level-1 information, which is the whole point of thinning the green out
  // everywhere else.
  parts.push('<text class="tval" x="' + (bx + t.box.w / 2) + '" y="' + (by + 28) +
             '" text-anchor="middle" fill="' + C.green + '" font-weight="600">' +
             (opts && opts.value ? opts.value : "") + '</text>');
  // What the scale actually is, stated on the tape that uses it. §5.3 asks for logical joint travel
  // and forbids compass letters, which the drawing honours - but on this station the joint numbers
  // are surprising enough to be misread: yaw travels -22.6 to +320.2 deg (the config says in terms:
  // "YAW IS A ~360 DEG CONTINUOUS-ROTATION AXIS") and pitch sits -74.7 to -4.9, which an operator
  // will read as elevation unless told otherwise. It is not elevation. The theodolite probe records
  // that camera-to-axis boresight is NOT separable from the principal point at the spans available
  // here, so the world elevation of this scale's zero has never been measured, and the tape says so
  // rather than borrowing an offset from somebody's recollection - mine included.
  // No caption under the value box (owner, 2026-09-28: "简单就是更好"). What used to sit here
  // read "JOINT TRAVEL, NOT HEADING" and "0 = TRAVEL MIDPOINT"; neither changed how anyone
  // read the tape. The knowledge stays where it is acted on: no compass letters are drawn
  // anywhere on this HUD, and pitch is joint travel -- not elevation, because the theodolite
  // probe never separated camera-to-axis boresight from the principal point, so there is no
  // measured offset to borrow. Say that in the design doc, not on the glass.
  return parts.join("");
}

function hudYawTapeRange(t) {
  // Where the yaw tape's endpoints come from, as a function so it can be executed instead of
  // read. Two sources, and the difference between them is stated by `ruler`:
  //   · an axis with a declared envelope -- the soft limits themselves;
  //   · `yaw_envelope: "none"` -- the *reference band* the station file still declares, centred
  //     on the homing origin. Free rotation needs nothing to stop it; an operator still wants
  //     to know how far the barrel has travelled since it was zeroed, and a ruler is not a wall.
  // Nothing to show (no homing, or a band that isn't a band) and the page falls back to the
  // unranged note rather than drawing a tape out of zeros.
  // With no envelope the axis is a full circle (owner, 2026-10-02: the ruler stopped at the +/-90 deg
  // reference band while the turret pointed at +104). The band is a homing reference, not travel.
  const toDeg = (r) => (Number.isFinite(r) ? r * 180.0 / Math.PI : NaN);
  const unbounded = String(t && t.yaw_envelope || "") === "none";
  const minDeg = unbounded ? -180 : toDeg(t.q_soft_min_yaw_rad);
  const maxDeg = unbounded ? 180 : toDeg(t.q_soft_max_yaw_rad);
  return {
    minDeg: minDeg, maxDeg: maxDeg, ruler: unbounded, continuous: unbounded,
    valid: Boolean(t && t.soft_limits_valid === true) &&
      Number.isFinite(minDeg) && Number.isFinite(maxDeg) && maxDeg > minDeg
  };
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

function deg(rad) { return Number.isFinite(rad) ? rad * 180.0 / Math.PI : NaN; }

// §15 color tokens, verbatim from the revision.
// ---------------------------------------------------------------------------
// Reticle cant (roll) reference. UI foundation only: nothing here reads the IMU, and the value is
// not wired to anything. Convention, screen space: 0 deg = horizontal, POSITIVE rotates the line
// CLOCKWISE on screen, negative counter-clockwise. Whatever the IMU's roll sign eventually means is
// a mapping problem for whoever connects it -- one expression, at one place, in front of this number.
//
// This is the single value that renders the line. To check the geometry by eye during development:
//     otaSetReticleCant(5)      // or -5, 2, -2, 0
// which repaints from the last telemetry it saw. There is deliberately no operator-facing control:
// an operator cannot change the roll of the camera by pressing a button.
let reticleCantDeg = 0;

const C = {
  // The dark partner of every primary overlay: a line over arbitrary video is only readable with
  // something non-luminous under it. Kept in the palette so no drawing site invents its own black.
  stroke: "#05070a", text: "#c5d0c5", text_dim: "#8c998c",
  green: "#95f58b", dim: "rgba(149,245,139,.56)", faint: "rgba(149,245,139,.22)",
  amber: "#f2b329", red: "#ff5d5d", white: "#edf2eb", black: "rgba(3,6,5,.80)",
  line: "rgba(230,245,230,.24)"
};

let lastTelemetry = null;
let lastTelemetryAt = 0;
// Consecutive ticks on which only the PAGE's own clock said 'quiet'. The §25 verdict is the
// station's condition; a browser that stalled decoding 1080p MJPEG is not the station losing
// telemetry, and a WebSocket that closed one second before its scheduled reconnect never said
// the station stopped. Both used to paint the overlay instantly or on a single tick, which is
// the flash the operator reported seeing.
let linkOverdue = 0;
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
  d.className = "chip " + (state === "red" ? "bad" : (state === "amber" ? "warn" : "ok"));
  const dot = state === "red" ? C.red : (state === "amber" ? C.amber : C.green);
  d.innerHTML = '<span class="dot" style="background:' + dot + '"></span>' +
                '<span class="lbl">' + label + '</span>' +
                (value ? '<span class="val">' + value + '</span>' : "");
  return d;
}

function render(t) {
  renderManualPad(t);
  const video = $("video"), svg = $("overlay");
  if (!video || !svg) return;

  const vw = window.innerWidth, vh = window.innerHeight;
  const iw = video.naturalWidth || 0, ih = video.naturalHeight || 0;
  const lay = hudLayout(vw, vh, iw, ih);
  const view = hudMainView(t);
  lay.k = view.k;
  if (window.otaPanesFollow) window.otaPanesFollow(view);
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
  const bore = hudBoreMark(t, stale);
  const c = hudProject(bore ? bore.u : axis.u, bore ? bore.v : axis.v, lay);
  if (c.ok) {
    const g = bore ? C.amber : C.green, r = 26, gap = 8, len = 12;
    const corner = (sx, sy) =>
      '<path d="M ' + (c.x + sx * r) + ' ' + (c.y + sy * gap) + ' L ' + (c.x + sx * r) + ' ' +
      (c.y + sy * r) + ' L ' + (c.x + sx * gap) + ' ' + (c.y + sy * r) + '" fill="none" ' +
      'stroke="' + g + '" stroke-width="3"/>';
    // The cant line, from the geometry module (see hudReticleCantSvg for why it is one line). The
    // corner brackets are the aiming reference and neither rotate nor move with cant.
    const cant = hudReticleCantSvg(c.x, c.y, r + 12, gap + 8, reticleCantDeg,
                                   {stroke: C.stroke, line: g});
    layers.reticle =
      corner(-1, -1) + corner(1, -1) + corner(-1, 1) + corner(1, 1) + cant[0] + cant[1] +
      (intr ? "" : '<text x="' + (c.x + r + 18) + '" y="' + (c.y + 4) + '" class="lbl" ' +
        'fill="' + C.amber + '">RETICLE UNCALIBRATED (assumed centre)</text>');
    if (bore) {
      const optical = hudProject(axis.u, axis.v, lay);
      if (optical.ok) layers.reticle += '<circle cx="' + optical.x + '" cy="' + optical.y +
        '" r="3" fill="none" stroke="' + C.dim + '" stroke-width="1"/>';
      layers.reticle += '<text x="' + (c.x+36) + '" y="' + (c.y+25) +
        '" class="lbl" fill="' + C.amber + '">' + bore.label + '</text>';
    } else if (t.alignment && t.alignment.mode === "manual_depth") {
      layers.reticle += '<text x="' + (c.x+36) + '" y="' + (c.y+25) +
        '" class="lbl" fill="' + C.amber + '">BORE ALIGNMENT UNAVAILABLE</text>';
    }
  }
  layers.sel += hudMeasurementPointSvg(t, lay, stale, C);

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
      const sel = tracks.find(x => x && x.selected);
      const measured = sel ? hudProject(sel.anchor_x, sel.anchor_y, lay) : null;
      layers.pred = hudPredictionSvg(
        hudPredictionBox({ cx: a.x, cy: a.y, w: 28, h: 28 }), C,
        { near: measured && measured.ok ? [measured.x, measured.y] : null });
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
  const yawRange = hudYawTapeRange(t);
  const yawTape = hudTravelTape({
    horizontal: true, x: vw * (1 - 0.575) / 2, y: vh * 0.125, length: vw * 0.575,
    minDeg: yawRange.minDeg, maxDeg: yawRange.maxDeg,
    markDeg: (dEdge && dEdge.axis === "YAW")
      ? (dEdge.side === "MIN" ? yawRange.minDeg : yawRange.maxDeg) : undefined,
    valueDeg: deg(t.q_yaw_rad), windowDeg: hudViewFov(t.effective_hfov_deg, view.k),
    valid: yawRange.valid, continuous: yawRange.continuous
  });
  // A continuous yaw reads as an angle, not as a count of turns since homing.
  const yawShown = yawRange.continuous ? hudWrapDeg(deg(t.q_yaw_rad)) : deg(t.q_yaw_rad);
  const pitchLen = vh * 0.425;
  const pitchSpan = hudPitchRange(t);
  const pitchTape = hudTravelTape({
    horizontal: false, x: vw - Math.max(78.0, vw * 0.055), y: vh / 2 - pitchLen / 2,
    length: pitchLen, minDeg: pitchSpan ? deg(pitchSpan.lo) : NaN,
    maxDeg: pitchSpan ? deg(pitchSpan.hi) : NaN,
    markDeg: (dEdge && dEdge.axis === "PITCH")
      ? (dEdge.side === "MIN" ? deg(hudPitch(t, t.q_soft_min_pitch_rad)) : deg(hudPitch(t, t.q_soft_max_pitch_rad)))
      : undefined,
    valueDeg: deg(hudPitch(t, t.q_pitch_rad)), windowDeg: hudViewFov(t.effective_vfov_deg, view.k),
    valid: t.soft_limits_valid === true
  });
  // §11: the FOR inset, drawn from the daemon's own block. The coordinate_frame check is not
  // ceremony - if the server ever starts sending a polygon in a different frame, drawing it as joint
  // travel would be quietly wrong rather than visibly wrong, and this station has already been burned
  // once by a number whose frame was assumed.
  const forB = (t.field_of_regard && typeof t.field_of_regard === "object") ? t.field_of_regard : null;
  layers.for = "";
  if (hudPitchCenter(t) !== null && forB && forB.valid === true && forB.coordinate_frame === "joint_deg" &&
      Array.isArray(forB.safe_envelope_points) && t.effective_hfov_deg > 0 &&
      t.effective_vfov_deg > 0 && typeof t.q_yaw_rad === "number" &&
      typeof t.q_pitch_rad === "number") {
    const hasIntent = t.intent_has_joint_target === true &&
                      typeof t.intent_q_yaw_rad === "number" && typeof t.intent_q_pitch_rad === "number";
    // PRED is the controller's resolved predicted aim, not the intermediate
    // trajectory reference. Hide it when there is no active prediction.
    const hasAim = !stale && pred && pred.valid === true && t.tracking_aim_joint_valid === true &&
                   Number.isFinite(t.tracking_aim_yaw_rad) && Number.isFinite(t.tracking_aim_pitch_rad);
    const gi = hudForInset({
      vw: vw, vh: vh, continuousYaw: yawRange.continuous,
      pts: forB.safe_envelope_points.map(p => [p[0], deg(hudPitch(t, p[1] * Math.PI / 180.0))]),
      hfovDeg: hudViewFov(t.effective_hfov_deg, view.k),
      vfovDeg: hudViewFov(t.effective_vfov_deg, view.k),
      los: [deg(t.q_yaw_rad), deg(hudPitch(t, t.q_pitch_rad))],
      target: hasIntent ? [deg(t.intent_q_yaw_rad), deg(hudPitch(t, t.intent_q_pitch_rad))] : null,
      pred: hasAim ? [deg(t.tracking_aim_yaw_rad), deg(hudPitch(t, t.tracking_aim_pitch_rad))] : null
    });
    layers.for = hudForInsetSvg(gi, C);
  }

  layers.tape =
    hudTravelTapeSvg(yawTape, C, { title: "YAW", vw: vw, vh: vh,
                                   value: hudDegLabel(yawShown, true) }) +
    hudTravelTapeSvg(pitchTape, C, { title: "PITCH", vw: vw, vh: vh,
                                     value: hudDegLabel(deg(hudPitch(t, t.q_pitch_rad)), true) }) +
    ((yawTape || pitchTape) ? ""
     : hudUnrangedNote(vw / 2, vh * 0.125, "YAW / PITCH"));

  $("g-candidates").innerHTML = layers.cand;
  $("g-selected").innerHTML = layers.sel;
  $("g-reticle").innerHTML = layers.reticle;
  $("g-prediction").innerHTML = layers.pred;
  $("g-for").innerHTML = layers.for;
  $("g-tapes").innerHTML = layers.tape;

  // The requested measurement point is a white diamond; the assumed bore sight
  // is an amber reticle. The optical axis remains a separate camera-centre mark.

  // §4.1 mode block, §21's state wording. Three lines, first line strongest.
  const st = hudStateLabel({ mode: t.operating_mode, phase: t.mode_phase, supervisory: t.phase,
                             rest: t.rest_park, jogging: !!t.manual_lease_active });
  if (window.otaPipNoteStreams) window.otaPipNoteStreams(t.video_streams);
  $("mode-block").innerHTML =
    '<div class="m1">' + st.line1 + '</div>' +
    '<div class="m2' + (st.named ? "" : " raw") + '">' + st.line2 + '</div>' +
    '<div class="m3">' + escapeMarkup(t.selected_label || t.selected_descriptor || (t.selected_display_index ? "Person #" + t.selected_display_index : "--")) +
    '</div>';

  // §8 health chips. Anything the snapshot does not carry is shown as absent, because
  // an HUD that invents health is worse than one that admits a gap.
  const hs = $("health");
  hs.innerHTML = "";
  // `controld_connected` is a /api/health field, not a field of the telemetry snapshot: reading it off
  // `t` asked for a key that is never published there, so the chip was red since the day it shipped
  // (it is red in the owner's screenshot from before any of today's changes). transportOk is the page's
  // own verdict, computed from /api/health and the socket, and it is the honest source.
  // The field is published on this payload now (webd's decorate), so the chip reads it as the ledger
  // says it does; transportOk is the fallback for a snapshot that predates the field, not a second
  // opinion. Before this, the expression was `!!t.controld_connected` against a key that was never
  // published here -- which is why the chip was red in every screenshot since it shipped.
  const connected = (typeof t.controld_connected === "boolean") ? t.controld_connected
                                                                : (transportOk === true);
  hs.appendChild(chip("CONNECTED", connected ? "ok" : "red"));
  const serviceReady = t.phase === "hold" && t.soft_limits_valid && t.supervisory_state === "READY";
  hs.appendChild(chip(serviceReady ? "HOMED" : "NOT READY", serviceReady ? "ok" : "amber"));
  const vis = (typeof t.vision_track_sets === "number" && t.vision_track_sets > 0) ? "ok" : "amber";
  hs.appendChild(chip("VISION", vis, vis === "ok" ? "" : "NO SETS"));
  // Was a hardcoded amber "ABSENT" -- an unconditional assertion on the one strip the operator
  // actually reads. The four states exist because they mean different work; the chip now derives
  // from the same payload the drawer uses.
  {
    const lbl = imuLabel(t.imu);
    const st = lbl.indexOf("FRESH") === 0 ? "ok" : "amber";
    // State, not details (owner, 2026-09-29): the row is a glance, and rate / accuracy / the sensor's
    // own attitude are still on the wire and in the drawer for anyone who asks.
    // FRESH alone is the glance; any other state keeps its words ("NO SAMPLES" read as "NO" on
    // 2026-10-03, the day the folded bar made the chip pop out).
    hs.appendChild(chip("IMU", st, st === "ok" ? lbl.split(" ")[0] : lbl));
  }
  {
    // Which network is actually producing the tracks -- the fact the whole Hailo switch turns on, and
    // the one thing a station running the other backend would otherwise hide behind a working picture.
    const nf = t.inference || {};
    // Any Hailo adapter is the accelerator in use: the station's is "hailo_pose", and a test for exactly
    // "hailo" kept this chip amber on a healthy station (owner's screenshot, 2026-10-03).
    const state = !nf.present ? "red" : (nf.fresh ? (/^hailo/.test(String(nf.adapter || "").toLowerCase()) ? "ok" : "amber") : "amber");
    // Only the backend's name (owner, 2026-09-29): HAILO or IMX500. The model id, the leg it reads and
    // the produced-versus-kept counters stay on the wire, where I read them to diagnose exactly what
    // is wrong tonight (emitted 91 671 frames' worth, tracks 0), without spending the operator's row.
    const shown = !nf.present ? "NO REPORT"
      : (!nf.fresh ? "STALE"
                   : String(nf.adapter || "?").toUpperCase());
    hs.appendChild(chip("NN", state, shown));
  }
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
  // The strip merged with the health chips (owner, 2026-10-03). MODE, STATE and SAFETY left it: the
  // mode block and the safety banner say them already, louder. What stays are readings.
  const cell = (k, v, cls) => '<span class="k">' + k + '</span><span class="' + (cls || "v") + '">' + v + '</span>';
  $("strip-cells").innerHTML =
    cell("TARGETS", String(t.track_count == null ? "--" : t.track_count), "v") + '<span class="sep">|</span>' +
    cell("FPS", fmt(t.camera_fps, 0), "v") + '<span class="sep">|</span>' +
    cell("AGE", fmt(t.vision_measurement_age_ms, 0, " ms"), stale ? "warn" : "v");
  renderStatusFold();
  renderStats(t);
}

// Folded by default: one summary dot, and a healthy chip stays hidden. A chip that is not healthy
// is never folded away; it shows on its own until it recovers. Unfolding is remembered per browser.
let statusExpanded = hudPref("ota.hud.status.expanded", false);
function renderStatusFold() {
  const strip = $("strip"), fold = $("status-fold");
  strip.classList.toggle("folded", !statusExpanded);
  const bad = $("health").querySelectorAll(".chip.bad").length;
  const warn = $("health").querySelectorAll(".chip.warn").length;
  const tone = bad ? C.red : (warn ? C.amber : C.green);
  const text = bad || warn ? (bad + warn) + (bad + warn === 1 ? " ALERT" : " ALERTS") : "ALL OK";
  const html = '<span class="dot" style="background:' + tone + '"></span><span class="lbl">' + text +
               '</span><span class="arr">' + (statusExpanded ? "\u25C2" : "\u25B8") + "</span>";
  if (fold.innerHTML !== html) fold.innerHTML = html;
  fold.setAttribute("aria-expanded", String(statusExpanded));
}

// --- stats overlay -----------------------------------------------------------------------------
let statsOn = hudPref("ota.hud.stats", false), statsPaintedAt = 0;
function renderStats(t, force) {
  const box = $("stats");
  if (!statsOn) { if (!box.hidden) { box.hidden = true; box.innerHTML = ""; } return; }
  const now = Date.now();
  if (!force && !box.hidden && now - statsPaintedAt < 250) return;   // 4 Hz is enough to read
  statsPaintedAt = now;
  box.hidden = false;
  box.innerHTML = '<div class="stitle"><span>STATS</span><button type="button" data-ui="stats" ' +
    'aria-label="Close stats">\u00D7</button></div>' + hudStatsSections(t).map((s) =>
    '<div class="ssec">' + escapeMarkup(s.title) + "</div>" + s.rows.map((kv) =>
      '<div class="srow"><span class="sk">' + escapeMarkup(kv[0]) + '</span><span class="sv">' +
      escapeMarkup(kv[1]) + "</span></div>").join("")).join("");
}

function hudPref(key, fallback) {
  try { const v = localStorage.getItem(key); return v === null ? fallback : v === "1"; }
  catch (e) { return fallback; }
}
function hudSetPref(key, on) {
  try { localStorage.setItem(key, on ? "1" : "0"); } catch (e) { /* private window: this page only */ }
}
function setStats(on) {
  statsOn = !!on;
  hudSetPref("ota.hud.stats", statsOn);
  renderStats(lastTelemetry || {}, true);
  if (drawerOpen === "MENU") renderDrawer();
}

function paint(t) {
  // Track churn must not replace MENU buttons between pointer-down and click.
  // Phase changes must refresh their Home/recovery gates even with no tracks.
  const drawerKey = value => JSON.stringify(drawerOpen === "MENU"
    ? [value && value.phase, value && value.cmd_ack_seq, value && value.rest_park, value && value.rest_park_on_stop,
       hudSpeedRows(value).map(r => [r.value, r.max])]
    : [value && value.operating_mode, value && value.cmd_ack_seq,
      value && value.selected_uuid, value && value.perception_session_uuid,
      ((value && value.tracks) || []).map(x => [x.uuid, x.selected, x.selectable, x.state])]);
  const drawerChanged = drawerKey(lastTelemetry) !== drawerKey(t);
  lastTelemetry = t; lastTelemetryAt = Date.now();
  transportOk = true;
  resolveAckFromTelemetry(t);
  render(t);
  if (drawerOpen && drawerChanged) renderDrawer();
}

function updateStaleness(t) {
  // §25. Applies the verdict, and returns it so the caller can colour the fields it is about to
  // draw. Re-rendering is how "stops visual interpolation" is enforced rather than merely asserted:
  // the overlay is rebuilt from the payload that is on hand, and when that payload is old the whole
  // viewport desaturates and declares itself - nothing keeps drifting on numbers that died.
  const serverStaleNow = !!(t && t.telemetry_stale === true)
      || (typeof healthAgeMs === "number" && healthAgeMs > STALE_AFTER_MS);
  const msgAgeMs = lastTelemetryAt ? (Date.now() - lastTelemetryAt) : null;
  const rawVerdict = hudStale({
    // A closed socket is the page's condition, not the station's. While the last message is still
    // inside the quiet grace the reconnect is already in flight, so it must not assert §25 staleness;
    // if the link really is gone, the message-age path below declares it within the same grace.
    transportOk: transportOk || (msgAgeMs !== null && msgAgeMs < QUIET_AFTER_MS),
    telemetryStale: !!(t && t.telemetry_stale === true),
    telemetryAgeMs: healthAgeMs,
    msgAgeMs: lastTelemetryAt ? (Date.now() - lastTelemetryAt) : null,
    trackListAgeMs: (t && typeof t.track_list_age_ms === "number") ? t.track_list_age_ms : null,
    staleAfterMs: STALE_AFTER_MS,
    quietAfterMs: QUIET_AFTER_MS,
    trackAfterMs: TRACK_AFTER_MS
  });
  const vp = document.getElementById("viewport");
  // Server-clock verdicts act at once - they are the station's own words. Verdicts resting only on
  // this page's clock need two consecutive ticks, because a stalled main thread drains its queued
  // messages the moment it resumes and so can never produce two, while a dead link produces all of
  // them. Detection of a real loss is therefore delayed by at most one 250 ms tick, not hidden.
  if (rawVerdict && !serverStaleNow) { linkOverdue += 1; }
  else if (!rawVerdict) { linkOverdue = 0; }
  const verdict = serverStaleNow || (rawVerdict && linkOverdue >= 2);
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

// The two camera panes. They differ in which role they show -- the station decides that, see
// hudMainView -- and in where a refusal is written; they recover by exactly the same rule
// (hudPaneStep). The PIP once had its own, weaker path: it re-requested a stream nobody had started,
// so after every deploy it stayed frozen until the page was reloaded while the main picture healed.
const panes = {
  main: { el: "video", role: "wide", epoch: "", lastAttemptMs: 0, busy: false },
  pip: { el: "pipimg", role: "detail", epoch: "", lastAttemptMs: 0, busy: false, wanted: true }
};

function paneSays(pane, ok, why) {
  if (pane === panes.main) {
    notice(ok ? "" : "VIDEO UNAVAILABLE: " + why + " - symbology below is live, the picture is not");
  } else if ($("pipfps")) {
    if (!ok) $("pipfps").textContent = "refused: " + why;
  }
}

function pointPane(pane, epoch) {
  pane.epoch = epoch || "";
  const img = $(pane.el);
  // A new URL every time: the browser never retries a failed or ended <img> on its own.
  if (img) img.src = "/api/video?camera=" + pane.role + "&t=" + Date.now();
  paneSays(pane, true, "");
}

async function startPane(pane) {
  if (pane.busy) return false;
  pane.busy = true;
  pane.lastAttemptMs = Date.now();
  try {
    const r = await fetch("/api/video/start?camera=" + pane.role, {
      method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" });
    const j = await r.json();
    if (r.status >= 400 || j.ok === false || j.running === false) {
      paneSays(pane, false, j.error || ("HTTP " + r.status));
      return false;
    }
    pointPane(pane, j.epoch);
    return true;
  } catch (e) {
    paneSays(pane, false, String(e));
    return false;
  } finally {
    pane.busy = false;
  }
}

async function pollPanes() {
  for (const pane of [panes.main, panes.pip]) {
    if (pane.wanted === false) continue;
    let s = null;
    try { s = await (await fetch("/api/video/state?camera=" + pane.role)).json(); }
    catch (e) { continue; }
    const step = hudPaneStep(pane, s, Date.now());
    if (step === "start") await startPane(pane);
    else if (step === "point") pointPane(pane, s.epoch);
    if (pane === panes.pip && $("pipfps") && step !== "start") {
      // visiond's measured rate for the stream this pane shows; unmeasured reads "rate n/m".
      $("pipfps").textContent = (typeof s.delivered_fps === "number")
        ? (s.delivered_fps.toFixed(1) + " fps") : "rate n/m";
    }
  }
}

// An <img> error is how a stopped stream shows up: start it again (spaced by hudPaneStep, so a
// source that keeps refusing is asked every 1.5 s rather than in a tight loop of 409s).
function paneErrored(pane) {
  if (hudPaneStep(pane, { running: false }, Date.now()) === "start") startPane(pane);
}

// The station moved the main display (this page's swap, another page's, or a restart back to wide):
// both panes change role, and both are started afresh rather than waiting for a poll.
let paneSeen = { boot: "", generation: -1 };
window.otaPanesFollow = function (view) {
  const seen = hudAcceptMainView(paneSeen, view);
  if (!seen) return;                 // an older report than the swap this page already saw answered
  paneSeen = seen;
  if ($("pipswap")) $("pipswap").style.display = view.canSwap ? "" : "none";
  if (view.main === panes.main.role) return;
  panes.main.role = view.main;
  panes.pip.role = view.pip;
  for (const pane of [panes.main, panes.pip]) {
    pane.epoch = "";
    pane.lastAttemptMs = 0;
    if (pane.wanted !== false) startPane(pane);
  }
  if ($("piplabel")) $("piplabel").textContent = "PIP " + panes.pip.role;
};

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
    // Self-heal both panes: a stopped stream is started, a restarted one re-pointed, instead of
    // letting a frozen frame keep looking like a live one.
    await pollPanes();
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
  dock.innerHTML = hudDockSpecs({ open: drawerOpen }).filter((b) =>
    b.key !== "MANUAL" || (lastTelemetry && lastTelemetry.operating_mode === "MANUAL")).map((b) =>
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

function escapeMarkup(value) {
  return String(value).replace(/[&<>"']/g, c =>
    ({'&':'&amp;', '<':'&lt;', '>':'&gt;', '"':'&quot;', "'":'&#39;'}[c]));
}

function renderDrawer() {
  if (!drawerOpen) { drawer.hidden = true; drawer.innerHTML = ""; return; }
  let rows;
  {
    rows = hudDrawerActions(drawerOpen, lastTelemetry || {}).map((a) => {
      const inert = a.command === null || a.kind === "current" || a.kind === "gated";
      const cls = "drow " + (a.kind === "stop" ? "stop" : a.kind === "danger" ? "danger" :
                             a.kind === "gated" ? "gated" : a.kind === "current" ? "on" : "");
      const waiting = pendingConfirm === a.command;
      return '<button type="button" class="' + cls + (waiting ? " confirm" : "") + '" data-cmd="' +
             escapeMarkup(a.command || "") + '" data-arg="' + escapeMarkup(a.arg || "") + '" data-kind="' + a.kind + '"' +
             (inert ? " disabled" : "") + '><span class="rl">' +
             escapeMarkup(waiting ? "CONFIRM " + a.label : a.label) + '</span><span class="rn">' +
             escapeMarkup(waiting ? "PRESS AGAIN" : (a.note || "")) + "</span></button>";
    }).join("");
    if (drawerOpen === "MENU") rows += renderSettings(lastTelemetry || {});
  }
  drawer.innerHTML = '<div class="dtitle">' + drawerOpen + "</div>" + rows +
    '<div class="dack ' + lastAck.kind + '" role="status">' + escapeMarkup(lastAck.text || " ") + "</div>";
  drawer.hidden = false;
}

function renderSettings(t) {
  const btn = (arg, text, label) => '<button type="button" class="sbtn" data-cmd="set_speed" data-kind="act"' +
    ' data-arg="' + escapeMarkup(arg || "") + '" aria-label="' + label + '"' + (arg ? "" : " disabled") + ">" +
    text + "</button>";
  const speeds = hudSpeedRows(t).map((r) =>
    '<div class="drow setting"><span class="rl">' + r.label + '</span><span class="stepper">' +
    btn(r.down, "\u2212", r.label + " slower") +
    '<span class="sval">' + (r.value === null ? "--" : r.value.toFixed(1) + "\u00B0/s") + "</span>" +
    btn(r.up, "+", r.label + " faster") +
    btn(r.reset, "\u21BA", r.label + " back to " + (r.base === null ? "default" : r.base.toFixed(1))) +
    '</span><span class="snote">' + (r.base === null ? "" : "default " + r.base.toFixed(1)) +
    (r.max === null ? "" : " \u00B7 max " + r.max.toFixed(1)) + "</span></div>").join("");
  return '<div class="dsec">SETTINGS</div>' + speeds +
    '<div class="dnote">Speeds hold until the station restarts.</div>' +
    '<div class="drow setting gated"><span class="rl">' + HUD_TARGET_POLICY.label + '</span><span class="rn">' +
    HUD_TARGET_POLICY.value + '</span><span class="snote">' + HUD_TARGET_POLICY.note + "</span></div>" +
    '<button type="button" class="drow' + (statsOn ? " on" : "") + '" data-ui="stats" aria-pressed="' +
    String(statsOn) + '"><span class="rl">STATS FOR NERDS</span><span class="rn">' +
    (statsOn ? "ON" : "OFF") + "</span></button>";
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
  if (cmd === "select_uuid" || (cmd === "clear_target" && lastTelemetry && lastTelemetry.perception_native)) {
    try {
      const body = cmd === "select_uuid" ? JSON.parse(arg) :
        {session_uuid: lastTelemetry.perception_session_uuid};
      body.type = cmd === "select_uuid" ? "select_target" : "clear_target";
      body.request_id = (globalThis.crypto && crypto.randomUUID) ? crypto.randomUUID() :
        String(Date.now()) + "-" + String(Math.random());
      const response = await fetch("/api/selection", {method: "POST",
        headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)});
      const ack = await response.json();
      lastAck = {text: body.type + "  " + String(ack.reason || "NO ACK"),
        kind: ack.accepted ? "" : "bad"};
    } catch (e) {
      lastAck = {text: "SELECTION NOT ACKNOWLEDGED", kind: "bad"};
    }
    pendingAck = null;
    pendingConfirm = null;
    renderDrawer();
    return;
  }
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

// The pad's DOM stays in place while telemetry updates, preserving pointer capture.
// Requests are serialized so a released pointer cannot be followed by a late start.
let jogActive = false, jogTimer = null, jogBusy = false;
let jogRequests = Promise.resolve();
function jogRequest(command, arg) {
  jogRequests = jogRequests.then(() => sendCommand(command, arg)).catch(() => {});
  return jogRequests;
}
function stopPadJog() {
  if (!jogActive) return;
  jogActive = false;
  clearInterval(jogTimer);
  document.querySelectorAll("#manual-pad button").forEach(b => b.classList.remove("pressed"));
  jogRequest("manual_jog_stop", "");
}
function padReady(t) {
  return t && t.operating_mode === "MANUAL" && t.phase === "hold" &&
    !t.telemetry_stale && transportOk && Date.now() - lastTelemetryAt < 300 &&
    (t.safety_action === "ALLOW" || t.safety_action === "DERATE");
}
function renderManualPad(t) {
  const pad = $("manual-pad");
  if (!pad) return;
  const hidden = !(t && t.operating_mode === "MANUAL" && t.phase === "hold");
  if (pad.hidden !== hidden) { pad.hidden = hidden; renderDock(); }
  const enabled = padReady(t);
  $("manual-mode").setAttribute("aria-pressed", String(!!t && t.operating_mode === "MANUAL"));
  $("auto-mode").setAttribute("aria-pressed", String(!!t && t.operating_mode !== "MANUAL"));
  // Park is a scripted move, not a place to be stuck (owner, 2026-10-03): from the park pose, Auto,
  // Manual and Shutdown are each one press. controld takes pitch off the stop before anything moves.
  $("auto-mode").disabled = !(t && (t.phase === "hold" || t.phase === "parked") && t.soft_limits_valid &&
                              !t.telemetry_stale);
  pad.querySelectorAll("button[data-jog]").forEach(b => { b.disabled = !enabled; });
  // Releasing an arrow is the stop. The centre is the pace: the camera's, or outlined when the
  // operator pinned it.
  const pace = padPace() === "precise" ? "FINE" : "COARSE";
  if ($("pad-pace").textContent !== pace) $("pad-pace").textContent = pace;
  $("pad-pace").setAttribute("aria-pressed", String(!!padPaceOverride));
  $("pad-pace").title = padPaceOverride ? "Pace pinned; tap to follow the camera" : "Pace follows the camera; tap to pin the other";
  if (!enabled) stopPadJog();
  if (drawerOpen === "MANUAL" && pad.hidden) { drawerOpen = null; renderDrawer(); }
}
// The pad's pace: null follows the main camera; "coarse" | "precise" is the operator's pin, which a
// camera swap clears (the new view brings its own pace back).
let padPaceOverride = null, padPaceRole = null;
function padPace() {
  if (padPaceRole !== panes.main.role) { padPaceRole = panes.main.role; padPaceOverride = null; }
  return padPaceOverride || (panes.main.role === "detail" ? "precise" : "coarse");
}
const manualPad = $("manual-pad");
manualPad.querySelectorAll("button[data-direction]").forEach(b => {
  b.dataset.jog = otaJogForArrow(b.dataset.direction);
});
manualPad.addEventListener("pointerdown", (e) => {
  const b = e.target.closest("button[data-jog]");
  if (!b || b.disabled || !padReady(lastTelemetry) || jogActive || (e.pointerType === "mouse" && e.button !== 0)) return;
  e.preventDefault();
  b.setPointerCapture(e.pointerId);
  jogActive = true;
  b.classList.add("pressed");
  jogBusy = true;
  // Owner, 2026-10-03: AUTO_ROAM's paces. "view" lets controld follow the camera on the main display
  // (coarse on wide, fine on detail); the centre button pins one of them instead.
  padPace();
  jogRequest("manual_jog_start", b.dataset.jog + ":" + (padPaceOverride || "view"))
    .finally(() => { jogBusy = false; });
  jogTimer = setInterval(() => {
    if (!padReady(lastTelemetry)) { stopPadJog(); return; }
    if (!jogActive || jogBusy) return;
    jogBusy = true;
    jogRequest("manual_jog_keepalive", "").finally(() => { jogBusy = false; });
  }, 75);
});
["pointerup", "pointercancel", "lostpointercapture"].forEach(event => manualPad.addEventListener(event, stopPadJog));
window.addEventListener("blur", stopPadJog);
document.addEventListener("visibilitychange", () => { if (document.hidden) stopPadJog(); });
$("pad-pace").addEventListener("click", () => {
  const camera = panes.main.role === "detail" ? "precise" : "coarse";
  const next = padPace() === "coarse" ? "precise" : "coarse";
  padPaceOverride = next === camera ? null : next;   // back to the camera's pace un-pins it
  renderManualPad(lastTelemetry);
});
// Manual / Hold is STOP MOTION everywhere except on the park pose, where stopping is already done and
// the press means "manual": select MANUAL, which brings pitch off the rest stop to the ready pose (as
// after homing) and puts the DPAD back.
$("manual-mode").addEventListener("click", () => {
  stopPadJog();
  if (lastTelemetry && lastTelemetry.phase === "parked") sendCommand("set_mode", "MANUAL");
  else sendCommand("stop_motion", "");
});
$("auto-mode").addEventListener("click", () => { stopPadJog(); sendCommand("set_mode", "AUTO_ROAM"); });
setInterval(() => renderManualPad(lastTelemetry), 100);

dock.addEventListener("click", (e) => {
  const b = e.target && e.target.closest ? e.target.closest("button[data-key]") : null;
  if (b) setDrawer(b.getAttribute("data-key"));
});

drawer.addEventListener("click", (e) => {
  const ui = e.target && e.target.closest ? e.target.closest("button[data-ui]") : null;
  if (ui) { setStats(!statsOn); return; }
  const b = e.target && e.target.closest ? e.target.closest("button[data-cmd]") : null;
  if (!b || b.disabled) return;
  const cmd = b.getAttribute("data-cmd");
  if (!cmd) return;
  if (b.getAttribute("data-kind") === "danger") {
    // Two presses for the actions that move the turret somewhere it was not just asked to go. The label
    // changes and the row says PRESS AGAIN, so the waiting state is on screen, not in someone's memory.
    // Match a stable command, since the displayed label changes to CONFIRM …
    // after the first press. Comparing that label made confirmation impossible.
    if (pendingConfirm !== cmd) { pendingConfirm = cmd; renderDrawer(); return; }
  }
  sendCommand(cmd, b.getAttribute("data-arg") || "");
});

renderDock();
$("status-fold").addEventListener("click", () => {
  statusExpanded = !statusExpanded;
  hudSetPref("ota.hud.status.expanded", statusExpanded);
  renderStatusFold();
});
$("stats").addEventListener("click", (e) => {
  if (e.target && e.target.closest && e.target.closest("button[data-ui]")) setStats(false);
});

window.addEventListener("resize", () => { if (lastTelemetry) render(lastTelemetry); });
document.addEventListener("DOMContentLoaded", () => {
  $("video").addEventListener("loadedmetadata", () => { if (lastTelemetry) render(lastTelemetry); });
  // An <img> error is how a stopped stream shows up; both panes answer it the same way.
  $("video").addEventListener("error", () => { paneErrored(panes.main); });
  if ($("pipimg")) $("pipimg").addEventListener("error", () => { paneErrored(panes.pip); });
  startPane(panes.main);
  startPane(panes.pip);
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

  // Development handle for the reticle cant (§16): set the angle and repaint from whatever telemetry
  // the page last held. Not a control, not a setting, and not persisted -- it is for checking the
  // geometry at 0/+2/-2/+5/-5 degrees by eye.
  window.otaSetReticleCant = function (deg) {
    reticleCantDeg = Number.isFinite(Number(deg)) ? Number(deg) : 0;
    if (lastTelemetry) paint(lastTelemetry);
    return reticleCantDeg;
  };
});
"""

HUD_CSS = r"""
/* One 1px shadow, not a stack of them: the outline lives in the geometry (see the under-strokes),
   and a filter that blurs two radii is how a reticle starts looking like a neon sign. */
#g-reticle, #g-prediction { filter: drop-shadow(0 0 1px #000); }
#g-prediction .tlbl { font-weight: 500; paint-order: stroke fill; stroke: var(--hud-stroke);
  stroke-width: 2.4px; stroke-linejoin: round; }
/* Over-video text: a crisp dark outline, not a halo. `paint-order: stroke fill` puts the stroke behind
   the fill, so the glyph stays thin while the background stops fighting it; the two drop-shadows that
   were carrying this were a glow, which is what made everything look equally loud. */
#overlay text { paint-order: stroke fill; stroke: var(--hud-stroke); stroke-width: 2px;
  stroke-linejoin: round; }
/* The pane's own frame, from the token rather than a literal grey: the inline block below owns the
   pane's pinned place and its metadata colours, and this is the one line about its border. */
#pip { border: 1px solid var(--hud-line-quiet); }

:root {
  --hud-green: #95f58b;
  --hud-green-dim: rgba(149,245,139,.56);
  --hud-green-faint: rgba(149,245,139,.22);
  --hud-amber: #f2b329;
  --hud-red: #ff5d5d;
  --hud-white: #edf2eb;
  --hud-black: rgba(3,6,5,.80);
  --hud-line: rgba(190,205,190,.40);
  --hud-line-quiet: rgba(190,205,190,.22);
  /* §8's four classes, and the reason they are tokens: the green is the identity, so anything that is
     not yaw, pitch, the tracked target or the reticle has to stop borrowing it. Neutral here means
     green-grey, not a grey dashboard. */
  --hud-text: #c5d0c5;
  --hud-text-dim: #8c998c;
  /* A token nobody spends is a lie about the palette, so there is one dark fill, not three: the
     pane's metadata bar. The overlay's own darkness is --hud-black plus the under-stroke. */
  --hud-dark-soft: rgba(0,0,0,.6);
  /* The dark under-stroke every primary overlay uses instead of a glow: crisp at 2px, and it is the
     only thing that keeps a green line readable across a white curtain or a window. */
  --hud-stroke: #05070a;
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
#health { display: flex; gap: 6px; align-items: center; flex-wrap: wrap; }
#strip.folded #health .chip.ok { display: none; }
#strip.folded #health:not(:has(.chip.warn, .chip.bad)) { display: none; }
#status-fold { display: flex; align-items: center; gap: 5px; padding: 2px 6px; font: inherit;
  font-size: 10px; letter-spacing: .1em; color: var(--hud-text); background: transparent;
  border: 1px solid var(--hud-line-quiet); border-radius: 3px; cursor: pointer; }
#status-fold .dot { width: 6px; height: 6px; border-radius: 50%; }
#status-fold .arr { color: var(--hud-text-dim); }
#strip-cells { display: flex; gap: 7px; align-items: baseline; }
#stats { position: absolute; left: 1%; top: 21%; z-index:40; width: min(380px, 94vw);
  max-height: calc(79% - 60px); overflow-y: auto; padding: 6px 9px 8px; box-sizing: border-box;
  background: rgba(5,7,10,.9); border: 1px solid var(--hud-line); border-radius: 4px;
  font-size: 10px; letter-spacing: .05em; color: var(--hud-text); }
#stats[hidden] { display: none; }
#stats .stitle { display: flex; justify-content: space-between; align-items: center;
  color: var(--hud-green); letter-spacing: .14em; margin-bottom: 2px; }
#stats .stitle button { font: inherit; font-size: 14px; line-height: 1; color: var(--hud-text);
  background: transparent; border: 0; cursor: pointer; padding: 0 2px; }
#stats .ssec { color: var(--hud-text-dim); margin-top: 6px; letter-spacing: .14em; }
#stats .srow { display: flex; gap: 8px; justify-content: space-between; line-height: 1.45; }
#stats .sk { color: var(--hud-text-dim); white-space: nowrap; }
#stats .sv { text-align: right; overflow-wrap: anywhere; }
.chip { display: flex; align-items: center; gap: 5px; padding: 3px 7px; font-size: 10px;
  letter-spacing: .1em; background: var(--hud-black); border: 1px solid var(--hud-line);
  border-radius: 3px; }
.chip .dot { width: 6px; height: 6px; border-radius: 50%; }
.chip .val { opacity: .7; }
/* The dot carries the state; the words are just words (§8). A chip whose text also glows green says
   nothing that the dot has not already said, and it competes with yaw and pitch for the same green. */
.chip { border-color: var(--hud-line-quiet); }
.chip .lbl { color: var(--hud-text); }
#strip { position: absolute; left: 1%; bottom: 1.4%; z-index: 20; display: flex; gap: 7px;
  align-items: center; flex-wrap: wrap; max-width: 70%; padding: 4px 9px; font-size: 11px; letter-spacing: .08em;
  background: var(--hud-black); border: 1px solid var(--hud-line); border-radius: 3px; }
#strip .k { color: var(--hud-text-dim); margin-right: 3px; }
#strip .v { color: var(--hud-text); }        /* ordinary value: light, neutral, readable */
#strip .hot { color: var(--hud-green); }     /* the state the operator is actually steering */
#strip .warn { color: var(--hud-amber); }
#strip .fault { color: var(--hud-red); }   /* §15: red is a fault, and the selector says so */
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
#manual-pad { position:absolute; left:24px; top:calc(50% - 95px); z-index:30; display:grid;
  grid-template-columns:repeat(3,48px); gap:5px; padding:10px; border:1px solid var(--hud-line);
  border-radius:12px; background:var(--hud-black); touch-action:none; user-select:none; }
#manual-pad[hidden] { display:none; }
#manual-pad button { height:46px; border:1px solid var(--hud-green-dim); border-radius:7px;
  background:rgba(149,245,139,.08); color:var(--hud-green); font:22px var(--hud-mono); touch-action:none; }
#manual-pad button.pressed { background:var(--hud-green); color:#05070a; }
#manual-pad button:disabled { opacity:.3; }
#manual-pad #pad-pace { font-size:10px; }
#manual-pad #pad-pace[aria-pressed="true"] { border-color:var(--hud-white); color:var(--hud-white); }
#mode-controls { position:absolute; bottom:65px; left:50%; transform:translateX(-50%);
  display:flex; gap:8px; z-index:30; }
/* An inactive control is not a state readout: neutral until it is the mode you are in. */
#mode-controls button { padding:10px 16px; background:rgba(3,6,5,.9); border:1px solid var(--hud-line);
  border-radius:7px; color:var(--hud-text); font:13px var(--hud-mono); cursor:pointer; }
#mode-controls button:hover { border-color:var(--hud-text-dim); color:var(--hud-white); }
#mode-controls button[aria-pressed="true"] { border-color:var(--hud-green); color:var(--hud-green); }
#mode-controls button:disabled { opacity:.35; cursor:default; }
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
/* MENU > SETTINGS: a section rule, steppers that send the exact value they show, and the restart note. */
#drawer .dsec { color:#95f58b; font-size:9px; letter-spacing:.14em; margin:9px 0 3px; padding-top:5px;
                border-top:1px solid rgba(149,245,139,.38); }
#drawer .dnote { font-size:9px; color:rgba(237,242,235,.55); padding:2px 1px 4px; }
#drawer .drow.setting { flex-wrap:wrap; align-items:center; cursor:default; box-sizing:border-box; }
#drawer .drow.setting:hover { background:none; }
#drawer .stepper { display:flex; align-items:center; gap:4px; }
#drawer .sbtn { min-width:24px; height:22px; font:inherit; font-size:12px; color:#95f58b;
                background:rgba(149,245,139,.08); border:1px solid rgba(149,245,139,.38); border-radius:3px;
                cursor:pointer; }
#drawer .sbtn:disabled { opacity:.25; cursor:default; }
#drawer .sval { min-width:52px; text-align:center; color:#edf2eb; }
#drawer .snote { width:100%; font-size:9px; color:rgba(237,242,235,.45); }

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
  <img id="video" alt="camera">
  <!-- The PIP: the camera that is NOT on the main display, for display only (owner, 2026-10-02:
       only the main display is inferred). Always open; which camera it shows is the station's
       choice, published in the inference report, and the swap button asks the station. -->
  <style>
    /* Pinned: the same column as the mode block, below it, always open, never repositioned.
       88px clears the mode block's three lines at every window ratio tried so far -- the earlier
       version computed the pane's place from the letterboxed picture and therefore moved when the
       window or the frame geometry changed, which the owner correctly called "还乱跑". The trade is
       accepted on purpose: at extreme ratios the pane sits over a black bar instead of over the
       picture; what it must never do is cover a control. */
    /* Centred horizontally, low, sitting directly above the operating-mode buttons: #mode-controls is
       anchored at bottom:65px and is ~34px tall, so 112px puts the pane's bottom edge clear of them
       at any window width. Still constants -- no measurement at run time, which is what made the pane
       drift before -- and still under the chrome layer, so if a ratio ever gets tight the buttons win
       the overlap and the preview is the thing that gets covered. */
    #pip { position: absolute; left: 50%; bottom: 112px; width: 280px; transform: translateX(-50%);
           border: 1px solid #444; background: #000;
           /* Below the chrome layer (every control sits at z-index 20), above the picture. The owner's
              ruling of 2026-09-29 after the pane covered the D-pad: keep the pinned position and let
              the D-pad paint over the preview -- chrome wins over a preview, always, which is cheaper
              to reason about than any placement. The swap button sits at the pane's right edge, which
              the aim pad does not reach, so the pane keeps its one control clickable. */
           z-index: 15; }
    #pip img { width: 100%; display: block; }
    /* Level 3: the pane's chrome is information about a second camera, not state about the turret.
       Neutral frame, dim metadata, and the one control brightens when the pointer is on it. */
    #pip .bar { display: flex; justify-content: space-between; font-size: 10px;
                color: var(--hud-text-dim); padding: 2px 4px; background: var(--hud-dark-soft); }
    #pip button { background: none; border: 0; color: var(--hud-text-dim); cursor: pointer;
                  font-size: 10px; }
    #pip button:hover { color: var(--hud-text); }
  </style>
  <div id="pip">
    <img id="pipimg" alt="secondary preview">
    <div class="bar"><span id="piplabel">PIP</span><span id="pipfps">rate n/m</span>
      <button id="pipswap" type="button">swap</button></div>
  </div>
  <script>
  (function () {
    function el(id) { return document.getElementById(id); }

    // Rendered state, not a control: a station publishing one stream has no second tap to start, so
    // the pane says so instead of showing a frozen frame or firing requests at a role nobody owns --
    // and says so only while it is true: the stream coming back (visiond restarted) brings the pane
    // back, where the old one-way latch kept it hidden until a reload.
    window.otaPipNoteStreams = function (streams) {
      if (!Array.isArray(streams) || typeof panes === "undefined") return;
      const two = streams.length >= 2;
      if (two === panes.pip.wanted) return;
      panes.pip.wanted = two;
      el("pipimg").style.display = two ? "" : "none";
      el("piplabel").textContent = two ? "PIP " + panes.pip.role : "NO SECOND STREAM";
      el("pipfps").textContent = "";
      if (two) { panes.pip.lastAttemptMs = 0; startPane(panes.pip); }
    };

    // Swap asks the STATION to change the main display -- which is also the camera the Hailo sees
    // (owner, 2026-10-02) -- and the panes follow the published answer (otaPanesFollow), so every
    // open page shows the same thing and a reload does not undo it.
    window.otaSwapPip = async function () {
      el("pipfps").textContent = "swapping...";
      try {
        const r = await fetch("/api/camera/main", { method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ role: panes.pip.role }) });
        const j = await r.json();
        if (!j.accepted) {
          el("pipfps").textContent = "refused: " + (j.detail || j.reason || "?");
          return;
        }
        const mc = j.main_camera || {};
        window.otaPanesFollow({ main: mc.role, pip: mc.role === "wide" ? "detail" : "wide",
                                canSwap: true, generation: mc.generation, boot: mc.boot });
      } catch (e) { el("pipfps").textContent = "refused: " + e; }
    };
    el("pipswap").addEventListener("click", window.otaSwapPip);
    el("piplabel").textContent = "PIP detail";       // every boot starts with wide on the main display
  })();
  </script>
  

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
  <div id="notices"></div>
  <!-- The status bar: the health chips folded behind one summary, then the readings (owner,
       2026-10-03: the top-right chip row merged into the bottom bar). -->
  <div id="strip" class="folded"><button id="status-fold" type="button" aria-expanded="false"
    aria-label="Show system status"></button><span id="health"></span><span id="strip-cells"></span></div>
  <div id="stats" hidden role="region" aria-label="Stats for nerds"></div>

  <!-- §13 dock and §14 drawer. Outside the SVG deliberately: these are the only parts of the overlay the
       operator presses, and real elements keep focus, hover and button semantics away from a hit-test on
       painted geometry. They come last in document order, which on this page IS the z-order (§18: dock
       30, drawer 40); the z-index in the CSS states it rather than relying on it. -->
  <div id="dock" role="toolbar" aria-label="context controls"></div>
  <div id="mode-controls" role="group" aria-label="Operating mode">
    <button id="manual-mode" type="button">Manual / Hold</button>
    <button id="auto-mode" type="button">Auto</button>
  </div>
  <div id="manual-pad" hidden role="group" aria-label="Manual direction pad">
    <button data-direction="up-left" aria-label="Aim camera up and left">↖</button><button data-direction="up" aria-label="Aim camera up">↑</button><button data-direction="up-right" aria-label="Aim camera up and right">↗</button>
    <button data-direction="left" aria-label="Aim camera left">←</button><button id="pad-pace" type="button" aria-pressed="false" aria-label="Jog pace">COARSE</button><button data-direction="right" aria-label="Aim camera right">→</button>
    <button data-direction="down-left" aria-label="Aim camera down and left">↙</button><button data-direction="down" aria-label="Aim camera down">↓</button><button data-direction="down-right" aria-label="Aim camera down and right">↘</button>
  </div>
  <!-- §22 safety indication. Outside the health chips, because BRAKING and FAULT are asked to be more
       prominent than a chip and a fault to interrupt normal operation. -->
  <div id="safety" hidden role="status" aria-live="assertive"></div>
  <div id="drawer" hidden role="dialog" aria-modal="false"></div>
</div>
<script>""" + HUD_JS + """</script>
</body>
</html>
"""
