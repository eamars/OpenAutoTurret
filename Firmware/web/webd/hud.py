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
    const len = tk.endpoint ? 13 : (tk.coarse ? 10 : 5);
    const col = tk.coarse ? base : fine;
    parts.push('<line ' + (w ? 'x1="' + tk.pos + '" y1="' + t.y + '" x2="' + tk.pos + '" y2="' + (t.y + len)
                           : 'x1="' + t.x + '" y1="' + tk.pos + '" x2="' + (t.x - len) + '" y2="' + tk.pos) +
               '" stroke="' + col + '" stroke-width="1"/>');
    if (tk.label) {
      parts.push('<text class="tlbl" ' +
        (w ? 'x="' + tk.pos + '" y="' + (t.y - 7) + '" text-anchor="middle"'
           : 'x="' + (t.x + 8) + '" y="' + (tk.pos + 4) + '" text-anchor="start"') +
        ' fill="' + lbl + '">' + tk.label + '</text>');
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
  const layers = { cand: "", sel: "", reticle: "", tape: "" };

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

  // §5 + §6 travel tapes. Placement is taken from the revision's own numbers rather than from
  // judgement: the yaw tape sits in the upper 10-15% band (12.5%) across the middle 55-60% of the
  // width (57.5%), the pitch tape in the middle 40-45% of the height (42.5%) near the right edge.
  // Those figures are asserted in the test, because a claim this specific is only worth writing if
  // something checks it, and "visually centered" is how a tape ends up wherever the last edit left
  // it. Both tapes show LOGICAL JOINT TRAVEL (§5.3) from the encoders, never a compass heading, and
  // no cardinal letters appear anywhere.
  const yawTape = hudTravelTape({
    horizontal: true, x: vw * (1 - 0.575) / 2, y: vh * 0.125, length: vw * 0.575,
    minDeg: deg(t.q_soft_min_yaw_rad), maxDeg: deg(t.q_soft_max_yaw_rad),
    valueDeg: deg(t.q_yaw_rad), valid: t.soft_limits_valid === true
  });
  const pitchLen = vh * 0.425;
  const pitchTape = hudTravelTape({
    horizontal: false, x: vw - Math.max(78.0, vw * 0.055), y: vh / 2 - pitchLen / 2,
    length: pitchLen, minDeg: deg(t.q_soft_min_pitch_rad), maxDeg: deg(t.q_soft_max_pitch_rad),
    valueDeg: deg(t.q_pitch_rad), valid: t.soft_limits_valid === true
  });
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
  $("g-tapes").innerHTML = layers.tape;

  // target_aim_x/y_norm (the point inside the target the controller is driving onto the axis) is
  // deliberately NOT drawn. v3.2 mentions an aiming marker exactly once - §7's open centre, which
  // IS the optical axis - and inventing a second marker would put an unspecced symbol on the
  // operator's screen. It is also unnecessary: the controller aims the head AT the axis, so what
  // the operator sees is the reticle sitting on the head, which is the acceptance rule as stated.
  // The field stays in telemetry for measurement and for the DIAG drawer.

  // §4.1 mode block: three lines, first line strongest.
  const mode = String(t.operating_mode || "--").replace("_", " ");
  $("mode-block").innerHTML =
    '<div class="m1">' + mode + '</div>' +
    '<div class="m2">' + (String(t.mode_phase || "--")) + '</div>' +
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
  const safe = String(t.safety_action || "").toUpperCase();
  hs.appendChild(chip("SAFETY", safe && safe !== "NONE" ? "amber" : "ok", safe && safe !== "NONE" ? safe : "ALLOW"));

  // §12 bottom strip. FPS is not in the snapshot yet, so it reads "--" rather than a guess.
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
}
/* §16: narrow monospaced sensor display, not a proportional UI font. */
html, body { margin: 0; height: 100%; background: #05070a; overflow: hidden;
  font-family: "IBM Plex Mono", "Roboto Mono", "SFMono-Regular", Consolas, monospace;
  color: var(--hud-white); }
#viewport { position: fixed; inset: 0; }
/* §18 layering, exactly as specified. */
#video { position: absolute; inset: 0; width: 100%; height: 100%; object-fit: contain;
  background: #000; z-index: 0; }
#overlay { position: absolute; inset: 0; z-index: 10; pointer-events: none; }
#mode-block { position: absolute; left: 1%; top: 1.2%; z-index: 20; text-shadow: 0 0 6px rgba(0,0,0,.9); }
#mode-block .m1 { color: var(--hud-green); font-size: 20px; letter-spacing: .12em; }
#mode-block .m2 { color: var(--hud-white); font-size: 13px; letter-spacing: .12em; opacity: .86; }
#mode-block .m3 { color: var(--hud-white); font-size: 12px; letter-spacing: .12em; opacity: .62; }
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
text.tlbl { font-size: 11px; letter-spacing: .04em; font-family: inherit; }   /* tape labels */
text.tval { font-size: 12px; letter-spacing: .06em; font-family: inherit; }   /* value boxes */
text.lbl { font-size: 11px; letter-spacing: .08em; font-family: inherit; }
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
    <g id="g-reticle"></g>
    <g id="g-tapes"></g>
  </svg>

  <div id="mode-block"></div>
  <div id="health"></div>
  <div id="notices"></div>
  <div id="strip"></div>
</div>
<script>""" + HUD_JS + """</script>
</body>
</html>
"""
