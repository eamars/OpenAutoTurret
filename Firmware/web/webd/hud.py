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
let staleNow = false;        // last computed §25 verdict, so a transition can repaint

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
  const layers = { cand: "", sel: "", reticle: "", };

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

  $("g-candidates").innerHTML = layers.cand;
  $("g-selected").innerHTML = layers.sel;
  $("g-reticle").innerHTML = layers.reticle;

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
  if (vp) vp.classList.toggle("stale", verdict);
  staleNow = verdict;
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
  // only mechanism that does not assume someone will eventually tell us.
  setInterval(() => { if (lastTelemetry) render(lastTelemetry); }, 250);
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
