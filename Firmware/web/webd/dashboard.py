"""Operator dashboard (architecture §42.1, §42.2, §42.3).

A single self-contained HTML page (inline CSS + JS, no build step) with the
§42.1 status panels and the §42.2 developer controls. It connects to the
``/ws`` WebSocket for live telemetry and POSTs to ``/api/command`` for
developer commands.

SAFETY: the page only DISPLAYS state and SUBMITS high-level commands. Every
command is validated by controld before it runs; the page has no motor/CAN
authority of its own. The video preview (§42.3) is a real IMX500 MJPEG feed on a
separate low-priority path (its own HTTP stream, never the control IPC); the on/off switch
opens/releases the camera, so the feed costs no CPU while off.
"""
from __future__ import annotations

DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__ — OpenAutoTurret</title>
<style>
  :root { --bg:#0e1116; --panel:#161b22; --border:#2b3240; --fg:#e6edf3;
          --dim:#8b949e; --ok:#3fb950; --warn:#d29922; --err:#f85149; --accent:#58a6ff; }
  * { box-sizing: border-box; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font:14px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; }
  header { display:flex; align-items:center; gap:12px; padding:10px 16px;
           border-bottom:1px solid var(--border); background:var(--panel); }
  header h1 { font-size:16px; margin:0; font-weight:600; }
  .dot { width:10px; height:10px; border-radius:50%; background:var(--err); }
  .dot.on { background:var(--ok); }
  #grid { display:grid; gap:12px; padding:12px;
          grid-template-columns:repeat(auto-fit, minmax(280px, 1fr)); }
  .panel { background:var(--panel); border:1px solid var(--border);
           border-radius:8px; padding:12px; }
  .panel h2 { font-size:12px; text-transform:uppercase; letter-spacing:.06em;
              color:var(--dim); margin:0 0 8px; }
  .row { display:flex; justify-content:space-between; padding:2px 0;
         font-variant-numeric:tabular-nums; }
  .row .k { color:var(--dim); }
  .badge { display:inline-block; padding:1px 8px; border-radius:10px;
           font-size:12px; font-weight:600; background:#222b38; }
  .badge.ok { background:rgba(63,185,80,.18); color:var(--ok); }
  .badge.warn { background:rgba(210,153,34,.18); color:var(--warn); }
  .badge.err { background:rgba(248,81,73,.18); color:var(--err); }
  .badge.info { background:rgba(88,166,255,.18); color:var(--accent); }
  #controls { grid-column:1/-1; display:flex; flex-wrap:wrap; gap:8px; }
  button { background:#21262d; color:var(--fg); border:1px solid var(--border);
           border-radius:6px; padding:7px 12px; cursor:pointer; font-size:13px; }
  button:hover { border-color:var(--accent); }
  button.primary { background:var(--accent); color:#04121f; border-color:var(--accent); }
  button.danger { background:transparent; color:var(--err); border-color:var(--err); }
  button:disabled { opacity:.45; cursor:not-allowed; }
  input { background:#0d1117; border:1px solid var(--border); color:var(--fg);
          border-radius:6px; padding:6px 8px; width:90px; }
  #log { grid-column:1/-1; font-family:ui-monospace, monospace; font-size:12px;
         max-height:160px; overflow:auto; white-space:pre-wrap; }
  .muted { color:var(--dim); }
  .video-ph { display:flex; align-items:center; justify-content:center; height:140px;
              border:1px dashed var(--border); border-radius:6px; color:var(--dim); }
  .video-panel { grid-column:1/-1; }
  .video-panel h2 { display:flex; justify-content:space-between; align-items:center; gap:12px; }
  .video-status { font-size:12px; margin:0 0 8px; min-height:1em; }
  .video-stage img { width:100%; display:block; background:#000; border-radius:6px; }
  .switch { position:relative; display:inline-block; width:44px; height:24px; flex:0 0 auto; }
  .switch input { opacity:0; width:0; height:0; }
  .slider { position:absolute; inset:0; background:#222b38; border:1px solid var(--border);
              border-radius:24px; transition:.15s; cursor:pointer; }
  .slider::before { content:""; position:absolute; width:18px; height:18px; left:3px; top:2px;
              background:var(--dim); border-radius:50%; transition:.15s; }
  .switch input:checked + .slider { background:rgba(63,185,80,.25); border-color:var(--ok); }
  .switch input:checked + .slider::before { transform:translateX(20px); background:var(--ok); }
</style>
</head>
<body>
<header>
  <span class="dot" id="conn"></span>
  <h1>__TITLE__ · OpenAutoTurret</h1>
  <span class="muted" id="state-word">connecting…</span>
</header>
<div id="grid">

  <section class="panel" id="p-system">
    <h2>System</h2>
    <div class="row"><span class="k">Loop phase</span><span class="badge" id="phase">—</span></div>
    <div class="row"><span class="k">Track state</span><span class="badge" id="track-state">—</span></div>
    <div class="row"><span class="k">Safety action</span><span class="badge" id="safety">—</span></div>
    <div class="row"><span class="k">Tracking active</span><span id="tracking-active">—</span></div>
    <div class="row"><span class="k">Feedback age</span><span id="fb-age">—</span></div>
    <div class="row"><span class="k">Control cycle</span><span id="cycle">—</span></div>
    <div class="row"><span class="k">At ready pose</span><span class="badge" id="at-ready">—</span></div>
    <div class="row"><span class="k">Fault</span><span id="fault">—</span></div>
  </section>

  <section class="panel" id="p-vision">
    <h2>Vision</h2>
    <div class="row"><span class="k">Publisher</span><span class="badge" id="vis-conn">—</span></div>
    <div class="row"><span class="k">Measurements</span><span id="vis-frames">—</span></div>
    <div class="row"><span class="k">Measurement age</span><span id="vis-age">—</span></div>
    <div class="row"><span class="k">Target confidence</span><span id="conf">—</span></div>
    <div class="row"><span class="k">Target az (world)</span><span id="taz">—</span></div>
    <div class="row"><span class="k">Target el (world)</span><span id="tel">—</span></div>
  </section>

  <section class="panel" id="p-yawpitch">
    <h2>Yaw · Pitch</h2>
    <div class="row"><span class="k">q yaw</span><span id="qy">—</span></div>
    <div class="row"><span class="k">q ref yaw</span><span id="qry">—</span></div>
    <div class="row"><span class="k">v yaw</span><span id="vy">—</span></div>
    <div class="row"><span class="k">q pitch</span><span id="qp">—</span></div>
    <div class="row"><span class="k">q ref pitch</span><span id="qrp">—</span></div>
    <div class="row"><span class="k">v pitch</span><span id="vp">—</span></div>
  </section>

  <section class="panel" id="p-cal">
    <h2>Calibration</h2>
    <div class="row"><span class="k">Installation</span><span class="badge" id="calib">—</span></div>
    <div class="row"><span class="k">Source</span><span id="calib-src">—</span></div>
    <div class="row"><span class="k">Base roll</span><span id="br">—</span></div>
    <div class="row"><span class="k">Base pitch</span><span id="bp">—</span></div>
    <div class="row"><span class="k">Base yaw</span><span id="by">—</span></div>
  </section>

    <section class="panel video-panel" id="p-video">
    <h2><span>Video — separate low-priority path (low priority)</span>
      <label class="switch" title="Open / close the camera feed">
        <input type="checkbox" id="video-toggle">
        <span class="slider"></span>
      </label>
    </h2>
    <div class="video-status muted" id="video-status">checking…</div>
    <div class="video-stage" id="video-stage">
      <img id="video-img" alt="camera feed" style="display:none">
      <div class="video-ph" id="video-ph">Camera off — flip the switch on to open the IMX500 feed (costs no CPU while off).</div>
    </div>
  </section>


  <section class="panel" id="p-can">
    <h2>CAN / Actuation</h2>
    <div class="row"><span class="k">Effort yaw (Nm)</span><span id="ey">—</span></div>
    <div class="row"><span class="k">Effort pitch (Nm)</span><span id="ep">—</span></div>
    <div class="muted">read-only; motors are driven solely by controld</div>
  </section>

  <section class="panel" id="p-can-link">
    <h2>CAN link (§55 / §54.4)</h2>
    <div class="row"><span class="k">Transport</span><span id="can-kind">—</span></div>
    <div class="row"><span class="k">Bus state</span><span id="can-state" class="badge">—</span></div>
    <div class="row"><span class="k">RX / RX error frames</span><span id="can-rx">—</span></div>
    <div class="row"><span class="k">TX / TX failed</span><span id="can-tx">—</span></div>
    <div class="row"><span class="k">Last RX age</span><span id="can-age">—</span></div>
    <div class="muted" id="can-note">waiting for telemetry…</div>
  </section>

  <section class="panel" id="p-payload">
    <h2>Payload (§28.5 / §31)</h2>
    <div class="row"><span class="k">Profile</span><span id="pp-name">—</span></div>
    <div class="row"><span class="k">Status</span><span id="pp-status" class="badge">—</span></div>
    <div class="row"><span class="k">Derated</span><span id="pp-derated">no</span></div>
    <div class="row"><span class="k">Check running</span><span id="pp-check">no</span></div>
    <div class="row"><span class="k">Stored profiles</span>
      <select id="profile-select" style="max-width:160px"></select></div>
    <div class="row"><span class="k"></span>
      <button data-cmd="select_payload_profile">Select profile</button></div>
    <div class="muted" id="profile-note">loading profiles…</div>
    <div class="muted">a mismatch derates motion limits until re-verified</div>
    <div class="muted">selecting a profile applies its motion CAPS at once; it
      is only commissioned as verified payload data by
      "Start payload verification" (§31.3)</div>
  </section>

  <section class="panel" id="p-mode" style="grid-column:1/-1">
    <h2>Operating mode (§45) — exactly one mode owns motion</h2>
    <div id="mode-controls">
      <button data-cmd="set_mode" data-mode="MANUAL">MANUAL</button>
      <button data-cmd="set_mode" data-mode="AUTO_TRACK">AUTO TRACK</button>
      <button data-cmd="set_mode" data-mode="AUTO_ROAM">AUTO ROAM</button>
      <button class="danger" data-cmd="stop_motion">STOP MOTION</button>
      <span id="mode" class="badge">—</span>
      <span id="mode-phase" class="badge">—</span>
      <span id="sup-state" class="badge">—</span>
    </div>
    <div><span id="mode-unsupported" class="muted"></span></div>
    <div><span id="intent">intent: —</span></div>
    <div><span id="ack" class="muted">last command: none since controld started</span></div>
    <div class="muted">STOP MOTION cancels the active intent and lands in
      MANUAL / HOLD. It does not disable the motors, power down, or park, and it is
      accepted in every state including fault (§27). A mode change the station is
      not entitled to is refused with the reason, shown above (§52). AUTO TRACK does
      not go looking for a target when it loses one — that is AUTO ROAM (§111.5).</div>
  </section>

  <section class="panel" id="p-controls" style="grid-column:1/-1">
    <h2>Developer controls (§42.2) — every request is validated by controld</h2>
    <div id="controls">
      <button class="primary" data-cmd="hold">Hold</button>
      <button data-cmd="start_tracking">Start tracking</button>
      <button data-cmd="stop_tracking">Stop tracking</button>
      <button data-cmd="enable_search">Enable search</button>
      <button data-cmd="disable_search">Disable search</button>
      <span>target
        <input id="sel-target" type="number" min="0" max="15" value="0" style="width:56px">
      </span>
      <button data-cmd="select_target">Select target</button>
      <button data-cmd="start_homing">Start homing</button>
      <button data-cmd="start_installation_calibration">Start visual calibration</button>
      <button data-cmd="start_payload_verification">Start payload verification</button>
      <span>test motion (rad)
        <input id="test-motion" type="number" step="0.05" value="0.3" style="width:80px">
      </span>
      <button data-cmd="run_test_motion">Run test motion</button>
      <button class="danger" data-cmd="request_park">Safe park</button>
      <button class="danger" data-cmd="request_shutdown">Safe shutdown</button>
    </div>
  </section>

  <section class="panel" style="grid-column:1/-1">
    <h2>Command log</h2>
    <div id="log"></div>
  </section>
</div>

<script>
const $ = (id) => document.getElementById(id);
const rad = (x) => (isFinite(x) ? (x * 57.29577951308232).toFixed(2) + "°" : "—");
const num = (x, d=4) => (isFinite(x) ? Number(x).toFixed(d) : "—");

function badge(el, text, kind) {
  el.textContent = text;
  el.className = "badge" + (kind ? " " + kind : "");
}
function safetyKind(s) {
  return s === "ALLOW" ? "ok" : s === "DERATE" ? "warn" : s === "HOLD" ? "info"
       : (s === "BRAKE" ? "warn" : "err");
}
function trackKind(s) {
  return (s === "tracking" || s === "coasting") ? "ok"
       : s === "search" ? "info" : s === "target_lost" ? "warn" : "info";
}
function payloadKind(s) {
  return s === "ok" ? "ok" : s === "mismatch" ? "warn"
       : s === "error" ? "err" : "info";
}

// Connection supervision. This page gets watched from across a room, usually in
// a background tab, and it is the operator's only view of the turret — so it has
// to survive a daemon restart by itself. The original reconnected only from
// ws.onclose through a single setTimeout, and websockets do drop (webd
// restarting, a NAT evicting the mapping, a laptop lid). A hidden tab's timers
// are throttled or frozen outright, so the retry never fired and the page sat on
// "disconnected" while the backend was perfectly healthy — indistinguishable
// from a dead station. The watchdog below is the difference between a page that
// recovers and a page somebody has to go and reload.
let ws = null;
let wsTry = 0;            // consecutive failed attempts, shown in the status line
let wsCloseCode = null;   // last close code, so one screenshot is diagnosable

function connNote() {
  $("conn").classList.remove("on");
  const code = wsCloseCode === null ? "" : " code " + wsCloseCode;
  $("state-word").textContent =
    "disconnected" + code + " — retry #" + wsTry;
}

function connect() {
  // Idempotent on purpose: the watchdog, visibilitychange and onclose all call
  // this, and two sockets delivering the same telemetry would double every
  // counter on screen while looking completely normal.
  if (ws && (ws.readyState === WebSocket.OPEN ||
             ws.readyState === WebSocket.CONNECTING)) return;
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  try { ws = new WebSocket(proto + location.host + "/ws"); }
  catch (e) { wsTry += 1; connNote(); setTimeout(connect, 1500); return; }
  ws.onopen = () => {
    const wasDown = wsTry > 0;
    wsTry = 0; wsCloseCode = null;
    $("conn").classList.add("on"); $("state-word").textContent = "connected";
    // Coming back is more than re-subscribing: the MJPEG request the <img> holds
    // died with the old socket, and the profile list may have changed while we
    // were away. Re-sync both, rather than keeping a frozen frame on screen and
    // calling it live video.
    if (wasDown) { refreshVideoState(); refreshProfiles(); }
  };
  ws.onclose = (ev) => {
    wsCloseCode = (ev && ev.code) ? ev.code : null;
    ws = null; wsTry += 1; connNote();
    setTimeout(connect, 1500);   // fast first retry; the watchdog is the backstop
  };
  // ws.onerror needs no handler of its own: it is always followed by onclose.
  ws.onmessage = (e) => {
    let t; try { t = JSON.parse(e.data); } catch { return; }
    if (t.type !== "telemetry") return;
    render(t);
  };
}
setInterval(() => {
  if (!ws || ws.readyState !== WebSocket.OPEN) connect();
}, 2000);
document.addEventListener("visibilitychange", () => {
  // Reveal the tab and it reconnects now, not up to two seconds later.
  if (!document.hidden) connect();
});
let LAST_T = null;              // last telemetry frame, for the profile marker
// CAN controller error state (CanIfState) -> [label, badge class]. An array of
// pairs, deliberately: a {-1: ...} object literal is a JavaScript SYNTAX ERROR
// (a leading "-" is not a property name), and one bad line in this <script>
// kills every statement in it — including connect(). The page still renders from
// HTML, so it looks like the backend is down when it is the page that died. That
// is what test_dashboard_js_parses.py now guards.
const CAN_STATE = [[-1, ["unknown", "info"]], [0, ["error-active", "ok"]],
                   [1, ["error-warning", "warn"]], [2, ["error-passive", "warn"]],
                   [3, ["BUS-OFF", "err"]], [4, ["stopped", "err"]],
                   [5, ["sleeping", "info"]]];
function canStateName(code) {
  const hit = CAN_STATE.find((p) => p[0] === code);
  return hit ? hit[1] : ["state " + code, "info"];
}

function renderCan(t) {
  // A simulated backend has no bus to report. Showing rx=0 tx=0 as if it were a
  // live-but-quiet bus would be the most misleading possible default, so the
  // whole panel changes tone when can_available is false.
  if (!t.can_available) {
    $("can-kind").textContent = "no CAN link";
    badge($("can-state"), "not applicable", "info");
    $("can-rx").textContent = "—"; $("can-tx").textContent = "—";
    $("can-age").textContent = "—";
    $("can-note").textContent = "can_available=false: this daemon has no CAN "
      + "transport (simulated backend). Zeros here are absence, not health.";
    return;
  }
  const st = canStateName(t.can_state);
  $("can-kind").textContent = (t.can_kind || "?") + " · " + (t.can_device || "?")
    + (t.can_up ? "" : " (down)");
  // "unknown" is not one state, it is two very different situations, and on
  // THIS station it is permanent: the yousee USB-CAN adapter exposes no
  // controller error state at all (yousee_transport.hpp), so the badge would sit
  // on "unknown" forever while the operator looked for a fault that isn't there.
  // SocketCAN (the MCP2515 HAT) reports the real netlink state. Distinguish the
  // two and point at the signals that DO exist on yousee.
  if (t.can_state === -1 && t.can_kind === "yousee") {
    badge($("can-state"), "not exposed by adapter", "info");
    $("can-note").textContent = "the yousee adapter exposes no controller error "
      + "state (§54.4 error-active/passive/bus-off are only readable on the "
      + "MCP2515/can0 path). Here the CAN-health signals are RX error frames "
      + "(codec resyncs = wire corruption) and the RX age; feedback staleness is "
      + "supervised regardless (§34).";
  } else {
    badge($("can-state"), st[0], st[1]);
    $("can-note").textContent = "error frames climbing while the bus stays "
      + "error-active points at the PHY/cabling; bus-off means the controller "
      + "stopped talking (§54.4)";
  }
  $("can-rx").textContent = t.can_rx_frames + " / " + t.can_rx_error_frames;
  $("can-tx").textContent = t.can_tx_frames + " / " + t.can_tx_failed;
  $("can-age").textContent = t.can_last_rx_age_ms < 0 ? "none received"
                                                      : t.can_last_rx_age_ms + " ms";
}

function render(t) {
  LAST_T = t;
  markActiveProfile();
  badge($("phase"), t.phase || "—", t.phase === "fault" ? "err" :
        (t.phase === "hold" || t.phase === "parked" ? "ok" : "warn"));
  badge($("track-state"), t.track_state, trackKind(t.track_state));
  badge($("safety"), t.safety_action, safetyKind(t.safety_action));
  $("tracking-active").textContent = t.tracking_active ? "yes" : "no";
  $("fb-age").textContent = num(t.feedback_age_ms, 1) + " ms";
  $("cycle").textContent = num(t.control_cycle_us, 1) + " µs";
  badge($("at-ready"), t.at_ready ? "homed + ready" : "not ready",
        t.at_ready ? "ok" : "warn");
  $("fault").textContent = t.fault || "—";
  badge($("vis-conn"), t.vision_connected ? "visiond connected" : "no publisher",
        t.vision_connected ? "ok" : "warn");
  $("vis-frames").textContent = (t.vision_frames || 0) +
    (t.vision_dropped ? " (" + t.vision_dropped + " dropped)" : "");
  $("vis-age").textContent = t.vision_measurement_age_ms < 0
    ? "never" : num(t.vision_measurement_age_ms, 0) + " ms";
  $("conf").textContent = num(t.target_confidence, 3);
  $("taz").textContent = rad(t.target_az_world_rad);
  $("tel").textContent = rad(t.target_el_world_rad);
  $("qy").textContent = num(t.q_yaw_rad);
  $("qry").textContent = num(t.q_ref_yaw_rad);
  $("vy").textContent = num(t.v_yaw_rad_s, 3);
  $("qp").textContent = num(t.q_pitch_rad);
  $("qrp").textContent = num(t.q_ref_pitch_rad);
  $("vp").textContent = num(t.v_pitch_rad_s, 3);
  const calib = t.installation_calibrated ? "calibrated" : "NOT calibrated";
  badge($("calib"), calib, t.installation_calibrated ? "ok" : "warn");
  $("calib-src").textContent = t.installation_source;
  $("br").textContent = rad(t.base_roll_rad);
  $("bp").textContent = rad(t.base_pitch_rad);
  $("by").textContent = rad(t.base_yaw_rad);
  $("ey").textContent = num(t.effort_yaw, 2);
  $("ep").textContent = num(t.effort_pitch, 2);
  // v3 §50: the three things the operator asks in order — who is driving, what
  // it is doing about it, and what the last button press actually accomplished.
  // A dashboard newer than the daemon is the normal state during an upgrade, and
  // it must not pretend otherwise. An old controld has no operating_mode in its
  // telemetry and no set_mode in its validator, so the buttons would sit there
  // looking pressable and answer "unknown command" — which is a true answer to a
  // question the page should not have been asking. Say what is missing instead.
  const v3 = !!t.operating_mode;
  const mode = t.operating_mode || "not reported";
  document.querySelectorAll("#mode-controls button[data-mode]").forEach((btn) => {
    btn.disabled = !v3;
  });
  const note = $("mode-unsupported");
  if (note) {
    note.textContent = v3 ? "" :
      "controld is not reporting an operating mode: this build predates v3 (§43). " +
      "Update and restart controld before using the mode buttons.";
    note.className = v3 ? "muted" : "err";
  }
  badge($("mode"), mode, mode === "MANUAL" ? "ok" : "warn");
  badge($("mode-phase"), t.mode_phase || "—", "info");
  badge($("sup-state"), t.supervisory_state || "—",
        t.supervisory_state === "READY" ? "ok" : "warn");
  markActiveMode(mode);
  $("intent").textContent = "intent: " + (t.intent_source || "none") + " / " +
    (t.intent_type || "hold") +
    (t.intent_reason ? " — " + t.intent_reason : "") +
    " · scale " + num(t.intent_velocity_scale, 2);
  renderAck(t);
  renderCan(t);
  $("pp-name").textContent = t.payload_profile_name || "—";
  badge($("pp-status"), t.payload_profile_status || "no_profile",
        payloadKind(t.payload_profile_status));
  $("pp-derated").textContent = t.payload_derated ? "yes" : "no";
  $("pp-check").textContent = t.payload_check_active ? "yes" : "no";
}

function markActiveMode(mode) {
  document.querySelectorAll("#mode-controls button[data-mode]").forEach((btn) => {
    const on = btn.dataset.mode === mode;
    btn.classList.toggle("primary", on);
    btn.setAttribute("aria-pressed", on ? "true" : "false");
  });
}

function renderAck(t) {
  // §52. The ack is read from telemetry, not from the button's HTTP reply:
  // controld executes commands on the control thread, so the answer is not
  // known yet when the request is accepted for execution. -1 means no command
  // has been executed since the daemon started, which is not the same claim as
  // "the last one succeeded" and must not be displayed as one.
  const el = $("ack");
  if (!el) return;
  if (t.cmd_ack_accepted === -1 || !t.cmd_ack_command) {
    el.textContent = "last command: none since controld started";
    el.className = "muted";
    return;
  }
  const ok = t.cmd_ack_accepted === 1;
  el.textContent = "last command: " + t.cmd_ack_command + " -> " +
    (ok ? "ACCEPTED" : "REJECTED") + " — " + (t.cmd_ack_reason || "") +
    " (state " + (t.cmd_ack_controller_state || "?") +
    ", safety " + (t.cmd_ack_safety_state || "?") + ")";
  el.className = ok ? "muted" : "err";
}

function logline(msg, kind) {
  const el = $("log");
  const time = new Date().toLocaleTimeString();
  el.textContent = `[${time}] ${msg}\n` + el.textContent;
  if (el.children.length > 200) el.textContent = el.textContent.split("\n").slice(0, 200).join("\n");
}

async function sendCommand(cmd, arg, btn) {
  if (btn) btn.disabled = true;
  try {
    const r = await fetch("/api/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ command: cmd, arg: arg || "" }),
    });
    const j = await r.json();
    if (j.ok) {
      logline(`${cmd}(${arg || ""}) -> OK`, "ok");
      if (cmd === "select_payload_profile") refreshProfiles();   // active moves
    } else {
      // The daemon's reason is the useful part: "no payload profile named …"
      // or "not at the ready pose yet" belong on the operator's screen, not in
      // a log file they would have to go and open.
      logline(`${cmd}(${arg || ""}) -> REJECTED: ${j.error}`, "err");
    }
    return j;
  } catch (e) {
    logline(`${cmd} -> ERROR: ${e}`, "err");
    return { ok: false, error: String(e) };
  } finally {
    if (btn) btn.disabled = false;
  }
}

function argFor(cmd, btn) {
  // Two buttons share set_mode and differ only by argument, so the argument
  // comes from the button rather than from a control elsewhere on the page.
  if (cmd === "set_mode") return (btn && btn.dataset.mode) || "";
  if (cmd === "select_target") return String($("sel-target").value || 0);
  if (cmd === "run_test_motion") return String($("test-motion").value || 0);
  if (cmd === "select_payload_profile") {
    const sel = $("profile-select");
    if (!sel.value) {
      logline("select_payload_profile -> nothing selected (no profiles listed; "
              + "see the note under Payload)", "err");
      return null;
    }
    return String(sel.value);
  }
  return "";
}

function wireCommands(rootId) {
  const root = document.getElementById(rootId);
  if (!root) return;
  root.addEventListener("click", async (ev) => {
    const btn = ev.target.closest("button[data-cmd]");
    if (!btn) return;
    const cmd = btn.dataset.cmd;
    const arg = argFor(cmd, btn);
    if (arg === null) return;           // refused locally, already logged
    await sendCommand(cmd, arg, btn);
  });
}
wireCommands("mode-controls");
wireCommands("controls");
wireCommands("p-payload");

let PROFILE_NAMES = [];
async function refreshProfiles() {
  const sel = $("profile-select"), note = $("profile-note");
  try {
    const j = await (await fetch("/api/payload_profiles")).json();
    PROFILE_NAMES = j.profiles || [];
    sel.innerHTML = "";
    for (const name of PROFILE_NAMES) {
      const o = document.createElement("option");
      o.value = name; o.textContent = name;
      sel.appendChild(o);
    }
    if (!PROFILE_NAMES.length) {
      const o = document.createElement("option");
      o.value = ""; o.textContent = "(none found)";
      sel.appendChild(o);
      note.textContent = j.error || "no profiles";
    } else {
      note.textContent = `${PROFILE_NAMES.length} in ${j.dir} — names come from `
        + `that directory; controld validates them and says why if one is bad`;
    }
    markActiveProfile();
  } catch (e) {
    note.textContent = "cannot read the profile list: " + e;
  }
}

function markActiveProfile() {
  // Annotate, do not overwrite: the operator may be pointing at a different
  // profile on purpose, and the dropdown must not fight the telemetry.
  const sel = $("profile-select");
  if (typeof LAST_T === "undefined" || !PROFILE_NAMES.length) return;
  const active = (LAST_T && LAST_T.payload_profile_name) || "";
  for (const o of sel.options) {
    o.textContent = o.value === active && o.value ? `${o.value} (active)` : o.value;
  }
}
refreshProfiles();

function videoStatus(text, err) {
  const el = $("video-status");
  el.textContent = text;
  el.style.color = err ? "var(--err)" : "";
}
function applyVideo(on, st) {
  const img = $("video-img"), ph = $("video-ph");
  $("video-toggle").checked = on;
  // The query string is not about caching. Re-assigning the SAME url to an <img>
  // whose multipart stream already died is a no-op in at least one engine, so a
  // reconnect would leave the last frame up and read as "the camera is frozen".
  // A fresh url forces a new request, which is what "video is back" has to mean.
  if (on) { img.src = "/api/video?t=" + Date.now(); img.style.display = "block"; ph.style.display = "none"; }
  else { img.removeAttribute("src"); img.style.display = "none"; ph.style.display = "flex"; }
  if (st && st.running)
    videoStatus(`${st.camera || "camera"} ${st.width}x${st.height} @ ${st.fps} fps · ${st.frames_published} frames`);
  else if (st && st.error) videoStatus(st.error, true);
  else videoStatus("off");
}
async function setVideo(on) {
  const tg = $("video-toggle"); tg.disabled = true;
  videoStatus(on ? "opening camera…" : "closing camera…");
  try {
    const r = await fetch(on ? "/api/video/start" : "/api/video/stop", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: "{}",
    });
    const j = await r.json();
    applyVideo(j.running, j);
    if (!j.running && j.error) { videoStatus(j.error, true); logline(`video -> ${j.error}`, "err"); }
    else logline(`video -> ${j.running ? "ON" : "off"}`, j.running ? "ok" : undefined);
  } catch (e) { videoStatus(`error: ${e}`, true); tg.checked = !on; }
  finally { tg.disabled = false; }
}
async function refreshVideoState() {
  try {
    const r = await fetch("/api/video/state");
    const st = await r.json();
    // Pass st through (it used to be null): after a reconnect the caption is the
    // only place that says which camera/resolution came back and whether frames
    // are actually moving, so restoring the picture without it is half a sync.
    applyVideo(st.running, st);
  }
  catch (e) { videoStatus(`webd unreachable: ${e}`, true); }
}
$("video-toggle").addEventListener("change", (e) => setVideo(e.target.checked));

connect();
refreshVideoState();
</script>
</body>
</html>
"""


def dashboard_html(title: str = "OpenAutoTurret") -> str:
    """Return the dashboard HTML with the given title substituted."""
    return DASHBOARD_HTML.replace("__TITLE__", title)
