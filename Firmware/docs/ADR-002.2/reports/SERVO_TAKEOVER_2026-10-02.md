# Servo takeover report — 2026-10-02 (overnight, station rpi-turret)

Owner rulings and the measured facts that justify this work are in
[STATION_OPERATIONS.md](../../STATION_OPERATIONS.md) §"Servo takeover"; procedure is the
[servo commissioning card](../../operations/servo-commissioning.md). This report is the evidence.

## Outcome

Both axes now track ADR-003-shaped references (hold, 0.1–2° steps, 1–20°/s ramps both ways,
reversal, ±90° moves, a walking-person profile) on the real station, in bounded commissiond
sessions, repeatably. Production is untouched and still uses the old paths (see "Not done").

| Yaw, same use-case script | Before (Candidate13, `yaw-demo-27`) | Now (6 full passes, final config) |
|---|---|---|
| 5°/s request | peaks 20–34°/s, jitter 4–8°/s | speed ratio 0.996–1.003, jitter 0.54–1.5°/s (one recovered stick event: 3.8°/s) |
| 1–2°/s ramps | not usable (stick-slip) | ratio 0.990–1.018, jitter 0.43–0.75°/s, P95–P5 0.04–0.09° (one pass stalled before stall recovery existed) |
| 10–20°/s ramps | bursts, 9°/s SD at 20°/s | error RMS 0.04–0.13°, jitter 0.54–0.99°/s at 20°/s |
| Small moves (0.1–2° steps) | endpoint errors up to 6.2° (5–30° moves) | final error 0.0–0.23°, no overshoot on ≥0.5° steps |
| ±90° moves | — | stop 0.15–0.24° short |
| Walking profile | — | 0.11–0.15° RMS, ≤0.5° max |
| Stalls | frequent | 0–1 per 140 s pass; recovered automatically since `yaw-rock-*` |

Pitch (CyberGear, speed mode, outer kp 35/s): ramps 2–20°/s error RMS 0.03–0.06°, max ≤0.17°;
steps final ≤0.024°; walking profile 0.05° RMS; two passes agree to ~0.01°. kp 50 also clean.

Speed jitter is computed from the 1 kHz encoder over 20 ms and has a quantization floor near
0.4°/s; values at that level are measurement-limited. Against the original ADR-002.2 limits: speed
ratio passes everywhere; detrended P95–P5 (≤0.15°) passes up to 10°/s and is 0.09–0.21° at 20°/s;
jitter passes at ≥10°/s and sits at the 0.5°/s floor below;
startup time was not separately measured; the 0.15° independent angle *prediction* gate belongs to
the model-first approach and was not used.

## What was wrong (root causes, in order of impact)

1. **GM6020 encoder current crosstalk.** The angle reading moves with phase current (periodic,
   nine cycles/rev, up to ±6.4 mrad/A, 2.1 ms). The controller's own current fed back as fake
   position error and produced the 14–17 Hz limit cycles that appeared at some angles and not
   others. Found by chirp (`claude-frf-hold4`: flat, phase-inverted 4.7 mrad/A from 8–60 Hz),
   calibrated by an 80 Hz probe-tone scan over 360° (`claude-gscan360`, `-b`; two scans agree),
   compensated with a −2 mrad/A safety bias (simulation shows over-compensation is the stable side).
2. **Wrong plant numbers in the old models.** Inertia is 0.03 A·s²/rad (old: 0.0895); friction
   falls with speed (≈0.6 A at 3–6°/s → 0.3 A at 65–120°/s); breakaway after loaded rest exceeds
   0.9 A at times; slowly rising force gives creep-and-restick (open-loop staircases
   `claude-ol-pos/-neg`), a brief reversal releases it.
3. **Controller structure.** The old core's position stiffness was ≈0.5 A/rad, a START state that
   slammed 0.9 A, and a hard abort on START timeout. Stick-slip and lurches followed.
4. **Sensing.** The BNO085 gyro lags the encoder by ≈100 ms; fusing it into the derivative path
   caused ringing. The servo uses the 1 kHz encoder only.

## What was built

- `axis_control_core/servo.{hpp,cpp}`: position servo — encoder Kalman observer with crosstalk
  removal, PID on position, reference-keyed friction/inertia feedforward, per-angle friction map
  learned online (carried between sessions), stall recovery (25 ms reverse rock: 20/20 releases
  when deliberately starved at 0.55 A, `yaw-rock-provoke2`), RMS thermal budget, following-error
  and data-staleness stops. No START state machine, no latching abort.
- `commission_runtime/yaw_control_session.cpp`: `servo_parameters` mode, event-driven at 1 kHz on
  encoder frames, gain schedules, probe-tone excitation, speed trip on 20 ms encoder displacement or
  gyro, temperature trip; references relative to start; absolute-angle coordinates.
- `commission_runtime/homing_session.cpp`: `servo_trial` for pitch — speed mode, SpdRef streamed at
  1 kHz from a host position loop, slow approach to the measured centre, absolute window,
  following-error stop; then the existing STOP/restore path.
- `tools/servo_commission/`: manifest builder, run wrapper, scorer, crosstalk calibration, templates.
- `config/servo/{yaw,pitch}_servo.json`: the calibrated assets with provenance.

Gains: yaw kq 90 A/rad, kv 1.6 A·s/rad, ki 135 — one step inside the stability boundary measured at
the worst angle (`claude-ladder-04`: kq 120/kv 2.0 onset) and verified around the whole circle
(`claude-circle-02`: no oscillation window). Pitch kp 35/s, ki 30/s², drive SpdKp 4 / SpdKi 0.05.

Workstation checks: native build clean; `ctest -E retained_homing` 82/83 — the one failure,
`test_mixed_station_config`, predates this work. `tools/tests/test_adr0022_baseline_bundle.py` fails
18/48 both before and after this work (stale hash/revision expectations).

## Not done, and what closes ADR-002.x

- **Production integration (3b).** Production yaw still runs the host speed PI and pitch the
  SpeedServo path. The servo takes one `{q, v, a}` reference per tick — the ADR-003
  `ReferenceSample` boundary — so integration is the first ADR-003 work item: the reference
  generator feeds the servo inside the backend at 1 kHz on yaw feedback, and pitch moves to the
  1 kHz speed-mode loop. Production stays off until then (owner ruling).
- **Pitch homing in the new path** keeps the existing sensorless homing; the servo trial assumes
  the 2026-09-30 endpoints (no reboot since). A reboot or remount requires homing first.
- **Bearing.** Breakaway after loaded rest can exceed the 0.9 A continuous rating; friction drifts
  with temperature. Mechanical inspection (preload, grease, the 240–255° bump) would widen margins
  more than any controller change. A warm-up rotation before service is a cheap mitigation.
- **Tests.** `servo_core` (CTest, `axis_control_core/tests/test_servo.cpp`) covers crosstalk
  removal, following-error and stale-data stops, the RMS budget, stall rocking and reference-only
  friction feedforward; it passes natively and under ARM64 emulation. Session-level fault injection
  for the new modes is not written (owner ordering: working product first).
