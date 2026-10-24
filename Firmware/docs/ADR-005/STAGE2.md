# ADR-005 stage 2: runs from the page — build plan and status

Stage 2 of [ADR-005](ADR-005.md) (D7, D8): VERIFY, RE-RUN and NEW's measurement run on the station
from MENU › CALIBRATION, in a CALIBRATING phase inside controld, orchestrated by `calibd`.
Stage 1 (records, switching, bore trims) is built and verified on the station (2026-10-24, `0309d0c`).

## What the code already gives (read 2026-10-24)

- **The pipeline is station-agnostic.** `commission.py` asks its station for one session at a time,
  `station.run(axis, manifest, label)`, and gets a journal dict back (`journal.yaw` / `journal.pitch`).
  `Station` does it over ssh with commissiond; `SimStation` with `servo-sim`. calibd adds a third,
  `ControldStation`, and the identification, design and scoring code stays as it is.
- **The journal interface is narrow.**
  - Yaw: `header`, `yaw_control_begin`, `yaw_feedback`, `yaw_current_tx`, `servo_cycle`,
    `servo_learned`, `servo_gains`, `footer`, and pitch `register_read` for the idle-axis record.
  - Pitch: `header`, `pitch_trial_window`, `pitch_trial_approach`, `pitch_trial`,
    `pitch_trial_gains`, `footer`, and yaw `can_rx` for the idle-axis record.
- **A session is a manifest.** It holds the servo or loop parameters, a 50 Hz reference table, an
  optional gain schedule and excitation, and its limits. commissiond's two executors
  (`yaw_control_session.cpp`, the trial in `homing_session.cpp`) step the same `axis::Servo` and
  `axis::PositionLoop` that production runs.
- **What production lacks.**
  - Kernel receive timestamps: `SocketCanBus` stamps `rx_ns` in user space; commissiond's
    `TimestampedReceiver` takes the kernel's.
  - A journal writer: commissiond's `Journal` is a single-producer ring with its own disk thread.
  - scipy (`identify.py`, `score.py`). An arm64 wheel exists for the station venv's Python 3.13
    (scipy 1.18.1, checked 2026-10-24); `requirements-station.txt` is installed by every deploy.

## The protocol: page → controld ← calibd

calibd is a second client of controld's web socket (`max_clients` 8) and reads its telemetry.

| Command | From | When | Effect |
|---|---|---|---|
| `verify_calibration` | page | Homed, IMU fresh, no switch | enters CALIBRATING, run kind VERIFY |
| `run_calibration <kind>` | page | the same; not on #1 | RERUN, RERUN+ACCURACY, RERUN+CROSSTALK; NEW comes from `new_calibration_record` |
| `stop_calibration` | page | CALIBRATING | ends the session now, both axes held, run STOPPED, MANUAL |
| `calibration_session <run> <axis> <manifest>` | calibd | CALIBRATING, no session | controld runs one session and writes its journal beside the manifest |
| `calibration_note <run> <text>` | calibd | CALIBRATING | the run view's last line |
| `calibration_finish <run> <run dir> <verdict>` | calibd | the run in progress, no session; once | installs the accepted parts from `<run dir>/install/` (if any) through the swap hold, then resumes the interrupted mode |

- **calibd must answer.** A run that sees no calibd command for 15 s, while no session is active and
  no finish is being installed, ends NOT RUN ("calibd is not answering"). This is a persistence
  timeout, not a fault.
- **Telemetry `calibration_run`:** run id, kind, state (`waiting`, `session`, `fitting`, `done`,
  `stopped`, `failed`), the steps, the current session (label, axis, elapsed, expected), the last
  note, and the verdict.
- **STOP MOTION and HOLD during a run are STOP.** STOP is the operator's word, so the run ends in
  MANUAL, as STOP does during a swap hold. A failed run, which no one stopped, returns to the mode it
  interrupted (ADR-005 D7, classification).

## Increments

| # | Scope | Proved by | Status |
|---|---|---|---|
| S2.1 | CALIBRATING, the run lifecycle and gates, `calibration_run` telemetry, the journal writer, the **pitch** session executor (yaw held) | unit tests on the sim backend | built 2026-10-24 |
| S2.2 | the **yaw** session executor (pitch held), kernel RX timestamps on the yaw bus | unit tests; production servo unchanged in the sim | built 2026-10-24 |
| S2.3 | calibd and `ControldStation`, scipy, the launcher; the page's run view and STOP | Python tests against a fake controld; the page in the browser | built 2026-10-24 |
| S2.4 | installing accepted parts (`history/`) and resuming; NEW's measurement | unit tests | built 2026-10-24 |
| S2.5 | VERIFY (`check yaw`), with D8's two fixes (session end status; the pitch gate) | Python tests; the first VERIFY on the station | built 2026-10-24 |
| S2.6 | the servo-commissioning card rewritten | doc-tree check | done 2026-10-24 |

**On the station, with the owner's go only** (no real calibration until the payload is mounted and
the owner says so): pitch identify through commissiond and through controld on the same plant,
comparing journals and fits (ADR-005 risk 1); then the first VERIFY; then a RE-RUN.

## As built

- **S2.1 (2026-10-24).**
  - CALIBRATING is an overlay on the homed hold, like the swap hold: the phase stays `hold`, the
    mode is MANUAL, and telemetry says `calibrating`. A new phase would have needed a second copy of
    the hold's stepping.
  - The session executor is `calibration/calibration_session.*`. The backend's pitch servo thread
    steps it under the production guards (drive running, fresh feedback, the end-stop guard and
    the travel governor).
  - commissiond's `Journal` moved to `capture/journal.*`, which both executables link.
  - A session ends Holding. The control loop then re-anchors MANUAL's hold where the axis stands,
    the backend hands the axis back to production, and the writer thread writes the idle axis's
    record and the footer.
  - A supervisor BRAKE, FAULT-STOP or DISABLE stops the session where it is.
- **S2.1, found on the station.** The first VERIFY was refused "HOME first" on a homed station.
  `homed_` is false there by design, because a continuous yaw has no endpoint; `position_ready()`
  is the station's "homed". The simulator never showed it, since its yaw is bounded.
- **S2.2 (2026-10-24).**
  - The yaw executor steps its own `axis::Servo` (the manifest's parameters) on every GM6020 frame,
    under production's guards. Its trips (following error, sensor data, 20 ms encoder speed,
    oscillation, temperature, transmit) end the session.
  - When the session ends it sends zero current until production's servo takes yaw back, which is
    one control cycle at most.
  - The speed check uses the encoder only. commissiond also read the gyro; the gyro still gates a
    run, but it does not reach the backend.
  - Every CAN frame now carries the kernel's receipt time (`SO_TIMESTAMPNS`, mapped to the
    monotonic clock), and production's servos step on it too.
  - A session's journal header holds the manifest re-emitted as JSON, as commissiond's does.
  - `journal.py` reads controld's `idle_axis` row. `tools/servo_commission/tests/test_controld_journal.py`
    parses the journals the C++ executors write.
- **S2.3 (2026-10-24).**
  - `tools/servo_commission/calibd.py` waits for a run in CALIBRATING. It runs the kind's
    pipelines (`check`, `pitch`, `yaw`, `calibrate`) with a `ControldStation`, which gives a
    session to controld and returns its journal parsed.
  - A heartbeat note every 5 s keeps controld's 15 s window open through long fits. Notes read
    `<step> | <line>`, which the run view shows.
  - Runs are kept in `<state>/calibration/runs/`. The pipelines write nothing to `config/`
    (`no_write`).
  - `commission.py` gained an injected station, a log hook, the asset taken from the record, and a
    runs root; it is otherwise unchanged.
  - The launcher starts calibd at nice 10, beside webd, and does not wait on it.
  - The page: VERIFY and RE-RUN (with +ACCURACY and +CROSSTALK) are live and take two presses.
    During a run the drawer is the run view (steps, the session with its time, calibd's last
    line, STOP), the mode block reads CALIBRATING, the dock is dimmed and the D-pad hidden. After
    the run the panel shows LAST RUN.
  - `scipy` is in `requirements-station.txt` and in the test venv.
- **Station, 2026-10-24 18:10, the first VERIFY.** It reached the station from the page and its
  yaw probe session started. Then the station faulted, with both motors off and pitch near level.
  Two bugs, both fixed:
  - The web thread prepared the session (parsed the manifest, allocated and fsynced the 64 MB
    journal ring) under `command_mutex_`, which the 200 Hz loop takes every cycle. The loop waited
    160 ms, and the independent motor watchdog tripped. Since then the web thread's preparation runs
    under its own `web_prep_mutex_`, and `command_mutex_` guards only the gate's state and the
    queue.
  - The run was stepped only inside the homed hold, so in Fault it never ended, and its CALIBRATING
    gate refused the HOME that recovers. Since then it is stepped every cycle in every phase, and a
    fault ends it as "failed".
  - Both have tests that fail on the old code (`CalibrationRun.PreparingABigSession…`,
    `CalibrationRun.AFaultDuringASession…`).
- **S2.4 (2026-10-24).**
  - calibd copies the parts of a run whose every pipeline was accepted into `<run>/install/`.
    Anything less installs nothing: a payload set is one set.
  - controld installs them into the record the run measured, on the web thread. It checks them
    first on a staged copy, keeps the replaced files in `history/<run>/`, marks the parts
    `measured`, and notes `last_run` in record.json.
  - It then puts the record back in force through the swap hold and resumes the interrupted mode.
  - **No HOME after a run** (D7.6 said HOME). The homed frame survives a run, because the encoders
    never lose their count, and the new servos go in force the way a switch puts them. The owner's
    ruling for after a run is "the mode it interrupted".
  - A finished VERIFY, or an installed measurement, clears the VERIFY nudge.
- **NEW's measurement.** `new_calibration_record <from> +MEASURE <name>` makes the copy, switches to
  it, and once the switch has settled starts a MEASURE run, resuming the interrupted mode after it.
  STOP during the switch cancels the measurement. From Shutdown the record is made and put in force,
  and the answer says to HOME and then RE-RUN. The NEW dialog offers MEASURE IT NOW when homed.
- **S2.5, D8's fixes.** The pitch gate (`score.gate_pitch`) passed anything. It now fails a run with
  no scored segments, or with a ramp or the walker above 0.5 deg error RMS: ten times the 10-02
  asset's worst, so it catches a loop that does not follow. `check yaw` counts a probe that did not
  end COMPLETE as a failure.
- **Station, 2026-10-24 18:27, the first VERIFY that ran.**
  - From the page, on #1 AS SHIPPED. The yaw probe session ended COMPLETE, calibd scored it on the
    Pi in about 3 s, and the run took 61 s. AUTO_ROAM resumed.
  - Verdict UPDATE_PARAMETERS. The tracking gates pass (speed ratio 0.965–1.004), but the friction
    is 26–50 % below #1's 10-02 asset. That is the drift record #2 (the 10-20 update, 38–62 % lower)
    was measured for.
  - The probe's journal is 52 MB. A full yaw run is about 1 GB, and journal pruning is still to do.
- **CHECK-PITCH (developer only, 2026-10-24).** It is `run_calibration CHECK-PITCH`, not offered on
  the page. It runs `commission.py check pitch` (the pitch use case with the record's pitch asset)
  in CALIBRATING and installs nothing. It exists so controld's pitch executor can be tried on the
  station before the first real RE-RUN: VERIFY is yaw only (D8).
- **Station, 2026-10-24 18:31, VERIFY on #2 FRICTION 10-20.** COMPLETE in 60 s, verdict
  UPDATE_PARAMETERS. The friction drift is −25, −20, −2, −54, +33 and 0 % across the six speed
  plateaus, against #1's uniform −26 to −50 %. So #2 fits better than #1, but fails the 25 % rule at
  two plateaus. Both records pass the tracking gates. #1 was put back in force afterwards.
- **Station, 2026-10-24 18:41, CHECK-PITCH on #1: controld's pitch executor on the hardware.**
  - The window came from controld's own homing (−0.611 to +0.601 rad).
  - The session ended COMPLETE in 60 s with verdict REUSE_EXACT: ramp RMS 0.018–0.022 deg, walker
    0.022 deg, final step error ≤ 0.003 deg. That is at least as good as the asset's 10-02
    validation with commissiond (0.021–0.045, 0.031 and ≤ 0.007 deg).
  - Held yaw moved 1.5 mrad over the session, under the 0.2 deg decoupling record.
  - **Still to run on the hardware:** the pitch identify sweep and its fit, the yaw survey,
    crosstalk, inertia, ladder and circle sessions, and an install. All are parts of a RE-RUN, which
    waits for the owner's payload and go-ahead.
- **Readiness check before the payload, 2026-10-24 19:00–21:30.**
  - STOP during a VERIFY's session, from the page: both axes held, MANUAL, LAST RUN STOPPED.
  - At 375 px the panel's notes ran off the edge (drawer rows are made for one-word states). They
    wrap now; nothing in the panel is wider than the phone.
  - A run that ends without calibd's finish (STOP, a fault, a silent calibd) is recorded as the
    record's LAST RUN by controld. calibd sends no finish after a STOP.
  - **A finish acted before it was accepted.** The web thread installed parts and wrote
    record.json for any finish, and the control thread refused a finish for a run that had ended
    only afterwards. So a late finish (calibd silent for 15 s during a fit, then done) would have
    put parts in the record that never took force. Only the run in progress, once, touches a
    record now. Once its finish is taken in, the run waits for it: the silence rule does not end it,
    and a STOP ends it in MANUAL after the install. A fault in that window still ends it; the parts
    are then in the record and take force at the next switch, and the refusal says so.
    `CalibrationRun.OnlyTheRunInProgressFinishesIntoARecord` fails on the old code.

- **PAYLOAD 10-24: why the yaw inertia identification failed, and the fix (2026-10-24, simulator
  and recorded data only; branch `claude/inertia-heavy-payload-7a56f6`).** RE-RUN +ACCURACY on #5
  ended `inertia: only 1 coherent frequency bands`; nothing was installed.
  - **The payload is about 5× the bare turret: J = 0.20 ± 0.04 A·s²/rad** (bare 0.0407). Two
    independent methods on the recorded sessions: the survey's speed changes, friction cancelled
    by pairing accelerating and decelerating current at the same speed (0.18–0.21 on the survey
    and the inertia session; the same method reads the bare turret at 0.036–0.040 on four
    sessions); and the VERIFY probe's step responses simulated on the asset's plant with today's
    friction, which excludes J ≥ 0.3 (the 2° steps would overshoot 16 mrad against the measured
    8–14) and sits best at 0.16–0.25. The main session's reversal regression (0.21) agrees.
  - **controld's executor is not the cause.** Frame timing 1.00 ms median, no reordering; the
    transmitted and measured current carry the sweep with coherence 0.99 over 10–24 Hz; and the
    raw encoder/current ratio along the sweep follows the asset's crosstalk table to within
    ~1 mrad/A, so the compensation's timing and table are right.
  - **Root cause, quantitatively.** A rigid body answers the 0.3 A sweep with 1/(J w²): at
    12.6 Hz that is 4.3 mrad/A for the bare turret and 0.8 mrad/A for the payload, against (a) the
    table's drift since 10-02, +1.1 mrad/A overall (10-20 showed +0.9) with ~0.5 mrad/A by angle,
    and (b) the encoder's own noise, 0.05–0.13 mrad per band at 8–30 Hz — the same at 5–40 deg/s
    and for both inertias, so neither a spatial pattern nor motion. The bare turret's response
    sat 4–10× above both; the payload's was below them. The fit's leftover-crosstalk term was
    bounded at the scan's repeatability (±0.48 mrad/A) and so could not absorb the drift: on
    10-20 it was already pinned at the bound. A Hanning window on the sweep's record also weighted
    its first band by a few per cent (8.3 Hz cohered at 0.05 on 10-02 too).
  - **Why the simulator did not show it.** Its plant had neither noise term: 5–10× quieter than
    the station at 8–30 Hz (it matched only above 200 Hz, where quantization dominates). With the
    measured encoder noise and a friction roughness calibrated to the un-excited ramp's 2–8 Hz
    motion, the old sweep on a 5× plant with a drifted table gives J −45…−66% while 15/16 bands
    cohere — worse than the station's failure: it would have installed a wrong inertia.
  - **The change** (`tools/servo_commission`, README "Calibrations"): the sweep's start follows the
    plant — the survey's rough inertia picks the nearest of 8, 5.6, 4, 3 Hz to twice the prior
    loop's resonance (lower, a light plant's sliding reverses or trips the speed limit at 3–4 Hz;
    proven in the simulator for 0.015–0.045) — at 5.5 s per octave; the bare turret's sweep is
    unchanged, a 5× payload gets 3–30 Hz over 18.3 s (session 45 s instead of 30). Flat-top
    window; leftover crosstalk bounded by the design's drift allowance; loop delay bounded to 4 ms;
    the design's and the ladder's gain floor is now a loop stiffness (kq 6.75 A/rad: the old
    15 rad/s at the bare turret, 5.5 at 5×). Proof in `tests/test_pipeline.py` on the noisy plants: identification within 15%
    for 0.015, 0.03, 0.075, 0.20, 0.225 with a scanned and with a drifted table, the drift found
    as r = −1.1 mrad/A; the survey estimate within 30%; the full 5× pipeline (from the prior, and
    `--update` with a drifted table) ACCEPTED: identification 0.242, design boundary wn 6.5 rad/s.
  - **The 5× design is capped by the drift allowance, not by identification.** With the table
    assumed wrong by its largest |g| (6 mrad/A, `crosstalk_drift_fraction` 1.0) the right-half-plane
    zero 1/sqrt(J g) sits at 26 rad/s for J 0.24, and the quiet gain is wn 6.5–9.3 (bare turret:
    30–55). The commissioned servo (wn 5.5, kq 7.3) passes the acceptance gates but tracks 2–3×
    outside the bare turret's calibrated accuracy limits (ramp RMS 0.30–0.59° against 0.19°). The
    station on 10-24 drifted 1.1 mrad/A in three weeks; whether a payload may be designed against a
    smaller allowance (and re-checked more often) is the owner's call, not this change's.
  - **10× is a prior problem, not an identification one.** Under kq 20 / kv 0.5 (ζ 0.08 at
    0.45 A·s²/rad) the simulated loop limit-cycles through friction, 2.7 A peaks and ±0.7 rad/s
    about the sliding speed, with or without noise, and the survey trips the speed limit. Pinned
    as an expected failure (`test_prior_gains_at_10x`).
  - **Still to confirm on the station (MEASURE on #5):** the survey's rough inertia (expect
    ≈0.2 → a 3 Hz sweep), the 3–30 Hz sweep's coherence (the sim's 2–8 Hz floor is calibrated on
    the bare turret; the payload's is about 2× it), the fit's r ≈ +1.1 mrad/A, and the design's
    outcome at 5× under the drift allowance.
- **Station, 2026-10-24 22:45–23:19: the inertia fix on the station, and three faults it surfaced.**
  - **The deploy's homing faulted with the payload** (`endpoint A homing failed: backoff: timeout`).
    From the rest stop, the pitch lurched 0.3° breaking away (one 50 ms step above half the approach
    speed), then crawled up against the payload's weight at ~0.025 rad/s while the drive wound up.
    The push cap armed on the lurch and took the crawl for a stop at the 0.8 N·m gate; the backoff
    then drove into the real stop. Fixed in `6752fa3`: the cap arms after two running steps (the
    station's recorded feedback is the test). HOME then passed in 35 s from the same pose.
  - **In that FAULT the pitch kept pushing into its stop for 11 minutes** (33 → 49 °C, 1.4 °C/min):
    the hold wrote SpdRef 0, which a CyberGear in position mode ignores, so the backoff's target
    past the stop stayed in force. Fixed in `e688606`: a hold writes SpdRef 0 and LocRef at the
    measured position, the neutral pair adoption writes, so the drive stops in either mode.
  - **MEASURE on #5, 22:58–23:19, NOT ACCEPTED at yaw validation.** Pitch ACCEPTED (ramp RMS
    0.027–0.053°). Yaw: the survey's rough inertia 0.216 → sweep 3–30 Hz over 18.3 s → J 0.2128,
    delay 1.97 ms, residual crosstalk +0.45 mrad/A, 5/16 bands (the prediction held). Design bias 3,
    boundary wn 9.7; the ladder was quiet to wn 12.6 at 156° and 324°; chosen wn 8.2 (the robust
    design, under the drift allowance); circle at 1.1× quiet. Validation: pass 1 ACCEPTED (speed
    ratio 0.959–1.051); pass 2 failed the slow ramps (±1 deg/s: 1.104 and 1.158 against 10%,
    3 stalls; steps stop short then lurch). Nothing was installed.
  - **That run's journals were pruned as it ended**: calibd sorted runs by name, and run ids restart
    with every controld, so the newest run (0001) looked oldest. Fixed (`calibd` orders runs by the
    stamp in their names). Its reports, logs and manifests remain; the per-segment rows are in
    `yaw/report.json`.
- **Why the 5× payload failed yaw validation at wn 8.2, and the fix (2026-10-25, simulator and
  recorded data; branch `claude/payload-slow-ramps-5x`).**
  - **The speed-ratio excess is a stuck start.** ramp±1's 1.104/1.158 over the 7 s window is
    0.7–1.1° of extra travel, the ramps end on target, and the "jitter" (0.9–1.1° p95-5) is the
    catch-up slope: the axis stood for ~1–1.5 s after the ramp began. The preceding step had left it
    0.5° ahead with the integral wound to ≈ −0.3 A against friction; the ramp started with FF +0.36,
    I −0.3, under the 0.37–0.42 A breakaway, and at kq 14.5 / ki 21.7 it took ~1.2 s and 1° of lag
    before P and the unwinding integral broke it free. The bare turret (kq 30.8, ki 46) does the same
    in ~0.4 s and passes. Pass 1's small steps overshooting 72–194% (a 0.1° step travelling 0.23°)
    is the mirror image: a release by a stall rock when the friction level sat below the feedforward.
  - **The cause is friction drift against the feedforward, not stick-slip while tracking.** Three
    surveys of the same payload the same evening (21:16, 22:58, 23:43) disagree by 0.06–0.3 A at
    0.25–5 deg/s; the validation passes meet a feedforward that is ±0.1–0.15 A off. The station
    stick-slips at slow speed under the prior's gains, bare and payload alike (survey plateaus:
    30–43% of the time stuck), with 60–100 ms stick phases: a lightly damped loop limit cycle, not
    slow stiction; the static excess at stick-end is +0.02…0.09 A only.
  - **Reproduced in the simulator** with a plant shaped as measured (`sim_cases/payload_5x_knee`:
    friction flat to ~2 deg/s then a knee of −2 A·s/rad that the fit's exponential cannot follow —
    `fit_friction` bounds stribeck_speed at 0.08 rad/s — and `friction_drift_A` 0.1: the simulated
    station moves the Coulomb levels ±0.1 A from one session to the next). With friction +0.15 A over
    the feedforward the candidate servo gives ramp+1 1.108/1.132 and 3–6 stalls (station 1.104/1.158,
    3 stalls); −0.10 A gives 0.88–0.92 and step overshoots of 39–133%. The full pipeline on that
    plant at integral rate 1.5 failed two of three start angles (speed ratio 0.874; 6 stalls).
  - **Fix: the design's integral rate 1.5 → 3 (`yaw_prior.json` design_rules), the PI corner never
    above 0.4·wn (`design.gains`).** Six of six start angles accepted (speed ratios 0.91–1.03, ≤3
    stalls); the crosstalk-robust boundary is unchanged (9.84 vs 9.7 rad/s). The corner bound comes
    from the `--update` test: under a drifted table the design lands at wn 4.6, where a 3 rad/s
    corner rang the 10–20 deg/s ramps to 12° of jitter. Not available instead: more damping (kv feeds the crosstalk loop: ζ 0.75
    lowers the boundary to 6.9 rad/s, ζ 1.0 to "inadequate"), breakaway help
    (`friction_correction_rate`: trades the speed ratio for 5–9 stalls), rocks during slow ramps
    (more stalls), a wider hold band (no effect), slow-speed friction learning (the first slow ramp
    is always cold). The bare turret's design gets ki 46 → 92 (PI corner 3 rad/s against wn 27.5);
    its pipeline test still passes.
  - **Still the owner's:** (a) under a +0.15 A rise the small steps need 4–8 stall rocks per pass
    at any gain tried, against a gate of 4 — a threshold question; (b) the crosstalk drift
    allowance (1.0 × max|g|) caps the payload at kq ≈ 14–21; at 0.5 the design gives wn 10.6
    (kq 24), speed ratios 0.976–1.031 everywhere and 1.6× better accuracy, for 3× less margin
    against a crosstalk drift the station has shown at 1.1 mrad/A in three weeks; (c) the FF's
    friction fit cannot follow the measured knee or the hour-to-hour level — the drift is real, so
    the loop must carry it; the fit's instability (23:43: + 0.18+0.62, creep 0.68) comes from the
    plateau means being limit-cycle averages under the prior.
  - **Confirmed on run 2's journals (MEASURE 23:55, same failure: pass 2 ramp−1 at 1.149).** Every
    0.2 s from that ramp's start: the feedforward steps to −0.19 A, the axis breaks away and slips
    0.5° ahead (2.5× the reference speed at 0.2 s); while ahead the loop winds the integral against
    the direction of travel (+0.04 → +0.13 A); the axis then stands from 0.6 to 1.8 s while the
    reference passes it (0.5° ahead → 0.63° behind) until the integral has unwound and P has grown
    to the −0.38 A breakaway; the catch-up after 1.8 s is the "speed". Pass 1's ramp−1 shows the same
    slip with a shorter stick (0.985). The unwinding rate, ki·e ≈ 20.8 × 5 mrad ≈ 0.1 A/s as seen,
    is what the rule change triples. The exact servo-level root — an integral that winds against a
    moving reference while the axis stands ahead of it — would be a conditional-integration change
    in `axis_control_core/servo.cpp` (production code; not made here).
  - **The next station run (MEASURE on #5 with this branch) must show:** `ki ≈ 3 × kq` in the ladder
    line (≈ 40–65), both passes accepted with speed ratios inside 0.93–1.07, and in the use-case
    journals a stick at ramp±1's start no longer than ~0.5 s with the integral unwinding at ≥ 0.3 A/s.
