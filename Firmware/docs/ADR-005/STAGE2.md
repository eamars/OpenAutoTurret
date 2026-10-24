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
- **Station, 2026-10-24 23:43 – 2026-10-25 00:38: a CAN hole, a homing regression, and the hold's
  second heating.**
  - **23:51: a 30 ms yaw CAN dropout ended run 1's inertia session** (`MEASUREMENT_LIMITED: servo
    sensor data stale or invalid`), and with it the run. Owner, the same night: the station is real,
    and one transient must not end a run. Fixed in `d00ff46`: a session that ends on a data hole, a
    late start or a missing footer runs again under a new label (`-r1`, `-r2`), at most twice. A
    result is never repeated: guard trips, the plant's limits and the operator's STOP all count as
    results. The run's report lists each retry with its reason.
  - **00:28: two-step arming broke the fine approaches.** The second fine approach never armed the
    cap, met the stop 0.5° deeper, and failed the repeatability check. Fixed in `961b685`: only the
    coarse approach needs two steps, since the payload's crawl off the rest stop happens there. The
    fine approaches arm on one step again. HOME passed at 00:38 in 28 s. Not explained: why two
    steps never armed a fine approach. The open-loop replay of the recorded feedback diverges after
    the first backoff.
  - **In that FAULT the pitch heated again (+1.3 °C/min), after `e688606`.** The approach drives
    the CyberGear in speed mode. Its integral, wound against the stop, survives SpdRef 0. LocRef at
    the measured position then holds the stop compressed. **Proposed to the owner (not made):** a
    hold that releases a push into a stop, for example by holding just off it. That changes how a
    FAULT behaves.
- **MEASURE on #5, 00:38–01:12 (release `961b685`, with the slow-ramp fix): validation ACCEPTED,
  then NOT ACCEPTED in the accuracy campaign, which had measured the wrong servo.**
  - **Pitch ACCEPTED.** Yaw identification:
    - The survey's rough J of 0.219 set the sweep to 3–30 Hz.
    - Result: J 0.2647, against 0.213 and 0.229 in the two runs before, a 25% scatter. Loop delay
      0.00 ms, which is the fit's bound. Residual crosstalk −0.12 mrad/A. 5 of 16 bands coherent.
  - **Yaw design:** bias 3, boundary wn 8.2. The ladder was quiet up to wn 10.7 at 156° and 324°.
    The chosen wn 7.0 gives kq 12.97, kv 1.853, ki 36.3, so ki = 2.8 kq under the new rule. The
    circle at 1.1× was quiet.
  - **Validation ACCEPTED on both passes: the slow-ramp fix works on the station.** Speed ratios
    0.98–1.071 and 0.975–1.06; ramp±1 now 1.025–1.071, against 1.104–1.158 before. 1 and 4 stalls.
  - **Accuracy is 2–3× worse than the bare turret's limits on every metric:**
    - rests up to 0.63° off;
    - ramp RMS 0.23–0.62°;
    - slow-ramp P95-5 up to 2.2°;
    - walker RMS 0.53–0.56°.

    The model predicted 0.03–0.09° RMS. **Owner, 01:05:** long-term reliability after the initial
    calibration matters more than accuracy, so the drift allowance stays at 1.0.
  - **The accuracy campaign ran the record's bare-turret yaw asset**, not the asset the run had just
    accepted. That asset has J 0.041, kq 30.8, ki 46 and an integral cap of 0.73 A.
    - Cause: calibd gave every step the record's asset, and #5, made NEW from #1, holds #1's.
    - Failure: its third FF+FB pass, ff_fb2, stuck for about 9 s after its positioning move. The
      error sat at +0.35° while the drive pushed 0.55 A. Five 25 ms rocks counted as 5 stalls,
      over the gate of 4, so no limits were written and nothing was installed.
    - Fix 1: a later step measures the asset this run accepted (`calibd.Runner.asset_for`).
    - Fix 2: `asset_validation_journals` now finds the validation passes in calibd's layout, under
      `sessions/`, and calibrate counts only COMPLETE attempts. The limits now include those passes,
      as the campaign's docstring promises.
    - Incidental data: on this inertia the bare gains are wn 10.8. They ran three passes on the
      payload without a limit cycle, with the same accuracy (rests 0.45–0.64°).
  - **The accepted servo's stalls come from the integral cap, not the gains.**
    - Validation pass 2 had four stalls, all near a stationary reference at 0.23–0.46° error. In two
      of them the integral was pinned at its 0.4 A cap.
    - The design caps the integral at the mean low-speed friction (`integral_cap = 1.0 × low`),
      here 0.39 A. That is under this payload's + breakaway of 0.435 A, and well under breakaway
      plus the ±0.15 A drift.
    - Each rock zeroes the integral. It winds back to the cap in about 2 s, and the axis stalls
      again.
    - The bare turret never showed this. Its 10-02 asset's 0.73 A cap sits well above its friction
      today, which measured 38–62% below that asset's fit on 10-20.
    - This is likely what the simulator's "4–8 stall rocks per pass at any gain" was.
    - Under study with Fable: a cap that covers the larger direction's breakaway plus the drift.
  - **The cap study's verdict (Fable, 01:50; branch `claude/yaw-integral-cap`): half the stalls were
    the cap, the other half the stall detector; both rules now follow the plant.**
    - On the journals: all five stalls (uc0 1, uc1 4) sat at a stopped reference 0.23–0.9° behind
      with the friction feedforward at zero (the fit's creep term puts the model's rest friction at
      0.06 A), so the integral was the only authority. Two had it pinned at the 0.4 A cap; three
      fired 0.3 s after the push crossed the stall current of 0.353 A while the integral was still
      winding at 0.17–0.25 A/s — the window between 0.9×low and 1.0×low is 0.05 A. The plant needed
      ≥ 0.40–0.45 A (−) and ≥ 0.44–0.69 A (+) where the fit said 0.349 / 0.435. Not one rock freed
      the axis: each zeroed the integral and the error grew 0.1–0.24° the wrong way until the
      integral had rebuilt (2 s) or the next reference move took over. No stall repeated within a
      segment, so the "winds back to the cap and stalls again" cycle did not occur in this run; the
      bare servo's five stalls in ff_fb2 (cap 0.73, I at 0.48–0.57, not pinned) were the detector's.
    - Broken invariant: a stall must mean "the whole authority is applied and nothing moves". The
      rule now: `integral_cap = max(breakaway+, breakaway−) + friction_drift_A` (0.15 A, a new design
      rule in `yaw_prior.json`; ceiling 1.2 A), `stall_current = integral_cap`. For this payload:
      cap 0.585 (was 0.40), stall current 0.585 (was 0.353). Bare turret: 0.91 (was 0.73 / 0.66).
    - Proof (`payload_5x_knee`, Coulomb levels −0.15 / 0 / +0.15 A, starts 1.0 / 3.5 / 5.2 rad,
      three noise draws; both the pipeline's servo for that plant, kq 20.8 / ki 62, and the
      station's accepted kq 13 / ki 36): worst stalls in the validation pass 5 → 2 and 4 → 2 at
      +0.15 A, 3 → 2 and 3 → 0 at 0; a higher cap alone (stall current unchanged) leaves every
      count as it was. Long-move finals (0.18–0.47°) and step overshoot (0.5–0.6° at −0.15 A, the
      feedforward's) unchanged; no oscillation onset. The campaign's feedforward-off pass, where the
      integral carries all the friction: 27 / 23 → 13 / 4 stalls at +0.15 A (recorded, not gated).
    - Not fixed, and not the cap's: with the station's kq 13 servo the knee plant at +0.15 A fails
      the ramp+1 speed gate at every start (1.10–1.17; 6 of 9 passes, cap or no cap) — the slow
      ramp's stuck start again, the stiffness the crosstalk allowance leaves (owner 01:05: keep it).
      Within one run the validation follows the survey by minutes, so the station's ramps passed at
      1.025–1.071; the campaign an hour later is where +0.15 A applies. The retry ruling covers a
      repeated pass; the journals will show the integral at the ramp start.
    - The rock keeps zeroing the integral: at this inertia P alone (0.07 A at 0.3°) cannot restart
      the axis, so the zeroing is what spaces the rocks by one integral rewind (2–3 s); without it a
      jammed axis would rock every 0.3 s. The simulator cannot weigh the rock itself (LuGre carries
      no contact loading); the station says 5 of 5 rocks did not free the payload.
    - Why the model's prediction was 5–10× optimistic: `design.plant()` runs the use case without
      the station's noise terms and with the fit's creep term, so its rest friction is 0.06 A and
      every move finishes. With the noise terms and `creep_drop` 0 the same servo on the fitted plant
      gives step finals ±0.58°, long finals 0.47°, ramp RMS 0.13–0.45°, walker 0.24° — within 1.3× of
      the station on everything but the walker (0.55°: slow-speed stick-slip). Drift adds the stalls
      and the speed ratios, not the accuracy gap. Not changed here (the prediction is advisory).
    - The next MEASURE on #5 must show: `integral_cap` ≈ breakaway + 0.15 (≈ 0.55–0.65) and
      `stall_recovery.current_A` equal to it in the accepted asset; ≤ 2 stalls per validation pass;
      in the journals no rock while `i` is still rising, and after the campaign's positioning moves
      the integral reaching the cap before any rock.
  - **MEASURE 02:04 confirmed the cap (0 stalls, cap 0.78) and failed the slow ramps (1.145 /
    1.210 / 1.124). Study and fix (Fable, 03:30; branch `claude/yaw-stuck-start`): the integral
    belongs to the direction of motion.**
    - On the journals (`ramp_start.py`): before ramp+1 the axis sits 0.43–0.47° short of the
      previous step's target, pulled by only 0.05–0.10 A (its integral, +0.19 A wound the wrong way
      during that step's own slip, had spent the whole rest unwinding). At the ramp start the
      feedforward steps to +0.33 A; the motor moves 0.5° at 2 deg/s under 0.1–0.15 A — the same free
      slip every force reversal shows (step+0.2 travels 0.32°, step+0.5 0.42°, the stall rocks moved
      the axis 0.1–0.24° backwards), so the mount has about 0.5° of lost motion between motor and
      load — then stands 0.4–2.4 s while the integral winds to −0.30 A against the move and the
      reference passes it; P + I need 0.3 A over the 0.33 A feedforward to reach the 0.63 A breakaway.
      On the steady ramps the integral is +0.05…+0.13 A moving + and −0.03…−0.11 A moving −, and at
      the walker's 13 reversals it takes a median 0.9 s (max 1.9 s) to cross to the new direction.
    - Invariant (servo.cpp): the integral holds the friction feedforward's shortfall along the
      commanded motion (yaw carries no load). Two clauses: it reverses sign when the motion
      reverses (direction = sign of v_ref + correction, through hold_speed), and it does not grow
      while the axis stands ahead of a moving reference (the stall detector's displacement
      judgement) — an axis moving ahead still integrates, so an over-estimating feedforward is still
      corrected. Three C++ unit tests beside the servo tests. No new parameter.
    - Proof (`stuck_start.py`; the 02:04 servo kq 13 / ki 36 with the fit's creep term on
      `payload_5x_knee` at −0.15 / 0 / +0.15 A, starts 1.0 / 3.5 / 5.2, two noise draws, and the same
      on a plant with a mount compliance of 0.5° per 0.2 A, which the sim otherwise lacks):

      | plant | servo | ramp ratios | failed passes | worst step final | worst long final | stalls |
      |---|---|---|---|---|---|---|
      | stiff | origin/main | 0.912–1.156 | 4 / 18 (ramp+1 1.14–1.16) | 1.27° | 0.78° | 0 |
      | stiff | friction frame | 0.911–1.056 | 0 / 18 | 0.76° | 0.78° (same cells) | 0 |
      | compliant | origin/main | 0.963–1.096 | 0 / 18 | 0.68° | 0.45° | 0 |
      | compliant | friction frame | 0.966–1.082 | 0 / 18 | 0.56° | 0.45° | 0 |

      Candidates that did not do it: freezing the integral while standing ahead, alone (5 / 9: the
      sim's integral is already wound against before the ramp, so the reversal is what must flip
      it); a higher cap (nothing); `friction_correction_rate` alone (3 / 9).
    - The pre-ramp rest offset (0.4–0.5° ahead) is the hold band plus the loop's lack of
      stiffness doing what they do (kq 13 holds 0.1 A at 0.45°), and it is what leaves the low end
      of the ratios at 0.91. The lever for it is the existing `friction_correction_rate` (friction
      feedforward toward the target outside a deadband of the hold band): at 4.4/s on top of the fix
      the ratios tighten to 0.943–1.041 / 0.971–1.052 and step finals fall to 0.42°, with 0 stalls.
      Not enabled here — it changes every asset's behaviour at rest (a 0.7 A push toward the target
      on the bare turret beyond 0.23°); the owner's call, with those numbers.
    - What the next MEASURE on #5 must show: both passes accepted with ramp±1 inside 0.93–1.07; in
      the journals the integral flipping sign at each ramp start and walker reversal, and no winding
      while the axis stands with the reference approaching (`i` flat until e crosses zero).
  - **MEASURE 03:04 (release 62f1c52): one pass accepted, the other missed by 0.004 (ramp−1 1.104,
    0 stalls). Refinement (Fable, 05:00; branch `claude/yaw-friction-frame-2`): "standing" was the
    wrong test, and the frame must survive a rest.**
    - On the journals (`ramp_start.py`, usecase1): at ramp−1 the motor slips 0.5° ahead at
      2–3 deg/s under 0.1 A, then springs back toward the reference at 1 deg/s; the displacement
      test read both as "moving", so the integral wound +0.02 → +0.26 A against the move in 0.8 s,
      the axis stood until 1.6 s after the reference had passed, lurched, and stuck again at 2.3 s
      (feedforward −0.34 A against a −0.59 A breakaway). At step−2.0 the same slip (9 deg/s, 1.5°
      ahead) and spring-back wound −0.13 → +0.36 A (1.1 A/s at that error); the rest unwound it only
      to +0.28, and the flip at ramp+1 made that −0.26 A against the ramp. The axis then crept at
      0.3–0.5 deg/s on the feedforward alone for a second.
    - Rules now (servo.cpp, no parameter): (1) the frame is the last motion's, through a rest (a
      rest winds toward the target by itself; turning the frame to the target at rest and flipping
      there kicked the axis back and forth across the target — step overshoot 2.9° in the sim);
      (2) "ahead" also requires the reference to still reach the axis — its stopping distance
      v_ref²/2a_ref covers the gap — an axis beyond that has overshot and the integral works toward
      the target (frozen, an overshot step sat 0.6° off for its whole settling tail); (3) while
      ahead, the integral grows only if the feedforward is carrying the axis: advancing along the
      direction faster than stall_speed but no faster than twice the reference. Velocity alone with
      a 0.3 s qualifier was tried: it read the sim's gentle feedforward run-ahead on steps as
      standing and doubled their overshoot. Five C++ tests: reversal, standing ahead, carried ahead,
      slip + spring-back, overshot target, frame through a rest.
    - Sim A/B (03:04 servo kq 14.4 / ki 43 / cap 0.75 on `payload_5x_knee` ±0.15 A, three starts,
      two noise draws):

      | plant | servo | ramp±1 and all ramps | stalls | step final | step overshoot | long final |
      |---|---|---|---|---|---|---|
      | stiff | e89e521 | 0.929–1.066 | 0 | 1.07° | 0.87° | 0.55° |
      | stiff | refined | 0.940–1.046 | 0 | 1.03° | 1.11° | 0.55° |
      | compliant | e89e521 | 0.962–1.075 | 0 | 0.82° | 0.77° | 0.34° |
      | compliant | refined | 0.959–1.071 | 0 | 0.80° | 0.89° | 0.34° |

      The ramps now sit inside 0.93–1.07 on every pass with margin to both gates; the sim cannot
      show the refinement's real gain (it has no free slip), and its one cost is step overshoot in
      a few cells (+0.2–0.5°, conformance, not gated). A 2 deg/s floor on the carried band did not
      move it (1.12°), so it is not the band; left as is.
    - Not changed, still the owner's call: `friction_correction_rate` 4.4/s (rest offsets 0.4–0.5°
      → ~0.3°, ramps 0.94–1.05 → 0.97–1.04 on e89e521's numbers).
    - What the next MEASURE on #5 must show: ramp±1 inside 0.93–1.07 on both passes; in the journals
      `i` flat through the slip and the spring-back at each ramp start (0.1–0.9 s), the flip at the
      ramp start carrying the previous rest's shortfall along the new motion (not against it), and
      the integral winding only once the reference has passed.
  - **MEASURE 04:03 (release 5153d15): the servo rule held (`i` flat +0.06 A through a 2 s stand),
    one pass accepted at 0.957–1.020, the other missed at 0.885 (ramp+1 too slow). A degenerate
    friction fit set the feedforward and the cap (Fable, 06:00; branch `claude/yaw-friction-fit`).**
    - The fit: stribeck 1.53 A against creep 1.25 A (creep speed at its 2.86 deg/s bound, where it
      meets the 4.6 deg/s Stribeck speed) for plateaus of 0.15–0.59 A; model 0.48 A at 1 deg/s against
      a 0.40 A plateau. The ramp+1 feedforward of 0.46 A slipped the motor 1.8° ahead at 6.5 deg/s and
      the axis stood 2 s for the reference. The cap rule read coulomb + stribeck as 1.66 A → 1.2 A.
    - The step overshoots (0.77–1.08° on step+0.2/+0.5) are not the cap: peak |i| in those steps was
      0.28–0.47 A, −0.07…−0.17 A at the moment of peak travel; they happen at +0.4 s with the
      feedforward at 0.44–0.47 A — the free slip again.
    - Fit (`identify.fit_friction`): creep speed ≤ 0.86 deg/s (`CREEP_SPEED_MAX` 0.015 rad/s) against
      a Stribeck speed ≥ 4.6 deg/s, so the exponentials are 5× apart and cannot trade off; and a
      model reading above a plateau costs twice what one reading below does (`OVER_READ_WEIGHT` 2:
      over-reading slips the standing axis ahead, under-reading leaves it behind for the integral).
      Over the seven surveys of the night:

      | survey | + g(1°/s) vs plateau | + static (Fc+Fs) old → new | − g(1°/s) vs plateau | model peak ≤ max plateau + spread |
      |---|---|---|---|---|
      | 21:16 | 0.33 vs 0.33±0.05 | 0.43 → 0.41 | 0.29 vs 0.32 | yes |
      | 23:43 | 0.45 vs 0.43±0.09 | 0.80 → 0.67 | 0.38 vs 0.52±0.14 (bump) | yes |
      | 23:55 | 0.31 vs 0.32 | 0.47 → 0.41 | 0.23 vs 0.25 | yes |
      | 00:38 | 0.33 vs 0.35 | 0.43 → 0.41 | 0.25 vs 0.32 | yes |
      | 02:04 | 0.35 vs 0.35 | 0.63 → 0.56 | 0.25 vs 0.27 | yes |
      | 03:04 | 0.38 vs 0.37 | 0.60 → 0.49 | 0.37 vs 0.50±0.14 (bump) | yes |
      | 04:03 | 0.43 vs 0.40±0.06 | **1.66 → 0.56** | 0.33 vs 0.40 | yes |

      The + side is read within +0.03 A; the − side is under-read where its plateaus bump at 1–2 deg/s
      (a shape the exponential cannot follow). Weighting plateaus by their spread was tried first and
      dropped: at 10–20 deg/s the spread is the loop ringing, not an error of the mean, and the
      down-weighted fit under-read the feedforward at speed — the 5× `--update` case (kq 4.9) then
      stick-slipped at 10 deg/s (ratio 0.884, 4.5° of jitter) with any cap.
    - Cap (`design.servo`): the larger direction's largest plateau (`plateau_max`, now in the asset)
      + `breakaway_over_plateau_A` 0.35 (00:38 needed 0.44–0.69 A over plateaus of 0.36; 02:04 0.63
      over 0.37) + `friction_drift_A` 0.15, ceiling 1.2; stall current = cap. Over the night's surveys:
      0.82–1.11 A (04:03: 1.09 instead of 1.2; 00:38: 0.86 instead of 0.585, above the 0.69 it needed).
      A high cap cost nothing the journals show (the integral never passed 0.47 A under the 1.2 A cap).
    - Honest limit: the 1 deg/s ramp's ratio on this mount is set by the free slip at its start
      (0.5–1.8°) and the rest offset before it (0.4–0.5°), divided by 1 deg/s against the gate's 1 s
      exclusion. The fit fix removes the over-reading feedforward that made the 04:03 slip 1.8°; it does
      not remove the slip. If the next run still misses at 1 deg/s, the morning's choices are
      `friction_correction_rate` (rest offset), a feedforward that waits for the reference while the
      axis stands ahead (a servo change: no force reversal on a standing axis, so no slip), or a slow-ramp
      gate that measures tracking after the start for heavy payloads.
- **Homing that is not a hazard no longer faults (owner, 2026-10-25 01:40: "no fault state if the
  issue is not safety related"; `25ef55a`, probe update `f2ca460`).**
  - The 00:28 FAULT was a repeatability miss: no hazard.
  - Every homing failure is now classified. Hazards still FAULT:
    - a drive fault, a torque abort or invalid feedback;
    - no stop after a full traverse;
    - travel that is not the configured mechanism's.
  - Everything else is not a fault: timeouts, settle, clearance, a fine contact that moved, a
    repeatability miss, a fine pass past the contact it had just observed. These back off the stop
    by the backoff distance, then the whole homing runs again, up to 3 attempts. After the third,
    the station holds unhomed in phase `idle` with the pitch held. Telemetry carries
    `homing_attempt` and `homing_failure`, the page shows HOMING FAILED (amber, with the reason), and
    HOME starts again without a recovery.
  - Deployed at 02:04 with the integral cap. HOME with the payload passed on the first attempt.
- **MEASURE on #5, 02:04–02:25 (release `f2ca460`: calibd's asset fix, homing, integral cap): the
  stalls are gone; NOT ACCEPTED on the slow ramps.**
  - **Pitch ACCEPTED.**
  - **Survey:** + 0.14+0.49 A, − 0.20+0.29 A. Breakaway +0.63 / −0.49 A, about 0.15–0.2 A above
    00:38's, so the drift came between the two runs.
  - **Identification and design:** J 0.2704 (loop delay 2.38 ms). wn 6.9 → kq 13.02, ki 36.15.
    `integral_cap` 0.78 = `stall_recovery.current_A`: the new rule, live. Ladder quiet to 10.6.
  - **Validation:** 0 stalls in both passes, against 1 and 4 at 00:38. But ramp+1 ran at
    1.145 / 1.210 and ramp−1 at 1.124, over the 10% gate. Rests reached 0.45–0.68°. The simulator had
    predicted 1.10–1.17 at +0.15 A.
  - **The stuck start, from pass 2's ramp+1 journal:**
    - The previous segment left the axis 0.47° *ahead* of the reference, with u −0.10 A holding it.
    - The ramp's feedforward step (+0.33 A) let it slip further ahead, to 0.78°.
    - From +0.4 to about +2.4 s it stood. Meanwhile the integral wound against the travel to −0.30 A,
      well short of the 0.78 cap, while the reference reached and passed it. u stayed at 0.03–0.07 A
      until +1.1 s and reached 0.55 A only at +2.4 s, when the axis lurched at 4 deg/s.
    - ramp−1 shows the same pattern, smaller.
    - The cap is not the cause. The root is the integral winding against a reference moving toward
      a standing axis: a conditional-integration change in `axis_control_core/servo.cpp`. Fable is
      proving it in the simulator (the knee plant reproduces 1.10–1.17).
- **MEASURE on #5, 03:04–03:25 (release `62f1c52`: Fable's friction-frame integral, `e89e521`):
  one validation pass accepted, the other missed by 0.004.**
  - **Before the run:** 3 min of AUTO_ROAM on the new servo were clean, with no servo episode and no
    warning.
  - **Pitch ACCEPTED.**
  - **Survey and identification:** + 0.16+0.44 A, − 0.19+0.40 A. J 0.2032; the night's five runs
    span 0.20–0.27.
  - **Design:** wn 8.4 → kq 14.37, ki 43.11. Ladder quiet to 12.9.
  - **Validation:**
    - usecase0 ACCEPTED: 0.952–1.061, 0 stalls.
    - usecase1 NOT ACCEPTED: ramp−1 at 1.104, 0 stalls, slow-ramp P95-5 2.28°.
  - **Two edge cases in the journals:**
    - **Spring-back.** On ramp−1 the axis slipped 0.65° ahead, then sprang back toward the reference
      at about 1 deg/s. The "standing ahead" test judges displacement without sign, so it counted the
      spring-back as moving, and the integral wound against the travel to +0.26 A. The axis then
      stood about 0.8 s after the reference passed.
    - **Flip after a rest.** At ramp+1 the reversal flip turned an integral that a rest had been
      working on (+0.28 A) into a push against the new motion (−0.26 A).
    - Back to Fable: judge "standing" by progress along the travel, and decide what a rest does to
      the integral's frame.
- **MEASURE on #5, 04:03–04:24 (release `5153d15`: Fable's refined friction frame, `3e5fbf2`):
  the servo rule works, a degenerate friction fit failed the run.**
  - **Pitch ACCEPTED.**
  - **Survey plateaus** look like earlier ones: + 0.40–0.44 A and − 0.31–0.46 A between 0.5 and
    5 deg/s.
  - **The fit degenerated:** + 0.13+1.53 A, − 0.21+1.26 A, creep 1.25 A. The Stribeck and creep
    terms trade off against each other.
    - The cap rule read breakaway as Coulomb + Stribeck = 1.66 A, so `integral_cap`, the stall
      current and the rock current all hit their ceilings (1.2 / 1.2 / 0.8 A).
    - The slow-speed feedforward came out at 0.46 A against a measured 0.40.
  - **Design:** J 0.2233, wn 7.8 → kq 13.41, ki 40.24.
  - **Validation:**
    - usecase0 ACCEPTED: 0.957–1.020, 0 stalls, slow-ramp RMS 0.30°, walker 0.37°, all the best of
      the night. But step overshoot 0.79°.
    - usecase1 NOT ACCEPTED: ramp+1 at 0.885 (too slow), overshoot 1.08°.
  - **What happened on usecase1's ramp+1:**
    - The over-estimating feedforward threw the axis ahead at 6.5 deg/s to 1.8° in front.
    - It stood about 2 s while the reference caught up, with the integral held flat (+0.06 A) as
      the new rule intends, then tracked to within 0.1°.
  - **The class:** every design quantity derived from the friction fit inherits its degeneracy:
    the feedforward, the cap, the stall current and the rock. Tonight's fits ranged from
    + 0.08+0.35 to + 0.13+1.53 A for plateaus within 0.1–0.2 A of each other. Back to Fable: bound
    the fit, or fit the plateau envelope, and base the cap on the survey's largest plateau plus the
    drift.
- **MEASURE on #5, 05:00–05:21 (release `362caf5`: Fable's identifiable friction fit, `abf550c`):
  the fit is sane; pass 1 accepted, pass 2 not. The last run of the night.**
  - **Fit:** + 0.14+0.47 A, − 0.21+0.17 A, creep 0.32 A, held at its new 0.86 deg/s bound. Largest
    plateau 0.48 A, so cap = stall current 0.984 A.
  - **Design:** J 0.2231, wn 8.1 → kq 14.57, ki 43.7.
  - **Validation:** usecase0 ACCEPTED (0.934–1.068, 0 stalls). usecase1 NOT ACCEPTED: ramp+1 at
    1.231.
  - **What happened on usecase1's ramp+1:**
    - The reference was still moving − just before the ramp, and the integral (+0.18 A) was settling
      that move.
    - The friction frame's reversal flip turned it into −0.17 A against the new + motion.
    - The axis stood, fell 0.33° behind, then caught up. The flip's assumption (the integral is
      friction shortfall along the motion) fails whenever the integral carries something else.
  - **The night's pattern since the friction frame (03:04, 04:03, 05:00):**
    - Pass 1 accepted every time (0.93–1.07). Pass 2 failed every time, on one ±1 deg/s ramp, in
      either direction (1.104, 0.885, 1.231).
    - 0 stalls in every pass since the integral-cap rule.
    - Both passes start at the same angle (about 100.5° mod 360); pass 2 follows pass 1 directly.
  - **Left for the owner:** how to make the 1 deg/s ramps robust on this mount. Fable is analysing
    the journals for: why pass 2, whether the flip is a net gain, and one recommendation among
    `friction_correction_rate`, a feedforward that waits while the axis stands ahead, a slow-ramp
    gate that judges tracking after the start for heavy payloads, or reverting the flip.
