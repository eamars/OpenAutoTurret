# Operate and adapt the camera station

Current operating runbook, **1 October 2026**. Read this before deploying,
starting, stopping or diagnosing the station. Dated run reports are historical.

> **How to read this file.** It holds the station's **current state, safety history, owner rulings
> and measured limits** — the reasons. Procedure lives in [`operations/`](operations/README.md), one
> card per operation, one hop from [`README.md`](README.md); start there when you are about to *do*
> something, and come here to understand why the card says what it says. Where the two disagree on a
> procedure, the card is current; where they disagree on a fact or a limit, this file is.
>
> The state recorded below is a **record taken on 2026-09-28**, not a live reading: the station has
> since moved release and address. Before acting on any line here, take a live reading —
> `run_application.sh status` for the release and run dir, and
> [`tools/station_address.sh`](../tools/station_address.sh) `print` for the address.

## Fault, hold, degrade: when the station may stop itself (owner ruling, 2026-10-02)

**Read this before adding, keeping or tuning any guard, trip, watchdog or threshold.** It came from
two field faults on 2026-10-02 (18:33 and 19:10): the yaw servo's oscillation guard tripped 50 ms
into a limit cycle that the owner, standing in front of the turret, could not see; the station
faulted and de-energised yaw while it was tracking him well.

The turret is an **industrial device**. It is deployed outdoors and indoors, in dust and rain, on a
cross-roller yaw bearing whose friction can change anywhere and at any time, under loads that are
not balanced. It is not a laboratory instrument, and a guard that trips without margin is a defect.

**De-energising is itself a hazard.** With an unbalanced load, a released axis falls or swings
free. "Fault → power off" is the dangerous direction, not the safe one.

### The three responses

| Response | When | What the motors do | How it ends |
|---|---|---|---|
| **FAULT** | Only for a **safety hazard**: harm to people, the machine or its surroundings if motion continued. | A controlled stop, then **hold position, energised**. An axis is released only when it cannot be held: its drive has itself faulted, or there is no way left to command it. | An operator recovers it. |
| **HOLD** | A **persistent** failure that is not a hazard. Examples: video lost for good, a servo still oscillating or stalled after its persistence time, repeated control-deadline misses. | Stop the motion (braked, as the supervisor's HOLD already does) and hold position, energised. | **By itself**, once the cause has cleared. |
| **DEGRADE** | A **transient**: a brief limit cycle, a short stall and its rock, a friction excursion, a single deadline overrun, a short frame gap. | Keep operating; log it with numbers and count it. | It clears itself; nothing to recover. |

### Rules for every guard

1. **Classify first.** Before writing a trip, say which row it belongs to and why. "Something
   unusual" is not a hazard.
2. **Margin and time.** A threshold sits above the worst *normal* operating value measured on the
   station, not at a commissioning-session limit. It also has a persistence time scaled to the harm:
   - Immediate (milliseconds) only where milliseconds matter, such as a runaway above the 100 RPM
     cap, or the end-stop guard on pitch.
   - Performance conditions, such as a stall or oscillation, are failures only after they persist,
     about **5 s**, the owner's own figure. Below that they are DEGRADE.
3. **Prefer holding to releasing.** A drive that can hold on its own encoder is told to hold speed
   zero, for example a CyberGear in speed mode. A dead-man that fires because *this host* stalled
   must leave the axes held by their drives, not released.
4. **Robustness is designed in, not re-commissioned in.** Friction drift (±35 % within an hour was
   measured on yaw) is a normal operating condition. A loop that only behaves at this morning's
   friction is not finished. Re-commissioning is not the answer to environmental change.
5. **Commissioning sessions are different.** A bounded commissiond session, run by the tool with
   nobody relying on the turret, may hard-abort quickly, because its job is to find limits. Those
   abort thresholds must not be copied into production.

### The guards in production, against this rule (2026-10-02)

Every row except the ones marked **open** was changed on 2026-10-02, with tests. The **open** rows
still violate the rule and are the next work. Do not cite them as precedent.

| Guard | Where | Was | Now |
|---|---|---|---|
| Yaw servo oscillation (fast RMS > 0.3 A of the current the servo did not plan: output minus feedforward) | `MixedCanMotorBackend` | FAULT after 50 ms, yaw de-energised | DEGRADE: logged per episode, with its peak. After 5 s continuously, HOLD until 1 s quiet (`EpisodeLatch`). Until 21:40 it watched the whole current, and a hard stop's own deceleration current (0.3 A) counted. |
| Yaw servo stall (stall recovery still rocking) | same | — | Rocking itself is DEGRADE. Still stuck after 5 s, HOLD until it clears. |
| Watchdog fault, pitch | `ControlLoop` (watchdog handling) | pitch de-energised for any pitch fault | Released only if the drive itself reports a fault (`fault_releases_axis`). Otherwise the Fault phase brakes and holds it. |
| CyberGear dead-man (host heartbeat > 100 ms, feedback > 100 ms, > 75 °C) | `CyberGearSystem` watchdog | STOP (release) to both drives every 5 ms | `SpdRef = 0` to an enabled, healthy drive, which holds on its own encoder. STOP only to a faulted or not-enabled drive. Owner to confirm the > 75 °C case: holding heats the motor, but releasing drops the load. |
| GM6020 encoder unwrap: one implausible reading, or a gap over 80 ms | `UnwrappedEncoder` | latched invalid for the session, then `feedback_unsafe` and a permanent FAULT. Station, 20:02:50: two frames bunched 3 µs apart in the receive queue | Production's `Recover` policy:<br>• the 1 ms device period is the floor on the time between readings;<br>• a doubtful reading is skipped, and a run of 20 is believed;<br>• a gap re-establishes the turn by nearest count (yaw is continuous).<br>Commissioning keeps `Latch`. |
| Guard: feedback unsafe, bus down or wrong, tx failing for 20 ms, heartbeat stale, motor heat | yaw guard thread, servo step, legacy command path | FAULT at once (two paths tripped directly) | The command paths refuse motion (zero current) and never trip. The guard faults only after 0.5 s without a break (`GuardFaultPersistence`). Until then it holds: a blind axis coasts on zero current, otherwise the servo keeps holding. |
| Observer refuses an encoder reading (`servo_encoder_rejected`) | yaw servo | FAULT | Released (zero current); it re-engages from the measured state on the next reference. |
| Following error > 15° (yaw), and the pitch following error | both servos | FAULT | HOLD. The servo lets go (yaw zero current; pitch drive at speed zero). The supervisor's stop starts at the measured axis (`kStopReanchorRad`), so nothing pushes toward the old reference. It clears 1 s later, once the reference is back within 0.05 rad of the axis. |
| Pitch servo feedback older than 20 ms | pitch servo | pitch servo fault | The servo lets go to the drive's speed-zero hold and re-engages when feedback is fresh. A fault only after 0.5 s without a break. A disabled or self-faulted drive is still a fault at once (it isn't holding). |
| Faults during homing, mode transitions, recovery failure | `ControlLoop` | `deenergize_all()`: release both axes | `stop_axes_safely()`: release only what `fault_releases_axis` says cannot be held. The pitch drive holds speed zero (`hold_axis`). `deenergize_all()` is kept for operator stop and shutdown only (next row). Motor recovery's own disable, needed to clear drive faults, is unchanged. |
| Operator stop and shutdown | launcher → `controld` | pitch STOP at the end | **open, owner decision:** park first so the release is safe, or hold. |
| Motor over-temperature (supervisor), drive-reported fault | supervisor | FaultStop / Disable | Consistent: a hazard, and a faulted drive is not holding anyway. |
| 100 RPM speed cap, pitch end-stop guard | servos | FAULT | Consistent: hazards, immediate. The pitch drive is now held at speed zero, not released. |
| BNO085 IMU drops off I2C (a known BNO085 behaviour; station 2026-10-03 14:27:32) | `imu-bno085`, launcher | One recovery, then exit; the launcher waited on it and stopped the **whole stack** | Fixed 2026-10-03: observe-only capture reconnects for as long as it runs (backoff 0.1 to 5 s); the launcher no longer waits on it. The HUD shows the IMU stale until it returns. Commissioning captures keep their single recovery. |

## The web MENU: Home, Park, Shutdown (owner ruling, 2026-10-03)

Before this, almost nothing in the MENU worked on this station. Pressing Park at 01:43:33 faulted the
station within 40 ms (`velocity_loop_invalid`), and from there Recover Motors answered "unsupported"
and Home was refused "system faulted". Only a process restart got it back. The menu now has three
actions. Each one works from wherever it is offered, and each asks for two presses:

| Action | What it does | Offered from | Ends in |
|---|---|---|---|
| **HOME** | Recovers whatever latched (yaw guard trip, pitch watchdog inhibit, pitch servo fault, a drive fault the drive itself has cleared), proves fresh, healthy feedback on both axes, then runs homing. | Any state except while homing or recovering already | The ready pose, then **AUTO ROAM**, as at power-up |
| **PARK** | Yaw to 0 by the nearest whole turn. Pitch first goes to a pose about 6° inside its soft limit, then onto its **rest stop** at 3°/s (homing's fine-approach speed). Both axes are then held there, energised. | A homed turret | phase `parked`. **Any MODE leaves it**; MANUAL returns to the ready pose. |
| **SHUTDOWN** | PARK, then both motors off. If the pitch stopped short of the stop, it touches the stop again first, so the payload is released resting on it. | A homed or parked turret | phase `idle`, motors off. **Only HOME** starts it again. |

- **The rest stop** is the camera-up end. That is the raw pitch **minimum**, measured by homing at
  −1.511 rad on 2026-10-03 (`shutdown.pitch_rest_end: min` in `turret_mixed.yaml`).
- **Touching the stop.** It counts as touched when the pitch stalls within 2° of the measured stop
  for 200 ms. In the parked hold, the pitch is pulled back if it drifts off the stop and is never
  pushed past where it touched.
- **What isn't a fault.** If the pitch stalls more than 2° short of the stop, goes past the measured
  stop, or runs out of time, the park holds where it is and says so (rule 2 above). STOP MOTION
  during the move gives an ordinary MANUAL hold in place. STOP MOTION on the stop holds there.
- **Live progress** is in telemetry `rest_park` (`moving`, `touching`, `parked`, `releasing`) and
  `rest_park_on_stop`.
- **The process-exit stop is unchanged.** `run_application.sh stop` still runs the older
  `start_parking` path. The `shutdown.*_park_*` keys now serve only that path.
- **Recover-only.** `recover_motors` is still accepted by controld, but it is no longer in the menu:
  HOME does the recovery itself.
- **The Park fault itself** was a clock-ordering bug in `MixedCanMotorBackend`. A legacy speed or
  position command sampled the time, then released the engaged ADR-003 yaw servo, which restarted
  the legacy loop at a later clock reading. The loop's first step therefore saw time run backwards
  and latched the trip. The time is now read after the release. The 2026-10-02 17:05:52 fault quoted
  in `control_loop.cpp` ("the legacy speed loop could not take it over") was most likely the same
  bug.
- **Recovering the pitch.** A drive that reports its own fault is cleared with the CyberGear fault
  clear, which is a STOP. A drive that is still holding speed zero is re-armed without being
  released (`CyberGearSystem::finish_axis_recovery`).

## Two states, Homed or Shutdown, and the boot (owner ruling, 2026-10-03)

**Ruling.** The station is either **Homed** (energised, AUTO_ROAM / tracking / MANUAL) or **Shutdown**
(both motors off, not homed, web and camera up). There is no third state.

- **A boot is Shutdown**, the way a printer's firmware comes up and waits for a home. MENU > HOME is
  what brings the unit up.
- **A deploy returns the station to the state it found.** `tools/station_state.py` decides: Homed
  needs a reachable controller with valid limits that is neither idle nor faulted.
- **A plain launcher `start` is Shutdown**; `--home` homes.

The boot needs one sudo step, once (`station_os_setup.sh --apply --autostart`). After it, every
deploy maintains the boot service by itself; see the [OS setup card](operations/os-setup.md),
"Start at boot".

Why the launcher no longer insists on the IMU: on 2026-10-03 at 16:35 the BNO085 held SDA low (the
known I2C fault, next to an under-voltage event). The launcher's mixed-station gate, which demanded a
fresh IMU tare, kept the station down after a deploy. In normal operation that wait now times out
into a start without the IMU; commissioning keeps the gate.

## The web page: less on screen, settings in MENU, stats on request (owner ruling, 2026-10-03)

**Ruling.** The page should inform without overwhelming:
- **Status bar.** The six health chips moved from the top right into the bottom bar, folded behind
  one summary (`ALL OK`, or `N ALERTS`).
  - A chip that is not healthy is never folded away.
  - MODE, STATE and SAFETY left the bar, because the mode block and the safety banner already say
    them.
- **Mode controls.** The Manual/Hold and Auto buttons and the MODE drawer stay as they were.
- **MENU > SETTINGS** holds:
  - The live speeds: patrol on wide, patrol on detail, and tracking. They use controld's
    `set_speed` and are bounded by each mode's maximum. They last **this session only**: a restart
    returns to `turret_mixed.yaml`, so a trial speed cannot quietly become the deployment's speed.
    The DPAD's COARSE and FINE paces are the two patrol paces, so they follow.
  - A placeholder for the target selection policy. Today the only policy is perception's
    `AUTO_SELECT_SINGLE`: one person, alone for 0.5 s.
  - The **STATS FOR NERDS** switch.
- **The stats overlay** is off by default and remembered per browser. It replaced both the DIAG
  drawer and the old `/dashboard` page, which was removed.
  - Every dashboard field was audited before the move. These were dropped:
    - the installation pose, which is always identity here
    - payload verification status, which is refused on the mixed backend
    - the GM6020 "effort", which is always null
    - v1 tracking state
    - the video caption, which reported config defaults rather than the served size
  - controld now publishes both CAN buses (`can_buses`; can1 had been invisible), the motor
    temperatures against the supervisor's trip, and the event history, which no page showed before.

**What became UI-unreachable.** Payload profile selection, `run_test_motion`, and the dead v1
controls: start/stop tracking, enable/disable search, visual calibration (always refused), and
payload verification (refused on mixed). All of them remain on `/api/command` and
`tools/station_ipc.py cmd`.

## Scheduling: what runs real time (owner ruling, 2026-10-03)

**Ruling.** Motor control comes first and the web UI last. Real time is given **per thread, only to
motor control**, never to a whole process, and work fused into a real-time path is split out rather
than promoted with it. Target tracking keeps the normal class unless a measurement shows it needs
more. The procedure, and the table of every thread's class, is the
[OS setup card](operations/os-setup.md).

**Why: measured on 2026-10-03, 15:22-16:15, release 077a330, before any of this.**

- Everything ran SCHED_OTHER at nice 0, unpinned. Load average 4.6-5.7 on 4 CPUs. Every device
  interrupt was on CPU 0. Memory was fine (6.3 GB available, no swap) and nothing was throttling at
  the time (59 °C, 2.4 GHz), though `throttled=0x50000` records past under-voltage from the 5 V rail
  through the slip ring.
- CPU: visiond 116% (one Python thread 55-65% of a core), webd 10.6%, controld 8.5%, the two SPI
  CAN controllers' kernel IRQ threads about 15% (about 1000 frames/s per bus).
- The 200 Hz control loop overran its 5 ms period by more than 2 ms **412 times in 52 minutes**,
  each a supervisor `DERATE 'control-loop cycle overrun'`. Worst cycle 12.5 ms; five in a row is a
  HOLD. Every cycle was also 55-60 µs late from the default 50 µs timer slack of a SCHED_OTHER sleep.
- controld's threads waited for a CPU longer than they ran: over 10 s, the GM6020 RX thread (on
  which the yaw servo steps) ran 239 ms and waited 322 ms in the run queue.
- visiond: 45-60 ms per frame, of which 26-29 ms was track association (the retired-track leak,
  fixed in 257744d), so 17.5 Hz. Frames then waited 65-100 ms in the camera queue; camera to
  controld took 127-151 ms.
- visiond learns the operating mode by polling the web UI's `/api/state` at 10 Hz (a new HTTP
  connection each time). Lowering the UI's priority would therefore have slowed perception; that
  path must go to controld directly.

**What follows from it.**

- controld's motor threads ask for SCHED_FIFO 44-48 when the launcher passes `OTA_RT=1`. That is
  below the kernel's CAN IRQ threads at 50, which feed them.
- controld's own web, IMU-observer and log threads are niced to 10.
- The launcher pins controld to CPU 3, and keeps visiond (nice 0), the IMU (nice 10) and webd
  (nice 19) on CPUs 0-2.
- The OS grant (limits.d: rtprio 49, unlimited memlock) is the one sudo step, and it is the
  owner's.

**Measured after each step (2026-10-03, same station, AUTO_ROAM/AUTO_TRACK with the owner in view).**

| Release | Overruns (DERATE) | Loop period p99 / worst | Notes |
|---|---|---|---|
| 077a330, before | 412 in 52 min (~475/h) | 5.67 / 12.5 ms | visiond 17.9 Hz at 116% CPU |
| 257744d, retired-track fix | 18 in 5 min (~216/h) | 5.49 / 9.0 ms | visiond 29.8 Hz at 84%; camera to controld 53 ms |
| 5676fbb, CPU split + nice, no FIFO | 1 in 6 min | 5.02 / 7.8 ms | step work p99 0.33 ms |
| 4705bb2 + OS grant, SCHED_FIFO | 0 in 11.5 min | 5.015 / 5.36 ms | step work p99 0.20 ms, worst 0.31 ms |

With SCHED_FIFO the motor threads' run-queue wait fell to 0-2 µs on average: over 10 s, `rx-can0`
ran 131 ms and waited 0 ms, where before it ran 239 ms and waited 322 ms. Load average is 3.1-3.3.
The loop's own work is under a tenth of its period, so splitting it further is not needed for
timing.

## ADR-003 camera tracking: ownership and the accuracy ruling (2026-10-02, local date)

- **Ownership.** The owner handed ADR-003 to the agent, with the architect's package as guidance.
  The plan, the decisions and the state of each stage are in
  [ADR-003/IMPLEMENTATION.md](ADR-003/IMPLEMENTATION.md).
- **Accuracy ruling.** The yaw accuracy limits are calibrated from the real tracking performance
  of the feedforward + feedback servo. This replaces the photography template's "do not infer
  tolerances from achieved performance" for this station.
  - **Result:** [`config/servo/yaw_accuracy.json`](../config/servo/yaw_accuracy.json), from
    calibration run `yaw-20261002-134637`: 4 angles, plus the asset's 2 validation passes.
  - **Limits:** pointing at rest 0.44 deg, ramp RMS 0.19 deg, walking profile 0.26 deg, moving
    peak 0.79 deg. That is 10.7, 4.6, 6.3 and 19.2 px in the 1920x1080 tracker frame.
  - **Use:** later yaw commissioning reports its conformance to these limits, and they are the
    servo share of ADR-003's framing budget
    ([`photography_spec.json`](../config/tracking/photography_spec.json)).
- **Pitch prefers undershoot (owner ruling, 2026-10-02 22:00).** The 16:9 frame gives pitch
  ±20° against yaw's ±35°, and the camera turns with pitch: running past a subject who stops or
  turns back moves the aim point out of the picture. Pitch is tuned less aggressive and biased to
  undershoot; a little overshoot is acceptable, chasing is not. In the tracking asset's Level 1
  (`config/tracking/tracking_prior.json`, provenance `level1_pitch`): a 3° dead band (head motion
  inside it moves nothing; beyond it only the excess counts), velocity feedforward × 0.7, λ 2.5,
  jerk 1500°/s³ (acceleration stays 60°/s²; end-stop braking is unchanged because the boundary
  governor uses the supervisor's 300°/s³). Proof: the recorded pitch goal of sessions human-3
  and human-4 replayed through `Level1Generator`. Passes beyond the target fell from 11/17 to 0/0,
  pitch travel by 75%, and the frame-error p95 from 4.1/7.2° to 3.6/6.0°. The stage-1 scenarios
  hold pitch to its band, not to the pixel noise. Yaw is unchanged.
  - **22:25, the band alone never converged** (owner: "at certain height the aim never
    converges"): pitch sat more than 1° off a still subject 92% of the time. A move still stops
    short. An offset whose 1 s average exceeds the 0.5° centre band is then closed at no more than
    3°/s (slow enough never to pass the subject), until within 0.25°. Replayed on human-3/4/5:
    more than 1° off while still went from 37/82/92% to 0.5/3.3/0%; passes 0/2 (2.5°)/0. The
    band-only replay of human-5 matches its recording.
- **Station facts for ADR-003 (2026-10-02).**
  - The camera timestamp clock (libcamera's CLOCK_BOOTTIME) equals CLOCK_MONOTONIC to within 1.2
    µs; there has been no suspend since boot.
  - The installed stack is libcamera 0.7.2+rpt20260817 with picamera2 0.3.37.
  - Production still runs the pre-ADR-002.2 yaw velocity PI and the stacked 140 ms lead; nothing
    of ADR-003 is in production yet.

## Servo takeover: owner rulings and measured facts (2026-10-02, local date)

**Owner rulings (2026-10-02).** The working product comes first: probe on the real station, make it
work, then test and harden. ADR-002.x architect documents are guidance only (the architect has no
station access); the model must be adjusted from real responses. Close ADR-002.x before ADR-003, with
limits that serve ADR-003's framing use. Full station authority with yaw/pitch speed below 100 RPM;
high acceleration at low speed is acceptable. Production stack stays off until ADR-003 starts. Pitch
control mode: best measured performance. Repeat tests: the cross-roller bearings are inconsistent.

**Measured facts (station, 2026-10-02).** Procedure: [servo commissioning card](operations/servo-commissioning.md);
evidence and numbers: [takeover report](ADR-002.2/reports/SERVO_TAKEOVER_2026-10-02.md).

- GM6020 angle feedback has **current crosstalk**: reading = angle + g(angle)·i(t−2.1 ms), g periodic
  (nine cycles per revolution, up to ±6.4 mrad/A). Uncompensated it caused every 14–17 Hz yaw limit
  cycle at stiff gains. Calibrated table: [`config/servo/yaw_servo.json`](../config/servo/yaw_servo.json).
- Yaw: inertia 0.03 A·s²/rad (open-loop excitation and chirp agree), loop delay ≈2 ms. Kinetic
  friction ≈0.4–0.5 A at 20 deg/s, ≈0.3–0.4 A at 60 deg/s; breakaway after loaded rest can exceed
  0.9 A; slowly rising force produces creep-and-restick, while a brief force reversal releases it.
  Friction rises through a cold night and falls after a warm-up rotation. A localized bump near
  240–255 deg absolute. The BNO085 gyro lags the encoder by ≈100 ms (not fused into the servo).
- **Yaw current authority (owner ruling, 2026-10-02 later the same day):** up to 3 A peak (the CAN
  protocol's full scale) and 1.62 A continuous (the GM6020 rating); 0.8 A is thermal *guidance* only,
  not a limit. The servo enforces the peak and an RMS budget (≤1.62 A over 3 s); the 55 C motor
  temperature trip is the thermal protection. Commissioning reports when sliding friction exceeds
  the 0.8 A guidance.
- **Commissioning is automatic** ([card](operations/servo-commissioning.md)): one command per axis
  identifies the plant from a prior that assumes nothing, designs the gains with the simulator and
  verifies them on the station. Measured by it on 2026-10-02: yaw inertia ≈0.04–0.047 A·s²/rad (the
  0.03 above was an early hand estimate), crosstalk delay 1.18 ms, sliding friction 0.17–0.59 A at
  0.25–2 deg/s falling to ≈0.3 A at 40–65 deg/s.
- Pitch endstops (production homing logs, 2026-09-30, MechPos): A = −0.115 rad, B = −1.511 rad
  (≈80 deg, not 60), midpoint −0.813 rad. Pitch rests unpowered where left. As-found native settings
  on 2026-10-02: RunMode 3, LimitCur 5 A, SpdKp 4, SpdKi 0.05; after these sessions the drive is left
  in RunMode 2 (production rewrites its own mode at start).
- Commissioning releases under `run/releases/`: 92 on 2026-10-02 13:00 (19 GB with journals, 86 GB
  free): the overnight `claude-*`/`yaw-*`/`pitch-*` ones and one per automatic run (`yaw-2026*`,
  `pitch-2026*`). They are evidence and may be pruned by the owner. Production checkout and stack
  untouched and stopped; station idle, both drives disabled, pitch parked at its window centre.
- **Decoupling (measured):** the unpowered idle axis holds by its own friction. The pitch moved at
  most 0.04 deg during 36 yaw sessions, the yaw 0.044 deg during pitch sessions. Commission pitch
  first; it parks at its centre.
- **commissiond's own pitch homing is not station-qualified.** Its torque-off rearm drops the loaded
  pitch; commissioning uses production's homed window instead (see the card).

## ADR-002.2 architect review priority (2026-10-01, local date)

The owner instructed the agent to continue under [architect review 02](ADR-002.2/architect_review_02/00_START_HERE.md)
and the [estimator recovery amendment](ADR-002.2/docs/09_ESTIMATOR_RECOVERY.md).
The earlier [review 01](ADR-002.2/architect_review_01/ADR-002.2-independent-review.md)
and [identification amendment](ADR-002.2/docs/08_IDENTIFICATION_REPAIR.md) remain history.
Yaw is UNQUALIFIED; Candidate14 is NONDEPLOYABLE and was never physically run.
Continue estimator repair, whole-run comparison and synthetic control verification
while physical promotion is gated. A predictively accepted physical model and the
applicable authorization are required before deployable gain synthesis or another
physical controller trial. Review 02 authorizes no new motion, setting change or deployment.
No further architect approval is required for this authorized offline work.

The owner confirms that the GM6020 uses CAN current control; PWM diagnosis is excluded.
Motor-specific firmware/applied settings, command scaling and the setup's physical
stop/fault behaviour still require supported evidence before physical qualification.
Existing limits and presence rulings below remain. Review 02 work has not contacted
the station; historical ending-state records are not live status.

## ADR-002.2 unattended startup ruling (2026-10-01, local date)

**Retired by the owner on 2026-10-02:** a new tripod lowered the centre of mass (the station itself is unchanged)
and the turret is much more robust, so the 30 degrees/s² yaw acceleration/braking limit below no longer applies; yaw is
back to 60 degrees/s² (`config/turret_mixed.yaml`). Kept as history.

Latest owner override: acceleration is guidance. Marginal exceedances and high IMU acceleration observations may pass and must not distract from ADR-002.2 tuning. Retain 30-degree/s² reference shaping and raw evidence, but disable the measured-current acceleration restriction for tuning when it blocks useful yaw drive. Keep true current, thermal, fresh-feedback and unsafe STOP protection. Higher-acceleration reference cases remain deferred until presence; incidental measured excursions do not block the present tuning path.

The owner reports that nobody will be near the station and sets **30 degrees/s²** for both yaw acceleration and deceleration, including startup assistance and controlled braking. Use measured IMU acceleration, gyro and orientation to assess uneven vibration. The owner reports the pitch platform is balanced and higher-RPM steady rotation is stable; do not introduce a new speed cap for this request. Higher-acceleration hardware tests wait until the owner confirms presence around 18:00 local. Continue deterministic automatic ADR-002.2 tuning across low/high acceleration and low/high speed, with higher-acceleration physical cases deferred. Preserve current, thermal, fresh-feedback and unsafe STOP protection. Before another physical run, verify the acceleration-limiting path and record the actual response. Reference shaping alone does not certify actual body acceleration or vibration. Candidate11 ended at 2026-09-30 23:28:57 UTC; the 23:29:13 UTC query found no output owners. Its sampled acceleration and motion failures remain recorded, and the observed limiter arbitration is being repaired before more motion. This is a recorded state, not a replacement for a fresh ownership check.

## ADR-002.2 只读盘点记录（2026-09-30，本地日期）

2026-09-30最新主人指令：后续阶段2不生成或检查hash，不以agent新增的噪声、编码器
差异或预期行程门槛拒绝标定观测；采用已有滤波并保留原始反馈。只做必要正常路径
开发检查，直接开展实机标定。既有真实保护、有限会话及唯一输出owner保持有效。

主人已授权阶段2，并确认无payload、pitch掉电保持原位、pitch总行程约60度且归零居中、
yaw滑环无限转动；电流按电机datasheet，不另加人为上限，采集时记录电机温度。
这些是主人提供的条件，不是当前模式/端点/停车资格的实测证明。

一次只读SSH盘点已留证。原盘点中的`ps --ww`参数错误和可选目录列表错误已被发现；
修正仅在本地验证。主人最新要求继续Step 2，不以此前失败作为门槛；置信百分比不是测试数推算的概率。
主人另确认只有手动电源切断；未确认独立自动断电或电调通信丢失停车能力。
2026-09-30 02:39:55–02:42:00 UTC，独立`c80f84a38e55.J5AR2G`采集release完成一次120秒baseline：
yaw 120001帧、pitch STOP确认5806次、寄存器读回5805次，socket和接口丢包增量均0。
未enable、未写mode、未发激励；production未启动。pitch实际读回mode=2、Iqf可读；
gyro约50.09Hz且accuracy=0；pitch温度22.6°C、yaw温度原始字节28（单位未标定）。
原始capture中的legacy yaw安培换算字段未获资格，离线分析只使用原始电流字节；修正只在本地验证。
后续只读检查未见controller/IMU进程，retained homing缓存不存在；运动前仍须既有homing流程。
已有日志显示最后一次yaw停车确认超时；不把后续`stopped cleanly`进程清理日志当作停车合格。
完整状态及证据见[阶段2准备情况](ADR-002.2/reports/STAGE2_READINESS.md)
（盘点工具与操作卡已于2026-10-02随ADR-002.x收尾删除，见git历史）。以下既有状态记录均需按其时间理解。

## 现状刷新（09-29 深夜，现读，非历史）

- **release**：`139551b5099c.psS9nL`（`run/releases/` 下只留这一个；清理前有 106 个、20 GB）。
- **模式**：`MANUAL / READY`（主人有意 park：yaw 无法稳定追踪，属 ADR-002；每次受控重启后会回到
  AUTO_ROAM，**要有人放回去**，`POST /api/command {"command":"set_mode","arg":"MANUAL"}`）。
- **双流**：`wide cam-baa28c2a by-path 1920x1080` / `detail cam-68510500 fwnode 1280x720`，两路
  delivered ~9 fps；第二路的**开关在 `perception_v1.json` 的 `vision.secondary`**（不是环境变量）。
  **2026-10-02 主人裁决**：只有**主画面**那一路进 Hailo，PIP 只供显示；HUD 的 swap 改的是站点状态
  （`POST /api/camera/main {"role":"wide|detail"}`，visiond 持有，`inference_health.json` 的 `main_camera` 发布），
  每次启动回到 wide。窄角的框以 1/`view_scale`（5.90）的居中窗口发布在广角坐标系里，controld 不变。
  设计与理由见 [工单末节](ADR-001/DUAL_STREAM_DISPLAY_WORK_ORDER.md)。
- **两颗都倒装**：朝向走**同一个传感器级 transform**，值各自配（wide 来自 `config/camera_install.yaml`，
  detail 来自 `vision.secondary.orientation`）；visiond 启动行会打印它**实际用了哪个**。
- **IMU**：BNO085 实测 ~216 Hz；controld 自己填 §20 的 `imu.present/gravity_valid`；
  `world_elevation_deg` 仍是 **null**——传感器装在俯仰组件上、无安装标定，**0.0 会谎称炮塔是平的**。

## Current deployment gate

**Current station state:** release `f8bcdb6` is running in operator-selected
MANUAL/HOLD after bounded D-pad tests. The owner raised the Pi input to 5.25 V
after release `8901808` reported active undervoltage/throttling (`0x50005`).
The subsequent normal IMX500/CAN/IMU run lasted about 5.5 minutes in
AUTO_ROAM/AUTO_TRACK with repeated `get_throttled=0x0`; a 60-frame IMX477/Hailo
camera-only run and a release build beside the active stack also returned 0x0.
PMIC EXT5V samples under these loads were about 4.87–5.11 V. This clears the
observed current power fault for those loads, but simultaneous dual-camera +
Hailo + motor peaks remain unmeasured. See [Raspberry Pi's bit definitions](https://www.raspberrypi.com/documentation/usage/raspberry-pi-os/raspberry-pi.html#get_throttled).

The latest full startup completed pitch homing and reached READY after the
continuous-yaw *pitch-homing-only* displacement tolerance was changed from
0.5° to 2°. An earlier attempt faulted at 0.527° yaw drift with fresh CAN
feedback; its launcher stop sent pitch STOP/yaw zero but could not confirm the
normal stopped state because the controller was already faulted. Preserve that
case for stop-path qualification. The latest manual yaw tests moved about 7.5°
in six seconds and 17.3° in twelve seconds, with no fault. A pitch manual
out/return test reached 15.63° above its initial pose after release at about
12° and transiently overshot 3.3° past its initial pose on return, then settled
within about 0.6°. Do not interpret working D-pad motion as pitch overshoot
qualification; see the [architect handoff](archive/partially-implemented/handoffs/ARCHITECTURE_HANDOFF_2026_09_27.md).

**The normal launcher selects the mixed split-bus profile. On release `cae41d0`,
two controlled stops succeeded after motion: one from AUTO_TRACK near +46° yaw,
and one from AUTO_ROAM near +76° yaw. Both confirmed fresh pitch-disabled
feedback and issued the final GM6020 zero request; GM6020 disable state remains
unavailable. These two observations do not complete stop qualification. An
intermittent feedback-readiness rejection seen on the previous release has not
yet been shown eliminated. Normal pitch homing completed, but repeated encoder
speed-ceiling/corridor warnings did not abort with
`homing.motion_checks_abort: false`; that guard behavior also remains
unqualified.**

**Current release:** `f8bcdb6` passed committed-source probe build and mixed
preflight, then was started through the launcher and reached READY. Its full
regression suite was deferred by `--probe-build`. The 13 targeted manual
controller tests and the isolated launcher lifecycle test passed; the
commissioning-ownership test cannot acquire the global station lock while the
live stack owns it. Release `43193dc` passed the
earlier 77-test suite, before these control changes. Current D-pad evidence
comes from the web command API and controller feedback, not a browser pointer
event trace. The live API reports valid pitch limits and a session-relative
±80° yaw operating sector; CAN errors are zero and BNO085 observation is fresh.

The installation has GM6020 yaw on `can0`, CyberGear pitch on `can1`, continuous
yaw without an endstop, IMX500 + IMX477 cameras, a PCIe Hailo device, and a
BNO085 on I2C. See [verified hardware and probes](archive/partially-implemented/hardware/HARDWARE_CURRENT.md).

The mixed runtime uses GM6020 yaw on CAN0 and CyberGear pitch on CAN1, with
pitch limited to 5 A. Yaw has no confirmed disable state; a zero request is not
proof of motor de-energization. The September 27 run confirms the mixed control,
perception and web path operated together; two subsequent controlled stops
passed the observed pitch-disable/yaw-zero checks, while broader stop
qualification and the intermittent readiness-rejection question remain open.
See the [hardware adaptation plan](archive/partially-implemented/hardware/HARDWARE_ADAPTATION_PLAN.md).

The project venv and a separate commissioning release are built on the Pi.
The minimal Hailo-8 kernel/runtime stack is now installed and passed a reboot;
an experimental IMX477-to-Hailo detector probe also passed finite-output and
timing checks. This does not establish detection accuracy, tracking identity or
production perception integration. See the [hardware inventory](archive/partially-implemented/hardware/HARDWARE_CURRENT.md)
and [AI plan](archive/partially-implemented/vision/AI_HAT_PERCEPTION_PLAN.md).

With the launcher stopped and camera ownership clear, the separate no-motion
probe exercises the pinned Hailo model without starting the controller/web:

```bash
run/station-venv/bin/python Firmware/tools/probe_hailo_camera.py \
  --hef /home/eamars/workspace/OpenAutoTurret/run/hailo-probe/yolov8n.hef --frames 30
```

Run from a committed release with the station project venv. It holds that
runtime directory's launcher lock, verifies the model SHA and HAILO8 identity,
and saves no images. Do not use another `OTA_RUN_DIR` to bypass ownership.

## Account, ownership and preserved operating contract

- SSH as `eamars@rpi-turret` using the existing key. Run station operations
  without `sudo`; uid 1000 owns `/tmp/ota-stack-1000` when the launcher runs.
- Checkout: `/home/eamars/workspace/OpenAutoTurret`. Preserve its local changes.
- After the Pi reboot, Windows DNS resolution for `rpi-turret` failed. The
  observed address was `192.168.2.100`; when needed, deploy with
  `--connect-address 192.168.2.100`. This is an observed address, not a static
  network setting. The option preserves the known `rpi-turret` SSH host-key
  identity while connecting to that address.
- One `Firmware/scripts/run_application.sh` launcher owns controller,
  `perception.visiond` and `web.webd.app`. Do not run old systemd services beside it.
- Normal startup uses AUTO_ROAM -> target tracking -> AUTO_ROAM after loss.
  Manual/Hold is an explicit web override, not a saved trial default.
- The web address is `http://rpi-turret:8080/` when the stack is running. A
  telemetry serialization fix now represents unavailable GM6020 torque as JSON
  `null`; the dashboard displays an em dash, and `/api/state` no longer fails on
  NaN yaw effort.
- Each physical camera has one owner; preview reads that owner's frames.
  The observed five-minute normal run used IMX500 and delivered 8,061 frames
  with zero drops while AUTO_ROAM and AUTO_TRACK/loss handoffs repeated. This is
  integration evidence, not an accuracy benchmark. The explicit
  `hailo_yolov8n` profile has passed a 60-frame IMX477 run through visiond in
  `--hold-motion` mode. A continuous BNO085 observer is launcher-supervised and
  observe-only; it has no motion-control authority. Simultaneous dual-camera
  operation remains unqualified.

## Inspect the stopped installation

Run from the Pi checkout/release as `eamars`:

```bash
bash Firmware/scripts/run_application.sh status
bash Firmware/scripts/run_application.sh check
ip -details -statistics link show can0
ip -details -statistics link show can1
rpicam-hello --list-cameras
lspci -nn
```

`check` inspects imports/config/files; it does not open motors or cameras and
does not prove motion readiness. The normal profile is now the mixed profile;
use `check --commission-hardware` for bounded commissioning preflight. Logs under
`/tmp/ota-stack-1000` exist only after a run.

After the September 27 reboot, both CAN links were DOWN; neither NetworkManager
nor the previous installation had a CAN startup profile. The one-time authorized
administrator setup installed and enabled `ota-can-links.service` from
[`../systemd/ota-can-links.service`](../systemd/ota-can-links.service) and its
[`../scripts/configure_can_links.sh`](../scripts/configure_can_links.sh) helper.
It brings `can0` and `can1` up at 1 Mbps classical CAN on boot, or validates an
already-up link without cycling it. It opens no motor transport. Check it as
`eamars` with `systemctl is-enabled ota-can-links.service` and
`systemctl is-active ota-can-links.service`, then inspect both links above.
Routine station operation remains unprivileged and uses the launcher. The
service also completed successfully at monotonic 5.76–5.82 s on the next
observed boot, and both links were UP, ERROR-ACTIVE, 1 Mbps. One successful
boot does not establish long-term recovery reliability. The Pi's idle
`get_throttled=0x0` after that boot does not replace a loaded power check.

Earlier September 27 large-motion sessions left both CAN links UP at 1 Mbps.
Their pitch drives ended with verified disabled feedback; yaw ended with zero
voltage requested and stationary feedback, but its disable state is unknown.
The current release is running in MANUAL/HOLD after D-pad tests. Inspect live
ownership/state before another session; do not cycle CAN links between tests.

The owner authorized the September 27 one-time privileged CAN boot setup after
the reboot. This does not change unprivileged launcher ownership.
Never put credentials in scripts or Git. For motor probes, identify the exact
protocol first; discovery must not enable, zero, home or actuate a motor.
GM6020 `0x1FF` is a voltage command, not a discovery request. See the
[GM6020 reference](references/gm6020/GM6020_AI_Reference.md) and
[CyberGear reference](references/cybergear/CyberGear_AI_Reference.md).

The existing IMU probe is `/home/eamars/workspace/imu-lab/imu_main`, with source
and README beside it. It soft-resets the IMU and enables three sensor reports;
it is not a passive bus read. Run it only when it owns the sensor and its reset
cannot disrupt a running consumer. Its gyro output label is wrong: values are
rad/s, not deg/s. See the inventory for measured results and remaining gaps.

Use the versioned replacement for further IMU work:

```bash
bash Firmware/scripts/run_application.sh run --probe-imu --imu-seconds 30
```

Deploy it with `deploy_station.py --probe-build --probe-imu`. It needs only
unprivileged I2C access, records `/tmp/ota-stack-1000/imu.ndjson`, and opens no
camera or motor transport. It establishes a stationary **host reference**, not
a mounting calibration. `--commission-hardware --with-imu` adds the same capture
to bounded motor probes. See [IMU evidence and coordinate meaning](archive/partially-implemented/commissioning/IMU_COMMISSIONING_2026_09_27.md).
Do not assign the pitch-mounted IMU pose directly to the base orientation.

For the tested camera-only Hailo application slice, use
`run --hold-motion --profile hailo_yolov8n --frames 60 --no-web`.
The shared model remains under the original checkout's `run/hailo-probe` and is
linked into releases, with SHA verification before use. IMX477 camera-to-axis
calibration is still required before its detections may guide physical motion;
the existing 1920x1080 camera calibration does not certify this 640x480 profile.

## Stop and preserve evidence

If a launcher-owned stack is running, stop it through the launcher:

```bash
bash Firmware/scripts/run_application.sh stop
bash Firmware/scripts/run_application.sh status
```

The launcher requests the controller's axis-specific safe stop action, then
shuts down its children. It never force-kills the controller. Pitch disable is
feedback-confirmed when fresh; GM6020 receives a zero request but its disable
state is unavailable. If its 120-second caller wait
expires, inspect status/logs; shutdown may still be in progress. Do not start a
second controller or use broad process kills. Stop can be issued from another
checkout because ownership is shared by account/runtime directory.

**Stop qualification includes two successful moving-stop observations on release
`cae41d0` and one further stop on release `8901808`; broader qualification
remains open.**
The previous middle-yaw/lowest-pitch release contract and CyberGear disabled-bit
verification do not transfer to GM6020. A zero GM6020 command does not certify
power removal or a supported load. Complete stop/park qualification before
unattended operation. The web's parking request is not equivalent to full
launcher stop.

On `cae41d0`, controlled stop completed successfully after AUTO_TRACK motion at
about +46° yaw and after AUTO_ROAM motion at about +76° yaw. Both recorded fresh
pitch-disabled feedback and a final GM6020 zero request. GM6020 disable state
remains unknown; zero request is never power-removal certification. The prior
release had intermittent feedback-readiness rejection. The two successes do
not prove that issue is eliminated or establish full stop/recovery/park
qualification.
Do not interpret a zero-voltage request as power removal.
The separate `--apply-pitch-limit` option only writes the configured volatile
CyberGear `LimitCur` value (5 A maximum) and checks three matching readbacks; it
does not enable or move pitch. It cannot be combined with yaw voltage or speed
actuation. Reapply and verify volatile pitch settings after reset and before
enable; the production position/speed mode paths must establish and verify the
current cap before enabling the drive.

Preserve numeric logs before restarting. Never overwrite retained homing data,
manually mark axes homed or bypass validation. Invalidate old calibration by
installation identity. Yaw needs reference initialization instead of endpoint
homing. Pitch homing has completed in the normal mixed controller, but repeated
speed-ceiling/corridor warnings did not abort with
`homing.motion_checks_abort: false`; this guard behavior remains unqualified.
IMU orientation is a secondary observation, not a replacement for
motor/reference validity.

### Who stopped it

`status` on a stopped stack prints `shutdown.cause` before the park outcome. The
cause line names the trigger, and the labels are deliberately not interchangeable:

| `cause=` | meaning | extra fields |
|---|---|---|
| `operator_stop` | someone ran `run_application.sh stop`; the stopper left a credential | `operator="who=operator pid=… uid=… utc=… launcher=…"` |
| `child_exit` | a supervised child died first, so cleanup followed; the child is blamed by name | `exited_child=… exited_pid=… wait_status=…` (`137(signal 9)` = SIGKILL; `wait_status=0` = that child ended by itself, e.g. a finite capture) |
| `external_signal` | the launcher was signalled with no stop credential | `signal=INT`/`TERM` |
| `unattributed` | cleanup ran and nothing above applies | — |

A credential outranks a signal: an operator stop *is* a SIGTERM, and naming the
caller beats naming the syscall. A clean-looking stop with `cause=unattributed`
is treated as an open question, not as a normal stop.

Because every run reuses `$RUN`, the previous round's `*.log`, `shutdown.result`,
`shutdown.cause`, `stack.info` and the whole `traces/` directory (trip files and
`stop-evidence.ndjson`) are moved to `$RUN/logs-history/<utc>Z-launcher<pid>/`
before anything is truncated; the newest ten rounds are kept and older ones are
pruned, since `$RUN` lives in `/tmp`. Read the archived round, not the fresh one,
when investigating a stop.

The claims above are rehearsed against a real stack (each scenario homes the
station, so this is a motion-bearing test):

```bash
ssh eamars@rpi-turret "bash <release>/Firmware/scripts/run_application.sh status"   # context
ssh eamars@rpi-turret "<release-venv>/bin/python <release>/Firmware/tools/rehearse_stop_cause.py"
```

`--selftest` exercises only the pass/fail logic, for machines with no station.
The rehearsal leaves the station stopped.

### Owner rulings of 2026-09-28 (midday), and what they changed

Four rulings, all of them about over-design inherited from the first pass. The reasoning
and the full list of sites that can remove motor power are in
[AUDIT_POWER_REMOVAL_2026-09-28.md](ADR-001/reports/AUDIT_POWER_REMOVAL_2026-09-28.md);
what an operator needs from them:

- **Removing power is the last resort, not a routine response.** The payload is an
  unbalanced load: if power goes, it drops onto a hard stop, and that collision costs
  more than never cutting power. So the yaw speed ceiling is a clamp on the ASK
  (`min(ceiling, requested)`, `kYawSpeedCeilingDegS = 30`), not a trip on a reading -- the
  `speed_over_ceiling` condition is deleted, not tuned. A non-finite feedback reading is
  still trusted-nothing; a fast one is not.
- **MANUAL is one gesture across both axes.** `axes.yaw` used to declare 10 deg/s against
  pitch's 30 under a shared `motion.modes` block asking both for 20, which is why manual
  felt like two different sticks. Both now declare 30; the config test asserts the
  invariant ("both axes resolve to the same speeds in every mode") instead of numbers.
- **The slip ring has no constraint** (owner, confirmed): free rotation, no turn counting.
  The cable-loop caution that used to sit around the yaw travel argument is retired --
  there is no mechanical objection left to unbounded yaw.
- **AUTO_ROAM patrols the circle (owner ruling 2026-10-02, replaces the parked redesign).**
  With `position_envelope: none` the roam is no longer a sweep between the ends of the
  +/-90 deg band around the power-up zero (which could sweep forever without facing the
  person, and could not be moved). It turns one way, continuously:
  - **pace per view:** 15 deg/s with wide on the main display, 3 deg/s with detail, so a
    person stays in the picture about 4.5 s either way;
  - **after a loss:** it resumes the way the lost person was moving, otherwise the way the
    last patrol went;
  - **pitch:** kept if within 10 deg of the median pitch at which people were tracked (the
    travel middle before the first one), otherwise moved only to that band's edge.

  Settings live in `v3.auto_roam` of `turret_mixed.yaml`. The AUTO_ROAM yaw target speed is
  15 to match. The bounded sweep still serves stations with a yaw envelope.
  - **MANUAL DPAD uses the same paces (owner, 2026-10-03):** COARSE is the wide patrol speed
    (15 deg/s, NORMAL's ramps), FINE the detail one (3 deg/s, FINE's ramps). By default the
    arrows send jog profile `view` and controld picks the pace matching the camera on the main
    display. The pad's centre (formerly a HOLD that was not a button) shows the pace; tapping
    it pins the other one (`coarse` / `precise`), and a camera swap un-pins it. Releasing an
    arrow is the stop; STOP MOTION stays in the MANUAL drawer.
  Meanwhile: a `no_progress` trip with the axis parked outside its computed sweep interval
  is a known open case (see the case file §4-§6). Since 2026-10-03 a latch is recovered from the
  web: HOME recovers the drives, then homes (see "The web MENU" above).

## What a stop proved, per axis (2026-09-28)

`shutdown.cause` says **who** stopped it. It cannot say **what we can prove about the
outcome**, so controld appends one line per stop to `$RUN/traces/stop-evidence.ndjson`
(archived with its round, like everything else in `traces/`):

```bash
ssh eamars@rpi-turret "cat /tmp/ota-stack-1000/traces/stop-evidence.ndjson"   # or the archived copy
```

Read it by `stage`, in pairs sharing one `stop_id`:

| `stage` | what it can claim |
|---|---|
| `requested` | we asked. Usually `completion_quality=unverified` with `missing_evidence` naming the gaps — that is the point, not a defect. |
| `parked` | what was observed: per axis `zero_requested` / `disable_requested` / `disable_confirmed` / `stationary_observed` over a stated `stationary_window_ms`, plus `feedback_age_ms`. |

Three vocabulary rules, because a stop is read later by someone who wasn't there:

- **`unsupported` is not `false`.** The GM6020 feedback frame has no enable bit, so yaw's
  `disable_requested`/`disable_confirmed` are `unsupported` forever. It neither helps nor
  hurts `completion_quality`: it is a fact about the drive, not a gap in this stop.
- **`stationary_observed` is the weakest claim on the sheet.** It means position held
  inside a tolerance for the stated window — not "de-torqued", and not "safe to put a hand
  on". The window is published so the claim's strength is visible.
- **No green for the pair.** There is no merged boolean; `completion_quality` is computed
  from the two axes' claims and `missing_evidence` names what is absent.

First real record (`stop-60378202284708`, 2026-09-28 08:46, release `c9b4ffb1282e`):
pitch `disable_confirmed=confirmed`, stationary 557.9 ms, feedback age 4.573 ms;
yaw stationary 502.2 ms (the dwell), age 0.265 ms, `disable_confirmed=unsupported`;
`completion_quality=verified`, `missing_evidence=[]`.

Not yet covered: a stop that **fails** writes only its `requested` line — `fail_parking()`
does not emit, so "requested with no completion" is currently the signal for a failed
stop. Linking `stop_id` to `shutdown.cause` (so you can tell who asked *and* what it
proved from one file) is an open item.

## Deployment and operation

Use the following launcher path for the mixed profile. Two controlled stops
have passed on `cae41d0`; do not regard normal operation as fully commissioned
until broader stop/recovery evidence and remaining acceptance gates are
reviewed, including the intermittent readiness rejection seen on the prior
release.

Use the existing project-local virtual environment with OS camera bindings and
install station requirements there. Never install pip dependencies globally or
commit the environment. `run/station-venv` exists with system camera bindings;
builds live in separate `run/releases/.../Firmware/build` directories. The
minimal Hailo-8 driver/runtime is installed and verified after reboot; do not
replace it with `hailo-all` or install Tappas as part of this minimal profile.

Deploy committed source with `Firmware/tools/deploy_station.py`. It archives
`HEAD`, creates a separate release under `run/releases`, records `REVISION`,
builds/tests and performs preflight while preserving the Pi checkout. It refuses
dirty source. Use a project-local Python interpreter; no push is required.

Without `--activate`, deployment does not start motors. `--probe-build` builds
the controller and commissioning probe and runs preflight while deferring regression tests; it is
probe-ready evidence only. `--activate` additionally stops/starts through the
launcher and verifies readiness. It can move motors and is inappropriate for
unadapted source.

After implementation and commissioning, the usual entry points are:

```bash
bash Firmware/scripts/run_application.sh deploy  # inactive checkout: build/test/check
bash Firmware/scripts/run_application.sh start   # also the no-argument default
bash Firmware/scripts/run_application.sh status
bash Firmware/scripts/run_application.sh stop
```

Start returns after child launch, not after readiness. Current runtime uses a
session-relative continuous-yaw reference, pitch-only homing, fresh feedback,
bus identity checks and valid perception. A successful AUTO_ROAM run does not
prove stop behavior, tracking accuracy or final station readiness.

`--sim` still opens the real camera; `--hold-motion` is perception-only and also
opens it. Neither replaces camera ownership checks or verifies the new backend.
Keep trial mode/speed/gain overrides out of normal releases. Rollback must select
a release qualified for this hardware or leave the station stopped; never
restart a dual-CyberGear build on the new mechanism.

## Bounded commissioning, without automatic startup

See [the September 26 implementation and test record](archive/partially-implemented/commissioning/HARDWARE_COMMISSIONING_2026_09_26.md)
for the tested revision and release path. Deploy a committed commissioning build
with the local project Python:

```bash
python Firmware/tools/deploy_station.py --probe-build --commission-hardware
```

If `rpi-turret` does not resolve from Windows after reboot, the observed
connection workaround is:

```bash
python Firmware/tools/deploy_station.py --connect-address 192.168.2.100 --probe-build --commission-hardware
```

The address is evidence from this session, not a static configuration promise.

On that release, as `eamars`, with both links already at 1 Mbps and UP:

```bash
bash Firmware/scripts/run_application.sh check --commission-hardware
bash Firmware/scripts/run_application.sh run --commission-hardware
# Optional non-motion operation: apply and verify the pitch LimitCur ceiling.
bash Firmware/scripts/run_application.sh run --commission-hardware --apply-pitch-limit
# Explicit motion: repeat only within the commissioned envelope and clear mechanism.
bash Firmware/scripts/run_application.sh run --commission-hardware --yaw-voltage 1000 --pulse-ms 150
bash Firmware/scripts/run_application.sh status
bash Firmware/scripts/run_application.sh stop
```

The default probe only receives yaw and queries pitch discovery/mechanical
position. A rejected pitch register read is reported unavailable, never treated
as a position. The probe does not home, enable, zero or actuate pitch. With
`--apply-pitch-limit`, it writes only volatile `LimitCur=5 A`, requires three
matching readbacks, and leaves pitch disabled; this must be reapplied and
verified after a reset before any enable. Following the owner's September 27
CyberGear 1.2.1.5 upgrade, the same UID returned valid `MechPos` (-0.710777 rad)
with status 0, and raw feedback reported mode 0/faults 0. The earlier rejected
register read is historical. Later normal mixed runtime completed pitch-only
homing, but repeated encoder-speed-ceiling and commanded-corridor warnings were
non-aborting under `homing.motion_checks_abort: false`; treat the guard behavior
as unresolved.
Do not combine limit setup with yaw actuation.

### Pitch motion session

The owner's tuning preference is to establish meaningful motion using the full
authorized output headroom first. For pitch that means **5 A maximum**, never the
motor's larger factory limit. Use a clear bounded target instead of escalating
from tiny current/speed commands. A current limit is available headroom; it does
not mean the controller must draw 5 A continuously.

Keep pitch enabled between movements, and keep CAN and IMU acquisition live
through the session. Do not cycle the stack, lower the CAN links or disable the
motor between individual stages. Stop on a fault or explicit session completion.
No persistent gain, homing, encoder-zero or calibration writes are part of this
probe. The commissioned ±15° pitch session is:

```bash
bash Firmware/scripts/run_application.sh run --commission-hardware --with-imu \
  --pitch-step-mdeg 15000 --pitch-test-gains
```

It verifies the 5 A cap and position mode, then enables once for two step/return
pairs: +15°, start, −15°, start, at a requested 10°/s. Pitch remains
energized while settling and between all four stages. The explicit gain trial
uses speed-loop Kp=4, Ki=0.05 and restores nominal 1/0.002 at session completion.
The final stop is not a parking/homing certification. Numeric traces are
`pitch-probe.csv`, `controller.log` and `imu.ndjson` in the launcher runtime.

The probe bounds excursion from initial position to 17°, encoder-derived speed
over at least 50 ms to 20°/s, feedback/heartbeat age to 100 ms, and temperature
to 45°C. The firmware's raw speed field has shown noise inconsistent with small
encoder changes; it remains logged but does not alone establish actual speed.
These bounds do not qualify an unknown pitch endpoint or automatic homing.
See [the paired large-motion and IMU record](archive/partially-implemented/commissioning/LARGE_MOTION_COMMISSIONING_2026_09_27.md).

### Yaw motion session

The commissioned yaw excursion is 30° out and back in one continuous CAN0
session with a fresh BNO085 host tare:

```bash
bash Firmware/scripts/run_application.sh run --commission-hardware --with-imu \
  --yaw-step-deg 30
```

The successful run moved 29.356° outbound and returned to +0.659° relative to
its start; the IMU independently measured +29.183° and −28.446° on the two
legs. The GM6020 voltage output ceiling is the vendor-documented ±25,000 raw,
while actual commands stayed within −4,268..+5,643 raw. It guards travel,
speed, stale feedback and stalled progress, then requests zero voltage and
observes a stationary motor. GM6020 zero voltage is not a verified disable or
mechanical park. See the [large-motion record](archive/partially-implemented/commissioning/LARGE_MOTION_COMMISSIONING_2026_09_27.md)
for bounds, failures and raw evidence.

`config/hardware_probe.yaml` is a separate probe schema, **not** a production
controller configuration. Fixed ceilings are |voltage| <= 3000 raw, pulse <=
500 ms, travel <= 5 degrees, speed <= 20 degrees/s, feedback age <= 20 ms and
heartbeat gap <= 40 ms. Recorded trials include +/-1000 and +/-1500 raw for
150 ms, +2000 raw for 100 ms, and bounded +/-3 deg/s PI requests for 500 ms.
The PI trial stayed within the guards but did not achieve its requested speed;
its 1500 raw ceiling and gains are not production-qualified. See the
[continuation evidence](archive/partially-implemented/commissioning/HARDWARE_CONTINUATION_2026_09_26.md). These raw voltage
commands are not amperes.

The probe verifies SPI parents, bitrate, ERROR-ACTIVE state, UID and stationary
yaw baseline before output. The 200 Hz pulse loop and separate in-process guard
serialize commands, stop on stale/invalid feedback or CAN error frames, and
request zero after pulse deadline/interruption. This guard cannot survive loss
of the process or Pi. Automatic operation and process-loss behavior remain
unqualified. No independent power-cutoff capability has been established.

The launcher and probe hold station-wide locks independent of `OTA_RUN_DIR`.
Do not run other motor transmitters alongside them. Numeric evidence is written
to `/tmp/ota-stack-1000/hardware-probe.csv` and `controller.log`; copy it into
ignored `run/` before the next probe replaces it. No camera or web process is
started in commissioning mode.

### Yaw runs without a position envelope (2026-09-28)

`config/turret_mixed.yaml` declares `axes.yaw.position_envelope: none`. Continuous GM6020
yaw therefore enforces **no position limit at runtime**: the provisional +/-90 degree
session sector is gone, and the travel band that used to define it stays only as the band
automatic regions are validated inside. The startup log says so rather than leaving it to
be inferred from a missing limit:

```
[warning] continuous yaw declared WITHOUT a position envelope; the sector is gone,
not merely unmeasured (AUTO_ROAM still sweeps a declared region, and every other
guard is unchanged)
```

What to expect, measured on this station rather than assumed:

- **Readiness still lights.** `soft_limits_valid` now means "every axis has a declared
  envelope state", and a declared-unbounded axis satisfies it. An axis nobody has written
  down yet does not.
- **Angles run past the encoder seam.** A manual jog took yaw from -0.3 to **+274.1 degrees**
  with no fault; the session angle is unwrapped, so it does not jump at 180 degrees.
- **AUTO_ROAM still sweeps a bounded region.** With no wall to inherit one, the region is
  declared: it is centred on the session reference and sized by the search span (the log
  reports `search sweep clamped to [-37.1, 37.1] deg`), so the mode keeps its promise of a
  deterministic bounded sweep that a person watching can predict.
- **The independent guards are untouched.** After the envelope went away, the first long
  approach drove yaw past the backend's own `speed_over_ceiling` guard (25 deg/s, hard
  coded). That trip was correct; the ask was wrong, and the fix capped the sweep at the
  yaw's declared maximum instead of widening the guard. The ceiling itself remains the
  operator's parameter.
- **What the telemetry says about it** (fixed the morning after, 2026-09-28): the wire
  carries `yaw_envelope:"none"` and publishes `q_soft_min_yaw_rad`, `q_soft_max_yaw_rad`
  and `soft_limit_distance_yaw_rad` as `null` -- not 0/0, and not the internal -1
  (`kNoBoundary`) walking around as if it were a distance. Consumers map "no boundary" to
  their own kind of absence; a zero there reads as "the wall is where you are standing",
  and the dashboard then lit it as near-limit, because `null < 0.05` is true in JS.
- **The yaw travel tape stays.** It is drawn from `yaw_band_min_rad`/`yaw_band_max_rad`,
  the band the station file still declares, centred on the homing origin -- a ruler, not a
  limit, which is why removing the sector never had to take the scale with it. With no band
  to show (never homed) the tape gives up to the `TRAVEL UNRANGED` note rather than drawing
  a tape out of zeros.

To revert: delete the `position_envelope: none` line. The +/-90 degree band with its 10
degree inset is enforced again on the next start, and nothing else about this change needs
undoing -- the fourth envelope state describes what an envelope is, not how big it is.

## Historical procedures

Detailed September 8-9 homing, recovery, tuning and parking instructions are
preserved in the [retired dual-CyberGear runbook](archive/superseded/operations/station_operations_dual_cybergear_2026_09_09.md).
Their measurements remain background, not certification of this mechanism.
`STATION_RUNBOOK.md`, MCP2515 setup/fault reports and `AS_BUILT_v1.md` describe
prior installations. The earlier BNO085 proposal is also historical design
input; its hardware-absent status is superseded, while integration remains open.
See [the documentation map](README.md).

## Owner rulings of 2026-09-28 (afternoon) — tapes, acceleration, and where yaw's zero comes from

**Tapes are cyclic, and the caret never moves.** Both axes, one widget: the marker sits at the
tape's midpoint always; the ruler slides under it; and past the end of the declared travel the
ruler **rolls over** instead of showing a dead region (`不需要死区，如果超过了值，就直接 roll over`).
yaw is a continuous axis, so the wrap is what the world already does; pitch is physically blocked
and rarely reaches the seam, but is drawn by the same rule. The window is the camera's own field
of view on that axis (`effective_hfov_deg` / `effective_vfov_deg`), and the tape says which source
it used. The seam — where the ruler wraps — is labelled with the endpoint's own number, in amber
when a DERATE names that end, and it never fades to zero opacity (an earlier draft painted the
approaching limit invisible exactly when it mattered most).

**Manual acceleration parity is a requirement, not a tuning opinion.** The complaint
("yaw的加速度在manual模式下…比pitch低得多") traced to `kYawMaxAccelerationRadS2 = 20 deg/s²`, a
codex-era constant living inside the yaw backend — the station file said 30 and pitch's drive ran
60. All three are now 60, and `test_mixed_station_config` pins the ramp constant to
`axes.yaw.max_acceleration_deg_s2` so a fourth opinion cannot appear again.

**Non-finite reference rate ⇒ hold and report; the session angle is never re-zeroed.** My ruling,
delegated by the owner ("#3，你来决定"): the division-by-zero theory does not survive the code
(the reference-rate path divides only under `dt > 1 ms`), and on a continuous axis a wrong zero
silently rewrites every number on his tape. A nonsense rate is a reason to stop and say so, not a
reason to move the origin.

**yaw's zero will come from the IMU; until then, the pitch homing origin is yaw's 0.** Recorded as
the owner's intent. **As built today this is NOT true**: yaw is not homed on this station, so its
session zero is wherever it happened to be when the process started (or the retained pose), which
means the tape's "0 = where I was zeroed" is not reproducible across restarts. The change is
small and named: where homing establishes the pitch origin, set the yaw session reference to the
yaw angle at that instant. Until then, treat yaw degree readings as relative, not as position.

### Rates are declared per axis (owner, 2026-09-28), and PID tuning is parked

> 「我建议还是两轴单独设置。我不能确保 yaw 和 pitch 真的能做到等同的加速度。所以分开设置（但是值可以设置成一样）。」

`motion.modes.<mode>.axes.{yaw,pitch}.{maximum,target}` — six numbers per mode, spelled out,
even where yaw and pitch agree. The loader accepts a mode-level shared pair only for older
files; a new file that omits it must cover **both** axes, and anything else is an error rather
than a fall-through onto `MotionRates`' service-cap defaults (20/30/120 nobody wrote).
`test_mixed_station_config` fails if an axis stops declaring its own six numbers.

Why the distinction matters on this station: yaw closes its own velocity loop in voltage mode
and was measured (2026-09-28) trailing its reference by ~1 s and settling ~8 deg/s low under the
3 kg payload, while pitch's drive closes its own loop internally. Declaring them jointly would
have encoded a claim about the hardware that nobody had measured.

**Parked until ADR-001 is done:** re-tuning the yaw velocity loop (Kp/Ki, and/or feeding the
shaped speed forward so the loop only closes the error). The owner accepts the current
slowness for now — "如果是PID导致的速度缓慢那我可以接受。目前先不改" — and wants to tune it
afterwards. `yaw_cmd_shaped_deg_s` / `yaw_cmd_output` in `/api/state` are the instruments for
that session: the measurement is the lag between ask and measured, in milliseconds.

## The station's SSH identity is passed in, not hoped for

Measured 2026-09-28: after the DSH container image was rebuilt, every `ssh` in
`tools/deploy_station.py` started failing with status 255 — the container's home is not durable, and the
ambient `~/.ssh/known_hosts` that earlier deploys had silently relied on went away with the old container.
A deploy that only works while one container's home directory survives is not a deploy.

So the station's key is pinned in a file and handed to the tool explicitly:

```bash
OTA_SSH_IDENTITY=/workspace/general_purpose/.secrets/ssh/id_ed25519 \
OTA_KNOWN_HOSTS=/workspace/general_purpose/.secrets/ssh/known_hosts_station \
  "$WS/.venv/bin/python" "$PWD/Firmware/tools/deploy_station.py" \
  --host eamars@rpi-turret --connect-address <observed> --activate --ready-timeout 420
```

`--known-hosts` sets `StrictHostKeyChecking=yes` and blanks `GlobalKnownHostsFile`, so a different box
wearing that address cannot be accepted silently. The pinned line is

    256 SHA256:ll1B6KKdmry4daddh4fMxJ4ecnLS7zp9hBH+3DO0fQw rpi-turret,192.168.2.100 (ED25519)

`Firmware/tools/station_address.sh` now assembles this call, resolves the address by verifying it,
and is the route the deploy card tells you to use; the invocation above is kept so the parts are
visible. Note that the interpreter is spelled out: a bare `python3` has no third-party packages here
and will render a red suite green. (`--identity` (env `OTA_SSH_IDENTITY`) does the same for the private key, with `IdentitiesOnly=yes`:
the same rebuild took `~/.ssh` away, and the resulting 255 reads like a host-key failure but is an
auth failure -- both halves of "the container's home is not durable" were measured this way.

which was checked against the host answering today (`hostname` = `rpi-turret`, `uname -m` = `aarch64`).
If that fingerprint ever changes, that is a finding to investigate, not a line to update.

## 站从网络上消失（2026-09-28 深夜，第九次 activate 期间）

现象：`--prebuilt` 部署跑到远端 ssh 那一步抛 `CalledProcessError`，随后
`ssh: connect to host 192.168.2.100 port 22: No route to host`，ping 100% 丢包。

排除网络侧：同一时刻 192.168.2.4（Synology）、.53（打印机）、.40（NVR）**全部 ping 通**，
默认路由正常；全 /24 广播探测后 `ip neigh` 里 `192.168.2.100` 为 `FAILED`，
且**没有任何地址带 MAC `88:A2:9E:D9:C9:DF`** ⇒ 不是换了 IP，是主机不在线。

未定原因（不猜成结论）：内核挂死 / 掉电 / 网络栈死。现场处置需要人（我无 sudo、也没有它的电源控制权）。
待回来后要查的第一样东西：`journalctl -b -1 | tail`（上一次 boot 的末尾）——**如果它是重启过的，
这段会告诉我们是谁干的；如果它被拔过电，这段会直接断掉。**

补：站上 journald **没有跨 boot 持久化**（`journalctl -b -1` 为空）。硬断电之后"上次 boot 的末尾"就查不到了，
也就是说这台站现在**无法自证死因**。要么开 `Storage=persistent`（要 sudo，等主人方便时），
要么承认崩溃取证只能靠 controld 自己落盘的 trace/evidence——这反过来正是 WP2 那些记录的价值所在。

## Reading the per-cycle control trace

`GET /api/control_trace` on webd returns controld's ring as-is; on the station
the same read is `Firmware/tools/pull_control_trace.py`. Both share
`Firmware/common/control_trace.py`, which is why neither has its own idea of how
big a reply can be: controld answers with a `control_trace` frame on a
`SOCK_SEQPACKET` socket, a full ring is on the order of a megabyte, and an
oversized datagram is truncated rather than split -- so a small receive buffer
turns evidence into a parse error.

Read it inside about twenty seconds of a trip: the ring wraps. A `frozen` window
is the trip's own snapshot and its summary line starts with `FROZEN_AT=`. The
route answers 503 with the socket path when controld is not reachable, which is
deliberately a different shape from a ring with no rows in it.
