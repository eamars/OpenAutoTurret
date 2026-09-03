# OpenAutoTurret — Phase Status

**Tracks** [`Firmware/docs/open_auto_turret_software_control_architecture_v1.md`](Firmware/docs/open_auto_turret_software_control_architecture_v1.md).

This file answers one question: **how far along is each phase, counting only what has
been observed doing what it claims.** It carries no feature detail, no file lists and
no measurements — those live in
[`Firmware/docs/AS_BUILT_v1.md`](Firmware/docs/AS_BUILT_v1.md), which is also where the
shipped defects are named. Where the two disagree, `AS_BUILT_v1.md` is the one that was
updated against the running code.

**Location rule:** all production software lives under `Firmware/` (`control/`,
`vision/`, `web/`, `config/`, `calibration/`, `tools/`); the spec's §50 tree is applied
relative to `Firmware/`.

## Phase status (spec §56)

| Phase | Code | On hardware | What the second column does and does not mean |
|---|:--:|:--:|---|
| 0 — Preserve / instrument POC | ✓ | n/a | POC archived under `Firmware/legacy/` |
| 1 — Production CAN/motor layer | ✓ | ✓ | Discovery, feedback, 200 Hz command/response 1:1, all driven by the real homing run. Drive fault injection (queue P5) has never been run |
| 2 — Homing and safety foundation | ✓ | ✓ | First live homing 2026-09-03: 106 s, pitch span 79.9°, yaw 352.8° to real stops. Hold and park observed on every daemon stop |
| 3 — Trajectory generator | ✓ | ◐ | Every live move uses it, and the search sweep held 9.8 °/s against a 10 °/s cap; it has never been profiled against the §44 response metrics on real metal |
| 4 — Vision daemon | ✓ | ✓ | Real IMX500 pipeline verified without CAN or motor. Detector *quality* is not characterised — nothing about detection may be concluded from guarded or synthetic runs |
| 5 — Geometry and estimator | ✓ | ◐ | Proven by replay and simulation. LOS has never been checked against a measured ground truth on the station |
| 6 — Closed-loop tracking | ✓ | ◐ | Cold-start roaming verified live 2026-09-03 (search 1.5 s after `start_tracking` with no target ever seen; swept 104.4…193.2°). Closed-loop tracking of a real target (P8) has not |
| 7 — Installation orientation calibration | ✓ | ✗ | The chain exists end to end and its file formats are now readable by `controld`; it has never been run, because the ChArUco board is not printed |
| 8 — Web UI and diagnostics | ✓ | ◐ | Served and used for today's hardware runs, with real video. Offline load rehearsal 26/26; the live end-to-end checklist (P12) is not signed |
| 9 — Payload profiling / tuning | ✓ | ◐ | The `conservative` profile is in service live and its derate is real; the §44 profiling battery has not been run on the station (P10) |
| Cross-cutting (§54/§55) | ◐ | ◐ | Simulated plant §54.2, replay §54.3, UI load §54.5 and §55 metric extraction exist; the HIL checklist (P13) is open |

`✓` done and observed · `◐` exists and works, verification incomplete · `✗` not run.

The two columns are separate on purpose. A single checkbox per phase is how this file
ended up saying Phase 7 was untouched while its code had been merged for a day: "no"
meant *not verified on hardware* and read as *not built*. If v3 wants one column, it
should pick which of the two it means and mean it.

## v3 — three-mode target tracking (spec §101)

**Tracks** [`Firmware/docs/open_auto_turret_v3_three_mode_target_tracking_architecture.md`](Firmware/docs/open_auto_turret_v3_three_mode_target_tracking_architecture.md).

Same two columns, same rule. **`controld` running on the station today is the pre-v3
binary** — every v3 row below is simulation, and no number of green tests changes the
second column. Deploying v3 means a ~106 s re-home, so it is arranged with the operator,
not done.

| Phase | Code | On hardware | What the second column does and does not mean |
|---|:--:|:--:|---|
| V3-1 — ModeManager, modes, telemetry | ✓ | ✗ | Refused transitions carry real reasons and are asserted in simulation; never observed on metal |
| V3-2 — TrackSet IPC (79 tracks, UUIDs) | ✓ | ✗ | 58 B v1 frames and 2562 B v3 frames share one socket, discriminated by length; controld must be upgraded before visiond |
| V3-3 — TargetSelectionManager | ✓ | ✗ | Selection survives mode changes and clears only on `clear_target`; labels are reused, UUIDs are not |
| V3-4 — AutoTrackController | ✓ | ✗ | Confidence derating and coast verified in simulation. No real person has ever been followed (P8 is still open in v1 too) |
| V3-5 — ManualController | ✓ | ✗ | Jog lease, three profiles, steps. A defect found here: the moving waypoint was built from a buffer zeroed each cycle, so a jog travelled 2.02° and stopped |
| V3-6 — RoamPlanner | ✓ | ✗ | Bounded sweep strictly inside the safe envelope, deterministic over 40 000 cycles. The envelope is derived from the station's own limits, not named in config |
| V3-7 — Mode transitions | ✓ | ✗ | Authority ramps over 300 ms at a handover; §34's "AUTO_TRACK never roams" asserted. Never seen by a person watching metal |
| V3-8 — Reacquisition and ambiguity | ✓ | ✗ | A tie between two candidates does not move the turret. The two refusals are distinct and both are tested: "no candidate is plausible" (long gap) and "two are, and neither is the chosen one" (short gap) |
| V3-9 — Operator surface and diagnostics | ◐ | ✗ | Candidate list, jog row, event feed, §50 fields shipped. The video overlay is not; selection still needs the operator to read a label off the overlay |

Rollout order is controld first, then webd/visiond — the dashboard's v3 row self-disables
when `operating_mode` is absent, and STOP MOTION is never in the disable path. Behaviour
changes an operator should be told about before the first v3 run: AUTO TRACK with no
selection now **holds** instead of searching; STOP MOTION stops **in place** rather than
returning to the ready pose; selection is mode-independent; and `selected_track_id` is the
identity acted on (0 while tracking is off), which is not the same field as
`selected_display_index`, the label that was typed.

## What has no phase, and bites anyway

Two shipped defects are named in `AS_BUILT_v1.md` rather than tracked as phases,
because they will otherwise be inherited as assumptions: the CyberGear mode-entry
recipe sleeps on the 200 Hz control thread (≈110 ms cycles, answered by `BRAKE`), and
the dashboard's speed readout spikes to ~2.8× the axis's real speed at a sweep
reversal.

**v3 inherited both, unchanged.** The recipe sleep is four `sleep_for(50 ms)` calls
inside `can_motor_backend.cpp`'s mode-entry recipes, invoked from homing on the control
thread; 28 cycles of 109–113 ms per homing run are the measurement. v3's `BRAKE`
answer is the same one, which means v3's mode machinery is currently being *exercised*
by a defect rather than by design: a station that is braking during homing is a station
whose 200 Hz assumptions are not holding. It is listed here rather than in a V3 phase
because it predates v3 and fixing it touches the code that moves real metal, which makes
it the operator's call about risk, not mine about convenience.

## §110 acceptance — what has to be measured, and by whom

v3's checklist cannot be closed from this repository, and pretending otherwise is how
the old `enable_search` checkbox survived. The items below need a person at the station;
each row says what is already known from simulation and what only metal can answer.

| Item | Known in simulation | Needs an operator because |
|---|---|---|
| First v3 homing + mode entry | v3 does not change homing | The re-home takes ~106 s and moves the turret; nobody is at the station now |
| Following a real person (P8, v1 and v3 alike) | Track → LOS → joint solve holds in sim; derating and coast asserted | Detection quality is uncharacterised. "It tracked" is a statement about the detector, not the controller |
| Ambiguous reacquisition (§21) | A tie does not move the turret; the event is recorded with both scores | Only a person can say whether the two candidates *looked* ambiguous from the room |
| Jog lease under a real tab death (§38) | Renewal at 100 ms against a 300 ms lease, asserted; expiry recorded as an event | Wifi drops, browser freezing, and a phone going to background are not reproducible from a test |
| Sweep inside the safe envelope (§33) | Deterministic over 40 000 cycles; envelope strictly inside derived limits | The envelope is derived from limits that have never been checked against a measured ground truth (Phase 5, `◐`) |
| STOP MOTION in place (§27) | Stops in place; no return-to-ready; asserted as a distance | Whether "in place" is where the operator wanted it to stop is a judgement about the room, not about the code |
| Handover continuity (§36/§44) | 300 ms authority ramp; excess decays; asserted per axis | A person has to say whether the motion *looked* continuous. A number inside a limit can still look wrong |
| Operator-visible diagnostics (§50/§78/§79) | Fields published, guarded, and rendered; events on the feed | Whether the panel answers the question the operator actually asks is only knowable by watching them use it |

Nothing in the right-hand column is recorded as done until a person at the station says
so with a measurement attached.

## Last updated

**2026-09-03** — v3 phases V3-1…V3-9 recorded above (`d4dcc05`…`015876a`, twelve commits:
V3-1 in two, V3-2…V3-8 one apiece, V3-9 in three slices).
Three sections followed, in the same style: **§80** black-box scene preservation (writer
opt-in via `OTA_BLACKBOX_DIR`), **§72** the configuration block an operator may name — roam
region, jog lease, step sizes, auto-track timings and §21's scorer numbers, all of which
may *narrow* what the turret does and never widen it — and **§81** session replay, which
drives ControlLoop itself from a recorded TrackSet stream plus operator actions
(`replay_session()` in `Firmware/control/src/control/session_replay.{hpp,cpp}`) and
`build/replay-session <session.txt>`, with an example at
`Firmware/tools/example_sessions/select_and_lose.txt` and a `ctest` entry that runs the
binary against it.

**The first thing the replay found was a real defect, and it is the kind no unit test had
caught**: `CLEAR_TARGET` while the camera stream was quiet did not stop the aim. The facts
the AUTO_TRACK state machine works from were refreshed when a TrackSet arrived, so with no
frames coming — precisely the degraded moment an operator reaches for CLEAR — the
controller kept being handed the last frame's answer, in which a target *was* selected, and
the axes went on following that line of sight. The transcript said it in one line,
`sel=0 intent=los_direction`. Selection facts are now refreshed every cycle from
controld's own selection (§13/§16), and the transcript asserts the invariant: **nothing is
aimed at while nothing is selected.** Behaviour change for the operator: clearing a target
during a dropout stops the follow on the next cycle instead of the next frame. Frames are
still never invented between detector frames (§58) — `just_reacquired` deliberately stays a
detector-stream event, because a cycle with no frames cannot contain a new acquisition.

`replay-session --config <turret.yaml>` now exists, and to write it honestly the
config→station mapping (`make_control_cfg`, `make_homing_plan`) had to leave `main.cpp` for
`control/src/config/station_wiring.{hpp,cpp}` — shared by controld and the tool. They were
static functions in `main.cpp`, which left a replay with two bad options: duplicate the
mapping and watch it drift, or replay against built-in defaults and produce a transcript
everyone trusts about a station that never existed. Two `ctest` entries now run the same
session twice, once against built-in defaults and once against the repo's `turret.yaml`:
when those two ever disagree, somebody changed a commissioning value that changes what the
operator may do, which is worth a failing test. Camera intrinsics/extrinsics are **not**
read by the tool, so a replay is exact about the machine's limits and approximate about the
camera; the tool's own header line states which replay you are looking at.

Writing that test produced the third visit to the same trap, and this one was the worst:
the band guard compared `confidence_medium_min >= confidence_high_min` while *both were
absent*, `0.0 >= 0.0` is true, so a commissioning file that named `coast_ms` and left the
bands alone was **refused at load — controld would not start**, and nothing in the file is
wrong. Every optional key now has to be optional in the strong sense: naming it must not
make its neighbours mandatory. `V3Config.NamingOneAutoTrackValueDoesNotDemandTheOthers`
holds that, alongside the check that a genuinely inverted pair is still refused.

**§93 (mode switching under active motion) is now tested at the turret**, and the exercise
produced a diagnostic finding rather than a bug: `ModeSwitchingUnderMotion` takes the three
switches the document names — AUTO_ROAM→AUTO_TRACK mid-sweep, AUTO_TRACK→MANUAL while
following, MANUAL jog→AUTO_ROAM with the lease still held — each with the previous mode
moving, and asserts that no cycle demands a step the machine cannot take (measured: 0.05°
per cycle against a 0.6° bound), that the speed ceiling holds, and that **once the new mode's
intent has appeared, the old mode's never comes back**. Getting there meant correcting a
false assumption of my own: `q_ref_pitch_rad`/`q_ref_yaw_rad` in the snapshot are the *goal*
the reference manager is executing, not its interpolated output — mid-sweep they sit at the
far end of the sweep while the turret travels, and after a handover they stay on the old goal
until that ramp lands. So a jump in `q_ref` is not a lurch, and continuity of the commanded
trajectory is v1's `TrajectoryGenerator` contract, tested where it lives.

**Closed the same day, and the diagnosis above was wrong in a way worth keeping.** The
three positions §92 asks for are now published and displayed: `intent_q_pitch_rad` /
`intent_q_yaw_rad` (what the mode asked for) beside the existing `q_ref_*` (the reference the
drive is being told to reach) and `q_*_rad` (where the axis is), with
`intent_has_joint_target` saying whether a pose was asked for at all — the page draws
"— none" rather than a number, and a pytest guard exists because *that* field is only worth
anything if the page consults it. There is no host-side interpolated reference to expose: on
the position-mode path controld sends a target plus `LimitSpd` and **the drive's own position
loop does the ramping** (measured: a steady 35 mrad lag between reference and actual during a
jog, which is a follower, not a curve). So controld cannot publish a trajectory it never
computed, and the honest fix was to publish the wish as well — where *requested ≠ reference*
the envelope clamped it (§33) or the LOS solver declined part of it (§67), which is the one
thing on this list an operator cannot infer from the axis positions.

What remains of that: a hold taken mid-sweep still displays a
reference parked at the far end of the sweep until the ramp lands. That is a property of
position-mode control (the target is the goal; the drive shapes the move), not a defect — but
it is the sort of readout an operator should be told about rather than left to interpret
during a sweep, and §110's acceptance walk is where that happens.

# OpenAutoTurret — Phase Status

**Tracks** [`Firmware/docs/open_auto_turret_software_control_architecture_v1.md`](Firmware/docs/open_auto_turret_software_control_architecture_v1.md).

This file answers one question: **how far along is each phase, counting only what has
been observed doing what it claims.** It carries no feature detail, no file lists and
no measurements — those live in
[`Firmware/docs/AS_BUILT_v1.md`](Firmware/docs/AS_BUILT_v1.md), which is also where the
shipped defects are named. Where the two disagree, `AS_BUILT_v1.md` is the one that was
updated against the running code.

**Location rule:** all production software lives under `Firmware/` (`control/`,
`vision/`, `web/`, `config/`, `calibration/`, `tools/`); the spec's §50 tree is applied
relative to `Firmware/`.

## Phase status (spec §56)

| Phase | Code | On hardware | What the second column does and does not mean |
|---|:--:|:--:|---|
| 0 — Preserve / instrument POC | ✓ | n/a | POC archived under `Firmware/legacy/` |
| 1 — Production CAN/motor layer | ✓ | ✓ | Discovery, feedback, 200 Hz command/response 1:1, all driven by the real homing run. Drive fault injection (queue P5) has never been run |
| 2 — Homing and safety foundation | ✓ | ✓ | First live homing 2026-09-03: 106 s, pitch span 79.9°, yaw 352.8° to real stops. Hold and park observed on every daemon stop |
| 3 — Trajectory generator | ✓ | ◐ | Every live move uses it, and the search sweep held 9.8 °/s against a 10 °/s cap; it has never been profiled against the §44 response metrics on real metal |
| 4 — Vision daemon | ✓ | ✓ | Real IMX500 pipeline verified without CAN or motor. Detector *quality* is not characterised — nothing about detection may be concluded from guarded or synthetic runs |
| 5 — Geometry and estimator | ✓ | ◐ | Proven by replay and simulation. LOS has never been checked against a measured ground truth on the station |
| 6 — Closed-loop tracking | ✓ | ◐ | Cold-start roaming verified live 2026-09-03 (search 1.5 s after `start_tracking` with no target ever seen; swept 104.4…193.2°). Closed-loop tracking of a real target (P8) has not |
| 7 — Installation orientation calibration | ✓ | ✗ | The chain exists end to end and its file formats are now readable by `controld`; it has never been run, because the ChArUco board is not printed |
| 8 — Web UI and diagnostics | ✓ | ◐ | Served and used for today's hardware runs, with real video. Offline load rehearsal 26/26; the live end-to-end checklist (P12) is not signed |
| 9 — Payload profiling / tuning | ✓ | ◐ | The `conservative` profile is in service live and its derate is real; the §44 profiling battery has not been run on the station (P10) |
| Cross-cutting (§54/§55) | ◐ | ◐ | Simulated plant §54.2, replay §54.3, UI load §54.5 and §55 metric extraction exist; the HIL checklist (P13) is open |

`✓` done and observed · `◐` exists and works, verification incomplete · `✗` not run.

The two columns are separate on purpose. A single checkbox per phase is how this file
ended up saying Phase 7 was untouched while its code had been merged for a day: "no"
meant *not verified on hardware* and read as *not built*. If v3 wants one column, it
should pick which of the two it means and mean it.

## v3 — three-mode target tracking (spec §101)

**Tracks** [`Firmware/docs/open_auto_turret_v3_three_mode_target_tracking_architecture.md`](Firmware/docs/open_auto_turret_v3_three_mode_target_tracking_architecture.md).

Same two columns, same rule. **`controld` running on the station today is the pre-v3
binary** — every v3 row below is simulation, and no number of green tests changes the
second column. Deploying v3 means a ~106 s re-home, so it is arranged with the operator,
not done.

| Phase | Code | On hardware | What the second column does and does not mean |
|---|:--:|:--:|---|
| V3-1 — ModeManager, modes, telemetry | ✓ | ✗ | Refused transitions carry real reasons and are asserted in simulation; never observed on metal |
| V3-2 — TrackSet IPC (79 tracks, UUIDs) | ✓ | ✗ | 58 B v1 frames and 2562 B v3 frames share one socket, discriminated by length; controld must be upgraded before visiond |
| V3-3 — TargetSelectionManager | ✓ | ✗ | Selection survives mode changes and clears only on `clear_target`; labels are reused, UUIDs are not |
| V3-4 — AutoTrackController | ✓ | ✗ | Confidence derating and coast verified in simulation. No real person has ever been followed (P8 is still open in v1 too) |
| V3-5 — ManualController | ✓ | ✗ | Jog lease, three profiles, steps. A defect found here: the moving waypoint was built from a buffer zeroed each cycle, so a jog travelled 2.02° and stopped |
| V3-6 — RoamPlanner | ✓ | ✗ | Bounded sweep strictly inside the safe envelope, deterministic over 40 000 cycles. The envelope is derived from the station's own limits, not named in config |
| V3-7 — Mode transitions | ✓ | ✗ | Authority ramps over 300 ms at a handover; §34's "AUTO_TRACK never roams" asserted. Never seen by a person watching metal |
| V3-8 — Reacquisition and ambiguity | ✓ | ✗ | A tie between two candidates does not move the turret. The two refusals are distinct and both are tested: "no candidate is plausible" (long gap) and "two are, and neither is the chosen one" (short gap) |
| V3-9 — Operator surface and diagnostics | ◐ | ✗ | Candidate list, jog row, event feed, §50 fields shipped. The video overlay is not; selection still needs the operator to read a label off the overlay |

Rollout order is controld first, then webd/visiond — the dashboard's v3 row self-disables
when `operating_mode` is absent, and STOP MOTION is never in the disable path. Behaviour
changes an operator should be told about before the first v3 run: AUTO TRACK with no
selection now **holds** instead of searching; STOP MOTION stops **in place** rather than
returning to the ready pose; selection is mode-independent; and `selected_track_id` is the
identity acted on (0 while tracking is off), which is not the same field as
`selected_display_index`, the label that was typed.

## What has no phase, and bites anyway

Two shipped defects are named in `AS_BUILT_v1.md` rather than tracked as phases,
because they will otherwise be inherited as assumptions: the CyberGear mode-entry
recipe sleeps on the 200 Hz control thread (≈110 ms cycles, answered by `BRAKE`), and
the dashboard's speed readout spikes to ~2.8× the axis's real speed at a sweep
reversal.

**v3 inherited both, unchanged.** The recipe sleep is four `sleep_for(50 ms)` calls
inside `can_motor_backend.cpp`'s mode-entry recipes, invoked from homing on the control
thread; 28 cycles of 109–113 ms per homing run are the measurement. v3's `BRAKE`
answer is the same one, which means v3's mode machinery is currently being *exercised*
by a defect rather than by design: a station that is braking during homing is a station
whose 200 Hz assumptions are not holding. It is listed here rather than in a V3 phase
because it predates v3 and fixing it touches the code that moves real metal, which makes
it the operator's call about risk, not mine about convenience.

## §110 acceptance — what has to be measured, and by whom

v3's checklist cannot be closed from this repository, and pretending otherwise is how
the old `enable_search` checkbox survived. The items below need a person at the station;
each row says what is already known from simulation and what only metal can answer.

| Item | Known in simulation | Needs an operator because |
|---|---|---|
| First v3 homing + mode entry | v3 does not change homing | The re-home takes ~106 s and moves the turret; nobody is at the station now |
| Following a real person (P8, v1 and v3 alike) | Track → LOS → joint solve holds in sim; derating and coast asserted | Detection quality is uncharacterised. "It tracked" is a statement about the detector, not the controller |
| Ambiguous reacquisition (§21) | A tie does not move the turret; the event is recorded with both scores | Only a person can say whether the two candidates *looked* ambiguous from the room |
| Jog lease under a real tab death (§38) | Renewal at 100 ms against a 300 ms lease, asserted; expiry recorded as an event | Wifi drops, browser freezing, and a phone going to background are not reproducible from a test |
| Sweep inside the safe envelope (§33) | Deterministic over 40 000 cycles; envelope strictly inside derived limits | The envelope is derived from limits that have never been checked against a measured ground truth (Phase 5, `◐`) |
| STOP MOTION in place (§27) | Stops in place; no return-to-ready; asserted as a distance | Whether "in place" is where the operator wanted it to stop is a judgement about the room, not about the code |
| Handover continuity (§36/§44) | 300 ms authority ramp; excess decays; asserted per axis | A person has to say whether the motion *looked* continuous. A number inside a limit can still look wrong |
| Operator-visible diagnostics (§50/§78/§79) | Fields published, guarded, and rendered; events on the feed | Whether the panel answers the question the operator actually asks is only knowable by watching them use it |

Nothing in the right-hand column is recorded as done until a person at the station says
so with a measurement attached.

## Last updated

**2026-09-03** — v3 phases V3-1…V3-9 recorded above (`d4dcc05`…`015876a`, twelve commits:
V3-1 in two, V3-2…V3-8 one apiece, V3-9 in three slices).
Three sections followed, in the same style: **§80** black-box scene preservation (writer
opt-in via `OTA_BLACKBOX_DIR`), **§72** the configuration block an operator may name — roam
region, jog lease, step sizes, auto-track timings and §21's scorer numbers, all of which
may *narrow* what the turret does and never widen it — and **§81** session replay, which
drives ControlLoop itself from a recorded TrackSet stream plus operator actions
(`replay_session()` in `Firmware/control/src/control/session_replay.{hpp,cpp}`) and
`build/replay-session <session.txt>`, with an example at
`Firmware/tools/example_sessions/select_and_lose.txt` and a `ctest` entry that runs the
binary against it.

**The first thing the replay found was a real defect, and it is the kind no unit test had
caught**: `CLEAR_TARGET` while the camera stream was quiet did not stop the aim. The facts
the AUTO_TRACK state machine works from were refreshed when a TrackSet arrived, so with no
frames coming — precisely the degraded moment an operator reaches for CLEAR — the
controller kept being handed the last frame's answer, in which a target *was* selected, and
the axes went on following that line of sight. The transcript said it in one line,
`sel=0 intent=los_direction`. Selection facts are now refreshed every cycle from
controld's own selection (§13/§16), and the transcript asserts the invariant: **nothing is
aimed at while nothing is selected.** Behaviour change for the operator: clearing a target
during a dropout stops the follow on the next cycle instead of the next frame. Frames are
still never invented between detector frames (§58) — `just_reacquired` deliberately stays a
detector-stream event, because a cycle with no frames cannot contain a new acquisition.

`replay-session --config <turret.yaml>` now exists, and to write it honestly the
config→station mapping (`make_control_cfg`, `make_homing_plan`) had to leave `main.cpp` for
`control/src/config/station_wiring.{hpp,cpp}` — shared by controld and the tool. They were
static functions in `main.cpp`, which left a replay with two bad options: duplicate the
mapping and watch it drift, or replay against built-in defaults and produce a transcript
everyone trusts about a station that never existed. Two `ctest` entries now run the same
session twice, once against built-in defaults and once against the repo's `turret.yaml`:
when those two ever disagree, somebody changed a commissioning value that changes what the
operator may do, which is worth a failing test. Camera intrinsics/extrinsics are **not**
read by the tool, so a replay is exact about the machine's limits and approximate about the
camera; the tool's own header line states which replay you are looking at.

Writing that test produced the third visit to the same trap, and this one was the worst:
the band guard compared `confidence_medium_min >= confidence_high_min` while *both were
absent*, `0.0 >= 0.0` is true, so a commissioning file that named `coast_ms` and left the
bands alone was **refused at load — controld would not start**, and nothing in the file is
wrong. Every optional key now has to be optional in the strong sense: naming it must not
make its neighbours mandatory. `V3Config.NamingOneAutoTrackValueDoesNotDemandTheOthers`
holds that, alongside the check that a genuinely inverted pair is still refused.

**§93 (mode switching under active motion) is now tested at the turret**, and the exercise
produced a diagnostic finding rather than a bug: `ModeSwitchingUnderMotion` takes the three
switches the document names — AUTO_ROAM→AUTO_TRACK mid-sweep, AUTO_TRACK→MANUAL while
following, MANUAL jog→AUTO_ROAM with the lease still held — each with the previous mode
moving, and asserts that no cycle demands a step the machine cannot take (measured: 0.05°
per cycle against a 0.6° bound), that the speed ceiling holds, and that **once the new mode's
intent has appeared, the old mode's never comes back**. Getting there meant correcting a
false assumption of my own: `q_ref_pitch_rad`/`q_ref_yaw_rad` in the snapshot are the *goal*
the reference manager is executing, not its interpolated output — mid-sweep they sit at the
far end of the sweep while the turret travels, and after a handover they stay on the old goal
until that ramp lands. So a jump in `q_ref` is not a lurch, and continuity of the commanded
trajectory is v1's `TrajectoryGenerator` contract, tested where it lives.

**Closed the same day, and the diagnosis above was wrong in a way worth keeping.** The
three positions §92 asks for are now published and displayed: `intent_q_pitch_rad` /
`intent_q_yaw_rad` (what the mode asked for) beside the existing `q_ref_*` (the reference the
drive is being told to reach) and `q_*_rad` (where the axis is), with
`intent_has_joint_target` saying whether a pose was asked for at all — the page draws
"— none" rather than a number, and a pytest guard exists because *that* field is only worth
anything if the page consults it. There is no host-side interpolated reference to expose: on
the position-mode path controld sends a target plus `LimitSpd` and **the drive's own position
loop does the ramping** (measured: a steady 35 mrad lag between reference and actual during a
jog, which is a follower, not a curve). So controld cannot publish a trajectory it never
computed, and the honest fix was to publish the wish as well — where *requested ≠ reference*
the envelope clamped it (§33) or the LOS solver declined part of it (§67), which is the one
thing on this list an operator cannot infer from the axis positions.

What remains of that: a hold taken mid-sweep still displays a
reference parked at the far end of the sweep until the ramp lands. That is a property of
position-mode control (the target is the goal; the drive shapes the move), not a defect — but
it is the sort of readout an operator should be told about rather than left to interpret
during a sweep, and §110's acceptance walk is where that happens.

**§73's overlay is wired, and it is wired as a data consumer.** Candidate boxes now travel
on the wire (`bbox`, normalised to the detector's frame), and the page draws them over the
MJPEG picture from the telemetry snapshot alone: no interpolation between snapshots, no
remembering the last box to make motion look smooth. A box drawn where the *controller* says
the target is can be compared with the picture and disagrees honestly when the two disagree;
a box the page smoothed into place is decoration. The staleness rule is the candidate list's,
drawn on the picture ("target list is N ms old — boxes are where controld last saw them"),
`OCCLUDED`/`LOST` are dashed, the selected candidate is amber, an out-of-frame anchor becomes
an edge cue rather than silence, and an absent `bbox` draws a crosshair labelled "anchor only"
— because four zeros would put a confident box in the corner of every frame against an older
controld. A pytest guard asserts all of that as properties of the *function*, including that
the canvas is cleared when the video is off and that it cannot swallow clicks.

**The reticle came next, and it is computed where the geometry lives.** controld projects the
commanded line of sight — the estimator's state predicted to the actuation time, §13.3, which
is the same LOS an AUTO_TRACK intent is built from — through the inverse gimbal transform and
the §28.2 intrinsics, and publishes it normalised exactly like a track anchor. The inverse
(`base_to_ray`) is written as the transpose of the existing `ray_to_base` product rather than
as three negated angles, so it cannot drift from the forward transform when the camera
extrinsic changes; the test for it is a round trip over five gimbal poses and sixty pixels
against the transform already trusted, plus the case with an answer nobody has to compute (a
ray down the optical axis projects to the principal point, at every joint angle). Then a
loop-level test follows a target all the way through — pixel → ray → base LOS → back to a
pixel — and asserts the marker lands within 5% of the box it is following, because a reticle
a few degrees off would look exactly like bad tracking, which is the worst failure a
diagnostic can have: it makes the controller look wrong when the drawing is.

Four conditions must hold before there is an answer to publish, each a different reason for
silence: an estimate must exist (the last aim point of a session that ended is not where the
turret is pointing now); the intrinsics must describe the **same picture the detector
reported** — if they disagree, projecting anyway puts the reticle a few per cent off the mark
the operator is being asked to trust, and a small systematic offset reads as controller error
rather than as configuration, so the loop says nothing instead (asserted both ways); the ray
must be in front of the camera, because `ray_to_pixel` answers a ray behind the camera with
the principal point, which would draw an astern target dead centre on the very mark that
means "here is where we are aiming"; and the result must be inside the frame — an off-screen
aim point belongs to the edge cues, not to a reticle painted on the bezel.

On the page the reticle is drawn *outside* the staleness fade that dims the boxes, and a
pytest guard asserts that ordering: the boxes are vision's and go stale when vision is quiet,
but the aim point is controld's own, live at 200 Hz, and the moment it matters most — nothing
detected, the turret still aimed at where it last believed somebody was — is the moment a fade
would erase the only live thing on the overlay.

What §73 still does not have: a **predicted target box**. A box has a size, and the size of a
target two control periods from now is the detector's business, not controld's — there is no
honest way to grow a bbox forward in time without a model of the target's shape and distance.
The reticle is what can be said truthfully: where the turret is aiming.

And the limit that matters for this one: **the overlay has never been rendered against a
real camera frame.** It is guarded by parse checks and by assertions about where its numbers
come from; nobody has looked at an IMX500 picture with these boxes on it. Alignment between
the detector's frame and the streamed picture (crop, aspect, orientation) is exactly the kind
of thing only a person notices, and it belongs to §110 with the operator.

**v3's cost per cycle has now been measured — in simulation, on this host, which is what
that sentence does and does not buy.** `tools/cycle-cost` times `ControlLoop::step()` around
the call and reports p50/p95/p99/worst per condition (quiet hold, leased manual jog,
AUTO_TRACK at 30 Hz of detector traffic, AUTO_ROAM sweeping, homing), and
`ctest -R cycle_cost_gate` runs it as a gate. Two runs of 2000–3000 cycles each: **every
condition landed at p99 ≤ 4 µs against a 5000 µs period, with zero cycles over the period and
zero over the §39.3 grace.** The spread between conditions (hold 1.3–2.1 µs p50, tracking
2.0–3.2) is inside the run-to-run spread of the *baseline itself*, so the honest statement is
that the three modes are indistinguishable from standing still at this resolution — not that
roam is cheaper than tracking. Nothing here is a station measurement: the sim backend answers
instantly, so a real cycle also pays the CAN exchange, and the pre-v3 figure of 5.06 ms p50 /
5.09 ms p99 remains the only number anyone has measured on the Pi. The gate's ceiling is
deliberately absurd (p99 < 4000 µs) because what it is for is a factor of ten — an
allocation, a file write, a socket appearing on the control path — not 40 µs of laptop
jitter. The two informational rows are not gated, and the first version of
this tool got one of them wrong in a way worth keeping: it blamed a 90–150 ms worst cycle on
homing's §46 recipe sleeps — and **the sim backend has no recipe sleeps**, because those live
in `can_motor_backend`, which this run never touches. Adding a 200-cycle idle row *in front
of* homing moved the spike into it, so the real cause is a once-per-process ~100 ms cost (the
first spdlog write, which in this tool happens when homing first trips the watchdog and
preserves a black-box scene). controld logs long before homing on the station, so the
practical consequence there is probably small — but the rule is general and this tool now
encodes it: **a one-time cost is charged to whoever runs first**, so the tool runs idle
cycles before it begins measuring anything. Single-cycle outliers of the same kind show up in
AUTO_TRACK (p99 2.6 µs, worst 113 µs in one cycle in 600) and are first-touch, not a trend.
What §46's sleeps *are* remains what metal said: 109–113 ms homing cycles on the CAN path,
unchanged by v3, still waiting on an operator's risk decision.

What that same run turned up, and what got fixed: **the first black-box scene this project
ever preserved named no operating mode.** A preserved scene copies the published view, and
when homing's recipe sleep trips the watchdog during homing, the view it copies had no mode
in it — so the artifact read "mode: ⌀", which nobody reading an investigation record will
ever hear as "not yet published". They will hear "it was in no mode". `preserve_scene` now
falls back to the loop's own authority for the two fields that can be empty, and says so in
the code; `BlackBox.AScenePreservedBeforeAnythingWasPublishedStillSaysWhatModeItWasIn`
asserts it, including that the record reaches the published view on the next cycle rather
than only existing privately.

**§110 now has a place to be answered, and it is not the document.** `tools/v3_acceptance.py`
parses the checklist out of the architecture document — so the report can never certify a copy
that drifted from the list — and records evidence in
`Firmware/docs/acceptance/v3_acceptance_log.json`. The checkboxes in the document are
deliberately **not** read for status: a box in a markdown file can be ticked by anyone with a
text editor, and the person most likely to tick one optimistically is the one who wrote the
code and wants it finished. An item reaches `passed` only with a named human, the station it
was observed on, and a sentence about what was seen; `--method simulation` cannot produce
`passed` under any argument, it produces `simulated`, which is what it is; an entry records the
item's wording, so if §110 is edited later the report flags that entry rather than carrying the
stamp forward against text that did not exist when it was accepted. `attach` reads one
`/api/state` snapshot as evidence and refuses any other URL — there is a test that the tool
never addresses the command endpoint, because an acceptance runner that can move the turret is
not a runner, it is an operator, and the operator in this project is a person standing where the
arms cannot reach them.

Where that leaves §110 today, from `report`: **30 items — 0 accepted on hardware by a named
person, 19 shown by simulation only, 11 untried.** The 11 untried are worth reading as work,
not as paperwork, because most are gaps in *simulation* coverage too: whether detections keep
arriving while roaming (AUTO_ROAM/4), whether a selection persists across a roam stretch
(/5), whether roaming refrains from pursuing a selected target by default (/6 — that one may be
an implementation gap rather than a test gap, and needs reading the code before writing the
test), whether switching into AUTO_TRACK acquires an already-selected visible target (/7), the
FINE/NORMAL/FAST profiles as behaviour rather than as telemetry strings (MANUAL/3), travel
limits under manual (MANUAL/5), target-unreachable and target-switch constraints
(AUTO_TRACK/9, /10), turnaround constraints (AUTO_ROAM/2), that no UI or vision process can
write motors (COMMON/4 — statically checkable and worth a guard), and the IMU-conditional frame
(MANUAL/7, which waits on hardware that may never arrive).

Two habits those rounds changed here: an unset config field is never expressed as `0` (a
draft wrote the ambiguity margin to 0.0 on every station that named nothing, which would
have switched §21's no-target-steering off by default — caught by the event test, not by
review), and the counts below are recounted by running the suites rather than copied
forward — §81's commit message claims "54/54" and is wrong; these are the real figures.
Evidence as of now, all of it simulation: 54 CTest binaries green (including the cycle-cost gate), 267 pytest green, and
the guards in `Firmware/web/webd/tests/` that parse controld's own source and this
document — command vocabulary, step sizes, the jog-lease ratio, and §79's event list —
because a copied list keeps certifying the world as it used to be. The loop's measured
5.06 ms p50 / 5.09 ms p99 at 200 Hz belongs to the **pre-v3** binary the station is
still running; v3's cost per cycle has not been measured on metal, and the recipe sleep
above is a known 109–113 ms excursion that no v3 measurement will hide.

Same day earlier: cleaned for the next architecture revision. The detailed per-phase
item lists moved to `Firmware/docs/AS_BUILT_v1.md`; the session log and the six items
still open moved to `Firmware/docs/archive/progress_before_v3.md`. Same day before that:
first live homing, first live cold-start roaming, and `enable_search` made to actually do
something.
