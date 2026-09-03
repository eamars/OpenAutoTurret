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
Two habits those rounds changed here: an unset config field is never expressed as `0` (a
draft wrote the ambiguity margin to 0.0 on every station that named nothing, which would
have switched §21's no-target-steering off by default — caught by the event test, not by
review), and the counts below are recounted by running the suites rather than copied
forward — §81's commit message claims "54/54" and is wrong; these are the real figures.
Evidence as of now, all of it simulation: 51 CTest binaries green, 257 pytest green, and
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
