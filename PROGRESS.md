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

## What has no phase, and bites anyway

Two shipped defects are named in `AS_BUILT_v1.md` rather than tracked as phases,
because they will otherwise be inherited as assumptions: the CyberGear mode-entry
recipe sleeps on the 200 Hz control thread (≈110 ms cycles, answered by `BRAKE`), and
the dashboard's speed readout spikes to ~2.8× the axis's real speed at a sweep
reversal.

## Last updated

**2026-09-03** — cleaned for the next architecture revision. The detailed per-phase
item lists moved to `Firmware/docs/AS_BUILT_v1.md`; the session log and the six items
still open moved to `Firmware/docs/archive/progress_before_v3.md`. Same day: first live
homing, first live cold-start roaming, and `enable_search` made to actually do
something.
