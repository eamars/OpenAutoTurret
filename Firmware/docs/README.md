# Documentation map

Every operation has exactly one card, and every card is reachable from this page in one hop. If you
cannot find an operation here, it is undocumented — say so in your report rather than improvising a
procedure from a flag list.

## Start here, by what you are about to do

| I am about to… | Read |
|---|---|
| put new source onto the station | [`operations/deploy.md`](operations/deploy.md) — including **where compilation happens**, which is the question every new operator gets wrong |
| start / stop / inspect the running stack | [`operations/start-stop-status.md`](operations/start-stop-status.md) |
| prepare the station's OS (fresh install, new SD card, new operator, or controld logs `SCHED_FIFO ... refused`) | [`operations/os-setup.md`](operations/os-setup.md) — the one sudo step, which the owner runs; which threads are real time and why |
| read what the station did (logs, traces, evidence) | [`STATION_OPERATIONS.md`](STATION_OPERATIONS.md) §"Reading the per-cycle control trace", §"Stop and preserve evidence" |
| run a bounded motor / IMU / pitch commissioning session | [`STATION_OPERATIONS.md`](STATION_OPERATIONS.md) §"Bounded commissioning, without automatic startup" |
| commission or re-commission the yaw / pitch servo (after a payload, camera, weight, bearing or motor change) | [`operations/servo-commissioning.md`](operations/servo-commissioning.md) — one automatic command per axis: identify, design, verify on the station, write the asset |
| work on camera tracking (ADR-003: target estimator, Level-1 reference, its stages and status) | [`ADR-003/IMPLEMENTATION.md`](ADR-003/IMPLEMENTATION.md) — the plan, the decisions taken against the architect's package, and the state of each stage; stage 1 is `tools/tracking/tracking.py stage1` |
| plan or record a design decision | a new `ADR-NNN/` directory (see "New document convention" below) |
| look up a legacy document | [`archive/README.md`](archive/README.md), which indexes them by implementation state and states each one's limits |
| look up a vendor protocol or manual | [`references/`](references/), grouped by device |

## The operating runbook

[`STATION_OPERATIONS.md`](STATION_OPERATIONS.md) stays at that path because the repository's
`AGENTS.md` requires it, and it remains authoritative for **current state, safety history, owner
rulings and measured limits**. Operational *procedure* lives in `operations/` so that it can be read
in one sitting without wading through history; where the two overlap, the runbook holds the facts
that justify the procedure.

## Cards

| Card | Covers |
|---|---|
| [`operations/README.md`](operations/README.md) | the card index and the fixed skeleton every card follows |
| [`operations/deploy.md`](operations/deploy.md) | build location, the two different things named "deploy", the three things a handover needs, and what to do on a host with no cross-toolchain |
| [`operations/start-stop-status.md`](operations/start-stop-status.md) | launcher actions, run dir, what `stop` proves, what it does not |
| [`operations/os-setup.md`](operations/os-setup.md) | the real-time grant, CPU split and UI-last priorities; the per-thread table; how to verify it took |

## Verify the tree

```bash
.venv/bin/python Firmware/tools/doc_tree_check.py     # or: python3, it uses only the standard library
```

It fails if any card is unreachable from this page, if any relative link in `docs/` is broken, or if
the runbook has moved off the path `AGENTS.md` promises. Run it after touching any document.

## New document convention

`ADR-001/` is the first development plan in the new convention. The next new project document starts
at `ADR-002/`; continue with monotonically increasing `ADR-NNN/` directories. Keep each new plan or
decision and its supporting material inside its own ADR directory. Do not add new dated project
reports to the legacy root or archive folders. Operational procedure is not a dated report: it goes
in `operations/`, and a procedure change is edited in place rather than appended as a new file.

## Legacy archive status

All pre-ADR project documents have been filed under `archive/` by their dominant implementation
state. The archive index lists each item and its limits. In particular, "implemented" means the
documented software or procedure existed; it does not establish physical acceptance on the current
station. "Partially implemented" records mixed progress or outstanding qualification. "Not
implemented" identifies a legacy proposal whose planned result was not delivered in that form.
"Superseded" marks retired hardware, old architecture, or stale status and operating material.
