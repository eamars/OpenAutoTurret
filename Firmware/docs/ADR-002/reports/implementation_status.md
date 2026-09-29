# ADR-002 implementation and acceptance ledger

Updated 2026-09-30 (runtime surface section by 小满). Work is in progress; no physical qualification is claimed.

## Identity and authorized scope

- Source baseline: `main@2e789c59dc9038228ac2aa91caab0f07d26d8137`, initially clean.
- Observed running release: `90aa1f5d4c9155310940813e3933ad49fa36901c`, directory
  `/home/eamars/workspace/OpenAutoTurret/run/releases/90aa1f5d4c91.uZvIsv`.
- Pi checkout HEAD is separately `6a47f1dd696d75b878b8138dcaceb9f455ab9147`.
- Initial live state: MANUAL / READY / hold, no fault. Preserve this operator intent.
- Owner authorized implementation, deployment and bounded motor/encoder/IMU trials.
  Actual pitch payload is **camera plus Raspberry Pi**, not an empty axis.
- Owner requested functional verification first, thermal testing last using motor
  reports; no external thermometer is available. Active cooling is a possible
  later hardware change, not present qualification.
- Owner's subsequent execution rule: **never roll back after a build or release
  failure**. The Pi has no critical service. Preserve failure evidence, fix the
  defect and deploy the next version; do not spend time restoring older releases.
- Unchanged initial limits: yaw current `0x1FE`, 0.8 A, Kp 1 A/(rad/s), Ki 0.6 A/rad;
  pitch LimitCur <=5 A; host 200 Hz. No new calibrated gains or friction values.

## Acceptance matrix

| Capability | Source/offline | Single-axis physical | Thermal/two-axis | Current scope |
|---|---|---|---|---|
| Output arbitration/recovery | PR1 native PASS; Pi 80/80 | Ordinary-TX continuity observed; physical fault injection NOT_RUN | NOT_RUN | 4387411 Manual commissioning |
| Yaw current velocity loop | PR2 math replay and native PASS | Baseline measured; latency/smoothness targets not met | NOT_RUN | Existing 0.8 A bound |
| Yaw friction compensation | Bounded state/current integration tests PASS | NOT_RUN | NOT_RUN | Disabled until calibration |
| Pitch native speed tuning | Hot update PASS (§6: prepare→apply→read-back→restore against drive registers) | NOT_RUN | NOT_RUN | Existing native mode and 5 A bound |
| Pitch homing | Existing native/Pi tests PASS | Two starts completed; repeatability not yet reduced | NOT_RUN | Do not inherit old mechanism results |
| Typed payload qualification | Profile binding PASS (a campaign bound to one profile refuses to run under another) | NOT_RUN | NOT_RUN | No new qualified profile |

## PR1 evidence so far

The public `gm6020::VelocityLoop` probe was compiled and executed with WSL G++.
At timestamps 6/31/36 ms with constant fresh-position input, baseline output was
0.1003/0/0 A and validity was true/false/false. The thin fix gave
0.1003/0.1003/0.1006 A and remained valid: the late cycle freezes integration,
then normal integration resumes. This verifies the deterministic PI boundary;
it does not verify Linux/CAN timing or physical response.

Changes under test:

- No-progress is a performance episode; the guard no longer inserts routine
  zero-current frames. Emergency trips latch under the same mutex as normal TX.
- Current link state and error deltas replace cumulative error-count latching.
  Yaw command health uses its own bus; sustained TX failure is bounded at 20 ms.
- Watchdog action identifies the affected axis, retaining the healthy axis's
  controlled support instead of automatically disabling both axes.
- Trace records carry RX sequence/raw encoder/current, successful TX sequence/time,
  encoded output, requested output, command kind/reason and effective PI values.
  Transport success is not a motor acknowledgement. Pitch host current command
  remains unknown; its command is SpdRef in rad/s.
- Raw RX timestamps are preserved separately from the legacy cycle-bounded
  supervisor timestamp, and the trace exports raw RX time.
- Legacy mixed-station payload loading is explicitly unqualified; the historical
  file remains a conservative cap while typed identity binding is implemented.

Offline package tests: 45 passed. Synthetic metrics and illustrative arithmetic
ran successfully. These tools establish no mechanism stability.

The old running release was observed read-only for 10.529 s (2,080 unique control
cycles). Average reported host period was 5,064 us. Pitch's trace-visible RX
updates averaged 55.59 Hz, with feedback age 3.45/11.74/24.37 ms min/mean/max.
Yaw feedback age was 0/0.49/2.48 ms; its trace-visible update rate of 197.46 Hz is
limited by controller sampling and **is not the true CAN RX rate**. Yaw raw
temperature was 33 (unverified degrees scaling); pitch temperature was absent
from that trace. Peak-to-peak encoder position range was 0.1093 degrees pitch
and 0.04395 degrees yaw. Raw capture is ignored at
`run/adr002/baseline-90aa/trace-10s-and-imu.txt`; the stack stayed MANUAL/READY.

Native builds use WSL Ubuntu, GCC 15 and CMake 4.2, in ignored
`run/adr002/native-build`. Full compilation succeeded. The final native suite
`ctest --test-dir run/adr002/native-build -E retained_homing --output-on-failure`
passed 78 of 79 entries initially; the remaining watchdog-event test expected
no-progress to outrank a lost heartbeat. Its assertion now requires the actual
fatal condition first, then no-progress after the heartbeat condition clears.
`cmake --build run/adr002/native-build --target test_watchdog_trip_events --parallel "$(nproc)"`
and `ctest --test-dir run/adr002/native-build -R "^test_watchdog_trip_events$" --output-on-failure`
passed. Combined result: 79/79 entries, including nine new recovery/arbitration
cases executing the production backend and control loop. Evidence logs:
`run/adr002/native-ctest-final.log`, `native-watchdog-rebuild.log`, and
`native-watchdog-rerun.log`. Earlier assertion-only failures are superseded;
neither simulated energized state nor mocked transport establishes physical hold.
No retained-homing test is claimed: the native suite excludes it as directed by
AGENTS because that test writes `/dev/shm`.

PR1 source commit: `02d5e8ce120eccda2a755b8815dbbb10ab4dedb8`.
An inactive release was created at `run/releases/02d5e8ce120e.yA9dLm` on the Pi.
Its native compile was an execution mistake: the owner had cleared WSL resources
specifically for local compilation. After the owner's correction, all build
processes with that exact release working directory were stopped; a subsequent
process check found none remaining. No new controller was activated. Preserve
this incomplete directory as evidence; it is not a usable/verified release.
The route is now **WSL native tests and ARM64 cross-compilation only**, then
binary transfer and station tests. Missing cross dependencies are to be installed
locally, not used as a reason to compile on the Pi again.

A real local Unix SOCK_SEQPACKET probe found a missing PR1 integration case:
256 live rows transmitted (188,615 bytes), but the 1024-row frozen fault window
(753,862 bytes) failed with EMSGSIZE under the old 256 KiB sender buffer.
The follow-up sizes the sender for 2 MiB and reuses the shared MAX_FRAME in the
response capture client. Repeated real socket probe passed both 256 rows
(188,615 bytes) and 1024 rows (753,862 bytes). The rebuilt `test_web_server`
passed, including a full frozen-window regression over a real packet socket;
logs are `run/adr002/trace-socket-build.log` and `trace-socket-test.log`.
No activation, calibrated parameter change, or hardware acceptance is included
in the native gate above.

## Local cross-build and inactive deployment

WSL Ubuntu hosts a signature-verified Debian 13 amd64 chroot at
`/home/rba90/.cache/ota-adr002/debian13-verified`, with the workspace bound at
`/workspace/OpenAutoTurret`. AArch64 GCC 14.2 and fmt 10.1.1, spdlog 1.15.2,
yaml-cpp 0.8 and GTest 1.16 are installed there. The first cross attempt needed
the missing `make` package for GTest bootstrapping; installing it and building
forward succeeded. No rollback was performed. The rootfs target libc is 2.41
with a newer Debian security patch than the Pi; target execution is verified
by the station tests, not inferred from that version similarity.

`cross_build.py` succeeded on `8838b1d62186dd106f9b47800e8d642ae9e3e16f`.
Its reported 152 ELF artifacts include build objects and are not a test count.
`deploy_station.py --prebuilt --commission-mixed-controller` shipped the local
build into `/home/eamars/workspace/OpenAutoTurret/run/releases/8838b1d62186.n6vz4e`.
The station ran 67 test binaries with zero failures and passed the manual
mixed-controller CAN/IMU preflight. The 13 additional CMake integration entries
were then run from relocated CTest metadata: 13/13 passed, 33.37 seconds.
All 80 registered entries, including retained homing, are therefore covered.
The deployment path now uses the complete CTest manifest rather than a filename
glob. Its two focused local regressions and seven real relocated CTest probes
passed; two pre-existing detached-process launcher tests timed out in WSL.
Logs: `pi-ctest-missing-13.log`, `pr1-cross-build.log`
and `pr1-deploy-final.log` under ignored `run/adr002/`.

## Old release incident before activation

The old `90aa1f5` release independently entered watchdog fault at 22:38:01 while
the mistaken Pi build was underway. Its frozen trace ends at a 21.108 ms
hold/DERATE cycle with fresh yaw feedback; about 90 ms later the log records
fault and an 84.487 ms cycle. This is consistent with the old late-cycle failure
path but does not establish scheduling attribution or the exact first trigger.
Captured evidence is in `run/adr002/old-release-trip/`; analysis is in
`run/adr002/analysis/old-release-trip-report.md`. At 22:47 the launcher stopped
the old stack and cleaned up its processes. It reported `STOP FAILED` because
the controller was already faulted; logs confirm STOP/zero requests and clean
process exit, not normal parking qualification or confirmed GM disable.

## Failure handling (owner override)

Release `8838b1d62186.n6vz4e` was started through the launcher in Manual mixed
commissioning at 23:02 NZDT. Pitch completed mandatory homing and reached hold.
Actual pitch readbacks: RunMode 2, LimitCur 5 A, SpdKp 4, SpdKi 0.05.
Yaw current-mode configuration remains 0x1FE, 0.8 A, Kp 1 A/(rad/s), Ki 0.6 A/rad.
The final 12-second stationary window measured yaw RX 1000.04 Hz and pitch
49.50 Hz, feedback-age p99 0.9992/19.73 ms respectively, host period p99 5.065 ms.
Yaw successful TX sequence advanced every cycle with ordinary reason; one raw
RX age was -0.834 microseconds (RX arrived after the cycle clock sample).
Pitch temperature was 25.9 C; yaw raw byte 31 has unverified degree scaling.
Pi throttling flags 0x50000 indicate historical events, no active low-bit flags.
Captures and analysis reside under ignored `run/adr002/8838` and `analysis`.

The first yaw fine jog was accepted but moved only about 0.75 degrees in twelve
seconds: this is a baseline observation, not a performance pass. An existing
response_probe was rejected because it required physical two-axis homing even
for continuous yaw. Its gate now uses position readiness and the declared
runtime envelope (including explicitly unbounded yaw); bounded-axis clearance
and stationary checks remain. The production command-gate simulation passed.

Future releases use locally cross-built
committed source and a separate release directory, with launcher-controlled
stop/start and manual commissioning. Build/deployment failures are fixed forward:
preserve diagnostics and publish the next corrected version, with no rollback.
This owner rule overrides the ADR's generic rollback procedure. Retained geometry
remains intact. A failed physical trial still preserves traces before the ring
wraps and uses the launcher for stopping as necessary; it does not trigger
restoration of an old release. Do not switch GM back to voltage, overwrite
calibration, or interpret a GM zero-current request as confirmed disable.

## PR2: executable path and bounded compensation

Source now sends service yaw through the shared position-P / velocity-feedforward
path. Previously finalizing pitch homing selected yaw's host position interface
unconditionally. Leased yaw jogs retain their explicit velocity even while their
position waypoint is bounded relative to feedback. The pitch service path is
unchanged in this PR2 slice. PI gains and the 0.8 A / 5 A caps are unchanged.

The new directional friction state machine defaults disabled. Explicit moving
intent permits one bounded attempt per direction; repeated lease refreshes do
not restart it. Directional fresh-RX displacement confirms motion; reversing
waits for stationary feedback. Timeout is a performance observation. Final
current and slew limits govern PI integration, transition handoffs use the last
delivered effort, quiet hold retains its integral, and a reduced current cap
retains signed braking authority. Unknown calibration is not promoted to a
measured production profile. New trace fields expose assist/state/exhaustion.

A fixed 128-sample raw-RX history supports 20/30/40 ms measurement windows and
ignores repeated/backwards timestamps. Default estimator selection remains the
legacy 50 ms filter until physical A/B data select a window. All three candidate
window observations are temporarily traced to compare against actual CAN RX.

Before regression expansion, standalone WSL C++ probes replayed the captured
8838 normal-jog samples through the production estimator and combined current
loop. The latter preserved finite <=0.8 A output and exactly one start attempt
after a tiny reference-sign crossing was given a direction deadband. These
replays use recorded feedback, not a simulated claim about changed mechanics.
The native suite passed 79/80 entries initially; two new test expectations
observed their 10 ms attempt after its deadline. After correcting those test
intervals, the rebuilt transport tests passed. The combined native result is
80/80 CTest entries excluding retained homing. Final focused current/estimator,
parser and transport tests passed 3/3; `pr2-ctest.log` and
`pr2-final-focused-tests.log` retain the evidence and initial failure.

Release `4387411e8be9.PLkMoF` passed all 80 station CTest entries and preflight,
then completed its mandatory pitch homing. Five +/- normal yaw pairs were
captured at the initial pitch pose, followed by three accepted +5-degree pitch
responses and another bounded yaw series. Baseline fine +/-3 deg/s requests
showed roughly 1.521 s / 9.713 s three-count motion-confirmation delays; positive
normal motion used up to 0.632 A and drifted about 0.395 degrees after stop.
The baseline therefore does not meet response/hold targets. Every observed
active yaw row had ordinary output reason and advancing successful TX sequence;
this confirms no interleaved guard zero in those trials, not emergency-stop
qualification. Detailed raw captures/analysis remain ignored under `run/adr002`.

## Stop clock correction and session-only yaw trials

Release `eaef375f9046.bzBrVq` was cross-built in local WSL; all 81 registered
station CTest entries and preflight passed. It was not started. Stopping the
previous `4387411` session at 23:35 NZDT exposed a readiness defect: shutdown
passed the previous cycle clock to a newer yaw RX snapshot, so 5.145 ms of
future feedback was rejected before the fresh-clock check. A production-backend
WSL probe reproduced `old_clock_feedback=0 q=nan fresh_clock_feedback=1 q=0.4`.
Shutdown now obtains its snapshot clock at entry. An accepted unverified stop
also no longer resumes ordinary Hold for the 55-second parking budget: only an
actual Parking phase runs that state machine. The old session exited at 23:36:53
with STOP FAILED; this is not counted as parking qualification. The station
remained stopped while the next fix-forward release was prepared. No rollback
was performed or is authorized after build/deployment failures.

`yaw_control_trial` uses the existing command queue only in an explicit Manual
commissioning launch, with fresh stationary axes, no jog/probe and ALLOW state.
Its eight colon-separated values are Kp [A/(rad/s)], Ki [A/rad], RX window [ms],
positive/negative breakaway [A], positive/negative running assist [A], and final
output slew [A/s]. RX windows are 0 (legacy), 20, 30 or 40 ms. Four zero assist
values disable compensation. Changes are volatile, retain the 0.8 A cap, preserve
quiet effort across gain application, and explicitly remain unqualified. Normal
startup and stored configuration are unchanged. Physical A/B of this interface
is pending deployment; it is not a calibration result.

The second baseline series achieved +15.235 degrees pitch displacement and five
yaw direction pairs at that pose. Hold yaw current remained roughly 0.44–0.55 A,
comparable to moving current, with post-stop drift up to 0.835 degrees. Thus the
observed moving total current must not be copied into a friction feedforward
term. The +/-1-degree yaw probes also failed to reach their requested travel.
These are performance failures without runtime faults, retained for comparison.

Validation for this correction: the WSL native build and all 80 CTest entries
excluding retained homing passed (`session-trial-build.log`,
`session-trial-tests.log`). Fresh-stop-clock, bounded trial settings and quiet
gain-change effort regressions passed in the two focused CTest entries after
their final rebuild (`session-trial-focused-tests.log`). Document links passed.
The command's real socket/backend application and physical stop are still
pending the next station run; local tests do not establish those results.

## Owner-requested stop and work-in-progress snapshot (2026-09-30)

The owner stopped further tuning and implementation, then requested a commit
of the current work and removal of the WSL environment installed for this task.
The detailed operation history, parameter trials, errors and remaining work are
in [the agent behavior review](AGENT_BEHAVIOR_REVIEW_2026-09-30.md).

The PR3 snapshot adds six asynchronous pitch register observations with request
and actual RX timestamps, transaction cancellation before mode changes, verified
pitch gain application, trace fields, and a commissioning-only command. Payload
response checks now measure installed gains rather than silently writing gains
through the old void setter. Homing jitter diagnostics no longer infer that
current must be increased. Hardware-clock shutdown checks are also corrected.

This is **unfinished work, not a release qualification**. The transport probe
passed; the last complete native run passed 79/81 entries excluding retained
homing. A subsequent focused run passed 5/6 entries, with `park_power_probe`
still failing on stale/untrusted feedback. No further repair or build was run
after the owner stopped work. The PR3 snapshot has not been deployed.

The last deployed revision remains `bad742dddba6a95d3d055e43998adfd0506b0a6e`.
At 00:12 NZDT its launcher reported STOPPED, pitch disable confirmed, and yaw
zero requested with disable state unavailable. No rollback occurred. Trial
gains were volatile; production configuration still has its original defaults.
Runtime captures and unintegrated PR4 drafts remain under ignored `run/adr002`;
they are not included as runtime artifacts in this commit.

## 2026-09-30 · ADR-002.1 runtime surface (小满)

Identity this section speaks about: branch `ADR-002` at `479857b`, deployed release
`0b1b4b2b3ba7.AJVn7p`, aarch64 `controld` `3f2cabc4fe78d0de…`, station-generated
`parameter_inventory.json` `498b4e80204a22…`, frozen design `f761cbec39f356db…`. The inventory's
`source_rev` comes from the release's own `REVISION`; the Pi checkout at `6a47f1dd…` is a different
tree and is no longer allowed to masquerade as the built source (`git -C` walked up to it until
`77f71a5`).

### The four gates docs/ADR-002.1/00_CODEX_START.md:58 asks for

| Gate | Status | What was actually run |
|---|---|---|
| Parameter hot update on real hardware | PASS | `tools/adr0021_acceptance.py` walked every `experiment_writable` entry: **19/19** prepare→apply→read-back→restore, binary digest unchanged across the set, zero compiles, zero redeploys; `yaw.host_current_limit_a` written as `protected_read_only` and refused **server-side**. Then a real campaign: 16 candidates, 32 applied writes, 16 restores accepted, `refused: []`, `blocked: []` |
| Apply failure blocks the trial | PASS | The gate now stands in front of both trial and `param_apply`; measured refusal on the station: `param_apply refuses: manual_commissioning_off+mode_not_manual`, snapshot stayed `revision=0`, candidate left staged. A refused restore is `BLOCKED_restore_failed_*`, not a shrug |
| Experiment freeze | PASS | `adr0021_plan.py` refuses under-sized/oversized grids, missing `coarse_count_reason`, `confirm.repeats < 2`, stop rules without bounds, and names D7 when a dimension may not move; `--check` refuses post-freeze edits and inventory/binary drift. Binding carries four legs: source rev, binary digest, inventory digest, config/hardware profile |
| Reversal and prescribed-pose re-verification | **NOT_RUN** | Not attempted yet; no cell in this table may be read as physical qualification until it is |

### What this section deliberately does not claim

- The campaign's levels were the **sample grid** (`manifests/campaign.example.json`), authorised by the
  owner's `跑！` without levels. It is mechanism acceptance, **not** a tuning result: no scorer took part,
  so no candidate may be described as better, and `metrics` is absent rather than zero.
- `RUN`, controlled teardown, complete-log and `SCORE` from `00_CODEX_START.md:46` are **not yet performed
  by the runner**; what ran was parameter exchange, read-back, trace identity and restore.
- Trace identity is per-record and measured: 16/16 trials, 256 records per window, 66 carrying the
  candidate tag, contiguous from announcement to newest. The check is contiguity to the newest record,
  not "every row tagged" — the window is rolling and its head predates the candidate.
- Two host-side python tests remain red and are named, not hidden: `test_install_station` validates
  `User=eamars` against the local user database (correct on the Pi, cannot pass as `dsh`); the
  `test_station_launcher` log-path assertion is still open.

