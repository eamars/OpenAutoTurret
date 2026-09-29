# ADR-002 implementation and acceptance ledger

Updated 2026-09-29. Work is in progress; no physical qualification is claimed.

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
| Output arbitration/recovery | PR1 native PASS (79 CTest entries) | NOT_RUN | NOT_RUN | Not deployed |
| Yaw current velocity loop | Late-cycle public API probe PASS | NOT_RUN | NOT_RUN | Existing 0.8 A bound |
| Yaw friction compensation | NOT_RUN | NOT_RUN | NOT_RUN | Disabled until calibration |
| Pitch native speed tuning | NOT_RUN | NOT_RUN | NOT_RUN | Existing native mode and 5 A bound |
| Pitch homing | NOT_RUN | NOT_RUN | NOT_RUN | Do not inherit old mechanism results |
| Typed payload qualification | NOT_RUN | NOT_RUN | NOT_RUN | No new qualified profile |

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

## Failure handling (owner override)

No new station release has been started. Future releases use locally cross-built
committed source and a separate release directory, with launcher-controlled
stop/start and manual commissioning. Build/deployment failures are fixed forward:
preserve diagnostics and publish the next corrected version, with no rollback.
This owner rule overrides the ADR's generic rollback procedure. Retained geometry
remains intact. A failed physical trial still preserves traces before the ring
wraps and uses the launcher for stopping as necessary; it does not trigger
restoration of an old release. Do not switch GM back to voltage, overwrite
calibration, or interpret a GM zero-current request as confirmed disable.
