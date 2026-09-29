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

Next is an inactive committed-source release and station suite before PR2
motion measurements. The owner permits Pi-native compilation as a fallback;
the Pi's fmt/spdlog/yaml-cpp/GTest development modules are present. Its missing
libcamera/OpenCV headers are irrelevant to this C++ build (vision is Python).
No activation, calibrated parameter change, or hardware acceptance is included
in the native gate above.

## Rollback

No new station release has been started. Keep the observed running release and
retained geometry intact. Future releases use committed source and a separate
release directory, with launcher-controlled stop/start and manual commissioning.
If a trial fails, preserve traces before the ring wraps and stop through the
launcher. Do not silently restore known competing guard output as a qualified
loaded configuration, switch the GM drive back to voltage, overwrite calibration,
or interpret a GM zero-current request as confirmed disable.
