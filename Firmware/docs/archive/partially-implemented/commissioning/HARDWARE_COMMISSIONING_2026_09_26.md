# Mixed-hardware implementation and first mechanical tests

26 September 2026. **Partially verified:** actual dual-bus transport and bounded
GM6020 motion are verified; the automatic station is not adapted or activated.
The owner authorized mechanical tests, confirmed pitch mechanical endstops,
and confirmed yaw has no endstop and can rotate through a slip ring.

## Source and host state

Work is on `codex/hardware-adaptation`. Tested runtime source:
`3edc2670cbfa3cc6cfff03d9b0c799f46dfbbe22`. Committed source was deployed using
`Firmware/tools/deploy_station.py --probe-build --commission-hardware` into:

```text
/home/eamars/workspace/OpenAutoTurret/run/releases/3edc2670cbfa.2BVtEy
```

The original Pi checkout at `6a47f1d` was preserved. The shared project-local
`run/station-venv` was created with `--system-site-packages`, then populated from
`requirements-station.txt`; pytest was added there for Linux launcher tests.
No global pip installation, boot changes or retained-calibration changes were
made. All probe/controller processes ran as `eamars`; elevation was limited to
temporary CAN interface setup and isolated virtual-CAN test provisioning.

## Hypothesis and implementation

The first hypothesis was that the production transport could exchange GM6020
standard CAN and CyberGear extended CAN on the two HAT channels without losing
frame type or starving feedback. A second, narrowly bounded hypothesis was that
standard `0x1FF` voltage commands would move the installed yaw drive in the
expected encoder direction and a zero request would end these short motions.

Implemented:

- `RawFrame` preserves SFF/EFF, RTR, error and DLC semantics; SocketCAN uses a
  separate error subscription and exposes explicit off-loop health refresh.
  Legacy extended-frame callers retain their API. The retired yousee path
  explicitly rejects unsupported standard-frame transmission.
- GM6020 voltage group encoding and strict feedback decoding; signed rpm and
  raw current/temperature remain distinct. A session-relative encoder unwraps
  modulo 8192 and invalidates continuity on implausible motion, bad timestamps
  or gaps over 50 ms. This does not establish mechanical homing or absolute yaw.
- `probe-mixed-hardware`, its separate `hardware_probe.yaml`, and launcher
  `--commission-hardware` mode. Both buses, SPI parents, bitrate and pitch UID
  are checked before motion. Pitch commands are discovery and a register read
  only. The automatic controller, cameras and web are not started.
- Bounded 200 Hz yaw pulses, fresh-feedback/speed/travel checks, a serialized
  zero-output guard on pulse deadline/heartbeat/interruption, and station-wide
  ownership. The guard is in the same process: it is not an independent cutoff.
- Normal hardware preflight rejects the legacy yousee profile on this
  split-bus installation. No dummy yaw endpoints or fabricated disable status
  were introduced to make the old automatic stack appear ready.

## Actual Pi probes

Both real buses were UP, ERROR-ACTIVE, classical CAN at 1 Mbps, with zero bus
error counters. The receive/discovery probe collected 2,600 yaw feedback frames
in approximately 2.6 seconds, matched pitch UID `0x7216313130333105`, and sent
zero GM6020 commands. After hardening, this probe was repeated successfully.

The following runs used the launcher and actual C++ SocketCAN transport:

| Test | Requested pulse | Peak absolute travel | Final signed displacement | Peak reported speed | Outcome |
|---|---|---:|---:|---:|---|
| First low output | +500 raw, 100 ms | 0.088 deg | +0.044 deg | 6 deg/s | Completed, stationary observed |
| Positive yaw | +1000 raw, 150 ms | 0.264 deg | +0.220 deg | 6 deg/s | Completed, stationary observed |
| Negative yaw | -1000 raw, 150 ms | 0.703 deg | -0.527 deg | 12 deg/s | Completed, stationary observed |
| Launcher stop during pulse | +1000 raw, requested 500 ms, interrupted after about 190 ms of commands | 0.835 deg | +0.747 deg | 12 deg/s | Interrupted, zero requested, stationary observed |

All motion runs reported `zero_tx_failed=0`, `yaw_errors=0`, `pitch_errors=0`.
Feedback age sampled by the 200 Hz loop stayed below 0.91 ms in these runs.
The positive and negative tests establish command sign against **motor encoder
counts**, not clockwise direction in camera/world coordinates or a measured
load-side transmission ratio. The 500-unit displacement is near quantization
and static noise. Speed feedback is integer rpm, hence 6 deg/s steps; it cannot
by itself measure smooth low-speed performance.

Zero output and observed stationarity do not establish electrical disable,
braking after a process crash, or support for a gravity-loaded pitch axis.
No full revolution, endpoint search, motor enable/zero/configuration write on
pitch, process-kill experiment, firmware change, or automatic tracking occurred.

The initial can0 RX-drop counter was 10,649 and did not increase during these
checks. Both kernel bus-error counters remained zero. This short observation is
not sustained-load qualification with two cameras and inference running.

Raw numeric traces and logs are retained locally under ignored
`run/hardware-adaptation/`, including `passive-hardened.csv`,
`yaw-positive-500.csv`, `yaw-positive-1000.csv`, `yaw-negative-1000.csv` and
`yaw-interrupted.csv`. Runtime captures are not committed. CSV timestamps are
host monotonic receipt/sample times, not motor-generated timestamps.

## Pitch compatibility defect found on hardware

For host ID 0, motor ID `0x7F`, the installed motor returned:

| Read | Response arbitration ID (EFF) | Eight payload bytes | Interpretation |
|---|---|---|---|
| `mechPos`, `0x7019` | `0x11017F00` | `19 70 00 00 30 33 31 05` | Nonzero reserved/status byte; payload tail repeated prior discovery bytes |
| `run_mode`, `0x7005` | `0x11007F00` | `05 70 00 00 00 33 31 05` | Documented successful header; u8 value 0 |
| `limit_cur`, `0x7018` | `0x11007F00` | `18 70 00 00 00 00 D8 41` | Documented successful header; raw float 27.0, not adopted as a safe setting |
| `mechVel`, `0x701B` | `0x11017F00` | `1B 70 00 00 00 00 D8 41` | Nonzero status; repeated prior value |
| `VBUS`, `0x701C` | `0x11017F00` | `1C 70 00 00 00 00 D8 41` | Nonzero status; repeated prior value |

The old parser accepted `0x11017F00` as a valid position of approximately
`8.33e-36` rad. It now rejects nonzero upper response status/reserved bits,
nonzero reserved payload bytes and wrong DLC before publishing a value.
The commissioning probe reports `pitch_position_valid=0` and status 1.

The exact firmware revision and meaning of status 1 were not identified. The
vendor reference conditions `0x7019..0x7020` availability on firmware 1.2.1.5;
an older/incompatible register implementation is a plausible explanation, not
a verified firmware identification. No upgrade or parameter reset was attempted.
See [CyberGear protocol reference](../../../references/cybergear/CyberGear_AI_Reference.md#20-comm_type_17--0x11---read-one-parameter).

## Regression verification and final state

After the live transport probe, focused regression coverage was added and run
on the Pi. All **76 CTest targets passed** (49.57 seconds). The new test binary's
**7 cases passed**, including an actual SFF/EFF/RTR round trip on isolated
`vcan0` with `OTA_TEST_VCAN=1`; this test was executed, not skipped. Other cases
cover malformed feedback, voltage group/range/sign, forward/reverse wraps,
continuity invalidation and the captured unsuccessful pitch reply.

Both Linux pytest launcher tests passed (4.24 seconds), covering detached
ownership/lifecycle, controlled child stop, no false PARKED result and exclusion
across distinct runtime directories. Bash syntax validation passed. Normal
hardware preflight was exercised and correctly rejected the legacy configuration.

The final real receive/discovery probe passed after the regression run:
2,600 yaw frames, zero yaw TX, stationary feedback, no CAN error frames. Both
physical interfaces were then restored DOWN/STOPPED; the temporary virtual
interface was removed. The launcher was stopped and the original Pi checkout
remained clean. Motor electrical disable is still unverified. Numeric logs are
in ignored `run/hardware-adaptation/`, including `ctest.log`,
`transport-tests.log`, `launcher-tests.log` and `passive-final.csv`.

## Remaining work

1. Resolve pitch's supported position/status interface and firmware identity
   before adapting pitch homing. Its mechanical endstops are owner-confirmed;
   bounds, signs, support and calibration still need physical commissioning.
2. Add the production mixed-drive composition and capability contract; select
   explicit continuous-yaw topology, session reference and pitch-only homing.
3. Prototype and measure a bounded GM6020 velocity/position loop. Current pulse
   data are insufficient to choose production gains or certify stopping distance.
4. Qualify loss-of-command/link/process behavior and the station stop/park
   contract, then continuous-yaw planning and camera geometry through wraps.
5. Integrate BNO085 observe-only acquisition and dual-camera/Hailo perception
   through their separately specified probes. This session did not provision
   Hailo or alter the cameras/IMU.

Continue with [the adaptation plan](../hardware/HARDWARE_ADAPTATION_PLAN.md) and operate via
[the current runbook](../../../STATION_OPERATIONS.md).
