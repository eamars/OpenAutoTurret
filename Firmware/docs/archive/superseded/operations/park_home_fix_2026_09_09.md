# Park power continuity and web recovery, 9 September 2026

Status at the initial investigation: **Partially verified; source changes had
not been activated on hardware.** The operator subsequently approved midpoint
yaw / lowest pitch as the safe release pose and requested deployment; see the
current station runbook for the resulting motor-feedback release policy.

The hypothesis was that parking could retain drive torque until both axes reach
the configured pose, and that the open web menu could follow controller phase
changes so recovery and Home remain usable. Verification used numeric logs,
production JavaScript execution, real web/controller services with simulated
motors, and the production CAN backend with an emulated loaded plant. No video
feed, browser, screenshot, or visual element inspection was used.

## Findings

The active station was release `96d3cd713c8c.qK3aUY`, running the prior Manual
commissioning configuration. Read-only launcher status showed Fault, with Home
rejected by the controller's fault gate. The retained controller log records:

- 09:20:48.623: parking still in `stop_tracking` at pitch -0.437743 rad and yaw
  -0.823034 rad, with park targets -1.216899 rad and +0.164606 rad.
- 09:20:48.831: mode setup faulted on more than 0.25 degrees of displacement.
- 09:21:37.689 and 09:21:52.111: Home rejected because the controller was faulted.

`ControlLoop::start_parking` scheduled two drive mode recipes even when service
already ran in speed mode. The CAN recipe deliberately sends STOP before mode
configuration. On a loaded axis this removes torque before the park move, and
the subsequent displacement gate can fault after the load moves. The executor
also scheduled a second torque-breaking transition into position mode at dwell.
The ordinary ideal simulator did not reveal these protocol side effects.

The web `paint` function's drawer refresh key omitted `phase`. An open menu
therefore retained its disabled Recover Motors row across Hold → Parking → Fault
when the mode and target list were unchanged. Conversely, unrelated target churn
could replace MENU buttons between pointer-down and click. The existing tests
called the row builder with fresh data, so they did not cover this update path.

## Changes

- Refresh MENU on phase or command acknowledgement changes; ignore unrelated
  target churn for that drawer.
- Preserve running drive modes throughout powered braking, parking moves and
  verification. Remove parking's disable/re-enable mode recipes.
- Require 150 ms of stationary feedback before movement, bounded by 2.5 seconds.
- Use bounded position correction in speed mode during the target dwell.
- Reject parking without fresh healthy running drives; latch unexpected drive
  disable before the release stages. Cancel manual motion leases on entry.
- Keep Stop/Hold available during entry and motion, retaining powered stop
  behavior. Existing watchdog, hard-fault and release gates remain authoritative.

## Executed evidence

| Probe/check | Observed result |
| --- | --- |
| `probe_menu_lifecycle.py`, before change | Recovery remained disabled in Fault, Idle and Parked; only one menu update across six phases |
| Same probe, after change | Correct recovery gates in all six phases; zero MENU replacements over 50 unrelated target updates |
| `probe-parking-motion`, before change | Two parking-entry mode transitions; power-continuity acceptance failed |
| Same probe, after change | Zero entry transitions/disables; drift, overspeed and operator stop remain latched |
| `probe-park-power`, production controller/CAN protocol, no independent sensor | Both axes moved to target; zero STOP, enable or mode-write frames; explicit missing-evidence fault |
| Same protocol probe, simulated independent sensor | Both axes moved to target; exactly two STOP frames after arrival; zero early STOP, enable or mode-write frames; Parked |
| Real controller/web HTTP lifecycle with simulated motors | Parked → Home and verification failure → Home; Home rejected during parking; Stop cancels parking and remains latched; services remain alive |
| Real controller/web HTTP recovery with simulated motors | Recovery, command rejection while recovering, cancellation, retry and re-homing passed |
| CTest | 75/75 entries passed, including existing position-mode parking and unexpected-disable regressions |
| Web drawer tests | 25/25 passed |
| GCC `-Wall -Wextra -fanalyzer -O0 -c` on control loop | Completed; same 21 warning messages as HEAD baseline; no new warning messages |

The GCC baseline includes unused-parameter, signedness, formatting and a
`std::deque` allocator uninitialized-value diagnostic. These are not reported as
a clean static-analysis result; the change introduced no new messages.

Runtime logs and analyzer output are retained under ignored
`run/park-fix-20260909/`, outside version control.

## Remaining boundary

At the initial investigation, the installed CAN backend supplied **no independent output-position evidence**.
The operating runbook requires that evidence before automatic power release.
Consequently this change addresses the premature torque loss and menu update
defects, but does not establish hardware PARKED or make the complete requested
park-and-power-off behavior available on the installed sensor configuration.
Healthy drives remain powered when release verification fails. Physical travel,
load holding and post-disable stability have not been verified for this source.
The initial investigation did not restart or send motor commands to the physical station.
