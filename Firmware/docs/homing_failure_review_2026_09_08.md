# Homing with unknown load direction — 8 September 2026

**Status: partially verified; physical station stopped.** The operator described
the recent pitch incident as load-assisted acceleration near an endpoint with
inadequate braking, then required that yaw homing handle other load conditions.
That explanation is evidence about this incident, not an invariant for either
axis. This correction supervises both axes and both directions. No video, image
or real target was inspected.

## Findings and corrections

The prior code treated inactive-axis drift as harmless and allowed movement
during mode changes because the next approach would re-measure contact. Neither
establishes safe motion. During the [recorded restart](monitored_restart_2026_09_08.md),
the configured 5-degree backoff accepted approximately 3 degrees of withdrawal,
followed by approximately 2.91 degrees of encoder motion toward the endpoint.

- **Both axes are supervised throughout homing**, including initial/final setup
  and pending mode changes. A stationary axis has a retained 0.5-degree corridor;
  repeated checks cannot move it along with drift. Active moves have bounded
  paths. Speed approaches cannot reverse more than 0.5 degrees from their
  furthest observed progress. The encoder-derived speed ceiling is the plan's
  maximum homing/move command plus 2 degrees/second: 7 degrees/second for the
  shipped plan. A 0.05-degree allowance accounts for position quantization.
- **Abnormal motion and feedback latch a fault before homing advances.** Existing
  homing failure behavior disables both drives and requires explicit recovery.
  Disable is not a physical brake or proof of load support. Numeric summaries
  for both axes at 2 Hz now continue during asynchronous setup.
- **Mode setup retains the pre-disable position**, rejects displacement above
  0.25 degrees, requires fresh disabled-state confirmation before writing RunMode,
  and reads MechPos immediately before enable. Missing readback or excessive
  displacement aborts without enabling. Stop-before-mode-change remains mandatory.
- **Backoff preserves clearance.** Arrival must be within at most 0.25 degrees
  of the requested position with 150 ms of encoder readings within 0.04 degrees
  of a fixed sample. A requested 5-degree backoff requires over 4.75 degrees of
  actual withdrawal. Repeatability passes use at least the same backoff distance.
  Failure times out; neither current nor arrival tolerance is raised to pass.
- **Settling requires observed stability**, with a bounded deadline. Losing
  backoff clearance during settling fails. Motor, finite-data and torque gates
  also cover backoff and settling. Fine re-approach is bounded to the previous
  contact plus the repeatability allowance, rather than another full traverse.
  A first fine contact inconsistent with the coarse contact fails.

These are abort thresholds, not measured stopping distances, load ratings, or
proof that the clearance is mechanically adequate.

## Failure coverage and remaining limits

| Condition | Response and evidence |
|---|---|
| Assisting, opposing or reversing load; excess inertia | Measured speed/path/reverse-progress bounds; injected both signs on both axes |
| Inactive axis drifts while the other homes | Retained hold corridor, including setup waits; slow drift injection faults |
| Mode change releases holding torque | Pre-disable anchor and fresh encoder read; signed drift injection rejects enable. Cannot arrest unpowered motion |
| Stop ignored | No RunMode write or enable without fresh disabled feedback; injected protocol response |
| New feedback packets contain frozen position | Pre-enable register read detects disagreement; injected frozen feedback with changing MechPos. Both readings may share a failed sensor |
| Feedback missing, stale, backwards in time or nonfinite | Fault before homing advances; injected loop faults and existing watchdog tests |
| Friction stall or insufficient backoff | Bounded timeout and strict clearance gate; shortened-backoff probe |
| Load creeps during settling | Stationary window and clearance check; installed-load proof outstanding |
| Previously observed endpoint disappears | Fine travel bound; missing-contact probe in both axes/directions |
| Inconsistent contact | Dwell, coarse/fine agreement and bounded repeatability retries; existing contact/homing tests |
| Repeatable obstruction, cable snag or persistent friction resembles a stop | May be indistinguishable using encoder and effort alone; independent reference sensing or verified mechanism clearance is needed for that guarantee |
| Wrong mapping, loose coupling, output slip or encoder wrap | Some cause path/cross-axis faults; others can look internally consistent. Requires commissioning and independent output position |
| Power loss, failed braking, host/drive failure | Watchdog authority remains; software cannot certify passive holding. Support or a brake is needed if passive holding is required |

The existing span checks remain broad sanity checks (yaw accepts 180–540 degrees),
not proof of true endpoints. The configured yaw 176-degree pose is a historical
installation setting, not a universal balance point; its hold is now supervised.
Changing load or mounting can invalidate clearance poses and braking behavior.

## Executable evidence

`probe-homing-motion` exercises the production ControlLoop with injected encoder
behavior and the endpoint FSM for clearance/missing-contact cases. Before the
correction, all 24 abnormal motion/feedback cases continued homing. Afterward,
they fault and remain latched; normal approaches still proceed. A simulated
1-degree/second hold drift faults after approximately 505 ms; overspeed injection
after 10 ms; reverse motion after 75 ms. These are simulated detection times,
not physical stopping times. Eight endpoint checks cover both axes and signs.
Sixteen additional cases check future/regressing timestamps, overtemperature
and motor faults. Every fault test restores healthy feedback before verifying
that both axes remain disabled and a direct Home request remains rejected.

`probe-mode-motion-guard` runs the production CAN backend and register matching
against simulated motor replies: 28 cases across both axes, both modes, signed
drift, frozen feedback, missing readback and ignored stop. No hardware is opened.
Results and regression output are stored locally under ignored
`run/optimization-resume-20260908/`.

All **74 CTest entries passed locally**. The real controller and web services
also passed the HTTP recovery, cancellation, rejection-during-recovery and
complete re-homing probe with simulated motors and video disabled. This verifies
the software lifecycle across processes; the plant/drive behavior is simulated.

Regression fixtures were corrected to model a slowing position-loop backoff
instead of a constant-speed relay, and to retain monotonic simulated time during
re-homing after recovery. Production thresholds were not relaxed for those tests.
The first Pi deployment also exposed a race in the existing commissioning
watchdog probe: the published trip reason preceded its stop-frame side effect.
The probe now waits for the stop effect within its original bounded timeout.
The production watchdog and its deadlines are unchanged.

## Next physical proof and hardware decision

First verify the corrected sequence through a restrained mode-change test with
numeric position observation on both axes. Free motion requires measured
clearance exceeding worst observed stopping/coast distance plus uncertainty in
each direction. Then use interior 0.5/1-degree target-free steps to measure onset,
settling and post-cancel travel. Repeated endpoint approaches are not the test
for discovering braking distance. Capture remains numeric and bounded.

If either direction cannot meet those bounds at existing current/speed limits,
assess balance, support, drive/brake authority and independent limit/output
sensing. A faster Pi or AI HAT cannot resolve those mechanical or observability
limits. The [latency evidence](optimization_cycle_2026_09_08.md) still does not
demonstrate that a compute upgrade is necessary.
