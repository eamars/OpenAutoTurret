<!-- doc-tree-check: ignore -->
> **Historical (2026-10-02).** The model-first code, tools and operation cards this document links to were removed when ADR-002.x closed; they remain in git history before the cleanup commit. Current path: [automatic servo commissioning](../../operations/servo-commissioning.md), closing report [SERVO_COMMISSIONING_2026-10-02](../reports/SERVO_COMMISSIONING_2026-10-02.md).

# Architect review 01 response — 2026-10-01

The [architect review](../architect_review_01/ADR-002.2-independent-review.md)
is the current authority, through the [identification amendment](../docs/08_IDENTIFICATION_REPAIR.md).
The amended Stage 1 is **IN_PROGRESS**. Yaw remains **UNQUALIFIED**;
Candidate14 remains **NONDEPLOYABLE** and was never physically run.
No accepted predictive model, replacement gains or physical certificate resulted.
The compact [machine status](ARCHITECT_REPAIR_PROGRESS.json) records the same boundaries.

This work ran on the Windows workstation and its local WSL environment. No station
connection, deployment, motion or fresh station-status query occurred. Existing dirty
source, captures, calibration and previous failures were preserved. No new hashes were
generated or checked. Numerical output and runtime captures remain under ignored `run/`.

## Audit and data contract

The supplied `architect_review_01/audit.py` ran unchanged against the original handoff ZIP.
Its summary JSON and 29-window inventory reproduce exactly. The 25-run CSV differs only
by floating-point roundoff, at most `6.66e-15`. The audit confirms 286,319 raw/decoded
CAN pairs with zero mismatches, the frozen gyro projection arithmetic, and 29 fitted
MOVE windows belonging to three physical journals whose training/prediction IDs coincide.
The descriptive TX/current alignment is 1 ms in 24 journals and 2 ms in one; this does
not measure physical torque delivery or replace the former delay with a qualified value.

[yaw_events.py](../../../commissioning/yaw_events.py) preserves every original record,
source line/run/configuration/calibration identity, raw parents, channel units, reset
generation and native timestamps. Requested, limited, successful TX and reported current
remain distinct. The fitter adapter uses native encoder/gyro/current observation masks,
actual successful-TX ZOH prehistory and one causal initial state per complete run.
Common count 5768 is a supplied diagnostic shaft datum, not qualified world zero.
Cross-session physical registration and the fixed-pitch gyro calibration remain unqualified.

The clock source audit reconstructs `sample_ns` exactly for 59,588 IMU samples in all
25 physical journals. `sh2_us` is SDK report time reconstructed from host polling time
with SH-2 report corrections; it is not an independent raw device clock. The producer's
epoch lift is verifiable per event. Physical sample offset/drift, polling-to-interrupt
error, internal filtering and physical latency remain unknown. No reset/recovery was
observed, so source inspection and continuity do not establish exercised reset behavior.
CAN receipts retain realtime/monotonic bracket uncertainty; motor sample time is unknown.

## Executable model and estimator

[identification_model.cpp](../../../axis_control_core/identification_model.cpp) and
[model_family.py](../../../commissioning/model_family.py) implement the four predeclared
algebraic/first-order actuator × Coulomb/Stribeck branches, constant/affine spatial load,
true sticking, breakaway, predicted-state direction, reversal, coast and reattachment.
Actuator state, gyro/current observation dynamics and fractional transport delays are
separate. Whole-run rollouts receive no future measured motion. LuGre, compliance and
the full two-axis physical parent remain unimplemented hypotheses or later scope.

Independent analytic/native probes pass: first-order current error is `1.11e-16 A`;
integration-resolution differences are below `2.5e-8 rad` and `2.5e-8 rad/s`.
The old validation path now passes actual preceding TX history, matching its fitting
objective; the analytic delayed-input regression no longer invents zero prehistory.

The [closed-loop estimator probe](../../../tools/adr0022_closed_loop_estimator_probe.py)
uses the actual native controller and independent analytic mechanics, three independent
seeds, 200 Hz feedback, 1 kHz encoder/current, 50 Hz gyro, quantization/noise, delays,
filtering, saturation/slew and state transitions. Pristine recovery passes. All three
noisy fits exhaust their predeclared 120-evaluation budget; seed 41 also fails two
separate whole-run angle prediction gates. Status is
**UNVERIFIED_SYNTHETIC_RECOVERY_FAILED**, with promotion blocked. This does not establish
a unique feedback-bias mechanism. Physical feedback data are excluded from training.

## Existing-data comparison and numerical limits

The [comparison operation](../../operations/adr0022-yaw-model-comparison.md) saves the
exact plan before fitting. Fifteen whole journals are assigned to eight open-loop TRAIN,
three SELECTION and four historical HOLDOUT blocks. Existing calibration exposure and
previous inspection of all archived data are recorded. No block is claimed unseen.
The final historical holdout has not been evaluated because no selection passed.

The earlier comparisons used numerical domains restricted to observed capture extrema:
four constant-load fits and four affine-load/current-observation variants ran with
a 200-evaluation limit. Those earlier optimizers reported convergence, but all returned
parameterizations failed whole-run selection. In that constant-load comparison, the 0.30-A run has
`2.266°` angle RMS and `1.731°/s` velocity RMS, against the unchanged `0.15°` angle target.
Other selection trajectories fail by tens or hundreds of degrees. Those earlier affine-load variants
also fail. These attempts are superseded by the corrected-domain comparisons below.
The added affine-load/current-observation terms are a combined diagnostic, not
evidence selecting either mechanism. Complete native trajectories, observations, masks,
initial states, successful TX and residual reports are retained.

The first comparison was numerically invalid: an out-of-domain seed generated flat
infeasibility penalties. It is retained and is not scientific model-rejection evidence.
Training-only inertia/drag backtracking now restores a feasible seed inside the same
declared bounds, preserves valid-run residuals and excludes penalties from sensitivity.
However, corrected fits move little and terminate on step size. Hard static thresholds
can have zero local sensitivity. Their failed predictions reject the returned parameters;
they do **not** establish that every bounded structure is physically inadequate.
A frozen training-only probe tested 24 static/sliding-friction and drag seeds, then
restarted the same bounded fitter. Training Huber cost fell from `499,756,855` to
`450,392,349` (9.88%), with large errors and zero static-threshold derivatives remaining.
The negative training trajectory ended only about `5.5e-5 rad` inside the numerical
domain boundary. That observed-travel boundary is not an approved physical yaw limit;
the retained slip-ring ruling allows continuous rotation. A separate training-only
domain probe replaced it with a conservative mathematical bound of ±16,939 rad,
derived from training histories and declared parameter/initial-state bounds. Training
cost fell another 66.71%, to `149,943,450`, confirming that the original numerical
domain obstructed identification. The frozen resulting model still failed all eight
training and three selection runs (selection angle RMS `72.77°`, `126.90°`, `117.25°`).
It grants no physical coverage or travel authority. All eight declared variants
have now been repeated with separate mathematical integration extent and measured
support. The corrected constant-load group is complete:

| Actuator / friction | Optimizer evaluations / success | TRAIN passed | SELECTION passed | Selection angle RMS, degrees |
|---|---|---:|---:|---|
| Algebraic / Coulomb | 57 / yes | 0/8 | 0/3 | 72.77, 126.90, 117.25 |
| Algebraic / Stribeck | 200 / no | 0/8 | 0/3 | 56.91, 114.13, 123.18 |
| First order / Coulomb | 146 / yes | 0/8 | 0/3 | 69.48, 109.01, 129.27 |
| First order / Stribeck | 200 / no | 0/8 | 0/3 | 58.24, 113.84, 114.45 |

The corrected affine-load/current-observation group is also complete:

| Actuator / friction | Optimizer evaluations / success | TRAIN passed | SELECTION passed | Selection angle RMS, degrees |
|---|---|---:|---:|---|
| Algebraic / Coulomb | 44 / yes | 0/8 | 0/3 | 94.91, 142.00, 126.48 |
| Algebraic / Stribeck | 200 / no | 0/8 | 0/3 | 2.266, 102.83, 93.14 |
| First order / Coulomb | 200 / no | 0/8 | 0/3 | 95.27, 141.68, 127.20 |
| First order / Stribeck | 200 / no | 0/8 | 0/3 | 2.266, 119.79, 98.07 |

Both corrected comparisons return **MODEL_DATA_FAILURE**. No model was selected;
no final holdout prediction was evaluated. Eighty-eight complete TRAIN/SELECTION
trajectory files are retained across the two corrected comparisons. Five fits
exhausted their unchanged 200-evaluation budget. These finite-budget failures and
local convergence still do not prove global structural rejection.
Current selection requires both training and separate selection prediction gates;
successful optimization alone cannot qualify the fit.
Training-only numerical estimator work remains necessary.
No physical identifiability or confidence ensemble is inferred from numerical rank.

The present family report includes complete angle/velocity/current errors, endpoint
drift, a lag-one residual diagnostic and initial-state horizons. Those early horizons
may cover only the leading baseline. Event-aligned reporting without state resets,
full transition metrics, residual tests against all prescribed factors, supported
uncertainty/profile evidence and independent prospective final blocks remain incomplete.
The two-axis/payload/configuration identification and qualified closed-loop estimator
also remain incomplete; this software slice is not the amended Stage 1 completion.

## Verification and remaining order

Local core/commissiond builds succeeded. **Nineteen focused native/data/history tests
pass**, including the per-event clock-provenance extension and rejection of inconsistent
training registration even when selection happens to pass. The workstation CTest run excluded
`retained_homing` as instructed: **82/83 passed**. The existing
`test_mixed_station_config` fails because its acceleration-parity/60-degree assertions
conflict with the already changed yaw-30/pitch-60 configuration. That configuration and
the production acceleration backend were not changed in this repair. The full suite is
not green; no privilege escalation or physical limit change was used to make it pass.

Continue in this order:

1. Resolve training-only numerical regime crossing and repeat the frozen diagnostic
   prediction comparison when a revised estimator has evidence. Preserve every attempt.
2. Verify the noisy closed-loop estimator, and qualify required clock/current/coordinate
   semantics; do not train on physical feedback data while that estimator is unverified.
3. Request only distinctions absent from existing data, using the program's information
   selector and actual approved envelopes. The two prior supplemental rounds remain
   consumed; do not silently reopen their budget. Higher-acceleration physical references
   still require explicit owner presence. No numerical fitting bound becomes a physical cap.
4. Freeze a supported configuration-specific predictive model before synthesis. Then save
   prospective closed-loop forecasts before physical 3a, followed by real production 3b.
   Pitch, simultaneous-axis coverage, payload/friction changes, return to baseline and
   undeclared-change detection remain part of complete ADR-002.2 delivery.

## Retained local evidence

All paths below are relative to the repository root and remain ignored runtime evidence:

| Evidence | Path |
|---|---|
| Exact supplied audit reproduction | `run/adr0022-stage2/architect-review-response-01/audit/` |
| Clock source/capture audit | `run/adr0022-stage2/architect-review-response-01/clock-audit/` |
| Frozen whole-journal/configuration plans | `run/adr0022-stage2/architect-review-response-01/comparison-config.json`, `comparison-affine-config.json` |
| Initial numerical-domain failure | `run/adr0022-stage2/architect-review-response-01/comparison/` |
| Feasible constant-load comparison | `run/adr0022-stage2/architect-review-response-01/comparison-feasible/` |
| Affine-load/current-observation comparison | `run/adr0022-stage2/architect-review-response-01/comparison-affine/` |
| Frozen training-only seed search | `run/adr0022-stage2/architect-review-response-01/comparison-training-seed-probe/` |
| Numerical domain diagnostic | `run/adr0022-stage2/architect-review-response-01/comparison-training-domain-probe/` |
| Frozen corrected-domain plans | `run/adr0022-stage2/architect-review-response-01/numerical-domain-plans/` |
| Corrected constant-load comparison | `run/adr0022-stage2/architect-review-response-01/comparison-constant-numerical-domain/` |
| Corrected affine-load comparison | `run/adr0022-stage2/architect-review-response-01/comparison-affine-numerical-domain/` |
| Independent closed-loop estimator verification | `run/adr0022-stage2/architect-review-response-01/closed-loop-estimator/verification/` |
| Native analytic/resolution probe | `run/adr0022-stage2/architect-review-response-01/native-family-probe.json` |
| Focused regression log | `run/adr0022-stage2/architect-review-response-01/targeted-tests.log` |
| Existing full workstation test log | `run/adr0022-local/firmware/Testing/Temporary/LastTest.log` |
