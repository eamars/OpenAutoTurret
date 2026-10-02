# Independent review of Codex's response evidence

**Date:** 1 October 2026. **Input:** `ADR-002.2-model-evidence-20261001.zip`.  
**Review action:** read the archive and independently recompute its exported prediction errors. No new physical measurements, simulator execution, native optimizer execution, motor-setting changes, or deployment were performed.

## Decision

Keep the actuator/mechanics/friction/measurement separation, but change the next work priority to **estimator recovery and fair model comparison**, with motor-interface verification in parallel. The new evidence does not justify either “the architecture is impossible” or “a more complicated friction model is definitely required.” It identifies specific, testable failure paths.

A failed model or quality gate blocks promotion. It must route to diagnosis and recovery, not automatically terminate the engineering task. Continue offline implementation and synthetic verification while physical yaw remains unqualified.

## R1 — What Codex has improved

The new archive contains 11 complete observation timelines: eight TRAIN and three SELECTION. There are eight final parameterizations and 88 model/run predictions. The native masks, input prehistory, filtered-output semantics, and single initial state per run are explicit. Selection initial states use causal seeds; there are no reported within-run state resets. The final holdout was not evaluated and no model was promoted.

These changes address the earlier tiny-window/train-as-validation criticism. Do not continue diagnosing the new fit as if it still used only 29 disconnected MOVE windows. It now fails on whole trajectories, which is more informative.

**Sources:** `DATA_FORMAT.json`; `results/constant.json` and `results/affine.json`, their comparison arrays and final-holdout fields; `READ_FIRST.txt`. State-reset and optimizer facts are supplied metadata, not independently observed native execution.

## R2 — Independent evidence audit

The executed script `tools/audit_evidence.py` loads NPZ files without pickle, checks timeline/mask/TX and prediction shapes/finite values, and recomputes q/gyro/current RMS from each native comparison. All **88** physical predictions and the two provided seed-41 cross-run predictions were processed. It also recomputes the supplied parameter-relative errors.

There are **470 numeric comparisons** against exported summaries, with maximum absolute discrepancy **0.0** in this execution. All 88 physical angle RMS values exceed 0.15 degrees; the smallest is approximately 0.547522 degrees. This corroborates rejection of these parameterizations. It does not independently verify the original optimizer, latent-state integration, implementation-to-binary equivalence, or physical torque/current semantics.

Eight unit tests of the audit utilities also pass. This is a data-audit result, not a controller test. `audit/audit_summary.json` and the other audit files contain the exact values and limitations.

**Sources:** `observations/*.npz`, `predictions/{constant,affine}/*/*.npz`, `estimator/seed41-predict-seed{17,83}.npz`, and their paired observations; executed outputs in `audit/`.

## R3 — Optimizer success and useful model prediction are different

| Family | Reported evaluations | Solver success | Selection q RMS, degrees: descended-03 / information-case13 / case20-negative |
|---|---:|---|---|
| algebraic-coulomb-constant | 57 | True | 72.772 / 126.901 / 117.251 |
| algebraic-stribeck-constant | 200 | False | 56.912 / 114.128 / 123.176 |
| first_order-coulomb-constant | 146 | True | 69.481 / 109.014 / 129.271 |
| first_order-stribeck-constant | 200 | False | 58.244 / 113.835 / 114.450 |
| algebraic-coulomb-affine | 44 | True | 94.907 / 142.002 / 126.479 |
| algebraic-stribeck-affine | 200 | False | 2.266 / 102.828 / 93.141 |
| first_order-coulomb-affine | 200 | False | 95.266 / 141.680 / 127.200 |
| first_order-stribeck-affine | 200 | False | 2.266 / 119.791 / 98.071 |

Three optimizations report local convergence; five exhaust 200 evaluations. Every final parameterization fails every TRAIN/SELECTION run. This is rejection of these fits, not a proof that no parameter vector in those structures can work. `xtol` is a numerical termination condition, not an engineering quality result. [S2]

**Sources:** `results/{constant,affine}.json#/comparisons/*/optimizer`; independently recomputed `audit/physical_predictions.json`.

## R4 — The apparently best 2.266-degree result is a no-motion prediction

In SELECTION run `yaw-descended-03`, measured shaft-angle span is **3.0322265625 degrees**. Both affine Stribeck predictions have **zero predicted angle span** in the exported float64 arrays. Their RMS values are 2.2663848135 and 2.2663848200 degrees. Holding the initial observed position constant gives 2.2663849535 degrees RMS, essentially the same result.

Thus these are not successful recoveries of small motion. Their lower score principally reflects avoiding the very large erroneous movement predicted by other fits. They still fail the absolute angle requirement. Add a constant-position baseline, displacement and event metrics; do not select based only on a smaller cost.

A related clue: the algebraic Coulomb constant fit sets total static magnitudes to approximately **0.26755 / 0.26915 A-equivalent**, while the physical-probe transmitted range is approximately **-0.24774 to +0.24902 A**. Its fitted zero-offset sticking model therefore has a natural no-motion explanation under those commands, yet the observed probe spans 1.49414 degrees. This points to a threshold/load/initial-state identification problem to investigate; it does not uniquely establish which real physical term is wrong.

**Sources:** the three native arrays for `yaw-descended-03`; `results/constant.json#/comparisons/0/model`; `observations/yaw-physical-probe-01.npz`; `audit/audit_summary.json#/no_motion_selection_cases` and `/coulomb_threshold_example`.

## R5 — The synthetic evidence is better, and more specific, than the headline failure

The pristine synthetic problem passes. The three pristine seed labels repeat the same deterministic trajectory; they are not three independent excitation cases. This verifies a narrow four-coordinate algebraic/Coulomb problem with other nuisances fixed, not the entire physical model family.

The noisy-case results are:

| Noisy seed used for fitting | Worst parameter error | 5% parameter gate | Reported optimizer result | Reported trajectory gates, train + two cross-runs |
|---|---:|---|---|---|
| noisy-seed-17 | 0.2357% | Pass | Budget exhausted at 120 | 3/3 pass |
| noisy-seed-41 | 3.9872% | Pass | Budget exhausted at 120 | 1/3 pass |
| noisy-seed-83 | 0.5364% | Pass | Budget exhausted at 120 | 3/3 pass |

All three noisy parameter fits satisfy the existing 5% parameter-recovery threshold. All three exhaust the optimizer budget. Seed 41 additionally fails two cross-run position predictions, which this review reproduces directly from the supplied arrays:

- On noisy seed 17: q RMS **0.0014181643743 rad**, against **0.0008023127185 rad**.
- On noisy seed 83: q RMS **0.0010986193478 rad**, against the same threshold.

The corresponding gyro and current gates pass. Seed 41's training trajectory passes; the seed-17 and seed-83 fits each pass all three reported trajectory tests. Overall, seven of nine noisy fit/run trajectory comparisons pass, but the full declared synthetic gate still fails. Do not waive that failure; classify and repair it accurately.

Small parameter error can still produce unacceptable accumulated trajectory error. Conversely, an exhausted budget is not equivalent to failure to recover parameters. Reproduce seed 41 with an oracle/derivative/event/noise diagnostic, then extend verified coverage. Do not add physical-model complexity merely to hide a failure on a known synthetic model.

**Sources:** `estimator/summary.json#/cases`; `estimator/frozen-contract.json`; the two seed-41 prediction NPZ files; `audit/synthetic_gate_breakdown.json` and `audit/synthetic_cross_predictions.json`. Only the provided cross-run prediction arrays were independently recomputed; other synthetic convergence/prediction outcomes are reported results.

## R6 — Local static derivatives and model-comparison confounds

The algebraic Coulomb fits in both load modes report exactly zero local sensitivity for the positive and negative static-excess parameters. This can occur because a small perturbation does not change the current stick/slip event sequence. It is not evidence that static friction is physically absent or accurately identified. Use meaningful threshold profiles and censored event constraints.

The constant-to-affine comparison changes more than spatial load: **current delay and current-filter time constant are additional free coordinates in the affine fits**. Repeat a load-only ablation with the same measurement freedoms, then test current observation dynamics separately.

The load offset is fixed at zero while directional friction is free. Simply freeing that offset creates a potential additive gauge with the directional friction terms. Identify supported combinations or a justified reference, not an arbitrary unique-looking coefficient vector. Details and the invariance equation are in `02_LOCAL_AGENT_INSTRUCTIONS.md`.

**Sources:** `results/{constant,affine}.json#/comparisons/*/optimizer/coordinates`, `/unmodified_objective_sensitivity/column_norms`, and `/model`.

## R7 — Coverage and horizon metrics need explicit interpretation

The data mix degree-scale creep and very large excursions. `yaw-information-round2-case14-positive` spans **3,165.249 degrees** and reaches a projected gyro rate around **1,008.8 degrees/s**. `yaw-information-case13-01` spans only **1.582 degrees**. This is a reason to inspect per-run/channel/regime objective contributions; it is not proof, without that breakdown, that weighting caused the failure.

The first command departure greater than 0.01 A from baseline occurs roughly **1.59–1.60 seconds after the normalized initial time** in these runs. This descriptive threshold is not a formal onset detector, but it shows why initial 50–1,000 ms prediction horizons can mostly measure pre-excitation rest. Add event-anchored diagnostics from the uninterrupted rollout; do not reset to future observations.

The named `yaw-information-case13-01` run is not the earlier `yaw-feedback-13-speed-pos5` Candidate13 run. Do not conflate their evidence.

The final holdouts reserved by this comparison include feedback runs already encountered in earlier development. They are useful historical regressions, not genuinely unseen lifetime validation. Keep them reserved under the declared comparison; any later data-role change needs an explicit plan revision and fresh prospective verification.

**Sources:** `audit/observation_inventory.json`; the frozen plans' split lists; the earlier independent review. No new physical run is present in this archive.

## R8 — Timing and GM6020 conclusions remain bounded

The clock audit reports consistent host-based epoch reconstruction in the captured generation, not a qualified physical sensor clock. It explicitly leaves sample offset/drift and filtering unknown. The data do not exercise reset/rollover recovery. Nanosecond precision in a host timestamp conversion is not nanosecond accuracy in a motor or IMU physical response.

The new screenshot shows PWM position mode and current-loop ON, with current P=1000 and I=500. It does not establish active CAN mode, motor firmware, historical applied settings, or physical gains. See `03_GM6020_INTERFACE.md` for the readback/command contract. Manufacturer descriptions distinguish CAN and PWM and give CAN priority when both are present; the particular firmware protocol still needs verification. [S1]

**Sources:** `assumptions/clock-findings.json`; `assumptions/measurement.json`; supplied `image(6).png`; S1. No motor firmware readback was obtained by this review.

## Scope and provenance

The previous review remains useful for original physical failures and acceptance criteria, but its small-window criticism should not be reapplied unchanged to this new whole-run evidence. The current next step is a structured recovery loop, not a new declaration that a single physical mechanism has been proven.

`MANIFEST.sha256` and the following input digest were generated during this review. They establish the bytes used here and package reproducibility, not equivalence to unseen original repository files or historical integrity:

```
c5d9171889770a3c894e50a42215920375224e468a6e56d087c93e0d975fe0d2  ADR-002.2-model-evidence-20261001.zip
```
