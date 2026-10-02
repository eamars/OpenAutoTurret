# ADR-002.2 yaw commissioning TODO

> **2026-10-02 (owner ruling):** the architect reviews are guidance, not instructions; the working product comes first. **ADR-002.x is closed**: commissioning is automatic and station-verified — [closing report](reports/SERVO_COMMISSIONING_2026-10-02.md), [servo commissioning card](../operations/servo-commissioning.md), overnight hand-tuned predecessor in the [takeover report](reports/SERVO_TAKEOVER_2026-10-02.md). The items below are historical context.

Highest current authority: [architect review 02](architect_review_02/00_START_HERE.md)
and its [estimator recovery amendment](docs/09_ESTIMATOR_RECOVERY.md). Follow WP0–WP7
in promotion dependency order; independent safe offline work continues after failed gates.
The owner confirms CAN current mode and excludes PWM. Latest executed evidence is in
[review 02 progress](reports/ARCHITECT_REVIEW_02_PROGRESS.md); the review 01 results below
remain historical evidence, including the original synthetic failures.

Current authority (2026-10-01): the owner's instruction to prioritize the
[architect review](architect_review_01/ADR-002.2-independent-review.md), implemented
as the [identification-repair amendment](docs/08_IDENTIFICATION_REPAIR.md).
**Yaw remains UNQUALIFIED. Candidate14 is NONDEPLOYABLE and was never physically run.
The amended Stage 1 is IN_PROGRESS; formal 3a/3b have not passed.** Preserve every
capture, prior fit and failure. This local repair makes no station access, deployment
or physical run and generates/checks no hashes.

The former policy permitting a bounded feedback probe after failed predictive-model
gates is superseded. The former next Candidate14 -5/+5 degrees/s ordering is superseded.
Current source/core replay can reveal arithmetic defects but cannot qualify a plant;
optimizer convergence, reduced-column rank and an acceptable mean do not qualify
burst/stall tracking. Keep the complete stopping and failed-motion windows.

Active goal: completely finish ADR-002.2, yaw first, followed by pitch and the full
configuration/operating-condition matrix. Both dedicated-program 3a and the actual
production 3b remain required for both axes. Local software success cannot mark that
physical scope complete.

- [x] R1. Reproduce the raw-byte, frozen gyro-projection, current-clock and training-window audit from existing evidence. Preserve source member/line/run/config/calibration provenance; distinguish arithmetic verification from physical calibration. Exact audit reproduced; physical timing and calibration remain unqualified.
- [ ] R2. Implement and verify the amended predeclared actuator/mechanical/friction/measurement family and run-block contract locally. Preserve true stick/breakaway/sliding/stop/reversal states, successful pre-window TX ZOH, coordinate registration, native sample clocks/freshness and current observation semantics. Record unsupported history/compliance hypotheses explicitly.
- [x] R3. Run the independent synthetic closed-loop estimator check with actual multi-rate sampling, sensor errors, feedback-generated input, saturation/slew and state transitions. Original execution failed optimizer termination and seed-41 cross prediction despite passing noisy parameter recovery. Review 02 repairs and fresh verification pass the bounded four-mechanics known-nuisance fixture. Broader nuisance/structure estimator verification remains outstanding; output-error fitting is not automatically unbiased.
- [ ] R4. Compare admissible structures on complete existing-data runs with frozen TRAIN/SELECTION/FINAL_VALIDATION roles. Initially train on open-loop current excitation and use feedback journals as diagnostic held-out comparisons. Report identifiable/prior-only parameters, unresolved timing, residual structure and complete forward errors. Previously inspected 0.60-A data remains retrospective, not an unseen prospective low-speed trial.
- [ ] R5. Freeze a supported configuration-specific model only if predictive gates pass. If no model passes, return the explicit model/data/measurement/information failure and the missing discriminating evidence. Any necessary physical information round must follow station cards and actual verified bounds; no gain search or Candidate14 trial follows this review.
- [ ] R6. After model freeze, synthesize using incremental/nonlinear dynamics and the shared core. Save prospective closed-loop predictions before any new trial, with 50/100/200/500 ms, 1 s and full-run reporting. Compare the frozen actual trial with saved predictions and unchanged targets, then complete yaw 3a → production 3b and the remaining ADR scope.

Current focus: audit → local architecture/estimator repair → existing-data structure
comparison → missing-information decision → predictive model freeze → synthesis →
saved prospective forecast → physical qualification. Keep real current, thermal,
freshness, mechanical and unsafe STOP protection. Reference shaping at 30 degrees/s²
remains guidance, not a demonstrated physical acceleration bound. Do not change speed
policy or acceptance thresholds to make a failed result pass. Freeze all edits/builds
during any later physical session and keep one output owner.

Current execution evidence: [architect repair progress](reports/ARCHITECT_REPAIR_PROGRESS.md).
The canonical whole-run adapter and four actuator/friction branches are implemented.
Eight corrected-domain constant/affine diagnostic fits failed every training and
selection run. Artificial capture-extrema constraints were removed using training-only
mathematical bounds; five fits still exhausted their fixed numerical budget. Estimator
and measurement limitations remain unresolved, so these outcomes do not uniquely reject
a physical mechanism. No model was frozen and no final historical holdout evaluation occurred.

<details>
<summary>Historical pre-review workflow and instructions (superseded where conflicting)</summary>

The following records the earlier acquisition and tuning sequence. Its sentences
labelled current/next and its focus rules are historical; they do not authorize the
next run or override R1–R6 above.

Status: items 1 and 2 complete. Physical acquisition and its rest/noise/timing analysis are recorded; full mounting and plant qualification remain pending.
Latest owner steering (2026-10-01): acceleration is guidance, not an acceptance blocker. Retain 30 degrees/s² reference shaping and acceleration/braking observations; allow marginal excursions and high IMU acceleration observations to pass without diverting the ADR-002.2 tuning task. The measured-current acceleration window that prevented actual yaw motion is being made explicitly optional at runtime. Preserve real current, thermal, freshness and unsafe STOP protection. Focus on deterministic model/controller calculation and actual starting, tracking, reversal and stopping. Higher-acceleration reference testing remains deferred until owner presence; incidental measured excursions do not block the current tuning run.

Owner update 2026-10-01: nobody will be near the station. Limit yaw acceleration and deceleration to **30 degrees/s²**, including startup assistance and controlled braking; use actual IMU acceleration/gyro/orientation data to assess uneven vibration before the next physical run. The owner reports the pitch platform is balanced and steady stationary/high-RPM rotation is stable; do not add or lower a speed cap for this request. High-acceleration testing remains parked until the owner is near the station around 18:00 local and confirms that condition. Current physical sessions are parked while the limiter and observation path are implemented and verified. Continue deterministic automatic tuning and validation for low/high acceleration and low/high speed; higher-acceleration hardware cases wait for presence. Keep unsafe STOP available. This is a mechanical requirement, not a model-qualification prerequisite.
Current task: yaw model update and feedback validation, within items 3–6. Both prescribed physical information rounds are complete. Their maximum-first profiles supplied independently confirmed body motion, and the numerical input retains actual posture readbacks and receipt ages. Candidate12 guidance-mode out/back ended at 2026-09-30 23:55:57 UTC, with 5.933-degree encoder excursion independently confirmed by IMU; settling/reversal still fail. Its same-parameter +5-degree/s validation ended at 2026-10-01 00:02:11 UTC: actual mean speed approximately 1.4 degrees/s and continuous motion fraction 0.546 demonstrate intermittent tracking. The 00:03:20 UTC query found no output owners. The actual combined data supplies 13 equations and a supported local a/b/two-direction-load information rank of four, condition 4.77831; the full eighteen-coordinate map remains unsupported. Update those measured dynamics through the existing constrained/native fitter before another candidate or expanded speed matrix. Preserve the 0.60-A independent holdout, frozen gyro calibration and its pose limitations. Thirty degrees of travel remains a diagnostic hint, not an acceptance threshold.
Current engineering policy: keep model prediction errors visible as uncertainty evidence, calculate feedback with measured load-equivalent stress cases, and verify actual closed-loop motion. Perfect long open-loop prediction is not a prerequisite for the first bounded feedback probe. Unknown stopping and startup-map qualification remain recorded as unknown. Current, finite duration, real feedback/thermal/STOP protection and declared mechanical bounds remain enforced. Explicit development session labels are now supported; no hash was generated or checked. Formal promotion remains unproven.

Latest calculation: actual12/13 supplies 43 equations and eight supported periodic/posture-tied columns, normalized condition 22.93572. The reusable updater fits those observed coordinates and retains unseen cells, delay and complete earlier measured dynamic alternatives with their failures. The first uncertainty-aware candidate14 invocation rejects all 256 points at unmeasured global yaw/pitch positions; its result is retained. The same fixed solver is now being rerun within the measured local domain. Actual velocity validation is reusable through `adr0022_yaw_validate.py` and reproduces the recorded candidate12/13 metrics exactly. Next physical ordering is prescribed -5 degrees/s from the final measured q1.760242954 rad, then +5 degrees/s with the same candidate through the return path; do not widen the physical case set before these actual tracking results.

Active goal: completely finish ADR-002.2. Yaw stays first; after yaw data, modelling, controller calculation and real 3a/3b evidence, complete the remaining axis and required operating-condition matrix. This checklist tracks the yaw sequence without marking the whole ADR complete.

- [x] 1. Make yaw acquisition runnable in commissiond. Add only the missing current-excitation path to its existing CAN owner and recorder. Run one local normal-path executable probe, then obtain physical acquisition evidence. Freeze edits and builds during physical sessions.
- [x] 2. Measure the yaw baseline at the current fixed pitch pose. Record raw encoder, current, temperature, IMU and actual command/receive timing. Start recording at least two seconds before excitation and retain the full motion and stopping window. Keep noise as calibration data and use existing filtering.
- [ ] 3. Acquire program-selected yaw excitation. Measure positive and negative starting-current intervals, sustained motion, acceleration, deceleration and directional load. Use the prescribed information-based signal selector for additional measurements; do not choose successive stimuli or gains manually.
- [ ] 4. Identify the yaw model. Fit i = a * angular_acceleration + b * angular_velocity + h(position, direction), starting with constrained integral regression and refining against successful transmitted commands and measured motion. Validate predictions on separate runs. Keep unsuccessful starting attempts as censored observations.
- [ ] 5. Calculate and apply one controller offline. Derive load/inertia feedforward and velocity PI gains from the measured model. Evaluate the prescribed bandwidth candidates on the computer with measured delay and uncertainty. Apply the selected parameters through the runtime registry and verify actual readback.
- [ ] 6. Validate 3a, then 3b. Test yaw starting, low-speed tracking, small moves, reversal and stopping in commissiond, then through the actual production chain using the same core and parameters. Record actual outcomes; do not promote an unverified result.

Current implementation sequence: maximum-first preparation, both physical information rounds, actual preceding-TX history repair, repeated fitting, guidance mode, sampled reference support and candidate12/candidate13 +5-degree/s feedback have executed; failures remain recorded. The supported local a/b/directional-load update ran through the existing constrained initializer and native output-error fitter, then the fixed offline controller calculation. Candidate13's same +5-degree/s case ended at 2026-10-01 00:17:23 UTC. Mean gyro/encoder speed reached 5.401/5.413 degrees/s, but actual encoder and IMU confirmed bursts above 20 degrees/s, renewed stalls, 0.896 continuous motion and 6.231-degree/s speed jitter. Final actual zero-current drift passed at 0.132 degrees; tracking and zero-reference stopping still failed. The 00:17:43 UTC query found no output owners. Next: update from these recorded responses and reproduce any observed core fault locally before another calculated candidate; repeat this tracking case before expanding the prescribed speed/move/reversal cases. The reusable updater selects full-map, supported local dynamics, or frozen-dynamics load-only branches mathematically; no agent selects gains. Keep actual movement and all failed windows explicit. Higher-acceleration reference cases wait until presence. Complete yaw 3a before the real production 3b chain. No physical gain search, review, new information stimuli or edge-case expansion.

## Focus rules

- ADR-002.2 yaw commissioning is the active scope. Pitch commissioning and other ADR work are parked.
- No hash generation or checking during development.
- No code review, general cleanup, edge-case development, expanded fault matrices or speculative hardening.
- Before the first physical probe, do only the implementation and normal-path execution needed to make yaw acquisition runnable.
- When an agent-added gate blocks or trips the next script, remove that specific gate first and rerun the same step. Do not pre-emptively clean up unrelated gates. Existing real electrical, thermal, mechanical, feedback-loss and STOP protection remains in effect.
- Reuse ADR-002.2's existing modelling and solver assets. No physical PID candidate search or agent-selected gains.
- Freeze all source edits and builds throughout each physical session; keep one motor-output owner.
- Enforce the latest unattended startup-acceleration requirement before another physical session; keep steady-speed policy unchanged and distinguish startup vibration from steady centripetal acceleration.
- Complete the current item with evidence before moving to the next. Yaw-first progress does not mark the whole ADR complete.

</details>

Method references: [identification repair](docs/08_IDENTIFICATION_REPAIR.md), [execution contract](00_CODEX_START.md), [model and synthesis](docs/02_MODEL_AND_SYNTHESIS.md), [data and adaptation](docs/04_DATA_AND_ADAPTATION.md), [dual validation](docs/05_DUAL_VALIDATION.md).
Physical progress and its limits: [yaw commissioning report](reports/YAW_COMMISSIONING_PROGRESS.md).

## Current evidence

- Corrected full-motion fitting and independent prediction executed. The same native fitter used 0.90/0.45 A training and the frozen gyro column, converged in 239 evaluations, and still failed separate 0.60 A prediction: positive/negative angle RMS 65.921/53.052 degrees, velocity RMS 68.392/79.069 degrees/s. The missing-coast defect is fixed but does not explain the entire failed model. No synthesis or parameter application followed. Evidence: `full-motion-numerical-fit.json` and `full-motion-independent-prediction.json` under ignored `run/adr0022-stage2/yaw-information-01/`.
- The final prescribed initial level, 0.90/3 = 0.30 A, ended normally at 2026-10-01 00:08:21 NZDT (2026-09-30 11:08:21 UTC). The corrected verifier observed only 1.136 seconds of brief body motion and 0.076699 rad (4.395 degrees) combined coverage, despite the COMPLETE footer. Actual numeric validation passed, then the existing initializer rejected INSUFFICIENT_EXCITATION: four equations for 18 unknowns, rank four and 12 unobserved load columns. FailurePolicy selected SELECT_AT_MOST_THREE_INFORMATION_CASES. Raw evidence is under ignored `run/adr0022-stage2/yaw-descended-03/`; the computed sufficiency result is `run/adr0022-stage2/yaw-information-01/descended03-information.json`.

- The next prescribed initial-descent factor, 0.90/2 = 0.45 A, completed at 2026-09-30 23:52:43 NZDT (10:52:43 UTC), using the same deployed binaries. It retained 12,002 yaw frames and 580 gyro samples, peak raw speed +65/-63 rpm and temperature raw 28. Independent excitation-phase motion was observed in both directions for 6.495 seconds; the final zero-current window included -60.381 degrees of coast. This is initial motion-region coverage, not a physical gain search or an information-selector supplemental round.
- Corrected observed defect: the postcapture movement analyzer ended its motion window at the start of final zero current. Positive intermediate coast was retained, but negative final coast was excluded from fitter motion windows. The full raw stopping window was present. Full-motion replay now reports 8.955/8.275/7.117 seconds of independent body motion for 0.90/0.60/0.45 A, including final zero-current deceleration. New `movement-evidence-full.json` files preserve the old reports; no new travel threshold or protection change was involved.

- The existing constrained initializer and C++ native output-error fitter executed on the 0.90 A training capture. The optimizer converged in 71 evaluations, but the frozen model failed the independent 0.60 A prediction: positive/negative angle RMS 98.725/26.756 degrees, versus the 0.15-degree requirement. Velocity errors also failed. These are unqualified numerical results, not a fitted plant certificate. No gains were calculated or applied; Stage 3 remains pending. Evidence: ignored `run/adr0022-stage2/yaw-information-01/numerical-fit.json` and `independent-prediction.json`.
- Frozen initializer comparison also executed. Output-error refinement improved the existing aggregate prediction score on training and holdout, while holdout velocity RMS worsened in both directions. Relative improvement cannot replace the absolute targets. The existing FailurePolicy returned `STOP_PROMOTION_MODEL_INADEQUATE`; evidence is `initializer-comparison.json` in the same ignored directory.
- Sensor capability analysis on training data measured gyro/encoder disagreement of 0.113 rad/s RMS at sensor sample timestamps. The excited coherent agreement band was 0.695–1.738 Hz; the existing causal-filter diagnostic returned MODEL_INADEQUATE. Sampling near 50 Hz does not establish a 50 Hz observation bandwidth. The gyro column, holdout and source were not changed by this analysis.
- The existing encoder converter reproduced all four captures' raw-count phase registration and unwrapping. Maximum adjacent raw-count step was 33 counts, and decoded CAN angle bytes agreed with every yaw feedback record. This supports the numerical periodic representation; it does not certify absolute world zero or full physical coordinate calibration.

- Maximum-first 0.90 A physical acquisition completed with approximately 7.381 seconds of sustained body rotation independently confirmed by encoder and IMU; 12,002 yaw frames were retained. The next prescribed descending level, 0.60 A, completed as a separate run for model holdout. Both sessions ended normally and their raw data is retained under ignored `run/adr0022-stage2/yaw-maximum-stationary-01/` and `yaw-descended-01/`. No candidate/controller/Stage 3 qualification has been claimed.

- Latest owner policy starts yaw at the applicable maximum continuous current and uses measured response rather than a fixed travel threshold. The executable movement analyzer reports encoder and independent gyro/quaternion evidence on actual timestamps. Existing 0.25 A data has 0.291 s of actual body rotation, approximately 1.1 degrees; the later hold remains a plateau. No model or Stage 3 qualification follows from this observation alone.

- Owner steering: obtain one additional sample with a 30-degree yaw target before continuing item 3. The existing first-stimulus program selected 0.25 A again. Manifest generation actually hit the 500 ms pulse-only guard; it now permits finite continuous-current captures within the manufacturer's 0.9 A continuous stall allowance. The normal executable endpoint probe passed, then the physical session completed at 2026-09-30 22:08:53 NZDT (09:08:53 UTC). Its finite 15-second excitation reached only 1.142578 degrees; final displacement after two seconds of zero current was 0.834961 degrees. The target was not reached. Preserve this response in ignored `run/adr0022-stage2/yaw-30deg-01/station/`; it does not count as 30-degree coverage. Item 3 now uses both captures.

- First local normal-path run: complete; 5,600 raw yaw frames, positive/negative/zero current transmissions, all four IMU streams retained and final current zero. Synthetic evidence is in ignored `run/adr0022-stage2/yaw-local-probe-01/`.
- Encountered blocker: initial signal selection required previously qualified stopping. Removed that qualification prerequisite from `first_stimulus`; the finite acquisition session records actual zero-current and stopping observations instead.
- First physical acquisition completed at 2026-09-30 21:24:53 NZDT (08:24:53 UTC). It retained 7,002 yaw frames, 584 pitch responses and all four IMU streams. Reported yaw current reached approximately +/-0.250 A, encoder span was 34 counts (1.494 degrees), and final zero-current drift was one count (0.044 degrees). Full evidence is under ignored `run/adr0022-stage2/yaw-physical-probe-01/station/`.
- Station session ended normally; no controller or IMU owner remained. Current zero is recorded; physical stopping qualification, fitted plant and 3a/3b remain pending.
- ARM64 compiled on the workstation. Its user-mode emulator could not expose required receive metadata; this emulator attempt is retained as an environment limitation. The physical ARM64 acquisition completed on the actual station.
- Baseline analysis: 2,001 actual yaw receptions over approximately two seconds, current scatter 1.616 mA, encoder detrended scatter 0.000371 rad, approximately 50.09 Hz gyro receptions, and recorded pitch native MechPos approximately -0.813148 rad. Detailed observations are in ignored `run/adr0022-stage2/yaw-baseline-01/`.
- Existing sustained-motion estimator produced positive total-current interval 0.14209 to 0.20306 A and negative interval -0.08203 to -0.14099 A for these ramps. The program selected `CONTINUE_PROGRAM_INFORMATION_SELECTION`; it did not request a larger initial starting ramp. These are local observations, not a complete directional load map.
