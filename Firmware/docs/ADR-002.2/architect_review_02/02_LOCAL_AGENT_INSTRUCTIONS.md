# Local-agent instructions — diagnosis, recovery, and identification

## 1. Decision and ownership

Continue ADR-002.2 in the existing implementation. Do not replace the task with another report-only loop, a manual PID search, or a larger collection of equally unverified friction models. Do not discard the whole implementation: native masks, continuous whole-run predictions, retained input prehistory, explicit splits, and the pristine synthetic result are useful progress.

The prior review's direction to “return a model/data failure” meant **reject promotion**. It must not be implemented as “terminate the task whenever no model passes.” Add a recovery transition to that outcome. The next action must depend on the failed mechanism, not merely the final Boolean.

Respect the single full-delivery contract: Stage 1 defines reusable mathematics, admissible structures, methods, interfaces, synthetic tests, and fault rules without requiring physical measurements. Stage 2 obtains physical evidence, estimates supported parameters, and injects them automatically. Stage 3a and 3b validate the independent program and production software physically. Current evidence can inform this architectural revision, but Stage 1 must remain runnable with synthetic fixtures alone.

Use concise, inspectable **decision records**, not requests for private chain-of-thought. A record needs the observation, competing explanations, discriminating test, result, changed decision, and next action. It should be sufficient for another engineer to reproduce the decision.

## 2. WP1 — Reproduce and classify the actual failure

### Required inspection

Find the real source named by `estimator/frozen-contract.json`, including `Firmware/tools/adr0022_closed_loop_estimator_probe.py`, its plant/residual dependencies, and the native shared control library. Verify these paths in the local checkout before use. Inspect only relevant files. Record source revision, native binary identity, Python/NumPy/solver versions, compile options that affect numerical behaviour, and the exact replay command.

The supplied archive reports an original offline command using that script, `--library run/adr0022-local/core/libaxis_control_core.so`, an output directory, `--duration-s 8`, `--seeds 17 41 83`, and `--max-nfev 120`. Reproduce it only after confirming that the local entry point is the same offline synthetic test and does not access the station. Use a new output directory, never overwrite the frozen evidence.

### Separate flags

For every candidate record:

```
data_integrity
forward_numerics
optimizer_termination_reason
optimizer_converged
synthetic_parameter_recovery
training_trajectory
selection_trajectory
historical_regression
prospective_prediction
physical_stage3a
physical_stage3b
deployment_authorized
```

Represent unperformed checks as `NOT_RUN` or `UNKNOWN`, never false evidence of failure or implicit success. A candidate can have an exhausted optimizer budget and good parameter/trajectory metrics; it can also have `xtol` convergence and terrible predictions. Neither is a deployable result without all required gates.

Keep the existing numerical recovery and prediction thresholds. In particular, the synthetic position gate near 0.0008023 rad is a **different test** from the physical 0.15-degree angle gate. Do not swap them to make seed 41 pass.

### Required output

One reproduction report with each failing predicate separately identified. Do not summarize it as “the estimator cannot recover the model.” For the supplied records: all three noisy cases meet the 5% parameter threshold; all exhaust 120 evaluations; the seed-41 fit fails two cross-run position predictions. The pristine test passes within its narrow, known-nuisance subspace.

## 3. WP2 — Repair the smallest synthetic failure before broad physical selection

The following is a diagnostic decision tree, not a demand to implement every possible numerical technique. Preserve a minimal deterministic regression for each confirmed defect. Make one causally interpretable change at a time.

### 3.1 Establish an oracle ladder

Evaluate the existing native simulator at the declared true synthetic parameters and compare with the independent synthetic generator, first pristine and then at the noisy runs' realized successful inputs. Reproduce the existing good pristine result before changing anything.

Distinguish:

- **Forward-model disagreement at truth:** units, clocks, zero-order hold, friction transitions, state initialization, sensor equations, or integration accuracy.
- **Forward agreement but optimizer cannot find a comparable fit:** scaling, derivative quality, nonsmooth event handling, initialization, loss geometry, or termination.
- **Good training fit but poor independent prediction:** insufficient excitation, nuisance assumptions, statistical objective/noise treatment, or overfitting.
- **Good narrow fixture but real-data mismatch:** physical input/measurement uncertainty, configuration inconsistency, or model structure remains unresolved.

Using truth as a diagnostic reference is allowed in synthetic tests. Starting the final benchmark at truth, fixing formerly free parameters to their known values, or changing test noise to claim recovery is not.

### 3.2 Verify integration and hybrid events

For fixed parameters and input history, reduce the integration step and tighten tolerances until changes in predicted native observations are comfortably below the frozen numerical error budget. This must include zero crossings, release from sticking, stopping, delayed input changes, and sensor filtering. Test both signs.

Split integration at known successful-command discontinuities and their delayed application times. Respect actual event timing rather than snapping every delay to a controller sample. At velocity zero, resolve whether the admissible static-friction interval can hold the load; do not use `sign(0)=0` to remove friction. Detect reattachment without oscillating numerically between positive and negative motion.

Inspect whether small parameter perturbations alter event times continuously enough for the chosen estimator. If adaptive integration or fixed-step event detection creates a jagged objective, fix or account for it before assigning the jaggedness to the hardware. Standard event solvers may miss multiple zero crossings inside one step; this is a reason for event-specific tests, not a universal fixed time-step prescription. [S3]

Maintain separate numerical-domain and physical-domain guards. The huge mathematical angle bounds in the affine fits are not valid travel bounds. Never clip a simulated angle into the observed range. On mathematical divergence, report where and why the candidate failed; do not return a flat, identical penalty for every invalid parameter vector and then interpret a zero gradient as model identification. Use admissible parameter constraints, meaningful partial-run diagnostics, and bounded trust-region/profile steps to find valid candidates.

### 3.3 Inspect the actual objective and derivatives

Log raw channel residuals, their normalization, the loss contribution per run/channel/regime, the fitted initial states, bound hits, optimality measure, step size, and event counts. Determine whether the numerical Jacobian is computed from raw or robustified residuals and use the correct interpretation. Report local sensitivity separately from global identifiability.

Scale parameters to characteristic units. Current-equivalent inertia, friction amplitudes, millisecond delays, filter constants, and initial angles should not all have an implicit characteristic scale of one. Use an explicit dimensionless parameterization or justified characteristic scales. Do not derive parameter scales from enormous numerical angle guard bounds.

For each critical coordinate, compare perturbations at several **absolute physical step sizes** and inspect the resulting residual and event-time changes. Include offsets or zero-valued time constants where a relative perturbation can be uninformative. A small step that changes no discrete event does not show the parameter has no physical effect.

SciPy's least-squares interface finds a local minimum; its `xtol` flag concerns parameter-step termination, and `x_scale` and `diff_step` have distinct roles. Complex-step derivatives require analytic continuation, so do not blindly enable them through `abs`, `sign`, clipping, or hard state transitions. Check the installed solver version instead of assuming documentation-version defaults. [S2]

The two algebraic Coulomb fits specifically report zero local sensitivity for both static-excess coordinates. Profile those thresholds across physically meaningful brackets; compare predicted versus observed start/no-start outcomes. Use censored inequalities or interval likelihoods for start thresholds, coupled to the load at that position. A bounded outer threshold search plus an inner smooth moving-parameter fit is an admissible repair. It is an automatic identification method, not manual controller tuning.

An optional smooth continuation can initialize a difficult fit, but acceptance must use the actual declared hybrid model and its transitions. Do not silently replace true sticking with a smooth sign approximation for the final prediction test.

### 3.4 Use seed 41 as a discriminating regression

Reproduce its two failed cross-run predictions. Compare true-parameter and fitted-parameter trajectories at identical input histories and initial conditions. Inspect where the trajectories first diverge: moving acceleration, near-zero threshold crossings, or accumulated load/damping error. Keep filtered gyro prediction separate from latent velocity.

Run bounded profiles in the four free coordinates and selected pairs, using training data only to choose a fit. Compare the existing initialization, a small predeclared multistart set, and a restart from the retained best point. Record whether different starts reach the same region and whether the objective is still decreasing at the budget boundary. Include the retained good seed-17/83 fits as **diagnostic cross-evaluations**, not as a secretly selected winner based on all future tests.

Extend the evaluation budget from a checkpoint when documented improvement or a repaired derivative warrants it. Do not merely change 120 or 200 to a much larger number and rerun the same stalled search. Conversely, an arbitrary finite budget is not a mathematical impossibility certificate. Keep evaluation counts and total wall time, including Jacobian work.

### 3.5 Test the noise/closed-loop assumptions

Use the independent generator to separate encoder quantization, encoder noise, gyro noise, reported-current noise, and feedback-induced changes in input. Start with one source at a time, then the full declared combination. Retain realistic channel sampling and sample freshness.

Compare raw least squares and justified robust/likelihood treatments only with explicit channel/noise assumptions. Robust loss reduces outlier influence; it does not establish absence of closed-loop estimation bias. A feedback-generated input is not statistically interchangeable with an externally prescribed noiseless excitation. Select a supported prediction-error/noise model, instrumental-variable approach, or equivalent estimator when the controlled synthetic tests show it is needed; do not add a statistical subsystem without such a diagnosis.

Quantized encoder observations can be represented by their observation intervals, with additional noise modelled separately. Do not divide quantization into a claimed exact instantaneous velocity. Avoid treating repeated filtered samples as independent innovations.

After the repair, predeclare additional noise seeds and genuinely different excitation/reference trajectories. Keep development and final synthetic verification partitions separate. Repeating the deterministic pristine trajectory under three seed labels is one excitation case, not three independent tests. Do not choose only seeds that pass or inflate noise bands until a structured error disappears.

### 3.6 Synthetic exit criterion

Preserve the original regression gates and show reproducible convergence behaviour, parameter recovery, and cross-run forward prediction. A configured budget may be resumed, but its final result must not simply ignore a failed predicate. Document any justified change to a *termination procedure* separately from unchanged engineering quality limits.

Verify additional actuator/friction/measurement structures on their own synthetic fixtures before using their rejection to draw physical conclusions. The current four-coordinate Coulomb fixture does not qualify Stribeck scale identification, unknown current dynamics, load-map estimation, or structure selection.

Do not wait for perfect universal statistical theory before using a passing, clearly bounded estimator. Equally, do not claim global unbiasedness from three seeds. The deliverable is a useful method with tested scope, sensitivity, and explicit limitations.

## 4. WP3 — Establish the physical input and measurement contract

Complete `03_GM6020_INTERFACE.md` in parallel with offline recovery. Each run's usable configuration must distinguish verified facts, reported settings, assumptions, and unknowns. Exact physical torque calibration is not automatically required: a stable, verified command-to-motion lumped model can be useful. But an unknown command mode or ambiguous sign/scale cannot be hand-waved away.

Retain four separate current channels: requested, post-limit/post-slew, successfully transmitted, and reported motor current. Reconstruct successful TX as causal ZOH with prehistory. Kernel acceptance is not measured physical application. The near-current correspondence in the prior review supports testing a fast electrical path; it does not establish a 1 ms physical torque time constant.

Keep distinct transport latency, regulated-current response, mechanical breakaway delay, sensor filtering, and sensor timestamp uncertainty. The host-based clock audit verifies arithmetic and observed transport; it does not qualify the physical sensor sample time, oscillator drift, or internal filter.

Retain native masks. Use projected gyro only within the validated mounting/posture domain. Keep shaft phase, session-relative angle, winding count, and world angle separate. A capture/procedure ID is not a physical configuration fingerprint. Audit whether payload, cable routing, mounting, motor settings, and temperature are actually compatible across pooled runs.

Source-to-binary and parser checks are finite software tests. Do not make hardware calibration a prerequisite for testing them. Conversely, a correct raw-byte decoder is not proof of an accurate physical torque signal.

## 5. WP4 — Identify physical structure without parameter proliferation

### 5.1 Deconfound the comparisons

Repeat the constant-versus-affine load comparison with **the same measurement/actuator parameter freedoms and bounds**. The supplied affine fits also free current delay and current-filter time constant. They are therefore joint changes, not a clean load-only ablation. A crossed comparison can then test the additional measurement dynamics separately.

Likewise, keep comparable initial-state policies, input timing, data partitions, and loss weights when comparing Coulomb and Stribeck. No model gets arbitrary per-window offsets or state resets. Multiple shooting is acceptable only with continuity constraints and a final uninterrupted rollout.

### 5.2 Identify combinations the data can support

For moving yaw, an A-equivalent model may be written

```
a * v_dot + B * v + L(q, posture, winding, configuration)
             + F(v, friction_state, temperature, configuration) = i_effective.
```

At a fixed position, positive and negative moving loads can be `L + Fc_positive` and `L - Fc_negative`. Freely estimating all three admits the gauge transformation

```
L' = L + d;  Fc_positive' = Fc_positive - d;  Fc_negative' = Fc_negative + d.
```

It leaves those two moving sums unchanged where the transformed parameters remain admissible. Static thresholds have an analogous confounding. Thus do not simply free the currently fixed zero load offset alongside independent directional friction and declare the problem solved. Identify total directional load/threshold surfaces, impose a documented gauge/reference constraint, or add independent evidence that separates conservative load from asymmetric friction. Do not force unsupported symmetry merely to get a unique number.

Similarly `a = J/K_effective` is a lumped input-to-acceleration parameter. Do not demand separate absolute inertia and torque constant without independent support. Keep units and configuration dependence explicit. Parameter uncertainty must reflect the identifiable combinations, not only a full-rank local Jacobian containing many fitted nuisance initial states. [S4]

### 5.3 Do not let no-motion or pre-excitation scores masquerade as prediction

Add explicit comparisons to constant-position and constant-velocity baselines, predicted/observed displacement, start events, moving duration, reversals, and stops. Beating a baseline does not replace the absolute engineering gate; failing to beat it is a valuable diagnostic.

Evaluate horizon errors anchored at command changes, observed motion onsets for retrospective diagnosis, reversals, and stopping, plus the whole run. For prospective evaluation, predeclare reference-event anchors or label outcome-triggered diagnostics separately. Slice errors from the same continuous prediction; do not reset to future measured states at each anchor. Early subsecond scores during pre-excitation rest are not evidence of tracking accuracy.

### 5.4 Balance information, not merely sample count

Report each run's contribution to the objective. The current dataset includes roughly one-degree creep and multi-thousand-degree excursions with speeds around 1,000 degrees/s. Establish whether long/high-excursion campaigns dominate the optimization; do not assert they do without the loss breakdown.

Predeclare weighting by run, independent sensor information, and control-relevant regimes using training data. High-speed segments may initialize moving dynamics; low-speed starts, sustained tracking, reversals, and stopping must influence the final fit and pass their own gates. Do not delete difficult data, tune weights against the final holdout, or silently shrink the promised domain to obtain a pass.

A local model is a legitimate intermediate scientific result if labelled with its support. It does not qualify the full yaw/pitch/payload delivery. Use it to understand and parameterize the full reusable model, not to redefine success.

### 5.5 Expand only when residuals discriminate

Start with the least complex structure whose estimation procedure is verified. Test speed dependence, spatial dependence, direction, dwell/reversal history, and actuator/measurement dynamics with controlled ablations. Use a predeclared dynamic-friction alternative only if history effects persist under sound timing and estimation. Test compliance/two-inertia behaviour only if independently mapped rotor/payload observations support it.

Do not assume all angle effects are periodic: winding/cable load can be nonperiodic. Do not extrapolate a local affine law over thousands of revolutions. Identify a supported interpolation law and domain. Correct repeatable differences between physical configurations before attributing them to one elaborate universal friction curve.

Select using blocked whole runs and independent trajectories. Save prediction files, residuals, initial state, parameter revision, and all failed alternatives. A lower aggregate cost is not a substitute for passing each required quality predicate. Equation fitting is useful initialization; complete simulation error remains essential. [S4]

## 6. WP5 — Obtain only the missing discriminating physical information

Acquire no new motion merely because the previous batch failed. First state what existing data cannot distinguish, the quantitative accuracy or interval needed, and how a proposed experiment changes identifiability.

Useful designs include matched positions at different established speeds, both directions, acceleration/coast/deceleration, dwell/restart/reversal, and controlled configuration changes. Current-path timing should be separated from first-motion timing. Where practical, use balanced repetitions/order to avoid mistaking temperature or cable-history drift for speed dependence.

The experiment planner must use validated current, slew, speed, travel, winding, thermal, supply, and timeout constraints, plus a physically adequate stop mechanism. Unknown limits remain unknown; manufacturer ratings and historical peak excursions are not automatically permitted experiment limits. Do not escalate current after a censored no-start trial unless a separately approved plan allows it. A current cap alone does not bound travel or prevent overspeed.

Stage 2 should execute this reusable selection/identification procedure and automatically update supported parameters. Changed payload or friction is not a request for a new architecture or human PID trial-and-error.

## 7. WP6–WP7 — Synthesize, predict, and validate

Use `04_FEEDFORWARD_AND_VALIDATION.md`. Controller implementation and synthetic unit tests can continue while the physical model is unqualified. Deployable gains and physical execution remain gated.

Retain the existing historical holdout policy for this comparison. The old feedback runs were used in earlier development: they are valuable held-out **regressions for this fit**, not genuinely unseen lifetime validation. Any change informed by a selection run makes that run development evidence for subsequent decisions. A genuinely prospective freeze-before-measurement experiment remains necessary.

When no controller passes, return a typed diagnostic with the model set, limiting criterion, and evidence. Separate sensing limitations, current/slew feasibility, model uncertainty, and mechanical load. Route back to the relevant work package. Do not return the last searched candidate as deployable or synthesize against all historically rejected fits as if they were a calibrated confidence set.

## 8. Persistence and escalation protocol

Every work cycle must produce either a verified improvement, a falsified hypothesis, a narrower uncertainty, or a precise external blocker. A giant new log without a changed conclusion is not progress.

Use bounded chunks of computation with saved checkpoints. A practical default is to allow up to three *distinct, justified* diagnostic strategies on an unresolved branch before an explicit branch review. That review selects another feasible branch or formulates the exact external blocker; it is not permission to terminate the overall task. Do not repeat identical failed commands indefinitely, spend unbounded compute, or consume holdouts looking for a lucky result.

Before requesting human input, inspect the local source, supplied evidence, read-only device metadata available under authorization, and safe offline alternatives. Request only information not resolvable there. State what work can continue independently. An unavailable station can block a physical gate without blocking implementation, synthetic verification, or a capture plan.

Never change frozen acceptance thresholds, silently omit an axis, pretend a simulation is a physical trial, invent missing facts, or remove a fault guard to obtain motion. Stopping unsafe motion is correct. Abandoning all safe diagnosis merely because motion was stopped is not.
