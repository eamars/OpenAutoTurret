# Architect amendment: identification repair and prospective validation

The owner's later [architect review 02 amendment](09_ESTIMATOR_RECOVERY.md) updates
the immediate priority to estimator recovery, separated gates and fair comparisons.
Its recovery routes supersede any interpretation that MODEL_DATA_FAILURE terminates
safe analysis. Current physical evidence is whole-run; earlier window criticisms below
describe the superseded fits only.

Authority: the owner's 2026-10-01 instruction to prioritize the complete
[independent architect review](../architect_review_01/ADR-002.2-independent-review.md).
The review's [evidence ledger](../architect_review_01/EVIDENCE.md) records what was
recomputed and what remains a supplied result. This amendment supersedes conflicting
identification, model acceptance, Stage 1 completion and next-trial instructions in
this package. The [offline/physical stage separation](07_STAGE1_OFFLINE_OVERRIDE.md),
both-axis scope, Stage 3a/3b, runtime parameters and real protections remain mandatory.

## 1. Current decision and limits

Yaw is **UNQUALIFIED**. Candidate14 is **NONDEPLOYABLE**, was never physically run,
and remains retained diagnostic rejection evidence. A further coefficient fit of
the restricted moving-load model, optimizer convergence, acceptable mean speed or
arithmetic core replay cannot authorize another controller trial. No station access,
deployment or physical motion is part of this local architecture repair.

The old Stage 1 handoff remains historical evidence for its then-current contract.
Stage 1 under this amended contract is **IN_PROGRESS** until the executable family,
estimator checks and prospective validation requirements below have evidence.
Architectural text, audit success and new local tests do not establish a physical
plant or complete Stage 1 by themselves. Record architecture, implemented branches,
synthetic estimator evidence, physical calibration, plant identification and 3a/3b
separately. Unsupported branches and absent evidence must say NOT_IMPLEMENTED or NOT_RUN.

Preserve all raw captures, failed fits, Candidate12/13 failures, Candidate14 rejection,
old reports and calibration limitations. The 29 latest MOVE windows belong to three
physical runs and are in-sample reconstructions. Rank eight in observed columns does
not qualify the full eighteen-coordinate map. Tied posture rows and unseen cells
remain prior-only. The 0.60-A run is a retrospective regression holdout that has been
inspected; it is not an unseen prospective low-speed validation experiment.

## 2. Predeclared bounded model family

Stage 1 defines structure, states, units, constraints, estimation and selection;
Stage 2 supplies physical values and selects within that definition. Synthetic values
never become physical calibration. No model structure is added silently during a
payload update, and no agent selects physical gains or stimuli from narrative metrics.

### Actuator and current observation

The admitted actuator alternatives are `ALGEBRAIC` and `FIRST_ORDER`:

`i_eff = g_i * u_TX(t - delta_i) + i_bias`

`T_i * d(i_eff)/dt + i_eff = g_i * u_TX(t - delta_i) + i_bias`

Use bounded, identifiable `g_i`, `i_bias`, `delta_i` and, only for FIRST_ORDER,
positive `T_i`. Saturation and slew are the declared actual command/actuator rules;
temperature and supply dependence need support before being estimated. Reported
current has a separate observation scale, filter, latency and uncertainty. Successful
kernel acceptance is not measured torque application. Slow slew data cannot identify
a precise electrical time constant.

The former 60.593 ms assumption is reopened, not silently changed to the review's
1–2 ms descriptive telemetry alignment. Command-to-current dynamics, breakaway delay,
gyro/encoder latency and transport timing are distinct terms or explicit bounds.
Unsupported delay components remain unresolved and block their dependent qualification.

### Mechanical parent and local reduction

The two-axis parent remains:

`M(q; pi_c) q_ddot + C(q,q_dot; pi_c) q_dot + g(q; pi_c,g_base)`
`+ tau_f(q,q_dot,xi,T;c) + tau_cable(q,chi;c)`
`= B_tau K_t(T) i_eff + tau_disturbance`.

Document axis signs, shaft/output transmission, frames, mounting and kinematics.
`c` binds payload, geometry, centre of mass, inertia distribution, mounting/cable
setup, posture, temperature and supply domain. Fixed-pitch yaw is the immediate
reduction; it does not identify coupled pitch or global posture coverage:

`a_c * v_dot = i_eff - load(q,c) - B_c*v - F(v,xi,q,T,c)`.

Keep signed spatial load separate from dissipative friction. A periodic motor load
may use a periodic map where justified; cable winding/history is not forced into
the same periodic map. Unobserved cells are not zero-valued measurements.

Without an independent torque/current datum, identify current-equivalent `a=J/K_eff`
and supported combinations, not separate mechanical inertia and torque gain.
Payload mass alone is not an inertia signature: its distribution and perpendicular
axis radius matter. Geometry and operating-point changes use reusable method assets
and new parameter snapshots, not an automatically valid old certificate.

### Friction states and admitted alternatives

The initial executable family has four combinations:

| Actuator | Moving friction | Required transition semantics |
|---|---|---|
| ALGEBRAIC | COULOMB | True stick, breakaway, sliding, stop/reattach, reversal |
| FIRST_ORDER | COULOMB | Same states plus continuous actuator state |
| ALGEBRAIC | STRIBECK | Same states with speed-dependent sliding friction |
| FIRST_ORDER | STRIBECK | Same states plus continuous actuator state |

In the local equation, `F` is the nonviscous friction term; `B*v` appears separately
and is added only once. Direction-specific sliding `F` is either `sign(v)*F_c^d`, or

`sign(v) * [F_c^d + (F_s^d-F_c^d)*exp(-(|v|/v_s^d)^p)]`.

Use nonnegative physical friction/viscous parameters and `F_s^d >= F_c^d` for the
Stribeck family. At rest, friction balances the other torques inside the directional
static interval; `sign(0)=0` is not a holding-friction model. Breakaway, stopping,
reattachment and reversal preserve actuator/friction history. Failed starts are
censored inequalities, successful starts are timing/slew-dependent intervals.
Carry approach direction, prior motion, dwell, position, posture and temperature.

LuGre/presliding and compliance/backlash/two-inertia are **nonselectable hypotheses**
in this initial repair, not identified mechanisms or silently available branches.
Repeated dwell/approach/reversal evidence can require a history model; systematic
rotor/payload divergence can require compliance. If the admitted family cannot explain
those observations, return MODEL_INADEQUATE, state the missing distinction and extend
the predeclared executable contract with estimator/synthetic evidence before selection.
Do not add arbitrary per-window offsets, negative constant viscosity or hidden-state
resets to disguise the unsupported mechanism.

### Measurement model

Represent quantized encoder position, session registration, native gyro projection
and bias, current telemetry semantics, causal filtering, timestamp mapping and sensor
freshness. The frozen single-yaw gyro column is valid only at its measured posture;
two-axis qualification requires the appropriate mounting/kinematic mapping. Gyro and
rotation-vector channels from one IMU are not independent physical instruments.
Native encoder evidence is a separate modality. One-count velocity differences and
integer RPM are not exact low-speed velocity observations.

## 3. Canonical evidence, clocks and coordinates

Each normalized event retains source archive/member or journal path, original line,
physical run ID, configuration ID, calibration revision, signal/units/frame, clock
identity, timestamp provenance, generation/reset counter and validity flags. Derived
events retain parent references and transformations. Mark physical, replay, simulation,
initialization and prior evidence separately. No new hash generation/checking is required.

Retain requested, post-limit/post-slew, successful kernel-accepted and reported motor
current as distinct channels. Reconstruct successful TX with causal zero-order hold,
including actual pre-window history. Do not interpolate steps into ramps or invent
zero input when preceding successful TX is known.

Map device/sample clocks to the common monotonic clock only with calibrated or bounded
offset, drift, uncertainty and reset handling. Retain receive/dequeue timestamps as
transport evidence; label receive-time-only observations explicitly. Passive pitch
receipt ZOH does not establish sample time. A reused 50 Hz gyro sample in a 200 Hz
control loop remains one observation, with age recorded.

Unwrap encoder per session and register shaft phase with an explicit supplied datum
and calibration. Keep shaft phase, accumulated winding, session-relative displacement
and world angle separate. A common diagnostic count is not world zero. Transform
pooling requires documented compatible registrations; preserve original values.
All rest, start, positive/negative motion, coast/deceleration, reversal, saturation
and uncertain transition intervals remain available even when excluded from a
particular initializer. Preserve the complete final stopping window.

## 4. Estimation, run blocks and selection

Freeze the split manifest **before** learning calibration, timing, noise, window rules
or structures. Allocate whole physical runs to TRAIN, SELECTION and FINAL_VALIDATION;
group repeated session/day blocks where available. No adjacent windows from one run
may cross these roles. Previously inspected runs may support retrospective comparisons,
but cannot be relabelled unseen. A missing independent final block is NOT_RUN and
blocks final predictive acceptance.

Use integral regression as a constrained initializer, then fit complete forward
trajectories to native quantized position, gyro and current observations, jointly
estimating supported parameters and bounded initial latent states. Multiple shooting
must constrain segment continuity. Predicted spatial/friction terms use predicted
state. No measured future motion or arbitrary short-window state reset may repair a
free-running prediction. Report identifiable, bounded-only and prior-only parameters.

Closed-loop data require a justified prediction-error/noise, instrumental-variable or
equivalent estimator. Nonlinear output-error fitting and Huber loss alone do not prove
absence of feedback/noisy-regressor bias. Stage 1 must execute an independently
generated synthetic closed-loop estimator check with correlated feedback input,
native multi-rate sampling, quantization, timing/filter errors, saturation/slew and
state transitions. Declare tolerances before execution, report recovery/prediction
errors and bias over independent generated runs, and reject unsupported estimators.
Use the actual controller/observer semantics in this check; measured station values
are not prerequisites. Mere synthetic model replay is not this estimator check.

Current estimator validity for feedback-generated data is **UNVERIFIED**. Initial
physical structure comparisons train on open-loop current-excitation runs; feedback
journals serve diagnostic held-out comparisons. A bounded deterministic output-error
fitter must not claim to be unbiased in feedback. The synthetic closed-loop check
can falsify its suitability; failed or absent estimator evidence blocks promotion.

Select the least complex supported structure that passes all declared run-block
trajectory and residual gates; use fixed structure/parameter-count ordering, then
selection loss and a stable identifier for deterministic ties. Freeze numeric bounds,
loss scaling, budgets and thresholds in machine-readable configuration before fitting.
Final validation is evaluated once per frozen selection, never used to pick that
selection. If it fails, preserve it and revise with a new split/decision record.

Assess residual structure versus angle, speed, acceleration, current, direction,
start/reversal age, posture and temperature; test innovation whiteness and appropriate
orthogonality to past information/exogenous excitation. Future controller action can
respond to a prior error, so its correlation alone is not a rejection rule. Use whole-run
uncertainty/profile checks only when independent runs support them. Distinguish plausible
identified uncertainty, deliberate stress models and already contradicted hypotheses.
Check band coverage **and width**; broad bands cannot waive the precision targets.

## 5. Model freeze, synthesis and tests A/B/C

Only a supported configuration-specific predictive model can be frozen for synthesis.
Its record binds family/version, hardware/firmware/mode, payload geometry, mounting/cable
setup, calibration/timing semantics, estimates, uncertainty, supported domain, split
manifest, reference/limits and validation results by explicit paths and revisions.
Failed alternatives remain diagnostic. A failed model cannot receive a bounded
controller trial merely because current, duration or stopping are constrained.

For nonlinear friction, synthesize against local incremental dynamics, including
the sliding-friction derivative, actuator and observation dynamics. The former ideal
`a/b` PI formula is only an eligible local special case. Include nonlinear full-manoeuvre
checks using the shared core, actual observer, limits, anti-windup and transitions.
All offline points failing returns a failure, never the last point as a candidate.

| Test | Input and permitted observations | Meaning |
|---|---|---|
| **A: input-driven plant validation** | Held-out complete run; initial observations and realized successful TX history; no future motion injection | Retrospective plant prediction conditioned on realized inputs |
| **B: prospective closed-loop forecast** | Frozen configuration/calibration/model/uncertainty/controller/reference; simulated sensor/actuator paths generate their own future current | Timestamped forecast saved before the physical run |
| **C: physical qualification** | Execute that frozen trial; compare native measurements with saved B trajectories/bounds and unchanged targets | Required physical 3a, then real production 3b |

Before C, save B's predictions, horizons, uncertainty bounds, initialization rule,
reference and limits with a reliable creation time and revision. Future recorded current
is allowed in A, never in B. Future sensor samples may not enter B's causal observer.
Any changed model/controller/calibration/domain invalidates the old B and dependent
certificates. Prediction-file creation alone grants no deployment or motion permission.

Report at 50, 100, 200 and 500 ms, 1 s and full-run horizons, with angle/velocity errors,
current, endpoint drift, startup/false starts, stall duration/continuity, reversal,
settling, zero-reference stop drift, overspeed and saturation. These are reporting
horizons, not alternative acceptance limits or windows selected after seeing errors.

The [frozen dual-validation targets](05_DUAL_VALIDATION.md) remain unchanged: independent
angle prediction RMS 0.15 degrees; speed error/jitter at 5 degrees/s at most 0.5 degrees/s;
mean tracking 0.90–1.10; continuity at least 95%; position P95–P5 0.15 degrees and
full span 0.30 degrees; startup at most 200 ms; fixed two-second **zero-reference**
drift at most 0.15 degrees. Later zero-current settling is a separate observation.
Retain all other case-dependent targets and both environment comparisons. Reference
shaping at 30 degrees/s² remains guidance, not a demonstrated physical acceleration bound.

## 6. Explicit outcomes and next work

| Outcome | Required action |
|---|---|
| ACCEPTED_WITHIN_DOMAIN | Record predictive gates and a supported model freeze; synthesis may begin, physical qualification remains NOT_RUN |
| DATA_INVALID / MEASUREMENT_LIMITED | Report provenance/clock/registration/sensor limitation; repair dependent evidence without relaxing targets |
| INSUFFICIENT_EXCITATION / MORE_INFORMATION_NEEDED | Record unsupported distinction; deterministic bounded selector requests only discriminating evidence |
| MODEL_INADEQUATE / STRUCTURALLY_REJECTED | Preserve full failed trajectories/residuals; no candidate application or gain trial |
| ENVELOPE_LIMITED / PERFORMANCE_INFEASIBLE | Identify current/slew, sensing, friction, uncertainty or envelope constraint; no last-grid-point deployment |
| OPERATING_POINT_CHANGED / INTEGRATION_MISMATCH | Invalidate affected applicability/certificates and use the existing update or integration repair procedure |
| HARD_ABORT | End through real protection/STOP procedures; preserve evidence and resolve the actual hazard |

The order is: reproduce audit → normalize provenance/clocks/coordinates → implement and
verify the declared family/closed-loop estimator → compare existing-data structures and
run blocks → report blocked distinctions → acquire only necessary discriminating evidence
under the station operation cards → predictive model freeze → synthesis → saved prospective
forecast → 3a → production 3b → both axes and full operating-condition matrix.
No present diagnostic result authorizes a physical Candidate14 run. Yaw-first work
does not remove pitch, simultaneous-axis coverage, payload/friction changes, return to
baseline or undeclared-change detection from the complete delivery.
