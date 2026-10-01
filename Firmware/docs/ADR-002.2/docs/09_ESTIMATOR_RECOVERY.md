# Architect review 02: estimator recovery and fair comparison

Authority: the owner's instruction to follow [architect review 02](../architect_review_02/00_START_HERE.md),
including its [local instructions](../architect_review_02/02_LOCAL_AGENT_INSTRUCTIONS.md),
[feedforward contract](../architect_review_02/04_FEEDFORWARD_AND_VALIDATION.md) and
[failure recovery](../architect_review_02/05_FAILURE_MODES_AND_RECOVERY.md).
This extends [review 01's identification amendment](08_IDENTIFICATION_REPAIR.md).
Where priorities or failure interpretations differ, review 02 takes precedence.
Both axes, configuration reuse, and physical Stage 3a and production Stage 3b remain mandatory.

The immediate order is evidence reproduction → separated synthetic gates → estimator
diagnosis/repair → independent synthetic verification → fair physical model comparison.
Current-interface evidence is investigated in parallel. A failed predicate rejects
promotion and routes to diagnosis; it does not terminate safe engineering work.
No new physical motion, settings change or deployment is authorized by these reviews.

## Established evidence and bounded conclusions

The evidence audit reproduces 11 observation runs, 88 physical predictions and 470
numeric comparisons. Eight audit tests pass. The local audit copy omits digest generation
to honor the owner's no-hash instruction; the numeric checks are unchanged. This
verifies exported evidence consistency, not native implementation or the physical plant.

The old physical predictions use complete trajectories and native masks. Do not apply
the earlier disconnected-window criticism to them. All fail the physical prediction
gate; three optimizers report convergence, five exhaust their budgets. Those facts reject
the parameterizations, not every possible parameter vector in the admitted families.
The affine comparison additionally freed current observation delay/filter parameters;
it was not a load-only ablation. New crossed plans must keep all other freedoms equal.

The original noisy seeds 17, 41 and 83 all met the 5% parameter gate. All exhausted
120 solver evaluations; seed 41 additionally failed two cross-run position gates.
The pristine labels repeat one deterministic trajectory. Preserve these as distinct
termination, parameter and prediction facts. The synthetic angle gate remains
0.0008023127185209158 rad for the full declared noise fixture; the physical gate is
0.15 degrees. Neither replaces the other.

The first implemented repair replaces relative forward residual differences with
bounded central differences at explicit absolute physical steps. The original six
cases then pass, with the same model, noise, initial states, loss, bounds and budget.
Independent excitation/noise verification must still pass before expanding that
estimator's declared scope. A newly discovered failing case becomes development
evidence; retain it and predeclare a fresh final partition after any resulting repair.
Unknown delay/filter/static/Stribeck coordinates require their own synthetic fixtures.

The subsequent moving-integral initialization repair resolves the retained reversal
failure. Fresh seeds 1019/1237/1423 and three fresh waveform parameterizations pass
all nine fits and 81 cross-run prediction gates. The verified scope remains the
four mechanics coordinates with known nuisances; these results do not expand it
to physical yaw or all admitted structures.

## Motor and measurement contract

The owner explicitly confirms GM6020 CAN supports current control only. PWM mode is
irrelevant to this operation and is excluded from the investigation. Preserve the
remaining [interface contract](../architect_review_02/03_GM6020_INTERFACE.md): exact
command/feedback schema, applied settings, native freshness, clock uncertainty,
configuration support and adequate stopping behavior.

Archived commands use 0x1FE and feedback uses standard 0x205/DLC8 near 1 kHz.
Software scales command counts by 3/16384 A-equivalent and uses the same scale for
reported current; the recorded reporting correspondence is close to unity. This
supports a bounded lumped input model. It does not independently calibrate physical
Iq/torque or equate a 1–2 ms reporting alignment with actuator latency. The defaults
screenshot is not motor-specific firmware/settings readback for historical runs.
Completed zero requests do not certify stationary hold or an independent cutoff.

## Executable recovery and diagnostics

`commissioning/recovery.py` records independent PASS/FAIL/NOT_RUN/UNKNOWN checks and
typed recovery routes. It never retries motion or rearms hardware. Model comparison
adds constant-position/velocity baselines, displacement, start/reversal/stop and
event-anchored horizons from the same uninterrupted prediction. Outcome-triggered
anchors and motion persistence are retrospective diagnostics, not prospective gates.
Run/channel/regime contributions use the actual normalized Huber objective; the
breakdown alone does not establish that weighting caused failure.

Static thresholds invisible to small local differences must be profiled over meaningful
brackets with start/no-start evidence. Keep load/friction gauges explicit. Do not free
one load offset beside independent directional friction and claim a unique physical
decomposition. Whole-run gates and retained holdout roles remain unchanged.

The shared native core's timeout/censored-start and reset-token faults have executable
regressions. Startup failure now latches HardAbort before issuing another active command;
successful explicit reset is required. Command tokens survive reset, rejecting old ACKs.
These offline software fixes do not establish an adequate physical stopping mechanism.

The [execution report](../reports/ARCHITECT_REVIEW_02_PROGRESS.md) records actual results,
remaining scope and evidence locations. Stage 1 remains IN_PROGRESS; yaw is UNQUALIFIED,
Candidate14 is NONDEPLOYABLE, and physical 3a/3b are NOT_RUN under this revision.

## Continued offline scope

The next execution cycle adds an independent hybrid oracle for first-order/Coulomb
and algebraic/Stribeck fixtures. Exact current-filter cascade propagation repairs
the original current-channel numerical error. A subsequent causality probe found
that delayed observation anchors also repartitioned the mechanical integration
mesh. Sensor queries now use retained history independently of propagation;
changing a reporting delay cannot change the plant or another sensor channel.
The original generator arrays and acceptance limits remain unchanged.
Unknown timing/filter and Stribeck coordinates are verified separately, with
information-limited groups reported independently from prediction success.

[Threshold identification](../../../commissioning/threshold_identification.py)
combines TRAIN-only censored start/no-start intervals with bounded outer threshold
search and inner moving-parameter estimation. Its initial scope is algebraic
Coulomb with known nuisance parameters and a fixed total-load gauge. Thresholds
inside an observationally equivalent bracket remain intervals; interior commands
return an ambiguous outcome rather than a uniquely identified start prediction.

[Configuration assessment](../../../commissioning/applicability.py) binds explicit
facts, confidence/source and model revisions into whole-run fitting and the native
FF adapter. Known conflicts prevent pooling. Changed facts beneath an unchanged
label invalidate parameters and certificates. Legacy missing context remains an
unqualified diagnostic assumption. Reuse requires snapshot/context-specific
response evidence; a global PASS cannot authorize rollback or rearm hardware.

[Assembly dynamics](../../../commissioning/assembly_dynamics.py) supplies runnable
two-axis rigid-body parent mathematics: geometry-dependent coupled inertia,
Coriolis and gravity, explicit causal cable/friction loads and a supplied actuator
map. Its independent mechanics probes qualify that bounded mathematical
implementation. Physical parent identification, native controller coupling and
production equivalence remain NOT_RUN.

## Additional verified scopes and control interfaces

The [Stribeck verification tool](../../../tools/adr0022_stribeck_verification.py)
now verifies joint moving mechanics and two Stribeck speed scales with known
static/input/sensor nuisances. Its initializer uses TRAIN-only integrated force
balance; final acceptance uses the actual hybrid model. A documented checkpoint
continuation follows review02 section3.4 without changing quality limits.

The [load/structure verification tool](../../../tools/adr0022_load_structure_verification.py)
verifies local constant/affine load estimation and blocked whole-run selection
with a documented directional-load gauge and explicit interpolation support.
Known nuisance assumptions remain part of its scope. Joint unknown mechanics
and timing/filter recovery passes a pristine ten-coordinate linear-loss ablation
from the original nontruth start after numerical derivative repairs. Its earlier
Huber and staged failures remain retained. Noisy verification uses an explicitly
empirical squared interval-residual objective, not a calibrated quantized-Gaussian
likelihood or an unbiasedness claim; fresh verification is a separate predicate.

[Selected-family linearization](../../../commissioning/family_analysis.py)
retains friction differential damping, actuator/sensor states and exact delay
factors. Native local probes pass, including negative incremental damping.
The [sampled family analysis](../../../commissioning/family_sampled_analysis.py)
adds exact held-input propagation and the native observe/output/successful-ACK
ordering. Its local sliding lift requires smooth sensors and inactive limits;
native finite-prefix agreement is distinct from a hidden-state Jacobian proof.
Gyro availability age and physical reporting delay jointly determine its source
age, used consistently by the observer and native timestamps. Pre-acquisition
source samples are excluded. The additive
[family synthesis](../../../commissioning/family_synthesis.py) evaluates a
predeclared finite bandwidth curve using signed differential damping and the
complete sampled family lift. Its initial zero-transport algebraic Stribeck
slice returns local candidates; full nonlinear and physical gates remain separate.
[Family forecasts](../../../commissioning/family_forecast.py) generate future
successful current through the existing native core and simulated native or
independent plant observations. They preserve gyro source times and one
acquisition initial state; faults end the simulation without rearming. Passing
this synthetic interface does not qualify its supplied controller baseline,
physical Stage3a/3b or production adoption. Motor FF now shares the updated native
posterior rather than a pre-step state. An explicit bounded steady-state reference
policy supports a dynamic actuator structurally, with no delay inversion.
Low-speed posterior policy remains unresolved motion, not proof of physical rest.
Explicit synthetic DEPARTURE metadata retains the same trajectory q/v/a and
original source-relative start anchor. BRAKING steers the existing STOP owner
and suppresses START re-entry; HOLD keeps the existing position correction.
A finite supported compensation grid and full original quality checks retain
their failures. Neither early onset nor horizon completion selects a qualified
START policy.

The original shaped velocity reference and sensor-based evaluator are reused for
synthetic plateau cases. Latent position span is reported separately from the
fixed two-second encoder drift anchored at its first sample. A static threshold
is likewise distinct from a supported sustained-motion START command: the
inclusive static boundary can hold the axis. Failed start/stop or tracking gates
retain their faults and route to bounded policy qualification; they do not permit
larger physical limits, timeout relaxation or an unverified hold deadband.
