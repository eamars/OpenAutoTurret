# ADR-002.2 whole-run yaw model comparison

## What this is for

Apply the [architect amendment](../ADR-002.2/docs/08_IDENTIFICATION_REPAIR.md)
to existing captures before any further controller calculation or physical trial.
Compare predeclared actuator/friction structures on complete physical journals.
The later [review 02 recovery amendment](../ADR-002.2/docs/09_ESTIMATOR_RECOVERY.md)
prioritizes synthetic estimator recovery and fair load/measurement ablations.

## Where the work happens

This operation runs entirely on the workstation's Linux environment. Use the
existing project-local venv and build the native mathematical library there.
The operation does not connect to the station. Captures and numerical output stay
under ignored `run/`; the Pi does not compile this software.

## The command

```bash
cmake -S Firmware/axis_control_core -B run/adr0022-local/core
cmake --build run/adr0022-local/core --target axis_control_core_native --parallel 2
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_yaw_compare.py \
  --config run/adr0022-stage2/architect-review-response-01/comparison-config.json \
  --library run/adr0022-local/core/libaxis_control_core.so \
  --output run/adr0022-stage2/architect-review-response-01/comparison-new \
  --max-nfev 200
```

The JSON configuration declares frozen calibration, shaft datum, training noise,
TRAIN/SELECTION/HOLDOUT journal blocks, timing search bounds, numerical model seeds
and parameter bounds. Keep numerical rollout extent separate from measured applicability:
observed encoder extrema are not yaw travel limits and can obstruct optimization.
For zero-spatial-load dissipative algebraic models with unit actuator gain and zero bias,
a conservative training-only
extent follows `|q| <= |q0|max + |v0|max*T + max|u_TX|*T²/(2*a_min)`.
That bound has explicit model assumptions and grants no physical certification.
A journal may occur in only one block. Training uses current
excitation journals while feedback-generated estimator validity remains unresolved.
The command saves its input inventory and numerical plan before fitting. It retains
all motion and stopping regimes, native freshness masks and successful TX prehistory.
Only the leading baseline initializes the single continuous state trajectory.
Use a fresh output directory for every execution. The command rejects a nonempty
directory so retained plans, failed fits and complete predictions cannot be overwritten.

## What it proves

The report records bounded numerical fits, whole-run training and selection gates and,
if a structure passes selection, one final historical holdout evaluation. It can
reject a model/data combination even when its optimizer converges. Sensor/current
semantics, prior-only coordinates, and historical calibration exposure remain explicit.

## What it does not prove

This is input-driven retrospective prediction. It is neither a prospective closed-loop
forecast nor a physical 3a/3b certificate. Previously inspected captures are not unseen
experiments. All result packages remain nondeployable; no gains are generated.

## When it fails

Retain the plan, failed fits, residuals and raw journals. A MODEL_DATA_FAILURE blocks
controller synthesis and returns a typed diagnosis/recovery route; safe offline
analysis continues. Resolve timing/registration/measurement validity or obtain
program-selected discriminating evidence inside verified physical bounds. Do not rerun
gain calculation on the last failed fit, relabel training windows as holdout, enlarge
uncertainty to hide error, or change the frozen acceptance targets.

For baseline, event and objective diagnostics on retained predictions without fitting,
use `adr0022_yaw_compare.py --diagnose-retained FROZEN_COMPARISON_DIRECTORY --output NEW_DIRECTORY`.
Repeat `--diagnose-retained` to inspect additional comparisons; final holdout is not read.
For the independent known-nuisance estimator probe, use
`adr0022_closed_loop_estimator_probe.py` and `adr0022_estimator_verification.py` with the
same workstation native library and fresh output directories. Declare verification
partitions before generation; a failing final case used for repair becomes development evidence.

Broader offline numerical/method probes are available as
`adr0022_family_oracle_verify.py`, `adr0022_nuisance_verification.py`,
`adr0022_threshold_verification.py`, `adr0022_configuration_probe.py` and
`adr0022_assembly_probe.py`. Each uses explicit synthetic inputs and fresh outputs;
passing one bounded scope does not qualify another family or the physical assembly.
`adr0022_fair_diagnostic.py` performs the clean constant/affine and fixed/free
current-measurement comparison on TRAIN/SELECTION only, retaining continuous
predictions. Its diagnostic status never promotes a physical model or consumes
historical final-holdout observations.

Additional workstation-only method/control probes are
`adr0022_load_structure_verification.py`, `adr0022_stribeck_verification.py`,
`adr0022_family_linearization_probe.py` and `adr0022_family_forecast_probe.py`.
The forecast core generates its own future commands from simulated sensors;
`--control-plant independent` uses the independent hybrid oracle throughout.
`--motor-ff-policy STEADY_STATE_REFERENCE` uses one bounded model FF term evaluated
from the updated native posterior. `--reference velocity-plateau --speed-deg-s 5`
reuses the existing shaper and frozen motion metrics. These are synthetic planning
values, with no new physical current, speed or stopping authority.
`--start-policy PLANNED_DEPARTURE_DIAGNOSTIC` adds coherent planned phases and
the frozen bounded development START candidate. Its original full quality gates
failed; this option does not supply a qualified controller policy.
`adr0022_family_sampled_probe.py` checks the selected-family local sampled lift;
smooth sliding and inactive limits are prerequisites, and complete nonlinear
forecasts remain required.
`adr0022_family_synthesis_probe.py` exercises the additive bounded local synthesis
route; it retains all candidates and unsupported/no-feasible outcomes.
These commands check bounded synthetic interfaces and methods, with physical
model/controller qualification retained as separate unperformed gates.
