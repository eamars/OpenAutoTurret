# ADR-002.2 local mathematical software

This package implements Stage 1 under the [architect override](../docs/ADR-002.2/docs/07_STAGE1_OFFLINE_OVERRIDE.md).
Its commands read files and run mathematics on the workstation. They do not connect to a
station, open CAN, deploy, or qualify hardware. Actual station parameters remain unknown.

The C++ [axis_control_core](../axis_control_core/axis_control_core.hpp) owns the causal
observer, position P / velocity PI, feedforward, motion states, successful-output
anti-windup, final limits and parameter switching. Python invokes that compiled code
through its versioned C ABI. `commissiond` and the real `controld` link the same object
implementation; their explicit `--axis-core-replay` path terminates before device or
configuration initialization. This replay is mathematical parity evidence. The normal
production output path and its physical adapters still require Stage 2/3 integration
and qualification; the replay is not a 3b certificate.

## Local build and reproduction

Run these commands from the repository root in Linux or **local WSL**, using a project
virtual environment. The full firmware build needs the existing project build dependencies.
The smaller core can also be built with `cmake -S Firmware/axis_control_core`.

```bash
python3 -m venv run/adr0022-local/.venv
run/adr0022-local/.venv/bin/python -m pip install -r Firmware/docs/ADR-002.2/requirements-offline.txt
cmake -S Firmware -B run/adr0022-local/firmware -DCMAKE_BUILD_TYPE=Release
cmake --build run/adr0022-local/firmware -j 4
export OTA_AXIS_CORE_LIBRARY="$PWD/run/adr0022-local/firmware/axis_control_core/libaxis_control_core.so"
export OTA_STAGE1_BUILD="$PWD/run/adr0022-local/firmware"
export OPENBLAS_NUM_THREADS=1
run/adr0022-local/.venv/bin/python -m unittest discover -s Firmware/commissioning/tests -v
ctest --test-dir run/adr0022-local/firmware -E retained_homing --output-on-failure
run/adr0022-local/.venv/bin/python -m Firmware.commissioning.qualification \
  --output run/adr0022-local/qualification --jobs 6
```

After preserving the complete test/build logs, the local evidence auditor
`Firmware/tools/adr0022_stage1_check.py` verifies source/build identities, every immutable
dataset/snapshot/candidate, all 18 conditions, every case report, test results and the
separate Stage 1 gate. It rejects incomplete evidence and cannot mark the full ADR done.

The condition sweep performs 128 whole-run bootstrap refits per axis/condition and
checks the nominal model plus all 128 correlated parameter vectors. It is substantially
longer than the regression suite. `--reuse-fits` accepts only matching numerical fitter,
native core, identities, data and independent holdout evidence. A retained method manifest
preserves the original fit identity when unrelated planning or scoring software changes;
controllers are recalculated under the current complete method. Output directories are locked against concurrent
matrix writers; each numerical worker owns a different axis/condition directory.

The same CLI also supports individual `synthetic-data`, `fit` and `solve` operations:

```bash
run/adr0022-local/.venv/bin/python -m Firmware.commissioning.calibrate synthetic-data \
  --axis pitch --condition PAYLOAD_UP --output run/adr0022-local/input.json
run/adr0022-local/.venv/bin/python -m Firmware.commissioning.calibrate fit \
  --dataset run/adr0022-local/input.json --output run/adr0022-local/assets
```

`fit` prints the immutable snapshot path. Supply that path with `solve --snapshot`,
the same `--dataset`, and `--output`. No source edit or recompilation changes numerical
parameters. A rejected calculation exits with code 2 and a prescribed reason.

## Data and parameter contract

[parameter_catalog.py](parameter_catalog.py) is the executable catalog: meanings, units,
frames, dimensions, dependencies, classification, estimator, information requirements,
constraints, uncertainty, reuse and unknown handling. `catalog(5)` describes finite
domains and `catalog(8)` the periodic yaw domain. `pending_document()` gives null,
explicitly pending values. The exported [catalog](../docs/ADR-002.2/contracts/parameter_catalog.json)
and [pending template](../docs/ADR-002.2/contracts/parameters.pending.json) contain no station measurements.

`ModelSpec` fixes the model structure and coordinates. `PlantSnapshot` contains the
joint a/b/h/delay vector, 128 uncertainty vectors, directed start intervals, data
identities and frequency domain. `ObserverSpec` binds calibrated noise and timing.
`PlantSnapshot.bind` validates version, units, dimensions, coordinates and H/C/O identity;
physical binding rejects synthetic provenance. Hardware qualifications remain separate.

`load_dataset` accepts either complete normalized `train`/`holdout` runs or `raw_train` /
`raw_holdout` plus a content-addressed calibration. Raw records pass through
[normalization.py](normalization.py): clock maps, shaft-unit conversion, mounting rotation,
actual new-sample masks, successful TX, current supervision and source validity checks.
The measurement module computes mounting/bias, clock relationships, lever arm,
noise, observation filtering, bandwidth and observer process variance from supplied
calibration records. Verified interface constants and physical boundaries are inputs;
missing values are never filled from the synthetic fixtures.

Artifacts are JSON named by their canonical content hash. Names alone grant no identity.
Raw records, snapshots and candidates stay immutable. Large local captures and binaries
belong under ignored `run/`; the compact validation report belongs with the ADR.

## Model boundaries and failure handling

| Condition | Mathematical treatment |
|---|---|
| Payload, centre of mass, friction, direction, other-axis posture | Current-equivalent a/b and directional h tables, three posture layers, identified operating-point identity |
| Rest, breakaway, motion, stopping and reversal | Directed total-start intervals and REST/START/MOVE/STOP/REVERSE transitions; no double addition of static load |
| Requested versus executed input | Successful TX drives identification; shared final current/slew limits and acknowledged-output anti-windup |
| Sensor mounting, bias, quantization, noise, filtering and asynchronous samples | Explicit calibration, causal observer, timestamp/generation checks and sampled stability model |
| Sensor or clock failure | Reject invalid data; encoder-only control requires its own verified fallback flag |
| Temperature, supply, mechanical range, stop capability | Injected external constraints and applicability checks; unknown temperature cannot establish thermal qualification |
| Unsupported resonance, coupling or changed physics | Independent prediction/residual checks return MODEL_INADEQUATE; no automatic model expansion |
| Uncovered parameter/domain or insufficient excitation | Reject extrapolation; prescribed information selector and finite retry/supplement budgets |

The fitter uses constrained integral initialization, normalized SVD checks, bounded
Huber output-error fitting and independent whole-run validation. Trust-region scaling
uses the unmodified observation Jacobian: the robust Jacobian must not create enormous
scales when the initializer is outside the Huber transition. Fractional actuator delay,
measurement age, causal sensor filtering and sampling cadence enter the stability
calculation. Model-consistent fixtures come from analytic inverse dynamics, independently
of the C++ forward integrator; adverse fixtures include data outside the model family.

The solver calculates 256 analytic bandwidth points and chooses the highest feasible
point after discrete poles, all crossing margins and the frozen response/quality gates.
All calculations are offline. A candidate remains `OFFLINE_CANDIDATE_ONLY`.

## Stage boundary

Stage 1 completion establishes mathematical software readiness. Stage 2 must verify
actual interfaces, stopping/protection, units, clocks, installation, sample rates and
approved boundaries before collecting the dependent measurements. It then supplies
real data to these existing algorithms. Independent-program 3a and normal-production
3b physical validation both remain mandatory for the full ADR. This local session stops
before Stage 2 and makes no statement about the station's present running state.
