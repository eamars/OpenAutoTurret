# Architect override: offline mathematics and measured parameter injection

Authority: the owner's architect instruction in the 2026-09-30 implementation session.
This decision overrides conflicting Stage 1/2 prerequisites, inputs, outputs and
completion rules throughout this package. Stage 3a/3b and the full-delivery rule remain.

## Stage 1: mathematical software, entirely offline

Deliver a complete executable parameterized model, measurement model, identification
pipeline and controller solver. Synthetic data must exercise the complete pipeline.
Hardware access, register readback, physical calibration and station measurements are
neither prerequisites nor evidence required for this stage. Unknown station inertia,
friction, delay and IMU mounting cannot block mathematical software completion.

The bounded model family remains current-equivalent a/b, directional continuous load
tables, three other-axis posture layers, directed breakaway intervals, end-to-end
delay, hybrid motion states and joint parameter uncertainty. Cover payload, centre of
mass, friction, posture, temperature/supply applicability, actuator limits and slew,
successful TX versus requested input, sampling, quantization, bias, timing, coordinates
and invalid measurements. Each case belongs to a model term, state transition, external
constraint or model-inadequacy detector. Do not invent an unlimited model family.

Every parameter contract defines meaning, SI units, coordinate frame, dimensions,
model dependencies, constant/estimated/derived/constraint classification, raw signals,
estimator, identifiability/data sufficiency, numerical/physical bounds, uncertainty,
applicability/reuse and unknown/invalid/stale handling. Unidentified values remain
explicitly pending; synthetic values never substitute for physical measurements.
Only identifiable combinations are required; do not report unsupported torque or inertia
in mechanical units from current-equivalent coefficients.

Implement before measuring: normalization → data/information checks → identification
→ joint uncertainty/applicability → model binding → feedforward/feedback calculation
→ offline constraints → candidate or the prescribed failure reason. Also implement
acquisition protocol, signal generation, deterministic information selection, bounded
retries and change handling using a simulated interface. Physical stimulus bounds are
injected after Stage 2 verification. No agent chooses fits, stimuli or PID values.

Completion requires unchanged software to identify and bind multiple synthetic systems
and calculate controllers within declared tolerances, and to reject invalid, insufficient,
unidentifiable, inapplicable and mismatched data. Test nonideal independently generated
data as well as model-consistent data. All mathematical branches must be executable,
with no placeholder awaiting measurement. Synthetic success qualifies mathematical
software only, never station performance or Stage 3.

## Stage 2: physical facts and automatic injection

First verify current hardware capabilities, actual mode/readback, stopping/protection,
encoder/current units, IMU mounting and clock calibration, actual rates/delays and
approved experimental envelope. Missing facts block only dependent physical steps.

Use compatible retained assets → verify hardware/measurement/operating point → collect
raw feedback → calibrate/align/normalize/check quality → call the Stage 1 estimator →
create an immutable PlantSnapshot → programmatically bind its ModelSpec/ObserverSpec →
call the existing solver → emit an identified candidate and offline evidence.
Binding checks units, frames, dimensions, version, completeness, provenance and domain.
Raw feedback is not a parameter snapshot; filenames are not identity.

Changed payload/friction/etc. updates data and snapshots with the existing method,
without source changes, compilation or deployment. Preserve raw evidence and previous
snapshots. Insufficient information uses the fixed selector; unexplained response is
MODEL_INADEQUATE. Changing model structure requires a new design decision, never an
unreported parameter update or relaxed threshold.

Outputs include raw/calibration/capability assets, identified uncertainty and domain,
complete model binding and computed controller candidate with source identities.

## Unchanged physical qualification

3a requires the independent program on actual hardware; 3b requires the actual production
executable, normal reference/output/stop chain and normal service load. Shared C++ core,
runtime parameters, single output owner, protections, double validation and applicability
aware rollback remain mandatory. Stage 1 simulated ownership/readback testing does not
qualify station ownership, mode transitions or stopping.

Record independently: mathematical software status, hardware capability status, physical
calibration status, plant identification status and 3a/3b qualification. The full ADR is
DONE only after the complete physical matrix. The original Stage 1 session ended at
the offline boundary. Subsequent Stage 2 authorization and the stricter requirement
after the failed inventory are recorded in [current readiness](../reports/STAGE2_READINESS.md).
That authorization does not permit using the station to verify software corrections.
