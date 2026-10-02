# Evidence ledger — independent ADR-002.2 review

This ledger distinguishes supplied results from independently recomputed diagnostics.
All original paths below are archive members of the uploaded ZIP. Repository-relative
paths inside supplied JSON files can be resolved through `FILE_INDEX.csv`.

## E1 — Raw journal and projection verification

**Inputs:** `EXPERIMENT_INDEX.json`; all 25 `raw_archive_path` journals listed there.
Projection constants: `evidence/yaw-feedback-13-speed-pos5/control-manifest.json`,
key `gyro_calibration`.

**Independent method:** `audit.py`, functions `audit_run` and `main`.
Decode raw yaw CAN bytes as unsigned 16-bit encoder, signed 16-bit RPM and signed
16-bit current; pair to decoded feedback by exact kernel monotonic timestamp.
Reconstruct per-session movement from wrapped encoder increments. Compare the
frozen calibrated projection `(gyro - bias) dot column / (column dot column)`
with Candidate13's observations using the exact gyro sample timestamps.

**Executed outputs:** `reproduced/audit_summary.json`, `reproduced/audit_runs.csv`.
Physical journals: 25; raw/decoded paired records:
286,319; mismatches: 0;
unpaired decoded records: 0.
All indexed event counts match.
Candidate13 compared cycles: 1449;
maximum projection discrepancy: 1.110223024625156e-16 rad/s.

**Limit:** arithmetic verification, not independent physical calibration.

## E2 — Successful TX versus decoded current timing

**Inputs:** the same raw journals; `yaw_current_tx.success`,
`kernel_accepted_ns`, `successful_tx_A`; `yaw_feedback.kernel_monotonic_ns`,
`current_A`.

**Independent method:** `audit.py:telemetry_lag`.
Search 0–150 ms in 1 ms steps; affine regression at each delay. Use a common valid
window beginning 150 ms after the first successful TX, through the last encoder
feedback sample. The fixed-lag columns instead use unity gain and zero intercept.
Commands are zero-order held. No future current observation is used to interpolate
the command. These are descriptive telemetry comparisons, not torque identification.

Best lag frequencies: 1 ms in 24 runs; 2 ms in 1 run.
Candidate13 affine gain at the best lag: 0.999940021.
Candidate13 unity-gain RMS at 1 ms: 0.003108062 A.
Candidate13 unity-gain RMS at 60.593108 ms: 0.053169497 A.
RMS ratio: 17.107.
Candidate12 +5 degrees/s unity-gain RMS: 0.003217410 A at 1 ms
versus 0.066212489 A at 60.593108 ms.

**Limits:** correlated slew waveforms; coarse lag grid; unknown current-feedback
bandwidth/meaning; application time not measured. Neither physical Iq calibration,
physical torque delay nor a deployment delay change is established.

## E3 — Fit support and in-sample reconstruction

**Input:** `evidence/yaw-feedback-local-update-06/numerical-fit.json`.
Keys: `training_run_descriptions`, `raw_prediction_errors[*].updated`,
`equations_by_direction`, `observed_periodic_information`,
`local_q_support_rad`, `actual_posture_support_rad`,
`optimizer_evaluations`, `optimizer_message`.

**Independent computation:** compare training and prediction run-ID sets; sum each
end-minus-start duration; count unique source journals and thresholded durations.
Executed inventory: `reproduced/fit_window_inventory.csv`.

- Training windows: 29.
- Distinct physical source journals: 3.
- Reported updated prediction windows: 29.
- Training and prediction ID sets identical: True.
- Total duration: 4.308642366 s.
- Median duration: 0.059985071 s.
- Minimum/maximum duration: 0.058979915/1.019642485 s.
- At or below 61 ms: 16.
- Native supplied updated errors fail velocity in 28 windows and angle in 8 windows.
- Integral equations: 38 positive; 5 negative.
- Observed reduced rank: 8 of 8; full posture-tied representation has 18 coordinates.
- Observed normalized condition: 22.935721795.
- Local yaw span: 31.289062500 degrees.
- Measured posture span: 0.025514172 degrees.
- Optimizer evaluations: 43; termination:
  ``ftol` termination condition is satisfied.`.

**Limit:** numerical fit errors were supplied by the local agent. This audit checks
their indexing, counts and support, and does not claim to reproduce the unavailable
native output-error implementation.

## E4 — Coefficient instability and structured residuals

**Inputs:** `MODEL_CATALOG.json`, `parameter_sets[*].theta`; the full layout is in
`theta_layout`. The first `theta` entry is the tied local inertia-equivalent coefficient.
Values in archive order: 0.0340194777669, 0.0895425031302, 0.263420215721, 0.224128549479 A*s^2/rad.
Maximum/minimum ratio: 7.743217504.

**Residual source:** `evidence/yaw-feedback-local-update-05/actual13-residual-diagnosis.json`.
Keys: `actual13_direction_summaries`, `burst_nearest_actual_integral_window`,
`all_integral_equations`. The supplied summary is also in `ARCHITECT_BRIEF.txt`.
Positive-direction load residual versus speed correlation: -0.84890437;
versus angle: -0.4452220; speed versus angle: 0.19643425.
Burst-window prior modeled load: approximately 0.562393698 A;
inferred equivalent load: approximately 0.431730637 A.

**Interpretation:** evidence of structured mismatch and competing parameter estimates,
not uniquely established Stribeck friction or changing physical inertia.

## E5 — Physical Candidate12/13 performance

**Inputs:**
- `ARCHITECT_BRIEF.txt`, observations 3–6 and frozen quality targets.
- `evidence/yaw-feedback-12-speed-pos5/actual-velocity-analysis.json`.
- `evidence/yaw-feedback-13-speed-pos5/actual-velocity-analysis.json`,
  keys `actual_reference_plateau`, `frozen_motion_metrics_degrees`,
  `independent_actual_motion`, `final_zero_observation`.
- `evidence/yaw-feedback-13-speed-pos5/actual-motion-transitions.json`.
- `evidence/yaw-feedback-local-synthesis-05/retained-failure-mathematical-handoff.json`,
  `measured_response`.

Candidate13 mean/reference 1.080154447; continuity 0.896; speed RMS 6.244434437
degrees/s; jitter 6.230777793 degrees/s; fixed 2-s zero-reference drift 3.69140625
degrees; later actual-zero-current drift 0.1318359375 degrees. Encoder 200-ms peak
20.3124 degrees/s; gyro bursts exceed 21 degrees/s. These supplied metrics use
their stated native definitions and are not replaced by a new smoothed metric.

**Independent illustration:** `candidate13_velocity.png`, from native calibrated gyro
samples and the recorded reference. Underlying values: `candidate13_raw_motion.csv`.
The plot is not a new physical experiment or a new calculation of all acceptance metrics.

## E6 — Candidate14 and qualification

**Input:** `evidence/yaw-feedback-local-synthesis-05/retained-failure-mathematical-handoff.json`.
Keys: `candidate14_physically_run`, `physical_qualification`,
`deterministic_candidate14`.

The supplied record says Candidate14 was not physically run; 54 of 54 linear
checks and 270 of 270 numerical motion evaluations failed; 12 margins are undefined
after failed poles; worst spectral radius 1.049325283964214. Nine total alternatives
are descriptive conflicting evidence, not a calibrated confidence ensemble.

No pitch commissioning or formal production validation is established by this archive.

## E7 — Coordinate and resolution audit

**Independent calculation:** `audit.py:audit_run`;
`reproduced/audit_runs.csv` fields `relative_vs_common_phase_offset_deg` and
`encoder_unwrap_max_abs_error_rad`. Common encoder datum: 5768 counts, used only
as a diagnostic comparison, not asserted as physical world zero.

Older captures have session-relative origins. Several older feedback sessions differ
by one to four counts from this common registration, including feedback11 at -0.17578125
degrees. The latest three fitted journals show zero phase discrepancy against it.

**Important countercheck:** `evidence/yaw-information-01/normalization-report.json`
already documents normalization of earlier acquisition origins. The finding is a
warning against pooling incompatible fields, not proof the existing normalizer
failed or the latest fit used the wrong zero.

Derived from 8192 counts/revolution: one count = 0.0439453125 degrees; one-count
difference / 5 ms = 8.7890625 degrees/s; / 20 ms = 2.197265625 degrees/s.
One integer RPM = 6 degrees/s.

## E8 — Status of further modelling proposals

The actuator/friction/measurement family, payload applicability rules, experiments
and prospective validation workflow in the review are proposed corrections.
They are not fitted/qualified replacement hardware models. The current audit does
not independently establish torque coefficients, dynamic friction state, sensor
mounting/filter latency, full-pitch coupling, or global-load maps.
