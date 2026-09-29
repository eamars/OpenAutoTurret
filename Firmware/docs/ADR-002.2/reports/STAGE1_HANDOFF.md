# Stage 1 local validation and Stage 2 handoff

Stage 1 mathematical software: **PASS**. Stopped at the Stage 2 entry.
Stage 2, hardware capability, physical calibration, actual plant identification,
3a and 3b: **NOT_RUN**. Full ADR: **NOT_DONE**.

The architect override governs this result. No station connection, SSH, CAN, physical
measurement, deployment, service action or motor command was performed. The current
physical station state is unknown because this task did not access it.

The engineering confidence assessment is **95% for the defined Stage 1 mathematical
software scope**. This is an evidence-based engineering judgment, not a statistical
probability certificate or a statement about hardware safety/performance. The assessed
boundary is the finite declared model, data assumptions and tested branches below.

## Reproducible evidence

- 91 mathematical/core/protocol regression tests passed.
- 84 ADR identity, schema and dual-validation contract tests passed.
- 82 existing local CTests passed; `retained_homing` was excluded as directed by the repository.
- 18/18 independent axis/condition pipelines passed. Each identified 128 whole-run bootstrap vectors and calculated a controller without changing numerical source or recompiling for parameter values.
- 417,960 C++ closed-loop case evaluations passed across nominal and uncertainty models. Causal feedback included declared encoder/gyro noise and quantization.
- Local x86_64 full build passed. Both executables and the shared library produce identical synthetic replay output.
- Local ARM64 core and independent replay program compile/link passed. No ARM64 execution or station-image ABI qualification is claimed; a full production ARM64 release was not deployed or tested.

The [machine-readable report](STAGE1_LOCAL_VALIDATION.json) includes source/dependency,
binary, log, snapshot, candidate and data identities, every requirement mapping and
all unmet full-ADR requirements. The local auditor checks the complete case reports,
immutable content hashes, source dependencies and stage boundary before producing PASS.

The starting source baseline was `b4701de6a021176bcb334f5015415c680eca3bda`;
the workspace HEAD at validation was `95c5e5c1c79d0d30d1e9f97365c7f7c4ee710d81`. The report binds
the actual file hashes, including any local changes after that commit. Raw synthetic
records, candidates, binaries and logs are retained under
`run/adr0022-local/` and excluded from version control. The original
[reference-only report](OFFLINE_VALIDATION.json) remains historical.

## Parameter recovery and controller checks

Frozen recovery limits: a relative error <3%, b <8%, h absolute error <0.005 A,
delay error <2 ms. Required stability margins: phase >=50 degrees, gain >=6 dB.

| Axis | Condition | a error | b error | h error (A) | Delay error (ms) | Phase (deg) / gain (dB) |
|---|---|---:|---:|---:|---:|---:|
| yaw | BASELINE | 0.0077% | 0.0074% | 0.000013 | 0.0622 | 53.60 / 17.12 |
| yaw | PAYLOAD_UP | 0.0123% | 0.0209% | 0.000026 | 0.0885 | 52.56 / 16.55 |
| yaw | PAYLOAD_DOWN | 0.0041% | 0.0017% | 0.000007 | 0.0445 | 53.73 / 16.90 |
| yaw | FRICTION_UP | 0.0039% | 0.0038% | 0.000014 | 0.0496 | 58.43 / 20.52 |
| yaw | FRICTION_DOWN | 0.0105% | 0.0131% | 0.000011 | 0.0726 | 51.87 / 16.05 |
| yaw | RETURN_BASELINE | 0.0077% | 0.0074% | 0.000013 | 0.0622 | 53.60 / 17.12 |
| yaw | UNDECLARED_CHANGE | 0.0039% | 0.0038% | 0.000014 | 0.0496 | 58.43 / 20.52 |
| yaw | CENTRE_OF_MASS | 0.0031% | 0.0158% | 0.000004 | 0.0307 | 54.23 / 17.44 |
| yaw | TEMPERATURE_SUPPLY | 0.0066% | 0.0060% | 0.000013 | 0.0582 | 54.06 / 17.33 |
| pitch | BASELINE | 0.0071% | 0.0286% | 0.000046 | 0.0260 | 53.74 / 17.34 |
| pitch | PAYLOAD_UP | 0.0123% | 0.0365% | 0.000081 | 0.0428 | 53.56 / 17.43 |
| pitch | PAYLOAD_DOWN | 0.0037% | 0.0281% | 0.000033 | 0.0251 | 54.22 / 17.42 |
| pitch | FRICTION_UP | 0.0067% | 0.0139% | 0.000041 | 0.0228 | 57.65 / 20.16 |
| pitch | FRICTION_DOWN | 0.0062% | 0.0507% | 0.000049 | 0.0294 | 52.55 / 16.60 |
| pitch | RETURN_BASELINE | 0.0071% | 0.0286% | 0.000046 | 0.0260 | 53.74 / 17.34 |
| pitch | UNDECLARED_CHANGE | 0.0067% | 0.0139% | 0.000041 | 0.0228 | 57.65 / 20.16 |
| pitch | CENTRE_OF_MASS | 0.0022% | 0.0066% | 0.000021 | 0.0486 | 52.86 / 16.73 |
| pitch | TEMPERATURE_SUPPLY | 0.0072% | 0.0230% | 0.000044 | 0.0249 | 53.94 / 17.38 |

Whole-run train/holdout splits use independent noise realizations. The excitation
oracle uses analytic inverse dynamics rather than the C++ forward integrator.
Additional tests exercise eight-node periodic yaw, changing posture, pitch support,
reversal, raw asynchronous normalization, nonidentity IMU mounting and body-frame
gravity, observer fitting, causal sensor filtering, quantization and fractional delay.

Negative tests cover missing information, unidentifiable parameters, renamed training
data reused as holdout, unsupported resonance, bad units/frames, stale identities,
timestamps/generations, sensor loss, TX failure, censored startup, failed writes,
readback mismatch, ownership/lease loss, process exit, capture overflow, unknown thermal
state and inapplicable rollback. Synthetic certificates cannot satisfy 3a/3b or full ADR.

An early pitch fit falsely stopped under Huber Jacobian scaling. The correction freezes
scales from the unmodified observation Jacobian and has a dedicated recovery regression.
Earlier exploratory failures remain in the ignored local evidence directory. The final
report uses the final solver and acceptance limits; historical candidate results do not
stand in for final validation. Reused fitting assets retain their original immutable
identities and require identical fitting source/core/dependencies, identical data and
fresh independent validation. Controller candidates are computed by the final method.

## Stage 2 entry contract — no work in this stage was started

1. Verify the real hardware identity, current-mode capabilities, readback, single output
   owner, support/stop/protection and approved experimental boundaries before any
   dependent physical test.
2. Obtain actual encoder/current unit evidence and IMU installation, clock, noise,
   causal-filter and sampling calibration. Bind complete calibration identities and
   keep unknown parameters pending; use no synthetic defaults.
3. Supply compatible raw or normalized whole-run records to the existing calibrator.
   It normalizes and checks data, identifies parameters, creates an immutable snapshot,
   binds the model/observer and computes one offline controller candidate. Its fixed
   information selector and failure budgets govern any supplemental measurement.
4. Complete the actual hardware adapters and normal production output integration
   against the shared core. The current explicit executable replay proves mathematical
   parity only and does not provide this physical integration evidence.
5. Perform both independent-program 3a and normal-production 3b validation for each
   applicable condition before granting physical qualification or promotion.

These are mandatory remaining parts of the same ADR, not deferred to a separate version.
The full model/identification/controller methods are implemented; actual physical facts,
hardware adapter qualification and the two physical validations remain separate states.

Implementation and reproduction instructions: [commissioning README](../../../commissioning/README.md).
