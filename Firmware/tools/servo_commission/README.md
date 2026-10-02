# servo_commission: automatic servo commissioning

Operator procedure: [`docs/operations/servo-commissioning.md`](../../docs/operations/servo-commissioning.md).
This file is the developer's map: what each part is, what the models assume, and which numbers
were calibrated how.

## Parts

| File | Role |
|---|---|
| `commission.py` | The pipeline and CLI (`yaw`, `pitch`, `check`, `validate`, `session`). Every decision is a rule on measured numbers; `report.json` records them. |
| `station.py` | `Station`: cross-build, one deploy per run, then upload manifest, run and fetch for each session. `SimStation`: the same interface answered by `servo-sim` against a hidden true plant. |
| `manifests.py` | Session scripts (survey, crosstalk scan, inertia sweep, ladder, circle, use case, pitch scripts) and the commissiond manifests built from them. |
| `identify.py` | Friction curve and maps, crosstalk table, inertia and loop delay (yaw), and speed-loop delay and lag (pitch). |
| `design.py` | The gain law, the simulated stability boundary, the bias choice, and the pitch phase-margin rule. |
| `sim.py` | Wrapper around the native `servo-sim` (`axis_control_core/simulate.cpp`). |
| `score.py`, `usecase.py` | ADR-003-shaped references (Level-1 generator), per-segment metrics, and the fixed acceptance limits. |
| `journal.py` | Reads commissiond journals into arrays. |
| `sim_cases/` | Hidden plants for the offline proof: this station's estimate, and a 2.5× payload. |
| `tests/test_pipeline.py` | The offline proof: identification accuracy, delay calibration, and whole-pipeline acceptance. |
| `templates/` | Session manifest skeletons (station topology, IMU calibration, limits). |

The C++ that commissiond and the simulator share lives in `axis_control_core/`:
- `servo.*`: the yaw position servo.
- `position_loop.*`: the pitch host loop.
- `session_parts.hpp`: the reference table, the excitation sweep and the oscillation guard.
- `servo_config.*`: the asset loader.
- `plant.*`: the plant models.
- `simulate.*` and `servo_sim.cpp`: the simulator.

## The yaw plant model (`plant.hpp`)

The plant is a rigid inertia `a` (A·s²/rad) driven by the current, which follows the command after
`actuation_delay_s` through a 0.5 ms lag.

Friction is LuGre:
- the contact state z obeys dz/dt = v − σ0·|v|·z/g(v);
- the friction is F = σ0·z + σ1·dz/dt + b·v;
- the steady level is g(v) = Fc + Fs·e^(−|v|/vs) − Fr·e^(−|v|/vr) + map(angle, direction).

So friction:
- rises out of a low creep level (Fr: the bearing creeps under 0.15–0.5 A);
- peaks around 2 deg/s;
- falls toward Fc at speed (velocity weakening).

σ0 = 2000 A/rad and σ1 = 5 A·s/rad are structural constants, not identified. The 80 Hz probe at
rest shows no measurable pre-sliding compliance, so σ0 must be well above 1000.

The encoder reads round(q + g(angle)·u(t_receipt − crosstalk_delay)) in 8192 counts, where g is
the measured crosstalk table.

What the model leaves out, and why that is acceptable:
- **Breakaway that grows with rest time and load.** The servo's stall rock handles it on the
  station.
- **Friction drift with temperature.** Re-run `check`.
- **The localized bump near 240–255 deg.** Partly captured by the friction maps.

The model is used to choose the bias and the candidate gain, and to bracket the station ladder. The
station decides the final gain and acceptance.

## The pitch plant model

The pitch speed follows the speed reference after a delay, through a first-order lag; the position
is quantized to 8π/65535 rad, with one reply per command. The drive's own speed PI absorbs gravity
and friction.

## Calibrations made in simulation (re-check them if the scripts change)

- **Loop delay:** the inertia sweep's fitted delay is the actuation delay plus 1.2 ms
  (`commission.DELAY_OFFSET_S`; `tests/test_pipeline.py::test_delay_calibration`).
- **Crosstalk table:** a 10 deg/s scan with 0.25 s windows reproduces a known table within
  0.6 mrad/A RMS. A 20 deg/s scan under-reads it by about 14%, which caused instability.
- **Prior gains (kq 20 / kv 0.5 / ki 20):** quiet through the survey for inertia 0.015–0.15 and
  uncorrected crosstalk up to 1.5× the measured size.

## Rules decided on the station (2026-10-02)

- **Oscillation guard at 0.3 A of fast current RMS:** normal runs peak at 0.14 A, the limit cycle
  reaches 0.3 A within 0.2 s of onset.
- **Integral rate ki = 1.5·kq, not scaled with wn:** the integral works against friction, which
  does not scale with inertia.
- **Stall recovery always on in a designed asset:** without it, ±90 deg moves stuck 0.33 deg short
  at 1.1 A. It acts when the axis is more than 0.14 deg behind while pushing, for 0.3 s, and only
  with |v_ref| < 0.01 rad/s, so it fires at targets and not at the walking profile's reversals.
- **Oscillation guard:** 50 ms above 0.3 A of fast RMS, with rocks excluded. The ladder and circle
  count a speed trip as an onset too.
- **Scoring against the measured (unbiased) crosstalk table:** the deliberate bias is real
  tracking error.
