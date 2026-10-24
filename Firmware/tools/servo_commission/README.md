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
| `score.py`, `usecase.py` | ADR-003-shaped references (Level-1 generator), per-segment metrics, the acceptance gates, and conformance to the calibrated accuracy limits. |
| `accuracy.py` | The yaw accuracy calibration (`commission.py calibrate`): per-pass metrics, the fixed limit rule (1.25 x worst pass), the motor-feedforward-off variant, and the comparison. |
| `journal.py` | Reads commissiond journals into arrays. |
| `sim_cases/` | Hidden plants for the offline proof: this station's estimate, a 2.5× payload, and the 5× (the 2026-10-24 payload's class) and 10× payloads, all with the station's measured encoder noise and friction roughness. |
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

The encoder reads round(q + g(angle)·u(t_receipt − crosstalk_delay) + n) in 8192 counts, where g is
the measured crosstalk table and n the encoder's own noise: 0.2 mrad rms, flat to 40 Hz and
falling as 1/f above (`encoder_noise_rad`, `encoder_noise_corner_hz`). It was measured on the
station 2026-10-24 on three inertia sessions and the survey plateaus: the same 0.05–0.1 mrad per
band from 8 to 60 Hz at every speed (5–40 deg/s) and for both inertias, so it is neither a
spatial pattern nor motion. Without it the simulator was 5–10× quieter than the station in the
very band the inertia sweep used, and passed sweeps that failed on the station.

The sliding friction level also wanders (`friction_noise_A` 0.03 A rms, 6 Hz corner; nothing at
rest): a torque roughness calibrated to the un-excited ramp's 2–8 Hz motion on the bare turret
(1.3, 1.3, 0.44 mrad in the 2–4, 4–6, 6–8 Hz bands at 30 deg/s under the prior's gains). The
payload's low-frequency motion is about twice what this level gives; the 5× case keeps the
bare level because a higher one made the prior's loop ring to 2 A where the station showed 0.9 A.

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
  (`commission.DELAY_OFFSET_S`; `tests/test_pipeline.py::test_delay_calibration`). The fit bounds it
  to 4 ms: a heavy plant's coherent bands sit below 10 Hz, where the phase cannot pin it.
- **Inertia sweep (`manifests.inertia_sweep_for`, 2026-10-24):** a rigid body answers the 0.3 A
  sweep with 1/(J w²), so a 5× payload's response at 8–30 Hz (0.2–0.5 mrad/A) sat under the encoder
  noise and the crosstalk table's ~1 mrad/A drift. The sweep now starts at the nearest of 8, 5.6, 4,
  3 Hz to twice the prior loop's resonance sqrt(kq/J) — below that the swing reverses a light
  plant's sliding or trips the speed limit — using the survey's rough inertia
  (`identify.inertia_from_speed_changes`: accelerating minus decelerating current at the same
  speed), and keeps 5.5 s per octave. The bare turret's 8–30 Hz, 10.5 s sweep is unchanged; the
  payload gets 3–30 Hz over 18.3 s (+16 s of session). The FRF uses a flat-top window: a Hanning
  window weighted the sweep's first and last bands by a few per cent, which is why the 8.3 Hz band
  never cohered on the station. The leftover-crosstalk term is bounded by the design's drift
  allowance (`crosstalk_drift_fraction` × max|g|), not the scan's repeatability: pinned at the old
  ±0.48 mrad/A bound, a drifted table biased a 5× inertia by −50% while 15/16 bands cohered.
- **Design floor (`design.wn_floor`, 2026-10-24):** the lowest gain the design and the ladder
  consider is a loop stiffness kq = J wn² of 6.75 A/rad (15 rad/s at the bare turret's 0.03, the
  fixed floor until then), never below 5 rad/s. Weaker, the loop cannot lift friction without a
  stick-slip cycle the oscillation guard reads as instability, and the ladder's ramps lag into the
  following-error limit. A 5× payload's quiet gain under the drift allowance is wn 6.5–9.3 (the
  right-half-plane zero 1/sqrt(J g) falls as 1/sqrt(J)), which the fixed 15 declared inadequate,
  leaving the blind ladder to fail the circle. The commissioned 5× servo (wn 5.5) passes the gates
  but tracks 2–3× outside the bare turret's calibrated accuracy limits: the allowance, not the
  identification, sets a payload's accuracy.
- **The prior's gains at 10×:** in the simulator kq 20 / kv 0.5 (ζ 0.08 at 0.45 A·s²/rad)
  limit-cycle through friction — 2.7 A peaks, ±0.7 rad/s about the sliding speed — and the survey
  trips the speed limit, with or without noise. Nothing can be identified in that state;
  `test_prior_gains_at_10x` is an expected failure until the prior changes.
- **Crosstalk table:** a 10 deg/s scan with 0.25 s windows reproduces a known table within
  0.6 mrad/A RMS. A 20 deg/s scan under-reads it by about 14%, which caused instability.
- **Prior gains (kq 20 / kv 0.5 / ki 20):** quiet through the survey for inertia 0.015–0.15 and
  uncorrected crosstalk up to 1.5× the measured size.

## Rules decided on the station (2026-10-02)

- **Oscillation guard at 0.3 A of fast current RMS:** normal runs peak at 0.14 A, the limit cycle
  reaches 0.3 A within 0.2 s of onset. (Production watches output minus feedforward instead: in
  tracking, hard stops at 24 rad/s² reach 0.3 A of planned current; commissioning scripts do not.)
- **Crosstalk drift allowance (`crosstalk_drift_fraction` 1.0, design.py):** the design boundary is
  taken with the table wrong by its own largest |g|. On 2026-10-02 the commissioned gains (wn 44.7,
  stable to -2 mrad/A of error) buzzed at standstill once the reading had drifted ~3 mrad/A; the
  redesign (wn 27.5, bias 3 mrad/A) is quiet to -7 mrad/A in simulation, at friction x0.5 to x2.
- **Integral rate ki = 1.5·kq, not scaled with wn:** the integral works against friction, which
  does not scale with inertia.
- **Stall recovery always on in a designed asset:** without it, ±90 deg moves stuck 0.33 deg short
  at 1.1 A. It acts when the axis is more than 0.14 deg behind while pushing, for 0.3 s, and only
  with |v_ref| < 0.01 rad/s, so it fires at targets and not at the walking profile's reversals.
- **Oscillation guard:** 50 ms above 0.3 A of fast RMS, with rocks excluded. The ladder and circle
  count a speed trip as an onset too.
- **Scoring against the measured (unbiased) crosstalk table:** the deliberate bias is real
  tracking error.
