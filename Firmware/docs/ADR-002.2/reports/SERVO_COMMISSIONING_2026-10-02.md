# Automatic servo commissioning: ADR-002.x closing report, 2026-10-02

Procedure: [servo commissioning card](../../operations/servo-commissioning.md). Developer map, plant
model and calibrations: [`tools/servo_commission/README.md`](../../../tools/servo_commission/README.md).
Overnight hand-tuned predecessor: [takeover report](SERVO_TAKEOVER_2026-10-02.md).

## What closes ADR-002.x

ADR-002.2 asked for these deliverables, all now in place:

- **Reusable identification assets (D2).**
- **Model-first design with closed-loop prediction (D3).**
- **A pitch interface (D4).**
- **One control core shared by commissioning and the simulator (D5).**
- **Payload changes handled as a feature (D6).**
- **Applied, read-back parameters (D7).**
- **Fixed evaluators (D8).**

Commissioning is now a single deterministic pipeline per axis. It starts from a prior that
assumes nothing about the plant and ends with an asset that the station has verified. The owner
ruled the architect documents to be guidance, so methods differ from the ADR text where the
station said otherwise:

- **Pitch** stays in the drive's speed mode under a 1 kHz host position loop. That performed best,
  per the owner's "pick what performs best" ruling.
- **Production integration (3b)** is the first ADR-003 work item (owner ruling: production stays off
  until ADR-003). Commissioning and the future production backend share `axis_control_core`.

## Yaw and pitch on the station, from nothing

Seven `commission.py yaw` runs, each starting from [`yaw_prior.json`](../../../config/servo/yaw_prior.json),
between 09:37 and 13:00. Each one exposed a pipeline defect, listed below. Run 7 used every fix, and
its asset is the one in [`yaw_servo.json`](../../../config/servo/yaw_servo.json). The bearing's
friction changed a lot during the day (peak 0.59 → 0.81 → 1.1 A within three hours); everything
else repeated.

| Run | Inertia | Crosstalk max / delay | Station onset wn | Final kq / kv / ki | Use case (gates) |
|---|---|---|---|---|---|
| 1 | 0.047 | 6.13 mrad/A / 1.18 ms | 54.5 | 78 / 1.92 / 89 | pass 2 accepted (pass 1 hit the stall-recovery bug) |
| 2 | 0.044 | 6.26 / 1.20 | 64.4 | 82 / 1.90 / 124 | the guard misread a stall rock as oscillation (fixed) |
| 4 | 0.044 | 6.67 / 1.18 | 59.4 | 109 / 2.19 / 163 | accepted ×2 |
| 5 | 0.042 | 6.27 / 1.20 | 59.4 | 68 / 1.69 / 101 | accepted ×2 (4 rocks in one walking profile; fixed) |
| 7 | 0.041 | 6.18 / 1.20 | 53.7 | **81 / 1.82 / 122** | **accepted ×2; the circle check was quiet first time** |
| Overnight hand tuning | 0.03 (est.) | 6.2–6.4 | kq 120 / kv 2.0 | 90 / 1.6 / 135 | re-scored: walking profile 0.15 deg |

Run 7's use case:
- **Speed ratio:** 0.997–1.024.
- **Ramp error RMS:** 0.055–0.15 deg.
- **Walking profile:** 0.14–0.16 deg RMS.
- **±90 deg moves:** stop within 0.15–0.26 deg.
- **Small steps:** settle within 0.25–0.30 deg (advisory above the provisional 0.25).
- **2 and 5 deg/s ramps:** detrended P95–P5 reached 0.19 and 0.31 deg in pass 2 (advisory).

Pitch: `commission.py pitch` from [`pitch_prior.json`](../../../config/servo/pitch_prior.json) was
accepted twice, giving [`pitch_servo.json`](../../../config/servo/pitch_servo.json):
- **Gain:** kp 60/s, ki 90/s², quiet on the station up to kp 90.
- **Ramps:** 0.021–0.045 deg RMS.
- **Walking profile:** 0.031 deg RMS.
- **Steps:** settle within 0.007 deg.

That beats the overnight hand-tuned kp 35 (0.03–0.06 deg).

**Acceptance.** The gates are the session completing without a guard trip, the axis really
tracking (speed ratio within 10%) and at most 4 stalls, plus the margin checks (ladder and circle).
The accuracy numbers are reported against provisional targets but do not gate. ADR-003's photography
spec leaves the framing budget unspecified and says not to infer tolerances from achieved
performance. Once it is filled in, the calibrated pixel-to-angle conversion gives the real limits
for `score.py`.

**Decoupling.** During every session the other axis is unpowered and rests where it was parked. Its
own friction held it:
- the pitch moved at most 0.04 deg across 36 yaw sessions (its sensor resolution is 0.02 deg);
- the yaw moved 0.044 deg (one count) during pitch sessions.

The pipeline records this for each session and warns above 0.2 deg. Commission pitch first: it parks
at the centre of its window, so yaw is identified at a defined pitch posture (recorded as the
operating point).

## Offline proof (simulated stations)

`tools/servo_commission/tests/test_pipeline.py` runs the identical pipeline against two hidden plants:

- this station's estimate;
- a 2.5× payload with 15% more friction.

| | Estimate | Heavy payload |
|---|---|---|
| Inertia, true / identified | 0.030 / 0.033 | 0.075 / 0.077–0.083 |
| Final kq / kv | 69 / 1.5 | about 160 / 3.7 |
| Use case | accepted twice | accepted twice |

The gains scale with the payload, with no code change. When friction rose 30% instead, slow
sliding needed more than the then-current budget. The pipeline stopped with a capability
diagnosis rather than a tuning failure.

## Corrections found on the way

None of these were visible until something ran:

1. **Slew limit drag.** The servo's slew limit followed the acknowledged *total* current. An
   identification sweep above 0.2 A per cycle therefore ratcheted the output to the cap. It now
   follows the servo's own output, with a regression test (`servo_core`).
2. **Biased scoring.** The overnight scores used the biased crosstalk table as "truth", which hid up
   to 0.07 deg of real error. The scorer now uses the measured table. Re-scored, the overnight
   result is a walking profile of 0.15 deg RMS (reported as 0.12) and ±90 deg moves 0.20–0.25 deg
   short (reported 0.17–0.20).
3. **Stall recovery missing from designed assets.** The prior has stall recovery off, and the
   designed asset inherited that. ±90 deg moves then stuck 0.33 deg short at 1.1 A. Designed assets
   now always enable it.
4. **Integral rate.** Scaling the integral with wn left steps 0.2 deg short at higher inertia. The
   integral works against friction, so ki = 1.5·kq.
5. **Guard false alarm.** A stall rock at the 240–255 deg bump looked like an oscillation to the
   guard. The guard now ignores the rock and the 0.2 s after it, and needs 0.15 s above its limit. A
   unit test covers both a 15 Hz cycle and a rock.
6. **Ladder angles.** The circle check exposed weak angles the ladder had not tested (150–175 deg,
   where the table is steep and the two scan directions disagree most). The ladder now tests the
   angle of greatest scan disagreement and the steepest part of the table.
7. **Current limits** (owner ruling): 3 A peak and 1.62 A continuous; 0.8 A is guidance.
8. **Guard timing.** The guard first required 0.15 s above its limit. That let a real limit cycle
   grow into the speed trip. With rocks excluded explicitly, 50 ms suffices, and the pipeline counts a
   speed trip as an onset too.
9. **Stall threshold.** Stall recovery used a 0.29 deg threshold, so small steps could sit 0.27-0.29
   deg short at 1.3 A, just inside it, at a sticky spot near 135 deg. It is now 0.14 deg, still
   outside the 0.23 deg hold band, so the servo never rocks at rest.
10. **Rock only near a stopped reference.** With the lower threshold, rocks fired during the walking
    profile's slow reversals: four rocks gave 0.26 deg RMS there. Rocks now need |v_ref| < 0.01
    rad/s, which holds in step and move tails but only for a moment at walking reversals. A unit test
    covers it.
11. **Model inadequate: the station decides.** At 12:23 the bearing was much stickier: 1.1 A at
    0.25 deg/s, with stick-slip in the survey's slow plateaus. The fitted friction curve then made
    the model unstable at every gain, and the pipeline ran its ladder at uselessly low gains. Now it
    says so (`MODEL INADEQUATE`) and ladders a fixed range on the station.
12. **Pitch homing.** commissiond's sensorless homing had only ever run in synthetic rehearsals.
    On the station, its torque-off rearm drops the loaded pitch. Pitch commissioning uses
    production's homed window; making commissiond's homing hold the payload is ADR-003 work.

## What the model does and does not predict

The simulator (`axis_control_core/plant.*`, `simulate.*`) runs the same Servo, PositionLoop and
session timing as commissiond.

| Quantity | Model vs station |
|---|---|
| Inertia | identified within about 10% in simulation; the two station runs repeat within 7% |
| Crosstalk | identified within 0.6 mrad/A in simulation; the station runs repeat within 0.13 mrad/A at the peak |
| Stability boundary | predicted within 11–17% of the station ladder |
| Weakest angles | ladder-04 overnight: 176 deg on the station, 182 deg in the simulation |
| Tracking error | optimistic, e.g. walking profile 0.03–0.06 deg predicted against 0.15–0.24 measured |

The tracking error is optimistic because the model has no rest-dependent breakaway, no friction
drift and no bump. The station therefore decides gains and acceptance; the model narrows where to
look and designs the starting point. Both numbers are kept side by side in every report.

## Open items

- **Production integration (ADR-003 start).**
- **Bearing friction.** The friction swings ±35% within an hour, and the slow-sliding current
  exceeds the 0.8 A thermal guidance. Mechanical inspection (preload, grease, the 240–255 deg bump)
  is still the cheapest improvement.
- **Small-step accuracy (≈0.2 deg).** This is bounded by the hold band (0.23 deg), which is chosen so
  the motor does not hold full current against static friction at rest.
