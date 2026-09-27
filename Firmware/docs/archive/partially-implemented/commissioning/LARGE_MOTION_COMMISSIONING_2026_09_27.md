# Large-motion commissioning on the split-bus camera station

Observed 27 September 2026. These are bounded single-axis commissioning sessions,
not approval to start the old automatic controller. The owner confirmed direct
drive on both axes and repositioned the station before the successful sessions.
Each launcher run established a fresh motor reference and BNO085 host tare.

## Pitch: continuous energized ±15°

Committed release `1c04087335ef`, path
`/home/eamars/workspace/OpenAutoTurret/run/releases/1c04087335ef.hNhDQE`.
The launcher started one CyberGear position-mode session on CAN1 with the
production 5 A readback interlock, service speed-loop gains 4/0.05, and a
requested 10°/s speed limit. Pitch stayed enabled through four direct targets:
origin → +15° → origin → −15° → origin. It settled at each target without
cycling CAN or the motor. The probe then restored nominal gains 1/0.002,
requested STOP, and verified disabled feedback and `LimitCur=5 A`.

| Leg | Encoder movement | Endpoint error | Game-RV incremental angle |
|---|---:|---:|---:|
| +15° | +15.038° | +0.038° | 14.538° |
| Return | −15.059° | −0.022° | 14.978° |
| −15° | −15.059° | −0.081° | 14.985° |
| Return | +15.125° | +0.022° | 14.889° |

Guard trips, faults, CAN errors, stop failures and I2C errors were all zero.
All four endpoints reported enabled, settled, fault-free feedback. Filtered
CyberGear current readbacks peaked at **1.034 A**; the verified 5 A ceiling was
available throughout. These samples do not bound instantaneous current.

Game-RV status was 3 for all 606 samples, with no sequence gaps or reorderings.
Its encoder-angle ratios for the four legs were 0.967, 0.995, 0.995 and 0.984;
the ±15° mean is 0.985. The earlier +3° session's approximately 0.862 ratio
did not persist. The station position and movement size both changed, so neither
can be isolated as the cause. Do not persist a scale correction from either run.
The return left about 0.377° post-tare IMU residual against −0.022° encoder
residual. This is an observer measurement, not a base-pose calibration.

Pitch retains physical endstops. Its absolute endpoint angles have not been
commissioned, and this movement does not certify a pitch soft-limit envelope or
homing routine. The runtime probe limited excursion from its starting pose to
17°, monitored encoder-derived speed, stopped on stale/fault/current/temperature
conditions, and checked progress under nonzero target demand.

## Yaw: 30° and return

The first 30° test (release `a1b1cf98b979`) used a 3,000-raw voltage ceiling
but demanded only 2,170 raw at peak. It moved 3.955° at most and hit its
15-second deadline. CAN, zero-output stop and IMU acquisition remained healthy.
The failure was a control-demand shortfall, not evidence of a yaw endpoint.

After the controller gained the GM6020 v1.4 documented ±25,000-raw voltage
headroom and restored speed-loop response, a fresh session from the owner's
repositioned pose succeeded on release `6d7210221dbf`, path
`/home/eamars/workspace/OpenAutoTurret/run/releases/6d7210221dbf.1rope2`.
The motor moved +29.356° before the return stage, finished +0.659° from its
session origin, and was observed stationary after repeated zero-voltage requests.
Peak encoder-derived speed was 19.309°/s under the 25°/s guard. Actual voltage
commands ranged from −4,268 to +5,643 raw; the larger ceiling was available
but did not require saturation. Raw feedback current ranged −3,348 to +4,375;
those are **not amperes**. Guard trips, CAN errors, TX failures and zero-output
failures were zero. GM6020 has no verified disable acknowledgement, so a zero
frame and stationary feedback are not proof of power removal or parking.

Paired BNO085 game-RV observations independently measured +29.183° outbound
against +29.356° encoder (ratio 0.994), then −28.446° on return against
−28.740° encoder (ratio 0.990). Gyro integrations were +29.503° and
−28.725°. All 731 game-RV samples reported status 3 with no capture gaps;
the p95 cadence was 22.70 ms and p95 age 12.34 ms. The post-tare IMU residual
was about +0.790° against the final +0.659° encoder residual. The estimated
outbound and return yaw directions differed by 0.92° after sign correction.
These comparisons confirm the motion and direction, but they are not a full
IMU-to-axis mounting calibration or an absolute world-heading reference.

Both CAN links were kept UP at 1 Mbps between sessions. The automatic stack
remained stopped. The launcher owns each probe and sends an additional zero/STOP
request if its commissioning child exits unexpectedly; this is still a host
process path and cannot survive Pi power or OS loss.

Ignored raw captures under `run/hardware-adaptation/`:
`imu-pitch-pm15.{csv,ndjson,log,analysis.json}` and
`imu-yaw-p30-v2.{csv,ndjson,log,analysis.json}`. Source and summary values are
committed, while numeric captures remain outside Git.
