# BNO085 acquisition, tare and motion evidence

Status: **verified acquisition and relative yaw observation; mounting calibration
and controller integration remain incomplete**. 27 September 2026.

The launcher now owns a versioned SH-2 Linux acquisition executable on I2C1,
address 0x4A. It records raw acceleration (m/s²), calibrated gyro reports
(rad/s), magnetic rotation vector and game rotation vector (quaternion xyzw).
The game rotation vector is the current relative-motion reference: it reports
accuracy 3, while magnetic RV and gyro reports currently report accuracy 0.
Do not claim a calibrated absolute heading from these readings.

## Startup and I2C behavior

The first live port exposed two real integration errors: the lab's 20 ms
post-reset delay raced startup, and a newly added check incorrectly expected
SHTP transfer sequence numbers to remain constant across I2C reads. A 300 ms
settling delay and accepting sequence advancement restored sensor identity and
samples. Product responses include part 10004148, version 3.2.13, build 6.

The owner supplied the earlier `components/bno08x/src/bno085_i2c.c` port as a
reference. Its relevant behavior is a header peek followed by one complete
packet receive, 300 ms reset settling and soft-reset recovery after I2C errors.
The Linux acquisition now follows that read structure. Recovery occurs outside
the SH-2 callback, retries the session at most once, increments a generation
and invalidates the old tare. No persistent calibration or tare writes occur.
The earlier ESP-IDF port uses 400 kHz; the Pi clock configuration was not changed
because the measured captures succeeded with its existing configuration.
Automatic recovery from an induced runtime I2C fault has not yet been verified.

An initial 10-second capture returned 497 samples per orientation/gyro stream,
no missing sequence numbers and no I2C errors. The 30-second whole-packet/tare
run returned 1,481 samples per orientation/gyro stream and 1,935 accelerometer
samples. No sequence gaps, I2C errors or recovery were recorded. Game-RV median
period was 20.03 ms, p95 22.70 ms; reported host-relative sample age p95 12.36 ms.
Stationary end-to-end orientation change was 0.099°. These are polling-derived
timestamps, not captured interrupt edges or a measured absolute latency.

## Host reference versus mounting alignment

After two seconds of stationary samples, the executable averages a normalized
game-RV reference and emits `q_ref^-1 * q_current` as `relative_xyzw`. Raw
quaternions remain available. Stationarity checks fresh gyro/acceleration,
plausible gravity magnitude, quaternion norm and game-RV accuracy. The reference
is valid only for its uninterrupted acquisition generation.

This zero reference uses the sensor axes at the initial pose. It does **not**
establish sensor-to-camera axes, pitch zero, world north or `R_W_B`. The IMU is
attached to the pitch assembly, so assigning its raw pose as the stationary base
orientation would count joint movement twice. Mount alignment remains explicitly
false until measured yaw and pitch axes and the camera boresight are reconciled.

## Paired yaw observations

All tests used the existing guarded commissioning path; pitch remained disabled.
The pitch current ceiling was written and read back as 5 A three times beforehand.

| Command | Encoder result | Independent IMU observation | Outcome |
|---|---|---|---|
| +2000 raw, 100 ms | +0.307617° final | 0.283929° common-window rotation | Completed; too little motion for axis acceptance |
| +2000 raw, 200 ms | 0° final, 0.043945° peak | No useful calibration motion | Completed |
| −2000 raw, 150 ms | 24°/s reported speed; −0.878906° final | Capture retained; common trace truncated by guard | Speed guard requested zero; stationary afterward |
| −1500 raw, 150 ms | −0.527344° final; 1.01074° peak; 18°/s peak speed | 0.467606° common-window rotation; 1.013604° peak after tare | Completed; stationary afterward |

The last trial gives a provisional positive-yaw axis of approximately
`[-0.980, -0.086, +0.181]` in the initial sensor frame. This is a single small
motion estimate, not a qualified mounting matrix. The peak displacement agrees
closely; final displacement differs by about 0.060°. Repeatability, pitch-axis
measurement, timestamp alignment under load and IMU drift remain to be qualified.
Do not raise the motor guards based on this result.

## Running and retaining evidence

From a committed release, with the station stopped:

```bash
bash Firmware/scripts/run_application.sh run --probe-imu --imu-seconds 30
run/station-venv/bin/python Firmware/tools/analyse_imu_motion.py \
  --imu /tmp/ota-stack-1000/imu.ndjson
```

`--commission-hardware --with-imu` captures the same stream, requires a fresh
host tare before starting the bounded motor probe, and stops the motor probe if
the IMU process exits. This is an observer, not an independent power cutoff.
The existing motor feedback, output, speed, travel and heartbeat guards remain.

Live acquisition/paired-motion release:
`/home/eamars/workspace/OpenAutoTurret/run/releases/f3accc6d096b.5muAaf`.
Numeric captures are ignored under `run/hardware-adaptation/imu-*.ndjson`,
paired `.csv`/`.log` files and analysis JSON. Captures are not committed.
