# Loop timing and capture-device inventory — 2026-09-04, round 33

Measured on this station (`rpi-turret`), by the agent that wrote the code. Unsigned. Nothing here is acceptance.

## 1. What `control_cycle_us` actually is — established from the expression, not from a summary

`control/src/control/control_loop.cpp:1368` — `snap.control_cycle_us = period_ns / 1000;`
`control/src/control/control_loop.cpp:243` — `Phase ControlLoop::step(TimeNs now_ns, TimeNs period_ns)`
`control/src/control/control_loop.cpp:369` — `if (period_ns > deadline_ns_) { overrun_us = (period_ns - deadline_ns_) / 1000; }`

`period_ns` is the interval the caller supplies per step, and line 369 can only detect an overrun if that
interval is the **measured** one. So `control_cycle_us` is the loop's real wake-to-wake interval, not a
configured constant. (Checked because the alternative reading — a mirror of the configured 5 ms — would have
made every number below meaningless. An earlier round mischaracterised a different metric exactly this way.)

## 2. Measurement

Sampled `/api/state` twelve times over ~22 s while the station sat homed, ready, MANUAL/HOLD, with synthetic
vision running (so the loop was doing its normal work, not idling):

    control_cycle_us   min=5052  p50=5054  max=5054  n=12

**Achieved rate ≈ 197.9 Hz against a nominal 200 Hz, with ~53–54 µs of consistent overrun past the 5 ms
deadline, and near-zero spread.** The uniformity says the loop overshoots its deadline by a fixed amount every
cycle rather than occasionally blowing it — the shape of a sleep-until-deadline loop whose wake-up latency plus
work exceeds the period slightly.

What this sample is NOT, stated plainly:

* It is **not** a distribution over cycles. Telemetry publishes the *most recent* cycle's value at the snapshot
  rate, so twelve samples ~2 s apart is twelve peeks, not thousands of cycles. A tail (a 5 ms spike) could sit
  between samples unnoticed. The narrow spread of what was seen argues against large jitter, it does not exclude it.
* **No overrun counter reaches the operator.** `overrun_us` is computed in the loop at line 370, but
  `grep -c overruns` returns **0** in `web_server.hpp` and **0** in `web/webd/protocol.py`. The number exists at
  the point of measurement and is then dropped — so a station running over its deadline all day shows nothing
  about it on screen. Until that is fixed, "200 Hz within measured limits" cannot be *seen*, only inferred.
* `docs/evidence/` contained **no** timing artifact of any kind before this file (count: 0).

## 3. Capture-device inventory — a blocker I asserted on the wrong grounds

For many rounds the record has said camera-geometry commissioning is blocked because `visiond`'s real path needs
an image-config JSON and an IMX500 detector `.rpk`, and neither asset exists. **That argument was about assets.
Nobody had checked the hardware.** Inventory now:

    /dev/media0 .. /dev/media3 present;  picamera2 importable;  /usr/bin/rpicam-still present

    $ timeout 40 rpicam-still -o /tmp/probe_r33.jpg -t 800 -n --width 1920 --height 1080 -q 90
    Sensor: /base/.../i2c@88000/imx500@1a - Selected sensor format: 2028x1520-SRGGB10_1X10/RAW
    Still capture image received          [exit 0]     1920x1080, 188,437 bytes

**The IMX500 captures.** So the sensor is attached and usable, and the missing detector package constrains
*automated detection*, not *capture*. Everything that follows from capture — real-frame FOV, principal point,
and the encoder-as-theodolite boresight sweep — is no longer asset-blocked. Frame statistics from the probe
capture (`1920x1080`, mean 93.7, **std 99.1**, per-column vertical-edge energy median 824 with a peak of 16,500
near column 826, a 20× ratio) say the scene has structure strong enough for frame-to-frame phase correlation,
which is how deg-per-pixel is measured from real optics without any detector.

Two honest limits on that optimism:

* **Boresight still needs a surveyed physical reference.** Real frames give scale and principal point; an
  absolute camera-to-axis offset needs a distant feature at a *known bearing*, which requires someone on site to
  place and survey one. That is physical, not software, and it is the operator's.
* **This model cannot look at images** (`read_image` refuses: no image input declared), so every measurement off
  real frames must be numeric — edge profiles, phase correlation — never "I saw the target". That rules out any
  acceptance claim that depends on reading the picture, and it is why §24 remains a human judgement.

## 4. Consequences for the running record

1. "200 Hz must stay within measured limits" now has a fresh measurement: **~197.9 Hz, ~54 µs consistent
   overrun, n=12 peeks, no overrun indicator published.** The clause is not satisfied by instrumentation the
   operator can see; publishing the overrun counter (and a p95/max over a window) is the next concrete step.
2. Objective item (c) is **no longer blocked on assets**, only on a physical surveyed reference. Real-frame FOV
   and principal-point measurement are actionable now; the previously locked FOV figures should be re-derived
   from real optics when the sweep is run, and the synthetic-derived numbers labelled as such until then.
