# GM6020 settings and actuator-interface contract

## 1. What the supplied screenshot actually shows

Source: user-provided `image(6).png` (copied unchanged as `reference/GM6020-default-settings.png`), RoboMaster Assistant **v2.7 (build 73)**. The user describes these as defaults. This is the application's version, not an established motor-firmware version. The screenshot does not prove that these settings were applied and read back on the yaw motor during every archived capture.

| Displayed field | Displayed value | Interpretation for this task |
|---|---:|---|
| ESC/motor type | GM6020 | Identify the actual motor instance and firmware separately. |
| **PWM mode** | **Position mode** | A PWM setting. It does not establish that CAN current commands run through the internal position controller. |
| Maximum angle | 360 | Do not use as a host-side validated travel/winding boundary. |
| Centre position | 4096 | Do not replace the recorded count-5768 registration or a world-frame zero with this number. |
| Position P / I | 80 / 0 | Internal coefficients with unprovided units/scaling and applicability. |
| Speed P / I | 64 / 100 | Not automatically the host velocity-PI gains. |
| Speed limit | 300 rpm | 1,800 degrees/s by unit conversion; not permission to test at that speed. Applicability to the active mode must be verified. |
| Data feedback frequency | 1 kHz | Reporting setting, not proof of current-loop bandwidth or torque response. |
| Current P / I | 1000 / 500 | Preserve and fingerprint; do not convert to external physical gains without firmware definitions. |
| Current-loop on/off | On | Important context for testing a regulated-current path; not proof of a calibrated torque/Iq signal. |

The manufacturer describes separate CAN and PWM interfaces and states that CAN has priority if both are connected. This supports treating the PWM selector separately, but does not supply the firmware-specific meaning of every displayed parameter or the newer current-command protocol. [S1]

**Do not disable the internal current loop to “remove one PID.”** A regulated-current inner loop and an external motion loop are different control levels. First establish which loops the chosen CAN command actually invokes. Conversely, do not assume an extra internal position or speed loop is active just because its gain fields are visible.

## 2. Facts to establish locally

Record these in the existing configuration manifest, not a new parallel settings subsystem:

1. **Identity and persistence:** physical axis, motor/device identity where available, firmware version, Assistant version, read-back date, boot/power-cycle state, and whether settings persist. A default-page screenshot is `USER_REPORTED_DEFAULT`, not `VERIFIED_READBACK`.
2. **Active interface and authority:** CAN vs PWM, motor ID/group, command source ownership, any other active transmitter, arbitration/timeout behaviour, and whether a residual PWM signal can become active after CAN disappears. The manufacturer's CAN-priority statement does not establish loss-of-CAN fallback timing or safety.
3. **Command/feedback schema:** exact firmware-supported frame IDs, standard/extended format, DLC, byte order, signedness, channel slots, command count-to-unit conversion, feedback count-to-unit conversion, saturation, and zero behaviour. Verify against firmware-matched documentation and local implementation; loopback decoder agreement alone is insufficient.
4. **Control semantics:** whether the chosen command targets voltage, current, or another normalized quantity; which internal loops and limits are active; current regulation and reported-current interpretation; supply and temperature effects relevant to the operating envelope.
5. **Protection and reporting:** actual feedback rate, stale-data behaviour, motor fault fields available, bus-off handling, watchdogs, and loss-of-command response. Identify which facts have been measured, documented, inferred, or remain unknown.

The earlier archive's `MEASUREMENT_SCHEMA.json#/raw_representative_records/yaw_current_tx` reports command ID **510 (0x1FE)**. That is a recorded software/traffic fact to cross-check, not this review's certification that a particular firmware implements its assumed physical scaling. Do not substitute an older voltage-mode manual simply because it also describes a GM6020.

No exact current-firmware protocol or motor-specific applied-settings readback was independently verified in this review. The local agent must resolve these from its actual motor/firmware evidence. Where the runtime protocol has no settings read API, use supported vendor readback/export rather than inventing one. Missing torque calibration may be absorbed into a supported lumped model; missing command semantics or safety behaviour is more fundamental.

## 3. Keep the motor configuration fixed during identification

Treat applied current-loop gains, firmware, active mode, supply arrangement, and sensor settings as part of the plant fingerprint. Changing any of them can change the model. Do not simultaneously modify internal current PI, host velocity PI, sensor filters, and friction coefficients; the resulting capture would be difficult to interpret.

Do not automatically restore factory settings, click Set, flash firmware, raise current limits, or change motor mode. Obtain authorization for a needed mutation, save the old configuration, apply one controlled change, read it back, and create a new configuration identity. Retain the earlier data under its original identity. A rollback needs verified readback, not merely a successful GUI click or CAN send.

Factory displayed gains alone cannot identify electrical time constants, physical torque constant, inertia, or closed-loop bandwidth. Product rated torque/current also should not be converted into a precise torque constant without the relevant electrical conventions and operating conditions. For this task, a stable command-to-motion A-equivalent coefficient is acceptable when validated within its configuration.

## 4. Current and timing experiments, only when needed and authorized

First use existing successful TX and current telemetry to assess gain, offset, sign, and timing consistency. Separate a reporting path from a physical actuator path; do not assume one reported-current sample is a direct, unfiltered measurement of torque-producing current.

If additional excitation is necessary, use a separately approved bounded plan that respects motion as well as electrical/thermal limits. Do not lock or stall a motor without a permitted method, or raise bandwidth/amplitude merely to get a cleaner identification signal. Observe mechanical motion independently while investigating current response.

Keep transport delay, actuator regulation, gyro/filter delay, and breakaway latency as separate model quantities or bounded uncertainties. Unknown physical latency is not automatically equal to a host receive-age statistic. A 1 kHz reporting rate is not evidence that all of these are sub-millisecond.

## 5. What this screenshot changes in the architecture

It strengthens the priority of verifying and modelling an **inner regulated-current path** before treating the whole command-to-motion onset as one fixed delay. It does not provide new deployable gains or invalidate the earlier decoder audit. The immediate action is an interface/configuration check in parallel with the synthetic estimator repair, not another motor-setting experiment.
