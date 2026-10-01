# ADR-002.2 bounded yaw feedback probe

The [2026-10-01 architect amendment](../ADR-002.2/docs/08_IDENTIFICATION_REPAIR.md)
supersedes the earlier failed-model trial ordering below. Yaw remains unqualified;
Candidate14 is nondeployable. Complete [whole-run structure comparison](adr0022-yaw-model-comparison.md),
freeze a predictively accepted model and save a prospective closed-loop forecast
before another controller trial. Preserve the earlier procedure as historical context.

## What this is for

Run one mathematically calculated yaw controller through the shared C++ core in commissiond. Record actual parameter readback, encoder/IMU observations, references, requested/limited/successful current and the final zero-current window. A provisional probe records failures without declaring 3a acceptance.

## Where the work happens

Compute the candidate and execute the normal process probe on the workstation in the existing project venv. Cross-compile ARM64 on the workstation. Deploy a separate session-labelled release; the Pi only runs its binaries as `eamars@rpi-turret`, without sudo, through the existing launcher. Read the [deployment card](deploy.md), [runbook](../STATION_OPERATIONS.md), [yaw checklist](../ADR-002.2/YAW_TODO.md) and [execution contract](../ADR-002.2/00_CODEX_START.md).

## The command

The manifest uses `adr0022.yaw-control/1`, purpose `yaw_shared_core_3a`, measured provenance, finite current/duration/freshness limits, complete `controller_parameters`, frozen `gyro_calibration`, the measured other-axis posture and either finite quintic `reference_segments` or a finite `reference_samples` table. The sampled table is exported from the existing ADR shaper and interpolated at actual elapsed control time; it expresses the prescribed constant-speed plateau. Parameters are calculated offline; never choose physical gains by trial.

```bash
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_yaw_synthesis.py \
  --fit-json run/adr0022-stage2/yaw-information-01/full-motion-numerical-fit.json \
  --output-dir run/adr0022-stage2/yaw-feedback-01 --session-label yaw-feedback-20261001-01
run/adr0022-local/.venv/bin/python run/adr0022-stage2/yaw-control-local-probe.py \
  --binary run/adr0022-local/firmware/axis_control_core/commissiond \
  --output run/adr0022-stage2/yaw-control-local-probe-next
# For an existing calculated candidate's prescribed speed case, use the latest
# measured absolute yaw position and change only reference/session numerics.
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_yaw_reference.py \
  --manifest <calculated-control-manifest> --output <speed-manifest> \
  --speed-deg-s 5 --position-rad <latest-measured-absolute-position>
# After copying the actual capture, reproduce the frozen motion/readback metrics.
run/adr0022-local/.venv/bin/python Firmware/tools/adr0022_yaw_validate.py \
  --journal <copied-yaw-control.jsonl> --manifest <speed-manifest> \
  --candidate-context <measured-candidate-context> \
  --expected-parameters <speed-manifest> --output <actual-velocity-analysis.json>
```

The validator reports actual successful-TX plateau support, independent encoder/gyro motion, continuous motion, speed and position jitter, startup and zero-reference/final-zero stopping. Its process exit is not motion acceptance: inspect `analysis_status`, exact readback and `frozen_motion_metrics`. Recorded candidate12/13 replays reproduce their original failed metrics exactly. Use the prescribed signed speed reference that stays within the current measured local position range; preserve unsupported range as unknown.

Package the calculated measured manifest and workstation-built commissiond/IMU binaries using `adr0022_baseline_bundle.py pack --session-label LABEL`, deploy with `deploy_station.py --baseline-bundle FILE`, then use the printed command:

```bash
OTA_RUN_DIR=<release>/run/stack bash <release>/Firmware/scripts/run_application.sh \
  run --control-yaw <deployed-manifest>
```

Freeze all source edits and builds throughout the physical session. This session exclusively owns motor output; pitch remains disabled and receives only discovery, STOP and register reads. Do not write retained calibration. Read the recorded configured core parameters before interpreting the motion. Preserve raw data in ignored `run/`.

Owner update 2026-10-01: nobody will be near the station. Before the next physical session, implement and verify **30 degrees/s²** limits on yaw acceleration and deceleration, including startup assistance and controlled braking, and retain IMU vibration evidence. High-RPM steady rotation is reported stable; this update does not authorize an additional speed cap. High-acceleration hardware cases wait until the owner confirms presence around 18:00 local. Preserve unsafe STOP authority and distinguish startup/braking vibration from steady centripetal acceleration. The limit and its measured response must be recorded; reference acceleration and current slew alone do not prove a bound on actual body acceleration. Continue the automatic mathematical tuning path and the low/high speed matrix with this current acceleration bound.

## What it proves

Latest owner steering (2026-10-01) supersedes strict acceleration acceptance: **30 degrees/s² is guidance**. Record marginal exceedances and high IMU acceleration without blocking tuning. Use explicit runtime guidance mode to disable the measured-current acceleration window when it prevents friction-limited starting/tracking, while retaining reference shaping, controlled braking and real electrical/thermal/feedback/unsafe STOP protection. Continue actual yaw motion and the existing low/high speed reference validation; higher-acceleration reference cases wait until owner presence.

An executable local probe tests the full shared-core IO/recording path with synthetic devices. A station run supplies actual feedback behavior and configured parameter readback. Evaluate actual body movement and the frozen motion metrics separately.

## What it does not prove

Completion alone does not qualify the model, sensor calibration, startup map, stopping, margins or motion quality. A provisional candidate is not eligible for formal promotion. Unknown thresholds remain censored even when a bounded startup request moves the axis.

## When it fails

Record shared-core `EnvelopeLimited` as a quality observation while finite current/slew/deadline protection remains active. Data/feedback/transport/thermal failure or interruption requests yaw current zero and pitch STOP. Preserve the raw observations and actual ending state. Use launcher `status`/`stop`; do not start a second output owner or kill broad process groups. Remove only encountered development qualification prerequisites; never remove real protection.
