# ADR-002.2: read-only inventory and local evidence review

## What this is for

Establish host, release, interface and previous-stop facts before preparing physical
acquisition. An inventory cannot qualify a motor, a measurement or a stop procedure.

## Where the work happens

Collection runs as `eamars` on the station; review, correction, rehearsal and tests
run on the workstation. The collector opens no CAN/I2C transport and performs no
service action, installation, build or deployment. Its only launcher action is
`status`, conditional on an exact hash of source whose status branch was inspected.

The current session is closed to further station access after an inventory command
failure. The correction has been tested locally only. See the
[Stage 2 readiness record](../ADR-002.2/reports/STAGE2_READINESS.md).

## The command

Review an already saved capture on Windows using the project venv:

```powershell
.venv/Scripts/python.exe Firmware/tools/adr0022_inventory_review.py --input run/adr0022-stage2/station-inventory.txt --output run/adr0022-stage2/new-inventory-review.json
```

The output path must be new. Exit 2 means the collection is incomplete or failed;
preserve the capture and report. Do not retry collection to validate a correction.

The collector source is [adr0022_station_inventory.sh](../../tools/adr0022_station_inventory.sh).
It accepts an absolute checkout path, absolute runtime path and trusted launcher
SHA-256, in that order. Keep its exact bytes and the pinned SSH identity with the
capture. Do not run it under another account, infer an active release from a directory
name, or substitute `check`, `start` or a commissioning probe for `status`.

Run the local integration tests under WSL from the project root:

```bash
run/adr0022-local/.venv/bin/python -m unittest Firmware.tools.tests.test_adr0022_inventory_review -v
```

These tests use a temporary synthetic station tree, real local `ps`/Git and the real
launcher status branch. Platform observations are fixture data. No SSH is used.

## What it proves

The reviewer distinguishes a completed transfer from successful collection. Missing,
failed, duplicate, malformed or truncated evidence prevents a collection PASS.
Original captures and failed observations remain evidence.

## What it does not prove

Even PASS leaves `motion_allowed=false` and ownership unqualified. CAN link status
does not establish motor mode, measurement bandwidth, loss-free capture or current
limits. A historical stop log and absence of a launcher PID do not prove every
transmitter is stopped or either motor is disabled.

## When it fails

Stop station experimentation. Diagnose and verify the correction locally; retain the
failed attempt and raise the next required confidence threshold as the owner directed.
The current sequence is 95%, then 98.5%, then 99%, then 99.95%. These are requested
engineering acceptance thresholds, not probabilities inferred from test counts.
