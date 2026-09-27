# Documentation map

## Current entry points

- [`STATION_OPERATIONS.md`](STATION_OPERATIONS.md) is the live station operating runbook and remains at this path for the repository's `AGENTS.md` operating instructions.
- `ADR-001/` is the first development plan under the new document convention. It was intentionally excluded from this legacy cleanup.
- [`archive/README.md`](archive/README.md) indexes the legacy project documents by implementation state and explains their limits.
- [`references/`](references/) contains vendor protocol references and manuals. [`evidence/`](evidence/) and [`acceptance/`](acceptance/) retain the measurement artifacts and acceptance data they support.

## New document convention

`ADR-001/` is the first development plan in the new convention. The next new project document starts at `ADR-002/`; continue with monotonically increasing `ADR-NNN/` directories. Keep each new plan or decision and its supporting material inside its own ADR directory. Do not add new dated project reports to the legacy root or archive folders.

## Legacy archive status

All pre-ADR project documents have been filed under `archive/` by their dominant implementation state. The archive index lists each item and its limits. In particular, “implemented” means the documented software or procedure existed; it does not establish physical acceptance on the current station. “Partially implemented” records mixed progress or outstanding qualification. “Not implemented” identifies a legacy proposal whose planned result was not delivered in that form. “Superseded” marks retired hardware, old architecture, or stale status and operating material.

The only legacy project document left at the root is `STATION_OPERATIONS.md`, because it remains the required operating entry point. Vendor references are grouped by device under `references/` rather than mixed with project decisions.
