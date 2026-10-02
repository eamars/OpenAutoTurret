# Sources, evidence scope, and methodological references

## Supplied evidence

- `ADR-002.2-model-evidence-20261001.zip`: all 115 archive members were inventoried. JSON assumptions/results and all 88 physical exported predictions were consumed; the two supplied failed synthetic cross-run predictions were independently recomputed. See exact member/JSON references in `01_EVIDENCE_REVIEW.md`.
- `ADR-002.2-independent-review.md`: prior review, supplied in this conversation. Source for retained physical acceptance limits, Candidate13/Candidate14 status, previous audit and architecture. The new review explicitly updates its small-window and task-termination implications.
- `ADR-002.2-yaw-physics-math-20261001.zip`: previously reviewed raw archive. In this next-stage work, the representative `MEASUREMENT_SCHEMA.json` command record was cross-checked; this is not a second claim to have reprocessed all 25 raw journals in the current audit.
- `image(6).png`: user-provided GM6020 defaults screenshot. Values were read visually. It is not motor-firmware readback for a specified historical run. A structured transcription is in `templates/gm6020_screenshot_context.json`.

No current project source was provided in the new ZIP. Referenced source paths and local native-library paths are provenance only. This package does not certify the native estimator/controller implementation or physical sensor/current semantics.

## Primary methodological/product references checked 1 October 2026

**S1 — DJI RoboMaster GM6020 product documentation.**
https://www.robomaster.com/zh-CN/products/components/general/gm6020
The manufacturer describes the integrated driver and separate CAN/PWM interfaces, including CAN priority when both are connected. It does not resolve the actual unit's firmware-specific current protocol or undocumented coefficient scaling. These remain explicit local checks.

**S2 — SciPy `least_squares` documentation.**
https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html
Relevant topics: local minimization, `xtol`/termination status, characteristic parameter scaling, finite-difference step semantics, and the analytic-continuation requirement for complex-step derivatives. Documentation version observed was 1.18.0; use the actual installed local version and record it. The recovery procedures in this package are recommendations for the supplied evidence, not claims that a solver guarantees physical identification.

**S3 — SciPy `solve_ivp` documentation.**
https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html
Relevant topic: event detection by sign changes across steps and possible missed multiple crossings. Event-specific regression and step convergence are recommended here; no specific solver has been shown to repair the local implementation.

**S4 — Russ Tedrake, Underactuated Robotics, Chapter 18: System Identification.**
https://underactuated.mit.edu/sysid.html
Relevant topics: equation versus simulation error, identifiable/lumped physical parameters, and experiment design. The particular model family, experiments, and stage gates in this package are project-specific recommendations.

## Interpretation of evidence

`audit/` reports directly recomputed exported-array metrics separately from reported optimizer metadata. Matching error summaries does not establish the source of the physical model mismatch. Source-specific observations and proposed repairs are kept distinct in the review.

New SHA-256 digests identify the archive used and the generated package files. They do not prove integrity of unseen upstream originals and do not retroactively supply a checksum to the original handoff. The source archive was not modified. No proprietary binaries, model gains for deployment, or font files are included.
