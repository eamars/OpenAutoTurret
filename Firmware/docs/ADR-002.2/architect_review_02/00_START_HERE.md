# ADR-002.2 — Next local-agent work package

**Date:** 1 October 2026. **Application:** benign camera pan/tilt and framing.  
**Decision:** recover and qualify the estimation procedure, then identify the physical model and synthesize feedforward/feedback. Continue useful offline work whenever hardware promotion is blocked.  
**Current status:** yaw UNQUALIFIED; Candidate14 NONDEPLOYABLE; no new physical trial, motor-setting change, or production deployment is authorized by this package.

## Start with this instruction

Read `01_EVIDENCE_REVIEW.md`, `02_LOCAL_AGENT_INSTRUCTIONS.md`, `03_GM6020_INTERFACE.md`, `04_FEEDFORWARD_AND_VALIDATION.md`, and `05_FAILURE_MODES_AND_RECOVERY.md`. Follow `WORK_QUEUE.json` in dependency order. These are parts of one delivery, not separate MVPs.

First reproduce the supplied evidence audit. Then locate the existing local source implementing the synthetic probe, plant integration, residual construction, optimizer, and native control core. Preserve the working parts. Address the smallest falsifiable failure before adding another family, doing another capture campaign, or generating another controller candidate.

The immediate scientific question is **why the known four-parameter noisy synthetic problem produces one poor cross-run predictor and three exhausted optimizer budgets**. It is not yet “which elaborate friction law must the real hardware have?” A pristine synthetic reference already works. Do not restart that work or incorrectly report that all parameter recovery failed.

At every failure, separate numerical execution, optimizer convergence, parameter recovery, trajectory prediction, physical qualification, and deployment authorization. A failed gate blocks its dependent promotion; it does not automatically end the engineering task.

## Reproduce this package's audit

From this package directory, with the evidence ZIP in the parent directory:

```bash
python -c "import sys, numpy; print(sys.version); print(numpy.__version__)"
python tools/audit_evidence.py ../ADR-002.2-model-evidence-20261001.zip --output audit-local
python -m unittest discover -s tools -p 'test_*.py'
```

Python 3.10+ and NumPy are sufficient. Use the project's existing environment; do not upgrade production dependencies to match documentation examples. An alternative ZIP location can be supplied as the positional argument. These commands are read-only with respect to the source archive and have no hardware or network access.

Expected audit: 11 observation runs, 88 predictions, 470 reproduced numeric comparisons, all evidence-consistency checks passing. **This result verifies the exported evidence, not the plant or controller.** The package's `audit/` directory contains the output actually executed by the reviewer.

Physical experiments may proceed only within independently established user authorization and verified limits; this package neither creates broader permission nor invalidates an existing explicit, applicable authorization.

No project implementation is included in the user's evidence archive. Source paths in its JSON are provenance, not code supplied to this reviewer. Native fitting and control changes must therefore be made and tested in your existing repository, not in a speculative replacement project. The executable in this package audits data; it does not claim to repair the unknown native implementation.

## First work cycle

1. Reproduce the audit and commit an evidence/source inventory without modifying the source evidence. Record the actual native library and source revision used locally.
2. Reproduce the existing pristine and three noisy synthetic cases. Preserve seeds 17, 41, and 83 as named regressions. Produce the separate gate flags described in WP1.
3. Reduce noisy seed 41 to one deterministic fit/replay command. Run the oracle, residual, numerical-step, scaling, and derivative/event diagnostics in WP2, choosing tests that distinguish live hypotheses.
4. In parallel, inventory the actual GM6020 interface and read-back configuration. The provided defaults are context, not proof of the historical settings. Do not energize or modify settings for this inventory without existing authorization.
5. Select one justified repair, preserve a before/after regression, and continue the work queue. Do not stop after writing a diagnostic report when an implementable offline repair remains.

## Completion, continuation, and a real blocker

**Complete** means the original scope passes Stage 1, Stage 2, physical Stage 3a, and production physical Stage 3b. The yaw-only audit and estimator repair are dependency work, not a reduced final deliverable. Payload/configuration changes must use the same automatic identification and verification procedure.

**Continue** means a gate failed but you still have safe authorized analysis, code inspection, simulation, or targeted repair available. Carry out the next diagnostic branch. A finite optimizer budget or local convergence with large errors is not a proof of infeasibility.

**Blocked pending input** is legitimate when the remaining useful action requires missing hardware access, an unprovided firmware/interface fact that affects safety, a physical configuration change, or explicit approval. State exactly what is missing, its required accuracy, which hypotheses it distinguishes, the smallest safe next action, and what work was completed meanwhile. Do not label uncertainty, a failed fit, or unfamiliar mathematics “too hard.” Do not manufacture certainty either.

## Files

- `01_EVIDENCE_REVIEW.md`: what the new records establish and what they do not.
- `02_LOCAL_AGENT_INSTRUCTIONS.md`: concrete recovery, identification, and decision procedure.
- `03_GM6020_INTERFACE.md`: settings interpretation and actuator-interface contract.
- `04_FEEDFORWARD_AND_VALIDATION.md`: feedforward integration, transitions, reuse, and full validation.
- `05_FAILURE_MODES_AND_RECOVERY.md`: failure modes, physical fault states, and recovery rules.
- `WORK_QUEUE.json`, `templates/`: lightweight records; adapt existing project formats rather than creating a parallel framework.
- `tools/`, `audit/`: executed evidence audit, tests, and results.
- `SOURCES.md`, `MANIFEST.sha256`: evidence provenance and source references. Newly generated hashes are not historical integrity attestations.
