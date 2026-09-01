---
name: probe-first-engineering
description: Use this skill when implementing or materially modifying software where success depends on runtime behavior, integration, state, I/O, external services, hardware, timing, concurrency, persistence, UI interaction, or other behavior that static inspection cannot prove. Force an early executable design probe before spending significant effort on new unit-test coverage, mocks, static analysis, lint/style cleanup, or exhaustive hardening. Do not use for explicitly test-only, lint-only, documentation-only, or purely mechanical edits with no meaningful runtime behavior.
metadata:
  philosophy: "runtime evidence before hardening"
  version: "1.0.0"
---

# Probe-First Engineering

## Purpose

Optimize engineering work for **early falsification of the design**, not early perfection of an unverified implementation.

The required order is:

**design hypothesis -> executable probe -> evidence -> hardening -> regression verification**

not:

**implementation -> unit-test saturation -> lint/static perfection -> first real run**

A system is not demonstrated to work merely because it compiles, passes static analysis, satisfies mocks, has high unit-test coverage, or passes tests derived from the same implementation assumptions.

Unit tests are valuable evidence of **local behavior**. They are not a substitute for observing the important behavior across the boundary where the design can fail.

---

## Core rule: probe before polish

As soon as there is enough implementation to exercise the critical design assumption, **stop adding code and run a probe**.

Before the first successful design probe, DO NOT:

- build out a broad new unit-test suite;
- chase coverage percentages;
- add large mock frameworks;
- perform blanket lint or static-analysis cleanup;
- spend time on formatting or style perfection;
- refactor working code only for elegance;
- build abstractions needed only for hypothetical future cases;
- make a failing design look complete by testing its internal implementation in isolation.

Allowed before the first probe:

- the minimum compile/build/type work required to make the probe executable;
- targeted safety checks required to avoid damaging data, systems, hardware, users, or external services;
- existing repository checks when they are a prerequisite to running the code;
- instrumentation needed to observe the probe;
- a small executable harness or disposable script used to run the probe.

These allowances are **preconditions for obtaining runtime evidence**, not a reason to postpone the probe.

If explicit higher-priority instructions require test-first development, follow them, but keep pre-probe tests minimal and still obtain meaningful runtime evidence as early as possible.

---

## Definitions

### Design hypothesis

A claim that must be true for the chosen implementation approach to succeed.

Examples of hypothesis classes include:

- two components can actually communicate as assumed;
- state survives or propagates as assumed;
- the runtime or dependency exposes the behavior the design relies on;
- timing, ordering, concurrency, or latency are acceptable;
- a control or feedback path has the expected sign and response;
- a public API behaves as assumed under representative input;
- a data transformation survives the complete path without losing required semantics.

Do not confuse an implementation detail with the design hypothesis. Focus on what could invalidate the architecture or approach.

### Critical boundary

The point where assumptions meet reality and where isolated code reasoning is weakest.

A critical boundary may be an external process, service, device, database, filesystem, browser, network, scheduler, runtime, protocol, model, sensor, operating system facility, or interaction between multiple production components.

### Probe

The **smallest safe executable experiment with enough fidelity to falsify the design hypothesis**.

A good probe:

- uses the actual production code path wherever practical;
- crosses the critical boundary rather than mocking it away;
- produces directly observable evidence;
- is narrow enough to run and diagnose quickly;
- has explicit pass/fail criteria;
- can fail even when the local implementation looks internally consistent.

A probe is not required to be a full end-to-end test. It must be high enough fidelity to answer the important design question.

### Hardening

Work performed after viability has evidence, including:

- unit tests;
- edge-case coverage;
- regression tests;
- fault handling;
- static analysis;
- type checking beyond basic execution needs;
- security analysis;
- linting and formatting;
- broader integration/system testing;
- maintainability refactoring.

---

# Required workflow

## 1. Identify what must be proven

Before substantial implementation, identify:

1. **Desired observable behavior** — what should an external observer see?
2. **Critical design hypothesis** — what assumption is most capable of invalidating the approach?
3. **Critical boundary** — where can reality disagree with the code's assumptions?
4. **Probe** — what is the cheapest safe run that can falsify the hypothesis?
5. **Pass signal** — what concrete observation will count as success?
6. **Failure signal** — what observation would invalidate or materially challenge the design?

Do not begin by asking, "What unit tests should I write?"

Begin by asking, **"What would prove or disprove that this approach can work?"**

---

## 2. Establish a baseline when modifying an existing system

For a bug fix, refactor, or behavioral change, obtain a lightweight baseline when feasible.

Prefer:

- reproducing the current failure;
- running the existing behavior through its public or integrated path;
- capturing relevant output, logs, traces, measurements, or state.

Do not spend significant effort repairing unrelated test, lint, or style failures just to create a perfect baseline. Record unrelated failures and isolate them if possible.

---

## 3. Implement the thinnest viable slice

Write only enough production code to exercise the critical hypothesis.

Prefer a **vertical slice** over completing every layer separately.

During this stage:

- avoid speculative abstractions;
- avoid exhaustive error handling that is not required for a safe probe;
- avoid large test scaffolds;
- add observability where it materially improves diagnosis;
- preserve safety limits and reversible behavior.

### Mandatory stop point

**Once the critical path is runnable, stop implementation and run the probe before continuing.**

Do not continue filling in secondary features merely because the code is already open.

---

## 4. Run the first design probe

Run the smallest meaningful experiment against the highest-fidelity practical boundary.

The probe should normally exercise:

**input -> production path -> critical boundary -> observable result**

Do not mock the exact dependency or behavior whose compatibility is the design question.

Mocks may stand in for **non-critical** dependencies when doing so keeps the probe small without hiding the assumption being tested.

Record enough evidence to distinguish:

- design failure;
- implementation bug;
- configuration/environment problem;
- dependency behavior;
- instrumentation error.

---

## 5. Enforce the viability gate

### If the probe passes

Proceed to broader representative verification and hardening.

A pass means the design now has runtime evidence. It does not mean the implementation is complete or robust.

### If the probe fails

**Do not expand unit tests, coverage, lint cleanup, or static-analysis work for the failed design.**

Instead:

1. inspect the observed failure;
2. decide whether the failure challenges the design hypothesis or only the implementation;
3. change the design or minimum implementation needed;
4. rerun the probe;
5. repeat until it passes or the approach is rejected.

Do not create mocks that reproduce the hoped-for behavior and then treat the resulting green tests as evidence that the failed real boundary works.

### If the probe cannot be run

Use the highest-fidelity safe substitute available, but explicitly mark the design as **unverified at the missing boundary**.

When execution is blocked by unavailable hardware, credentials, services, datasets, permissions, environments, or other dependencies:

- create the smallest runnable harness or probe instructions that will exercise the missing boundary;
- verify everything that can actually be verified;
- state exactly what remains unverified;
- do not claim success based solely on unit tests or static analysis.

---

# Choosing the right probe

Prefer the **highest-information probe that is safe, practical, and fast enough to run early**.

A useful fidelity order is:

1. controlled run in the real or production-like environment;
2. actual critical dependency through the production adapter/interface;
3. faithful sandbox, emulator, simulator, loopback, or test instance;
4. integration harness spanning the relevant production components;
5. isolated execution of the public component API;
6. unit-level execution.

Do not mechanically insist on the highest level. Choose the **lowest-cost level that can genuinely falsify the critical assumption**.

A lower-fidelity probe is inadequate if it can pass while the real design is broken.

### A unit test may count as the design probe only when

- the unit itself is effectively the complete relevant system boundary; or
- there is no meaningful integration/runtime boundary beyond the unit; and
- the test executes behavior rather than merely reproducing the implementation's internal assumptions.

For systems with meaningful integration, I/O, timing, state, external dependencies, or physical behavior, isolated unit tests do not satisfy the viability gate.

---

# After viability: harden deliberately

Once the design probe passes, improve confidence systematically.

## 6. Run representative integration/system checks

Before exhaustive unit work, exercise a small number of representative scenarios through the real path.

Prefer scenarios that cover:

- the normal path;
- the most important realistic failure path;
- any boundary condition discovered during probing.

Do not turn this into an exhaustive matrix unless the task requires it.

---

## 7. Add unit tests for local contracts and edge conditions

Use unit tests to protect behavior that benefits from fast deterministic isolation.

Good unit-test targets include:

- invariants;
- boundary values;
- malformed inputs;
- state transitions;
- retry/backoff rules;
- error mapping;
- numerical or algorithmic edge cases;
- regression cases discovered by real failures;
- deterministic branches that would be expensive to reproduce through the full system.

### Unit-test discipline

Tests should assert **behavioral contracts**, not mirror implementation structure.

Avoid:

- tests coupled to private call order without a behavioral reason;
- mocking every collaborator until no production interaction remains;
- reproducing the production algorithm inside the assertion;
- generating tests from the current implementation and calling that independent verification;
- changing production behavior solely to satisfy a brittle test unless the test represents a real requirement;
- optimizing for coverage percentage as an end in itself.

Coverage is a diagnostic signal, not the definition of correctness.

If a refactor preserves externally required behavior but breaks many tests, inspect whether the tests are over-coupled before changing production code to satisfy them.

---

## 8. Run static analysis and code-quality checks

After the behavior has evidence, run the repository's relevant quality checks, such as:

- compiler diagnostics;
- type checking;
- linters;
- static analyzers;
- security scanners;
- formatting;
- dependency checks.

Fix meaningful findings.

Do not let cosmetic cleanup obscure behavioral changes or consume disproportionate effort.

Static analysis can detect real defects, but a clean result is not evidence that the runtime design works.

---

## 9. Re-run the real probe after hardening

After unit tests, refactoring, and static-analysis fixes, repeat the important runtime probe.

This closes the loop:

**proved viable -> hardened -> proved still viable**

If hardening breaks the probe, the runtime evidence wins. Diagnose the change rather than assuming the test suite is authoritative.

---

# Safety and destructive systems

For code that can create physical, financial, privacy, security, availability, or irreversible consequences, **probe-first does not mean reckless live testing**.

Before executing:

- use dry-run/read-only modes where possible;
- constrain outputs, rate, range, speed, power, permissions, or affected data;
- prefer sandbox, simulator, bench, staging, canary, or isolated resources;
- include emergency-stop/rollback behavior when appropriate;
- run targeted pre-flight safety checks;
- instrument the system so unexpected behavior is visible quickly.

In these cases, safety work is part of making the probe valid.

A simulation can establish preliminary evidence, but do not silently promote simulated success into real-world verification when the unsimulated boundary remains important.

---

# Special cases

## Bug fixes

Preferred sequence:

**reproduce through the meaningful path -> minimal fix -> rerun reproduction -> add targeted regression tests -> static checks -> rerun meaningful path**

A unit regression test is valuable after the bug has been demonstrated and fixed, but it should not replace reproduction at the level where the bug actually manifested when that level is accessible.

## Refactors

Capture representative behavior before the refactor, preserve it through the change, then rerun the same probe afterward.

Do not spend the entire task strengthening unit tests around the old implementation before demonstrating that the relevant system behavior can be observed.

## Pure deterministic libraries or algorithms

A direct invocation of the public API may be the highest-fidelity meaningful probe. In this case, unit or property tests may legitimately provide strong evidence.

Still distinguish:

- representative viability checks; from
- exhaustive edge-case hardening.

## Performance-sensitive work

The design probe should include a measurement against a representative workload.

Do not infer performance viability from code shape, micro-unit tests, or static analysis alone.

## Nondeterministic behavior

Use repeated representative probes and define a tolerance or acceptance criterion before interpreting the result.

Do not "fix" nondeterminism by mocking away the behavior that matters.

---

# Common failure modes

## Test theater

**Pattern:** many green tests exist, but the actual system path has never been executed.

**Response:** stop adding tests and run the design probe.

## Mock bubble

**Pattern:** the component under test is surrounded by mocks that encode the same assumptions as the implementation.

**Response:** replace the critical mock with the real boundary, a sandbox, emulator, or higher-fidelity contract probe.

## Coverage chasing

**Pattern:** effort is driven by uncovered lines rather than failure risk.

**Response:** choose tests from meaningful contracts, edge conditions, and observed risks. Treat coverage as secondary information.

## Polish before proof

**Pattern:** formatting, linting, typing, abstractions, and documentation are perfected before the design has run.

**Response:** do only what is required to make the critical path safe and executable, then probe.

## Architecture by imagination

**Pattern:** multiple layers and extensibility mechanisms are completed before any vertical slice crosses the real boundary.

**Response:** cut a thin path through the architecture and validate the riskiest assumption first.

## Green therefore done

**Pattern:** the agent claims the feature works because unit tests and static checks pass.

**Response:** require runtime evidence at the meaningful boundary before making a "works" claim.

## Unavailable environment therefore unit tests are enough

**Pattern:** an external boundary cannot be accessed, so the agent substitutes mocks and reports success.

**Response:** report partial verification and identify the exact unresolved boundary. Provide or create the probe needed to verify it later.

---

# Evidence hierarchy for claims

Use precise language based on the evidence obtained.

### Verified

Use only when the critical runtime path was actually exercised with sufficient fidelity and produced the required observable result.

### Partially verified

Use when important production code was exercised but one or more meaningful boundaries were substituted, simulated, or unavailable.

### Unverified

Use when only source inspection, compilation, static analysis, mocks, or isolated tests were available for a design whose important behavior depends on a richer boundary.

Never upgrade the claim merely because the test suite is large.

---

# Completion report

When reporting completion of implementation work, include concise evidence in this order:

**Status:** Verified / Partially verified / Unverified

**Design hypothesis:**  
The important assumption that had to hold.

**Probe performed:**  
The executable action, command, scenario, or procedure used to test it.

**Observed evidence:**  
The concrete result that supported or falsified the hypothesis.

**Hardening performed:**  
Relevant unit/regression tests, static analysis, lint/type checks, and other quality work completed after viability.

**Remaining verification gaps:**  
Any boundary not actually exercised or any limitation of the evidence.

Do not hide a missing real-world probe inside a long list of passing unit tests.

---

# Final checklist

Before declaring implementation complete, confirm:

- [ ] The externally observable success criterion is clear.
- [ ] The riskiest design hypothesis was identified.
- [ ] A probe capable of falsifying that hypothesis was defined.
- [ ] The critical path was made runnable before broad polish.
- [ ] The probe actually ran, or the missing boundary is explicitly marked unverified.
- [ ] Critical behavior was not mocked away in the evidence used to claim viability.
- [ ] Probe failure caused design/implementation iteration rather than test-suite expansion.
- [ ] Unit tests were added for useful local contracts, edge conditions, and regressions rather than coverage theater.
- [ ] Static analysis and code-quality checks were performed at the appropriate stage.
- [ ] The meaningful runtime probe was repeated after hardening.
- [ ] The final claim matches the strength of the evidence.
