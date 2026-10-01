# Architect review 02 execution

Current work follows [review 02](../architect_review_02/00_START_HERE.md) and the

[recovery amendment](../docs/09_ESTIMATOR_RECOVERY.md). Stage 1 remains IN_PROGRESS;

yaw is UNQUALIFIED, Candidate14 NONDEPLOYABLE. Both axes/configurations and physical

3a/production 3b remain mandatory. No station action, settings mutation or deployment

has occurred during this response. Evidence is under ignored

`run/adr0022-stage2/architect-review-response-02/`.

## Current checkpoint - cycle 09

The frozen fitted controllers now drive a separate supplied synthetic plant. All 18 signed 5°/s cases complete and pass the original motion limits and independent own-command checks, with 26,172 complete native readbacks. New noise draws use the same consumed reference design and supplied truth point; lifetime freshness and wider generalization are not claimed. The controller and feedforward retain their fitted models while the simulator supplies actual plant/filter/source timing. All 19 retained default arrays replay exactly; 15 forecast contracts pass in 1.757 s.

The 36 signed 0.5°/1°/5° position steps fault when HOLD requests a second START beyond the unchanged single-attempt guard. Full position quality is NOT_RUN in all 36 truncated windows. Independent forward checks and 20,400 native readbacks pass. The first −0.5° case enters MOVE at 2.180 s with actual velocity −0.05065 rad/s against −0.02224 reference, overshoots, corrects in REVERSE and stops beyond the target. At HOLD at 2.560 s it requests the rejected second start. Latent position is −0.76936° for a −0.5° target. This identifies a startup/braking interaction without establishing a repair or global infeasibility.

Separately planned two-leg reversals complete all 12 cases, pass independent forward, accepted-current dose and fixed two-second stop checks, and produce 19,224 complete native readbacks. Full reversal tracking quality remains NOT_RUN because no complete predicate is declared. These results do not qualify general corrective starts or target tracking.

The chronological fitted-model information diagnostic is recorded as `FITTED_EXPECTED_INFORMATION_NUMERICAL_PASS`. Original numerical failures and bounded refinements are retained before any admitted inverse. The 31.25→15.625 µs pair passes the unchanged 1% derivative/bin-information gates; rank is 10, normalized condition 889.34 and inverse identity error 1.30e−11. The source model was fitted at 125 µs and has a local score norm of 12.985 at the finer resolution, so this inverse cannot be treated as a calibrated fitted confidence set. The independent saved-data peer confirms the joint matrices, units, score and retained failure sequence; its inverse identity error is 1.48e−11. Uncertainty remains UNKNOWN. The small-step transition repair, supported uncertainty, configuration/axis coverage and physical 3a/production 3b remain open. No station action or deployment occurred; the goal remains active.

Compact architect update: `run/adr0022-stage2/architect-review-response-02/ADR-002.2-review02-cycle09-final-evidence.zip` (40,623 bytes, 27 data/prose records; readback PASS). No source, binaries, hashes, bulk captures or repeated historical archives.

## Previous checkpoint - cycle 08

The fitted Coulomb family now passes its bounded speed-control development check. With each fitted model's own WN 2 rad/s, zeta 1.5 gains, the original 20/40/60/80 mA startup grid completes 72 cases. The first three amounts pass all 18 original and revised quality sets; 80 mA passes three. Selecting the minimum amount that passes all nine cases of each sign yields 20/20 mA. That pair is frozen and actually replayed in 18 separate cases; all pass the original limits. Across all 90 trajectories, independent forward checks and 130,860 complete native readbacks pass. Six protocol checks pass.

For the separate combined replay, detrended position jitter is 0.031–0.102°, startup is 150–185 ms, and START-interval command dose is 0.00307–0.00560 A²s against 0.026 A²s. Actual native START totals are −0.164/+0.204 A after the declared current map. The selected local curve also meets the original 50°/6 dB margin requirement: worst phase margins are 69.673–69.675° and gain margins 16.59–16.60 dB. The owner-approved 45°/0.16° comparison remains recorded; it is not needed for this current plateau result.

This repairs the consumed fitted-plant departure failure, not the complete controller. Signed position steps and planned reversals now run with the frozen 20/20 mA setting. Unknown-true-plant forecasts, supported joint uncertainty, changed-configuration identification, full axis/configuration coverage and physical 3a/production 3b remain open. No station action or deployment occurred. The goal remains active.

## Previous checkpoint - cycle 07

The fresh fitted-model bridge is complete. All three fitted Coulomb models now synthesize their own gains through the existing family dynamics rather than borrowing the Stribeck candidate. Both signed native tangent prefixes pass. All18 full plateaux complete and pass independent forward comparison and complete native readback (26,172 comparisons). None passes motion quality under either the original or revised contract: jitter0.251–0.358°, range0.312–0.438°, and gyro-band RMS fails nine positive-direction cases. Startup, speed, stop drift, current, slew and dose predicates pass. The first retained pristine traces stay in SLIDE/MOVE with constant FF; curved departure transients remain inside the fixed quality window. Model uncertainty and separate true-plant control validation remain unqualified.

The bounded position-response search evaluates72 exact local candidates, with16 feasible. Its three preselected full-case candidates pass7/12,0/12 and5/12 revised motion sets; all36 conditional numerical checks pass. Thirteen locally feasible candidates remain untested in full motion. This is a bounded selection failure, not a proof of global infeasibility. No further threshold, reference-window or guard change was made.

Configuration lifecycle now passes its native interface probe: one runtime owner, one uninterrupted sticking acquisition,1,205 actual successful10mA receipts and five native generations. Four declared PAYLOAD+/− and FRICTION+/− changes inhibit before another command and preserve the accepted current/time. RETURN keeps the same fitted revision and requires an explicit synthetic stationary reset; prior-generation tokens reject. Delayed gyro sources predating reset stay invalid until a new source arrives. Hidden inertia/friction changes produce exactly zero residual at rest, so informative motion is required for detection/identification. Changed-configuration fitting, tracking qualification and physical reentry remain NOT_RUN.

The subsequent bounded nonlinear screen completes54/54 trajectories and independent comparisons, with78,516 complete native readbacks. Curves WN4/zeta1.5,WN2/zeta1 andWN2/zeta1.5 pass6/18,0/18 and9/18 owner quality sets (original3/18,0/18,9/18). All are rejected;16 other local-feasible curves remain untested. The best curve's nine negative cases pass, while its positive cases expose a remaining startup/feedback transient. Family-specific START selection now runs inside the original20/40/60/80mA grid; its first positive60mA noisy case passes, with full72-case selection/combined18 verification still in progress.

The actual-gravity coupled interface passes for a separate near-balanced supplied COM[.001,.01,.0005]m geometry. The original off-centre supplied geometry requires−9.659A pitch holding and is rejected before motion under the existing0.35A cap. Near-balanced known-equilibrium prehistory supports a2s stationary hold and simultaneous move plus full2s stop, with maximum0.02819A current/1.34128A/s slew and zero causal replay. A retained2.5ms filtered-gyro numerical failure passes at the first predeclared1.25ms refinement (6.25218e-7 against1e-6). Independent new-COM gravity/cross-inertia formulas pass; the sampled gravity-table error is distinguished from its analytic continuous upright interpolation remainder6.42651e-6A. These supplied-map/noise-free/frictionless results do not qualify physical loaded pitch or stopping.

All36 subsequent focused tests pass in2.701s; seven loaded contracts pass after the analytic-bound correction, and six nonlinear-selection protocol checks pass. These overlap earlier suites and are not added into a total. Supported uncertainty, changed-configuration identification, full motor/control coverage and physical3a/production3b remain required. No station action or deployment occurred; the goal remains active.

Compact completed evidence: `run/adr0022-stage2/architect-review-response-02/ADR-002.2-review02-cycle07-evidence.zip` (77,384bytes,42 data/prose records; readback PASS). No source, binaries, hashes or bulk captures. Prior archives remain unchanged.

## Previous checkpoint - cycle 06

The owner permits reasonable motor-performance relaxation because target tracking runs above the motor loop. A separate comparison uses45° local phase margin (previously50°) and0.16° detrended P95−P5 position jitter (previously0.15°, a6.7% change). Gain margin remains6dB. Current, slew, travel, fault,200ms START, body evidence and dose guards remain unchanged, as do stop/step/speed/range/gyro metrics and estimator recovery limits. Original verdicts and defaults remain recorded.

The fresh joint ten-coordinate exact-bin estimator passes all three noise seeds10103/10301/10613 from the original nontruth initializer. It takes27/27/29 evaluations, with maximum relative parameter error0.943926% against the original5% noisy limit. TRAIN3/3, twelve new held-out predictions (six pristine/six noisy), six historical regressions and independent forward numerics pass. All three vectors and six native plant predictions were frozen before generating any validation truth or noise. This is one new720s TRAIN design at one supplied parameter point with known Gaussian-before-quantization noise, gauges, static thresholds and clocks. `ftol` termination and large optimizer optimality are reported separately; global/statistical/physical scope is not established.

The45° synthesis selects WN2,zeta1.5,Kp0.841143576,Ki0.4,Kpos0.4,Kaw3. Signed margins are45.47°/9.88dB and56.40°/12.55dB; it fails the old50° requirement. The fixed symmetric20/40/60/80mA START grid has no candidate passing all six plateaux. A separately replayed negative60/positive80mA pair passes all six under the revised jitter limit and five under the original. Only the negative5° position step passes; the other five signed0.5/1/5° cases overshoot into a guarded corrective START. All conditional forward/causal comparisons pass. The controller remains unqualified.

An optional immutable two-leg program permits a genuinely planned opposite departure after accepted MOVE and causal rest, with one persistent actual-ACK START-interval command-dose ledger capped at0.026A²s. Ten faults, actual delayed/failed receipts, upstream rejection and final partial-control tails pass; six legacy forecasts remain exactly equivalent. Separate revised-controller reversal probes complete only positive pristine (two MOVE admissions, dose0.01285A²s, final2s drift0); the other three fault on later corrective HOLD. No retry/rearm or physical cutoff is inferred. Full reversal-quality qualification remains NOT_RUN.

The family asset/runtime and family information-selector bridges now pass their bounded native probes and tests. The former reuses fitted Coulomb parameters and validates full native readback and serialized rebinding; UNKNOWN uncertainty prevents qualification. The latter preserves existing templates, occupancy objective, deterministic selector and finite3-case/2-round policy, with explicit unsupported physical/quantized/correlated boundaries. The fresh fitted Coulomb models are now being connected to their own synthesis and control forecasts; Stribeck gains are not reused for them.

All149 current focused tests pass in7.297s on the canonical ABI4 core, supplemented by independent runtime and receipt audits. Both axes, supported uncertainty, loaded pitch/coupled synthesis, configuration lifecycle and physical3a/production3b remain required. No station action or deployment occurred. The goal remains active and authorized offline work continues without an architect approval round.

Compact evidence is frozen in `run/adr0022-stage2/architect-review-response-02/ADR-002.2-review02-cycle06-evidence.zip` (42,022bytes,25 JSON/CSV/Markdown records; readback PASS). It contains no source, binaries, hashes or bulk captures. Earlier archives remain unchanged.

## Previous checkpoint - cycle 05

The exact encoder-bin likelihood passes its numerical prerequisite: independent quadrature agrees within7.04e-12 and all50 physical-mean derivative checks pass. The single consumed7103 development fit now converges in25 evaluations/526 residual calls, with all ten recovery, TRAIN and six consumed-regression gates passing. Maximum parameter error is1.78925%; original2% pristine/5% noisy limits remain unchanged. Fresh verification is not yet complete. A genuinely different720s TRAIN information design passes before observations (gyro-delay SE46.4629us against53.75us); its fresh protocol and independent audit continue. Production objective defaults remain unchanged.

Automatic dynamic synthesis retains both failed curves and selects one third-curve local candidate: WN0.5,zeta4,Kp0.641143576,Ki0.025,Kpos0.1,Kaw3. Both signed5deg/s local margins exceed50deg/6dB. Positive native-prefix agreement passes; bounded negative cold-START warmup does not establish the frozen sliding point. All six complete plateau forecasts fault, leaving full quality/stop windows NOT_RUN. The first successful START must not be described as failed merely because a later entry exhausts the lifetime count.

The new complete-manoeuvre runner applies the same selected candidate through the actual public MotorFF/core. Pristine and noisy stationary cases pass. Signed0.5/1/5deg steps overshoot and request corrective START; both smooth reversals request a planned opposite leg. Those eight cases censor at the unchanged one-entry guard. Two earlier0.5deg reference-phase midpoint failures are retained and corrected by choosing the phase from signed acceleration, without clipping q/v/a or relaxing admission. Conditional forward numerics and causal sensor replay pass throughout; motion qualification does not.

The coupled two-axis native parent now passes nonunit gain/bias maps, offmesh transport/filter/source timing, an independent piecewise filter oracle, nine fault cases and six focused tests. A successful yaw receipt survives a later failed pitch ACK. The retained2.5ms integration failure led to1.25ms refinement; filtered-gyro half-step error3.98049e-7 passes the unchanged1e-6 gate. This is a supplied frictionless, gravity-free interface proof. Gravity-loaded pitch, coupled identification/synthesis, full manoeuvres, watchdog/physical stop and production adoption remain required.

All108 tests covering the changed forecast, sampled, synthesis, coupled, FF/fault and nuisance components pass in3.619s on the canonical ABI4 core. A family asset/runtime bridge and the existing information selector's family-aware bridge are in progress. Both axes, configuration reuse, supported uncertainty and physical3a/production3b remain part of the original goal. No station action or deployment occurred. No architect approval round is needed for this continuing offline work.

## Previous checkpoint — cycle 04

The native sensor-query repair removes reporting-time anchors from the mechanical

mesh. Reporting-delay perturbations now leave q/current identical in all15 probes;

independent forward gates pass and all6 original generator datasets are unchanged.

At the retained near-fit candidate,18 independent transport time-shift derivative

checks pass a predeclared1e-3 relative budget without a transport-source repair.

The joint ten-coordinate pristine fit now passes from the original nontruth start

with explicitly declared linear loss:22 evaluations, maximum relative recovery

error0.00063555%, and unchanged forward/TRAIN/historical/2% recovery gates. The

Huber120 and staged116 failures remain retained. This establishes a loss-sensitive

noiseless result, not noisy/global recovery; no production objective default changed.

The repaired local information forecast gives gyro-delay SE68.7336 microseconds

against71.6667, conditional on the declared supplied model. Fresh noise verification

is next after freezing its method.

FF now runs at the sole existing native FF insertion using the exact posterior

from that tick and its coherent reference. Constant callback equivalence and

exception, inhibition, re-entry, stale/future ACK and lifetime guards pass. Synthetic

first-order actuation supports an explicit steady-state reference policy retaining

full causal lag/history; no dynamic inverse is inferred. A bounded directional

START policy still fails the original200-ms startup gate:270/250ms. Even optimistic

maximum existing slew to the existing cap is too late after the current START

entry. Planned departure intent and a declared20/40/60/80mA compensation grid
select60mA for the pristine signed startup slices. Explicit BRAKING now uses the
existing STOP owner, preventing a same-cycle repeat START. Both full pristine
runs complete but fail their original quality gates; a noisy development run
retains its valid200-ms START timeout. No policy is qualified.

Ten causal feedback forecast checks pass native/independent forward gates; nine

reach their horizon and one pristine run latches START timeout. Four finish in

START, so horizon completion is not successful start recovery. Six positive and

negative5-deg/s plateau runs all fail the unchanged motion-quality gates. The

original metrics use fixed first-sample stop drift, not position peak-to-peak;

older mislabeled span records remain frozen with an explicit correction. All120

retained numerical comparisons remeasure to within3.11e-16.

The sampled source-clock repair passes14/14 native-prefix cases, maximum error

3.70e-8 against1e-7, and17 focused tests. Availability age and physical signal

delay now give the actual source age used by value history, covariance/freshness

and native timestamps; pre-acquisition samples are excluded. Ten Coulomb margin

cases pass; four Stribeck cases remain locally stable but fail50-degree phase

margin. No eigenmode is discarded. A new additive automatic selected-family

synthesis selects4rad/s from its declared four-point curve, with both-sign
54.345-degree/12.949-dB margins and passing native prefixes. Two0.6-second
lower-level native callback forecasts pass numerical/causal checks; public
MotorFF moving reset remains unsupported. Full nonlinear manoeuvres remain
required, and legacy/default gains are unchanged.

All192 focused tests pass on the final canonical core in17.626s; core/full
firmware builds,399-link documentation check and whitespace check pass. Physical3a/3b remain NOT_RUN, with no station action.

The first coupled two-axis native probe also passes its bounded pristine interface:
600 shared vector FF/accepted-command pairs, one parent plant, nonzero inertia
coupling, causal replay0 and halved-step difference7.77e-16. Fault/domain/filter
hardening continues; the192-test checkpoint precedes that new runner.

Compact cycle04 evidence is saved at
`run/adr0022-stage2/architect-review-response-02/ADR-002.2-review02-cycle04-evidence.zip`
(489,152bytes). Its55 selected records contain no source, binaries or hashes.
Twelve original metrics recompute from the exported record within2.26e-17; the
exported independent/native numerical pair passes the original numerical gates.
Full training captures and repeated predictions remain local. The goal stays
active: exact encoder likelihood, coupled native hardening and broader controller
qualification continue without an architect approval round.

## Earlier execution history

## Executed and retained

- WP0: 11 observation runs, 88 predictions and 470 numeric comparisons reproduce;

  all eight audit tests pass. Digest generation was omitted per owner instruction.

  Source/library paths, dirty revision, build flags and environment are recorded in

  `reproduction.json`; existing evidence remains intact.

- WP1: all original cases reproduce. Pristine passes; noisy 17/41/83 already meet

  the 5% parameter gate, all exhaust 120 evaluations, and 41 fails two cross-run

  position gates. Independent gate flags preserve that distinction.

- WP2: absolute central derivatives repair the original failure. A subsequent

  training-only moving-integral initializer escapes a genuine hybrid grazing basin

  exposed by quintic reversal seed 227. All six original cases, all nine noisy

  cross-run predictions and all nine consumed development cases now pass. Fresh

  seeds 1019/1237/1423 on three new reference cases pass nine fits and all 81

  cross-run predictions. Maximum angle RMS is 0.000329392 rad against the unchanged

  0.000802313-rad gate; worst parameter error is 0.3955% against 5%. Native physics,

  loss, noise, bounds, latent initial state and solver budget remain unchanged.

  The data-only parameter initialization is an explicit procedure change.

- WP3: the owner confirms CAN current-only mode; PWM is excluded. All 25 archived

  runs use 0x1FE commands and standard 0x205/DLC8 feedback near 1 kHz. Reporting

  correspondence is close to unity in protocol A-equivalent units. Exact applied

  firmware/settings and physical Iq/torque/latency remain unverified; completed zero

  requests do not qualify physical stopping or an independent cutoff.

- WP4 preparation: all 88 old whole-run predictions have baseline, displacement,

  transition, event-horizon and exact objective diagnostics. There are 19 no-motion

  contradictions. The two largest training runs contribute 50.94–61.53% of cost;

  this does not by itself prove a weighting defect. Four clean crossed load/current-

  measurement plans are prepared. The automatic bounded threshold grid evaluates

  640 uninterrupted TRAIN predictions: finite threshold changes affect outcomes

  despite flat local derivatives, but none of 80 candidates passes all training

  gates. Cost improvements of 2.313% and 0.461% still suppress small-probe movement.

  These were preparation results. The subsequent four-cell algebraic Coulomb

  diagnostic is recorded below; full crossed qualification remains incomplete.

- WP6 independent software work: stalled/censored starts now latch an abort before

  further active command issue; command tokens persist through resets to reject stale

  ACKs. A bounded synthetic algebraic FF adapter replaces one FF term in the existing

  native PI/limiter/accepted-TX path. It validates reference time/frame/freshness,

  causal state, model/configuration support and applied native settings; invalid

  inputs latch inhibition. Dynamic/delayed actuation, shared posterior-state FF

  equivalence, model-switch bumpless transfer and physical/production qualification

  remain unsupported. All original generator arrays remain exactly equal.

  These are software inhibition checks, not a physical stop certificate.

The new reversal failure is a genuine hybrid grazing event: tiny parameter changes

move a zero crossing across an input change, selecting different sticking histories.

The independent analytic oracle reproduces the finite trajectory jump. The verified

training-only moving-dynamics initializer finds a different basin; the actual hybrid

model remains the final acceptance model. The first seed-131/227/419 final batch became

development when its failure informed repair. Later seed-593/701/887 verification used

fresh noise on development-consumed waveforms; the final seed-1019/1237/1423 batch also

uses fresh waveform parameters. These partitions remain separately retained.

All 56 focused Python/native tests and four direct C++ fault scenarios pass. The

full workstation suite, excluding retained_homing, passes 82/83; its existing

`test_mixed_station_config` failure expects yaw/pitch acceleration equality despite

the retained yaw-30/pitch-60 configuration. Document-tree and whitespace checks pass.

Forwardable evidence: `run/adr0022-stage2/architect-review-response-02/ADR-002.2-review02-model-update.zip`

(4.16 MB). It contains the progress/decision records, nine new native observation

datasets, all 81 final metric comparisons, 25 raw limiting prediction pairs and the

new reversal failure/repair evidence. No source, binary, digest, old archive or bulk

profile capture is included. Full predictions remain locally retained; raw export

selection is explicit in DATA_FORMAT.json.

## Promotion and remaining dependencies

No physical model, controller gains or prospective physical trial was promoted.

The historical final holdout remains reserved and unexamined in this response.

Broader static/Stribeck, actuator/filter/timing and structure-selection estimator

fixtures remain required; the four-coordinate known-nuisance result does not qualify them.

Further physical work needs the applicable authorization and setup-specific verified

stop/limits. Missing physical facts do not block continuing the offline work queue.

Concise decision records and native prediction arrays retain the evidence needed

to distinguish numerical convergence, recovery, prediction and physical qualification.

## Continued execution cycle 02

The independent first-order/Coulomb and algebraic/Stribeck forward fixtures exposed

two current-measurement numerical errors: approximate filter-cascade propagation

and interpolation at fractional delayed sample times. Exact propagation and actual

sample-time integration repair both. Current RMS falls from roughly 2.5–4.5e-6 A

to below 1.2e-14 A under the unchanged 1e-9-A numerical gate. Independent motion,

filter and event refinement pass. The original six generator datasets remain

exactly equal; all six original fits and nine noisy cross predictions still pass.

Automatic threshold inference now requires independent actual-rest support.

Review reproduced a quiet-creep counterexample: 0.012 rad/s sliding passed the

observation rest bands and falsely implied a static threshold below 0.16 A.

The repaired method retains that trial as ambiguous and adds no static inequality.

Synthetic certificates cannot establish measured rest. Fresh seeds 2017/2371/2801

and fresh input cases pass 27 inner fits and nine predictions; maximum angle RMS

is 0.000258759 rad and moving-parameter error 0.07520%. Directional static totals

remain conditional intervals, negative [0.1362,0.1442) and positive [0.1762,0.1842)

A-equivalent; commands inside a bracket are explicitly ambiguous.

The two-direction Stribeck speed-scale method passes fresh pristine/noisy TRAIN

and whole-run SELECTION verification, with worst noisy recovery error 0.206%.

Its mechanics/static/current/sensor nuisances are known. Joint six-coordinate

mechanics plus Stribeck fitting fails pristine recovery under both the original

and one staged TRAIN-only strategy, each retaining the 120-evaluation budget.

That broader method remains NOT_QUALIFIED; final joint tests were not opened.

Six unknown timing/filter coordinates are now verified together with known

mechanics, Coulomb friction and constant load under prescribed synthetic inputs.

The retained four-second gyro-pair test passed trajectory/termination gates but

failed separate parameter recovery: correlation between delay and filter time

constant left insufficient information. A prior-information design selected richer

training and declared a precision target before generation. Fresh 210-second

TRAIN seeds 4001/4507/4801 all pass with worst parameter error 3.47315%; the

optimizer takes 22/23/27 evaluations. Separate actuator/current final cases pass

24/24 and fresh gyro-pair cases pass 3/3. No fitter, quality limit or solver budget

changed. Unknown mechanics plus nuisance dynamics and feedback-generated nuisance

identification remain NOT_RUN. Local information forecasts are design diagnostics,

not calibrated physical uncertainty.

The fair physical diagnostic now runs four algebraic Coulomb cells with identical

freedoms except the declared constant/affine load and fixed/free current-observation

factors. Every optimizer terminates by xtol; every cell fails all eight TRAIN and

three SELECTION trajectory gates and retains two no-motion contradictions.

Affine load reduces TRAIN cost by 3.2–3.4%; free current-observation dynamics reduce

it by 0.035–0.269%. Neither isolated change resolves prediction failure. All 44

continuous predictions and fitted initial states are retained; 132 independently

remeasured channel metrics agree exactly. Historical HOLDOUT observations were

not loaded. These results remove the old comparison confound for this bounded

diagnostic, without universally falsifying structures or qualifying pooled context.

Structured configuration facts now feed whole-run fitting and the actual native

FF adapter. Conflicting known facts prevent pooling, including when the first run

has unknown context. A changed payload beneath an unchanged label latches native

inhibition. Rollback needs snapshot-specific supported context and response evidence;

global PASS and Boolean/numeric value aliasing cannot authorize reuse. Missing

historical facts remain visible diagnostic assumptions.

The runnable two-axis rigid parent includes geometry-dependent coupled inertia,

Coriolis and gravity, supplied causal cable/friction loads and explicit actuator

mapping. Independent analytic, frame, coupling and energy probes pass. Physical

parent parameters, native controller coupling and production equivalence remain

NOT_RUN. Equal mass relocated off-axis changes inertia and invalidates parameters.

All new evidence remains under `run/adr0022-stage2/architect-review-response-02/`;

the current mathematical scope does not qualify physical yaw, pitch or Candidate14.

The next unresolved estimator branch is TRAIN-only joint Stribeck initialization

followed by unchanged hybrid acceptance. Current/sensor timing and configuration

qualification remain distinct from solver termination and prediction scores.

The rebuilt core and full firmware pass compilation; 101 focused tests pass.

Workstation CTest passes 81/83 on its first run: the existing yaw-30/pitch-60

configuration assertion still fails, and the UDP overflow test transiently rejects

a receive-clock discontinuity. The capture test passes its targeted retry with

the clock guard unchanged. Neither station configuration nor capture guards were

altered to obtain a pass. Document-tree and whitespace checks pass.

The Stribeck evidence originally declared 5% recovery for both observation classes.

A preserved-record audit reapplies the original 2% pristine and 5% noisy limits:

both narrow pristine fits pass and both joint pristine fits still fail. Future

execution uses the original class-specific limits; frozen records remain intact.

This cycle's compact handoff is

`run/adr0022-stage2/architect-review-response-02/ADR-002.2-review02-cycle02-evidence.zip`

(1.34 MB). It includes concise decisions, all final metric summaries and 11 selected

raw exports. Five observation/prediction pairs independently reproduce their saved

channel RMS values exactly. The ZIP contains no source, binaries, digests, logs,

previous archive or bulk 210-second training arrays. Raw export selection and scope

are explicit in DATA_FORMAT.json; the earlier 4.16-MB package is the previous cycle.

## Continued execution cycle 03

Joint moving mechanics/Stribeck recovery now passes within its known-nuisance

scope. TRAIN-only integrated force balance initializes four moving coordinates

and two speed scales. The retained pristine case passes recovery/prediction at

120 evaluations but fails termination; documented coherent cost descent supports

one checkpoint continuation under architect section3.4. Five more evaluations

converge. The 49 initializer rollouts and diagnostic work are counted separately.

The frozen method then passes fresh inputs6211/6841/7307: pristine102 and noisy28

evaluations, worst noisy recovery1.8155%, with original2%/5% and prediction limits.

Automatic constant/affine selection passes six fresh synthetic worlds under

known nuisances and a documented directional-load gauge. All12 candidate fits

converge; constant truth retains constant and affine truth rejects constant.

All six selected HOLDOUTs pass inside TRAIN interpolation support, maximum angle

RMS0.000276128rad. Two earlier input/support failures remain development evidence;

support and acceptance limits were not widened. Forty-two retained predictions

independently reproduce126 channel metrics exactly.

Selected-family local linearization includes signed friction differential damping,

affine load gradient, actuator/sensor states and exact separate delays. Its64 matrix

and32 delayed-step probes pass a1e-7 gate with maximum errors4.67e-10/1.13e-9;

14 focused tests and independent review pass. Actual sampled observer/controller/FF

margins remain required; this tangent supplies neither gains nor rest/reversal proof.

A causal shared-core forecast now generates its own future successful current

through the declared family plant. Three source-clock native cases exercise start,

reversal and stop; an independent-oracle feedback case also passes. Its native

realized-input replay errors are1.09e-12rad,2.62e-7rad/s and5.23e-16A. Gyro source

timestamps, preacquisition samples and supplied initial velocity are explicit;

eight tests and independent review pass after an initial-velocity repair. The

supplied legacy-controller baseline remains unqualified, with material reference

tracking error. Dynamic FF inversion and physical qualification remain NOT_RUN.

Joint unknown mechanics plus six timing/filter coordinates remains unqualified.

Three distinct strategies pass forward numerics but fail optimizer/recovery and

historical prediction gates. The mixed-input forecast's nominal gyro-delay

precision margin is smaller than its derivative instability; peer review rejects

using it to establish adequate precision. No fresh mixed-input noise was generated.

The next branch checks physical derivative steps, analytic gyro-delay sensitivity

and numerical grid invariance before a pristine joint fit. A sensor delay must not

alter the physical plant trajectory; any confirmed numerical coupling will be

repaired in an isolated library before further estimator promotion.

All137 focused Python/native tests pass. Earlier full CTest results and the

capture retry remain as recorded in cycle02; no station tests or actions occurred.

The full ADR goal remains active, with physical yaw/pitch and Candidate14 unqualified.

Latest compact handoff:

`run/adr0022-stage2/architect-review-response-02/ADR-002.2-review02-latest-evidence.zip`

(1.98 MB). It combines the cycle02 evidence with new method/control decisions,

native tangent comparisons and three additional selected raw datasets. No source,

binaries, logs, hashes, earlier ZIP or bulk training arrays are included.

