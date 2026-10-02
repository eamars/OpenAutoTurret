# Feedforward, feedback, configuration reuse, and prospective validation

## 1. Preserve the existing control ownership

This work is for benign camera pan/tilt. ADR-002.2 identifies and controls the motor/assembly. ADR-003's target-motion design remains downstream compatibility work unless separately authorized for implementation.

Two feedforward levels have different jobs:

- **Target/framing level:** a time-stamped camera target-motion estimate and framing feedback generate a single constrained position/velocity/acceleration reference. Retain the existing constant-velocity target estimator unless evidence and scope justify a change. Do not introduce another independent position integrator, MPC, or neural controller merely because the motor model is difficult.
- **Motor level:** the plant model estimates the command needed for that reference; the existing feedback loop corrects residual error. Target state never bypasses the motor/reference limits to write current directly.

Use one authoritative reference object with `q_ref`, `v_ref`, `a_ref`, time, frame, freshness, and validity. Its components must describe the same shaped trajectory. Do not differentiate noisy target positions or raw encoder velocity to fabricate acceleration feedforward. A stationary target has approximately zero target-motion feedforward, but framing feedback and the motor's holding/load compensation may still be needed.

## 2. Implement motor feedforward in explicit units

For a supported fixed-configuration yaw reduction:

```
a * v_dot + L(q, posture, winding, configuration)
          + B * v + F(v, friction_state, temperature, configuration) = i_effective.
```

A reference-based effective-current demand can be:

```
i_effective_ff = a_hat * a_ref
               + L_hat(q_causal, posture_causal, winding_causal, configuration)
               + B_hat * v_ref
               + F_hat(v_ref, friction_state_causal, temperature, configuration).
```

This is a proposed design equation, not an identified parameter set. Define whether the load uses the causal estimated state or the planned state and verify the choice. Offline prospective simulation must use its own simulated state, not the real future angle. Choose a friction state/transition policy consistent with the identified model; do not silently use a memoryless sign switch where the fit required state.

Map effective current to the actual command domain with a verified actuator map. With a supported algebraic path `i_effective = g_i*u + i_bias`, use `u_ff = (i_effective_ff-i_bias)/g_i` with valid bounded `g_i`. A dynamic or delayed actuator requires a justified causal prediction/reference policy; do not naively invert an unknown delay, differentiate the reference repeatedly, or command unlimited high-frequency compensation.

In the host command units, combine with the existing velocity loop:

```
e_v = v_reference_for_the_existing_loop - v_hat
u_raw = u_ff + Kp*e_v + eta
u_limited = existing_current_and_slew_limiter(u_raw)
```

Retain the existing outer-position correction and shared control core; do not add a duplicate PI path or count the same positional correction twice. Distinguish the shaped reference from feedback corrections where the current code does so. The estimator/model may be replaced without proliferating independent command owners.

Use explicit actual-command accounting for anti-windup. A back-calculation form is

```
eta_dot = Ki*e_v + Kaw*(u_achieved_command - u_raw).
```

Here `u_achieved_command` is the consistently time-aligned limited/accepted command according to the verified actuator-interface policy, not the last requested value and not an arbitrarily time-shifted noisy current-feedback sample. A rejected send does not become applied current. If acknowledgments arrive later, reconcile them with the command/time they represent; validate the controller's handling of transport delay rather than comparing mismatched cycles blindly. This equation is illustrative; integrate with the existing native implementation and test its exact discrete behaviour.

## 3. Model rest, starting, sliding, reversal, and stopping explicitly

**Rest/hold:** zero velocity does not imply zero load or universally zero current. Static friction balances within an interval, not a single known cancellation value. Avoid sign chatter and integrator accumulation against a stuck axis. Pitch may require gravitational holding torque or a mechanical support; zero current is not a universal safe state.

**Start:** use an identified total directional breakaway interval plus a bounded start policy. Any additional start compensation must have validated amplitude, slew, time/energy limits, and abort conditions. It cannot be a constant kick retained throughout sliding. A failed start at the tested ceiling is censored evidence, not permission to increase it or repeat heating pulses indefinitely.

**Slide:** use identified moving load/friction compensation and feedback. Feedforward can reduce feedback workload but cannot rescue a plant model that predicts no motion during observed movement or has unbounded trajectory error.

**Reverse:** decelerate under the shaped trajectory, resolve zero crossing/sticking, then establish the opposite direction. Do not instantly reverse a large static-friction term while the measured shaft is still moving in the old direction. Observe acceleration/travel/current limits and state uncertainty.

**Stop:** validate the exact two-second window after the reference becomes zero. Later settling after actual current reaches zero is a different measurement. Ensure integrator/friction-state updates are consistent with the stopping policy.

**Model/FF invalid:** disable or bound model compensation only within an independently validated feedback-only fallback envelope. No such yaw fallback is established by these archives. Until it is, invalid FF/model state requires the validated stop/inhibit procedure, not the assertion that `FF=0` makes the motion safe.

For a normal, authorized change between valid FF values, a bounded compensating integrator change may provide bumpless transfer: `eta_new = eta_old + u_ff_old - u_ff_new`, subject to limiter/state rules. Never use bumpless-transfer logic to preserve a dangerous command after a fault.

## 4. Synthesis must use the selected plant, observer, and transitions

Local incremental mechanical damping can include `B + dF/dv`; a velocity-weakening friction law can therefore differ fundamentally from the former nonnegative constant-B approximation. Include friction-state dynamics where selected. Do not insert a new inertia into the old linear gain formula and claim it covers startup, reversal, or nonlinear low-speed motion.

Use the actual sample times, sample freshness, sensor filters, quantization, observer, current map, saturation, slew, anti-windup, and timing bounds. Retain both justified local stability/margin checks and complete nonlinear manoeuvre simulations. A failed pole calculation or undefined margin is not an acceptable margin.

Uncertainty must distinguish supported parameter sets from adversarial stress cases and previously contradicted hypotheses. Do not treat every old failed coefficient fit as a statistically calibrated uncertainty ensemble. Avoid widening prediction intervals until a failed trajectory is enclosed without useful precision.

Implement and unit-test this interface offline now. Synthesize deployable parameters only after the required plant/measurement gates pass. This package supplies no replacement gains.

## 5. Payload and configuration reuse

Store a reusable model family with configuration-specific supported parameters and validity bounds. Relevant configuration includes payload mass distribution, mounting, cable path/winding, posture/base orientation, transmission, applied motor settings, supply, temperature regime, and sensor calibration.

The geometric axis-inertia contribution of a payload is

```
J_axis = e.T @ R @ I_COM @ R.T @ e + m*(r.T@r - (e.T@r)**2).
```

Mass alone is insufficient; moving the same mass changes inertia. In an ideal vertical yaw axis, off-axis mass alone does not create direct gravitational yaw torque, though bearing load/friction can change. Pitch and tilted axes require the appropriate gravity and coupling terms. Define the full two-axis parent model in Stage 1, but do not claim the present fixed-pitch yaw data identify it.

“Tune once” means reuse the validated configuration within its tested envelope. A supported scheduled/robust variation need not cause a fresh fit every run. A materially changed payload distribution, cable route, motor firmware/gain setting, or out-of-envelope friction invokes the **same automatic identification and revalidation method**, not manual PID search and not a new architectural redesign. Keep explicit invalidation rules and evidence for re-entry.

## 6. Three different validation products

**A. Input-driven held-out plant prediction:** initialize once from information available at run start, use actual successful TX prehistory and future realized input, and predict complete outputs without future measured-state injection. This tests the plant conditional on the input that happened. It is not a pre-run closed-loop forecast.

**B. Prospective closed-loop forecast:** before the next authorized experiment, freeze the configuration, source/binary/calibration/model/controller identities, initial information, reference, limits, uncertainty set, and predicted trajectories/bounds. Simulate the control core with simulated sensors so it generates its own future current. Save predictions before viewing actual measurements. No future recorded current or angle may be substituted.

**C. Physical qualification:** execute the frozen test and compare against the saved predictions and original acceptance limits. Stage 3a must pass in the independent validation program; Stage 3b must also pass in production software using the same control/measurement semantics. Similar plots or a common configuration file alone do not prove implementation equivalence.

Evaluate whole runs and event-anchored horizons, direction, range, low-speed continuity, start/stop/reversal, saturation, overspeed, and model uncertainty coverage/width. Maintain independent modality checks where supported, without treating channels from the same IMU as independent instruments.

### Retain the original physical limits

| Predicate | Required value |
|---|---:|
| Mean actual/reference speed ratio | 0.90–1.10 |
| Continuous-motion fraction | At least 0.95 |
| Speed RMS error and jitter | At most max(0.5 deg/s, 10% of reference magnitude) |
| Detrended position P95–P5 | At most 0.15 deg |
| Detrended full position span | At most 0.30 deg |
| Fixed two-second zero-reference drift | At most 0.15 deg |
| Startup for the defined tests at at least 5 deg/s | At most 0.200 s |
| Independent angle-prediction RMS | At most 0.15 deg |

Use the original exact metric definitions, filtering, and time anchors; resolve genuine ambiguities before tests, not after failures. Preserve corresponding zero/stationary and other-domain tests from the existing ADR. The prior review is the source of the listed numeric limits; this table is not a substitute for the complete local contract.

Historical feedback runs reserved from this fit remain regression evidence already encountered in earlier development. New prospective runs are required for claims about genuinely future prediction. Full completion still includes the promised axes and operating/configuration coverage.
