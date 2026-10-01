"""Local sliding differential models of an explicitly selected offline family.

These matrices are plant/measurement tangents, not controller gains or a
qualification receipt. Transport delays remain separate exact delay factors.
Rest, breakaway and reversal require the complete nonlinear hybrid model.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numbers

import numpy as np

from .contracts import Reason, Rejected, require
from .model_family import FamilyModel, MODEL_FIELDS


def _validate_model(model):
    require(isinstance(model, FamilyModel), Reason.DATA_INVALID, "typed selected family required")
    require(all(isinstance(getattr(model, name), numbers.Real) and
                not isinstance(getattr(model, name), (bool, np.bool_)) and math.isfinite(getattr(model, name))
                for name in MODEL_FIELDS), Reason.DATA_INVALID, "finite real family quantities required")
    model.validate()
    require(model.actuator != "algebraic" or model.actuator_tau == 0, Reason.DATA_INVALID,
            "algebraic actuator must declare zero unused actuator time constant")


@dataclass(frozen=True, kw_only=True)
class SlidingSupport:
    configuration_id: str
    frame: str
    q_min_rad: float
    q_max_rad: float
    v_min_rad_s: float
    v_max_rad_s: float
    qualification: str = "SYNTHETIC_OFFLINE"

    def validate(self, model):
        require(self.qualification == "SYNTHETIC_OFFLINE" and
                isinstance(self.configuration_id, str) and bool(self.configuration_id.strip()) and
                self.frame == "output_shaft_rad", Reason.DATA_INVALID,
                "synthetic configuration label and output-shaft radian frame required")
        values = (self.q_min_rad, self.q_max_rad, self.v_min_rad_s, self.v_max_rad_s)
        require(all(isinstance(x, numbers.Real) and not isinstance(x, (bool, np.bool_)) and math.isfinite(x)
                    for x in values), Reason.DATA_INVALID, "finite numeric sliding support required")
        require(model.q_min <= self.q_min_rad < self.q_max_rad <= model.q_max and
                self.v_min_rad_s < self.v_max_rad_s and
                (self.v_min_rad_s > 1e-12 or self.v_max_rad_s < -1e-12),
                Reason.ENVELOPE_LIMITED, "local support must stay inside rollout domain and one sliding direction")
        return self


@dataclass(frozen=True, kw_only=True)
class SlidingPoint:
    q_rad: float
    v_rad_s: float
    configuration_id: str
    frame: str
    friction_state: str = "SLIDING"

    def validate(self, support):
        require(self.friction_state == "SLIDING" and self.frame == support.frame and
                self.configuration_id == support.configuration_id,
                Reason.OPERATING_POINT_CHANGED, "sliding state/frame/configuration must match supplied support")
        require(all(isinstance(x, numbers.Real) and not isinstance(x, (bool, np.bool_)) and math.isfinite(x)
                    for x in (self.q_rad, self.v_rad_s)), Reason.DATA_INVALID, "finite numeric operating point required")
        require(support.q_min_rad < self.q_rad < support.q_max_rad and
                support.v_min_rad_s < self.v_rad_s < support.v_max_rad_s,
                Reason.ENVELOPE_LIMITED, "point needs a two-sided differential neighbourhood inside sliding support")
        return self


@dataclass(frozen=True)
class FamilyLinearization:
    model: FamilyModel
    point: SlidingPoint
    support: SlidingSupport
    state_names: tuple[str, ...]
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    D: np.ndarray
    friction_derivative_A_s_rad: float
    incremental_damping_A_s_rad: float
    load_gradient_A_rad: float
    input_delay_s: float
    output_delays_s: tuple[float, float, float]

    def assert_context(self, *, model, support, frame, configuration_id, friction_state):
        """A cached tangent cannot be reused after a family/support change."""
        _validate_model(model)
        require(isinstance(support, SlidingSupport), Reason.DATA_INVALID, "typed sliding support required")
        support.validate(model)
        require(isinstance(model, FamilyModel) and isinstance(support, SlidingSupport) and
                model == self.model and support == self.support and frame == self.point.frame and
                configuration_id == self.point.configuration_id and friction_state == "SLIDING",
                Reason.OPERATING_POINT_CHANGED, "relinearize after model/support/frame/configuration/friction-state changes")

    def assert_trajectory(self, q_rad, v_rad_s, **context):
        """Reject hybrid transitions or leaving the declared numerical region.

        Staying inside this region does not certify a frozen-point approximation
        error. The selected nonlinear trajectory still needs its own simulation.
        """
        self.assert_context(**context)
        q, v = np.asarray(q_rad), np.asarray(v_rad_s)
        require(q.ndim == v.ndim == 1 and len(q) > 0 and q.shape == v.shape and
                q.dtype.kind in "iuf" and v.dtype.kind in "iuf" and
                np.isfinite(q).all() and np.isfinite(v).all(),
                Reason.DATA_INVALID, "finite numerical position/velocity trace required")
        s = self.support
        require(np.all((q > s.q_min_rad) & (q < s.q_max_rad) &
                       (v > s.v_min_rad_s) & (v < s.v_max_rad_s)),
                Reason.ENVELOPE_LIMITED, "trajectory leaves sliding support: rest/reversal needs nonlinear hybrid analysis")

    def frequency_response(self, omega_rad_s):
        """Exact delay factors on the continuous frozen tangent; no Padé model.

        This is the open-loop plant/measurement response, not an implemented
        sampled controller/observer loop. DC integrator singularities and failed
        solves are rejected rather than reported as a valid margin.
        """
        omega = np.asarray(omega_rad_s)
        require(omega.dtype.kind in "iuf" and omega.ndim <= 1 and omega.size > 0 and
                np.isfinite(omega).all() and np.all(omega > 0),
                Reason.DATA_INVALID, "positive finite rad/s frequencies required")
        s = 1j*omega
        try:
            response = (self.C @ np.linalg.solve(s[..., None, None]*np.eye(len(self.A))-self.A,
                         np.broadcast_to(self.B, s.shape+self.B.shape)) + self.D)[..., 0]
        except np.linalg.LinAlgError as exc:
            raise Rejected(Reason.MODEL_INADEQUATE, "local frequency response has a singular solve") from exc
        response *= np.exp(-s[..., None]*(self.input_delay_s+np.array(self.output_delays_s)))
        require(np.isfinite(response).all(), Reason.MODEL_INADEQUATE, "local frequency response is nonfinite")
        return response

    def document(self):
        return {"schema": "adr0022.selected-family-local-linearization/1",
            "qualification": "SYNTHETIC_OFFLINE_UNQUALIFIED", "structure": self.model.structure,
            "configuration_id": self.point.configuration_id, "frame": self.point.frame,
            "point_q_rad": self.point.q_rad, "point_v_rad_s": self.point.v_rad_s,
            "state_names": list(self.state_names), "output_names": ["q_rad", "gyro_rad_s", "reported_current_A"],
            "input_name": "accepted_CAN_command_current_A", "A": self.A.tolist(), "B": self.B.tolist(),
            "C": self.C.tolist(), "D": self.D.tolist(),
            "friction_derivative_A_s_rad": self.friction_derivative_A_s_rad,
            "incremental_damping_A_s_rad": self.incremental_damping_A_s_rad,
            "load_gradient_A_rad": self.load_gradient_A_rad, "input_delay_s": self.input_delay_s,
            "output_delays_s": list(self.output_delays_s),
            "delay_representation": "EXACT_TRANSFER_FACTORS; NO_PADE_OR_DELAY_INVERSION",
            "limitations": ["frozen sliding tangent; no rest/breakaway/reversal coverage",
                "declared support is numerical, not physically certified",
                "affine load with moving state need not be an equilibrium",
                "no implemented sampled observer/controller stability or margins",
                "no physical model qualification, controller gains or deployment"]}


def linearize_sliding_family(model, point, support):
    """Frozen-point tangent, with input in accepted CAN command A.

    Outputs are q [rad], gyro [rad/s], reported current [A]. Biases disappear
    from perturbation matrices; actuator/current gains remain explicit. An
    affine load at nonzero speed is not an equilibrium: this is a local tangent
    of its moving trajectory, not a global time-invariant replacement.
    """
    require(isinstance(model, FamilyModel) and isinstance(point, SlidingPoint) and
            isinstance(support, SlidingSupport), Reason.DATA_INVALID, "typed family/point/support required")
    _validate_model(model); support.validate(model); point.validate(support)
    names = ["q_rad", "v_rad_s"]
    if model.actuator == "first_order": names.append("effective_current_A")
    if model.gyro_tau > 0: names.append("gyro_filter_rad_s")
    if model.current_tau > 0: names.append("current_filter_A")
    n = len(names)
    A, B, C, D = np.zeros((n, n)), np.zeros((n, 1)), np.zeros((3, n)), np.zeros((3, 1))
    derivative = 0.
    if model.friction == "stribeck":
        positive = point.v_rad_s > 0
        fc = model.coulomb_positive if positive else model.coulomb_negative
        fs = model.static_positive if positive else model.static_negative
        vs = model.stribeck_positive if positive else model.stribeck_negative
        if fs > fc:
            # Evaluate the derivative in log space, including the zero limiting
            # value at very high speed, without inf*0 or power overflow.
            log_speed = math.log(abs(point.v_rad_s))-math.log(vs)
            log_power = model.stribeck_power*log_speed
            if log_power <= math.log(np.finfo(float).max):
                log_magnitude = (math.log(fs-fc)+math.log(model.stribeck_power)-math.log(vs) +
                    (model.stribeck_power-1)*log_speed-math.exp(log_power))
                require(log_magnitude <= math.log(np.finfo(float).max), Reason.MODEL_INADEQUATE,
                        "friction differential exceeds finite numerical range")
                derivative = -math.exp(log_magnitude)
    damping = model.viscous + derivative
    gradient = model.load_slope if model.load == "affine" else 0.
    A[0, 1], A[1, 0], A[1, 1] = 1., -gradient/model.a, -damping/model.a
    current_state = np.zeros(n)
    current_input = 0.
    if model.actuator == "first_order":
        j = names.index("effective_current_A")
        A[1, j], A[j, j], B[j, 0] = 1/model.a, -1/model.actuator_tau, model.actuator_gain/model.actuator_tau
        current_state[j] = 1.
    else:
        B[1, 0] = model.actuator_gain/model.a
        current_input = model.actuator_gain
    C[0, 0] = 1.
    if model.gyro_tau > 0:
        j = names.index("gyro_filter_rad_s")
        A[j, 1], A[j, j], C[1, j] = 1/model.gyro_tau, -1/model.gyro_tau, 1.
    else: C[1, 1] = 1.
    if model.current_tau > 0:
        j = names.index("current_filter_A")
        A[j] += current_state/model.current_tau
        A[j, j] = -1/model.current_tau
        B[j, 0], C[2, j] = current_input/model.current_tau, model.current_gain
    else:
        C[2] = model.current_gain*current_state
        D[2, 0] = model.current_gain*current_input
    require(all(np.isfinite(matrix).all() for matrix in (A, B, C, D)),
            Reason.MODEL_INADEQUATE, "local matrices exceed finite numerical range")
    # Immutable byte backing also prevents callers from re-enabling writes on a
    # cached matrix and silently changing the supported differential model.
    A, B, C, D = (np.frombuffer(matrix.tobytes(), dtype=float).reshape(matrix.shape)
                  for matrix in (A, B, C, D))
    return FamilyLinearization(model, point, support, tuple(names), A, B, C, D,
        derivative, damping, gradient, model.transport_delay, (0., model.gyro_delay, model.current_delay))
