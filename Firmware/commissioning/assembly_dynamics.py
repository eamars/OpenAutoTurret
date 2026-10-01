"""Supplied-parameter two-revolute rigid-assembly mechanics, in SI units.

This is a runnable Stage 1 parent model, not identification of the station.
Joint coordinates remain unwrapped; cable/winding/friction loads are supplied
separately. No current-to-torque constant, mass or geometry is inferred here.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
import numpy as np


class AssemblyInvalid(ValueError):
    pass


def vector(value, shape, name):
    try:
        if np.asarray(value).dtype.kind not in "iuf":
            raise AssemblyInvalid(f"{name}: real numeric SI values required")
        result = np.asarray(value, dtype=float).copy()
    except (ValueError, TypeError) as exc:
        raise AssemblyInvalid(f"{name}: explicit finite SI values required") from exc
    if result.shape != shape or not np.isfinite(result).all():
        raise AssemblyInvalid(f"{name}: expected finite shape {shape}")
    result.flags.writeable = False
    return result


def rotation(value, name):
    result = vector(value, (3, 3), name)
    if not np.allclose(result.T@result, np.eye(3), atol=1e-10, rtol=0) or not math.isclose(np.linalg.det(result), 1., abs_tol=1e-10):
        raise AssemblyInvalid(f"{name}: proper orthonormal rotation required")
    return result


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def unit_axis(value):
    axis = vector(value, (3,), "revolute axis")
    norm = np.linalg.norm(axis)
    if not math.isfinite(norm) or not norm > 0:
        raise AssemblyInvalid("axis must have finite nonzero magnitude")
    return vector(axis/norm, (3,), "unit revolute axis")


def axis_rotation(axis, angle):
    if not isinstance(angle, Real) or isinstance(angle, (bool, np.bool_)) or not math.isfinite(angle):
        raise AssemblyInvalid("finite joint angle required")
    s = skew(unit_axis(axis))
    return np.eye(3) + math.sin(angle)*s + (1-math.cos(angle))*(s@s)


@dataclass(frozen=True, kw_only=True)
class RigidBody:
    mass_kg: float
    com_in_body_m: np.ndarray
    inertia_com_body_kg_m2: np.ndarray
    mount_translation_m: np.ndarray  # from this body's owning joint, in that joint's rotating frame
    mount_rotation: np.ndarray  # body frame to owning joint's rotating frame

    def __post_init__(self):
        if not isinstance(self.mass_kg, Real) or isinstance(self.mass_kg, (bool, np.bool_)) or not math.isfinite(self.mass_kg) or self.mass_kg <= 0:
            raise AssemblyInvalid("body mass must be supplied, finite and positive")
        object.__setattr__(self, "com_in_body_m", vector(self.com_in_body_m, (3,), "body COM"))
        object.__setattr__(self, "mount_translation_m", vector(self.mount_translation_m, (3,), "body mount translation"))
        object.__setattr__(self, "mount_rotation", rotation(self.mount_rotation, "body mount rotation"))
        inertia = vector(self.inertia_com_body_kg_m2, (3, 3), "COM inertia")
        if not np.allclose(inertia, inertia.T, atol=1e-12, rtol=0):
            raise AssemblyInvalid("COM inertia must be symmetric")
        eigenvalues = np.linalg.eigvalsh(inertia)
        if eigenvalues[0] < -1e-12 or eigenvalues[-1] > eigenvalues[:2].sum()+1e-12:
            raise AssemblyInvalid("COM inertia must be positive semidefinite with physical principal moments")
        object.__setattr__(self, "inertia_com_body_kg_m2", inertia)

    @property
    def com_in_joint_m(self):
        return self.mount_translation_m + self.mount_rotation@self.com_in_body_m


def axis_inertia(body, axis, axis_origin_in_joint_m):
    """e' R I_COM R' e + m (r'r - (e'r)^2), kg m²."""
    if not isinstance(body, RigidBody):
        raise AssemblyInvalid("explicit rigid body required")
    e = unit_axis(axis)
    r = body.com_in_joint_m-vector(axis_origin_in_joint_m, (3,), "axis origin")
    inertia = body.mount_rotation@body.inertia_com_body_kg_m2@body.mount_rotation.T
    return float(e@inertia@e + body.mass_kg*(r@r-(e@r)**2))


@dataclass(frozen=True, kw_only=True)
class ActuatorMap:
    torque_per_effective_amp_Nm: np.ndarray  # signed calibration for each positive joint axis
    command_gain: np.ndarray
    command_bias_A: np.ndarray

    def __post_init__(self):
        for name in ("torque_per_effective_amp_Nm", "command_gain", "command_bias_A"):
            object.__setattr__(self, name, vector(getattr(self, name), (2,), name))
        if np.any(self.torque_per_effective_amp_Nm == 0) or np.any(self.command_gain == 0):
            raise AssemblyInvalid("supplied actuator scales must be nonzero")

    def command_for_torque(self, torque_Nm):
        effective = vector(torque_Nm, (2,), "torque")/self.torque_per_effective_amp_Nm
        command = (effective-self.command_bias_A)/self.command_gain
        if not np.isfinite([effective, command]).all():
            raise AssemblyInvalid("actuator current inversion overflow")
        return effective, command

    def torque_for_command(self, command_A):
        torque = self.torque_per_effective_amp_Nm*(self.command_gain*vector(command_A, (2,), "command current")+self.command_bias_A)
        if not np.isfinite(torque).all():
            raise AssemblyInvalid("actuator torque mapping overflow")
        return torque


@dataclass(frozen=True, kw_only=True)
class CausalJointLoad:
    # All three terms are generalized load torques on the left of the dynamics
    # equation: positive values require positive actuator compensation.
    cable_torque_Nm: np.ndarray
    friction_torque_Nm: np.ndarray
    external_load_torque_Nm: np.ndarray
    time_s: float
    configuration_id: str
    valid: bool

    def __post_init__(self):
        for name in ("cable_torque_Nm", "friction_torque_Nm", "external_load_torque_Nm"):
            object.__setattr__(self, name, vector(getattr(self, name), (2,), name))

    def total(self, *, now_s, max_age_s, configuration_id):
        times = (self.time_s, now_s, max_age_s)
        if self.valid is not True or self.configuration_id != configuration_id or not all(isinstance(t, Real) and not isinstance(t, (bool, np.bool_)) and math.isfinite(t) for t in times) or not 0 <= now_s-self.time_s <= max_age_s:
            raise AssemblyInvalid("explicit valid, configuration-matched causal joint loads required")
        return self.cable_torque_Nm+self.friction_torque_Nm+self.external_load_torque_Nm


@dataclass(frozen=True)
class Dynamics:
    M_kg_m2: np.ndarray
    C_kg_m2_s: np.ndarray
    G_Nm: np.ndarray
    dM_dq: np.ndarray
    potential_J: float
    kinetic_J: float


@dataclass(frozen=True, kw_only=True)
class SerialAssembly:
    yaw_carriage: RigidBody
    pitch_payload: RigidBody
    yaw_axis_in_base: np.ndarray
    yaw_origin_in_base_m: np.ndarray
    pitch_axis_in_yaw: np.ndarray
    pitch_origin_in_yaw_m: np.ndarray
    base_rotation_world: np.ndarray
    base_origin_world_m: np.ndarray
    gravity_world_m_s2: np.ndarray
    configuration_id: str

    def __post_init__(self):
        for name in ("yaw_axis_in_base", "pitch_axis_in_yaw"):
            object.__setattr__(self, name, unit_axis(getattr(self, name)))
        for name in ("yaw_origin_in_base_m", "pitch_origin_in_yaw_m", "base_origin_world_m", "gravity_world_m_s2"):
            object.__setattr__(self, name, vector(getattr(self, name), (3,), name))
        object.__setattr__(self, "base_rotation_world", rotation(self.base_rotation_world, "base orientation"))
        if not isinstance(self.yaw_carriage, RigidBody) or not isinstance(self.pitch_payload, RigidBody) or not self.configuration_id:
            raise AssemblyInvalid("both rigid bodies and explicit configuration identity required")

    def _bodies(self, q):
        yaw, pitch = vector(q, (2,), "unwrapped joint position")
        rotation_yaw = self.base_rotation_world@axis_rotation(self.yaw_axis_in_base, yaw)
        e0 = self.base_rotation_world@self.yaw_axis_in_base
        e1 = rotation_yaw@self.pitch_axis_in_yaw
        p0 = self.base_origin_world_m+self.base_rotation_world@self.yaw_origin_in_base_m
        p1 = p0+rotation_yaw@self.pitch_origin_in_yaw_m
        rotation_pitch = rotation_yaw@axis_rotation(self.pitch_axis_in_yaw, pitch)
        result = []
        for index, body, frame, origin in ((0, self.yaw_carriage, rotation_yaw, p0),
                                            (1, self.pitch_payload, rotation_pitch, p1)):
            com = origin+frame@body.com_in_joint_m
            body_rotation = frame@body.mount_rotation
            inertia = body_rotation@body.inertia_com_body_kg_m2@body_rotation.T
            jv, jw = np.zeros((3, 2)), np.zeros((3, 2))
            jv[:, 0], jw[:, 0] = np.cross(e0, com-p0), e0
            if index == 1:
                jv[:, 1], jw[:, 1] = np.cross(e1, com-p1), e1
            djv, djw, di = np.zeros((2, 3, 2)), np.zeros((2, 3, 2)), np.zeros((2, 3, 3))
            for k in range(index+1):
                djv[k, :, 0] = np.cross(e0, jv[:, k])
                spin = skew(e0 if k == 0 else e1)
                di[k] = spin@inertia-inertia@spin
                if index == 1:
                    de1 = np.cross(e0, e1) if k == 0 else np.zeros(3)
                    dp1 = np.cross(e0, p1-p0) if k == 0 else np.zeros(3)
                    djv[k, :, 1] = np.cross(de1, com-p1)+np.cross(e1, jv[:, k]-dp1)
                    djw[k, :, 1] = de1
            result.append((body.mass_kg, com, inertia, jv, jw, djv, djw, di))
        return result

    def dynamics(self, q, velocity):
        v = vector(velocity, (2,), "joint velocity")
        mass, derivatives, gravity = np.zeros((2, 2)), np.zeros((2, 2, 2)), np.zeros(2)
        potential = 0.
        for m, com, inertia, jv, jw, djv, djw, di in self._bodies(q):
            mass += m*jv.T@jv+jw.T@inertia@jw
            gravity -= jv.T@(m*self.gravity_world_m_s2)
            potential -= m*self.gravity_world_m_s2@com
            for k in range(2):
                derivatives[k] += m*(djv[k].T@jv+jv.T@djv[k])+djw[k].T@inertia@jw+jw.T@di[k]@jw+jw.T@inertia@djw[k]
        if not all(np.isfinite(x).all() for x in (mass, derivatives, gravity)) or not math.isfinite(potential):
            raise AssemblyInvalid("assembly dynamics overflow")
        if not np.linalg.eigvalsh(mass).min() > 0:
            raise AssemblyInvalid("assembly kinetic matrix is singular; independent joint dynamics unavailable")
        coriolis = np.zeros((2, 2))
        for i in range(2):
            for j in range(2):
                coriolis[i, j] = sum(.5*(derivatives[k, i, j]+derivatives[j, i, k]-derivatives[i, j, k])*v[k] for k in range(2))
        return Dynamics(mass, coriolis, gravity, derivatives, float(potential), float(.5*v@mass@v))

    def current_demand(self, q, velocity, acceleration, actuator, loads, *, now_s, max_load_age_s):
        if not isinstance(actuator, ActuatorMap) or not isinstance(loads, CausalJointLoad):
            raise AssemblyInvalid("explicit actuator map and causal load packet required")
        v, a = vector(velocity, (2,), "joint velocity"), vector(acceleration, (2,), "joint acceleration")
        d = self.dynamics(q, v)
        load = loads.total(now_s=now_s, max_age_s=max_load_age_s, configuration_id=self.configuration_id)
        torque = d.M_kg_m2@a+d.C_kg_m2_s@v+d.G_Nm+load
        effective, command = actuator.command_for_torque(torque)
        return {"torque_Nm": torque, "effective_current_A": effective, "command_current_A": command,
                "configuration_id": self.configuration_id, "qualification": "SUPPLIED_MATHEMATICS_ONLY"}

    def acceleration(self, q, velocity, command_A, actuator, loads, *, now_s, max_load_age_s):
        if not isinstance(actuator, ActuatorMap) or not isinstance(loads, CausalJointLoad):
            raise AssemblyInvalid("explicit actuator map and causal load packet required")
        v = vector(velocity, (2,), "joint velocity")
        d = self.dynamics(q, v)
        load = loads.total(now_s=now_s, max_age_s=max_load_age_s, configuration_id=self.configuration_id)
        return np.linalg.solve(d.M_kg_m2, actuator.torque_for_command(command_A)-d.C_kg_m2_s@v-d.G_Nm-load)

    def rollout(self, times_s, commands_A, initial_q, initial_velocity, actuator, load_function, *, max_load_age_s):
        """RK4 with supplied per-interval ZOH commands; no coordinate wrapping."""
        t = np.asarray(times_s, dtype=float)
        if t.ndim != 1 or len(t) < 2 or not np.isfinite(t).all() or not np.all(np.diff(t) > 0):
            raise AssemblyInvalid("strictly increasing finite rollout times required")
        u = vector(commands_A, (len(t)-1, 2), "per-interval successful commands")
        result = np.empty((len(t), 4))
        result[0] = np.r_[vector(initial_q, (2,), "initial joint position"), vector(initial_velocity, (2,), "initial joint velocity")]
        for k, dt in enumerate(np.diff(t)):
            def derivative(at, x):
                loads = load_function(at, x[:2].copy(), x[2:].copy())
                return np.r_[x[2:], self.acceleration(x[:2], x[2:], u[k], actuator, loads, now_s=at, max_load_age_s=max_load_age_s)]
            x, at = result[k], t[k]
            k1 = derivative(at, x)
            k2 = derivative(at+dt/2, x+dt*k1/2)
            k3 = derivative(at+dt/2, x+dt*k2/2)
            k4 = derivative(at+dt, x+dt*k3)
            result[k+1] = x+dt*(k1+2*k2+2*k3+k4)/6
            if not np.isfinite(result[k+1]).all():
                raise AssemblyInvalid(f"nonfinite forward rollout at interval {k}")
        return result
