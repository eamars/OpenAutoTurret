"""Bindings to the same C++ mathematical implementation linked by firmware."""
from __future__ import annotations
import ctypes as ct
import os
from dataclasses import asdict
from pathlib import Path
import numpy as np
from .contracts import ModelSpec, Reason, array, require


class CModel(ct.Structure):
    _fields_ = [("n", ct.c_int), ("periodic", ct.c_int), ("q", ct.c_double * 8),
                ("z", ct.c_double * 3), ("theta", ct.c_double * 55)]


class CObserver(ct.Structure):
    _fields_ = [(name, ct.c_double) for name in ("encoder_variance", "gyro_variance",
        "process_variance", "max_encoder_age_s", "max_gyro_age_s",
        "initial_position_variance", "initial_velocity_variance")] + [("encoder_only_verified", ct.c_int)]


CONTROL_FIELDS = ("kp", "ki", "kpos", "kaw", "current_cap", "slew", "integral_cap", "velocity_cap",
                  "dt_min", "dt_max", "intent_threshold", "rest_speed", "sustained_s", "start_timeout_s")
ACCELERATION_FIELDS = ("acceleration_cap", "acceleration_noise_sigma", "acceleration_sample_period_s",
                       "acceleration_current_window_enabled")


class CParameters(ct.Structure):
    _fields_ = [("model", CModel), ("observer", CObserver)] + [(k, ct.c_double) for k in CONTROL_FIELDS] + [
        ("start_total", ct.c_double*48), ("start_censored", ct.c_int*48)] + [
        (k, ct.c_double) for k in ACCELERATION_FIELDS]


class CObservation(ct.Structure):
    _fields_ = [(k, ct.c_double) for k in ("now", "encoder_time", "gyro_time", "position", "gyro_rate")] + [
        (k, ct.c_uint64) for k in ("encoder_seq", "gyro_seq", "generation")] + [
        ("encoder_valid", ct.c_int), ("gyro_valid", ct.c_int)]


class CReference(ct.Structure):
    _fields_ = [(k, ct.c_double) for k in ("position", "velocity", "acceleration", "posture")]


class CPosterior(ct.Structure):
    """Read-only snapshot made inside the core, after this cycle's observe()."""
    _fields_ = [(k, ct.c_double) for k in ("now", "dt", "position", "velocity",
        "encoder_time", "gyro_time", "accepted_current", "accepted_time")] + [
        (k, ct.c_uint64) for k in ("encoder_seq", "gyro_seq", "generation")] + [
        (k, ct.c_int) for k in ("encoder_only", "motion", "accepted_actual_time")]


POSTERIOR_FEEDFORWARD_CALLBACK = ct.CFUNCTYPE(ct.c_int, ct.c_void_p,
    ct.POINTER(CPosterior), ct.POINTER(CReference), ct.POINTER(ct.c_double))


class COutput(ct.Structure):
    _fields_ = [(k, ct.c_double) for k in ("requested", "limited", "position", "velocity",
        "integral", "feedforward", "start_increment")] + [("sequence", ct.c_uint64)] + [
        (k, ct.c_int) for k in ("status", "motion", "encoder_only")] + [
        (k, ct.c_double) for k in ("requested_reference_velocity", "shaped_reference_velocity",
        "requested_reference_acceleration", "shaped_reference_acceleration", "measured_acceleration",
        "acceleration_sample_time", "acceleration_interval_s", "acceleration_noise_sigma",
        "acceleration_feedback_horizon_s", "delayed_applied_current", "acceleration_current_min",
        "acceleration_current_max", "acceleration_limited_request")] + [
        (k, ct.c_int) for k in ("acceleration_fresh", "acceleration_valid",
                              "acceleration_limit_reason", "current_history_actual_time")]


class Simulation(ct.Structure):
    _fields_ = [(k, ct.c_double) for k in ("dt", "encoder_quantum", "encoder_noise", "gyro_noise",
        "measurement_delay", "gyro_filter_tau")] + [("encoder_period", ct.c_int),
        ("gyro_period", ct.c_int), ("seed", ct.c_uint64)]


def model(spec, theta):
    theta = array(theta, (spec.size,), "theta")
    return CModel(len(spec.q_nodes), spec.periodic,
                  (ct.c_double * 8)(*spec.q_nodes), (ct.c_double * 3)(*spec.z_nodes),
                  (ct.c_double * 55)(*theta))


class Native:
    def __init__(self, path: Path | None = None):
        path = path or Path(os.environ.get("OTA_AXIS_CORE_LIBRARY", ""))
        require(path.is_file(), Reason.INTEGRATION_MISMATCH,
                "set OTA_AXIS_CORE_LIBRARY to the locally built shared library")
        self.path = path.resolve()
        self.lib = ct.CDLL(str(self.path))
        require(self.lib.ota_core_abi() == 4, Reason.INTEGRATION_MISMATCH, "core ABI differs")
        self.lib.ota_model_rollout.argtypes = [ct.POINTER(CModel), ct.c_int,
            *([ct.POINTER(ct.c_double)] * 3), ct.POINTER(ct.c_int), ct.c_double,
            ct.c_double, ct.POINTER(ct.c_double)]
        self.lib.ota_model_rollout.restype = ct.c_int
        self.history_rollout = getattr(self.lib, "ota_model_rollout_with_history", None)
        if self.history_rollout is not None:
            self.history_rollout.argtypes = [ct.POINTER(CModel), ct.c_int,
                ct.POINTER(ct.c_double), ct.POINTER(ct.c_double), ct.POINTER(ct.c_int),
                ct.c_int, ct.POINTER(ct.c_double), ct.POINTER(ct.c_double),
                ct.c_double, ct.c_double, ct.POINTER(ct.c_double)]
            self.history_rollout.restype = ct.c_int
        self.lib.ota_controller_create.argtypes = [ct.POINTER(CParameters)]
        self.lib.ota_controller_create.restype = ct.c_void_p
        self.lib.ota_controller_destroy.argtypes = [ct.c_void_p]
        self.lib.ota_controller_reset.argtypes = [ct.c_void_p, *([ct.c_double]*4), ct.c_uint64]
        self.lib.ota_controller_reset_at.argtypes = [ct.c_void_p, *([ct.c_double]*4), ct.c_uint64, ct.c_double]
        self.lib.ota_controller_step.argtypes = [ct.c_void_p, ct.POINTER(CObservation),
                                                ct.POINTER(CReference), ct.POINTER(COutput)]
        self.controller_step_ff = getattr(self.lib, "ota_controller_step_ff", None)
        if self.controller_step_ff is not None:
            self.controller_step_ff.argtypes = [ct.c_void_p, ct.POINTER(CObservation),
                                                ct.POINTER(CReference), ct.c_double, ct.POINTER(COutput)]
            self.controller_step_ff.restype = ct.c_int
        self.controller_step_posterior_ff = getattr(self.lib, "ota_controller_step_posterior_ff", None)
        if self.controller_step_posterior_ff is not None:
            self.controller_step_posterior_ff.argtypes = [ct.c_void_p, ct.POINTER(CObservation),
                ct.POINTER(CReference), POSTERIOR_FEEDFORWARD_CALLBACK, ct.c_void_p, ct.POINTER(COutput)]
            self.controller_step_posterior_ff.restype = ct.c_int
        self.controller_step_posterior_ff_intent = getattr(self.lib, "ota_controller_step_posterior_ff_intent", None)
        if self.controller_step_posterior_ff_intent is not None:
            self.controller_step_posterior_ff_intent.argtypes = [ct.c_void_p, ct.POINTER(CObservation),
                ct.POINTER(CReference), ct.c_int, POSTERIOR_FEEDFORWARD_CALLBACK, ct.c_void_p, ct.POINTER(COutput)]
            self.controller_step_posterior_ff_intent.restype = ct.c_int
        self.controller_step_posterior_ff_phase = getattr(self.lib, "ota_controller_step_posterior_ff_phase", None)
        if self.controller_step_posterior_ff_phase is not None:
            self.controller_step_posterior_ff_phase.argtypes = [ct.c_void_p, ct.POINTER(CObservation),
                ct.POINTER(CReference), ct.c_int, ct.c_int, POSTERIOR_FEEDFORWARD_CALLBACK, ct.c_void_p, ct.POINTER(COutput)]
            self.controller_step_posterior_ff_phase.restype = ct.c_int
        self.controller_inhibit = getattr(self.lib, "ota_controller_inhibit", None)
        if self.controller_inhibit is not None:
            self.controller_inhibit.argtypes = [ct.c_void_p]
            self.controller_inhibit.restype = ct.c_int
        self.controller_parameters = getattr(self.lib, "ota_controller_parameters", None)
        if self.controller_parameters is not None:
            self.controller_parameters.argtypes = [ct.c_void_p, ct.POINTER(CParameters)]
            self.controller_parameters.restype = ct.c_int
        self.lib.ota_controller_ack.argtypes = [ct.c_void_p, ct.c_uint64, ct.c_int, ct.c_double]
        self.controller_ack_at = getattr(self.lib, "ota_controller_ack_at", None)
        if self.controller_ack_at is not None:
            self.controller_ack_at.argtypes = [ct.c_void_p, ct.c_uint64, ct.c_int, ct.c_double, ct.c_double]
            self.controller_ack_at.restype = ct.c_int
        self.lib.ota_controller_switch.argtypes = [ct.c_void_p, ct.POINTER(CParameters), ct.POINTER(CReference)]
        self.lib.ota_closed_rollout.argtypes = [ct.POINTER(CParameters), ct.POINTER(CParameters),
            ct.POINTER(Simulation), ct.c_int, ct.POINTER(CReference), ct.c_double, ct.c_double,
            ct.POINTER(ct.c_double)]

    def rollout(self, spec: ModelSpec, theta, t, tx, z, direction, initial, *,
                tx_history_t=None, tx_history_A=None):
        n = len(t)
        vectors = [np.ascontiguousarray(array(v, (n,), name)) for v, name in
                   ((t, "time"), (tx, "successful TX"), (z, "posture"))]
        dirs = np.ascontiguousarray(direction, dtype=np.int32)
        require(dirs.shape == (n,) and np.isin(dirs, [-1, 1]).all(), Reason.DATA_INVALID,
                "invalid running direction")
        out = np.empty((n, 2), dtype=np.float64)
        m = model(spec, theta)
        require((tx_history_t is None) == (tx_history_A is None), Reason.DATA_INVALID,
                "successful TX history requires both timestamps and currents")
        if tx_history_t is None:
            # Legacy callers explicitly retain the first-window-current hold.
            status = self.lib.ota_model_rollout(ct.byref(m), n,
                *(x.ctypes.data_as(ct.POINTER(ct.c_double)) for x in vectors),
                dirs.ctypes.data_as(ct.POINTER(ct.c_int)), *initial,
                out.ctypes.data_as(ct.POINTER(ct.c_double)))
        else:
            require(self.history_rollout is not None, Reason.INTEGRATION_MISMATCH,
                    "native library lacks successful TX history rollout; rebuild it")
            count = len(tx_history_t)
            history = [np.ascontiguousarray(array(value, (count,), name)) for value, name in
                       ((tx_history_t, "successful TX history time"),
                        (tx_history_A, "successful TX history current"))]
            require(count > 0 and np.all(np.diff(history[0]) > 0), Reason.DATA_INVALID,
                    "successful TX history must increase uniquely")
            status = self.history_rollout(ct.byref(m), n,
                vectors[0].ctypes.data_as(ct.POINTER(ct.c_double)),
                vectors[2].ctypes.data_as(ct.POINTER(ct.c_double)),
                dirs.ctypes.data_as(ct.POINTER(ct.c_int)), count,
                *(x.ctypes.data_as(ct.POINTER(ct.c_double)) for x in history), *initial,
                out.ctypes.data_as(ct.POINTER(ct.c_double)))
        require(status == 0, Reason.MODEL_INADEQUATE if status == 2 else Reason.DATA_INVALID,
                ("actual successful TX history does not cover window start minus model delay"
                 if status == 3 else f"native rollout rejected input/domain (status={status})"))
        return out

    def closed_rollout(self, control, plant, simulation, references, initial):
        refs = np.ascontiguousarray(references, dtype=np.float64)
        require(refs.ndim==2 and refs.shape[1]==4 and np.isfinite(refs).all(),
                Reason.DATA_INVALID, "finite q/v/a/posture reference required")
        out = np.empty((len(refs),12), dtype=np.float64)
        status = self.lib.ota_closed_rollout(ct.byref(control),ct.byref(plant),ct.byref(simulation),
            len(refs),refs.ctypes.data_as(ct.POINTER(CReference)),*initial,
            out.ctypes.data_as(ct.POINTER(ct.c_double)))
        require(status==0, Reason.ENVELOPE_LIMITED if status==2 else Reason.DATA_INVALID,
                f"closed C++ simulation rejected trajectory/state ({status})"+
                (f" q/v/posture/tick={out[0,:4].tolist()}" if status==2 else ""))
        return out


def acceleration_values(values):
    """Keep old offline profiles explicitly inactive; never invent live guard values."""
    supplied = set(values) & set(ACCELERATION_FIELDS)
    require(not supplied or supplied == set(ACCELERATION_FIELDS), Reason.DATA_INVALID,
            "complete acceleration runtime values required")
    if not supplied:
        return {k: 0.0 for k in ACCELERATION_FIELDS}, "LEGACY_OFFLINE_ACCELERATION_INACTIVE"
    result = {k: values[k] for k in ACCELERATION_FIELDS}
    require(all(not isinstance(v, (bool, np.bool_)) and isinstance(v, (int, float, np.number))
                and np.isfinite(v) and v >= 0 for v in result.values()), Reason.DATA_INVALID,
            "acceleration runtime fields must be finite nonnegative numbers")
    require(result["acceleration_cap"] == 0 or result["acceleration_sample_period_s"] > 0,
            Reason.DATA_INVALID, "active acceleration guard requires a positive measured sample period")
    require(result["acceleration_current_window_enabled"] in (0, 1), Reason.DATA_INVALID,
            "acceleration current window mode must be explicit zero or one")
    return result, ("EXPLICIT_RUNTIME_ACCELERATION" if result["acceleration_cap"] > 0
                    else "EXPLICIT_OFFLINE_ACCELERATION_INACTIVE")


def parameters(spec, theta, observer, values, start_total, start_censored, *, acceleration=None):
    if acceleration is not None:
        require(set(acceleration) == set(ACCELERATION_FIELDS), Reason.DATA_INVALID,
                "complete supplied acceleration fields required")
        values = {**values, **acceleration}
    require(set(CONTROL_FIELDS) <= set(values) <= set(CONTROL_FIELDS + ACCELERATION_FIELDS), Reason.DATA_INVALID,
            "complete runtime controller values required; no implicit tuning defaults")
    require(all(not isinstance(v,(bool,np.bool_)) and isinstance(v,(int,float,np.number)) and np.isfinite(v)
                for v in values.values()),Reason.DATA_INVALID,"runtime numerical fields must be finite numbers")
    obs = asdict(observer)
    cobs = CObserver(*(obs[k] for k, _ in CObserver._fields_))
    n = 6*len(spec.q_nodes)
    starts = array(start_total, (2, 3, len(spec.q_nodes)), "start currents")
    censored = np.asarray(start_censored)
    require(censored.shape == starts.shape and censored.dtype == bool, Reason.DATA_INVALID,
            "breakaway censor mask invalid")
    guard, provenance = acceleration_values(values)
    result = CParameters(model(spec, theta), cobs, *(values[k] for k in CONTROL_FIELDS),
                         (ct.c_double*48)(*starts.ravel()), (ct.c_int*48)(*(int(v) for v in censored.ravel())),
                         *(guard[k] for k in ACCELERATION_FIELDS))
    result.acceleration_binding_provenance = provenance
    return result


class Controller:
    def __init__(self, native, params):
        self.native, self.params = native, params
        self._posterior_call_active = False
        self.handle = native.lib.ota_controller_create(ct.byref(params))
        require(bool(self.handle), Reason.DATA_INVALID, "C++ controller rejected complete parameter set")

    def __enter__(self): return self
    def __exit__(self, *args): self.close()
    def close(self):
        if self._posterior_call_active:
            self.inhibit()
            require(False, Reason.DATA_INVALID, "cannot destroy controller inside its FF call")
        if self.handle:
            self.native.lib.ota_controller_destroy(self.handle); self.handle = None

    def reset(self, t, q, v, current, generation=1, *, accepted_time=None):
        reset = (self.native.lib.ota_controller_reset(self.handle, t, q, v, current, generation)
                 if accepted_time is None else
                 self.native.lib.ota_controller_reset_at(self.handle, t, q, v, current, generation, accepted_time))
        require(bool(reset),
                Reason.DATA_INVALID, "controller initialization rejected")

    def step(self, observation, reference):
        result = COutput()
        require(bool(self.native.lib.ota_controller_step(self.handle, ct.byref(observation),
                ct.byref(reference), ct.byref(result))), Reason.INTEGRATION_MISMATCH, "core call failed")
        return result

    def step_feedforward(self, observation, reference, command_feedforward):
        require(self.native.controller_step_ff is not None, Reason.INTEGRATION_MISMATCH,
                "native library lacks explicit feedforward step; rebuild it")
        result = COutput()
        require(bool(self.native.controller_step_ff(self.handle, ct.byref(observation),
                ct.byref(reference), command_feedforward, ct.byref(result))),
                Reason.INTEGRATION_MISMATCH, "feedforward core call failed")
        return result

    def step_posterior_feedforward(self, observation, reference, compute_feedforward, *, planned_start_intent=0,
                                  reference_phase=0):
        """Compute one FF term at the core's current posterior insertion point.

        Callback exceptions become a native latched abort before a command token,
        then are re-raised here. No observer copy or pre-step read is involved.
        """
        require(self.native.controller_step_posterior_ff is not None,
                Reason.INTEGRATION_MISMATCH, "native library lacks posterior FF step; rebuild it")
        if type(planned_start_intent) is not int or planned_start_intent not in (-1, 0, 1):
            self.inhibit()
            require(False, Reason.DATA_INVALID, "planned departure intent must be -1, 0 or +1")
        require(not planned_start_intent or self.native.controller_step_posterior_ff_intent is not None,
                Reason.INTEGRATION_MISMATCH, "native library lacks additive departure intent; rebuild it")
        # Explicit offline phase: 0=legacy, 1=departure, 2=braking. The older
        # signed-intent interface remains additive and maps itself to departure.
        if type(reference_phase) is not int or reference_phase not in (0, 1, 2) or (
                reference_phase == 1 and not planned_start_intent or reference_phase == 2 and planned_start_intent):
            self.inhibit()
            require(False, Reason.DATA_INVALID, "invalid explicit reference phase/direction")
        require(not reference_phase or self.native.controller_step_posterior_ff_phase is not None,
                Reason.INTEGRATION_MISMATCH, "native library lacks additive reference phase; rebuild it")
        if self._posterior_call_active:
            self.inhibit()
            require(False, Reason.DATA_INVALID, "posterior FF call cannot re-enter its command owner")
        if not callable(compute_feedforward):
            self.inhibit()
            require(False, Reason.DATA_INVALID, "explicit posterior FF callback required")
        failures = []

        def callback(_context, posterior, native_reference, command):
            try:
                # Copies prevent retaining pointers to the C++ stack after return.
                state = CPosterior.from_buffer_copy(posterior.contents)
                packet = CReference.from_buffer_copy(native_reference.contents)
                value = compute_feedforward(state, packet)
                require(isinstance(value, (int, float)) and not isinstance(value, bool)
                        and np.isfinite(value), Reason.DATA_INVALID, "finite scalar posterior FF required")
                command[0] = value
                return 1
            except BaseException as exc:
                failures.append(exc)
                return 0

        bridge = POSTERIOR_FEEDFORWARD_CALLBACK(callback)
        result = COutput()
        self._posterior_call_active = True
        try:
            if reference_phase:
                accepted = self.native.controller_step_posterior_ff_phase(self.handle, ct.byref(observation),
                    ct.byref(reference), reference_phase, planned_start_intent, bridge, None, ct.byref(result))
            elif planned_start_intent:
                accepted = self.native.controller_step_posterior_ff_intent(self.handle, ct.byref(observation),
                    ct.byref(reference), planned_start_intent, bridge, None, ct.byref(result))
            else:
                accepted = self.native.controller_step_posterior_ff(self.handle, ct.byref(observation),
                    ct.byref(reference), bridge, None, ct.byref(result))
            require(bool(accepted),
                    Reason.INTEGRATION_MISMATCH, "posterior FF core call failed")
        finally:
            self._posterior_call_active = False
        if failures:
            raise failures[0]
        return result

    def inhibit(self):
        require(self.native.controller_inhibit is not None, Reason.INTEGRATION_MISMATCH,
                "native library lacks latched inhibit; rebuild it")
        require(bool(self.native.controller_inhibit(self.handle)), Reason.INTEGRATION_MISMATCH,
                "native inhibit failed")

    def read_parameters(self):
        require(self.native.controller_parameters is not None, Reason.INTEGRATION_MISMATCH,
                "native library lacks active parameter readback; rebuild it")
        result = CParameters()
        require(bool(self.native.controller_parameters(self.handle, ct.byref(result))),
                Reason.INTEGRATION_MISMATCH, "native parameter readback failed")
        return result

    def ack(self, result, successful=True, applied=None, *, accepted_time=None):
        current = result.limited if applied is None else applied
        if accepted_time is not None:
            require(self.native.controller_ack_at is not None, Reason.INTEGRATION_MISMATCH,
                    "actual accepted-time acknowledgement unavailable in this core")
            require(np.isfinite(accepted_time), Reason.DATA_INVALID, "finite accepted time required")
            self.current_history_provenance = "ACTUAL_ACCEPTED_TIME"
            return bool(self.native.controller_ack_at(self.handle, result.sequence, successful,
                                                     current, accepted_time))
        self.current_history_provenance = "LEGACY_DECISION_TIME"
        return bool(self.native.lib.ota_controller_ack(self.handle, result.sequence, successful, current))

    def switch(self, params, reference):
        return bool(self.native.lib.ota_controller_switch(self.handle, ct.byref(params), ct.byref(reference)))
