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


class CParameters(ct.Structure):
    _fields_ = [("model", CModel), ("observer", CObserver)] + [(k, ct.c_double) for k in CONTROL_FIELDS] + [
        ("start_total", ct.c_double*48), ("start_censored", ct.c_int*48)]


class CObservation(ct.Structure):
    _fields_ = [(k, ct.c_double) for k in ("now", "encoder_time", "gyro_time", "position", "gyro_rate")] + [
        (k, ct.c_uint64) for k in ("encoder_seq", "gyro_seq", "generation")] + [
        ("encoder_valid", ct.c_int), ("gyro_valid", ct.c_int)]


class CReference(ct.Structure):
    _fields_ = [(k, ct.c_double) for k in ("position", "velocity", "acceleration", "posture")]


class COutput(ct.Structure):
    _fields_ = [(k, ct.c_double) for k in ("requested", "limited", "position", "velocity",
        "integral", "feedforward", "start_increment")] + [("sequence", ct.c_uint64)] + [
        (k, ct.c_int) for k in ("status", "motion", "encoder_only")]


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
        require(self.lib.ota_core_abi() == 2, Reason.INTEGRATION_MISMATCH, "core ABI differs")
        self.lib.ota_model_rollout.argtypes = [ct.POINTER(CModel), ct.c_int,
            *([ct.POINTER(ct.c_double)] * 3), ct.POINTER(ct.c_int), ct.c_double,
            ct.c_double, ct.POINTER(ct.c_double)]
        self.lib.ota_model_rollout.restype = ct.c_int
        self.lib.ota_controller_create.argtypes = [ct.POINTER(CParameters)]
        self.lib.ota_controller_create.restype = ct.c_void_p
        self.lib.ota_controller_destroy.argtypes = [ct.c_void_p]
        self.lib.ota_controller_reset.argtypes = [ct.c_void_p, *([ct.c_double]*4), ct.c_uint64]
        self.lib.ota_controller_step.argtypes = [ct.c_void_p, ct.POINTER(CObservation),
                                                ct.POINTER(CReference), ct.POINTER(COutput)]
        self.lib.ota_controller_ack.argtypes = [ct.c_void_p, ct.c_uint64, ct.c_int, ct.c_double]
        self.lib.ota_controller_switch.argtypes = [ct.c_void_p, ct.POINTER(CParameters), ct.POINTER(CReference)]
        self.lib.ota_closed_rollout.argtypes = [ct.POINTER(CParameters), ct.POINTER(CParameters),
            ct.POINTER(Simulation), ct.c_int, ct.POINTER(CReference), ct.c_double, ct.c_double,
            ct.POINTER(ct.c_double)]

    def rollout(self, spec: ModelSpec, theta, t, tx, z, direction, initial):
        n = len(t)
        vectors = [np.ascontiguousarray(array(v, (n,), name)) for v, name in
                   ((t, "time"), (tx, "successful TX"), (z, "posture"))]
        dirs = np.ascontiguousarray(direction, dtype=np.int32)
        require(dirs.shape == (n,) and np.isin(dirs, [-1, 1]).all(), Reason.DATA_INVALID,
                "invalid running direction")
        out = np.empty((n, 2), dtype=np.float64)
        m = model(spec, theta)
        status = self.lib.ota_model_rollout(ct.byref(m), n,
            *(x.ctypes.data_as(ct.POINTER(ct.c_double)) for x in vectors),
            dirs.ctypes.data_as(ct.POINTER(ct.c_int)), *initial,
            out.ctypes.data_as(ct.POINTER(ct.c_double)))
        require(status == 0, Reason.MODEL_INADEQUATE if status == 2 else Reason.DATA_INVALID,
                f"native rollout rejected input/domain (status={status})")
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


def parameters(spec, theta, observer, values, start_total, start_censored):
    require(set(values) == set(CONTROL_FIELDS), Reason.DATA_INVALID,
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
    return CParameters(model(spec, theta), cobs, *(values[k] for k in CONTROL_FIELDS),
                       (ct.c_double*48)(*starts.ravel()), (ct.c_int*48)(*(int(v) for v in censored.ravel())))


class Controller:
    def __init__(self, native, params):
        self.native, self.params = native, params
        self.handle = native.lib.ota_controller_create(ct.byref(params))
        require(bool(self.handle), Reason.DATA_INVALID, "C++ controller rejected complete parameter set")

    def __enter__(self): return self
    def __exit__(self, *args): self.close()
    def close(self):
        if self.handle:
            self.native.lib.ota_controller_destroy(self.handle); self.handle = None

    def reset(self, t, q, v, current, generation=1):
        require(bool(self.native.lib.ota_controller_reset(self.handle, t, q, v, current, generation)),
                Reason.DATA_INVALID, "controller initialization rejected")

    def step(self, observation, reference):
        result = COutput()
        require(bool(self.native.lib.ota_controller_step(self.handle, ct.byref(observation),
                ct.byref(reference), ct.byref(result))), Reason.INTEGRATION_MISMATCH, "core call failed")
        return result

    def ack(self, result, successful=True, applied=None):
        return bool(self.native.lib.ota_controller_ack(self.handle, result.sequence, successful,
                    result.limited if applied is None else applied))

    def switch(self, params, reference):
        return bool(self.native.lib.ota_controller_switch(self.handle, ct.byref(params), ct.byref(reference)))
