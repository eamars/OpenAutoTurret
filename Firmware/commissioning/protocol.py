"""Executable acquisition contract with a simulated, single-owner endpoint.

No SSH, SocketCAN, device opening or physical run implementation exists here.
The same state/receipt checks are exercised before Stage 2 supplies an adapter.
"""
from __future__ import annotations
from dataclasses import dataclass
import copy
import json
from pathlib import Path
import time

from .contracts import Reason, Rejected, digest, require


class Ownership:
    """OS exclusion plus monotonic lease and epoch; no second sender mutex."""
    def __init__(self, lock_path: Path, lease_s: float, clock=time.monotonic):
        require(lease_s > 0, Reason.DATA_INVALID, "positive lease required")
        self.path, self.lease_s, self.clock = lock_path, lease_s, clock
        self.stream = None; self.epoch = None; self.expires = 0.

    def acquire(self):
        import fcntl
        require(self.stream is None, Reason.DATA_INVALID, "owner already acquired")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        stream = self.path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            stream.close()
            raise Rejected(Reason.HARD_ABORT, "another output owner holds the OS lock") from exc
        stream.seek(0); raw = stream.read()
        try:
            previous = int(raw) if raw else 0
        except ValueError:
            stream.close(); raise Rejected(Reason.DATA_INVALID, "owner epoch record is corrupt")
        self.epoch = previous+1
        stream.seek(0); stream.truncate(); stream.write(str(self.epoch)); stream.flush()
        self.stream = stream; self.expires = self.clock()+self.lease_s
        return self.epoch

    def check(self, epoch):
        require(self.stream is not None and self.epoch == epoch and self.clock() < self.expires,
                Reason.HARD_ABORT, "owner/epoch/lease invalid; output inhibited")

    def renew(self, epoch):
        self.check(epoch); self.expires = self.clock()+self.lease_s

    def close(self):
        if self.stream:
            self.stream.close(); self.stream = None


@dataclass
class Field:
    unit: str
    writable: bool
    classification: str
    shape: tuple


class SimulatedEndpoint:
    """Full transactional protocol, with injected refusal/readback/dropout faults."""
    def __init__(self, owner, registry: dict[str, Field], values: dict, test_spec_hash: str, validator=None):
        require(set(registry) == set(values), Reason.DATA_INVALID, "registry bindings incomplete")
        self.owner, self.registry = owner, registry
        self.actual = copy.deepcopy(values); self.test_spec_hash = test_spec_hash
        self.pending = None; self.verified_hash = None; self.revision = 0
        self.acquisition_ready = False; self.active_case = None; self.run_count = 0
        self.capture = []; self.capture_capacity = 0; self.data_invalid = False
        self.failed_writes = set(); self.readback_overrides = {}
        self.validator=validator

    def describe(self):
        return {k: {**v.__dict__, "actual": copy.deepcopy(self.actual[k])}
                for k, v in self.registry.items()}

    def snapshot(self): return copy.deepcopy(self.actual)

    def prepare_profile(self, values, *, epoch):
        self.owner.check(epoch)
        self.verified_hash = None
        require(self.active_case is None, Reason.DATA_INVALID, "cannot change profile during a case")
        require(set(values) == set(self.registry), Reason.DATA_INVALID, "profile must bind every field")
        for name, value in values.items():
            field = self.registry[name]
            require(field.writable or value == self.actual[name], Reason.ENVELOPE_LIMITED,
                    f"{name} is an external constraint/read-only field")
            require(_shape(value) == field.shape, Reason.DATA_INVALID, f"{name} dimensions differ")
        digest(values)  # rejects NaN, infinity and non-JSON values
        if self.validator:self.validator(values)
        self.pending = copy.deepcopy(values)
        return digest(values)

    def apply_profile(self, *, epoch):
        self.owner.check(epoch)
        require(self.pending is not None, Reason.DATA_INVALID, "no prepared transaction")
        self.verified_hash = None
        # Hardware may partially write. A partial set is never committed or runnable.
        for name, value in self.pending.items():
            require(name not in self.failed_writes, Reason.DATA_INVALID, f"write refused: {name}")
            self.actual[name] = copy.deepcopy(value)

    def verify_profile(self, expected_hash, *, epoch):
        self.owner.check(epoch)
        observed = {**self.actual, **self.readback_overrides}
        if self.validator:self.validator(observed)
        require(self.pending is not None and digest(self.pending) == expected_hash and
                observed == self.pending and digest(observed) == expected_hash,
                Reason.DATA_INVALID, "actual readback differs from the requested complete profile")
        self.revision += 1; self.verified_hash = expected_hash; self.pending = None
        return {"applied": True, "readback_verified": True, "parameters_hash": expected_hash,
                "revision": self.revision, "owner_epoch": epoch}

    def prepare_capture(self, capacity):
        require(type(capacity) is int and capacity > 0, Reason.DATA_INVALID, "bounded capture required")
        self.capture=[];self.capture_capacity=capacity;self.data_invalid=False;self.acquisition_ready=True

    def run_case(self, case_id, expected_hash, expected_revision, test_spec_hash, *, epoch):
        self.owner.check(epoch)
        require(self.active_case is None and self.acquisition_ready and not self.data_invalid and
                self.verified_hash == expected_hash and digest(self.actual) == expected_hash and
                self.revision == expected_revision and test_spec_hash == self.test_spec_hash,
                Reason.DATA_INVALID, "RUN blocked: receipt/revision/capture/test identity mismatch")
        require(isinstance(case_id, str) and bool(case_id), Reason.DATA_INVALID, "case identity missing")
        self.active_case = case_id; self.run_count += 1

    def sample(self, row, *, epoch):
        self.owner.check(epoch)
        require(self.active_case is not None, Reason.DATA_INVALID, "sample without a running case")
        if len(self.capture) >= self.capture_capacity:
            self.data_invalid = True
            raise Rejected(Reason.DATA_INVALID, "bounded capture overflow; never silently drop data")
        self.capture.append(copy.deepcopy(row))

    def finish_case(self, *, epoch):
        self.owner.check(epoch)
        require(self.active_case is not None, Reason.DATA_INVALID, "no active case")
        report = {"case_id": self.active_case, "valid": not self.data_invalid,
                  "capture_hash": digest(self.capture), "execution": "SYNTHETIC"}
        self.active_case=None;self.acquisition_ready=False
        return report


def _shape(value):
    if isinstance(value, list):
        require(bool(value), Reason.DATA_INVALID, "empty parameter array")
        child = _shape(value[0])
        require(all(_shape(x) == child for x in value), Reason.DATA_INVALID, "ragged parameter array")
        return (len(value),)+child
    require(type(value) in (int, float, bool, str), Reason.DATA_INVALID, "unknown parameter value")
    return ()
