"""One HailoRT context per process, shared by every camera that infers on it.

Why this file exists at all: H5 added a second camera by building a second ``HailoYoloAdapter``,
and the second adapter opened a second ``VDevice`` on the same PCIe chip. The device answered
``HAILO_DEVICE_IN_USE(73)``, the adapter raised, and the daemon did exactly what it is supposed to
do on a model refusal — ``EXIT_MODEL`` — which took the previews down with it. A per-camera adapter
was the right call; a per-camera *device* was the bug, and the two words are one letter apart, so
the difference is written down here instead of being remembered.

Two facts decide the design:

* One physical Hailo-8 tolerates **one** HailoRT ``VDevice``. So the device object is created once
  and both cameras borrow it.
* The network runs on one artifact and one input shape, so two cameras means two *calls*, and calls
  on one accelerator are serialised whether I organise that or not. Organising it buys three
  things: an order I can state (strict alternation while both are waiting), a queue wait measured
  per feed instead of absorbed into the caller's latency, and a contention count that says whether
  the ordering ever mattered. Without those three, "fairness >= 0.9" is a hope, not a reading.

The turnstile, in words: a camera may take the turn when nobody holds it. On release, if another
camera is waiting, the turn goes to it; otherwise the same camera keeps going, because an idle
accelerator serves nobody. A holder never waits, so no ordering of arrivals can deadlock it, and a
throwing ``infer()`` still releases through ``finally``.

Two identities, because they arrive at different times: a *member* is an adapter that registered on
this device (known at ``open()``), a *camera* is what a frame is attributed to (known at ``run()``,
after the daemon has identified the sensor). Counting is per camera; refusal messages name both.
"""
from __future__ import annotations

import hashlib
import os
import threading
import time
from contextlib import ExitStack
from typing import Any, Dict, List, Optional

from ..errors import ModelRejected

UNBOUND = "(unbound)"


class HailoDevice:
    """The accelerator itself: opened once, turned over one inference at a time."""

    def __init__(self) -> None:
        self._stack: Optional[ExitStack] = None
        self._infer: Any = None
        self._input_name = ""
        self._output_name = ""
        self._device_id = ""
        self._architecture = ""
        self._artifact_path = ""
        self._artifact_sha256 = ""
        self._members: List[str] = []                # adapters registered on this device
        self._cond = threading.Condition()
        self._turn: Optional[str] = None             # whose frame is on the chip right now
        self._want: Dict[str, bool] = {}             # who is asking, by camera id
        self._closed = False
        # Counters, keyed by camera id: a claim about one feed is answered with that feed's numbers.
        self._served: Dict[str, int] = {}
        self._failures: Dict[str, int] = {}
        self._queue_wait_ms: Dict[str, float] = {}
        self._device_ms: Dict[str, float] = {}
        self._contested = 0                          # turns passed while another camera also wanted one

    # --- opening ---------------------------------------------------------------

    @classmethod
    def for_testing(cls, infer: Any, *, device_id: str = "fake:0",
                    architecture: str = "HAILO8", artifact_sha256: str = "0" * 64,
                    input_name: str = "input_layer", output_name: str = "output0",
                    member: str = "") -> "HailoDevice":
        """A device with a stand-in runtime: the fairness mechanism is testable off the station.

        The fake stands in for ``InferVStreams`` and nothing else. The turnstile, the counters and
        the join-time refusals — the code this file was written for — are the shipped ones.

        ``member`` seeds one adapter as already registered, which is how a test exercises the
        *second* camera's path (join an open device) without a chip under it.
        """
        device = cls()
        device._infer = infer
        device._device_id = device_id
        device._architecture = architecture
        device._artifact_sha256 = artifact_sha256
        device._artifact_path = "fake.hef"
        device._input_name = input_name
        device._output_name = output_name
        if member:
            device._members.append(member)
        return device

    def open_for(self, member: str, *, artifact_path: str, expected_sha: str,
                 require_input: tuple, profile: str) -> None:
        """First member opens the chip; a later one joins it or is refused with a reason.

        ``member`` names who is asking — a camera id when it is known, otherwise the profile —
        because "the second camera could not open" tells the next reader nothing while "detail: one
        Hailo context runs one artefact, the first opened X and this manifest pins Y" tells them
        exactly what to look at.
        """
        with self._cond:
            if self._closed:
                raise ModelRejected(f"{member}: the Hailo device is closed")
            if member in self._members:
                raise ModelRejected(f"{member}: already registered on this Hailo device; one "
                                    "adapter cannot open the same device twice")
            if self._members:
                self._join(member, artifact_path, expected_sha)
                return
        # First member: the slow work happens outside the lock, then the join is recorded.
        self._open_runtime(artifact_path, expected_sha, require_input, profile)
        with self._cond:
            self._members.append(member)

    def _join(self, member: str, artifact_path: str, expected_sha: str) -> None:
        if expected_sha and self._artifact_sha256 and expected_sha != self._artifact_sha256:
            raise ModelRejected(
                f"{member}: one Hailo context runs one artefact. The first camera opened "
                f"{self._artifact_path} ({self._artifact_sha256[:12]}) and this manifest pins "
                f"{expected_sha[:12]}. Two networks need two devices, not two guesses")
        if not os.path.exists(artifact_path):
            raise ModelRejected(f"{member}: {artifact_path} does not exist")
        self._members.append(member)

    def _open_runtime(self, artifact_path: str, expected_sha: str,
                      require_input: tuple, profile: str) -> None:
        digest = hashlib.sha256()
        with open(artifact_path, "rb") as artifact:
            for block in iter(lambda: artifact.read(1024 * 1024), b""):
                digest.update(block)
        self._artifact_sha256 = digest.hexdigest()
        if self._artifact_sha256 != expected_sha:
            raise ModelRejected(
                f"Hailo HEF SHA-256 mismatch for {artifact_path}: expected {expected_sha}, "
                f"got {self._artifact_sha256}")

        stack = ExitStack()
        try:
            import hailo_platform as runtime
            Device, HEF, VDevice = runtime.Device, runtime.HEF, runtime.VDevice
            ConfigureParams, FormatType = runtime.ConfigureParams, runtime.FormatType
            HailoStreamInterface = runtime.HailoStreamInterface
            InferVStreams, InputVStreamParams, OutputVStreamParams = (
                runtime.InferVStreams, runtime.InputVStreamParams, runtime.OutputVStreamParams)

            device_ids = Device.scan()
            if len(device_ids) != 1:
                raise ModelRejected(f"expected one Hailo device, found {len(device_ids)}: {device_ids}")
            with Device(device_ids[0]) as physical:
                board = physical.control.identify()
                self._architecture = str(board.device_architecture)
            if self._architecture != "HAILO8":
                raise ModelRejected(f"pinned HEF targets HAILO8; device reports {self._architecture}")

            hef = HEF(artifact_path)
            inputs = hef.get_input_vstream_infos()
            outputs = hef.get_output_vstream_infos()
            if len(inputs) != 1 or len(outputs) != 1:
                raise ModelRejected(
                    f"expected one input and output vstream; got {len(inputs)} and {len(outputs)}")
            if tuple(inputs[0].shape) != tuple(require_input):
                raise ModelRejected(
                    f"HEF input shape is {inputs[0].shape}, expected {tuple(require_input)}; "
                    f"the {profile} manifest was written against that shape")
            self._input_name, self._output_name = inputs[0].name, outputs[0].name

            vdevice = stack.enter_context(VDevice(device_ids=device_ids))
            configure = ConfigureParams.create_from_hef(hef, HailoStreamInterface.PCIe)
            groups = vdevice.configure(hef, configure)
            if len(groups) != 1:
                raise ModelRejected(f"expected one Hailo network group, got {len(groups)}")
            network_group = groups[0]
            input_params = InputVStreamParams.make(
                network_group, quantized=True, format_type=FormatType.UINT8)
            output_params = OutputVStreamParams.make(
                network_group, quantized=False, format_type=FormatType.FLOAT32)
            self._infer = stack.enter_context(InferVStreams(
                network_group, input_params, output_params))
            stack.enter_context(network_group.activate(network_group.create_params()))
            self._stack = stack
            self._artifact_path = artifact_path
            self._device_id = str(device_ids[0])
        except ModelRejected:
            stack.close()
            raise
        except Exception as exc:  # noqa: BLE001 - any runtime failure is a model refusal
            stack.close()
            raise ModelRejected(f"HailoRT could not open {artifact_path}: {exc}") from exc

    # --- the turnstile ---------------------------------------------------------

    def run(self, tensor: Any, *, camera_id: str) -> Any:
        """One frame's worth of accelerator, taken in turn. Blocks until it is this camera's turn.

        The raw NMS output comes back; the caller owns decoding, because the padding being undone
        was added by that caller. ``queue_wait_ms`` and ``device_ms`` are recorded apart: the first
        is what sharing costs this feed, the second is the chip, and one blended number hides which
        of the two to act on.
        """
        key = (camera_id or "").strip() or UNBOUND
        if self._infer is None:
            raise ModelRejected("HailoDevice.run() before open_for()")
        started_waiting = time.monotonic_ns()
        with self._cond:
            if self._closed:
                raise ModelRejected(f"{key}: the Hailo device is closed")
            if key == UNBOUND and any(c != UNBOUND for c in self._want):
                raise ModelRejected(
                    "a frame reached the shared Hailo device with no camera_id while another camera "
                    "is already being counted there. One device, two feeds: without an owner on the "
                    "frame the per-camera rate would be a guess")
            if key not in self._want:
                self._want[key] = False
                self._served.setdefault(key, 0)
                self._failures.setdefault(key, 0)
            self._want[key] = True
            while self._turn is not None and self._turn != key:
                self._cond.wait(0.5)
                if self._closed:
                    self._want[key] = False
                    raise ModelRejected(f"{key}: the Hailo device closed while waiting")
            self._queue_wait_ms[key] = (time.monotonic_ns() - started_waiting) / 1_000_000.0
            self._turn = key
            self._want[key] = False
        started = time.monotonic_ns()
        try:
            result = self._infer.infer({self._input_name: tensor})
        except Exception as exc:  # noqa: BLE001 - counted per camera, then raised as a refusal
            with self._cond:
                self._failures[key] = self._failures.get(key, 0) + 1
                if self._turn == key:
                    self._release_locked(key)
            raise ModelRejected(f"Hailo inference failed: {exc}") from exc
        with self._cond:
            self._device_ms[key] = (time.monotonic_ns() - started) / 1_000_000.0
            self._release_locked(key)
        return result

    def _release_locked(self, key: str) -> None:
        """Hand the accelerator on. Under contention the camera that waited goes next, always."""
        waiting = [other for other in self._want if other != key and self._want[other]]
        self._served[key] = self._served.get(key, 0) + 1
        if waiting and len(self._want) > 1:
            self._contested += 1
        self._turn = waiting[0] if waiting else None
        self._cond.notify_all()

    # --- what to publish -------------------------------------------------------

    def facts_for(self, camera_id: str) -> Dict[str, Any]:
        """The device's facts plus this camera's own numbers. Never another feed's."""
        key = (camera_id or "").strip() or UNBOUND
        with self._cond:
            return {"hailo_device_id": self._device_id,
                    "device_architecture": self._architecture,
                    "artifact_path": self._artifact_path,
                    "artifact_sha256": self._artifact_sha256,
                    "shared_with": sorted(c for c in self._want if c != key),
                    "members": len(self._members),
                    "queue_wait_ms": round(float(self._queue_wait_ms.get(key, 0.0)), 3),
                    "device_ms": round(float(self._device_ms.get(key, 0.0)), 3),
                    "device_served": int(self._served.get(key, 0)),
                    "device_failures": int(self._failures.get(key, 0)),
                    "turn_contested": self._contested}

    def cameras(self) -> List[str]:
        with self._cond:
            return sorted(self._want)

    def release(self, member: str) -> None:
        """One adapter is done with the device. The last one out closes the chip."""
        with self._cond:
            if member in self._members:
                self._members.remove(member)
            if self._members:
                return
            self._closed = True
            self._cond.notify_all()
        stack, self._stack = self._stack, None
        self._infer = None
        if stack is not None:
            stack.close()

    def close(self) -> None:
        self._closed = True
        with self._cond:
            self._members = []
            self._turn = None
            self._cond.notify_all()
        stack, self._stack = self._stack, None
        self._infer = None
        if stack is not None:
            stack.close()

    @property
    def opened(self) -> bool:
        return self._infer is not None

    @property
    def output_name(self) -> str:
        """The HEF's output vstream name, which the caller needs to read the result it was handed."""
        return self._output_name

    @property
    def artifact_sha256(self) -> str:
        return self._artifact_sha256
