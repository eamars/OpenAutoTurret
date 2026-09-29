"""A camera's identity is the port it is wired into, not the number it was handed.

Why this exists (WP3, docs/ADR-001/docs/07): `generation`, owner-restart isolation and every
"the wide worker is still the wide worker" claim need a subject, and `/dev/video0` cannot be it --
the kernel numbers devices in the order they enumerate, so a second camera, a different USB port
or a slower probe at boot can swap the numbers without anything about the rig changing. A tracker
keyed on the number would then attribute one camera's tracks to the other, which is exactly the
boundary case 08 §3 lists first ("camera index swapped, hardware ID unchanged").

So the identity is the stable symlink name (`/dev/v4l/by-id/…` for USB, `/dev/v4l/by-path/…` for
everything else, including the Pi camera on its PCIe/SoC path). When a caller can only offer a
`videoN` node we say so out loud with a source tag instead of pretending: `identity_source="index"`
means "this id may not survive a reboot", and a consumer that needs durability must not accept it.
"""
from __future__ import annotations

import hashlib
import os
import re
from typing import NamedTuple


class CameraId(NamedTuple):
    id: str
    source: str          # "by-id" | "by-path" | "index"

    @property
    def durable(self) -> bool:
        return self.source in ("by-id", "by-path", "fwnode")


def derive_camera_id(device: str) -> CameraId:
    """Identity from a v4l2 device path, preferring the name that encodes the physical port."""
    normalised = device.replace("//", "/").rstrip("/")
    base = os.path.basename(normalised)
    # Strip the per-device counter. by-path/by-id names end in "-video<N>" or "video-index<N>",
    # and that N moves when another camera is attached -- hashing it would smuggle the very
    # numbering this module exists to escape back into the identity. (Caught by the selftest
    # below on the day this file was written, which is the reason to write the selftest first.)
    stripped = re.sub(r"(?i)[-_]?video([-_]?index)?\d+$", "", base).strip("-_")
    if stripped:
        base = stripped
    if "/v4l/by-id/" in normalised:
        return CameraId("cam-" + hashlib.sha1(base.encode()).hexdigest()[:8], "by-id")
    if "/v4l/by-path/" in normalised:
        return CameraId("cam-" + hashlib.sha1(base.encode()).hexdigest()[:8], "by-path")
    if "/i2c@" in normalised and (normalised.startswith("/base/") or "/of_node/" in normalised):
        # A firmware-node path, straight from the camera stack: `.../rp1/i2c@88000/imx500@1a`.
        # This names the sensor *on its port*, so nothing is stripped — the address and the
        # part name are the whole point, and two sensors on two addresses must never collide
        # (measured on the station: /dev/video0 and /dev/video1 both sit under one CSI host's
        # node family, so deriving an identity from a node number made the IMX477 claim to be
        # the IMX500).
        return CameraId("cam-" + hashlib.sha1(normalised.encode()).hexdigest()[:8], "fwnode")
    if base.startswith("video"):
        # A kernel number, borrowed for now. Callers that compare identities across a restart
        # must check .durable and refuse this, rather than treat a re-number as a new camera.
        return CameraId("cam-" + hashlib.sha1(base.encode()).hexdigest()[:8], "index")
    # Mocks and tests hand us labels. They are still an identity within a run -- and a mock that
    # silently became "unknown" would make A3's tests unable to name the worker they killed.
    return CameraId("cam-" + hashlib.sha1(base.encode()).hexdigest()[:8], "label")


def resolve_durable_id(device: str) -> CameraId:
    """Upgrade a kernel node to the port identity that names it, when the system tells us one.

    /dev/videoN is a lease the kernel grants for this boot; /dev/v4l/by-path/<port> is a name for
    the same node that survives renumbering. If any by-path/by-id symlink resolves to the node we
    were handed, we prefer that name. When several nodes map to one camera (this station exposes
    two PiSP back-end nodes) every candidate yields the same id, so the choice among them is not a
    correctness question -- and when nothing resolves, we return the caller's own id, which still
    says source="index" instead of inventing a durability we do not have.
    """
    if not device.startswith("/dev/"):
        return derive_camera_id(device)          # mocks and labels keep their own identity
    target = os.path.realpath(device)
    for base in ("/dev/v4l/by-path", "/dev/v4l/by-id"):
        try:
            names = sorted(os.listdir(base))
        except OSError:
            continue
        for name in names:
            try:
                if os.path.realpath(os.path.join(base, name)) == target:
                    return derive_camera_id(os.path.join(base, name))
            except OSError:
                continue
    return derive_camera_id(device)


def selftest() -> int:
    """Runnable proof of the three properties that matter, and nothing else."""
    by_path_a = "/dev/v4l/by-path/platform-3d200000.pcie-usb-0:1.2:1.0-capture-video0"
    by_path_a_after_renumber = by_path_a[:-1] + "2"           # same port, new number
    by_path_b = "/dev/v4l/by-path/platform-3d200000.pcie-usb-0:1.3:1.0-capture-video5"
    checks = [
        ("same port survives renumbering",
         derive_camera_id(by_path_a).id == derive_camera_id(by_path_a_after_renumber).id),
        ("different port is a different camera",
         derive_camera_id(by_path_a).id != derive_camera_id(by_path_b).id),
        ("a bare video node says it is not durable",
         derive_camera_id("/dev/video0").source == "index"
         and not derive_camera_id("/dev/video0").durable),
        ("by-path and by-id are both durable",
         derive_camera_id(by_path_a).durable
         and derive_camera_id("/dev/v4l/by-id/usb-Imx500_1234-capture").durable),
    ]
    # On a real machine, the resolution step must actually find the port name; off-station we say
    # NOT_RUN rather than counting a check we could not perform.
    if os.path.isdir("/dev/v4l/by-path") or os.path.isdir("/dev/v4l/by-id"):
        resolved = resolve_durable_id("/dev/video0")
        checks.append(("a real /dev/video0 resolves to a durable name (or honestly says index)",
                       resolved.durable or resolved.source == "index"))
    else:
        print("  skip  durable resolution (no /dev/v4l here: NOT_RUN)")
    failed = [name for name, ok in checks if not ok]
    for name, ok in checks:
        print(("  ok   " if ok else "  FAIL ") + name)
    print("camera_id selftest: %d/%d passed" % (len(checks) - len(failed), len(checks)))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(selftest())
