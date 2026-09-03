"""§80 — preserving a scene that cannot be reconstructed.

controld captures the relevant state the instant it stops believing what it was doing and
publishes it until someone takes it. Someone, here, is this module: it writes each new
capture to disk exactly once.

Why the *daemon* writes it and controld does not: writing a file is an unbounded-time
operation, and the thread that owns a 5 ms deadline must not do the disk's work. webd
already runs at telemetry rate, already has the parsed frame, and a slow or failing disk
must never become a reason the turret's control loop misses a cycle. If webd is down when
the fault happens, the scene is still on the socket stream and in controld's memory until
replaced — which is a real limitation, stated rather than hidden: a station that needs the
artifact guaranteed should run webd as a supervised service, not as something started by
hand.

The default directory is empty, which means *disabled*. Nothing about this behaviour
appears on a station until someone points it at a path. That is deliberate: writing files
is a side effect, and a side effect that arrives because code was merged is a surprise to
the person who has to explain it.
"""
from __future__ import annotations

import json
import logging
import os
import re
from dataclasses import asdict
from typing import Optional

log = logging.getLogger("ota.webd.blackbox")

_UNSAFE = re.compile(r"[^A-Za-z0-9._+-]+")


def _slug(text: str, limit: int = 40) -> str:
    cleaned = _UNSAFE.sub("_", (text or "scene").strip())[:limit]
    return cleaned.strip("._") or "scene"


class BlackBoxWriter:
    """Writes each new §80 capture to ``<directory>/blackbox_<id>_<reason>.json``.

    Use as the ``on_telemetry`` callback (compose with any existing callback — see
    ``app.py``), or call ``observe(telemetry)`` directly.
    """

    def __init__(self, directory: str) -> None:
        self.directory = (directory or "").strip()
        self._last_id = 0
        self.last_path: Optional[str] = None

    @property
    def enabled(self) -> bool:
        return bool(self.directory)

    def observe(self, telemetry) -> Optional[str]:
        """Return the path written, or None when there was nothing new to write."""
        capture_id = int(getattr(telemetry, "blackbox_capture_id", 0) or 0)
        if not self.enabled or capture_id <= 0:
            return None
        if capture_id <= self._last_id:
            # The scene is republished every frame on purpose (a 15 Hz reader must not
            # need to be unlucky to see it), so "same id again" is not news. Writing it
            # again would fill the disk with copies of one accident and bury the count of
            # how many accidents there were.
            return None
        body = getattr(telemetry, "blackbox", None)
        if not isinstance(body, dict) or not body:
            # An id with no object means the wire dropped the payload — most likely a
            # field that exists in controld but was never declared here. Say so loudly:
            # silently doing nothing is how this project lost `selected_track_id` for an
            # evening.
            log.error("black-box capture %d arrived with no payload; is `blackbox` "
                      "declared in protocol.py?", capture_id)
            return None

        path = os.path.join(
            self.directory,
            f"blackbox_{capture_id:04d}_{_slug(str(body.get('reason', '')))}.json",
        )
        try:
            os.makedirs(self.directory, exist_ok=True)
            # Write-then-rename, the same idiom controld uses for stored profiles: a file
            # that exists is a file that is complete. The artifact is the thing someone is
            # going to read *because* the turret misbehaved; a half-written one is worse
            # than none, because it looks like an answer.
            tmp = path + ".part"
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump({"schema": "ota-blackbox/1", **body}, fh, indent=2)
                fh.write("\n")
            os.replace(tmp, path)
        except OSError as exc:
            # A failure to preserve evidence must not take the dashboard down, and must
            # not be quiet about itself either.
            log.error("could not preserve black-box scene %d to %s: %s", capture_id,
                      path, exc)
            return None

        self._last_id = capture_id
        self.last_path = path
        log.warning("black-box scene %d preserved to %s (%s in %s, safety %s)",
                    capture_id, path, body.get("operating_mode"), body.get("phase"),
                    body.get("safety_action"))
        return path

    def __call__(self, telemetry) -> None:
        self.observe(telemetry)


def load(path: str) -> dict:
    """Read a preserved scene back (§81's starting point)."""
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)
