"""§20 of the v3.2 revision, checked against what the page actually receives.

The existing ledger walks the v3 architecture document's runtime schema. Nothing in the suite walked
§20 of the revision - so the list the revision actually demands could shrink, drift, or go undelivered
and every test would stay green. That is how "the data contract is nearly complete" became a feeling
rather than a fact.

So the names are parsed out of the revision file itself, and every one of them must be either

  - mapped: a path that resolves in the payload the page reads, optionally with the claim that the
    dashboard visibly reads it, or
  - declared: a named absence, either a key that is not on the wire or one that is present and
    honestly null, with a reason.

Anything else fails. A mapped name that stops resolving is a regression, and a declared gap that
quietly fills in is drift in the other direction - both mean the ledger and the system have parted
company, which is the only failure mode a document-as-truth test can have.

Nulls are tracked separately from absences because they mean different things. `camera.measurement_age_ms`
arrives as JSON null: the field exists and its answer is "unknown". Absence would mean the daemon never
answered. Only the first one is honest, and only a test that distinguishes them can tell them apart.
"""

from __future__ import annotations

import json
import os
import re
import tempfile
import time
import unittest
from pathlib import Path

from fastapi.testclient import TestClient

from ..app import create_app
from ..controld_client import ControldClient
from ..config import WebConfig
from ..fake_controld import FakeControld
from ..hud import HUD_JS
from . import fake_camera

DOC = Path(__file__).resolve().parents[3] / "docs" / "open_auto_turret_v3_2_apache_hud_ui_revision.md"


def _section_20_names() -> list:
    """The field list from §20's fenced block, read from the document every round."""
    text = DOC.read_text(encoding="utf-8")
    fences = re.findall(r"```[a-zA-Z]*\n(.*?)```", text, flags=re.S)
    block = [b for b in fences if "system.connected" in b and "prediction.valid" in b]
    assert len(block) == 1, (
        "§20's field list has moved, been duplicated, or been removed from the revision; this test is "
        "worth nothing if it silently stops reading it (found %d candidate blocks)" % len(block))
    names = []
    for line in block[0].splitlines():
        line = line.strip()
        # Two OR three dotted segments. The first version allowed only two and so silently skipped
        # every `axes.yaw.actual_deg` style name - a parser that quietly reads less than the document
        # says is the one failure mode this file exists to prevent, found here by the mismatch check
        # it was written for rather than by reading the list by eye.
        if re.match(r"^[a-z][a-z0-9_]*(\[\])?(\.[a-z][a-z0-9_]*){1,2}(\[\])?$", line):
            names.append(line)
    assert len(names) >= 25, "only %d names parsed from §20; the block is not what this test expects" % len(names)
    return names


# name -> (kind, path, note). kind:
#   "path"  - must resolve in the payload
#   "null"  - key must be present and None (an answer of "unknown", not a missing answer)
#   "absent"- key must not be present at all
# The `page` set below lists the names whose values the dashboard visibly reads.
LEDGER = {
    "system.mode_phase": ("path", "mode_phase", "the FSM phase the mode block shows"),
    "system.operating_mode": ("path", "operating_mode", ""),
    "system.safety_action": ("path", "safety_action", "§22's safety presentation reads this"),
    "system.connected": ("path", "controld_connected",
                         "webd's own view of the daemon link. It lives on /api/health, and the §8 chip "
                         "read it off /api/state, where it was never present - so a working station "
                         "showed a permanent red CONNECTED. Both endpoints now report the same "
                         "client.connected() call: one fact behind two spellings."),
    "system.homed": ("path", "at_ready", "the same fact the mode block turns into READY"),

    "axes.yaw.actual_deg": ("path", "q_yaw_rad", "logical joint travel in radians; §5.3 forbids calling it a heading"),
    "axes.yaw.soft_min_deg": ("path", "q_soft_min_yaw_rad", "radians, converted by the page"),
    "axes.yaw.soft_max_deg": ("path", "q_soft_max_yaw_rad", "radians, converted by the page"),
    "axes.pitch.actual_deg": ("path", "q_pitch_rad", "logical joint travel in radians"),
    "axes.pitch.soft_min_deg": ("path", "q_soft_min_pitch_rad", "radians, converted by the page"),
    "axes.pitch.soft_max_deg": ("path", "q_soft_max_pitch_rad", "radians, converted by the page"),

    "camera.effective_hfov_deg": ("path", "camera.effective_hfov_deg", "commissioned by the encoder-as-theodolite pass"),
    "camera.effective_vfov_deg": ("path", "camera.effective_vfov_deg", "commissioned by the encoder-as-theodolite pass"),
    "camera.fps": ("path", "camera.fps", "inter-TrackSet cadence: the only camera rate the daemon can observe"),
    "camera.measurement_age_ms": ("path", "camera.measurement_age_ms",
                                 "controld publishes the calibration file's own mtime aged per snapshot and "
                                 "says so as null where there is no calibration file; sent as null rather "
                                 "than 0, because 0 would claim freshly commissioned geometry"),

    "field_of_regard.safe_envelope_points[]": ("path", "field_of_regard.safe_envelope_points",
                                               "joint degrees, §11.3; four corners from the measured soft travel"),

    "target_selection.selected_track_uuid": ("path", "selected_uuid", "gated by selected_uuid_valid"),
    "target_selection.selected_label": ("path", "selected_display_index", "the label §17 asks the page to show"),
    "target_selection.visibility": ("path", "selection_visibility", "which edge of the frame the target sits on"),
    "target_selection.confidence": ("path", "selected_confidence", ""),

    "tracks[].track_uuid": ("track", "uuid", ""),
    "tracks[].display_label": ("track", "display_index", "plus label, which carries the class name"),
    "tracks[].confidence": ("track", "confidence", ""),
    "tracks[].state": ("track", "state", ""),
    "tracks[].bbox_norm": ("track", "bbox", "normalised image corners, as §11.3 wants for overlays"),
    "tracks[].anchor_norm": ("track", "anchor_x", "two flat fields, anchor_x and anchor_y, not a pair"),

    "prediction.valid": ("path", "prediction.valid", ""),
    "prediction.predicted_anchor_norm": ("path", "prediction.predicted_anchor_norm", ""),
    "prediction.predicted_los_yaw_deg": ("path", "prediction.predicted_los_yaw_deg", "camera-frame LOS"),
    "prediction.predicted_los_pitch_deg": ("path", "prediction.predicted_los_pitch_deg", "camera-frame LOS"),
    "prediction.horizon_ms": ("path", "prediction.horizon_ms", "control delay + motor response, 40 ms on this station"),

    "imu.present": ("path", "imu.present", "false: no inertial sensor on this station, operator-confirmed"),
    "imu.gravity_valid": ("path", "imu.gravity_valid", "false, and it cannot be otherwise with no sensor"),
    "imu.world_elevation_deg": ("null", "imu.world_elevation_deg",
                                "no sensor, so no value. 0.0 would assert the turret is level, which is a "
                                "safety claim nothing on this station is entitled to make"),
}

# Names the dashboard visibly reads. Checked by looking for the field in the page's own script, because
# "published" and "visible" have been confused on this project before.
PAGE_READ = {
    "axes.yaw.actual_deg": "t.q_yaw_rad",
    "system.connected": "t.controld_connected",
    "system.safety_action": "t.safety_action",
    "camera.fps": "t.camera_fps",
    "camera.effective_hfov_deg": "t.effective_hfov_deg",
}

# A sample track so the per-item names have something to resolve against.
SAMPLE_TRACK = {
    "track_id": 7, "uuid": "7f3c9a5e-1b2d-4a6e-9c8d-0e1f2a3b4c5d", "display_index": 1,
    "label": "person", "class_name": "person", "state": "TRACKING", "confidence": 0.83,
    "anchor_x": 0.512, "anchor_y": 0.478, "bbox": [0.47, 0.40, 0.56, 0.56],
    "selectable": True, "selected": True,
}


def _resolve(payload: dict, dotted: str):
    cur = payload
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None, False
        cur = cur[part]
    return cur, True


class Section20Ledger(unittest.TestCase):
    maxDiff = None

    @classmethod
    def setUpClass(cls) -> None:
        cls.names = _section_20_names()

    def test_every_name_is_ledgered_and_every_ledger_entry_is_still_asked_for(self) -> None:
        doc, ledger = set(self.names), set(LEDGER)
        untracked = sorted(doc - ledger)
        self.assertFalse(untracked,
                         "§20 names these and this test says nothing about them: %s. Either map them "
                         "to a published path or declare the gap with a reason." % untracked)
        stale = sorted(k for k in ledger if k not in doc and k.replace("[]", "") not in doc
                       and k.replace(".[]", "[]") not in doc)
        self.assertFalse(stale,
                         "the ledger tracks names §20 no longer asks for; a ledger that outlives the "
                         "document stops being a check: %s" % stale)

    def test_the_document_still_names_the_things_this_ledger_claims(self) -> None:
        # Cheap and deliberate: if someone edits the doc's list rather than the system, this test must
        # notice that a requirement disappeared, instead of quietly passing on a shorter list.
        for must in ("system.connected", "camera.fps", "field_of_regard.safe_envelope_points[]",
                     "prediction.valid", "imu.world_elevation_deg"):
            self.assertIn(must, self.names, "§20 no longer lists %s" % must)

    def test_every_declared_gap_has_a_reason(self) -> None:
        for name, (kind, path, note) in LEDGER.items():
            if kind in ("null", "absent"):
                self.assertTrue(len(note.strip()) > 20,
                                "%s is declared %s without explaining why" % (name, kind))


class Section20AgainstThePayload(unittest.TestCase):
    """The names resolved against the payload the page reads, through the real client."""

    def setUp(self) -> None:
        self.tc = fake_camera.install(frame_delay=0.0)
        self._tmp = tempfile.TemporaryDirectory(prefix="ota_webd_ledger_")
        self.sock = os.path.join(self._tmp.name, "controld.sock")
        self.fake = FakeControld(self.sock, telemetry_hz=20.0)
        self.fake.set_telemetry(
            track_state="tracking", tracking_active=True, track_count=1,
            tracks=[dict(SAMPLE_TRACK)],
            prediction={"valid": True, "predicted_los_yaw_deg": 3.4, "predicted_los_pitch_deg": -1.1,
                        "predicted_anchor_norm": [0.531, 0.492], "anchor_in_frame": True,
                        "horizon_ms": 40},
        )
        self.fake.start()
        self.client = ControldClient(self.sock, reconnect_interval=0.05)
        self.app = create_app(self.client, WebConfig(host="127.0.0.1", port=0, socket_path=self.sock))
        self.http = TestClient(self.app)
        self.http.__enter__()
        deadline = time.time() + 5.0
        while time.time() < deadline:
            body = self.http.get("/api/state").json()
            if isinstance(body.get("prediction"), dict) and isinstance(body.get("camera"), dict):
                break
            time.sleep(0.05)
        self.body = self.http.get("/api/state").json()

    def tearDown(self) -> None:
        try:
            self.http.__exit__(None, None, None)
        finally:
            self.client.stop()
        self.fake.stop()
        fake_camera.restore(self.tc)
        self._tmp.cleanup()

    def test_every_mapped_name_resolves(self) -> None:
        missing = []
        for name, (kind, path, _note) in LEDGER.items():
            if kind == "path":
                value, found = _resolve(self.body, path)
                if not found:
                    missing.append("%s -> %s" % (name, path))
                elif value is None:
                    missing.append("%s -> %s is null but the ledger calls it a value" % (name, path))
        self.assertFalse(missing, "these §20 fields do not arrive: %s" % missing)

    def test_declared_nulls_are_null_and_not_zero(self) -> None:
        for name, (kind, path, note) in LEDGER.items():
            if kind != "null":
                continue
            value, found = _resolve(self.body, path)
            self.assertTrue(found, "%s is declared null but the key is absent; absence and 'unknown' "
                                   "are different answers (%s)" % (name, note))
            self.assertIsNone(value, "%s should be null; it is %r" % (name, value))
            self.assertNotEqual(value, 0, "%s flattened to a number" % name)

    def test_track_item_names_resolve(self) -> None:
        tracks = self.body.get("tracks")
        self.assertIsInstance(tracks, list)
        self.assertTrue(tracks, "no tracks to check the per-item names against")
        for name, (kind, key, _note) in LEDGER.items():
            if kind != "track":
                continue
            self.assertIn(key, tracks[0], "%s is not on a track item; §20 asks for %s" % (name, key))

    def test_values_are_the_right_shape_not_just_present(self) -> None:
        # Presence alone has fooled this project before. The ones that carry geometry get checked.
        self.assertAlmostEqual(self.body["camera"]["effective_hfov_deg"], 69.3002, places=3)
        self.assertEqual(self.body["prediction"]["predicted_anchor_norm"], [0.531, 0.492])
        pts = self.body["field_of_regard"]["safe_envelope_points"]
        self.assertTrue(all(isinstance(p, list) and len(p) == 2 for p in pts),
                        "§20 wants points; a point is a [yaw, pitch] pair")
        self.assertIs(self.body["imu"]["present"], False, "a truthy-but-not-boolean present would read as installed")

    def test_names_the_page_is_claimed_to_read_are_really_in_the_page_script(self) -> None:
        for name, needle in PAGE_READ.items():
            self.assertIn(needle, HUD_JS,
                          "%s is claimed to be visible on the dashboard, but %s is not in its script"
                          % (name, needle))

    def test_the_page_never_turns_a_null_into_a_number(self) -> None:
        # The rule that makes the nulls above safe: an unknown must render as a dash. Were this to
        # return "0", "no IMU" would display as "level", and the ledger's careful nulls would buy
        # nothing at the last centimetre.
        self.assertIn('typeof v !== "number"', HUD_JS)
        self.assertIn('return "--"', HUD_JS)


if __name__ == "__main__":
    unittest.main()
