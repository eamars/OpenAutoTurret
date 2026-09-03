"""§80 — the artifact a fault has to leave behind."""
import json
import os

from ..blackbox import BlackBoxWriter, load
from ..protocol import Telemetry


def frame(capture_id, reason="brake in Ready", **over):
    body = {
        "id": capture_id,
        "reason": reason,
        "operating_mode": "AUTO_TRACK",
        "mode_phase": "TRACKING",
        "phase": "ready",
        "safety_action": "BRAKE",
        "selected_uuid": 11,
        "selected_label": "Person #1",
        "candidate_count": 2,
        "candidates": [
            {"uuid": 11, "label": "Person #1", "state": "CONFIRMED", "confidence": 0.9},
            {"uuid": 22, "label": "Person #2", "state": "CONFIRMED", "confidence": 0.86},
        ],
        "q_actual_rad": [-0.69, 2.59],
    }
    body.update(over)
    return Telemetry(blackbox_capture_id=capture_id, blackbox=body)


def test_a_scene_is_written_once_and_reads_back(tmp_path):
    w = BlackBoxWriter(str(tmp_path))
    path = w.observe(frame(1))
    assert path and os.path.exists(path)
    again = w.observe(frame(1))
    assert again is None, "the same scene was written twice; the capture id is a id"
    assert len(os.listdir(tmp_path)) == 1

    w.observe(frame(2, reason="fault in ready"))
    files = sorted(os.listdir(tmp_path))
    assert len(files) == 2, files
    doc = load(path)
    assert doc["schema"] == "ota-blackbox/1"
    assert doc["operating_mode"] == "AUTO_TRACK"
    # The candidates are the point of the record: a scene that shows only the chosen
    # track cannot be used to judge whether the choice was defensible (§80).
    assert [c["label"] for c in doc["candidates"]] == ["Person #1", "Person #2"]


def test_nothing_is_written_when_it_is_not_configured(tmp_path):
    w = BlackBoxWriter("")
    assert not w.enabled
    assert w.observe(frame(1)) is None
    assert os.listdir(tmp_path) == []


def test_an_id_without_a_payload_is_reported_not_swallowed(tmp_path, caplog):
    """The failure mode this project has already been bitten by: controld sends a field
    that webd never declared, so it vanishes without an error anywhere. An id with no
    body is that signature, and it has to be loud."""
    w = BlackBoxWriter(str(tmp_path))
    assert w.observe(Telemetry(blackbox_capture_id=5)) is None
    assert os.listdir(tmp_path) == []
    assert any("protocol.py" in r.getMessage() for r in caplog.records), caplog.text


def test_a_failing_disk_does_not_raise_into_the_telemetry_path(tmp_path):
    """The reader thread must not die over a log file, and the failure must not be
    silent either. A directory path that cannot hold a file stands in for a full disk."""
    blocker = tmp_path / "not-a-dir"
    blocker.write_text("x")
    w = BlackBoxWriter(str(blocker / "sub"))
    assert w.observe(frame(1)) is None  # no exception


def test_the_reason_is_not_allowed_to_become_a_path(tmp_path):
    """The reason string comes from controld and lands in a filename. It is trusted to
    describe a fault and not to be a relative path."""
    w = BlackBoxWriter(str(tmp_path))
    path = w.observe(frame(3, reason="../../etc/passwd"))
    assert path and os.path.dirname(path) == str(tmp_path)
    assert os.path.basename(path).startswith("blackbox_0003_")
    assert not (tmp_path.parent / "etc").exists()
