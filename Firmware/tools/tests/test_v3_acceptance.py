"""§110 acceptance tool: the rules that make it worth having.

The tool's entire value is that it cannot be talked into saying "accepted". So the tests are
not about formatting the report; they are attempts to get a false acceptance out of it.
"""
from __future__ import annotations

import json
import pathlib
import subprocess
import sys

TOOLS = pathlib.Path(__file__).resolve().parents[1]
ROOT = TOOLS.parents[1]
sys.path.insert(0, str(TOOLS))

import v3_acceptance as va  # noqa: E402

DOC = va.DEFAULT_DOC


def run(*argv):
    return subprocess.run([sys.executable, str(TOOLS / "v3_acceptance.py"), *argv],
                          capture_output=True, text=True)


def test_checklist_is_read_from_the_document_itself():
    """If §110's items were duplicated inside the tool, the report would certify the copy."""
    doc = DOC.read_text(encoding="utf-8")
    items = va.parse_checklist(doc)
    assert len(items) == 30, "expected the 30 items §110 contains, got %d" % len(items)
    groups = sorted({g for (g, _n, _t) in items})
    assert groups == ["AUTO_ROAM", "AUTO_TRACK", "COMMON", "MANUAL"], groups
    # Every item is unique by id, and no group renumbered itself out of existence.
    ids = [va.item_id(g, n) for (g, n, _t) in items]
    assert len(set(ids)) == len(ids)


def test_a_nonexistent_or_ambiguous_item_is_refused_not_guessed():
    """Picking the wrong checklist item and stamping it accepted is worse than refusing."""
    doc = DOC.read_text(encoding="utf-8")
    items = va.parse_checklist(doc)
    for bad in ("COMMON/99", "not a phrase in the doc anywhere"):
        try:
            va.resolve(items, bad)
        except SystemExit:
            continue
        raise AssertionError("%r should not resolve" % bad)
    # `target` appears in many items; the error must list them rather than choose one.
    try:
        va.resolve(items, "target")
        raise AssertionError("an ambiguous phrase resolved")
    except SystemExit as e:
        assert "matches" in str(e), str(e)


def test_simulation_cannot_be_recorded_as_acceptance(tmp_path):
    log = tmp_path / "log.json"
    r = run("--log", str(log), "record", "--item", "COMMON/2", "--method", "simulation",
            "--by", "ctest test_control_loop", "--evidence",
            "watched the transcript stop motion fall back to MANUAL/HOLD in the simulator")
    assert r.returncode == 0, r.stderr
    entry = json.loads(log.read_text())["entries"]["COMMON/2"]
    assert entry["status"] == "simulated", entry
    assert "not counted as accepted" in r.stdout + r.stderr


def test_acceptance_needs_a_person_a_station_and_a_sentence(tmp_path):
    log = tmp_path / "log.json"
    base = ["--log", str(log), "record", "--item", "COMMON/2", "--method", "hardware"]
    # No person.
    r = run(*(base + ["--by", "agent", "--station", "rpi-turret",
                      "--evidence", "watched it happen with my own eyes, twice, at 3 metres"]))
    assert r.returncode != 0 and "not a person" in r.stderr, r.stderr
    # No station.
    r = run(*(base + ["--by", "R. Ams", "--evidence",
                      "watched it happen with my own eyes, twice, at 3 metres"]))
    assert r.returncode != 0 and "station" in r.stderr, r.stderr
    # A person, a station, and a checkbox instead of evidence.
    r = run(*(base + ["--by", "R. Ams", "--station", "rpi-turret", "--evidence", "ok"]))
    assert r.returncode != 0 and "evidence" in r.stderr, r.stderr
    assert not log.exists(), "a refused record must write nothing at all"
    # And the same claim, properly evidenced, is accepted.
    r = run(*(base + ["--by", "R. Ams", "--station", "rpi-turret", "--evidence",
                      "pressed STOP MOTION mid-sweep; the axes stopped and the page read "
                      "MANUAL / HOLD within about a second"]))
    assert r.returncode == 0, r.stderr
    assert json.loads(log.read_text())["entries"]["COMMON/2"]["status"] == "passed"


def test_a_recorded_entry_cannot_be_reinterpreted_after_the_checklist_changes(tmp_path):
    """§110 might be edited after an item was accepted. The report has to say so rather than
    carry the old stamp forward against new wording."""
    log = tmp_path / "log.json"
    r = run("--log", str(log), "record", "--item", "COMMON/2", "--method", "hardware",
            "--by", "R. Ams", "--station", "rpi-turret", "--evidence",
            "watched the axes stop and the page read MANUAL / HOLD within about a second")
    assert r.returncode == 0, r.stderr
    data = json.loads(log.read_text())
    data["entries"]["COMMON/2"]["item_text"] = "STOP MOTION does something nice."
    log.write_text(json.dumps(data))
    out = run("--log", str(log), "report").stdout
    assert "different wording" in out


def test_the_tool_never_provides_for_moving_the_turret():
    """Read-only is a claim about the code, so it is tested as one. The web API's command
    endpoint is a POST to /api/command on the very base URL this tool is handed for
    telemetry, so 'it only reads' has to be true of the source and not of the intent."""
    src = (TOOLS / "v3_acceptance.py").read_text(encoding="utf-8")
    # The command endpoint may be *mentioned* — it is worth saying why it is off limits —
    # but never addressed outside a comment.
    for line in src.splitlines():
        if "/api/command" in line:
            assert line.lstrip().startswith("#"), "the command endpoint is addressed: %s" % line
    assert 'TELEMETRY_PATHS = ("/api/state", "/api/health")' in src
    assert src.count("urlopen") == 1, "a second fetch path would slip past the URL guard"
    # Behaviour too: the guard is on the parsed path, so a command URL is refused outright.
    r = run("attach", "--item", "COMMON/2", "--by", "R. Ams",
            "--url", "http://127.0.0.1:9/api/command")
    assert r.returncode != 0 and "refusing" in r.stderr, r.stderr


def test_a_corrupt_log_is_not_overwritten(tmp_path):
    log = tmp_path / "log.json"
    log.write_text("{ not json")
    r = run("--log", str(log), "report")
    assert r.returncode != 0 and "refusing to overwrite" in r.stderr, r.stderr
    assert log.read_text() == "{ not json"
