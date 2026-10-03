"""With no browser connected, a telemetry frame costs the web UI nothing (2026-10-03)."""
from web.webd import app as webd_app


def test_no_browser_means_no_serialisation(monkeypatch):
    hub = webd_app.TelemetryHub()
    calls = []
    monkeypatch.setattr(webd_app, "telemetry_to_json", lambda t: calls.append(t) or "{}")
    hub.decorate = lambda payload: calls.append("decorate") or payload
    hub.on_telemetry(object())
    assert calls == []


def test_a_status_file_is_parsed_again_only_when_it_changes(tmp_path):
    path = tmp_path / "inference_health.json"
    path.write_text('{"inference_fps": 30}')
    first = webd_app._read_json_cached(str(path))
    assert webd_app._read_json_cached(str(path)) is first
    path.write_text('{"inference_fps": 29.5, "x": 1}')
    assert webd_app._read_json_cached(str(path))["inference_fps"] == 29.5
