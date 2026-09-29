"""Slice 2: visiond is the only process allowed to name a stream, so test how it names it.

The two things worth guarding here are both about lies a manifest can tell:
a role borrowed from configuration instead of from the sensor, and a rate nobody measured yet
written down as 0 (which a UI renders as "dead stream" during the first second of every boot).
"""
from __future__ import annotations

import os
import stat
import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from perception.stream_manifest import StreamManifest  # noqa: E402
from perception.visiond import _StreamAnnouncer, _role_for_stream  # noqa: E402


def test_the_role_comes_from_what_the_sensor_reported_not_from_a_hope():
    assert _role_for_stream({"sensor_model": "IMX500"}) == "wide"
    assert _role_for_stream({"sensor_model": "imx477"}) == "detail"
    assert _role_for_stream({"sensor_model": "ov5647"}) is None, "没映射的传感器不借名字"
    assert _role_for_stream({}) is None


def test_an_unmapped_sensor_publishes_no_manifest_rather_than_a_wrong_one(tmp_path, monkeypatch):
    monkeypatch.setenv("OTA_VISION_STREAM_MANIFEST", str(tmp_path / "video_streams.json"))
    ident = types.SimpleNamespace(id="cam-aaaa1111", source="by-path", durable=True)
    announcer = _StreamAnnouncer.from_environment(role=_role_for_stream({"sensor_model": ""}),
                                                 ident=ident, size=(1920, 1080), preview=None)
    assert announcer is None
    assert not (tmp_path / "video_streams.json").exists()


def _fake_preview(**overrides) -> types.SimpleNamespace:
    payload = dict(published=0, failures=0, path="/run/ota/preview.jpg")
    payload.update(overrides)
    return types.SimpleNamespace(**payload)


def test_the_first_publish_carries_identity_and_an_unknown_rate(tmp_path):
    target = tmp_path / "video_streams.json"
    ident = types.SimpleNamespace(id="cam-baa28c2a", source="by-path", durable=True)
    announcer = _StreamAnnouncer(path=str(target), role="wide", ident=ident,
                                 size=(1920, 1080), preview=_fake_preview(path=str(tmp_path / "p.jpg")))
    announcer.publish_once()
    manifest = StreamManifest.read(target)
    assert manifest is not None
    stream = manifest.streams["wide"]
    assert (stream.camera_id, stream.identity_source, stream.durable) == \
        ("cam-baa28c2a", "by-path", True)
    assert stream.path == str(tmp_path / "p.jpg"), "消费者按清单取帧，路径必须是真的那个"
    assert stream.delivered_fps is None, "没测过的一律 None，不是 0"
    assert (stream.width, stream.height) == (1920, 1080)


def test_the_rate_is_measured_over_a_window_and_then_appears(tmp_path):
    target = tmp_path / "video_streams.json"
    preview = _fake_preview()
    ident = types.SimpleNamespace(id="cam-baa28c2a", source="by-path", durable=True)
    announcer = _StreamAnnouncer(path=str(target), role="wide", ident=ident,
                                 size=(1920, 1080), preview=preview, interval_s=1.0)
    announcer.publish_once()
    preview.published = 12
    announcer._last_ns -= 2_000_000_000            # 假装两个窗口过去了
    announcer.publish_once()
    measured = StreamManifest.read(target).streams["wide"].delivered_fps
    assert measured == pytest.approx(6.0, abs=0.5), f"12 帧 / 2 s 应是 6 fps，实得 {measured}"


def test_a_manifest_that_cannot_be_written_degrades_loudly_instead_of_killing_capture(tmp_path):
    read_only = tmp_path / "ro"
    read_only.mkdir()
    os.chmod(read_only, stat.S_IRUSR | stat.S_IXUSR)
    ident = types.SimpleNamespace(id="cam-baa28c2a", source="index", durable=False)
    announcer = _StreamAnnouncer(path=str(read_only / "video_streams.json"), role="detail",
                                 ident=ident, size=(1920, 1080), preview=_fake_preview())
    try:
        announcer.publish_once()                    # must not raise: a bad file is not a lost camera
    finally:
        os.chmod(read_only, stat.S_IRWXU)
    assert announcer.failures == 1, announcer.last_error
    # 计数与错误名都要留下，但错误名记的是**真实那个类**（PermissionError 是 OSError 的子类），
    # 测试只要求它"是一个 I/O 失败的名字"，不逼实现去抹平子类。
    import builtins
    name = announcer.last_error.split(":", 1)[0]
    assert issubclass(getattr(builtins, name, OSError), OSError), announcer.last_error


def test_a_borrowed_index_identity_is_published_as_not_durable(tmp_path):
    target = tmp_path / "video_streams.json"
    ident = types.SimpleNamespace(id="cam-cccc3333", source="index", durable=False)
    _StreamAnnouncer(path=str(target), role="wide", ident=ident, size=(1280, 720),
                     preview=_fake_preview()).publish_once()
    stream = StreamManifest.read(target).streams["wide"]
    assert stream.durable is False and stream.identity_source == "index"
