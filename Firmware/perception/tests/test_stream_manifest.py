"""Contract tests for the named-stream manifest — the visiond↔webd boundary.

These exist because the (b) decision moves camera ownership into visiond. The moment webd stops
opening a physical camera, it needs something to read instead of a filename it invented, and that
something has to fail loudly when it is missing rather than quietly serving the wide stream under
the name `detail`.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from perception.stream_manifest import (  # noqa: E402
    ROLES,
    StreamDescriptor,
    StreamManifest,
    StreamManifestError,
    publish,
)

WIDE = dict(role="wide", camera_id="cam-baa28c2a", identity_source="by-path", durable=True,
            path="/run/ota/preview.jpg", width=1920, height=1080, delivered_fps=30.02)


def _descriptor(**overrides) -> StreamDescriptor:
    payload = dict(WIDE)
    payload.update(overrides)
    return StreamDescriptor(**payload)


def test_only_the_named_roles_are_public():
    assert ROLES == ("wide", "detail")
    with pytest.raises(StreamManifestError, match="left"):
        _descriptor(role="left")


def test_a_stream_that_only_knows_its_index_says_so_instead_of_looking_durable(tmp_path):
    target = tmp_path / "video_streams.json"
    publish(path=str(target), descriptors=[_descriptor(identity_source="index", durable=False)])
    back = StreamManifest.read(target)
    assert back is not None
    assert back.streams["wide"].durable is False
    assert back.streams["wide"].identity_source == "index"


def test_unmeasured_rate_stays_unknown_rather_than_becoming_zero_fps(tmp_path):
    target = tmp_path / "video_streams.json"
    fresh = _descriptor(delivered_fps=None, dropped=None)
    assert fresh.delivered_fps is None
    published = publish(path=str(target), descriptors=[fresh])
    payload = json.loads(target.read_text(encoding="utf-8"))
    assert payload["streams"]["wide"]["delivered_fps"] is None
    assert published.streams["wide"].camera_id == WIDE["camera_id"], "身份必须原样往返"


def test_a_missing_or_half_written_manifest_reads_as_nothing_published(tmp_path):
    assert StreamManifest.read(tmp_path / "absent.json") is None, "webd 先起来是正常场景"
    half = tmp_path / "half.json"
    half.write_text('{"version": 1, "strea', encoding="utf-8")
    assert StreamManifest.read(half) is None


def test_an_empty_manifest_is_refused_because_it_would_lie_about_the_station(tmp_path):
    with pytest.raises(StreamManifestError, match="empty"):
        publish(path=str(tmp_path / "video_streams.json"), descriptors=[])
    assert not os.path.exists(tmp_path / "video_streams.json")


def test_roles_are_lookupable_by_identity_because_the_api_answers_by_name():
    pair = [_descriptor(),
            _descriptor(role="detail", camera_id="cam-1f2e3d4c",
                        path="/run/ota/preview_detail.jpg", delivered_fps=29.98)]
    manifest = StreamManifest(streams={d.role: d for d in pair})
    assert manifest.role_for("cam-1f2e3d4c") == "detail"
    assert manifest.role_for("cam-nope") is None
    assert set(manifest.to_dict()["streams"]) == {"wide", "detail"}
