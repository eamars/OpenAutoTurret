"""The named video stream manifest: how visiond tells the rest of the station which streams exist.

Why a manifest and not a hard-coded filename. 架构师选的是 (b): **visiond 是两颗物理相机的唯一持有者**,
webd 只消费"有名字的视频流"(`wide` / `detail`). 那 webd 和 visiond 之间就需要一份小契约, 说明:
"这一路叫什么、由哪颗**身份**的相机产出、去哪里取帧、走的是什么 transport". 少了它, webd 只能猜文件名,
而"猜文件名"迟早退化成"猜相机"——那正是这套站一路在躲的 0/1 索引坑.

Transport 是清单里的一个**字段**, 不是契约本身 (`transport: "atomic-jpeg-file"`). 换成 unix socket 或
共享内存时, 改的是 producer/consumer 两端, **相机归属不用搬家**.

Identity 随流走: 每条流都带 `camera_id` + `identity_source` + `durable`, 由 `camera_id.py` 从
`/dev/v4l/by-path/...` 派生. 一条流如果只能报出 `/dev/videoN`, 它会带着 `identity_source="index"`
和 `durable=False` 出现在清单里 —— **未知就明说未知**, 不拿 0 冒充.

写盘方式沿用本仓库既有的规矩: 同目录临时文件 + `os.replace` 原子替换, 读者永远看不到半份 JSON.
"""
from __future__ import annotations

import json
import os
import tempfile
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable

MANIFEST_VERSION = 1
ROLES = ("wide", "detail")
JPEG_FILE_TRANSPORT = "atomic-jpeg-file"


class StreamManifestError(ValueError):
    """The manifest does not describe a stream this station can consume."""


@dataclass(frozen=True)
class StreamDescriptor:
    """One named stream. `role` is the name API callers use; it is not a device path."""

    role: str
    camera_id: str
    identity_source: str          # "by-path" | "by-id" | "fwnode" | "index"
    durable: bool
    path: str                     # where the pixels live, under `transport`
    transport: str = JPEG_FILE_TRANSPORT
    width: int = 0
    height: int = 0
    delivered_fps: float | None = None      # measured, never the requested rate
    dropped: int | None = None
    updated_ns: int = 0

    def __post_init__(self) -> None:
        if self.role not in ROLES:
            raise StreamManifestError(
                f"unknown stream role {self.role!r}; the API only speaks {', '.join(ROLES)}")
        if not self.camera_id:
            raise StreamManifestError(f"{self.role}: a stream with no camera_id cannot be "
                                      "attributed, and an empty id would look like one")
        if not self.identity_source:
            raise StreamManifestError(f"{self.role}: identity_source is required even when the "
                                      "answer is 'index' — unknown is a real answer")


@dataclass
class StreamManifest:
    """The whole published set, keyed by role, written as one small JSON file."""

    streams: dict[str, StreamDescriptor] = field(default_factory=dict)
    producer: str = "visiond"
    version: int = MANIFEST_VERSION

    def role_for(self, camera_id: str) -> str | None:
        for role, stream in self.streams.items():
            if stream.camera_id == camera_id:
                return role
        return None

    def to_dict(self) -> dict[str, Any]:
        return {"version": self.version, "producer": self.producer,
                "streams": {role: asdict(stream) for role, stream in sorted(self.streams.items())}}

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "StreamManifest":
        streams = {}
        fields = StreamDescriptor.__dataclass_fields__
        for role, raw in (payload.get("streams") or {}).items():
            # `asdict` writes `role` into the body too; if the body disagrees with the key, the
            # file describes two different things and a silent rename would hide that.
            if raw.get("role", role) != role:
                raise StreamManifestError(f"stream keyed {role!r} but its body says {raw.get('role')!r}")
            known = {k: v for k, v in raw.items() if k in fields and k != "role"}
            streams[role] = StreamDescriptor(role=role, **known)
        return cls(streams=streams, producer=payload.get("producer", "visiond"),
                   version=int(payload.get("version", MANIFEST_VERSION)))

    def write(self, path: str | Path) -> None:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            # 文本模式：NamedTemporaryFile 默认是二进制的，而 JSON 是要写 str 的——
            # 这条在 selftest 第一次跑的时候就把模块本身逮住了（写路径没跑过就等于没写）。
            with tempfile.NamedTemporaryFile(dir=target.parent, suffix=".json.part",
                                             delete=False, mode="w", encoding="utf-8") as handle:
                temporary = handle.name
                json.dump(self.to_dict(), handle, separators=(",", ":"), allow_nan=False)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, target)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)

    @classmethod
    def read(cls, path: str | Path) -> "StreamManifest | None":
        """``None`` when nothing is published yet — a webd that starts before visiond is normal."""
        try:
            payload = json.loads(Path(path).read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError):
            return None
        return cls.from_dict(payload)


def publish(*, path: str | Path, descriptors: Iterable[StreamDescriptor],
            producer: str = "visiond") -> StreamManifest:
    """Publish a set of streams and read it back, so a writer never claims what it did not write."""
    manifest = StreamManifest(streams={d.role: d for d in descriptors}, producer=producer)
    if not manifest.streams:
        raise StreamManifestError("refusing to publish an empty manifest: 'no streams' and "
                                  "'visiond is not up' must not be the same file")
    manifest.write(path)
    read_back = StreamManifest.read(path)
    if read_back is None or set(read_back.streams) != set(manifest.streams):
        raise StreamManifestError(f"manifest at {path} did not survive the round trip")
    return manifest


def publish_merged(path: str | Path, descriptor: StreamDescriptor, *,
                   producer: str = "visiond") -> StreamManifest:
    """Publish one stream's entry without touching the entries another owner published.

    Two capture paths share this one file. A writer that sends only its own descriptor would
    erase the other's every second, and the surviving stream would look like the only camera the
    station has. So: read what is published, replace exactly my role, write the set back.
    """
    existing = StreamManifest.read(path)
    merged = dict(existing.streams) if existing else {}
    merged[descriptor.role] = descriptor
    return publish(path=path, descriptors=merged.values(), producer=producer)


def _selftest() -> int:
    import tempfile as _tempfile

    checks = 0
    wide = StreamDescriptor(role="wide", camera_id="cam-baa28c2a", identity_source="by-path",
                            durable=True, path="/run/preview_wide.jpg", width=1920, height=1080,
                            delivered_fps=30.02, updated_ns=1234)
    detail = StreamDescriptor(role="detail", camera_id="cam-1f2e3d4c", identity_source="index",
                              durable=False, path="/run/preview_detail.jpg")
    with _tempfile.TemporaryDirectory() as box:
        target = os.path.join(box, "video_streams.json")
        published = publish(path=target, descriptors=[wide, detail])
        back = StreamManifest.read(target)
        assert back is not None and set(back.streams) == {"wide", "detail"}
        assert back.streams["wide"].delivered_fps == 30.02, "实测帧率必须原样回来"
        assert back.streams["detail"].durable is False, "index 来源不得伪装成 durable"
        assert back.streams["detail"].delivered_fps is None, "没测过是 None，不是 0"
        assert back.role_for("cam-1f2e3d4c") == "detail"
        checks += 1
        # A reader that starts before the writer must see "nothing yet", not a crash or a half file.
        assert StreamManifest.read(os.path.join(box, "nothing_here.json")) is None
        with open(os.path.join(box, "half.json"), "w", encoding="utf-8") as handle:
            handle.write('{"version": 1, "strea')
        assert StreamManifest.read(os.path.join(box, "half.json")) is None, "半份 JSON 不能当清单用"
        checks += 1
    try:
        StreamDescriptor(role="left", camera_id="cam-x", identity_source="by-path", durable=True,
                         path="/p")
    except StreamManifestError as exc:
        assert "left" in str(exc) and "wide" in str(exc), exc
    else:
        raise AssertionError("一个新角色名必须被点名拒绝，而不是静默出现在 API 里")
    checks += 1
    try:
        publish(path=os.path.join(_tempfile.gettempdir(), "ota_manifest_probe.json"), descriptors=[])
    except StreamManifestError as exc:
        assert "empty" in str(exc), exc
    else:
        raise AssertionError("空清单必须被拒：'没有流' 不等于 'visiond 没起'")
    checks += 1
    print(f"stream manifest selftest: {checks}/4 checks passed（不碰硬件）")
    return 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
