"""The IMU block webd publishes: read straight from the acquisition process's NDJSON trace.

Why webd reads the trace instead of asking controld. Controld already ingests the same file
(``control/src/control/imu_trace_ingest.cpp``) and its telemetry §20 block declares
``imu_present`` — but that field was written when the station genuinely had no inertial sensor,
and nothing assigns it today. Wiring the C++ side means a wire-schema bump and a rebuild on the
station; the operator's question ("is there an IMU, is it alive, what is it saying") does not wait
for that. So this reads the same file the control side reads, publishes what it can *see*, and
leaves a note in §20's owner's hands rather than pretending the number came from controld.

What "unknown" looks like here: ``None``, and a chip that says NOT CONFIGURED / NO SAMPLES.
An age of 0 ms would claim the sample arrived at the instant we looked, and a rate of 0 fps would
claim a dead sensor when the truth is "we have not had a full window yet" — those are different
statements and the UI must be able to tell them apart.
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional

#: Kinds the driver emits. Anything else is counted and ignored: a new line kind must not be
#: mistaken for a healthy one just because it parsed.
SAMPLE = "sample"
TARE = "tare"
GAP = "gap"
TRACE_RESET = "trace_reset"
SUMMARY = "summary"


def _blank_trace_stats() -> Dict[str, Any]:
    return {"lines": 0, "samples": 0, "parse_errors": 0, "gaps": 0, "resets": 0,
            "truncations": 0}


@dataclass
class ImuTraceReader:
    """Tails one IMU trace file and answers the UI's four questions about it.

    ``present`` — did this file ever contain a sample.
    ``fresh``   — is the newest sample inside ``fresh_ms``.
    ``age_ms``  — how stale the newest sample is (``None`` when there is no sample at all).
    ``rate_hz`` — samples per second **measured** across reads, ``None`` until a full window
                 of at least one second has two counts to compare.
    """

    path: str
    fresh_ms: int = 100
    clock: Callable[[], float] = time.monotonic
    max_bytes_per_read: int = 1 << 20
    _offset: int = 0
    _inode: Optional[int] = None
    stats: Dict[str, Any] = field(default_factory=_blank_trace_stats)
    _sensors: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    _tare: Dict[str, Any] = field(default_factory=dict)
    _summary: Dict[str, Any] = field(default_factory=dict)
    _last_rate_count: int = 0
    _last_rate_ns: int = 0
    _rate: Optional[float] = None

    def read_once(self) -> Dict[str, Any]:
        """Consume whatever was appended since last time and return the publishable block."""
        now_ns = int(self.clock() * 1_000_000_000)
        try:
            size = os.path.getsize(self.path)
            inode = os.stat(self.path).st_ino
        except OSError:
            # No file yet is a normal state at boot, not an error: the acquisition process may not
            # have started, and saying "NOT CONFIGURED" is a different claim from "the IMU is dead".
            return self.block(now_ns, present=self.stats["samples"] > 0, configured=False)
        if self._inode is not None and inode != self._inode:
            # Same path, different file: the writer restarted. Old ages must not be reported
            # against a file that was replaced underneath us.
            self.stats["resets"] += 1
            self._offset = 0
            self._sensors = {}
        self._inode = inode
        if size < self._offset:
            self.stats["truncations"] += 1
            self._offset = 0
        if size > self._offset:
            with open(self.path, "r", encoding="utf-8", errors="replace") as handle:
                handle.seek(self._offset)
                chunk = handle.read(self.max_bytes_per_read // 2)
                self._offset = handle.tell()
            for line in chunk.splitlines():
                line = line.strip()
                if line:
                    self._consume(line)
        return self.block(now_ns, present=self.stats["samples"] > 0, configured=True)

    def _consume(self, line: str) -> None:
        self.stats["lines"] += 1
        try:
            payload = json.loads(line)
        except ValueError:
            # A half line at the tail is normal (the writer is mid-write); a *pattern* of them is
            # not, so both are counted and the count is published instead of swallowed.
            self.stats["parse_errors"] += 1
            return
        kind = payload.get("kind")
        if kind == SAMPLE:
            self.stats["samples"] += 1
            name = str(payload.get("sensor") or "unknown")
            values = payload.get("values") or []
            entry = self._sensors.setdefault(name, {"samples": 0, "last_rx_ns": 0,
                                                    "status": None, "values": None})
            entry["samples"] += 1
            entry["last_rx_ns"] = int(payload.get("rx_ns") or 0)
            entry["status"] = payload.get("status")
            entry["values"] = values
            entry["sequence"] = payload.get("sequence")
            if payload.get("relative") is not None:
                entry["relative"] = payload["relative"]
        elif kind == TARE:
            self._tare = {"valid": bool(payload.get("mount_alignment_valid")),
                          "generation": payload.get("generation"),
                          "method": payload.get("method"),
                          "q_ref_xyzw": payload.get("q_ref_xyzw")}
        elif kind == GAP:
            self.stats["gaps"] += 1
        elif kind == TRACE_RESET:
            self.stats["resets"] += 1
        elif kind == SUMMARY:
            self._summary = {k: payload.get(k) for k in ("counts", "read_errors", "recoveries",
                                                        "failed", "tared")}

    def block(self, now_ns: int, *, present: bool, configured: bool) -> Dict[str, Any]:
        newest = max((entry["last_rx_ns"] for entry in self._sensors.values()), default=0)
        age_ms = None if not newest else max(0.0, round((now_ns - newest) / 1e6, 1))
        if self._last_rate_ns == 0:
            self._last_rate_ns = now_ns
            self._last_rate_count = self.stats["samples"]
        elif now_ns - self._last_rate_ns >= 1_000_000_000:
            delta = self.stats["samples"] - self._last_rate_count
            if delta > 0:
                self._rate = round(delta * 1e9 / (now_ns - self._last_rate_ns), 1)
            self._last_rate_count = self.stats["samples"]
            self._last_rate_ns = now_ns
        gyro = self._sensors.get("gyroscope") or self._sensors.get("gyro")
        rv = (self._sensors.get("game_rotation_vector") or self._sensors.get("game_rv")
              or self._sensors.get("rotation_vector"))
        return {
            "configured": configured,
            "present": present,
            "fresh": bool(age_ms is not None and age_ms <= float(self.fresh_ms)),
            "age_ms": age_ms,
            "rate_hz": self._rate,
            "trace": self.path,
            "generation": max([int(e.get("generation") or 0) for e in self._sensors.values()]
                              or [self._tare.get("generation") or 0]),
            "sensors": sorted(self._sensors),
            "gyro_rad_s": [float(v) for v in gyro["values"][:3]] if gyro and
                          isinstance(gyro.get("values"), list) and len(gyro["values"]) >= 3
                          else None,
            "gyro_status": gyro.get("status") if gyro else None,
            "game_rv_quat": [float(v) for v in rv["values"][:4]] if rv and
                            isinstance(rv.get("values"), list) and len(rv["values"]) >= 4
                            else None,
            "game_rv_accuracy": rv.get("status") if rv else None,
            "relative_quat": rv.get("relative") if rv else None,
            "tare": dict(self._tare),
            "acquisition": dict(self._summary),
            "stats": dict(self.stats),
        }


def _selftest() -> int:
    """A fake trace on disk, so the parsing and the lies-it-must-not-tell rules are testable."""
    import tempfile

    checks = 0
    with tempfile.TemporaryDirectory() as box:
        path = os.path.join(box, "imu.ndjson")
        reader = ImuTraceReader(path=path, fresh_ms=100)
        empty = reader.read_once()
        assert empty["configured"] is False and empty["present"] is False
        assert empty["age_ms"] is None, "没样本时年龄必须是未知，不是 0"
        checks += 1

        with open(path, "w", encoding="utf-8") as handle:
            handle.write('{"kind":"product","part":55,"version":"1.2.3","build":1}\n')
            handle.write('{"kind":"sample","sensor":"gyroscope","rx_ns":1000,"sample_ns":990,'
                         '"generation":1,"sequence":7,"status":3,"values":[0.01,0.02,-0.03]}\n')
            handle.write('{"kind":"sample","sensor":"game_rotation_vector","rx_ns":1500,'
                         '"sample_ns":1490,"generation":1,"sequence":8,"status":3,'
                         '"values":[0,0,0,1],"relative":[0,0,0,1]}\n')
            handle.write('{"kind":"tare","rx_ns":1600,"generation":1,'
                         '"method":"host_stationary_game_rv","q_ref_xyzw":[0,0,0,1],'
                         '"mount_alignment_valid":false}\n')
            handle.write("a half line the writer has not finished")
        # 最新样本 rx_ns=1500 ns；把时钟摆到 1.5 µs + 1 ms 处，年龄就该是 1.0 ms。
        now = (1_500 + 1_000_000) / 1e9
        block = ImuTraceReader(path=path, fresh_ms=100, clock=lambda: now).read_once()
        assert block["present"] is True and block["fresh"] is True
        assert block["age_ms"] == 1.0, f" newest rx_ns=1500 → 年龄应 1.0 ms，实得 {block['age_ms']}"
        assert block["gyro_rad_s"] == [0.01, 0.02, -0.03]
        assert block["game_rv_quat"] == [0.0, 0.0, 0.0, 1.0]
        assert block["game_rv_accuracy"] == 3 and block["tare"]["valid"] is False
        assert block["stats"]["parse_errors"] == 1, "半行要计下来，不能吞掉"
        assert block["rate_hz"] is None, "只有一个窗口时不能凭空报速率"
        checks += 1

        # Appending must be incremental: an old sample ages out rather than looking fresh forever.
        appended = ImuTraceReader(path=path, fresh_ms=100, clock=lambda: 200.0)
        aged = appended.read_once()
        assert aged["present"] is True and aged["fresh"] is False, "两百年前的样本不能算新鲜"
        assert aged["age_ms"] > 1000
        checks += 1

        # The writer restarting (same path, truncated) must not report the old sample as current.
        with open(path, "w", encoding="utf-8") as handle:
            handle.write('{"kind":"gap","rx_ns":9000000,"reason":"reset_recovery",'
                         '"tare_invalidated":true}\n')
        restarted = ImuTraceReader(path=path, fresh_ms=100, clock=lambda: 200.0)
        restarted._offset = 400                      # pretend we had read past the new end
        restarted._inode = os.stat(path).st_ino
        after = restarted.read_once()
        assert after["stats"]["truncations"] == 1, "被截断的 trace 要说出来，不能当没发生"
        checks += 1

    try:
        ImuTraceReader(path="/nonexistent/imu.ndjson").block(0, present=False, configured=False)
    except Exception as exc:                                            # noqa: BLE001
        raise AssertionError(f"空状态不该抛异常：{exc}")
    checks += 1
    print(f"imu trace reader selftest: {checks} checks passed（不碰硬件）")
    return 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
