"""§42's event log: named lifecycle facts, bounded in memory, persisted on loss.

The handover's central complaint is not that identities die but that nobody can say why
(§41: "every identity loss has an observable cause"). A per-frame trace is too much to
keep and too little to read; an event per state transition is the right resolution. The
names below are §42's names, verbatim, so a log line from the station can be quoted back
into this document.

Two properties are non-negotiable and both come from §41:

**Bounded memory.** A ring buffer, not an ever-growing list. A daemon that leaks a
kilobyte of history per identity transition is a daemon that dies during a long run, and
it dies in the one situation — many identities, much churn — where the history matters.

**Persistence on the events that matter.** Selection loss and merge are the moments an
operator will ask about, hours later. Emitting them into memory only means the answer is
gone as soon as the process is. ``persist_critical`` therefore appends the ring to a
JSONL file, rate-limited — a station that flaps selection at 20 Hz must not turn the
frame path into a disk flush.
"""
from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

from .protocol.jsonio import JsonlWriter, iter_jsonl


class EventType(str, Enum):
    """§42's event vocabulary. ``str`` valued so JSON carries the name, not an index."""

    MODEL_LOADED = "MODEL_LOADED"
    MODEL_REJECTED_INCOMPATIBLE = "MODEL_REJECTED_INCOMPATIBLE"

    DETECTION_DUPLICATE_SUPPRESSED = "DETECTION_DUPLICATE_SUPPRESSED"
    TRACK_CREATED = "TRACK_CREATED"
    TRACK_CONFIRMED = "TRACK_CONFIRMED"
    TRACK_OCCLUDED = "TRACK_OCCLUDED"
    TRACK_LOST = "TRACK_LOST"
    TRACK_REACQUIRED = "TRACK_REACQUIRED"
    TRACK_RETIRED = "TRACK_RETIRED"
    TRACK_MERGED = "TRACK_MERGED"
    TRACK_CAPACITY_DROP = "TRACK_CAPACITY_DROP"

    TARGET_SELECTED = "TARGET_SELECTED"
    TARGET_SELECTION_REJECTED = "TARGET_SELECTION_REJECTED"
    TARGET_SELECTION_IDEMPOTENT = "TARGET_SELECTION_IDEMPOTENT"
    TARGET_CLEARED = "TARGET_CLEARED"
    TARGET_AMBIGUOUS = "TARGET_AMBIGUOUS"
    TARGET_STALE = "TARGET_STALE"

    # Beyond §42's enumeration, and named deliberately: §26's "report, don't absorb" is
    # unenforceable if a frame that failed has no event to fail into. A pipeline that only
    # logs §42's happy-path types cannot distinguish "the scene was empty" from "inference
    # has not answered for forty seconds", which is the distinction that matters at 3 a.m.
    PIPELINE_FRAME_FAULT = "PIPELINE_FRAME_FAULT"
    PUBLISH_FAILED = "PUBLISH_FAILED"
    CAMERA_FRAME_STALLED = "CAMERA_FRAME_STALLED"


#: The events an operator will ask about after the fact (§41's "persist on
#: fault/selection-loss event"). A merge is here because it silently moves a selection
#: from one UUID to another; TARGET_STALE is here because it is the moment the turret
#: stops having a subject at all.
CRITICAL_EVENTS: Tuple[EventType, ...] = (
    EventType.MODEL_REJECTED_INCOMPATIBLE,
    EventType.TRACK_MERGED,
    EventType.TRACK_LOST,
    EventType.TRACK_RETIRED,
    EventType.TRACK_CAPACITY_DROP,
    EventType.TARGET_SELECTION_REJECTED,
    EventType.TARGET_AMBIGUOUS,
    EventType.TARGET_STALE,
)


@dataclass
class Event:
    """One lifecycle fact.

    Carries BOTH clocks: ``sensor_timestamp_ns`` says when the scene it describes was
    captured, ``receive_timestamp_ns`` when the daemon learned about it. Reconstructing
    "why did the turret swing at the empty doorway" needs the scene time; diagnosing a
    two-second processing stall needs the other one. Recording only one of them guarantees
    an argument about which the timestamps meant.
    """

    event_type: EventType
    receive_timestamp_ns: int = 0
    sensor_timestamp_ns: int = 0
    track_uuid: str = ""
    frame_sequence: int = 0
    fields: Dict[str, Any] = field(default_factory=dict)

    @property
    def name(self) -> str:
        return self.event_type.value

    def to_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "event": self.event_type.value,
            "receive_timestamp_ns": int(self.receive_timestamp_ns),
            "sensor_timestamp_ns": int(self.sensor_timestamp_ns),
            "frame_sequence": int(self.frame_sequence),
        }
        if self.track_uuid:
            out["track_uuid"] = self.track_uuid
        if self.fields:
            out["fields"] = dict(self.fields)
        return out

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Event":
        raw = data.get("event", "")
        try:
            event_type = EventType(raw)
        except ValueError:
            # An event name from a newer build must not abort a replay. Unknown events
            # stay in the trace as their own string rather than being dropped: the point
            # of §41 is that nothing about an identity loss goes unrecorded.
            event_type = raw  # type: ignore[assignment]
        return cls(
            event_type=event_type,  # type: ignore[arg-type]
            receive_timestamp_ns=int(data.get("receive_timestamp_ns", 0)),
            sensor_timestamp_ns=int(data.get("sensor_timestamp_ns", 0)),
            track_uuid=str(data.get("track_uuid", "") or ""),
            frame_sequence=int(data.get("frame_sequence", 0)),
            fields=dict(data.get("fields") or {}),
        )

    def describe(self) -> str:
        """One human-readable line, for the journal and for a test failure message."""
        parts = [self.event_type.value]
        if self.track_uuid:
            parts.append(self.track_uuid[:8])
        for key, value in sorted(self.fields.items()):
            parts.append(f"{key}={value}")
        return " ".join(str(p) for p in parts)


class EventLog:
    """Ring buffer of §42 events, with optional append-on-critical persistence."""

    def __init__(self, capacity: int = 4096, persist_path: str = "",
                 persist_critical: bool = True,
                 persist_min_interval_s: float = 2.0) -> None:
        if capacity < 16:
            raise ValueError("EventLog capacity must be at least 16 events")
        self.capacity = int(capacity)
        self._events: Deque[Event] = deque(maxlen=self.capacity)
        self._counts: Dict[str, int] = {}
        self._dropped = 0
        self._persist_path = persist_path or ""
        self._persist_critical = bool(persist_critical)
        self._persist_min_interval_s = float(persist_min_interval_s)
        self._persist_writer: Optional[JsonlWriter] = None
        self._last_persist_s = 0.0
        self._persisted_count = 0
        self.total_emitted = 0

    # -- emit ---------------------------------------------------------------
    def emit(self, event_type: EventType, *, track_uuid: str = "",
             sensor_timestamp_ns: int = 0, receive_timestamp_ns: int = 0,
             frame_sequence: int = 0, **fields: Any) -> Event:
        """Record one event. Never raises: a logger that can fail belongs on the frame
        path of a safety-relevant pipeline, which is exactly where it must not be."""
        if not isinstance(event_type, EventType):
            raise TypeError(f"not an EventType: {event_type!r}")
        event = Event(
            event_type=event_type,
            receive_timestamp_ns=(int(receive_timestamp_ns) if receive_timestamp_ns
                                  else int(time.monotonic_ns())),
            sensor_timestamp_ns=int(sensor_timestamp_ns),
            track_uuid=track_uuid or str(fields.pop("uuid", "") or ""),
            frame_sequence=int(frame_sequence),
            fields={k: v for k, v in fields.items() if v is not None},
        )
        if len(self._events) == self.capacity:
            self._dropped += 1
        self._events.append(event)
        self._counts[event.name] = self._counts.get(event.name, 0) + 1
        self.total_emitted += 1
        if (self._persist_critical and self._persist_path
                and event.event_type in CRITICAL_EVENTS):
            self._persist_now(event)
        return event

    # -- queries ------------------------------------------------------------
    def recent(self, limit: int = 50,
               types: Optional[Sequence[EventType]] = None) -> List[Event]:
        """Most recent first, optionally filtered by type. Bounded by ``limit``."""
        events = list(self._events)
        if types:
            wanted = set(types)
            events = [e for e in events if e.event_type in wanted]
        return list(reversed(events[-int(limit):])) if events else []

    def for_track(self, track_uuid: str, limit: int = 100) -> List[Event]:
        """Everything this subsystem has said about one identity, newest first.

        The question this exists to answer: "why did the selected identity stop receiving
        measurements?" — the answer is the ordered transitions of that UUID, and it is
        already in this ring.
        """
        if not track_uuid:
            return []
        hits = [e for e in self._events if e.track_uuid == track_uuid]
        return list(reversed(hits[-int(limit):]))

    def tail(self, limit: int = 0) -> List[Event]:
        """Whole ring in chronological order (oldest first)."""
        events = list(self._events)
        return events[-int(limit):] if limit else events

    def count(self, event_type: EventType) -> int:
        return self._counts.get(
            event_type.value if isinstance(event_type, EventType) else str(event_type), 0)

    def counts(self) -> Dict[str, int]:
        return dict(self._counts)

    @property
    def dropped(self) -> int:
        """Events overwritten by the ring. §16.4's principle applies to logs too."""
        return self._dropped

    def to_dicts(self, limit: int = 0) -> List[Dict[str, Any]]:
        return [e.to_dict() for e in self.tail(limit)]

    # -- persistence --------------------------------------------------------
    def _persist_now(self, trigger: Event) -> None:
        now_s = time.monotonic()
        if (self._last_persist_s
                and now_s - self._last_persist_s < self._persist_min_interval_s):
            return
        try:
            if self._persist_writer is None:
                self._persist_writer = JsonlWriter(self._persist_path, flush_every=1)
            self._persist_writer.write({
                "persisted_reason": trigger.event_type.value,
                "persisted_at_ns": int(time.monotonic_ns()),
                "ring": self.to_dicts(),
            })
            self._persisted_count = self.total_emitted
            self._last_persist_s = now_s
        except OSError as exc:  # a full SD card must not stop perception
            self._persist_critical = False
            self._persist_writer = None
            print(f"perception: event log persistence disabled: {exc}", flush=True)

    def close(self) -> None:
        if self._persist_writer is not None:
            self._persist_writer.close()
            self._persist_writer = None

    def __len__(self) -> int:
        return len(self._events)

    def __bool__(self) -> bool:
        """Always true. An event log with nothing in it yet is still the log to write to.

        This exists because ``log or EventLog()`` is the natural way to default a collaborator,
        and a freshly built log has ``len`` zero — so the falsy-empty version of this class
        throws away the caller's log (and its persistence path) at the start of every run,
        which is the moment §41's guarantees matter most.
        """
        return True

    def __iter__(self):
        return iter(list(self._events))


def load_event_log(path: str) -> List[Event]:
    """Read back what ``persist_critical`` wrote (§52's replay-of-diagnostics path)."""
    events: List[Event] = []
    for record in iter_jsonl(path):
        ring = record.get("ring")
        if isinstance(ring, list):
            events.extend(Event.from_dict(item) for item in ring)
        elif record.get("event"):
            events.append(Event.from_dict(record))
    return events
