"""§25.1's alias map: ``retired_uuid -> surviving_uuid``, for the rest of the session.

When the DuplicateTrackResolver decides that two identities are one physical person, the
survivor keeps its UUID and the other is retired. The alias exists because three parties
keep referring to the retired identity after it stops existing:

* the **selection**, which must follow it atomically (§25.1) — otherwise a merge silently
  un-selects the person the turret is pointing at;
* the **browser**, whose click may already be in flight when the merge happens;
* the **diagnostics**, whose whole purpose is explaining an identity's history (§41),
  which is broken if the history stops at a UUID that no longer means anything.

Resolution is followed to the end of the chain and cycle-guarded. The cycle guard is not
theoretical: two merges in opposite directions inside one dwell period (``a->b`` then,
after a resolver disagreement, ``b->a``) produce a two-element cycle, and a resolver that
follows aliases recursively without it hangs the selection thread. The map does not try to
pretend the disagreement away — it resolves to the newest hop and reports the chain, so the
disagreement is visible in the trace.

The map is session-scoped and memory-only (§24's persistence policy applies to identity
routing as much as to appearance data): a restart mints a new ``session_uuid``, and a UUID
from the previous session must not be honoured as if it still named a person.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterator, List, Mapping, Optional, Tuple


@dataclass
class AliasRecord:
    """One hop, with the evidence that produced it (§41)."""

    retired_uuid: str
    survivor_uuid: str
    reason: str = ""
    sensor_timestamp_ns: int = 0
    frame_sequence: int = 0

    def to_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"retired_uuid": self.retired_uuid,
                               "survivor_uuid": self.survivor_uuid}
        if self.reason:
            out["reason"] = self.reason
        if self.sensor_timestamp_ns:
            out["sensor_timestamp_ns"] = int(self.sensor_timestamp_ns)
        if self.frame_sequence:
            out["frame_sequence"] = int(self.frame_sequence)
        return out


class AliasMap:
    """Session-long ``retired -> surviving`` identity routing."""

    #: Chain-walk bound. A chain longer than this is not a plausible merge history, it is
    #: a resolver loop, and the resolver is the thing under test.
    MAX_CHAIN = 16

    def __init__(self) -> None:
        self._map: Dict[str, AliasRecord] = {}

    # -- mutation -----------------------------------------------------------
    def add(self, retired_uuid: str, survivor_uuid: str, *, reason: str = "",
            sensor_timestamp_ns: int = 0, frame_sequence: int = 0) -> AliasRecord:
        """Record a merge. Refuses the two degenerate aliases rather than storing them:
        ``a -> a`` says a track merged into itself, and ``a -> b -> a`` is §25's resolver
        contradicting itself. Both must surface as bugs, not as silently working maps."""
        if not retired_uuid or not survivor_uuid:
            raise ValueError("alias endpoints must be non-empty UUIDs")
        if retired_uuid == survivor_uuid:
            raise ValueError("a track may not merge into itself")
        existing = self.resolve(survivor_uuid)
        if existing == retired_uuid:
            raise ValueError(
                f"alias {retired_uuid[:8]}->{survivor_uuid[:8]} would create a cycle "
                f"(survivor already resolves to the retired identity)")
        record = AliasRecord(retired_uuid=retired_uuid, survivor_uuid=survivor_uuid,
                             reason=reason, sensor_timestamp_ns=int(sensor_timestamp_ns),
                             frame_sequence=int(frame_sequence))
        self._map[retired_uuid] = record
        return record

    # -- queries ------------------------------------------------------------
    def resolve(self, track_uuid: str) -> Optional[str]:
        """The identity ``track_uuid`` now means, or ``None`` if it never aliased."""
        if not track_uuid:
            return None
        chain = self.resolve_chain(track_uuid)
        return chain[-1] if len(chain) > 1 else None

    def resolve_or_self(self, track_uuid: str) -> str:
        return self.resolve(track_uuid) or track_uuid

    def resolve_chain(self, track_uuid: str) -> List[str]:
        """The full hop list, oldest first. ``[a]`` when ``a`` never merged."""
        chain = [track_uuid]
        if not track_uuid:
            return chain
        seen = {track_uuid}
        current = track_uuid
        for _ in range(self.MAX_CHAIN):
            record = self._map.get(current)
            if record is None:
                return chain
            nxt = record.survivor_uuid
            if nxt in seen:      # cycle: stop at the contradiction, report the chain
                chain.append(nxt)
                return chain
            chain.append(nxt)
            seen.add(nxt)
            current = nxt
        return chain             # chain too deep: report what was walked, do not spin

    def has(self, track_uuid: str) -> bool:
        return track_uuid in self._map

    def record_for(self, track_uuid: str) -> Optional[AliasRecord]:
        return self._map.get(track_uuid)

    def records(self) -> List[AliasRecord]:
        return list(self._map.values())

    def survivors(self) -> List[str]:
        return sorted({r.survivor_uuid for r in self._map.values()})

    def __len__(self) -> int:
        return len(self._map)

    def __bool__(self) -> bool:
        """Always true. An empty alias map is still the map a caller passed in.

        Without this, ``aliases or AliasMap()`` silently replaces the caller's object every time
        it happens to be empty — and empty is the normal state at startup, which is exactly when
        a caller is most likely to be relying on it.
        """
        return True

    def __iter__(self) -> Iterator[Tuple[str, str]]:
        for record in self._map.values():
            yield record.retired_uuid, record.survivor_uuid

    # -- serialization (diagnostics dumps, replay state restoration) --------
    def to_dict(self) -> Dict[str, Any]:
        return {"aliases": [r.to_dict() for r in self._map.values()]}

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "AliasMap":
        out = cls()
        for item in (data or {}).get("aliases") or ():
            try:
                out.add(str(item["retired_uuid"]), str(item["survivor_uuid"]),
                        reason=str(item.get("reason", "")),
                        sensor_timestamp_ns=int(item.get("sensor_timestamp_ns", 0)),
                        frame_sequence=int(item.get("frame_sequence", 0)))
            except (KeyError, ValueError):
                continue   # a dumped alias that would now be a cycle is not restorable
        return out
