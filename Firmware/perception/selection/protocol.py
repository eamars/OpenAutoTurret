"""The selection command protocol (§29, §30).

The retired protocol sent ``select_target 2`` — a number the browser had painted, into a
daemon whose identity list moved underneath it. The handover's evidence for why that
cannot work is a single log line (§3.4): ``select_target 2 -> "selected Person #1"``,
with labels recycled underneath the click. A display index is a *rendering*, and a
rendering cannot be a control identifier because two clients can be looking at different
renders of the same instant.

So the request carries the immutable ``track_uuid`` plus ``track_set_sequence_seen_by_ui``
— the version of the candidate list the operator was actually looking at. That second
field is what makes the failure honest: without it, a click on a track that retired four
frames ago is indistinguishable from a click on a live one, and the daemon either
accepts a command about a nonexistent person or rejects a command it should have honoured.
Either way nobody can tell what the operator saw.

The ACK is authoritative (§29's closing line: "the browser displays selection only after
authoritative state confirms it"). A client that painted its own success the moment it
sent a request made a rejected selection look like an accepted one, and an operator who
believes the turret is following a person it is not following has been told a lie by the
interface.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict

from ..protocol.jsonio import dumps, loads


class SelectionReason(str, Enum):
    """§29's reason codes, plus the two the state machine needs to be complete.

    ``SELECTION_UNCHANGED`` is §30's idempotency report: an ACK on its own cannot tell
    "this command changed something" from "this command was a duplicate of one already in
    force", and the difference is exactly what decides whether the controller resets
    acquisition. ``TRACK_IDENTITY_UNRESOLVED`` is Appendix A's rejection for an ambiguous
    or still-merging track, which is neither "not found" nor "not confirmed" — claiming
    otherwise would make §32's carefully separated ambiguity look like a dropout.
    """

    ACCEPTED = "ACCEPTED"
    SELECTION_UNCHANGED = "SELECTION_UNCHANGED"
    CLEARED = "CLEARED"
    NOTHING_TO_CLEAR = "NOTHING_TO_CLEAR"

    TRACK_NOT_FOUND = "TRACK_NOT_FOUND"
    TRACK_NOT_CONFIRMED = "TRACK_NOT_CONFIRMED"
    TRACK_NOT_CURRENTLY_SELECTABLE = "TRACK_NOT_CURRENTLY_SELECTABLE"
    TRACK_IDENTITY_UNRESOLVED = "TRACK_IDENTITY_UNRESOLVED"
    TRACK_MERGED_USE_SURVIVOR = "TRACK_MERGED_USE_SURVIVOR"
    STALE_UI_TRACK_SET = "STALE_UI_TRACK_SET"
    POLICY_DENIED = "POLICY_DENIED"


REJECT_REASONS = frozenset({
    SelectionReason.TRACK_NOT_FOUND,
    SelectionReason.TRACK_NOT_CONFIRMED,
    SelectionReason.TRACK_NOT_CURRENTLY_SELECTABLE,
    SelectionReason.TRACK_IDENTITY_UNRESOLVED,
    SelectionReason.TRACK_MERGED_USE_SURVIVOR,
    SelectionReason.STALE_UI_TRACK_SET,
    SelectionReason.POLICY_DENIED,
})


@dataclass
class SelectTargetRequest:
    """§29's request. Display labels are not accepted, at any level of this API (§27)."""

    request_id: str = ""
    track_uuid: str = ""
    track_set_sequence_seen_by_ui: int = 0
    # Diagnostics only: which surface the command came from. Never consulted for
    # authority — an operator click and a scripted test command mean the same thing.
    source: str = "ui"

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": "select_target",
            "request_id": self.request_id,
            "track_uuid": self.track_uuid,
            "track_set_sequence_seen_by_ui": int(self.track_set_sequence_seen_by_ui),
            "source": self.source,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SelectTargetRequest":
        return cls(
            request_id=str(data.get("request_id", "")),
            track_uuid=str(data.get("track_uuid", "")),
            track_set_sequence_seen_by_ui=int(
                data.get("track_set_sequence_seen_by_ui", 0)),
            source=str(data.get("source", "ui")),
        )


@dataclass
class ClearTargetRequest:
    """Explicitly drop the selection (§31's "Operator can: clear").

    Explicit rather than inferred from "the track went away": §31 says a selection
    survives loss, so the only way a selection may end on its own is the identity TTL
    expiring, and that transition is reported as ``SELECTED_STALE``, not as silence.
    """

    request_id: str = ""
    source: str = "ui"

    def to_dict(self) -> Dict[str, Any]:
        return {"type": "clear_target", "request_id": self.request_id,
                "source": self.source}

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ClearTargetRequest":
        return cls(request_id=str(data.get("request_id", "")),
                   source=str(data.get("source", "ui")))


@dataclass
class SelectTargetAck:
    """§29's response, plus §30's ``selection_unchanged`` flag.

    ``authoritative_track_set_sequence`` is the sequence the daemon actually used to
    decide, so a client that was looking at an older list can repaint and let the operator
    re-decide on information that matches the machine's.
    """

    request_id: str = ""
    accepted: bool = False
    reason: SelectionReason = SelectionReason.ACCEPTED
    detail: str = ""

    selected_track_uuid: str = ""
    selected_display_label: str = ""

    authoritative_track_set_sequence: int = 0
    selection_generation: int = 0
    selection_unchanged: bool = False

    @property
    def rejected(self) -> bool:
        return not self.accepted

    @property
    def is_idempotent(self) -> bool:
        """§30: accepted, and nothing changed as a result."""
        return self.accepted and self.selection_unchanged

    @classmethod
    def accept(cls, request: SelectTargetRequest, *, track_uuid: str,
               display_label: str, track_set_sequence: int,
               selection_generation: int,
               reason: SelectionReason = SelectionReason.ACCEPTED,
               detail: str = "") -> "SelectTargetAck":
        if reason in REJECT_REASONS:
            raise ValueError(f"{reason} is a rejection, not an acceptance")
        return cls(
            request_id=request.request_id, accepted=True, reason=reason,
            detail=detail, selected_track_uuid=track_uuid,
            selected_display_label=display_label,
            authoritative_track_set_sequence=int(track_set_sequence),
            selection_generation=int(selection_generation),
            selection_unchanged=reason is SelectionReason.SELECTION_UNCHANGED)

    @classmethod
    def reject(cls, request: SelectTargetRequest, reason: SelectionReason, *,
               detail: str = "", track_set_sequence: int = 0,
               selection_generation: int = 0,
               selected_track_uuid: str = "",
               selected_display_label: str = "") -> "SelectTargetAck":
        """A refusal, with the currently authoritative selection echoed back.

        The echo matters: when a request is refused, the client still needs to know what
        the daemon believes is selected. Without it the client keeps painting its own
        optimistic guess — which is the §29 failure this ACK was written to prevent.
        """
        if reason not in REJECT_REASONS:
            raise ValueError(f"{reason} is not a rejection reason")
        return cls(
            request_id=request.request_id, accepted=False, reason=reason, detail=detail,
            selected_track_uuid=selected_track_uuid,
            selected_display_label=selected_display_label,
            authoritative_track_set_sequence=int(track_set_sequence),
            selection_generation=int(selection_generation),
            selection_unchanged=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": "select_target_ack",
            "request_id": self.request_id,
            "accepted": bool(self.accepted),
            "reason": self.reason.value,
            "detail": self.detail,
            "selected_track_uuid": self.selected_track_uuid,
            "selected_display_label": self.selected_display_label,
            "authoritative_track_set_sequence": int(
                self.authoritative_track_set_sequence),
            "selection_generation": int(self.selection_generation),
            "selection_unchanged": bool(self.selection_unchanged),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SelectTargetAck":
        try:
            reason = SelectionReason(data.get("reason", SelectionReason.ACCEPTED.value))
        except ValueError:
            reason = SelectionReason.TRACK_NOT_FOUND
        return cls(
            request_id=str(data.get("request_id", "")),
            accepted=bool(data.get("accepted", False)),
            reason=reason,
            detail=str(data.get("detail", "")),
            selected_track_uuid=str(data.get("selected_track_uuid", "")),
            selected_display_label=str(data.get("selected_display_label", "")),
            authoritative_track_set_sequence=int(
                data.get("authoritative_track_set_sequence", 0)),
            selection_generation=int(data.get("selection_generation", 0)),
            selection_unchanged=bool(data.get("selection_unchanged", False)),
        )

    def to_json(self) -> str:
        return dumps(self.to_dict())

    @classmethod
    def from_json(cls, text: str) -> "SelectTargetAck":
        return cls.from_dict(loads(text))
