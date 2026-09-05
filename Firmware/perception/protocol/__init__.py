"""The subsystem's published messages (§13 DetectionSet, §9 TrackSet, §34 observation).

JSON on the wire, versioned, with a stable field list per message. A binary struct was
the alternative and was rejected for a specific reason: these messages are consumed by a
browser, by the recorder (§43), by the replay harness and by a C++ controller that only
needs one of them. A length-dispatched struct keeps one of those consumers happy and
makes every schema change a two-language coordinated release; a versioned JSON envelope
makes the schema readable in the recording it describes, which is the difference between
a replay that can be audited and a file nobody can open after the fact.

The controller-facing surface stays minimal: `selected_target.py` contains no motor ID,
no CAN frame, no CyberGear term, no servo gain and no drive mode (§2).
"""
