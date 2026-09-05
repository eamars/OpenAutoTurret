"""Operator-facing commands built on the subsystem, not on top of it.

These modules import the same objects the daemon imports — ``VisionConfig``, ``ReplaySource``,
``run_level_b`` — so a measurement taken here cannot drift from the one the running system
makes. That matters more than it sounds: a bake-off script with its own tracker call is a second
implementation, and the second implementation is where the friendly numbers come from.

Two tools, both deliberately incapable of inventing data:

* :mod:`perception.tools.bakeoff` — Vision-6's deliverable, "an evidence-based selected model
  profile". It compares recordings; it does not capture them, because capturing needs the sensor
  and a tool that fakes capture would be a machine for producing confident nonsense.
* :mod:`perception.tools.commission` — turns §50's four score thresholds and §16's three dedup
  numbers from *invented* into *reviewed*, by measuring what the recording actually separates.
  It refuses to propose a number when the recording does not support one.
"""
from __future__ import annotations

__all__ = ["compare_recordings", "plan_captures", "profile_metrics", "propose_thresholds"]

# Resolved on attribute access rather than imported eagerly: ``python -m
# perception.tools.bakeoff`` executes that module *after* the package __init__ has already
# imported it, which prints a RuntimeWarning about a module being "found in sys.modules" and,
# worse, makes the run depend on import order. A tool nobody can launch cleanly is a tool that
# gets copy-pasted into a scratch script instead.
_MODULES = {"compare_recordings": "bakeoff", "plan_captures": "bakeoff",
            "profile_metrics": "bakeoff", "propose_thresholds": "commission"}


def __getattr__(name: str):
    module_name = _MODULES.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    from importlib import import_module

    return getattr(import_module(f".{module_name}", __name__), name)
