"""Failure vocabulary for the perception subsystem.

Every error here names a *direction of failure*, which matters because this subsystem
sits on top of a machine that can hurt somebody. The rule the types below enforce:

* a **value** that cannot be trusted must fail fast at the boundary that received it
  (§14), not be clamped into plausibility;
* a **model** that disagrees with its manifest must be refused before inference, not
  discovered during it (§9.3);
* a **configuration** that still contains a ``COMMISSION`` placeholder is not a
  configuration (``ConfigPlaceholderError``, §50);
* none of these may ever cause a motor command. The safe outcome of every error in
  this file is "no target published", which controld ages into coast-then-hold (§34).
"""
from __future__ import annotations


class PerceptionError(Exception):
    """Base class for every failure raised by this subsystem."""


class ValidationError(PerceptionError):
    """A value failed a §14 sanity check (out of range, non-finite, wrong dimensions).

    Raised rather than logged, on the theory that a bounding box with ``x_max < x_min``
    is not a measurement with a large error bar — it is a different quantity from the
    one the field name claims, and the controller would happily steer toward it.
    """


class ConfigError(PerceptionError):
    """The configuration cannot be used as written."""


class ConfigPlaceholderError(ConfigError):
    """A required tuning value is still the literal ``COMMISSION`` (§50).

    Separate from ``ConfigError`` because the remedy differs: a missing key is fixed in
    one line, while a placeholder is a measurement task on station recordings. Silently
    defaulting a placeholder to a demo threshold is how one universal ``0.50`` came to
    be applied across five detectors with different score semantics (§3.5).
    """


class ModelRejected(PerceptionError):
    """A model/manifest mismatch (§9.2, §9.3, Appendix D).

    Carries the reason a consumer can publish: ``MODEL_REJECTED_INCOMPATIBLE`` (§42) is
    an event, so the caller decides whether to fall back to a different profile rather
    than inheriting a stack trace from the camera layer.
    """


class AssociationError(PerceptionError):
    """The assignment solver could not be satisfied (a programming error, not a scene)."""


class SelectionError(PerceptionError):
    """A selection command could not be interpreted at all.

    Note the difference from a *rejected* selection: rejection is a normal, publishable
    outcome with a reason code (§29), so it is an ``Ack``, not an exception. An exception
    here means the request was not even a request.
    """
