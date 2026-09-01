"""Shared, hardware-agnostic utilities for OpenAutoTurret production software.

Kept dependency-light (no numpy / picamera2 required just to import) so both the
web daemon (``web.webd``) and the vision daemon (``vision``) can use the same
primitives without pulling in a camera stack.
"""
