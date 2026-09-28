"""The seam ``pipeline.py`` and ``tests/test_track_manager.py`` import from.

The implementation lives in ``vision/track_manager.py``; this module re-exports rather than
copies it, because a second copy of track identity/aliasing is how two managers start disagreeing
about which track is selected. The package path exists so perception does not reach into the
vision service for its own tracking state.
"""
from vision.track_manager import Track, TrackManager, TrackManagerConfig

__all__ = ["Track", "TrackManager", "TrackManagerConfig"]
