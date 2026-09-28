"""Perception subpackage: identity formation (§17–§26).

``track.py`` holds the identity type and its geometry; ``track_manager.py`` owns the
time-based lifecycle; ``byte_association.py`` owns the two-stage assignment;
``duplicate_track_resolver.py`` owns §25's merges; ``diagnostics.py`` owns §41's bounded
trace. No module here may import a motor backend (§49).
"""
