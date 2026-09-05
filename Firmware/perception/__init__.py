"""OpenAutoTurret perception & target-selection subsystem (Vision 1.0).

The independent subsystem defined by
``docs/open_auto_turret_perception_target_selection_architecture_v1.md``:

    IMX500 model adapter -> DetectionNormalizer -> class filter ->
    DetectionDeduplicator -> (CameraMotionCompensator) -> TrackManager ->
    DuplicateTrackResolver -> TargetSelectionManager -> TrackSet +
    SelectedTargetObservation

The output boundary is ``SelectedTargetObservation`` (§34). Nothing in this package
imports a motor backend, a CyberGear term, a CAN frame, a servo gain or a drive mode
(§2, §49): the controller is a consumer of target measurements, which is the whole
reason the subsystem can be replayed against recorded data with the motors disabled
(§43, §55.18).

Why ``perception/`` and not the ``vision/`` directory of §49's tree: §49 lists
``vision/protocol/`` as a package, and the retired v1/v3 tree already has a module
named ``vision/protocol.py``. A package and a module with the same dotted name in the
same directory do not fail loudly — the package wins the import silently and the old
``from .protocol import ...`` lines in the retired tree would resolve against the new
wire types. That is precisely the "two state sources that disagree" failure this
document exists to remove, so the new tree gets its own root instead. The internal
layout below this line is §49's, unchanged.
"""

SUBSYSTEM_VERSION = "1.0.0"

#: Version tag carried by every published message (§34 ``protocol_version``).
PROTOCOL_VERSION = 1
