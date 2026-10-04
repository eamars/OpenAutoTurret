import uuid
from perception.selection.control_context import ControllerContext


def test_wire_uuid_and_camera_hex_uuid_identify_the_same_session():
    session = uuid.uuid4()
    context = ControllerContext('unused')
    context._state = (1_000_000_000, 990_000_000, str(session), 'AUTO_ROAM')
    assert context.allows_auto_select(session.hex, 1_050_000_000)
    assert not context.allows_auto_select(uuid.uuid4().hex, 1_050_000_000)
    assert not context.allows_auto_select('malformed', 1_050_000_000)
    assert not context.allows_auto_select(session.hex, 1_300_000_000)


def test_fresh_http_cannot_refresh_stale_controller_telemetry():
    session = uuid.uuid4()
    context = ControllerContext('unused')
    context._state = (1_000_000_000, 500_000_000, str(session), 'AUTO_TRACK')
    assert not context.allows_auto_select(session.hex, 1_010_000_000)
    context._state = (1_000_000_000, 990_000_000, str(session), 'MANUAL')
    assert not context.allows_auto_select(session.hex, 1_010_000_000)


def test_selection_flags_follow_the_mode_and_the_return_mode():
    # SURVEILLANCE (owner, 2026-10-05): ranking applies at the watch point and to a track that will
    # return there; "lost" is AUTO_TRACK's own word for having given the target up.
    session = uuid.uuid4()
    context = ControllerContext('unused')

    def flags(mode, phase='', back='AUTO_ROAM'):
        context._state = (1_000_000_000, 990_000_000, str(session), mode, phase, back)
        return context.selection_flags(session.hex, 1_050_000_000)

    watching = flags('SURVEILLANCE', 'WATCH', 'SURVEILLANCE')
    assert watching.auto_track_enabled and watching.auto_roam_enabled and watching.surveillance
    assert not watching.controller_lost
    tracking = flags('AUTO_TRACK', 'TRACKING', 'SURVEILLANCE')
    assert tracking.surveillance and not tracking.auto_roam_enabled and not tracking.controller_lost
    assert flags('AUTO_TRACK', 'LOST_HOLD', 'SURVEILLANCE').controller_lost
    roaming = flags('AUTO_ROAM', 'PATROL')
    assert roaming.auto_roam_enabled and not roaming.surveillance
    assert not flags('AUTO_TRACK', 'LOST_HOLD').surveillance
    assert flags('MANUAL') == type(flags('MANUAL'))()
    # A stale sample says nothing at all, rather than a stale mode.
    assert context.selection_flags(session.hex, 1_400_000_000) == type(flags('MANUAL'))()
    assert context.allows_auto_select(session.hex, 1_050_000_000) is False
    context._state = (1_000_000_000, 990_000_000, str(session), 'SURVEILLANCE')
    assert context.allows_auto_select(session.hex, 1_050_000_000)
