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
