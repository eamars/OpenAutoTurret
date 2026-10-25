"""Relay native selection commands without selecting by display index."""
import json
import socket
import time
import uuid


def canonical_uuid(text):
    if ':' in str(text):
        hi, lo = map(int, str(text).split(':'))
        if not (0 <= hi < 2**64 and 0 <= lo < 2**64):
            raise ValueError('UUID component out of range')
        return str(uuid.UUID(int=(hi << 64) | lo))
    return str(uuid.UUID(str(text)))


def _exchange(path, data):
    payload = json.dumps(data, allow_nan=False).encode()
    if len(payload) > 4096:
        raise ValueError('selection request too large')
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as client:
        client.settimeout(1)
        client.connect(path)
        client.sendall(payload)
        return json.loads(client.recv(16384))


def request_main_camera(path, role):
    """Ask visiond to put `role` on the main display (and so on the Hailo)."""
    if role not in ('wide', 'detail'):
        return {'accepted': False, 'reason': 'INVALID_REQUEST',
                'detail': f'unknown camera role {role!r}; legal: wide, detail'}
    try:
        return _exchange(path, {'type': 'set_main_camera', 'role': role})
    except (OSError, ValueError) as exc:
        return {'accepted': False, 'reason': 'SELECTION_UNAVAILABLE', 'detail': str(exc)}


def request_selection(path, request):
    try:
        data = dict(request)
        data['session_uuid'] = canonical_uuid(data['session_uuid'])
        if data.get('type') == 'select_target':
            data['track_uuid'] = canonical_uuid(data['track_uuid'])
        return _exchange(path, data)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return {'accepted': False, 'reason': 'SELECTION_UNAVAILABLE', 'detail': str(exc)}


def request_snapshot(path, role):
    """Ask visiond to keep `role`'s next full-resolution frame (ADR-006). The reply names the file and
    the request id its JPEG will carry."""
    if role not in ('wide', 'detail'):
        return {'accepted': False, 'reason': 'INVALID_REQUEST',
                'detail': f'unknown camera role {role!r}; legal: wide, detail'}
    try:
        return _exchange(path, {'type': 'snapshot', 'role': role})
    except (OSError, ValueError) as exc:
        return {'accepted': False, 'reason': 'SELECTION_UNAVAILABLE', 'detail': str(exc)}


def snapshot_comment(jpeg):
    """The OTA_FRAME comment visiond writes right after the JPEG's start (perception/snapshot.py)."""
    marker = b'OTA_FRAME\x00'
    if len(jpeg) < 6 or jpeg[:4] != b'\xff\xd8\xff\xfe':
        return None
    payload = jpeg[6:4 + int.from_bytes(jpeg[4:6], 'big')]
    if not payload.startswith(marker):
        return None
    try:
        value = json.loads(payload[len(marker):].decode('utf-8'))
    except ValueError:
        return None
    return value if isinstance(value, dict) else None


def wait_for_snapshot(path, request_id, timeout_s, poll_s=0.02):
    """The snapshot file once it carries `request_id`: (jpeg, its comment), or (None, None)."""
    end = time.monotonic() + timeout_s
    while True:
        try:
            with open(path, 'rb') as f:
                jpeg = f.read()
        except OSError:
            jpeg = b''
        meta = snapshot_comment(jpeg) if jpeg else None
        if meta is not None and meta.get('request') == request_id:
            return jpeg, meta
        if time.monotonic() >= end:
            return None, None
        time.sleep(poll_s)
