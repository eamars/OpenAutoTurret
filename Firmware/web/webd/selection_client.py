"""Relay native selection commands without selecting by display index."""
import json
import socket
import uuid


def canonical_uuid(text):
    if ':' in str(text):
        hi, lo = map(int, str(text).split(':'))
        if not (0 <= hi < 2**64 and 0 <= lo < 2**64):
            raise ValueError('UUID component out of range')
        return str(uuid.UUID(int=(hi << 64) | lo))
    return str(uuid.UUID(str(text)))


def request_selection(path, request):
    try:
        data = dict(request)
        data['session_uuid'] = canonical_uuid(data['session_uuid'])
        if data.get('type') == 'select_target':
            data['track_uuid'] = canonical_uuid(data['track_uuid'])
        payload = json.dumps(data, allow_nan=False).encode()
        if len(payload) > 4096:
            raise ValueError('selection request too large')
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as client:
            client.settimeout(1)
            client.connect(path)
            client.sendall(payload)
            return json.loads(client.recv(16384))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return {'accepted': False, 'reason': 'SELECTION_UNAVAILABLE', 'detail': str(exc)}
