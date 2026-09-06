"""Bounded local command transport; decisions run on the perception frame owner.

Socket I/O never owns tracker state. Expired queued requests cannot execute later.
The caller receives the selection manager's actual ACK, not a queue receipt.
"""
import collections
import errno
import json
import os
import queue
import socket
import stat
import threading
import time
import uuid

from .protocol import ClearTargetRequest, SelectTargetRequest


class SelectionService:
    def __init__(self, path):
        self.path = path
        self.pending = queue.Queue(maxsize=16)
        self.closed = threading.Event()
        self.responses = collections.OrderedDict()
        self.listener = None
        self.thread = None
        self.owner_lock = None
        self.inode = None

    def start(self):
        import fcntl  # local Linux IPC, like SOCK_SEQPACKET below
        owner_lock = open(self.path + '.lock', 'a')
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            fcntl.flock(owner_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            try:
                old = os.lstat(self.path)
            except FileNotFoundError:
                old = None
            if old is not None:
                if not stat.S_ISSOCK(old.st_mode) or old.st_uid != os.getuid():
                    raise OSError('selection path is not an owned socket')
                # Also protect older daemons that do not participate in our lock.
                with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as check:
                    check.settimeout(.1)
                    try:
                        check.connect(self.path)
                    except OSError as exc:
                        if exc.errno != errno.ECONNREFUSED:
                            raise
                    else:
                        raise OSError('selection service is already running')
                if os.lstat(self.path).st_ino != old.st_ino:
                    raise OSError('selection socket changed during recovery')
                os.unlink(self.path)
            listener.bind(self.path)
            self.inode = os.stat(self.path).st_ino
            os.chmod(self.path, 0o660)
            listener.listen(8)
            listener.settimeout(.1)
        except Exception:
            listener.close()
            owner_lock.close()
            raise
        self.owner_lock = owner_lock
        self.listener = listener
        self.inode = os.stat(self.path).st_ino
        self.thread = threading.Thread(target=self._serve, name='selection-ipc', daemon=True)
        self.thread.start()

    def _serve(self):
        while not self.closed.is_set():
            try:
                client, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with client:
                client.settimeout(.8)
                try:
                    raw = client.recv(4097)
                    if len(raw) > 4096:
                        raise ValueError('selection request exceeds 4096 bytes')
                    data = json.loads(raw)
                    if not isinstance(data, dict):
                        raise ValueError('selection request must be an object')
                    item = dict(data=data, expires=time.monotonic()+.6,
                                lock=threading.Lock(), done=threading.Event(), reply=None)
                    self.pending.put_nowait(item)
                    item['done'].wait(.65)
                    with item['lock']:
                        if item['reply'] is None:
                            item['expires'] = 0
                            item['reply'] = {'accepted': False, 'reason': 'PERCEPTION_UNAVAILABLE'}
                        reply = item['reply']
                except (ValueError, OSError, queue.Full) as exc:
                    reply = {'accepted': False, 'reason': 'INVALID_OR_BUSY', 'detail': str(exc)}
                try:
                    client.sendall(json.dumps(reply, allow_nan=False).encode())
                except OSError:
                    pass

    def process(self, selector, track_set, now_ns):
        for _ in range(16):
            try:
                item = self.pending.get_nowait()
            except queue.Empty:
                return
            with item['lock']:
                if time.monotonic() > item['expires']:
                    continue
                try:
                    item['reply'] = self._decide(selector, track_set, now_ns, item['data'])
                except (ValueError, TypeError, KeyError) as exc:
                    item['reply'] = {'accepted': False, 'reason': 'INVALID_REQUEST', 'detail': str(exc)}
                item['done'].set()

    def _decide(self, selector, track_set, now_ns, data):
        # The web relay uses hyphenated UUIDs; the tracker creates uuid4().hex.
        # Compare identities, not their textual spelling.
        if uuid.UUID(str(data.get('session_uuid', ''))) != uuid.UUID(track_set.session_uuid):
            return {'accepted': False, 'reason': 'STALE_SESSION'}
        if not 0 <= now_ns-track_set.sensor_timestamp_ns <= 250_000_000:
            return {'accepted': False, 'reason': 'STALE_PERCEPTION_FRAME'}
        request_id = str(data.get('request_id', ''))
        if not request_id or len(request_id) > 128:
            raise ValueError('request_id is required (maximum 128 characters)')
        key = (track_set.session_uuid, request_id)
        fingerprint = json.dumps(data, sort_keys=True)
        if key in self.responses:
            previous, reply = self.responses[key]
            if previous != fingerprint:
                raise ValueError('request_id reused for a different command')
            return reply
        if data.get('type') == 'select_target':
            requested = uuid.UUID(data['track_uuid']).hex
            request = SelectTargetRequest(request_id=request_id, track_uuid=requested,
                track_set_sequence_seen_by_ui=int(data['track_set_sequence_seen_by_ui']))
            if request.track_set_sequence_seen_by_ui > track_set.track_set_sequence:
                raise ValueError('UI sequence is in the future')
            ack = selector.select(request, track_set, now_ns)
        elif data.get('type') == 'clear_target':
            ack = selector.clear(ClearTargetRequest(request_id=request_id), track_set, now_ns)
        else:
            raise ValueError('unknown selection command')
        reply = ack.to_dict()
        reply['session_uuid'] = track_set.session_uuid
        self.responses[key] = (fingerprint, reply)
        while len(self.responses) > 128:
            self.responses.popitem(last=False)
        return reply

    def close(self):
        self.closed.set()
        if self.listener is not None:
            self.listener.close()
        if self.thread is not None:
            self.thread.join(timeout=2)
        try:
            if os.stat(self.path).st_ino == self.inode:
                os.unlink(self.path)
        except (FileNotFoundError, AttributeError):
            pass
        if self.owner_lock is not None:
            self.owner_lock.close()
            self.owner_lock = None
