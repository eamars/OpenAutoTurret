"""Read-only, expiring controller context for optional AUTO_SELECT_SINGLE.

The source is controld's own telemetry socket (``unix:<path>``, the launcher's default since
2026-10-03) or, for tools, an HTTP state URL. It used to be the web UI's ``/api/state``, polled at
10 Hz: the perception pipeline then waited on the one process the station deliberately runs last
(docs/operations/os-setup.md).
"""
import json
import socket
import threading
import time
import urllib.request
import uuid
from dataclasses import dataclass

#: The modes in which perception may pick a target on its own.
AUTO_MODES = ('AUTO_TRACK', 'AUTO_ROAM', 'SURVEILLANCE')
#: The modes that are searching: a target missing for a second is released.
SEARCH_MODES = ('AUTO_ROAM', 'SURVEILLANCE')
#: AUTO_TRACK's own phases for "the target is given up".
LOST_PHASES = ('LOST_HOLD', 'WAIT_TARGET')


@dataclass(frozen=True)
class SelectionFlags:
    """What the selector needs to know about the controller, all False when the context is stale.

    ``surveillance`` (owner ruling 2026-10-05): the automatic cycle waits at a watch point --
    SURVEILLANCE itself, or AUTO_TRACK whose loss returns there -- and candidates are ranked.
    ``controller_lost``: AUTO_TRACK reports the followed target given up.
    """
    auto_track_enabled: bool = False
    auto_roam_enabled: bool = False
    surveillance: bool = False
    controller_lost: bool = False


class ControllerContext:
    def __init__(self, url):
        self.url = url
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._state = None
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, name='selection-context', daemon=True)
        self._thread.start()

    def _run(self):
        if self.url.startswith('unix:'):
            self._run_socket(self.url[len('unix:'):])
            return
        while not self._stop.is_set():
            try:
                with urllib.request.urlopen(self.url, timeout=.15) as response:
                    state = json.loads(response.read(65537))
                sample = self._sample(state)
            except (OSError, ValueError, KeyError, TypeError):
                sample = None
            with self._lock:
                self._state = sample
            self._stop.wait(.1)

    def _run_socket(self, path):
        """controld pushes one telemetry frame per SEQPACKET message (15 Hz); nothing is sent."""
        buffer = bytearray(1 << 20)
        while not self._stop.is_set():
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            try:
                sock.settimeout(.25)
                sock.connect(path)
                while not self._stop.is_set():
                    try:
                        n = sock.recv_into(buffer)
                    except socket.timeout:
                        continue
                    if n <= 0:
                        break
                    try:
                        state = json.loads(buffer[:n])
                        if state.get('type') != 'telemetry':
                            continue
                        sample = self._sample(state)
                    except (ValueError, KeyError, TypeError, AttributeError):
                        sample = None
                    with self._lock:
                        self._state = sample
            except OSError:
                pass
            finally:
                sock.close()
            with self._lock:
                self._state = None
            self._stop.wait(.5)

    @staticmethod
    def _sample(state):
        observed = time.monotonic_ns()
        # Reject cached telemetry even when the transport itself is responding.
        stamp = int(state['ts_ns'])
        hi, lo = map(int, state['perception_session_uuid'].split(':'))
        if not (0 <= hi < 2**64 and 0 <= lo < 2**64):
            raise ValueError('invalid session')
        native = state.get('perception_native', False)
        return (observed, stamp, str(uuid.UUID(int=(hi << 64) | lo)),
                state['operating_mode'] if native else '',
                str(state.get('mode_phase', '')) if native else '',
                str(state.get('auto_return_mode', '')) if native else '')

    def allows_auto_select(self, session_uuid, now_ns):
        return self.operating_mode(session_uuid, now_ns) in AUTO_MODES

    def operating_mode(self, session_uuid, now_ns):
        return self._fresh(session_uuid, now_ns)[0]

    def selection_flags(self, session_uuid, now_ns):
        mode, phase, return_mode = self._fresh(session_uuid, now_ns)
        return SelectionFlags(
            auto_track_enabled=mode in AUTO_MODES,
            auto_roam_enabled=mode in SEARCH_MODES,
            surveillance=mode == 'SURVEILLANCE' or (mode == 'AUTO_TRACK' and return_mode == 'SURVEILLANCE'),
            controller_lost=mode == 'AUTO_TRACK' and phase in LOST_PHASES)

    def _fresh(self, session_uuid, now_ns):
        """(mode, mode_phase, auto_return_mode) of a fresh sample for this session, else blanks."""
        with self._lock:
            sample = self._state
        if sample is None:
            return '', '', ''
        received, stamp, session, mode, *rest = sample
        phase, return_mode = (list(rest) + ['', ''])[:2]
        try:
            same_session = uuid.UUID(session) == uuid.UUID(session_uuid)
        except (ValueError, TypeError, AttributeError):
            return '', '', ''
        if (same_session and 0 <= now_ns-received <= 250_000_000 and
                0 <= now_ns-stamp <= 250_000_000):
            return mode, phase, return_mode
        return '', '', ''

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)
