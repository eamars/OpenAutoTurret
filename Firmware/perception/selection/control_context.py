"""Read-only, expiring controller context for optional AUTO_SELECT_SINGLE."""
import json
import threading
import time
import urllib.request
import uuid


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
        while not self._stop.is_set():
            try:
                with urllib.request.urlopen(self.url, timeout=.15) as response:
                    state = json.loads(response.read(65537))
                observed = time.monotonic_ns()
                # Reject cached telemetry even when HTTP itself is responding.
                stamp = int(state['ts_ns'])
                hi, lo = map(int, state['perception_session_uuid'].split(':'))
                if not (0 <= hi < 2**64 and 0 <= lo < 2**64):
                    raise ValueError('invalid session')
                sample = (observed, stamp, str(uuid.UUID(int=(hi << 64) | lo)),
                          state['operating_mode'] if state.get('perception_native', False) else '')
            except (OSError, ValueError, KeyError, TypeError):
                sample = None
            with self._lock:
                self._state = sample
            self._stop.wait(.1)

    def allows_auto_select(self, session_uuid, now_ns):
        return self.operating_mode(session_uuid, now_ns) in ('AUTO_TRACK', 'AUTO_ROAM')

    def operating_mode(self, session_uuid, now_ns):
        with self._lock:
            sample = self._state
        if sample is None:
            return ''
        received, stamp, session, mode = sample
        try:
            same_session = uuid.UUID(session) == uuid.UUID(session_uuid)
        except (ValueError, TypeError, AttributeError):
            return ''
        if (same_session and 0 <= now_ns-received <= 250_000_000 and
                0 <= now_ns-stamp <= 250_000_000):
            return mode
        return ''

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)
