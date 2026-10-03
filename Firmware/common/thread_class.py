"""Per-thread priority for the Python daemons (docs/operations/os-setup.md).

On Linux a thread's niceness is its own, so a daemon's side work (preview encoding, diagnostic
snapshots, health files) can yield to the thread that does the daemon's job without a separate
process. Lowering is always allowed; raising is not, so this only ever adds niceness.
"""
import os
import threading


def lower_this_thread(delta: int = 10) -> bool:
    """Add ``delta`` to the calling thread's niceness (capped at 19). Never raises."""
    try:
        tid = threading.get_native_id()
        current = os.getpriority(os.PRIO_PROCESS, tid)
        os.setpriority(os.PRIO_PROCESS, tid, min(19, current + int(delta)))
        return True
    except (AttributeError, OSError):
        return False
