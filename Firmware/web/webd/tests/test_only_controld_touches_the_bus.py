"""§110 COMMON/4: no UI or vision process can write motors directly.

The architecture makes this structural rather than a matter of good behaviour: the CAN bus is
opened by controld and nothing else on the station; webd relays *commands* to controld; visiond
publishes *tracks*. That deserves more than a review note, so it is checked — with the limits of
the check written down beside it, because a guard that overstates itself is worse than none:

  * In scope: the two processes the item names — the web daemon and the vision service. Bench
    tools (`tools/can_supervisor.py`, `turret_can.cpp`) are out of scope on purpose: they are
    diagnostic programs a person runs deliberately, not services running unattended beside a
    live turret, and this station has already caught hardware faults with them that no test
    suite could.
  * What is searched is the vocabulary of *doing it* — importing a CAN or serial stack, asking
    the socket layer for a CAN family, configuring a port, opening a device node. Not the word
    "CAN": these packages discuss the bus constantly, and every one of those mentions is prose
    saying they do not touch it. A guard that flagged prose would be switched off within a week
    by the next person to write a comment.
  * Test files are excluded, since a guard necessarily names the things it forbids.
  * What this cannot show is that no path exists through a dependency. It shows that the source
    of these two services contains no way to open a bus, and that the code which does open one
    lives in the daemon.
"""
from __future__ import annotations

import pathlib

REPO = pathlib.Path(__file__).resolve().parents[4]  # .../web/webd/tests/this -> repo root
WEB = REPO / "Firmware" / "web"
VISION = REPO / "Firmware" / "vision"

BUS_CODE_TOKENS = (
    "import can",
    "from can import",
    "socket.AF_CAN",
    "socket.PF_CAN",
    "CAN_RAW",
    "import spidev",
    "spidev.",
    "import serial",
    "serial.Serial",
    "termios",
    'open("/dev/tty',
    "open('/dev/tty",
    "fcntl.ioctl(",
)


def _sources(root: pathlib.Path):
    """Every non-test .py file under `root`. The exclusion is not cosmetic: this very file
    spells out the tokens it forbids elsewhere, and a guard that scans itself reports a
    violation every time it runs — one way to learn that nobody reads the output."""
    out = []
    for path in sorted(root.rglob("*.py")):
        rel = "/" + path.relative_to(REPO).as_posix()
        if "__pycache__" in rel or "/tests/" in rel:
            continue
        out.append(path)
    return out


def test_both_packages_exist_to_be_checked():
    """A guard pointed at a directory that has moved reads as a clean pass. Assert the scope
    is real before trusting that nothing was found in it."""
    assert WEB.is_dir(), "webd is not where this guard is looking"
    assert VISION.is_dir(), "visiond is not where this guard is looking"
    assert len(_sources(WEB)) > 3, "webd has almost no sources left to guard"
    assert len(_sources(VISION)) > 3, "visiond has almost no sources left to guard"


def test_no_ui_or_vision_source_opens_a_bus():
    offenders = []
    for root in (WEB, VISION):
        for path in _sources(root):
            text = path.read_text(encoding="utf-8")
            for lineno, line in enumerate(text.splitlines(), 1):
                for token in BUS_CODE_TOKENS:
                    if token in line:
                        offenders.append("%s:%d: %s — %r"
                                         % (path.relative_to(REPO).as_posix(), lineno, token,
                                            line.strip()))
    assert not offenders, (
        "§110 says no UI/vision process can write motors directly, and these lines are code "
        "that could open a bus (prose about the bus is fine — the dashboard may explain CAN "
        "health in as many words as it likes):\n  " + "\n  ".join(offenders)
    )


def test_the_bus_is_opened_by_the_control_daemon():
    """The other half. A guard that only looks for forbidden things also passes when the
    permitted implementation disappears, and a safety property with nothing on the allowed
    side of it is vacuous, not true."""
    can_layer = REPO / "Firmware" / "control" / "src" / "can"
    assert can_layer.is_dir(), "the daemon's CAN layer has moved; this guard is now vacuous"
    src = "\n".join(p.read_text(encoding="utf-8")
                    for p in sorted(can_layer.rglob("*")) if p.is_file())
    assert src, "no files under control/src/can"
    assert ("ttyUSB" in src or "CAN_RAW" in src or "spidev" in src), (
        "control/src/can names no device, so nothing here shows where the bus is opened"
    )
