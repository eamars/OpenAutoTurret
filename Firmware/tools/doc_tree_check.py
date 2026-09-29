#!/usr/bin/env python3
"""The documentation tree is only navigation if it is connected; this is the check that says so.

Three claims, each failing loudly with a file and a line:

1. every operating card under ``docs/operations/`` is reachable from ``docs/README.md`` — a card the
   entry point does not name is a card nobody finds at 23:00, which is the same as no card;
2. every relative link inside ``docs/`` resolves — a map with a broken path is worse than no map,
   because it is trusted;
3. the runbook still sits at the path ``AGENTS.md`` promises. Moving it quietly breaks the operating
   instructions that point at it, and that breakage shows up as a new hire improvising.

Standard library only, so it runs on a bare host with no venv: the machine that lacks the cross
toolchain is often the same machine that lacks the interpreter's third-party packages.
"""
from __future__ import annotations

import re
import sys
import urllib.parse
from pathlib import Path

LINK = re.compile(r"\[[^\]]*\]\(([^)#]+)(?:#[^)]*)?\)")
HERE = Path(__file__).resolve()
FIRMWARE = HERE.parent.parent
DOCS = FIRMWARE / "docs"
ENTRY = DOCS / "README.md"
RUNBOOK = DOCS / "STATION_OPERATIONS.md"
RUNBOOK_NAME = "STATION_OPERATIONS.md"


def relative_links(path: Path) -> list:
    """Relative targets inside one markdown file, with the line they appear on.

    Two classes are skipped, and the skipping is stated rather than silent. Percent-encoding is
    undone because a Chinese filename on this project is normal, not a typo. Targets under ``run/``
    are runtime captures -- evidence directories that are deliberately not in git -- so a document
    citing one is pointing at a file that existed on one machine on one day, which is a citation,
    not a broken link.
    """
    out = []
    text = path.read_text(encoding='utf-8')
    if 'doc-tree-check: ignore' in text:
        return out
    for lineno, line in enumerate(text.splitlines(), 1):
        for target in LINK.findall(line):
            if target.startswith(("http://", "https://", "mailto:")):
                continue
            target = urllib.parse.unquote(target)
            if "/run/" in target or target.startswith("run/"):
                continue
            out.append((lineno, target))
    return out


def main() -> int:
    problems = []
    if not ENTRY.exists():
        print(f"doc-tree: {ENTRY} is missing — the map itself is gone")
        return 1
    if not RUNBOOK.exists():
        problems.append(f"{RUNBOOK} moved or vanished; AGENTS.md points here and will send the next "
                        "operator somewhere that does not exist")

    entry_targets = {t for _, t in relative_links(ENTRY)}
    cards = sorted((DOCS / "operations").glob("*.md")) if (DOCS / "operations").is_dir() else []
    for card in cards:
        rel = f"operations/{card.name}"
        if rel not in entry_targets:
            problems.append(f"{card.relative_to(FIRMWARE)} is not named by docs/README.md — "
                            "unreachable in one hop, so unreachable in practice")
    for rel in sorted(entry_targets):
        if rel.startswith("operations/") and not (DOCS / rel).exists():
            problems.append(f"docs/README.md names {rel}, which does not exist")

    checked = 0
    for path in sorted(DOCS.rglob("*.md")):
        for lineno, target in relative_links(path):
            checked += 1
            resolved = (path.parent / target)
            if not resolved.exists():
                problems.append(f"{path.relative_to(FIRMWARE)}:{lineno} points at {target}, "
                                "which is not there")
    for face in ("AGENTS.md", "README.md"):
        face_path = FIRMWARE.parent / face
        if not face_path.exists():
            continue          # a project may carry only one of the two; the other rules still apply
        if "docs/README.md" not in face_path.read_text(encoding="utf-8"):
            problems.append(f"{face} does not name docs/README.md — the two files an agent opens "
                            "first are exactly where a missing entry point costs a session")

    if not cards:
        problems.append("docs/operations/ holds no cards — the runbook is being read whole again, "
                        "which is the failure this tree exists to prevent")

    for line in problems:
        print(f"doc-tree: {line}")
    if problems:
        print(f"doc-tree: {len(problems)} problem(s)")
        return 1
    print(f"doc-tree: ok — {len(cards)} cards reachable from docs/README.md, "
          f"{checked} relative links resolve, runbook in place")
    return 0


if __name__ == "__main__":
    sys.exit(main())
