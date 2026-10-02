"""Which camera is on the operator's main display -- and therefore the one camera the Hailo sees.

Owner, 2026-10-02: "only the one on the main display should be fed into the AI HAT, while the one in
the PIP is for display purpose." So the swap button on the HUD is not a page-local layout choice: it
is a station state, held here, changed through the selection socket, published in the inference
health document, and every page (and every reload) draws its panes from it. Every start begins on
`wide` (owner: the wide view is what AUTO_ROAM searches with); the choice is not persisted.
"""
from __future__ import annotations

import threading
from typing import Dict, Iterable, Tuple

ROLES = ("wide", "detail")


class MainCamera:
    def __init__(self, available: Iterable[str] = ("wide",)) -> None:
        self._lock = threading.Lock()
        self._available = {r for r in available if r in ROLES}
        self._role = "wide"
        self._generation = 0
        self.last_refusal = ""

    def make_available(self, role: str) -> None:
        with self._lock:
            if role in ROLES:
                self._available.add(role)

    def make_unavailable(self, role: str) -> None:
        """A camera that went away cannot stay on the main display: inference returns to wide."""
        with self._lock:
            self._available.discard(role)
            if self._role == role and role != "wide":
                self._role, self._generation = "wide", self._generation + 1

    def request(self, role: str) -> Tuple[bool, str]:
        role = str(role or "").strip().lower()
        with self._lock:
            if role not in ROLES:
                self.last_refusal = f"unknown camera role {role!r}; legal: {', '.join(ROLES)}"
                return False, self.last_refusal
            if role not in self._available:
                self.last_refusal = f"the {role} camera is not running in this boot"
                return False, self.last_refusal
            if role != self._role:
                self._role, self._generation = role, self._generation + 1
            return True, role

    @property
    def role(self) -> str:
        with self._lock:
            return self._role

    def state(self) -> Tuple[str, int]:
        with self._lock:
            return self._role, self._generation

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            return {"role": self._role, "generation": self._generation,
                    "available": sorted(self._available), "last_refusal": self.last_refusal}
