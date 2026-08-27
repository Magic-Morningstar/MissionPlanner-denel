"""Live panel state: which menu is selected, which bindings are engaged.

This is the mutable half. model.py/config.py describe what *exists* and never
change; this describes what is *currently true* and changes constantly.

The backend owns the instance and mutates it. The GUI reads it and subscribes
for repaints. Every public method is safe to call from any thread.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Callable, Iterable

from .model import Binding, Menu, MenuConfig, Unassigned

__all__ = ["PanelState", "PanelSnapshot", "Listener"]


@dataclass(frozen=True)
class PanelSnapshot:
    """A consistent picture of the state at one instant.

    Handed to listeners and returned by `snapshot()`. Frozen, so the GUI can
    render from it without holding a lock or worrying that the backend moved
    something out from under it halfway through a repaint.
    """
    menu_id: str
    menu_bit: int
    active: frozenset[str] = frozenset()

    @property
    def register_value(self) -> int:
        """The bitmask to write to the firmware's menu register."""
        return 1 << self.menu_bit


Listener = Callable[[PanelSnapshot], None]


class PanelState:
    """Current selection + active flags, with change notification."""

    def __init__(self, config: MenuConfig, menu_id: str | None = None) -> None:
        self._config = config
        self._lock = threading.RLock()
        self._menu: Menu = config.by_id(menu_id or config.default_menu_id)
        self._active: set[str] = set()
        self._listeners: list[Listener] = []

    # -- reading ----------------------------------------------------------

    @property
    def config(self) -> MenuConfig:
        return self._config

    @property
    def menu(self) -> Menu:
        """The currently selected menu."""
        with self._lock:
            return self._menu

    @property
    def menu_id(self) -> str:
        with self._lock:
            return self._menu.id

    @property
    def register_value(self) -> int:
        """Bitmask to write to the firmware's menu register."""
        with self._lock:
            return self._menu.mask

    def is_selected(self, menu: Menu | str) -> bool:
        """For painting the active tab."""
        target = menu if isinstance(menu, str) else menu.id
        with self._lock:
            return self._menu.id == target

    def is_active(self, binding: Binding | str) -> bool:
        """For painting a lit/engaged control."""
        key = binding if isinstance(binding, str) else _key(binding)
        if key is None:
            return False
        with self._lock:
            return key in self._active

    def snapshot(self) -> PanelSnapshot:
        with self._lock:
            return PanelSnapshot(
                menu_id=self._menu.id,
                menu_bit=self._menu.bit,
                active=frozenset(self._active),
            )

    # -- writing ----------------------------------------------------------

    def select(self, menu: Menu | str) -> Menu:
        """Switch menus. No-op (and no notification) if already there."""
        target = menu.id if isinstance(menu, Menu) else menu
        with self._lock:
            if self._menu.id == target:
                return self._menu
            self._menu = self._config.by_id(target)
            menu, snap = self._menu, self._snapshot_locked()
        self._notify(snap)
        return menu

    def select_bit(self, bit: int) -> Menu:
        """Sync from a firmware register read, by bit index."""
        return self.select(self._config.by_bit(bit))

    def select_register(self, value: int) -> Menu:
        """Sync from a raw register value (one-hot bitmask)."""
        return self.select(self._config.by_mask(value))

    def set_active(self, binding: Binding | str, value: bool) -> bool:
        """Set a binding's engaged flag. Returns the new value."""
        key = binding if isinstance(binding, str) else _key(binding)
        if key is None:
            raise ValueError(f"{binding!r} cannot hold active state")
        with self._lock:
            changed = (key in self._active) != value
            if not changed:
                return value
            self._active.add(key) if value else self._active.discard(key)
            snap = self._snapshot_locked()
        self._notify(snap)
        return value

    def toggle(self, binding: Binding | str) -> bool:
        """Flip a binding's flag. Returns the new value."""
        key = binding if isinstance(binding, str) else _key(binding)
        if key is None:
            raise ValueError(f"{binding!r} cannot hold active state")
        with self._lock:
            new_value = key not in self._active
        # set_active re-checks under the lock, so a racing writer just means
        # one of the two flips wins — never a notification sent while held.
        return self.set_active(key, new_value)

    def clear_active(self, keys: Iterable[str] | None = None) -> None:
        """Drop all flags, or just the named ones."""
        with self._lock:
            before = set(self._active)
            if keys is None:
                self._active.clear()
            else:
                self._active.difference_update(keys)
            if self._active == before:
                return
            snap = self._snapshot_locked()
        self._notify(snap)

    # -- notification -----------------------------------------------------

    def subscribe(self, listener: Listener) -> Callable[[], None]:
        """Register a repaint callback. Returns an unsubscribe function.

        The callback fires on whichever thread made the change, which is
        usually NOT the GUI thread — marshal it before touching widgets.
        """
        with self._lock:
            self._listeners.append(listener)

        def unsubscribe() -> None:
            with self._lock:
                if listener in self._listeners:
                    self._listeners.remove(listener)

        return unsubscribe

    def _notify(self, snap: PanelSnapshot) -> None:
        # Copy under the lock, then call outside it: a listener that reads
        # state (or unsubscribes) would otherwise re-enter and can deadlock
        # once a second thread is involved.
        with self._lock:
            listeners = tuple(self._listeners)
        for fn in listeners:
            fn(snap)

    def _snapshot_locked(self) -> PanelSnapshot:
        return PanelSnapshot(
            menu_id=self._menu.id,
            menu_bit=self._menu.bit,
            active=frozenset(self._active),
        )

    def __repr__(self) -> str:
        s = self.snapshot()
        return f"<PanelState menu={s.menu_id!r} active={sorted(s.active)}>"


def _key(binding: Binding) -> str | None:
    """The state key for a binding, or None if it can't hold state.

    Toggles key off their 'on' field, so `is_active` means "the on-state is
    engaged" — recording, not merely bound to record.
    """
    if isinstance(binding, Unassigned):
        return None
    for attr in ("field", "field_on"):
        value = getattr(binding, attr, None)
        if value is not None:
            return value
    return None