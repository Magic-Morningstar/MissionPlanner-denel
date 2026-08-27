"""The GUI's entire view of the backend.

ui.py imports this and nothing else. Answers three questions:
  what does each button do?  which menu is on?  which flags are set?
"""

from __future__ import annotations

from typing import Any, Callable

from .config import CONFIG
from .model import Button, Hold, Menu, Toggle, Unassigned
from .state import PanelState, PanelSnapshot

_state = PanelState(CONFIG)


# -- what each button does --------------------------------------------------

def menus() -> list[dict[str, Any]]:
    """Every menu, for building the tab strip."""
    return [
        {
            "id": m.id,
            "title": m.title,
            "short_title": m.short_title,
            "selected": _state.is_selected(m),
            "implemented": m.status.value == "implemented",
        }
        for m in CONFIG
    ]


def controls(menu_id: str | None = None) -> list[dict[str, Any]]:
    """Every control in a menu (defaults to the current one), with live flags."""
    menu = CONFIG.by_id(menu_id) if menu_id else _state.menu
    return [_describe(b) for b in menu.bindings]


def _describe(b) -> dict[str, Any]:
    out: dict[str, Any] = {
        "label": b.label,
        "button": getattr(b, "button", None) and b.button.value,
        "active": _state.is_active(b),
        "assigned": b.is_assigned,
        "kind": type(b).__name__.lower(),
    }
    if isinstance(b, Toggle):
        out["does"] = f"{b.command_on} / {b.command_off}"
    elif isinstance(b, Hold):
        out["does"] = b.command_press
    elif isinstance(b, Unassigned):
        out["does"] = None
    else:
        out["does"] = b.field
    return out


# -- which menu is on -------------------------------------------------------

def current_menu() -> str:
    return _state.menu_id


def select(menu_id: str) -> None:
    _state.select(menu_id)


def register_value() -> int:
    """The current menu as the firmware's 2-bit binary index (0-3),
    matching sync_from_register()'s input format exactly — not the
    internal one-hot mask _state.register_value holds. If this is ever
    used to write a menu selection back to the STM32, it needs to be
    in the same format menu_select decodes on the way in."""
    return _state.menu.bit


def sync_from_register(value: int) -> None:
    """Firmware changed the menu on its own; catch up.

    `value` is the firmware's raw menu_select field — a 2-bit BINARY
    index (0-3: 0=menu1, 1=menu2, 2=menu3, 3=menu4), not a one-hot mask.
    Everything in model.py/state.py (Menu.mask, by_mask, register_value)
    works in one-hot masks instead — by_mask() specifically requires
    exactly one bit set, so feeding it the raw binary index directly
    is wrong for 3 of the 4 possible values (0 and 3 raise outright,
    2 silently decodes as the wrong menu). Converting here, at the one
    place raw wire data enters this module, rather than changing the
    one-hot convention everywhere else that already depends on it.
    """
    _state.select_register(1 << value)


# -- flags ------------------------------------------------------------------

def is_active(field: str) -> bool:
    return _state.is_active(field)


def set_active(field: str, value: bool) -> None:
    _state.set_active(field, value)


def toggle(field: str) -> bool:
    return _state.toggle(field)


def active_flags() -> set[str]:
    return set(_state.snapshot().active)


# -- repaint hook -----------------------------------------------------------

def subscribe(callback: Callable[[PanelSnapshot], None]) -> Callable[[], None]:
    """Called whenever the menu or any flag changes. See state.py on threads."""
    return _state.subscribe(callback)