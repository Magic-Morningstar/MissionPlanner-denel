"""Menu/binding types.

Schema only — the actual data lives in config.py. Both the backend
(command dispatch, firmware bitmask) and the UI (rendering, hit-testing)
import from here, so neither side owns the schema and neither can drift.

Requires Python 3.10+ (kw_only dataclasses, PEP 604 unions).
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from enum import Enum
from typing import Any, Iterable, Iterator

__all__ = [
    "Button", "Status",
    "Binding", "FieldBinding", "ButtonBinding",
    "Unassigned", "Toggle", "Hold",
    "Menu", "MenuConfig", "ConfigError",
    "REGISTER_FIELD",
]

# Protocol field holding the bitmask that selects the active menu.
REGISTER_FIELD = "menu_register"


class ConfigError(ValueError):
    """Raised when config.py describes something impossible."""


# --------------------------------------------------------------------------
# Enums
# --------------------------------------------------------------------------

class Button(str, Enum):
    """The six physical panel buttons."""
    RED = "RED"
    GREEN = "GREEN"
    BLUE = "BLUE"
    YELLOW = "YELLOW"
    WHITE = "WHITE"
    BLACK = "BLACK"


class Status(str, Enum):
    IMPLEMENTED = "implemented"
    PROPOSED = "proposed"


# --------------------------------------------------------------------------
# Bindings
# --------------------------------------------------------------------------

@dataclass(frozen=True, kw_only=True)
class Binding:
    """Base for anything a menu can expose."""
    label: str | None = None
    note: str = ""

    @property
    def is_assigned(self) -> bool:
        return True


@dataclass(frozen=True, kw_only=True)
class FieldBinding(Binding):
    """A protocol field with no physical button attached."""
    field: str


@dataclass(frozen=True, kw_only=True)
class ButtonBinding(Binding):
    """Base for anything bound to a physical button."""
    button: Button


@dataclass(frozen=True, kw_only=True)
class Unassigned(ButtonBinding):
    """A button slot that exists in the layout but does nothing yet."""

    @property
    def is_assigned(self) -> bool:
        return False


@dataclass(frozen=True, kw_only=True)
class Toggle(ButtonBinding):
    """Press flips state: sends command_on or command_off."""
    firmware_handler: str
    field_on: str
    field_off: str
    command_on: str
    command_off: str

    def command_for(self, turning_on: bool) -> str:
        return self.command_on if turning_on else self.command_off

    def field_for(self, turning_on: bool) -> str:
        return self.field_on if turning_on else self.field_off


@dataclass(frozen=True, kw_only=True)
class Hold(ButtonBinding):
    """Press sends a command; release may send another (or nothing)."""
    firmware_bit: str
    field: str
    command_press: str
    command_release: str | None = None


# --------------------------------------------------------------------------
# Menus
# --------------------------------------------------------------------------

@dataclass(frozen=True, kw_only=True)
class Menu:
    id: str
    bit: int
    title: str
    short_title: str
    status: Status = Status.IMPLEMENTED
    bindings: tuple[Binding, ...] = ()
    status_note: str = ""
    menu_note: str = ""

    @property
    def mask(self) -> int:
        """This menu's bit as a one-hot mask, e.g. bit 5 -> 0b0100000."""
        return 1 << self.bit

    @property
    def buttons(self) -> tuple[ButtonBinding, ...]:
        return tuple(b for b in self.bindings if isinstance(b, ButtonBinding))

    @property
    def fields(self) -> tuple[FieldBinding, ...]:
        return tuple(b for b in self.bindings if isinstance(b, FieldBinding))

    def button(self, button: Button) -> ButtonBinding | None:
        """The binding for a physical button, or None if this menu ignores it."""
        for b in self.buttons:
            if b.button is button:
                return b
        return None

    def __iter__(self) -> Iterator[Binding]:
        return iter(self.bindings)


@dataclass(frozen=True, kw_only=True)
class MenuConfig:
    """Root object. Import the instance from config.py; don't build your own."""
    menus: tuple[Menu, ...]
    default_menu_id: str
    description: str = ""

    def __post_init__(self) -> None:
        self.validate()

    # -- lookups ----------------------------------------------------------

    def by_id(self, menu_id: str) -> Menu:
        for m in self.menus:
            if m.id == menu_id:
                return m
        raise KeyError(f"no menu with id {menu_id!r}")

    def by_bit(self, bit: int) -> Menu:
        for m in self.menus:
            if m.bit == bit:
                return m
        raise KeyError(f"no menu on bit {bit}")

    def by_mask(self, mask: int) -> Menu:
        """Decode a raw register value back into a menu."""
        if mask == 0 or mask & (mask - 1):
            raise ValueError(f"register value {mask:#b} is not a single bit")
        return self.by_bit(mask.bit_length() - 1)

    @property
    def default(self) -> Menu:
        return self.by_id(self.default_menu_id)

    def implemented(self) -> tuple[Menu, ...]:
        return tuple(m for m in self.menus if m.status is Status.IMPLEMENTED)

    def __iter__(self) -> Iterator[Menu]:
        return iter(self.menus)

    def __len__(self) -> int:
        return len(self.menus)

    # -- validation -------------------------------------------------------

    def validate(self) -> None:
        """Runs at import. A typo should stop the app, not wait for a click."""
        _no_duplicates([m.id for m in self.menus], "menu id")
        _no_duplicates([m.bit for m in self.menus], "menu bit")

        for menu in self.menus:
            if menu.bit < 0:
                raise ConfigError(f"menu {menu.id!r} has negative bit {menu.bit}")
            _no_duplicates(
                [b.button for b in menu.buttons], f"button in menu {menu.id!r}"
            )

        if not any(m.id == self.default_menu_id for m in self.menus):
            raise ConfigError(f"default_menu_id {self.default_menu_id!r} is not a menu")


def _no_duplicates(values: Iterable[Any], what: str) -> None:
    seen: set[Any] = set()
    for v in values:
        if v in seen:
            raise ConfigError(f"duplicate {what}: {v!r}")
        seen.add(v)