# serial_controller/input_router.py
#
# Where physical state becomes meaning. Everything the firmware used to
# do with menu_register, Poll_Debounced, and the on*_Button_Press
# handlers happens here instead.
#
# Responsibilities:
#   - track the menu index (the firmware no longer has one)
#   - detect rising edges on slot levels
#   - hold latched toggle/radio state
#   - emit PanelEvent objects to the translator
#   - write continuous facts (pots, menu) straight to SystemState
#   - build the outgoing LedState
#
# Pots and menu go to SystemState directly because they're facts in the
# same category as altitude — system_state.py's own docstring draws that
# line. Discrete presses go through the translator to the command bus.

import logging
from dataclasses import dataclass

from state.system_config import SLOT_MENU_UP, SLOT_MENU_DOWN
from serial_controller.panel_bindings import (
    bindings_for, Momentary, Toggle, PairToggle, Radio,
)
from serial_controller.protocol.messages import LedState

logger = logging.getLogger(__name__)

# Menu bits present in API/config.py, in cycle order. Kept as a literal
# rather than imported so serial_controller doesn't depend on the API
# package; when panel_bindings.py folds into API/config.py this becomes
# tuple(m.bit for m in CONFIG).
MENU_BITS = (0, 1, 2, 3, 4, 5)



@dataclass(frozen=True)
class PanelEvent:
    """One discrete thing a person did. `field` matches API/config.py's
    field strings exactly, so the translator's dispatch table and
    api.set_active() both key off the same name.

    `pressed` is True for a rising edge or a momentary hold, False for a
    momentary release or a latch turning off. Momentary controls emit on
    both edges so the translator can send a stop command on release
    without tracking its own previous state."""
    field: str
    pressed: bool
    menu_bit: int
    latched: bool = False   # True if this came from a Toggle/Radio/PairToggle


class InputRouter:

    def __init__(self, state, translator):
        self.state = state
        self.translator = translator

        self._menu_idx = 0
        self._prev_slots = None          # None until the first frame lands
        self._latched = {}               # field -> bool
        self._radio_selected = {}        # group -> field or None
        self._held = set()               # fields currently held down

    # ── Entry point ───────────────────────────────────────────────────────────

    def handle(self, panel_state):
        """Called once per PANEL_STATE frame, from the serial processor
        thread. Returns True if anything changed that affects the LEDs."""
        self._write_pots(panel_state.pots)

        slots = panel_state.slots

        # First frame after connect: seed the edge baseline and fire
        # nothing. This is what the old HELLO/reset was actually buying —
        # without it, any switch left in the pressed position reads as a
        # fresh press the moment Python attaches.
        if self._prev_slots is None:
            self._prev_slots = slots
            logger.info(f"InputRouter: baseline seeded, slots={slots:#012b}")
            return True

        rising = slots & ~self._prev_slots
        falling = self._prev_slots & ~slots
        self._prev_slots = slots

        changed = self._handle_menu(rising)
        if changed:
            # A menu change suppresses every other edge in this frame.
            # Holding a button across a menu switch would otherwise fire
            # whatever that slot means in the NEW menu — the held button
            # is treated as released and needs a fresh press.
            self._held.clear()
            return True

        changed |= self._handle_bindings(slots, rising, falling)
        return changed

    # ── Facts ─────────────────────────────────────────────────────────────────

    def _write_pots(self, pots):
        """pots is (ch9, ch15, ch6, ch7). Channels 9 and 15 are the
        flight joystick, 6 and 7 the payload joystick.

        Heads up: ADC_CHANNEL_9 and ADC_CHANNEL_15 were commented out in
        the old main.c, so pots[0] and pots[1] were always zero. They're
        re-enabled in the new firmware — if the flight stick reads
        garbage, that's why, and the two lines to check are in the main
        loop's ADC block."""
        self.state.update_Joystick(pots[0], pots[1])
        self.state.update_Payload_Joystick(pots[2], pots[3])

    # ── Menu ──────────────────────────────────────────────────────────────────

    def _handle_menu(self, rising) -> bool:
        up = bool((rising >> SLOT_MENU_UP) & 1)
        down = bool((rising >> SLOT_MENU_DOWN) & 1)

        if up == down:          # neither, or both in the same frame
            return False

        n = len(MENU_BITS)
        self._menu_idx = (self._menu_idx + (1 if up else -1)) % n

        # Modulo both ways, so up and down traverse the same set. The old
        # firmware had up cycling through seven positions and down
        # through six, with the seventh having no bindings at all.
        bit = self.current_menu_bit
        self.state.update_Current_Menu(bit)
        logger.info(f"InputRouter: menu -> {bit}")
        return True

    @property
    def current_menu_bit(self) -> int:
        return MENU_BITS[self._menu_idx]

    # ── Bindings ──────────────────────────────────────────────────────────────

    def _handle_bindings(self, slots, rising, falling) -> bool:
        menu_bit = self.current_menu_bit
        bindings = bindings_for(menu_bit)

        if not bindings:
            logger.debug(f"InputRouter: menu {menu_bit} has no bindings")
            return False

        changed = False

        for slot, behaviour in bindings.items():
            went_down = bool((rising >> slot) & 1)
            went_up = bool((falling >> slot) & 1)

            if isinstance(behaviour, Momentary):
                if went_down:
                    self._held.add(behaviour.field)
                    self._emit(behaviour.field, True, menu_bit)
                    changed = True
                elif went_up:
                    self._held.discard(behaviour.field)
                    self._emit(behaviour.field, False, menu_bit)
                    changed = True

            elif isinstance(behaviour, Toggle):
                if went_down:
                    new = not self._latched.get(behaviour.field, False)
                    self._latched[behaviour.field] = new
                    self._emit(behaviour.field, new, menu_bit, latched=True)
                    changed = True

            elif isinstance(behaviour, PairToggle):
                if went_down:
                    new = not self._latched.get(behaviour.field_on, False)
                    self._latched[behaviour.field_on] = new
                    field = behaviour.field_on if new else behaviour.field_off
                    self._emit(field, True, menu_bit, latched=True)
                    changed = True

            elif isinstance(behaviour, Radio):
                if went_down:
                    changed |= self._select_radio(behaviour, menu_bit)

        return changed

    def _select_radio(self, behaviour, menu_bit) -> bool:
        """Rising edge selects this field and clears the rest of its
        group. Pressing the already-selected one deselects it, matching
        the old toggle feel."""
        group = behaviour.group
        current = self._radio_selected.get(group)

        if current == behaviour.field:
            self._radio_selected[group] = None
            self._latched[behaviour.field] = False
            self._emit(behaviour.field, False, menu_bit, latched=True)
            return True

        if current is not None:
            self._latched[current] = False
            self._emit(current, False, menu_bit, latched=True)

        self._radio_selected[group] = behaviour.field
        self._latched[behaviour.field] = True
        self._emit(behaviour.field, True, menu_bit, latched=True)
        return True

    def _emit(self, field, pressed, menu_bit, latched=False):
        event = PanelEvent(
            field=field, pressed=pressed, menu_bit=menu_bit, latched=latched
        )
        try:
            self.translator.handle(event)
        except Exception:
            # One bad handler must not kill the serial processor thread
            # and stall every subsequent frame.
            logger.exception(f"InputRouter: translator raised on {field}")

    # ── Outgoing LEDs ─────────────────────────────────────────────────────────

    def led_state(self) -> LedState:
        """Just the menu index. What it looks like — colour, brightness,
        whether the strip reads as a gauge or a single dot — is main.c's
        MENU_PIXELS table, not this file's business."""
        return LedState(menu_index=self._menu_idx)

    # ── Introspection ─────────────────────────────────────────────────────────

    def held_fields(self) -> set:
        """Momentary controls currently held down."""
        return set(self._held)

    def latched_fields(self) -> dict:
        """Snapshot of every latched field. panel_sync.py can read this
        instead of rebuilding recording state from two momentary pulses."""
        return dict(self._latched)
