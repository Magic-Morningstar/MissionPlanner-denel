# serial_controller/protocol/messages.py
#
# Two wire messages, both carrying pure physical state. Neither payload
# means anything on its own — meaning is applied in input_router.py.
#
# There are no bit-position constants in this file because there are no
# named bits left. A slot bitmap is ten switches; which switch matters
# depends on a menu the firmware has never heard of. bit_definitions.py
# is deleted entirely.

import struct
from dataclasses import dataclass

from serial_controller.protocol.registry import Decoder, MessageType, register
from state.system_config import (
    SLOT_COUNT, POT_COUNT,
    PANEL_STATE_PAYLOAD_LEN, LED_STATE_PAYLOAD_LEN,
)


# ── Incoming: STM32 -> PC ────────────────────────────────────────────────────

@dataclass(frozen=True)
class PanelState:
    """
    One snapshot of the physical panel, sent every 10 ms.

    `slots` is a bitmap of debounced button levels — bit N is slot N, set
    while pressed. Levels, not edges: edge detection happens in
    input_router.py, where the menu context needed to interpret an edge
    actually exists.

    `pots` is four raw 12-bit ADC averages in ADC channel order
    (9, 15, 6, 7). Continuous values, written to SystemState as facts.
    """
    slots: int
    pots: tuple

    def is_pressed(self, slot: int) -> bool:
        return bool((self.slots >> slot) & 1)

    def pressed_slots(self):
        return [i for i in range(SLOT_COUNT) if (self.slots >> i) & 1]


class PanelStateDecoder(Decoder):
    TYPE = MessageType.PANEL_STATE

    _FMT = '<H' + 'H' * POT_COUNT   # uint16 bitmap + 4 x uint16 pots = 10 bytes

    def decode(self, payload: bytes) -> PanelState:
        if len(payload) != PANEL_STATE_PAYLOAD_LEN:
            raise ValueError(
                f"PANEL_STATE expects {PANEL_STATE_PAYLOAD_LEN} bytes, got {len(payload)}"
            )
        values = struct.unpack(self._FMT, payload)
        return PanelState(slots=values[0], pots=values[1:])

    def encode(self, obj: PanelState) -> bytes:
        return struct.pack(self._FMT, obj.slots, *obj.pots)


register(PanelStateDecoder())


# ── Outgoing: PC -> STM32 ────────────────────────────────────────────────────

@dataclass(frozen=True)
class LedState:
    """
    Which menu is selected. One byte, and that is the entire outgoing
    protocol.

    No pixel data, no colours, no status. All LED behaviour is in main.c:
    what a menu looks like, every GCS blink rate, and the state machine
    that decides between them. Python owns menu selection because menu
    up/down are just slots it edge-detects — the firmware turns that into
    light however it likes.

    Link state in particular is never sent. The firmware infers waiting /
    connected / synced / lost from when these frames arrive, which is the
    only workable arrangement: a PC that has stopped running cannot send
    a frame announcing it.

    Display only — an out-of-range index lights nothing and changes no
    behaviour.
    """
    menu_index: int


class LedStateDecoder(Decoder):
    TYPE = MessageType.LED_STATE

    def decode(self, payload: bytes) -> LedState:
        # The PC never receives this frame. Kept so a round-trip test can
        # assert encode -> decode without a second code path.
        if len(payload) != LED_STATE_PAYLOAD_LEN:
            raise ValueError(
                f"LED_STATE expects {LED_STATE_PAYLOAD_LEN} bytes, got {len(payload)}"
            )
        return LedState(menu_index=payload[0])

    def encode(self, obj: LedState) -> bytes:
        return bytes((obj.menu_index & 0xFF,))


register(LedStateDecoder())