# serial_controller/protocol/messages.py
#
# One dataclass + one Decoder per wire message type. This is the ONLY
# place bit positions or struct layouts are touched. Everything downstream
# — the translator, mavlink code, anything — only ever sees named fields
# on these objects, never a raw int and never a bit position.

import struct
from dataclasses import dataclass
from serial_controller.protocol.registry import Decoder, MessageType, register
from serial_controller.protocol.bit_definitions import (
    ARM_BIT, RTL_BIT, MANUAL_BIT, AUTOTAKEOFF_BIT, EMERGENCY_BIT,
    SYSTEM_CHECK_BIT, POT_VALUE_SHIFT, POT_VALUE_MASK,
    STATUS_ARMED_BIT, STATUS_MANUAL_MODE_BIT, STATUS_IS_FLYING_BIT,
)


# ── Incoming: STM32 -> PC ────────────────────────────────────────────────────

@dataclass
class Joystick2:
    x: int
    y: int


class Joystick2Decoder(Decoder):
    TYPE = MessageType.JOYSTICK2

    def decode(self, payload: bytes) -> Joystick2:
        x, y = struct.unpack('<HH', payload)
        return Joystick2(x=x, y=y)

    def encode(self, obj: Joystick2) -> bytes:
        return struct.pack('<HH', obj.x, obj.y)


register(Joystick2Decoder())

@dataclass
class ButtonState:
    """
    Discrete button flags + the pot reading, decoded from the STM32's
    button register. arm/rtl/manual/takeoff/emergency/system_check are
    LEVEL values here (current switch position) — edge detection happens
    later, in commands/translator.py. This class only decodes; it does
    not interpret.
    """
    arm: bool
    rtl: bool
    manual: bool
    takeoff: bool
    emergency: bool
    system_check: bool
    pot_value: int   # 0-255, raw


class ButtonStateDecoder(Decoder):
    TYPE = MessageType.BUTTON_STATE

    def decode(self, payload: bytes) -> ButtonState:
        value = struct.unpack('<I', payload)[0]
        return ButtonState(
            arm          = bool((value >> ARM_BIT) & 1),
            rtl          = bool((value >> RTL_BIT) & 1),
            manual       = bool((value >> MANUAL_BIT) & 1),
            takeoff      = bool((value >> AUTOTAKEOFF_BIT) & 1),
            emergency    = bool((value >> EMERGENCY_BIT) & 1),
            system_check = bool((value >> SYSTEM_CHECK_BIT) & 1),
            pot_value    = (value >> POT_VALUE_SHIFT) & POT_VALUE_MASK,
        )

    def encode(self, obj: ButtonState) -> bytes:
        # Included for symmetry / test-harness use (e.g. simulating STM32
        # input from a PC-side test script). Not used in normal operation.
        value = 0
        value |= (int(obj.arm)          << ARM_BIT)
        value |= (int(obj.rtl)          << RTL_BIT)
        value |= (int(obj.manual)       << MANUAL_BIT)
        value |= (int(obj.takeoff)      << AUTOTAKEOFF_BIT)
        value |= (int(obj.emergency)    << EMERGENCY_BIT)
        value |= (int(obj.system_check) << SYSTEM_CHECK_BIT)
        value |= ((obj.pot_value & POT_VALUE_MASK) << POT_VALUE_SHIFT)
        return struct.pack('<I', value)


register(ButtonStateDecoder())


@dataclass
class Joystick:
    """Raw 12-bit ADC readings. Continuous/analog — not edge-detected,
    not routed through the command bus. Written straight to SystemState
    as a fact, same category as altitude."""
    x: int
    y: int


class JoystickDecoder(Decoder):
    TYPE = MessageType.JOYSTICK

    def decode(self, payload: bytes) -> Joystick:
        x, y = struct.unpack('<HH', payload)
        return Joystick(x=x, y=y)

    def encode(self, obj: Joystick) -> bytes:
        return struct.pack('<HH', obj.x, obj.y)


register(JoystickDecoder())


# ── Outgoing: PC -> STM32 ────────────────────────────────────────────────────

@dataclass
class StatusUpdate:
    """
    What the STM32 needs to know about current UAV state, e.g. to drive
    LEDs. Built from SystemState facts (is_UAV_Armed, current mode,
    is_flying) — see serial_controller/status_builder.py. Deliberately a
    SEPARATE type from ButtonState: this is PC-owned data, not a reflection
    of the incoming register, so the two can evolve independently.
    """
    armed: bool
    manual_mode: bool
    is_flying: bool


class StatusUpdateDecoder(Decoder):
    TYPE = MessageType.STATUS

    def decode(self, payload: bytes) -> StatusUpdate:
        value = struct.unpack('<I', payload)[0]
        return StatusUpdate(
            armed       = bool((value >> STATUS_ARMED_BIT) & 1),
            manual_mode = bool((value >> STATUS_MANUAL_MODE_BIT) & 1),
            is_flying   = bool((value >> STATUS_IS_FLYING_BIT) & 1),
        )

    def encode(self, obj: StatusUpdate) -> bytes:
        value = 0
        value |= (int(obj.armed)       << STATUS_ARMED_BIT)
        value |= (int(obj.manual_mode) << STATUS_MANUAL_MODE_BIT)
        value |= (int(obj.is_flying)   << STATUS_IS_FLYING_BIT)
        return struct.pack('<I', value)


register(StatusUpdateDecoder())
