# serial_controller/protocol/messages.py
#
# One dataclass + one Decoder per wire message type. This is the ONLY
# place bit positions or struct layouts are touched. Everything downstream
# — the translator, mavlink code, anything — only ever sees named fields
# on these objects, never a raw int and never a bit position.

import struct
from dataclasses import dataclass
from serial_controller.protocol.registry import Decoder, MessageType, register
from serial_controller.protocol.bit_definitions import *


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
    auto: bool
    takeoff: bool
    emergency: bool
    autoland: bool
    speedup: bool
    speeddown: bool
    zoomin: bool
    zoomout: bool
    widein: bool
    wideout: bool
    tracking: bool
    focus_in: bool
    focus_out: bool
    video_ip: bool
    laser_on_off: bool
    laser_cont_mode: bool
    laser_single_mode: bool
    ai_tracking: bool
    joystick_track: bool



class ButtonStateDecoder(Decoder):
    TYPE = MessageType.BUTTON_STATE

    def decode(self, payload: bytes) -> ButtonState:
        value = struct.unpack('<I', payload)[0]
        return ButtonState(
            arm          = bool((value >> ARM_BIT) & 1),
            manual       = bool((value >> MANUAL_BIT) & 1),
            takeoff      = bool((value >> 25) & 1),
            auto         = bool((value >> AUTO_BIT) & 1),
            autoland     = bool((value >> AUTO_LAND_BIT) & 1),
            emergency    =  bool((value >> 25) & 1), 
            speedup      = bool((value >> SPEED_UP_BIT) & 1),
            speeddown    = bool((value >> SPEED_DOWN_BIT) & 1),
            zoomin       = bool((value >> ZOOM_IN_BIT) & 1),
            zoomout      = bool((value >> ZOOM_OUT_BIT) & 1),
            rtl          = bool((value >> 25) & 1),
            widein       = bool((value >> WIDE_IN_BIT) & 1),
            wideout      = bool((value >> WIDE_OUT_BIT) & 1),
            tracking     = bool((value >> TRACKING_BIT) & 1),
            focus_in         = bool((value >> FOCUS_IN_BIT) & 1),
            focus_out        = bool((value >> FOCUS_OUT_BIT) & 1),
            video_ip         = bool((value >> VIDEO_IP_BIT) & 1),
            laser_on_off     = bool((value >> LASER_ON_OFF_BIT) & 1),
            laser_cont_mode  = bool((value >> LASER_CONT_MODE_BIT) & 1),
            laser_single_mode= bool((value >> LASER_SINGLE_MODE_BIT) & 1),
            ai_tracking      = bool((value >> AI_TRACKING_ON_OFF_BIT) & 1),
            joystick_track   = bool((value >> JOYSTICK_TRACK_BIT) & 1),

        )


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
    auto_mode: bool
    autoland_mode: bool
    is_flying: bool


class StatusUpdateDecoder(Decoder):
    TYPE = MessageType.STATUS

    def decode(self, payload: bytes) -> StatusUpdate:
        value = struct.unpack('<I', payload)[0]
        return StatusUpdate(
            armed       = bool((value >> ARM_STATUS_BIT) & 1),
            manual_mode = bool((value >> MANUAL_STATUS_BIT) & 1),
            auto_mode = bool((value >> AUTO_STATUS_BIT) & 1),
            autoland_mode = bool((value >> AUTO_LAND_STATUS_BIT) & 1),
            is_flying   = bool((value >> STATUS_IS_FLYING_BIT) & 1),
        )

    def encode(self, obj: StatusUpdate) -> bytes:
        value = 0
        value |= (int(obj.armed)       << ARM_STATUS_BIT)
        value |= (int(obj.manual_mode) << MANUAL_STATUS_BIT)
        value |= (int(obj.auto_mode) << AUTO_STATUS_BIT)
        value |= (int(obj.autoland_mode) << AUTO_LAND_STATUS_BIT)
        value |= (int(obj.is_flying)   << STATUS_IS_FLYING_BIT)
        return struct.pack('<I', value)


register(StatusUpdateDecoder())