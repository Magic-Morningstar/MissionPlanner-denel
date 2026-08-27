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
    Flight/status bits ONLY, decoded from the STM32's BUTTON_STATE
    register. Every payload/camera/laser/tracking concept that used to
    live here (zoomin, focus_in, laser_on_off, ...) has been removed —
    they're decoded from PayloadCommand now (its own 32-bit register,
    message type 0x04), which is where the real firmware actually puts
    them. Keeping them here too was decoding stale bit positions that
    haven't corresponded to anything the firmware sends since the
    PAYLOAD_COMMAND split — see bit_definitions.py's BUTTON_STATE
    section for the current (much shorter) real bit list.

    arm/rtl/manual/auto/takeoff/emergency/autoland/speedup/speeddown are
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
    menu_select: int   # 0-3, raw 2-bit value: 0=menu1, 1=menu2, 2=menu3, 3=menu4



class ButtonStateDecoder(Decoder):
    TYPE = MessageType.BUTTON_STATE

    def decode(self, payload: bytes) -> ButtonState:
        value = struct.unpack('<I', payload)[0]
        return ButtonState(
            arm          = bool((value >> ARM_BIT) & 1),
            manual       = bool((value >> MANUAL_BIT) & 1),
            auto         = bool((value >> AUTO_BIT) & 1),
            speedup      = bool((value >> SPEED_UP_BIT) & 1),
            speeddown    = bool((value >> SPEED_DOWN_BIT) & 1),
            # Two adjacent bits, MENU_SELECT_BIT_0 is the low bit — a
            # plain 2-bit mask shifted down gives the 0-3 value directly.
            menu_select  = (value >> MENU_SELECT_BIT_0) & 0b11,
            # FIXED (carried over from before, still true): takeoff,
            # emergency, and rtl were all previously decoded from bit 25
            # — the same bit the old (now-removed) laser_single_mode
            # mapping also used. None of these three have a dedicated
            # bit anywhere in the current main.c BUTTON_STATE list, which
            # only defines ARM/ARM_STATUS/AUTO/AUTO_STATUS/MANUAL/
            # MANUAL_STATUS/SPEED_UP/SPEED_DOWN. Hardcoded False until
            # real bits are actually assigned, same treatment autoland
            # already has.
            takeoff      = False,
            emergency    = False,
            rtl          = False,
            # FIXED: previously read from AUTO_LAND_BIT, which was
            # actually IR_POLARITY (bit 6) in real firmware — meaning
            # pressing the White button also fired LandCommand. There's
            # no dedicated autoland bit in the current main.c list, so
            # this is hardcoded False until one is actually assigned.
            autoland     = False,
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


@dataclass
class PayloadCommand:
    """
    Payload/gimbal/camera button state — its own 32-bit register,
    separate from BUTTON_STATE, so payload buttons never collide with
    flight-control bits (see bit_definitions.py's note on the old
    autoland/IR-polarity collision this kind of thing caused before).

    30 fields, bits 0-29 of a uint32 (bits 30/31 spare) — see
    bit_definitions.py's PAYLOAD_COMMAND section for the exact numbering,
    verified against the real firmware.
    """
    zoomin: bool
    zoomout: bool
    widein: bool                    # a.k.a. "FOV Plus"
    wideout: bool                   # a.k.a. "FOV Minus"
    focus_in: bool
    focus_out: bool
    laser_on_off: bool
    laser_cont_mode: bool
    laser_single_mode: bool
    laser_zoom_in: bool
    laser_zoom_out: bool
    tracking_search_on_off: bool
    ai_tracking_on_off: bool
    tracking_template_toggle: bool
    tracking_source_toggle: bool
    joystick_track: bool
    take_picture: bool
    start_record: bool
    stop_record: bool
    picture_record_mode_toggle: bool
    image_sensor_change: bool
    ir_polarity: bool
    ir_camera_dzoom_plus: bool
    ir_camera_dzoom_minus: bool
    near_infrared_toggle: bool
    eo_image_on_off: bool
    motor_on_off: bool
    video_ip: bool
    eo_dzoom_toggle: bool
    ir_rainbow: bool
    
class PayloadCommandDecoder(Decoder):
    TYPE = MessageType.PAYLOAD_COMMAND

    def decode(self, payload: bytes) -> PayloadCommand:
        value = struct.unpack('<I', payload)[0]
        return PayloadCommand(
            zoomin                      = bool((value >> PAYLOAD_ZOOM_IN_BIT) & 1),
            zoomout                     = bool((value >> PAYLOAD_ZOOM_OUT_BIT) & 1),
            widein                      = bool((value >> PAYLOAD_FOV_PLUS_BIT) & 1),
            wideout                     = bool((value >> PAYLOAD_FOV_MINUS_BIT) & 1),
            focus_in                    = bool((value >> PAYLOAD_FOCUS_IN_BIT) & 1),
            focus_out                   = bool((value >> PAYLOAD_FOCUS_OUT_BIT) & 1),
            laser_on_off                = bool((value >> PAYLOAD_LASER_ON_OFF_BIT) & 1),
            laser_cont_mode             = bool((value >> PAYLOAD_LASER_CONT_MODE_BIT) & 1),
            laser_single_mode           = bool((value >> PAYLOAD_LASER_SINGLE_MODE_BIT) & 1),
            laser_zoom_in               = bool((value >> PAYLOAD_LASER_ZOOM_IN_BIT) & 1),
            laser_zoom_out              = bool((value >> PAYLOAD_LASER_ZOOM_OUT_BIT) & 1),
            tracking_search_on_off      = bool((value >> PAYLOAD_TRACKING_SEARCH_ON_OFF_BIT) & 1),
            ai_tracking_on_off          = bool((value >> PAYLOAD_AI_TRACKING_ON_OFF_BIT) & 1),
            tracking_template_toggle    = bool((value >> PAYLOAD_TRACKING_TEMPLATE_TOGGLE_BIT) & 1),
            tracking_source_toggle      = bool((value >> PAYLOAD_TRACKING_SOURCE_TOGGLE_BIT) & 1),
            joystick_track              = bool((value >> PAYLOAD_JOYSTICK_TRACK_BIT) & 1),
            take_picture                = bool((value >> PAYLOAD_TAKE_PICTURE_BIT) & 1),
            start_record                = bool((value >> PAYLOAD_START_RECORD_BIT) & 1),
            stop_record                 = bool((value >> PAYLOAD_STOP_RECORD_BIT) & 1),
            picture_record_mode_toggle  = bool((value >> PAYLOAD_PIC_RECORD_MODE_TOGGLE_BIT) & 1),
            image_sensor_change         = bool((value >> PAYLOAD_IMAGE_SENSOR_CHANGE_BIT) & 1),
            ir_polarity                 = bool((value >> PAYLOAD_IR_POLARITY_BIT) & 1),
            ir_camera_dzoom_plus        = bool((value >> PAYLOAD_IR_DZOOM_PLUS_BIT) & 1),
            ir_camera_dzoom_minus       = bool((value >> PAYLOAD_IR_DZOOM_MINUS_BIT) & 1),
            near_infrared_toggle        = bool((value >> PAYLOAD_NEAR_IR_TOGGLE_BIT) & 1),
            eo_image_on_off             = bool((value >> PAYLOAD_EO_IMAGE_ON_OFF_BIT) & 1),
            motor_on_off                = bool((value >> PAYLOAD_MOTOR_ON_OFF_BIT) & 1),
            video_ip                    = bool((value >> PAYLOAD_VIDEO_IP_BIT) & 1),
            eo_dzoom_toggle             = bool((value >> PAYLOAD_EO_DZOOM_TOGGLE_BIT) & 1),
            ir_rainbow                  = bool((value >> PAYLOAD_IR_RAINBOW_BIT) & 1),
        )

    # No encode(): PAYLOAD_COMMAND is STM32 -> PC only, same as
    # BUTTON_STATE above — the PC never needs to build this frame.


register(PayloadCommandDecoder())