# serial_controller/protocol/bit_definitions.py
#
# Shared bit definitions between STM32 firmware and Python.
# This file is the single source of truth for all register layouts.

# ─────────────────────────────────────────────────────────────────────────────
# Incoming BUTTON_STATE register (message type 0x01) — flight/status only.
# Every payload concept (zoom, focus, laser, tracking, video, IR,
# image sensor) has its own bits in the PAYLOAD_COMMAND section further
# down — this section no longer defines bit positions for them at all,
# since ButtonState in messages.py doesn't decode them anymore. Old
# names like ZOOM_IN_BIT/WIDE_IN_BIT/LASER_ON_OFF_BIT etc. are gone
# entirely rather than just unused, so nothing can accidentally import
# them and reintroduce the collision this split was meant to fix.
# ─────────────────────────────────────────────────────────────────────────────

ARM_BIT                 = 0
ARM_STATUS_BIT          = 1

AUTO_BIT                = 2
AUTO_STATUS_BIT         = 3

MANUAL_BIT              = 4
MANUAL_STATUS_BIT       = 5

# Menu selection is NOT a packed binary value — it's six separate
# one-hot bits, one dedicated bit per menu position. The firmware sets
# exactly one of these based on Get_Menu_Index() (0-5) and clears the
# rest every loop iteration:
#
#   if (menu_idx & 0b000001) set BIT_MENU_SELECT_0 else clear it
#   if (menu_idx & 0b000010) set BIT_MENU_SELECT_1 else clear it
#   ... through BIT_MENU_SELECT_5
#
# Decoding this means checking which ONE of these six bits is set and
# reporting its index — NOT shifting/masking a contiguous 2-or-3-bit
# window the way an earlier version of this file assumed. That earlier
# assumption was actively wrong: reading bits 6-7 as a 2-bit binary pair
# only ever produces {0, 1, 2} — menu 0 decodes as 1, menu 1 decodes as
# 2, and menus 2 through 5 (which set bits 10-13, entirely outside that
# 2-bit window) all silently decode as 0, indistinguishable from each
# other or from genuinely being on menu 0.
#
# Bits 0/1 sit in the gap freed by the earlier payload split (used to be
# IR_POLARITY/IMAGE_SENSOR_CHANGE, long since moved to PAYLOAD_COMMAND).
# Bits 2-5 pick up at position 10, after SPEED_UP/SPEED_DOWN (8/9) —
# not contiguous with 0/1, which is exactly why treating this as one
# shiftable field was wrong in the first place.
MENU_SELECT_BIT_0       = 6
MENU_SELECT_BIT_1       = 7

SPEED_UP_BIT            = 8
SPEED_DOWN_BIT          = 9

MENU_SELECT_BIT_2       = 10
MENU_SELECT_BIT_3       = 11
MENU_SELECT_BIT_4       = 12
MENU_SELECT_BIT_5       = 13


STROBE_BIT              = 14

# In bit-index order — messages.py's decoder walks this to find which
# one is set, rather than hardcoding six separate checks.
MENU_SELECT_BITS = (
    MENU_SELECT_BIT_0, MENU_SELECT_BIT_1, MENU_SELECT_BIT_2,
    MENU_SELECT_BIT_3, MENU_SELECT_BIT_4, MENU_SELECT_BIT_5,
)


# ─────────────────────────────────────────────────────────────────────────────
# Potentiometer value
# Stored in bits 16–23 (one byte)
# ─────────────────────────────────────────────────────────────────────────────

POT_VALUE_SHIFT = 16
POT_VALUE_MASK  = 0xFF

# ─────────────────────────────────────────────────────────────────────────────
# Outgoing STATUS register (message type 0x10)
# These bits are sent FROM the PC TO the STM32
# ─────────────────────────────────────────────────────────────────────────────

STATUS_ARMED_BIT       = 0
STATUS_MANUAL_MODE_BIT = 1
AUTO_LAND_STATUS_BIT   = 7
STATUS_IS_FLYING_BIT   = 20

# ─────────────────────────────────────────────────────────────────────────────
# PAYLOAD_COMMAND register (message type 0x04), STM32 -> PC
#
# Own, independent bit numbering — deliberately NOT reusing names like
# ZOOM_IN_BIT from the BUTTON_STATE section above. This file is imported
# everywhere as `from bit_definitions import *`; a name collision here
# would silently redefine ZOOM_IN_BIT out from under ButtonStateDecoder
# rather than raising an error. Hence the PAYLOAD_ prefix on every name
# below, even where the underlying concept (zoom, focus...) also exists
# in BUTTON_STATE — they are two separate wire messages now.
#
# Verified bit-for-bit against the current main.c's BIT_* PAYLOAD_COMMAND
# defines (0-29) earlier in this project — every position below matches
# the real firmware exactly, not just eyeballed.
# ─────────────────────────────────────────────────────────────────────────────

PAYLOAD_ZOOM_IN_BIT                  = 0
PAYLOAD_ZOOM_OUT_BIT                 = 1
PAYLOAD_FOV_PLUS_BIT                 = 2   # a.k.a. "wide in"
PAYLOAD_FOV_MINUS_BIT                = 3   # a.k.a. "wide out"
PAYLOAD_FOCUS_IN_BIT                 = 4
PAYLOAD_FOCUS_OUT_BIT                = 5

PAYLOAD_LASER_ON_OFF_BIT             = 6   # matches main.c's BIT_LASER_ON_OFF
PAYLOAD_LASER_CONT_MODE_BIT          = 7
PAYLOAD_LASER_SINGLE_MODE_BIT        = 8
PAYLOAD_LASER_ZOOM_IN_BIT            = 9
PAYLOAD_LASER_ZOOM_OUT_BIT           = 10

PAYLOAD_TRACKING_SEARCH_ON_OFF_BIT   = 11
PAYLOAD_AI_TRACKING_ON_OFF_BIT       = 12
PAYLOAD_TRACKING_TEMPLATE_TOGGLE_BIT = 13
PAYLOAD_TRACKING_SOURCE_TOGGLE_BIT   = 14
PAYLOAD_JOYSTICK_TRACK_BIT           = 15

PAYLOAD_TAKE_PICTURE_BIT             = 16
PAYLOAD_START_RECORD_BIT             = 17
PAYLOAD_STOP_RECORD_BIT              = 18
PAYLOAD_PIC_RECORD_MODE_TOGGLE_BIT   = 19

PAYLOAD_IMAGE_SENSOR_CHANGE_BIT      = 20
PAYLOAD_IR_POLARITY_BIT              = 21
PAYLOAD_IR_DZOOM_PLUS_BIT            = 22
PAYLOAD_IR_DZOOM_MINUS_BIT           = 23
PAYLOAD_NEAR_IR_TOGGLE_BIT           = 24
PAYLOAD_EO_IMAGE_ON_OFF_BIT          = 25
PAYLOAD_MOTOR_ON_OFF_BIT             = 26
PAYLOAD_VIDEO_IP_BIT                 = 27
PAYLOAD_EO_DZOOM_TOGGLE_BIT          = 28
PAYLOAD_IR_RAINBOW_BIT               = 29
# 30, 31 reserved / spare