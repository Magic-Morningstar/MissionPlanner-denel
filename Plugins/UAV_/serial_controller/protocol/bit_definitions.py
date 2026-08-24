# serial_controller/protocol/bit_definitions.py
#
# Shared bit definitions between STM32 firmware and Python.
# This file is the single source of truth for all register layouts.

# ─────────────────────────────────────────────────────────────────────────────
# Incoming BUTTON_STATE register (message type 0x01)
# ─────────────────────────────────────────────────────────────────────────────

ARM_BIT                 = 0
ARM_STATUS_BIT          = 1

AUTO_BIT                = 2
AUTO_STATUS_BIT         = 3

MANUAL_BIT              = 4
MANUAL_STATUS_BIT       = 5

# FIXED: these were previously (wrongly) AUTO_LAND_BIT/AUTO_LAND_STATUS_BIT
# for the INCOMING register. main.c confirms bits 6/7 are actually
# BIT_IR_POLARITY ("WHITE BTN") and BIT_IMAGE_SENSOR_CHANGE ("GREEN
# BTN") — completely unrelated to autoland. With the old assumption,
# pressing the White (IR polarity) button would ALSO have fired
# LandCommand, since `autoland` decoded from this same bit. There is no
# dedicated autoland bit anywhere in the current main.c incoming bit
# list — see messages.py for how `autoland` is now handled (hardcoded
# False, not read from a bit that belongs to something else).
IR_POLARITY_BIT          = 6   # "WHITE BTN"
IMAGE_SENSOR_CHANGE_BIT   = 7   # "GREEN BTN"

# NOTE: AUTO_LAND_STATUS_BIT below is for the OUTGOING status register
# (PC -> STM32, message type 0x10) — a completely separate 32-bit value
# from the incoming BUTTON_STATE register above. It coincidentally also
# equals 7, but that's unrelated to IMAGE_SENSOR_CHANGE_BIT; kept as-is
# so StatusUpdateDecoder (further down / in messages.py) keeps working
# unchanged. Not touching the outgoing side's bit-numbering confusion
# (it separately also has STATUS_ARMED_BIT etc. that aren't consistently
# used) since that's a different, unrequested problem.
AUTO_LAND_STATUS_BIT     = 7

SPEED_UP_BIT            = 8
SPEED_DOWN_BIT          = 9

ZOOM_IN_BIT             = 10
ZOOM_OUT_BIT            = 11

WIDE_IN_BIT             = 12
WIDE_OUT_BIT          = 13


# ─────────────────────────────────────────────────────────────────────────────
# Object tracking — matches main.c's BIT_TRCKING_START_STOP (26). Firmware
# also defines BIT_FOCUS_IN(14)/BIT_FOCUS_OUT(15), BIT_VIDEO_IP(21),
# BIT_LASER_ON_OFF(22)/BIT_LASER_CONT_MODE(24)/BIT_LASER_SINGLE_MODE(25),
# BIT_AI_TRACKING_ON_OFF(27), and BIT_JOYSTICK_TRACK(28) — all now wired
# up below too.
# ─────────────────────────────────────────────────────────────────────────────

TRACKING_BIT            = 26
FOCUS_IN_BIT             = 14
FOCUS_OUT_BIT            = 15
VIDEO_IP_BIT             = 21
LASER_ON_OFF_BIT         = 22
LASER_CONT_MODE_BIT      = 24
LASER_SINGLE_MODE_BIT    = 25
AI_TRACKING_ON_OFF_BIT   = 27
JOYSTICK_TRACK_BIT       = 28



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