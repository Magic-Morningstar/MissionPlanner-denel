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

AUTO_LAND_BIT           = 6  
AUTO_LAND_STATUS_BIT    = 7

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