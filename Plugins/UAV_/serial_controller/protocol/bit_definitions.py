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