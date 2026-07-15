# serial_controller/protocol/bit_definitions.py
#
# Bit layout for the incoming BUTTON_STATE register (message type 0x01).
# This is the ONLY file allowed to know these numbers. Every other file —
# on the serial side AND the mavlink side — must go through
# serial_controller.protocol.messages.ButtonState and never touch a bit
# position directly.
#
# Values below match the existing config.py assignments so firmware
# doesn't need to change.

ARM_BIT             = 4
RTL_BIT             = 2
MANUAL_BIT          = 11
AUTOTAKEOFF_BIT     = 6
EMERGENCY_BIT       = 0
SYSTEM_CHECK_BIT    = 8

# Reserved for future discrete commands — just pick the next free bit
# and add a field to ButtonState in messages.py. Nothing else changes.
# e.g. SYSTEM_ARM_OVERRIDE_BIT = 9

# Pot value occupies its own byte, not a single bit
POT_VALUE_SHIFT = 16
POT_VALUE_MASK  = 0xFF


# ── Outgoing STATUS register (message type 0x10) ────────────────────────────
# Separate bit space from the incoming register above — these are PC-owned
# facts sent TO the STM32, not button state read FROM it. Kept in the same
# file since both belong to the same "shared wire vocabulary" concern.

STATUS_ARMED_BIT       = 0
STATUS_MANUAL_MODE_BIT = 1
STATUS_IS_FLYING_BIT   = 2
