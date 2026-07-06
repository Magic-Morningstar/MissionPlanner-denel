# ============================================
# UAV SYSTEM BIT DEFINITIONS (32-bit state)
# ============================================

# ----------------------------
# FLIGHT CONTROL BITS
# ----------------------------
EMERGENCY_BIT     = 0
RTL_BIT           = 2
ARM_BIT           = 4
AUTOTAKEOFF_BIT   = 6
SYSTEM_CHECK      = 8
MANUAL_BIT        = 11


# ----------------------------
# FLIGHT STATE INDICATOR BITS
# ----------------------------

EMERGENCY_STATE_BIT     = 1
RTL_STATE_BIT           = 3
ARM_STATE_BIT           = 5
AUTOTAKEOFF_STATE_BIT   = 7
SYSTEM_UNHEALTH_BIT  = 9
DISARM_ARM        = 10
RF_BIT            = 12
LTE_BIT           = 13
IS_FLYING_BIT     = 15
