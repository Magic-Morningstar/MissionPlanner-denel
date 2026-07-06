# ============================================
# SERIAL CONFIGURATION
# ============================================

SERIAL_PORT = "COM7"
BAUDRATE = 115200
SERIAL_TIMEOUT = 1
CONNECTION_CHECK_TIMEOUT = 3 # seconds before assuming disconnected



# ============================================
# MAVLINK CONFIGURATION
# ============================================

MAVLINK_CONNECTION = 'tcp:127.0.0.1:5763'
HEARTBEAT_TIMEOUT = 5


# ============================================
# PACKET CONFIGURATION
# ============================================

START_BYTE = 0xAA
END_BYTE = 0x55
PACKET_SIZE = 6


# ============================================
# SYSTEM LOOP CONFIGURATION
# ============================================

MAIN_LOOP_DELAY = 0.01   # 100Hz


# ============================================
# UAV CONFIGURATION
# ============================================

DEFAULT_TAKEOFF_ALTITUDE = 50

SAFE_BATTERY_LEVEL = 20


# ============================================
# BIT DEFINITIONS
# ============================================

ARM_BIT = 4

RTL_BIT = 2

MANUAL_BIT = 11

TAKEOFF_BIT = 6

EMERGENCY_BIT = 0

SYSTEM_CHECK_BIT = 8




# ============================================
# SAFETY CONFIGURATION
# ============================================

ENABLE_PREFLIGHT_CHECKS = True

ENABLE_FAILSAFE = True

ENABLE_ARMING_CHECKS = True


# ============================================
# CONTROLLER CONNECTION SETTINGS
# ============================================

CONTROLLER_USB_VID = 0x0483  # STM32 VID — update if switching hardware
CONTROLLER_USB_PID = 0x5740  # STM32 PID — update if switching hardware

STM32_RECONNECT_DELAY = 2

STM32_TIMEOUT = 3


# ============================================
# GCS NOTIFICATION SETTINGS
# ============================================

GCS_NOTIFICATION_PORT = 5764


# ============================================
# DEBUG SETTINGS
# ============================================

DEBUG_MODE = True

PRINT_RAW_PACKETS = True

PRINT_SYSTEM_STATE = True