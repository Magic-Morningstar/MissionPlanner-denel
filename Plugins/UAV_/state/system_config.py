# config.py

SERIAL_PORT = "COM7"
BAUDRATE = 115200
SERIAL_TIMEOUT = 1
CONNECTION_CHECK_TIMEOUT = 3

# 14551, not 14550. Mission Planner's AutoConnect binds 14550 at startup and
# never releases it, and both it and pymavlink set SO_REUSEADDR -- so on Windows
# a second bind SUCCEEDS while datagrams reach only one socket, arbitrarily. The
# loser receives nothing, with no error anywhere. 14551 is reserved for this
# bridge: ExtLibs/Utilities/AutoConnect.cs disables Mission Planner's own 14551
# entry specifically to keep it free (see the comment at AutoConnect.cs:28-33).
#
# The MQTT bridge's GCS-side fan-out also targets 14551, so with this setting the
# panel receives telemetry whether the aircraft is on Herelink or on the MQTT
# backup link. See mqtt_bridge/local_hub.py.
#
# NOTE: both workers currently name the same port. Only UAVCommandSender connects
# (Mavlink_controller.connect() has the state poller commented out), so there is
# one bind today. Re-enabling UAVStatePoller needs a DIFFERENT port here, or the
# two will double-bind and split the datagrams between them -- the same silent
# failure this change exists to avoid. Avoid 14552-14559 too: _connect_with_retry
# walks that range.
MAVLINK_STATE_PORT = 'udpin:0.0.0.0:14551'

MAVLINK_COMMAND_PORT = 'udpin:0.0.0.0:14551'
HEARTBEAT_TIMEOUT = 5

START_BYTE = 0xAA
END_BYTE = 0x55
PACKET_SIZE = 6

MAIN_LOOP_DELAY = 0.01

DEFAULT_TAKEOFF_ALTITUDE = 50
SAFE_BATTERY_LEVEL = 20

ARM_BIT = 4
RTL_BIT = 2
MANUAL_BIT = 11
TAKEOFF_BIT = 6
EMERGENCY_BIT = 0
SYSTEM_CHECK_BIT = 8

ENABLE_PREFLIGHT_CHECKS = True
ENABLE_FAILSAFE = True
ENABLE_ARMING_CHECKS = True

STM32_RECONNECT_DELAY = 2
STM32_TIMEOUT = 3

DEBUG_MODE = True
PRINT_RAW_PACKETS = True
PRINT_SYSTEM_STATE = True
