
from config import *


def system_Print(msg, condition=True):
    if DEBUG_MODE and condition:
        print(msg)

#TO BE REMOVED USELES IN A NON-DEBUG STATE
def print_HeartBeat(heartbeat):
    if DEBUG_MODE:
        print('=== Heartbeat contents ===')
        print('Vehicle type    : ' + str(heartbeat.type))
        print('Autopilot       : ' + str(heartbeat.autopilot))
        print('Base mode       : ' + str(heartbeat.base_mode))
        print('Custom mode     : ' + str(heartbeat.custom_mode))
        print('System status   : ' + str(heartbeat.system_status))
        print('MAVLink version : ' + str(heartbeat.mavlink_version))



