from pymavlink import mavutil
from utils.helper import *
import time


class AutoTakeOff:

    def __init__(self, drone):
        self.drone = drone

    def initiate_takeoff_mode(self):
        mode_mapping = self.drone.mode_mapping()
        if "TAKEOFF" not in mode_mapping:
            system_Print("TAKEOFF mode not available — check firmware/build type.")
            return False

        self.drone.mav.set_mode_send(
            self.drone.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_mapping["TAKEOFF"]
        )
        system_Print("Switched to TAKEOFF mode — climbing to TKOFF_ALT.")
        return True

    def get_altitude(self):
        """Reads back current altitude — purely for monitoring, doesn't drive anything."""
        msg = self.drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg is None:
            return None
        return msg.relative_alt / 1000.0
    
    def get_current_mission_item(self):
        msg = self.drone.recv_match(type='MISSION_CURRENT', blocking=True, timeout=1)
        if msg is None:
            return None
        return msg.seq

    def takeoff_complete(self):
        """True once the autopilot has moved past mission item 0 (the takeoff item)."""
        current_item = self.get_current_mission_item()
        return current_item is not None and current_item > 0