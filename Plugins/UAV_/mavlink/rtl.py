from pymavlink import mavutil
import time
import sys
from utils.helper import *


class RTL:
    def __init__(self, Drone):
        self.drone = Drone

    def initiate_RTL(self):
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

    def get_current_mode(self, wait_time=2):
        """
        Listen for a HEARTBEAT and decode the current flight mode name.
        Returns the mode string (e.g. 'RTL', 'AUTO', 'MANUAL', 'QLOITER', etc.)
        """
        start = time.time()
        while time.time() - start < wait_time:
            msg = self.drone.recv_match(type="HEARTBEAT", blocking=True, timeout=wait_time)
            if msg is None:
                continue
            mode_mapping = self.drone.mode_mapping()
            if mode_mapping is None:
                return None
            for name, number in mode_mapping.items():
                if number == msg.custom_mode:
                    return name
        return None

    def is_in_rtl(self):
        """Return True if the vehicle's current mode is RTL (or QRTL for VTOL)."""
        mode = self.get_current_mode()

        system_Print("Could not determine current mode (no heartbeat received).", mode is None)
        if mode is None:
            return False, None

        system_Print(f"Current mode: {mode}")
        return mode in ("RTL", "QRTL"), mode