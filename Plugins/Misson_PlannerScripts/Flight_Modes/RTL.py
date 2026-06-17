from pymavlink import mavutil
import time
import sys


class RTL:
    def __init__(self,Drone):
        self.drone = Drone

    def initiate_RTL(self):
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
