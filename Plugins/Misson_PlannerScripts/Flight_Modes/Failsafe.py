from pymavlink import mavutil
import time

class Failsafe:

    def __init__(self):
        pass

    def initiate_Failsafe(self):
        # Simulate RC failsafe by sending invalid throttle
        self.drone.mav.rc_channels_override_send(
            self.drone.target_system,
            self.drone.target_component,
            0, 0,
            900,    # throttle below failsafe threshold
            0, 0, 0, 0, 0
        )
 