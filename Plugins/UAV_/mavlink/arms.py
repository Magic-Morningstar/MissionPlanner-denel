
from pymavlink import mavutil
import time
import sys

class Arms:

    def __init__(self,DRONE):
            self.drone = DRONE

    def initiate_Arm(self):
        print('Arming...')
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0, 0, 0, 0, 0
        )
        self.drone.motors_armed_wait()
        print('  Armed!')

    def initiate_Disarming(self):
        print('Disarming...')
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,
            0, 0, 0, 0, 0
        )
        self.drone.motors_armed_wait()
        print('Disarmed!')

    def is_armed(self, timeout=3):
        """
        Return True if the vehicle is armed, False if disarmed, None if no
        heartbeat was received within timeout.
        """
        msg = self.drone.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
        if msg is None:
            print("No HEARTBEAT received - could not determine arm state.")
            return None
        
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"Vehicle is {'ARMED' if armed else 'DISARMED'}")
        return armed



    
    