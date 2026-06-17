from pymavlink import mavutil

class Manual_mode:

    def __init__(self):
        pass

    def initiate_Manual_Mode(self,Drone):
        Drone.mav.command_long_send(
            Drone.target_system,
            Drone.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param1: base mode
            0,          # param2: custom mode (0 = MANUAL for ArduPilot)
            0,          # param3-7: unused
            0, 0, 0, 0
        )  
        
          