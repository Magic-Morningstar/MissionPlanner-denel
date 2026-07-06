from pymavlink import mavutil

class Manual_mode:

    def __init__(self,Drone):
        self.drone = Drone

    def initiate_Manual_Mode(self):
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param1: base mode
            0,          # param2: custom mode (0 = MANUAL for ArduPilot)
            0,          # param3-7: unused
            0, 0, 0, 0
        )  

    def is_in_manual_mode(self, timeout=3):
        """
        Check whether the vehicle is currently in MANUAL mode.
        Reads the next HEARTBEAT and decodes custom_mode using the
        vehicle's own mode_mapping (works across Plane/Copter/Rover).
        Returns True/False, or None if no heartbeat was received in time.
        """
        msg = self.drone.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
        if msg is None:
            print("No HEARTBEAT received - could not determine flight mode.")
            return None

        mode_mapping = self.drone.mode_mapping()
        if not mode_mapping:
            print("No mode mapping available for this vehicle/connection.")
            return None

        manual_mode_id = mode_mapping.get("MANUAL")
        if manual_mode_id is None:
            print("MANUAL mode not found in mode mapping for this vehicle type.")
            return None

        in_manual = msg.custom_mode == manual_mode_id
        print(f"Vehicle is {'IN' if in_manual else 'NOT in'} MANUAL mode")
        return in_manual