from pymavlink import mavutil    

class Utils:

    def confirm_Mode_Change(self,drone):
        msg = drone.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg:
            current_mode = mavutil.mode_string_v10(msg)
            print('  Mode confirmed: ' + str(current_mode))


    def get_Current_Mode(self,drone):
        hb = drone.recv_match(type='HEARTBEAT', blocking=True, timeout=3)

        if hb:
            mode = hb.base_mode
            print(f"Raw base_mode: {mode}")

            # Check each flag individually using bitwise AND
            if mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
                print("Manual input: ON")
            if mode & mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED:
                print("Stabilize: ON")
            if mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED:
                print("Guided: ON")
            if mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED:
                print("Auto: ON")
