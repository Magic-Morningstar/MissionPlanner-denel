from pymavlink import mavutil
import time
class Guided_mode:

    def __init__(self):
        pass


    def initiate_GuidedMode(self):
        print('Setting GUIDED mode...')
        mode_id = self.drone.mode_mapping()['GUIDED']
        self.drone.mav.set_mode_send(
            self.drone.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        time.sleep(2)
