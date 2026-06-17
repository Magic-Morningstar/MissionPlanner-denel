from pymavlink import mavutil
import time
class Landing_Takeoff_controller:

    def __init__(self,DRONE):
        self.drone = DRONE


    def initiate_Takeoff(self, alt=15):
        # Step 1: Switch to GUIDED mode first (RTL and other modes block arming)
        print('Switching to GUIDED mode...')
        mode_id = self.drone.mode_mapping().get('GUIDED')
        if mode_id is None:
            print('  ERROR: GUIDED mode not available on this vehicle')
            return
        self.drone.mav.set_mode_send(
            self.drone.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        time.sleep(1)
        print('  GUIDED mode set')

        # Step 2: Arm
        print('Arming...')
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        self.drone.motors_armed_wait()
        print('  Armed!')

        # Step 3: Send takeoff command
        print('Taking off to ' + str(alt) + 'm...')
        self.drone.mav.command_long_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt
        )

        # Step 4: Wait until target altitude reached
        while True:
            msg = self.drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                current_alt = msg.relative_alt / 1000.0
                print('  Climbing: ' + str(round(current_alt, 2)) + 'm')
                if current_alt >= alt - 1:
                    break
            time.sleep(0.1)

        print('  At altitude!')



    def initiate_Landing(self):
        print('Landing...')
        mode_id = self.drone.mode_mapping()['LAND']
        self.drone.mav.set_mode_send(
            self.drone.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

        print('Waiting to land...')
        while True:
            msg = self.drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                alt = msg.relative_alt / 1000.0
                print('  Alt: ' + str(round(alt, 1)) + 'm')
                if alt <= 0.3:
                    break
            #time.sleep(0.5)

        print('Landed successfully!')



    def send_velocity(self, vx, vy, vz, duration):
        print('Moving vx=' + str(vx) + ' vy=' + str(vy) + ' vz=' + str(vz))
        end = time.time() + duration
        while time.time() < end:
            self.drone.mav.set_position_target_local_ned_send(
                0,
                self.drone.target_system,
                self.drone.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111,
                0, 0, 0,
                vx, vy, vz,
                0, 0, 0,
                0, 0
            )
            time.sleep(0.1)
  