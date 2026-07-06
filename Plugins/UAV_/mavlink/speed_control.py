from pymavlink import mavutil
import time
class Speed_Controller:

    def send_change_speed(master, speed_type=0, speed=-1, throttle_pct=5):
        """
        Send MAV_CMD_DO_CHANGE_SPEED.
        speed_type: 0 = Airspeed, 1 = Ground Speed, 2 = Climb Speed, 3 = Descent Speed
        speed: target speed in m/s (-1 = no change, -2 = return to default)
        throttle_pct: target throttle % (-1 = no change, -2 = return to default)

        """
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,            # confirmation
            speed_type,
            speed,
            throttle_pct,
            0, 0, 0, 0,   # unused params
        )
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        
        return ack