from Heart import Heart
from Flight_Modes.Arms import Arms
from Flight_Modes.Manual_mode import Manual_mode
from pymavlink import mavutil
import time
import sys


class Controller:

    def __init__(self):
        # Compose Heart — handles connection and heartbeat
        self.heart = Heart()
        self.drone = None

    # ─────────────────────────────────────────────────────────────────────────
    #  Startup — call this first before anything else
    # ─────────────────────────────────────────────────────────────────────────

    def start(self):
        """Connect, wait for heartbeat, and request data streams."""
        self.heart.initiate_Connection()
        self.heart.heartbeat()
        self.heart.request_data_stream()
        self.drone = self.heart.get_Drone()   # grab the live mavlink connection
        print('[Controller] Ready')


    # ─────────────────────────────────────────────────────────────────────────
    #  Flight modes
    # ─────────────────────────────────────────────────────────────────────────

    def set_mode(self, mode_name: str):
        """
        Switch to any ArduPilot flight mode by name.
        Examples: 'AUTO', 'GUIDED', 'QLOITER', 'QSTABILIZE', 'RTL', 'LAND'
        """
        mode_id = self.drone.mode_mapping().get(mode_name)
        if mode_id is None:
            print(f'[Controller] Unknown mode: {mode_name}')
            return

        self.drone.mav.set_mode_send(
            self.drone.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

        # Wait for the mode change to be confirmed via HEARTBEAT
        deadline = time.time() + 10
        while time.time() < deadline:
            hb = self.drone.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if hb and hb.custom_mode == mode_id:
                print(f'[Controller] Mode → {mode_name}')
                return

        print(f'[Controller] Mode change to {mode_name} not confirmed')

    def rtl(self):
        """Return to launch."""
        self.set_mode('RTL')

    def land(self):
        """Land in place immediately."""
        self.set_mode('LAND')

    def auto(self):
        """Start executing the uploaded mission."""
        self.set_mode('AUTO')

    def guided(self):
        """Switch to GUIDED — allows sending target positions via MAVLink."""
        self.set_mode('GUIDED')


    # ─────────────────────────────────────────────────────────────────────────
    #  Mission
    # ─────────────────────────────────────────────────────────────────────────

    def check_mission(self):
        """Return the number of mission items stored on the flight controller."""
        self.drone.mav.mission_request_list_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
        msg = self.drone.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
        count = msg.count if msg else 0
        print(f'[Controller] Mission has {count} items')
        return count

    def set_mission_start(self, item: int = 1):
        """Set which mission item to start from (default 1 skips home)."""
        self.drone.mav.mission_set_current_send(
            self.drone.target_system,
            self.drone.target_component,
            item,
        )
        print(f'[Controller] Mission start item → {item}')

    # ─────────────────────────────────────────────────────────────────────────
    #  Telemetry
    # ─────────────────────────────────────────────────────────────────────────

    def get_position(self):
        """Return the vehicle's current (lat, lon, alt_relative) or None."""
        msg = self.drone.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True, timeout=5
        )
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0   # mm → metres
            print(f'[Controller] Position: lat={lat:.6f}  lon={lon:.6f}  alt={alt:.1f} m')
            return lat, lon, alt
        print('[Controller] Could not get position')
        return None

    # ─────────────────────────────────────────────────────────────────────────
    #  Internal helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _wait_ack(self, command: int, timeout: float = 10.0) -> bool:
        """Block until COMMAND_ACK for the given command is received."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.drone.recv_match(
                type='COMMAND_ACK', blocking=True, timeout=1
            )
            if msg and msg.command == command:
                ok = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                mark = '✓' if ok else '✗'
                print(f'    {mark} ACK cmd={command}  result={msg.result}')
                return ok
        print(f'    ✗ ACK timeout  cmd={command}')
        return False
    

    def main(self):
        self.start
        while True:
            command = input("command: ")
            if command == "arm":
                Arms.initiate_Arm(self.drone)
            elif command == "rtl":
                self.rtl
            elif command == "manual":
               Manual_mode.initiate_Manual_Mode(self.drone)
            elif command == "emergency":
                pass
            elif command == "takeoff":
                pass
            
if __name__ == "__main__":
    Controller.main
 