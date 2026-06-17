"""
Controller simulator — keyboard interface to test MAVLink commands
without the physical STM32 controller.

Run from Misson_PlannerScripts\ directory:
    python test_simulator.py

Requires a MAVProxy output on port 14552:
    In MAVProxy console: output add 127.0.0.1:14552
"""

from pymavlink import mavutil
from Flight_Modes.Arms import Arms
from Flight_Modes.RTL import RTL
from Flight_Modes.Manual_mode import Manual_mode
from Services.Pre_Flight_Checks.flight_Checks import FlightChecks
from Services.lto_controller import Landing_Takeoff_controller

MENU = """
==============================
  CONTROLLER SIMULATOR
==============================
  a  -  Arm motors
  d  -  Disarm motors
  r  -  RTL (Return to Launch)
  m  -  Manual mode
  t  -  Auto Takeoff (50m)
  c  -  Run pre-flight checks
  q  -  Quit
==============================
"""

def main():
    print("Connecting to MAVLink on udpin:0.0.0.0:14552 ...")
    drone = mavutil.mavlink_connection('udpin:0.0.0.0:14552')
    print("Waiting for heartbeat...")
    drone.wait_heartbeat()
    print(f"Connected!  System {drone.target_system} / Component {drone.target_component}\n")

    # Request data streams so telemetry flows
    drone.mav.request_data_stream_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10, 1
    )

    arms  = Arms(drone)
    rtl   = RTL(drone)
    fc    = FlightChecks(drone)
    ato   = Landing_Takeoff_controller(drone)

    print(MENU)

    while True:
        try:
            cmd = input("Command: ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            print("\nExiting simulator.")
            break

        if cmd == 'a':
            arms.initiate_Arm()
        elif cmd == 'd':
            arms.initiate_Disarming()
        elif cmd == 'r':
            rtl.initiate_RTL()
        elif cmd == 'm':
            Manual_mode.initiate_Manual_Mode(drone)
        elif cmd == 't':
            ato.initiate_Takeoff(50)
        elif cmd == 'c':
            fc.run_all_checks()
        elif cmd == 'q':
            print("Exiting simulator.")
            break
        elif cmd == '':
            pass
        else:
            print(f"  Unknown command '{cmd}'. Use a/d/r/m/t/c/q")

if __name__ == "__main__":
    main()
