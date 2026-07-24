from pymavlink import mavutil

# Common default — but confirm your Herelink's actual IP/port (see note below)
master = mavutil.mavlink_connection('udpin:192.168.43.1:14550')

master.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" %
      (master.target_system, master.target_component))