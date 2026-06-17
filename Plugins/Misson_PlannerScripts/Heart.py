from pymavlink import mavutil
import time
import sys
import serial
from datetime import datetime
from Flight_Modes.Arms import Arms
from Flight_Modes.RTL import RTL
from Flight_Modes.Manual_mode import Manual_mode
from Services.Pre_Flight_Checks.flight_Checks import FlightChecks
from Services.lto_controller import Landing_Takeoff_controller
sys.path.append('Pre_Flight_Checks')

class Heart:

    def __init__(self):
        self.connection_details = 'udpin:0.0.0.0:14551'
        self.drone = None
        self.heartbeat_msg = None
        self.flying = True
        self.serial_port = "COM7"
        self.baudrate = 115200
        self.ser = None
        self.controllerCommands = 0
        self.connected = False



    def initiate_Connection(self):
        print('Connecting...')
        self.drone = mavutil.mavlink_connection(self.connection_details)

    def heartbeat(self):
        print('Waiting for heartbeat...')
        self.heartbeat_msg = self.drone.wait_heartbeat()
        print('Connected!')
        self.get_Heartbeat()

    def get_Heartbeat(self):
        print('=== Heartbeat contents ===')
        print('Vehicle type    : ' + str(self.heartbeat_msg.type))
        print('Autopilot       : ' + str(self.heartbeat_msg.autopilot))
        print('Base mode       : ' + str(self.heartbeat_msg.base_mode))
        print('Custom mode     : ' + str(self.heartbeat_msg.custom_mode))
        print('System status   : ' + str(self.heartbeat_msg.system_status))
        print('MAVLink version : ' + str(self.heartbeat_msg.mavlink_version))
        print('System ID       : ' + str(self.drone.target_system))
        print('Component ID    : ' + str(self.drone.target_component))

    def initiate_Modes(self):
        if self.drone:
            self.Arms = Arms(self.drone)
            self.RTL = RTL(self.drone)
            self.Manual = Manual_mode()
            self.ATO = Landing_Takeoff_controller(self.drone)
            self.FC = FlightChecks(self.drone)


    def request_data_stream(self):
        print('Requesting data streams...')
        self.drone.mav.request_data_stream_send(
            self.drone.target_system,
            self.drone.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1
        )
        time.sleep(2)
        print('  Data streams active')

    def get_Drone(self):
        return self.drone

    


    def connect_serial(self):
        """Try to connect to STM32 on COM7"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=1
            )
            self.connected = True
            print(f"Controller connected on {self.serial_port}")
            print("-" * 40)
            return True
        except serial.SerialException:
            self.connected = False
            return False

    def disconnect_serial(self):
        """Close serial port cleanly"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False
        print("Controller disconnected")
        print("-" * 40)

    def listen(self):
        current = None
        last_data_time = None
        TIMEOUT = 3  # seconds before assuming disconnected

        print(f"Waiting for controller on {self.serial_port}...")

        while True:
            # ── Try to connect if not connected ──
            if not self.connected:
                if self.connect_serial():
                    last_data_time = time.time()
                else:
                    print(f"Controller not found — retrying in 2 seconds...")
                    time.sleep(2)
                    continue

            # ── Check for timeout — no data received ──
            if last_data_time and (time.time() - last_data_time) > TIMEOUT:
                print("Controller stopped sending data — restarting connection...")
                self.disconnect_serial()
                continue

            # ── Read data ──
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    last_data_time = time.time()  # reset timer on data received
                    
                    if line.startswith("NUM:"):
                        num = int(line.split(":")[1])

                        if current != num:
                            current = num
                            timestamp = datetime.now().strftime("%H:%M:%S")
                            self.controllerCommands = num
                            #Getting the arm status
                            ARM_bit = (current >> 4) & 1  
                            EMERGENCY_bit =  (current >> 0) & 1  
                            RTL_bit =  (current >> 2) & 1 
                            AUTOTAKEOFF_bit =  (current >> 6) & 1       
                            MANUAL_bit =  (current >> 11) & 1        
                            SYSTEM_CHECK_bit =  (current >> 8) & 1             
                            if(ARM_bit==1):
                                self.Arms.initiate_Arm()
                            #if(EMERGENCY_bit==1):
                            #    self.Arms.initiate_Arm()
                            if(RTL_bit==1):
                                self.RTL.initiate_RTL()
                            if(MANUAL_bit==1):
                                self.Manual.initiate_Manual_Mode(self.drone)
                            if(SYSTEM_CHECK_bit==1):
                                self.FC.run_all_checks()
                            if(AUTOTAKEOFF_bit==1):
                                self.ATO.initiate_Takeoff(50)
                            print(f"[{timestamp}]")
                            print(f"  Decimal:     {current}")
                            print(f"  Hex:         0x{num:08X}")
                            print(f"  Binary:      {num:032b}")
                            print(f"  Max 32-bit:  {num} / 4294967295")
                            print("-" * 40)

            except serial.SerialException:
                print("Connection lost!")
                self.disconnect_serial()

            except KeyboardInterrupt:
                print("\nStopped by user")
                self.disconnect_serial()
                break

if __name__ == "__main__":
    heart = Heart()
    heart.initiate_Connection()
    heart.heartbeat()
    heart.get_Heartbeat()
    heart.request_data_stream()
    heart.initiate_Modes()
    heart.listen()
    