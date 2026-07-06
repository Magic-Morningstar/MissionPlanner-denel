
from utils.helper import *
from pymavlink import mavutil
from mavlink.speed_control import Speed_Controller
from mavlink.arms import Arms
from mavlink.ato import AutoTakeOff
from mavlink.rtl import RTL
from mavlink.Services.Pre_Flight_Checks.flight_Checks import FlightChecks
from mavlink.manual import Manual_mode
from config import *
class Mavlink_controller:

    def __init__(self,STATE):
        self.drone = None
        self.state = STATE
        self.heartbeat_MSG = None

    def initiate_Connection(self):
        system_Print("Connecting...")
        self.drone = mavutil.mavlink_connection(MAVLINK_CONNECTION,wait_ready=True)
        self.state.UAV = self.drone
        self.Manual = Manual_mode(self.drone)
        self.Arms = Arms(self.drone)
        self.rtl = RTL(self.drone)
        self.sc = FlightChecks(self.drone)
        self.ato = AutoTakeOff(self.drone)


    def heartbeat(self):
        system_Print('Waiting for heartbeat...')
        self.heartbeat_msg = self.drone.wait_heartbeat()
        system_Print("UAV connected successfully!",self.drone,)
        print_HeartBeat(self.heartbeat_msg)

    def getDrone(self):
        return self.drone
    
    

    def send_speed(self):
        return Speed_Controller.send_change_speed(self.drone,0,-1,self.state.SPEED)
    
    def update_State(self):
        self.state.ALTITUDE = self.ato.get_altitude()
        self.state.ARMED = self.Arms.is_armed()
        self.state.RTL =   self.rtl.is_in_rtl()
        self.state.ATO_Success = self.ato.takeoff_complete()
        self.state.SYSTEM_HEALTHY = self.sc.is_System_Good()
        self.state.MANUAL_MODE = self.Manual.is_in_manual_mode()

    def setManual_Mode(self):
        self.Manual.initiate_Manual_Mode()

    def start_AutoTakeOff(self):
        self.ato.initiate_takeoff_mode()
    
    def setArm_Mode(self):
        self.Arms.initiate_Arm()

    def initiate_RTL(self):
        self.rtl.initiate_RTL()

    
    def initiate_System_Checks(self):
        self.sc.run_all_checks()

    def execute_instructions(self,data):
        system_Print("Connection with misson planner lost please restart connection",(not self.state.UAV))
        if self.state.UAV:
            if data.get("arm") == 1:

                self.state.ARMED_SWITCH = True
                if not self.state.ARMED:
                    self.setArm_Mode()
            else:
                if not self.state.is_flying:
                    if self.state.ARMED:
                        self.Arms.initiate_Disarming()

            if data.get("rtl") == 1:
                system_Print("RTL is on")
                self.initiate_RTL()

            if data.get("manual") == 1:
                self.setManual_Mode()

            if data.get("takeoff") == 1:
                self.start_AutoTakeOff()



            if data.get("system_check") == 1:
                self.state.SYSTEM_CHECKED = True
                self.initiate_System_Checks()

            if data.get("emergancy") == 1:
                pass

            self.update_State()
        
        
        
        
      
        
