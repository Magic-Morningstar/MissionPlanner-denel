
from pymavlink import mavutil

class SystemState:

    def __init__(self):

        #mavlink
        self.UAV = None

        #Properties
        self.ALTITUDE = 0
        self.SPEED = 0

        #states
        self.ARMED = False
        self.ARMED_SWITCH = False
        self.LTE_SIGNAL = True
        self.RF = True
        self.MANUAL_MODE = False
        self.RTL = False
        self.ATO = False
        self.ATO_Success = False
        self.SYSTEM_HEALTHY = False
        self.SYSTEM_CHECKED = False


        #serial
        self.CONVERSATION_STARTED = False
        self.SERIAL_CONNECTION = None
        self.SERIAL_CONNECTION_STATUS = False
        self.PREVIOUS_MESSAGE = 8


    @property
    def is_flying(self):
        return self.ALTITUDE > 1
    
    @property
    def flying_bit(self):
        if self.is_flying:
            return 1
        else:
            return 0

    @property
    def armed_State(self):
        return self.ARMED

    @property
    def rtl_Returning_State(self):
        return (self.RTL and (self.is_flying == True))
    
    @property
    def rtl_Landed_State(self):
        return self.RTL and ( self.is_flying == False)
    
    @property
    def is_ATO_Successful(self):
        return self.ATO_Success
    
    @property
    def is_ATO_InProgress(self):
        return self.ATO
    
    @property
    def system_Check_Healthy(self):
        return self.SYSTEM_HEALTHY
    
    @property
    def system_Check_Unealthy(self):
        return self.SYSTEM_HEALTHY == False

    @property
    def disArm_State(self):
        return (self.ARMED == False)

        
    @property
    def emergency_State(self):
        pass
     

    @property
    def manual_State(self):
        return self.MANUAL_MODE

    @property
    def rf_State(self):
        return self.RF

    @property
    def lte_State(self):
        return self.LTE_SIGNAL

    def UAV_connected(self):
        return self.UAV.wait_heartbeat()
    
    def update_serialConnection(self, is_Connected,ser):
        self.SERIAL_CONNECTION_STATUS = is_Connected
        self.SERIAL_CONNECTION = ser
        self.PREVIOUS_MESSAGE = 0
    
    def is_Serial_connected(self):
        return self.SERIAL_CONNECTION_STATUS
