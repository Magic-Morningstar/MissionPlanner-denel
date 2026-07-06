from serial_controller.protocol.bit_definitions import *
class PacketBuilder:

    def __init__(self,state):
        self.state = state

    def build(self,message):

        # Convert to bytes for sending
        return f"NUM:{message}\n".encode()
    
    def serialize(self):

        value = 0

        if self.state.ARMED_SWITCH:
            if not self.state.armed_State:
                value |= (1 << ARM_STATE_BIT)


        if self.state.manual_State:
            value |= (1 << MANUAL_BIT)

        if self.state.is_flying:
            value |= (1 << IS_FLYING_BIT)
        
        if self.state.SYSTEM_CHECKED:
            if self.state.system_Check_Healthy:
                value |= (1 << SYSTEM_UNHEALTH_BIT)

        if self.state.is_ATO_InProgress:
            value |= (1 << AUTOTAKEOFF_STATE_BIT)


        if self.state.is_ATO_Successful:
            self.state.ATO = False
            value |= (1 << AUTOTAKEOFF_BIT)
        if self.state.RTL:
            if self.state.rtl_Returning_State:
                value |= (1 << RTL_STATE_BIT)

            if self.state.rtl_Landed_State:
                self.state.RTL = False

            

        if self.state.rf_State:
            value |= (1 << RF_BIT) 

        if self.state.lte_State:
            value |= (1 << LTE_BIT) 


        return value