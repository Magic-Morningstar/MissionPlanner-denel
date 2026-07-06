from serial_controller.protocol.bit_definitions import *
from utils.helper import *
class PacketParser:

    @staticmethod
    def parse(line):
        if not line:
            return None

        if line.startswith("NUM:"):
            value = int(line.split(":")[1])
            #PacketParser.print_USBMessage(value)
            return {
                "raw": value,
                "arm": (value >> ARM_BIT) & 1,
                "rtl": (value >> RTL_BIT) & 1,
                "manual": (value >> MANUAL_BIT) & 1,
                "takeoff": (value >> AUTOTAKEOFF_BIT) & 1,
                "emergancy": (value >> EMERGENCY_BIT) & 1,
                "system_check": (value >> SYSTEM_CHECK) & 1,
                "radio_frequency": (value >> RF_BIT) & 1,
                "LTE": (value >> LTE_BIT) & 1,
                "flying_Indicator": (value >> IS_FLYING_BIT) & 1,
            }

        return None
    @staticmethod
    def print_USBMessage(value):
        system_Print("-"*40)
        system_Print(f"arm: {value >> ARM_BIT & 1}")
        system_Print(f"arm_status: {value >> ARM_STATE_BIT & 1}")
        system_Print(f"disarm: {value >> DISARM_ARM & 1}")
        system_Print(f"rtl: {value >> RTL_BIT & 1}")
        system_Print(f"rtl_status: {value >> RTL_STATE_BIT & 1}")
        system_Print(f"manual: {value >> MANUAL_BIT & 1}")
        system_Print(f"takeoff: {value >> AUTOTAKEOFF_BIT & 1}")
        system_Print(f"takeoff_status: {value >> AUTOTAKEOFF_STATE_BIT & 1}")
        system_Print(f"emergency: {value >> EMERGENCY_BIT & 1}")
        system_Print(f"emergency_status: {value >> EMERGENCY_STATE_BIT & 1}")
        system_Print(f"system_check: {value >> SYSTEM_CHECK & 1}")
        system_Print(f"system_check_Unhealthy: {value >> SYSTEM_UNHEALTH_BIT & 1}")
        system_Print(f"radio_frequency: {value >> RF_BIT & 1}")
        system_Print(f"LTE: {value >> LTE_BIT & 1}")
        system_Print(f"flying_Indicator: {value >> IS_FLYING_BIT & 1}")
        system_Print("-"*40)