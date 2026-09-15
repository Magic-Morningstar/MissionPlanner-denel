# mavlink/uav_commads/arms_commands.py

from pymavlink import mavutil
from mavlink.uav_commads.commands.base_command import BaseCommand
import logging
logger = logging.getLogger(__name__)


class Arms(BaseCommand):

    def initiate_Arm(self):
        if not self._check(
            self._requires_connection,
            self._requires_disarmed,
            self._requires_not_flying
        ):
            return False
        
        logger.info("Arming...")
        '''
        self._send_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            p1=1
        )'''
        STROBE_SERVO_CHANNEL = 7
        STROBE_PWM_ON = 1900 

        self.command_drone.mav.command_long_send(
            self.command_drone.target_system,
            self.command_drone.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            STROBE_SERVO_CHANNEL,
            STROBE_PWM_ON,
            0, 0, 0, 0, 0
        )
        logger.info("Strobe ON")
        self.state.update_UAV_Armed_Status(True)
        logger.info("Arm command sent.")
        return True   # was `return False` — reported failure on success

    def initiate_Disarming(self):
        if not self._check(
            self._requires_connection,
            self._requires_armed,
            self._requires_not_flying
        ):
            return False

        logger.info("Disarming...")
        self._send_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            p1=0
        )
        self.state.update_UAV_Armed_Status(False)
        logger.info("Disarm command sent.")
        return True
