# mavlink/uav_commads/strobe_commands.py

from pymavlink import mavutil
from mavlink.uav_commads.commands.base_command import BaseCommand
import logging
logger = logging.getLogger(__name__)


class Strobe_Controller(BaseCommand):


    STROBE_SERVO_CHANNEL = 7
    STROBE_PWM_ON = 1900    
    STROBE_PWM_OFF = 1100   

    def strobe_on(self):
        if not self._check(
            self._requires_connection,
        ):
            return False

        logger.info(f"Strobe ON — channel {self.STROBE_SERVO_CHANNEL}, PWM {self.STROBE_PWM_ON}")
        self._send_command(
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            p1=self.STROBE_SERVO_CHANNEL,
            p2=self.STROBE_PWM_ON
        )
        return True

    def strobe_off(self):
        if not self._check(
            self._requires_connection,
        ):
            return False

        logger.info(f"Strobe OFF — channel {self.STROBE_SERVO_CHANNEL}, PWM {self.STROBE_PWM_OFF}")
        self._send_command(
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            p1=self.STROBE_SERVO_CHANNEL,
            p2=self.STROBE_PWM_OFF
        )
        return True