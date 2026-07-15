# mavlink/uav_commads/mode_commands.py

from pymavlink import mavutil
from mavlink.uav_commads.base_command import BaseCommand
import logging
logger = logging.getLogger(__name__)


class ModeCommander(BaseCommand):

    def initiate_FBWA_Mode(self):
        if not self._check(self._requires_connection):
            return False
        return self._set_mode("FBWA")

    def initiate_Guided_Mode(self):
        if not self._check(
            self._requires_connection,
            self._requires_armed
        ):
            return False
        return self._set_mode("GUIDED")

    def initiate_Auto_Mode(self):
        if not self._check(
            self._requires_connection,
            self._requires_armed
        ):
            return False
        return self._set_mode("AUTO")

    def initiate_QLOITER_Mode(self):
        if not self._check(
            self._requires_connection,
            self._requires_armed
        ):
            return False
        return self._set_mode("QLOITER")

    def initiate_RTL(self):
        if not self._check(
            self._requires_connection,
            self._requires_armed
        ):
            return False
        logger.info("Initiating RTL...")
        self._send_command(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
        return True
