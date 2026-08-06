# mavlink/uav_commads/arms_commands.py

from pymavlink import mavutil
from mavlink.base_command import BaseCommand
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
        self._send_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            p1=1
        )
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
