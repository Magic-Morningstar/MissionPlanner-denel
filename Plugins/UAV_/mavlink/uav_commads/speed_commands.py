# mavlink/uav_commads/speed_commands.py

from pymavlink import mavutil
from mavlink.uav_commads.base_command import BaseCommand
import logging
logger = logging.getLogger(__name__)


class Speed_Controller(BaseCommand):

    def set_airspeed(self, speed_ms):
        if not self._check(
            self._requires_connection,
            self._requires_armed,
            lambda: self._requires_mode_in("FBWA", "FBWB"),   # was missing — only enforced by the caller before
        ):
            return False

        logger.info(f"Setting airspeed to {speed_ms} m/s")
        self._send_command(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_SPEED,
            p1=0,
            p2=speed_ms
        )
        return True

    def send_change_speed(self, speed_type=0, speed=-1, throttle_pct=5):
        if not self._check(
            self._requires_connection,
            self._requires_armed
        ):
            return False

        logger.info(f"DO_CHANGE_SPEED: type={speed_type} speed={speed}m/s throttle={throttle_pct}%")
        self._send_command(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            p1=speed_type,
            p2=speed,
            p3=throttle_pct
        )
        ack = self._wait_for_ack(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED)
        return ack
