# mavlink/uav_commads/direction_commands.py

from pymavlink import mavutil
from mavlink.uav_commads.base_command import BaseCommand
import logging
logger = logging.getLogger(__name__)
import threading
import time


class DirectionCommander(BaseCommand):
    """Heading and altitude commands for GUIDED mode."""

    def turn_left(self, degrees=30):
        if not self._check(
            self._requires_connection,
            self._requires_armed,
            self._requires_flying,
            lambda: self._requires_mode("GUIDED")
        ):
            return False
        current_heading = self._get_current_heading()
        if current_heading is None:
            logger.warning("DirectionCommander: could not read heading.")
            return False
        target = (current_heading - degrees) % 360
        logger.info(f"Turn left {degrees}deg -> {target:.1f}deg")
        return self._send_heading(target)

    def turn_right(self, degrees=30):
        if not self._check(
            self._requires_connection,
            self._requires_armed,
            self._requires_flying,
            lambda: self._requires_mode("GUIDED")
        ):
            return False
        current_heading = self._get_current_heading()
        if current_heading is None:
            logger.warning("DirectionCommander: could not read heading.")
            return False
        target = (current_heading + degrees) % 360
        logger.info(f"Turn right {degrees}deg -> {target:.1f}deg")
        return self._send_heading(target)

    def set_heading(self, heading_deg):
        if not self._check(
            self._requires_connection,
            self._requires_armed,
            self._requires_flying,
            lambda: self._requires_mode("GUIDED")
        ):
            return False
        heading_deg = heading_deg % 360
        logger.info(f"Set heading to {heading_deg}deg")
        return self._send_heading(heading_deg)

    def _send_heading(self, heading_deg):
        self._send_command(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
            p1=0,
            p2=heading_deg,
            p3=0
        )
        return True

    def set_altitude(self, target_alt_m):
        if not self._check(
            self._requires_connection,
            self._requires_armed,
            self._requires_flying,
            lambda: self._requires_mode("GUIDED")
        ):
            return False
        target_alt_m = max(5.0, target_alt_m)
        logger.info(f"Set altitude to {target_alt_m}m")
        self._send_command(
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
            p1=target_alt_m,
            p2=0,
            p3=0
        )
        return True

    def _get_current_heading(self):
        msg = self.command_drone.messages.get('VFR_HUD')
        return float(msg.heading) if msg else None

    def _get_current_altitude(self):
        msg = self.command_drone.messages.get('GLOBAL_POSITION_INT')
        return msg.relative_alt / 1000.0 if msg else None


class ManualController(BaseCommand):
    """
    Sends MANUAL_CONTROL messages continuously at 10Hz for FBWA/FBWB.
    Runs on its OWN thread (not the shared command-queue thread) — which
    is exactly why it needs `self.lock`: this is the thread that was
    writing to command_drone.mav unsynchronized against every other
    command before this fix.
    """

    RATE_HZ  = 10
    DEAD_LOW  = int(0.45 * 4095)
    DEAD_HIGH = int(0.55 * 4095)

    def __init__(self, command_drone, state, lock):
        super().__init__(command_drone, state, lock)
        self._streaming  = False
        self._thread     = None
        self._throttle   = 500

    def start_streaming(self):
        if self._streaming:
            return
        self._streaming = True
        self._thread = threading.Thread(
            target=self._stream_loop,
            daemon=True,
            name="ManualControl"
        )
        self._thread.start()
        logger.info("ManualController: streaming started.")

    def stop_streaming(self):
        self._streaming = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2)
        logger.info("ManualController: streaming stopped.")

    def set_neutral(self):
        pass

    def _stream_loop(self):
        interval = 1.0 / self.RATE_HZ

        while self._streaming:
            joy_x = self.state.get_Joystick_X
            joy_y = self.state.get_Joystick_Y

            roll     = self._to_manual(joy_x)
            alt_rate = self._to_manual(joy_y)

            try:
                with self.lock:   # was unsynchronized — the core fix
                    self.command_drone.mav.manual_control_send(
                        self.command_drone.target_system,
                        roll,
                        alt_rate,
                        self._throttle,
                        0,
                        0
                    )
            except Exception as e:
                logger.error(f"ManualController: send error: {e}")
                self._streaming = False
                break

            time.sleep(interval)

        logger.info("ManualController: stream loop exited.")

    def _to_manual(self, raw_adc):
        if self.DEAD_LOW <= raw_adc <= self.DEAD_HIGH:
            return 0
        if raw_adc < self.DEAD_LOW:
            ratio = (self.DEAD_LOW - raw_adc) / self.DEAD_LOW
            return -int(ratio * 1000)
        else:
            ratio = (raw_adc - self.DEAD_HIGH) / (4095 - self.DEAD_HIGH)
            return +int(ratio * 1000)
