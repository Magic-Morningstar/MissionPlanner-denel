# mavlink/uav_commads/analog_input_handler.py

import logging

logger = logging.getLogger(__name__)

DEAD_LOW  = int(0.45 * 4095)   # 1842 — 45% of ADC range
DEAD_HIGH = int(0.55 * 4095)   # 2252 — 55% of ADC range


class AnalogInputHandler:
    """


    Completely stateless with respect to MAVLink — it calls methods on
    UAVCommandSender (set_airspeed, turn_left, set_altitude etc.) and
    never touches the drone connection directly.

    Owns:

      - Joystick2 Y → relative speed delta
      - Joystick X/Y → heading/altitude in GUIDED mode
      - ManualController start/stop based on mode transitions
    """

    def __init__(self, state, manual_ctrl, sender):
        self.state       = state
        self.manual_ctrl = manual_ctrl   # ManualController instance
        self.sender      = sender        # UAVCommandSender for command dispatch


        self._last_speed_delta    = 0.0
        self._last_turn_degrees   = 0.0
        self._last_altitude_delta = 0.0

    def process(self):
        current_mode = self.state.get_UAV_Current_Mode

        # Manual throttle (via ManualController) now owns joystick2 Y in
        # FBWA/FBWB — GUIDED_CHANGE_SPEED only applies in GUIDED, where
        # there's no manual_control stream to conflict with.
        self._handle_joystick2_speed(current_mode, ("GUIDED",))
        self._handle_joystick(current_mode)



    # ── Joystick2 Y → relative speed delta ───────────────────────────────────

    def _handle_joystick2_speed(self, mode, active_modes):
        if mode not in active_modes:
            return

        payload_joy_y = self.state.get_Payload_Joystick_Y
        speed_delta = self._joystick2_y_to_speed_delta(payload_joy_y)

        if speed_delta == 0:
            self._last_speed_delta = 0.0
            return

        if abs(speed_delta - self._last_speed_delta) <= 0.3:
            return

        current_speed = (
            self.state.get_UAV_Current_Air_Speed
            or self.state.get_UAV_Current_Ground_Speed
            or 0
        )
        min_speed = getattr(self.state, 'MIN_SPEED', 5.0)
        max_speed = getattr(self.state, 'MAX_SPEED', 25.0)
        new_target = max(min_speed, min(max_speed, current_speed + speed_delta))

        logger.info(
            f"Joystick2 Y: {speed_delta:+.1f} m/s delta, "
            f"{current_speed:.1f} -> {new_target:.1f} m/s"
        )
        self.sender.set_airspeed(new_target)
        self._last_speed_delta = speed_delta

    # ── Joystick X/Y → mode-dependent control ────────────────────────────────

    def _handle_joystick(self, mode):
        joystick_x = self.state.get_Joystick_X
        joystick_y = self.state.get_Joystick_Y

        if mode in ("FBWA", "FBWB"):
            # Start streaming if not already — ManualController reads
            # state directly at 10Hz so no values need passing here
            if self.manual_ctrl and not self.manual_ctrl._streaming:
                self.manual_ctrl.start_streaming()



    # ── Mapping helpers ───────────────────────────────────────────────────────


    def _joystick2_y_to_speed_delta(self, raw_adc):
        """
        Relative speed delta — above 55%: speed up, below 45%: slow down.
        Capped at ±5 m/s per nudge. Adjust MAX_DELTA to taste.
        """
        MAX_DELTA = 5.0
        if DEAD_LOW <= raw_adc <= DEAD_HIGH:
            return 0.0
        if raw_adc < DEAD_LOW:
            return -((DEAD_LOW - raw_adc) / DEAD_LOW) * MAX_DELTA
        return +((raw_adc - DEAD_HIGH) / (4095 - DEAD_HIGH)) * MAX_DELTA

    def _joystick_to_degrees(self, joystick_x):
        """Maps joystick X (0-4095) to heading change in degrees. Max ±45deg."""
        if DEAD_LOW <= joystick_x <= DEAD_HIGH:
            return 0
        if joystick_x < DEAD_LOW:
            return -((DEAD_LOW - joystick_x) / DEAD_LOW) * 45.0
        return +((joystick_x - DEAD_HIGH) / (4095 - DEAD_HIGH)) * 45.0

    def _joystick_to_altitude_delta(self, joystick_y):
        """Maps joystick Y (0-4095) to altitude delta in meters. Max ±20m."""
        if DEAD_LOW <= joystick_y <= DEAD_HIGH:
            return 0
        if joystick_y < DEAD_LOW:
            return -((DEAD_LOW - joystick_y) / DEAD_LOW) * 20.0
        return +((joystick_y - DEAD_HIGH) / (4095 - DEAD_HIGH)) * 20.0