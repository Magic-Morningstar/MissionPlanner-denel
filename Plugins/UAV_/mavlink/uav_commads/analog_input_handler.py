# mavlink/uav_commads/analog_input_handler.py

import logging

logger = logging.getLogger(__name__)

DEAD_LOW  = int(0.45 * 4095)   # 1842 — 45% of ADC range
DEAD_HIGH = int(0.55 * 4095)   # 2252 — 55% of ADC range

# Gimbal angle limits — tune to the A20KTR's actual mechanical range.
GIMBAL_YAW_MAX_DEG   = 45.0    # +/- from forward
GIMBAL_PITCH_MIN_DEG = -90.0   # straight down
GIMBAL_PITCH_MAX_DEG = 45.0    # up from level

# Minimum angle change (deg) before a new point-angle command is sent,
# to avoid flooding the MAVLink link with near-identical setpoints.
GIMBAL_SEND_THRESHOLD_DEG = 1.0


class AnalogInputHandler:
    """
    Completely stateless with respect to MAVLink — it calls methods on
    UAVCommandSender (set_airspeed, turn_left, point_gimbal etc.) and
    never touches the drone connection directly.

    Owns:

      - Joystick2 X/Y → gimbal yaw/pitch (absolute angle, always active)
      - Joystick X/Y → heading/altitude in GUIDED mode
      - ManualController start/stop based on mode transitions
    """

    def __init__(self, state, manual_ctrl, sender):
        self.state       = state
        self.manual_ctrl = manual_ctrl   # ManualController instance
        self.sender      = sender        # UAVCommandSender for command dispatch

        self._last_turn_degrees   = 0.0
        self._last_altitude_delta = 0.0
        self._last_gimbal_pitch   = 0.0
        self._last_gimbal_yaw     = 0.0

    def process(self):
        current_mode = self.state.get_UAV_Current_Mode

        # Gimbal pointing is independent of flight mode — always active.
        self._handle_joystick2_gimbal()
        self._handle_joystick(current_mode)

    # ── Joystick2 X/Y → gimbal yaw/pitch ─────────────────────────────────────

    def _handle_joystick2_gimbal(self):
        payload_joy_x = self.state.get_Payload_Joystick_X
        payload_joy_y = self.state.get_Payload_Joystick_Y

        yaw_deg = self._joystick_to_gimbal_yaw_degrees(payload_joy_x)
        pitch_deg = self._joystick_to_gimbal_pitch_degrees(payload_joy_y)

        if (abs(yaw_deg - self._last_gimbal_yaw) < GIMBAL_SEND_THRESHOLD_DEG
                and abs(pitch_deg - self._last_gimbal_pitch) < GIMBAL_SEND_THRESHOLD_DEG):
            return

        logger.info(f"Joystick2 gimbal: pitch={pitch_deg:+.1f}deg, yaw={yaw_deg:+.1f}deg")
        self.sender.point_gimbal(pitch_deg, yaw_deg)
        self._last_gimbal_pitch = pitch_deg
        self._last_gimbal_yaw = yaw_deg

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

    def _joystick_to_gimbal_yaw_degrees(self, joystick_x):
        """Maps joystick2 X (0-4095) to gimbal yaw. Max +/-GIMBAL_YAW_MAX_DEG."""
        if DEAD_LOW <= joystick_x <= DEAD_HIGH:
            return 0.0
        if joystick_x < DEAD_LOW:
            return -((DEAD_LOW - joystick_x) / DEAD_LOW) * GIMBAL_YAW_MAX_DEG
        return +((joystick_x - DEAD_HIGH) / (4095 - DEAD_HIGH)) * GIMBAL_YAW_MAX_DEG

    def _joystick_to_gimbal_pitch_degrees(self, joystick_y):
        """
        Maps joystick2 Y (0-4095) to gimbal pitch. Below center tilts the
        gimbal down (toward GIMBAL_PITCH_MIN_DEG), above center tilts up
        (toward GIMBAL_PITCH_MAX_DEG).
        """
        if DEAD_LOW <= joystick_y <= DEAD_HIGH:
            return 0.0
        if joystick_y < DEAD_LOW:
            return -((DEAD_LOW - joystick_y) / DEAD_LOW) * abs(GIMBAL_PITCH_MIN_DEG)
        return +((joystick_y - DEAD_HIGH) / (4095 - DEAD_HIGH)) * GIMBAL_PITCH_MAX_DEG

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