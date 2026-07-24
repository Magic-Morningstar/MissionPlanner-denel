# mavlink/uav_commads/analog_input_handler.py

import logging
import time

logger = logging.getLogger(__name__)

DEAD_LOW  = int(0.45 * 4095)   # 1842 — 45% of ADC range
DEAD_HIGH = int(0.55 * 4095)   # 2252 — 55% of ADC range

# Gimbal angle limits — tune to the A20KTR's actual mechanical range.
GIMBAL_YAW_MAX_DEG   = 45.0    # +/- from forward
GIMBAL_PITCH_MIN_DEG = -90.0   # straight down
GIMBAL_PITCH_MAX_DEG = 45.0    # up from level

# Rate control: at full joystick deflection, the commanded angle changes
# by this many degrees per second. Center stick = 0 deg/s (no movement).
# This is what "how many degrees the joystick represents at a time" tunes —
# raise it for a snappier stick, lower it for finer control.
GIMBAL_YAW_DEG_PER_SEC   = 30.0
GIMBAL_PITCH_DEG_PER_SEC = 30.0

# Minimum change (deg) between the last angle actually SENT to the gimbal
# and the current accumulated angle before a new point-angle command is
# sent, to avoid flooding the MAVLink link while the stick is held at a
# small deflection.
GIMBAL_SEND_THRESHOLD_DEG = 1.0


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


class AnalogInputHandler:
    """
    Completely stateless with respect to MAVLink — it calls methods on
    UAVCommandSender (set_airspeed, turn_left, point_gimbal etc.) and
    never touches the drone connection directly.

    Owns:

      - Joystick2 X/Y → gimbal yaw/pitch RATE control. Deflection sets a
        deg/sec rate; each process() tick adds (rate * dt) to a locally
        tracked commanded angle, clamped to the gimbal's limits, and sends
        an updated absolute point-angle command when it's moved enough to
        be worth another MAVLink message. Centering the stick stops motion
        at whatever angle was last reached — it does NOT re-center to 0.
      - Joystick X/Y → heading/altitude in GUIDED mode
      - ManualController start/stop based on mode transitions
    """

    def __init__(self, state, manual_ctrl, sender):
        self.state       = state
        self.manual_ctrl = manual_ctrl   # ManualController instance
        self.sender      = sender        # UAVCommandSender for command dispatch

        self._last_turn_degrees   = 0.0
        self._last_altitude_delta = 0.0

        # Locally tracked commanded gimbal angle (what we believe we've
        # asked the gimbal to point at). Starts at 0/0, matching the
        # gimbal's home position — see reset_gimbal_tracking() below for
        # why this can drift and when to reset it.
        self._current_gimbal_pitch = 0.0
        self._current_gimbal_yaw   = 0.0

        # Last angle actually SENT as a MAVLink point-angle command —
        # separate from _current_gimbal_* so we can rate-limit sends
        # without losing accumulated position between sends.
        self._last_sent_gimbal_pitch = 0.0
        self._last_sent_gimbal_yaw   = 0.0

        self._last_gimbal_tick_time = None

    def process(self):
        current_mode = self.state.get_UAV_Current_Mode

        # Gimbal pointing is independent of flight mode — always active.
        self._handle_joystick2_gimbal()
        self._handle_joystick(current_mode)

    def reset_gimbal_tracking(self, pitch_deg=0.0, yaw_deg=0.0):
        """
        Re-sync the locally tracked commanded angle to a known value.
        Call this after sending a Home/Retract/Neutral command (raw or
        MAVLink), or after any external gimbal telemetry confirms an
        actual position — otherwise this handler's notion of "current
        angle" can silently drift from where the gimbal really is, since
        there's no feedback loop here, only accumulation.
        """
        self._current_gimbal_pitch = pitch_deg
        self._current_gimbal_yaw = yaw_deg
        self._last_sent_gimbal_pitch = pitch_deg
        self._last_sent_gimbal_yaw = yaw_deg

    # ── Joystick2 X/Y → gimbal yaw/pitch (rate control) ──────────────────────

    def _handle_joystick2_gimbal(self):
        now = time.monotonic()
        if self._last_gimbal_tick_time is None:
            # First tick — nothing to integrate yet, just establish dt baseline.
            self._last_gimbal_tick_time = now
            return
        dt = now - self._last_gimbal_tick_time
        self._last_gimbal_tick_time = now

        payload_joy_x = self.state.get_Payload_Joystick_X
        payload_joy_y = self.state.get_Payload_Joystick_Y

        yaw_rate_deg_s = self._joystick_to_gimbal_yaw_rate(payload_joy_x)
        pitch_rate_deg_s = self._joystick_to_gimbal_pitch_rate(payload_joy_y)

        if yaw_rate_deg_s == 0.0 and pitch_rate_deg_s == 0.0:
            return  # stick centered — hold current angle, nothing to add

        self._current_gimbal_yaw = _clamp(
            self._current_gimbal_yaw + yaw_rate_deg_s * dt,
            -GIMBAL_YAW_MAX_DEG, GIMBAL_YAW_MAX_DEG
        )
        self._current_gimbal_pitch = _clamp(
            self._current_gimbal_pitch + pitch_rate_deg_s * dt,
            GIMBAL_PITCH_MIN_DEG, GIMBAL_PITCH_MAX_DEG
        )

        yaw_moved = abs(self._current_gimbal_yaw - self._last_sent_gimbal_yaw)
        pitch_moved = abs(self._current_gimbal_pitch - self._last_sent_gimbal_pitch)
        if yaw_moved < GIMBAL_SEND_THRESHOLD_DEG and pitch_moved < GIMBAL_SEND_THRESHOLD_DEG:
            return

        logger.info(
            f"Joystick2 gimbal: pitch={self._current_gimbal_pitch:+.1f}deg, "
            f"yaw={self._current_gimbal_yaw:+.1f}deg"
        )
        self.sender.point_gimbal(self._current_gimbal_pitch, self._current_gimbal_yaw)
        self._last_sent_gimbal_pitch = self._current_gimbal_pitch
        self._last_sent_gimbal_yaw = self._current_gimbal_yaw

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

    def _joystick_to_gimbal_yaw_rate(self, joystick_x):
        """
        Maps joystick2 X (0-4095) to a yaw RATE in deg/sec. 0 inside the
        deadzone, scaling linearly out to +/-GIMBAL_YAW_DEG_PER_SEC at
        full deflection. This is a velocity, not a target position.
        """
        if DEAD_LOW <= joystick_x <= DEAD_HIGH:
            return 0.0
        if joystick_x < DEAD_LOW:
            return -((DEAD_LOW - joystick_x) / DEAD_LOW) * GIMBAL_YAW_DEG_PER_SEC
        return +((joystick_x - DEAD_HIGH) / (4095 - DEAD_HIGH)) * GIMBAL_YAW_DEG_PER_SEC

    def _joystick_to_gimbal_pitch_rate(self, joystick_y):
        """
        Maps joystick2 Y (0-4095) to a pitch RATE in deg/sec. Below center
        drives the gimbal down, above center drives it up. 0 inside the
        deadzone. This is a velocity, not a target position.
        """
        if DEAD_LOW <= joystick_y <= DEAD_HIGH:
            return 0.0
        if joystick_y < DEAD_LOW:
            return -((DEAD_LOW - joystick_y) / DEAD_LOW) * GIMBAL_PITCH_DEG_PER_SEC
        return +((joystick_y - DEAD_HIGH) / (4095 - DEAD_HIGH)) * GIMBAL_PITCH_DEG_PER_SEC

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