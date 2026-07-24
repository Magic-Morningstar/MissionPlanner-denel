# mavlink/uav_commads/analog_input_handler.py

import logging
import time

logger = logging.getLogger(__name__)

ADC_MAX = 4095
ADC_MID = ADC_MAX / 2.0   # 2047.5

# Rate control: at full joystick deflection (+/-100%), the gimbal moves at
# this many degrees per second. Center stick = 0 deg/s. Azimuth and tilt
# are both continuous/360° on this gimbal, so there is intentionally no
# angle clamping anywhere in this file — only the RATE is controlled,
# total travel is never capped.
GIMBAL_YAW_DEG_PER_SEC   = 360.0   # azimuth
GIMBAL_PITCH_DEG_PER_SEC = 360.0   # tilt

# Small per-tick deltas are buffered and only flushed as an actual
# relative-angle command once the pending amount reaches this size, so
# holding the stick at a small deflection doesn't flood the link with
# near-zero commands. This only limits how OFTEN a command goes out —
# it never limits how far the gimbal can travel.
GIMBAL_SEND_THRESHOLD_DEG = 0.5


# Small deadzone around center — any deflection producing a percent in
# this range is treated as exactly 0%, so tiny stick noise near center
# doesn't produce drift on any axis using _percent_from_joystick.
DEADZONE_PERCENT = 6


def _percent_from_joystick(value):
    """
    Maps a raw ADC value (0-4095) to a signed percent, split at the
    midpoint (2047.5):
        0    -> -100%
        mid  ->    0%
        4095 -> +100%
    Values within +/-DEADZONE_PERCENT of center are snapped to exactly 0%.
    If a given axis moves the wrong physical direction, flip its sign at
    the call site rather than here, since this mapping itself is
    direction-agnostic.
    """
    if value <= ADC_MID:
        percent = (value - ADC_MID) / ADC_MID * 100.0
    else:
        percent = (value - ADC_MID) / (ADC_MAX - ADC_MID) * 100.0

    if -DEADZONE_PERCENT <= percent <= DEADZONE_PERCENT:
        return 0.0
    return percent


class _EdgeTrigger:
    """Fires 'start' on False->True, 'stop' on True->False, else None."""

    def __init__(self):
        self._prev = False

    def update(self, active: bool):
        if active and not self._prev:
            self._prev = True
            return "start"
        if not active and self._prev:
            self._prev = False
            return "stop"
        self._prev = active
        return None


class AnalogInputHandler:
    """
    Completely stateless with respect to MAVLink — it calls methods on
    UAVCommandSender (set_airspeed, turn_left, point_gimbal, zoom_in_start,
    etc.) and never touches the drone connection directly.

    Gimbal control (Joystick2 X/Y) — pure RATE control, sent as relative
    angle deltas:
      Deflection percent (see _percent_from_joystick, +/-2% deadzone
      around center) sets a deg/sec rate
      per axis. Each tick's delta (rate * dt) is added to a small pending
      buffer; once the buffer is big enough to be worth a MAVLink message
      it's flushed as one initiate_PointAngle_Relative_Raw call and reset
      to zero. There is deliberately NO local "current angle" tracking and
      NO clamping — azimuth and tilt are both continuous 360° on this
      gimbal, so a relative delta is always valid no matter where the
      gimbal currently is. Centering the stick just stops producing
      deltas, so the gimbal holds wherever it last moved to.

    Zoom / focus — level-triggered, NOT proportional to deflection:
      As long as the matching *_Pressed flag on `state` is True, a single
      "start" command fires once (rising edge) and the gimbal keeps
      zooming/focusing on its own, per the ICD's "rising edge is valid"
      behavior; a single "stop" fires once the flag goes False (falling
      edge). How long the flag stays True is entirely up to whatever sets
      it — this class doesn't decide "how far," only start/stop.

      NOTE: this assumes `state` exposes get_Zoom_In_Pressed /
      get_Zoom_Out_Pressed / get_Focus_Plus_Pressed / get_Focus_Minus_Pressed
      as booleans — rename these in _handle_zoom_focus() if your State
      class uses different names. If they don't exist yet, the getattr
      default of False just means zoom/focus never fires (safe no-op)
      rather than crashing.
    """

    def __init__(self, state, manual_ctrl, sender):
        self.state       = state
        self.manual_ctrl = manual_ctrl   # ManualController instance
        self.sender      = sender        # UAVCommandSender for command dispatch

        self._last_turn_degrees   = 0.0
        self._last_altitude_delta = 0.0

        # Buffered, not-yet-sent relative gimbal motion (degrees). Reset
        # to 0 each time it's flushed as an actual command.
        self._pending_yaw_delta   = 0.0
        self._pending_pitch_delta = 0.0
        self._last_gimbal_tick_time = None

        # Edge triggers for level-based zoom/focus control.
        self._zoom_in_trigger     = _EdgeTrigger()
        self._zoom_out_trigger    = _EdgeTrigger()
        self._focus_plus_trigger  = _EdgeTrigger()
        self._focus_minus_trigger = _EdgeTrigger()

    def process(self):
        current_mode = self.state.get_UAV_Current_Mode

        # Gimbal pointing is independent of flight mode — always active.
        self._handle_joystick2_gimbal()
        self._handle_joystick(current_mode)
        self._handle_zoom_focus()

    # ── Joystick2 X/Y → gimbal yaw/pitch (pure rate, relative-delta) ────────

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

        yaw_percent = _percent_from_joystick(payload_joy_x)
        pitch_percent = _percent_from_joystick(payload_joy_y)

        yaw_rate_deg_s = (yaw_percent / 100.0) * GIMBAL_YAW_DEG_PER_SEC
        pitch_rate_deg_s = (pitch_percent / 100.0) * GIMBAL_PITCH_DEG_PER_SEC

        self._pending_yaw_delta += yaw_rate_deg_s * dt
        self._pending_pitch_delta += pitch_rate_deg_s * dt

        if (abs(self._pending_yaw_delta) < GIMBAL_SEND_THRESHOLD_DEG
                and abs(self._pending_pitch_delta) < GIMBAL_SEND_THRESHOLD_DEG):
            return

        logger.info(
            f"Joystick2 gimbal delta: pitch={self._pending_pitch_delta:+.2f}deg, "
            f"yaw={self._pending_yaw_delta:+.2f}deg"
        )
        self.sender.point_gimbal(self._pending_pitch_delta, self._pending_yaw_delta)
        self._pending_pitch_delta = 0.0
        self._pending_yaw_delta = 0.0

    # ── Joystick X/Y → mode-dependent control ────────────────────────────────

    def _handle_joystick(self, mode):
        joystick_x = self.state.get_Joystick_X
        joystick_y = self.state.get_Joystick_Y

        if mode in ("FBWA", "FBWB"):
            # Start streaming if not already — ManualController reads
            # state directly at 10Hz so no values need passing here
            if self.manual_ctrl and not self.manual_ctrl._streaming:
                self.manual_ctrl.start_streaming()

    # ── Zoom / focus — level-triggered start/stop ────────────────────────

    def _handle_zoom_focus(self):
        
        zoom_in = self.sender.state.ZOOMIN_PRESSED
        if zoom_in == True:
            self.sender.zoom_in_start()
            self.sender.state.ZOOMING_IN = True
        elif self.sender.state.ZOOMING_IN:
            self.sender.state.ZOOMING_IN = False
            self.sender.zoom_in_stop()

        zoom_out =self.sender.state.ZOOMOUT_PRESSED
        if zoom_out == True:
            self.sender.zoom_out_start()
            self.sender.state.ZOOMING_OUT =False
        elif self.sender.state.ZOOMING_OUT:
            self.sender.state.ZOOMING_OUT =True
            self.sender.zoom_out_stop()

        '''
        focus_plus = self.state.FOCUSIN_PRESSED
        if focus_plus == True:
            self.sender.focus_plus_start()
        else:
            self.sender.focus_plus_stop()

        focus_minus = self.state.FOCUSOUT_PRESSED
        if focus_minus == True:
            self.sender.focus_minus_start()
        else:
            self.sender.focus_minus_stop()'''

    # ── Mapping helpers — heading / altitude, unrelated to the gimbal ────────

    def _joystick_to_degrees(self, joystick_x):
        """Maps joystick X (0-4095) to heading change in degrees. Max ±45deg."""
        percent = _percent_from_joystick(joystick_x)
        return (percent / 100.0) * 45.0

    def _joystick_to_altitude_delta(self, joystick_y):
        """Maps joystick Y (0-4095) to altitude delta in meters. Max ±20m."""
        percent = _percent_from_joystick(joystick_y)
        return (percent / 100.0) * 20.0