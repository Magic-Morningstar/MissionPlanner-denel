# mavlink/payload_commands/payload_analog_input_handler.py
#
# Payload-only half of what used to be AnalogInputHandler — gimbal rate
# (Joystick2 -> Manual Speed Mode), tracking-search cross nudging, and
# zoom/focus edge-triggered start/stop. Deliberately takes only (state,
# sender) — no manual_ctrl, since flight-mode joystick streaming isn't
# this class's concern. See analog_input_handler.py for the flight-only
# counterpart (joystick -> FBWA/FBWB manual-control streaming).
#
# `sender` here is a PayloadCommandSender — every method call on it in
# this file is a payload action (set_gimbal_rate, tracking_search,
# zoom_in_start, etc.), same as it always was; the only thing that
# changed is which class answers to "sender".

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
GIMBAL_YAW_DEG_PER_SEC   = 45.0   # azimuth
GIMBAL_PITCH_DEG_PER_SEC = 45.0   # tilt

# Gimbal rate is sent via Manual Speed Mode (set_gimbal_rate), not
# repeated relative-angle deltas — see PayloadAnalogInputHandler's
# docstring. A new command only goes out when the desired rate changes
# by at least this much...
GIMBAL_RATE_CHANGE_THRESHOLD_DEG_S = 2.0
# ...or when this much time has passed since the last send, whichever
# comes first. The keepalive exists purely as a safety net: Manual Speed
# Mode is "fire and forget" (the gimbal keeps moving until told
# otherwise), so a single dropped command on a lossy link would otherwise
# leave the gimbal spinning indefinitely at a stale rate.
GIMBAL_RATE_KEEPALIVE_ACTIVE_SEC = 0.3
GIMBAL_RATE_KEEPALIVE_IDLE_SEC   = 2.0
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


class PayloadAnalogInputHandler:
    """
    Completely stateless with respect to MAVLink — it calls methods on
    PayloadCommandSender (set_gimbal_rate, tracking_search, zoom_in_start,
    etc.) and never touches the drone connection directly.

    Gimbal control (Joystick2 X/Y) — pure RATE control via Manual Speed
    Mode, sent on change + keepalive:
      Deflection percent (_percent_from_joystick, +/-DEADZONE_PERCENT
      dead zone around center) maps directly to a deg/sec rate per axis.
      A new set_gimbal_rate() call only goes out when the desired rate
      changes by more than GIMBAL_RATE_CHANGE_THRESHOLD_DEG_S, or when
      the keepalive interval has elapsed since the last send (whichever
      comes first) — plus immediately, unconditionally, the moment both
      axes return to zero, so releasing the stick stops promptly rather
      than waiting on the change threshold.

    Zoom / focus — level-triggered via _EdgeTrigger, NOT proportional to
    deflection:
      As long as the matching *_PRESSED flag on `state` is True, a single
      "start" fires once (rising edge) and the gimbal keeps zooming/
      focusing on its own, per the ICD's "rising edge is valid" behavior;
      a single "stop" fires once the flag goes False (falling edge).
    """

    def __init__(self, state, sender):
        self.state  = state
        self.sender = sender   # PayloadCommandSender for command dispatch

        # Last rate actually sent via set_gimbal_rate, and when — used to
        # decide whether a new send is warranted (change or keepalive).
        self._last_sent_yaw_vel     = 0.0
        self._last_sent_pitch_vel   = 0.0
        self._last_gimbal_send_time = None

        # Edge triggers for level-based zoom/focus control.
        self._zoom_in_trigger     = _EdgeTrigger()
        self._zoom_out_trigger    = _EdgeTrigger()
        self._focus_plus_trigger  = _EdgeTrigger()
        self._focus_minus_trigger = _EdgeTrigger()

        # Last tracking-search nudge sent, when JOYSTICK_TRACK_MODE is on
        # (see _handle_joystick2_tracking_search) — send-on-change, same
        # spirit as the gimbal-rate keepalive logic above, just without a
        # keepalive since a search nudge is a discrete move, not a
        # continuous rate the gimbal needs reminding of.
        self._last_search_azimuth = 0
        self._last_search_tilt    = 0

    def process(self):
        # Gimbal pointing is independent of flight mode — always active.
        self._handle_joystick2_gimbal()
        self._handle_zoom_focus()

    # ── Joystick2 X/Y → gimbal yaw/pitch (Manual Speed Mode, send-on-change) ─
    def _handle_joystick2_gimbal(self):
        if getattr(self.state, 'TRACKING_ENGAGED', False) or getattr(self.state, 'AI_TRACKING_ENGAGED', False):
            # Tracking (or AI tracking) owns the gimbal now. The keepalive
            # below sends A1=SERVO_MANUAL_SPEED roughly every
            # GIMBAL_RATE_KEEPALIVE_ACTIVE_SEC regardless of stick position —
            # including at zero velocity — which is a genuine servo-mode
            # change away from SERVO_TRACKING_MODE, not a no-op. Left
            # running, it forces the gimbal back out of tracking-follow
            # on the very next keepalive tick. Suppressing entirely here —
            # including search-cross nudging, since nudging a cross
            # you're already locked onto doesn't do anything useful —
            # until tracking_stop()/ai_tracking_off() clears these flags.
            return

        if getattr(self.state, 'JOYSTICK_TRACK_MODE', False):
            self._handle_joystick2_tracking_search()
            return

        payload_joy_x = self.state.get_Payload_Joystick_X
        payload_joy_y = self.state.get_Payload_Joystick_Y

        yaw_percent = _percent_from_joystick(payload_joy_x)
        pitch_percent = _percent_from_joystick(payload_joy_y)

        yaw_vel = (yaw_percent / 100.0) * GIMBAL_YAW_DEG_PER_SEC
        pitch_vel = (pitch_percent / 100.0) * GIMBAL_PITCH_DEG_PER_SEC

        now = time.monotonic()
        yaw_changed = abs(yaw_vel - self._last_sent_yaw_vel) >= GIMBAL_RATE_CHANGE_THRESHOLD_DEG_S
        pitch_changed = abs(pitch_vel - self._last_sent_pitch_vel) >= GIMBAL_RATE_CHANGE_THRESHOLD_DEG_S

        # Keepalive cadence depends on whether the gimbal is currently
        # moving or at rest. While actively commanding a non-zero rate, keep
        # the fast keepalive — that's what protects against a dropped
        # in-motion command leaving the gimbal spinning at a stale rate.
        # Once the last command sent was a stop (rate = 0 on both axes),
        # back off to a much slower keepalive: a dropped "still at zero"
        # message costs nothing, since there's no motion to correct if it's
        # lost. This is what stops idle stick position from generating
        # constant zero-rate traffic into command_queue.
        at_rest = (self._last_sent_yaw_vel == 0.0 and self._last_sent_pitch_vel == 0.0)
        keepalive_interval = (
            GIMBAL_RATE_KEEPALIVE_IDLE_SEC if at_rest else GIMBAL_RATE_KEEPALIVE_ACTIVE_SEC
        )
        keepalive_due = (
            self._last_gimbal_send_time is None
            or (now - self._last_gimbal_send_time) >= keepalive_interval
        )

        # Returning to a full stop should feel immediate, not wait on the
        # change threshold or the next keepalive tick.
        just_stopped = (
            yaw_vel == 0.0 and pitch_vel == 0.0
            and (self._last_sent_yaw_vel != 0.0 or self._last_sent_pitch_vel != 0.0)
        )

        if not (yaw_changed or pitch_changed or keepalive_due or just_stopped):
            return

        logger.info(f"Gimbal rate: pitch={pitch_vel:+.1f}deg/s, yaw={yaw_vel:+.1f}deg/s")
        self.sender.set_gimbal_rate(pitch_vel, yaw_vel)
        self._last_sent_yaw_vel = yaw_vel
        self._last_sent_pitch_vel = pitch_vel
        self._last_gimbal_send_time = now

    def _handle_joystick2_tracking_search(self):
        """
        When JOYSTICK_TRACK_MODE is on (set by JoystickTrackModeOnCommand,
        BIT_JOYSTICK_TRACK in firmware), Joystick2 nudges the tracking
        search cross (E1 Search command, ICD 3.11.1.1) instead of setting
        gimbal rate. Reuses the same percent/deadzone mapping, scaled into
        the search command's -15..+15 nudge range, and only sends on
        change rather than every tick.
        """
        payload_joy_x = self.state.get_Payload_Joystick_X
        payload_joy_y = self.state.get_Payload_Joystick_Y

        x_percent = _percent_from_joystick(payload_joy_x)
        y_percent = _percent_from_joystick(payload_joy_y)

        azimuth_nudge = int(round((x_percent / 100.0) * 15))
        tilt_nudge = int(round((y_percent / 100.0) * 15))

        if azimuth_nudge == self._last_search_azimuth and tilt_nudge == self._last_search_tilt:
            return
        self._last_search_azimuth = azimuth_nudge
        self._last_search_tilt = tilt_nudge

        if azimuth_nudge == 0 and tilt_nudge == 0:
            return  # centered — nothing to nudge

        logger.info(f"Tracking search nudge: azimuth={azimuth_nudge:+d}, tilt={tilt_nudge:+d}")
        self.sender.tracking_search(azimuth_nudge, tilt_nudge)

    # ── Zoom / focus — level-triggered start/stop, via _EdgeTrigger ─────────

    def _handle_zoom_focus(self):
        zoom_in_edge = self._zoom_in_trigger.update(bool(getattr(self.state, "ZOOMIN_PRESSED", False)))
        if zoom_in_edge == "start":
            self.sender.zoom_in_start()
        elif zoom_in_edge == "stop":
            self.sender.zoom_in_stop()

        zoom_out_edge = self._zoom_out_trigger.update(bool(getattr(self.state, "ZOOMOUT_PRESSED", False)))
        if zoom_out_edge == "start":
            self.sender.zoom_out_start()
        elif zoom_out_edge == "stop":
            self.sender.zoom_out_stop()

        focus_plus_edge = self._focus_plus_trigger.update(bool(getattr(self.state, "FOCUSIN_PRESSED", False)))
        if focus_plus_edge == "start":
            self.sender.focus_plus_start()
        elif focus_plus_edge == "stop":
            self.sender.focus_plus_stop()

        focus_minus_edge = self._focus_minus_trigger.update(bool(getattr(self.state, "FOCUSOUT_PRESSED", False)))
        if focus_minus_edge == "start":
            self.sender.focus_minus_start()
        elif focus_minus_edge == "stop":
            self.sender.focus_minus_stop()