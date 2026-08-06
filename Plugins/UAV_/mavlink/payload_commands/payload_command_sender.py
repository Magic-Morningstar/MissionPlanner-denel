# mavlink/uav_commads/payload_command_sender.py
#
# Everything payload/gimbal/camera. Does NOT own the MAVLink connection —
# attach() is called once by FlightCommandSender right after IT connects,
# handing over the shared command_drone + _mav_lock. This mirrors the
# "one shared connection, two dispatcher classes" design: only one class
# in this pair actually does _connect_with_retry()/heartbeat/threading.
#
# Registered handlers for every payload-related Command live at the
# bottom of this file, using the same commands.registry module
# FlightCommandSender's handlers use — registry._HANDLERS is a single
# global dict regardless of which file calls register_handler(), so this
# only stays correct because each Command type is only ever enqueued
# onto ONE bus (see commands/translator.py's two EDGE_TABLEs) — a
# payload Command is never even offered to FlightCommandSender's
# dispatch() call, and vice versa.

import logging
import queue
import threading
import time
from mavlink.payload_commands.payload_commands import Payload
from mavlink.payload_commands.payload_services import GimbalFrameBuilder
from commands.intents import *
from commands.registry import register_handler, dispatch
logger = logging.getLogger(__name__)


class PayloadCommandSender:

    def __init__(self, state, payload_command_bus):
        self.state = state
        self.command_bus = payload_command_bus
        self.command_queue = queue.Queue()

        self.command_drone = None
        self._mav_lock = None
        self.Payload = None

        self._motor_on = False          # local toggle state for motor_toggle() — see that method's docstring
        self._near_infrared_on = False  # local toggle state for near_infrared_toggle()
        self._eo_dzoom_on = False       # local toggle state for eo_dzoom_toggle() (standalone; zoom_in/out_start enable EO Dzoom unconditionally on their own, independent of this flag)
        self._eo_image_on = False       # local toggle state for eo_image_toggle()

        self._video_source_cycle = None  # set lazily on first use, avoids importing GimbalFrameBuilder at class-definition time
        self._ir_polarity_cycle = None
        self._tracking_template_cycle = None

    # ── Attach to the shared connection (called by FlightCommandSender) ────

    def attach(self, command_drone, mav_lock):
        self.command_drone = command_drone
        self._mav_lock = mav_lock
        self.Payload = Payload(state=self.state, command_drone=command_drone, lock=mav_lock)

    def detach(self):
        self.command_drone = None
        self._mav_lock = None
        self.Payload = None

    # ── Queues — mirrors FlightCommandSender's shape exactly ────────────────

    def enqueue(self, cmd, *args, **kwargs):
        self.command_queue.put((cmd, args, kwargs))

    def drain_input_commands(self):
        while True:
            try:
                cmd = self.command_bus.get_nowait()
            except queue.Empty:
                break
            dispatch(cmd, self)

    def process_queue(self):
        """Called every tick by FlightCommandSender._run_once() — this
        class has no background thread of its own."""
        while True:
            try:
                cmd, args, kwargs = self.command_queue.get_nowait()
            except queue.Empty:
                break
            cmd(*args, **kwargs)
            self.command_queue.task_done()

    # ── Gimbal pointing / rate ───────────────────────────────────────────────

    def point_gimbal(self, pitch_deg, yaw_deg):
        """
        pitch_deg / yaw_deg here are RELATIVE deltas (degrees to move by
        this call), not absolute targets. Still available for one-shot
        nudges (e.g. a UI "+5deg" button) — AnalogInputHandler no longer
        uses this for continuous joystick control; see set_gimbal_rate
        for that.
        """
        if not self.Payload:
            logger.warning("point_gimbal called with no Payload connection — ignoring.")
            return
        self.enqueue(
            self.Payload.initiate_PointAngle_Relative_Raw,
            yaw_deg,     # azimuth_delta_deg
            pitch_deg,   # tilt_delta_deg
        )

    def set_gimbal_rate(self, pitch_deg_s, yaw_deg_s):
        """
        Set the gimbal moving at a continuous azimuth/tilt rate (Manual
        Speed Mode) — send once when the desired rate changes, the
        gimbal keeps moving on its own until the next call. (0, 0) stops
        it. This is what AnalogInputHandler calls for joystick control.
        """
        if not self.Payload:
            logger.warning("set_gimbal_rate called with no Payload connection — ignoring.")
            return
        self.enqueue(
            self.Payload.initiate_ManualSpeed_Raw,
            yaw_deg_s,     # azimuth_vel_deg_s
            pitch_deg_s,   # tilt_vel_deg_s
        )

    # ── Zoom / focus — raw, direct, no EO Dzoom pre-enable ──────────────────

    def payloadZoomIn(self):
        self.enqueue(self.Payload.initiate_EODzoomOn_Raw)
        self.enqueue(self.Payload.initiate_ZoomIn_Raw)

    def payloadZoomOut(self):
        self.enqueue(self.Payload.initiate_EODzoomOn_Raw)
        self.enqueue(self.Payload.initiate_ZoomOut_Raw)

    # ── Zoom / focus — level-triggered raw start/stop ───────────────────────
    # Called by AnalogInputHandler on rising/falling edges of the held
    # zoom/focus flags, or directly by registered handlers below. Per the
    # ICD, zoom-in/out and focus+/- are "rising edge valid" — one start
    # command keeps the gimbal moving until a matching stop is sent, so
    # these are one-shot per press/release, not something to call
    # repeatedly while held.

    def zoom_in_start(self, speed=4):
        if self.Payload:
            self.enqueue(self.Payload.initiate_EODzoomOn_Raw)
            self.enqueue(self.Payload.initiate_ZoomIn_Raw, speed)

    def zoom_in_stop(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_ZoomStop_Raw)

    def zoom_out_start(self, speed=4):
        if self.Payload:
            self.enqueue(self.Payload.initiate_EODzoomOn_Raw)
            self.enqueue(self.Payload.initiate_ZoomOut_Raw, speed)

    def zoom_out_stop(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_ZoomStop_Raw)

    def focus_plus_start(self, speed=4):
        if self.Payload:
            self.enqueue(self.Payload.initiate_FocusPlus_Raw, speed)

    def focus_plus_stop(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_FocusStop_Raw)

    def focus_minus_start(self, speed=4):
        if self.Payload:
            self.enqueue(self.Payload.initiate_FocusMinus_Raw, speed)

    def focus_minus_stop(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_FocusStop_Raw)

    # ── Motor power ──────────────────────────────────────────────────────────

    def motor_toggle(self):
        """
        Toggle the gimbal servo motor on/off. initiate_MotorPower_Raw
        needs an explicit bool (on/off), not a toggle, and nothing here
        tracks the payload's actual motor state — so this flips a local
        `_motor_on` flag and sends that. If the PC and gimbal ever
        disagree (e.g. state reset without a matching physical motor
        change), this flag can drift from the real motor state; there's
        no query for actual motor status in payload_commands.py to
        reconcile against.
        """
        if not self.Payload:
            return
        self._motor_on = not self._motor_on
        self.enqueue(self.Payload.initiate_MotorPower_Raw, self._motor_on)

    def _ensure_motor_on(self):
        """
        Turns the motor on if `_motor_on` says it isn't already — used
        by features (tracking-follow, AI tracking) that need the servo
        powered on to have any physical effect, so the caller doesn't
        have to separately call motor_toggle() first.
        """
        if not self.Payload:
            return
        if not self._motor_on:
            self._motor_on = True
            self.enqueue(self.Payload.initiate_MotorPower_Raw, True)

    # ── IR camera digital zoom (direct, one-shot) ────────────────────────────

    def ir_camera_dzoom_plus(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_IRCameraDzoomPlus_Raw)

    def ir_camera_dzoom_minus(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_IRCameraDzoomMinus_Raw)

    # ── Near-infrared / EO Dzoom / EO image power — standalone toggles ──────

    def near_infrared_toggle(self):
        """Toggle near-infrared mode on/off (ICD 3.8 C2 command table, 0x4A/0x4B)."""
        if not self.Payload:
            return
        self._near_infrared_on = not self._near_infrared_on
        if self._near_infrared_on:
            self.enqueue(self.Payload.initiate_NearInfraredOn_Raw)
        else:
            self.enqueue(self.Payload.initiate_NearInfraredOff_Raw)

    def eo_dzoom_toggle(self):
        """
        Toggle EO digital zoom on/off directly (ICD 3.8 C2 0x06/0x07).
        Note zoom_in_start()/zoom_out_start()/payloadZoomIn()/payloadZoomOut()
        already enable EO Dzoom unconditionally before every zoom action —
        this is for a dedicated toggle button independent of an actual
        zoom press, and tracks its own state rather than sharing zoom's
        unconditional-enable behavior.
        """
        if not self.Payload:
            return
        self._eo_dzoom_on = not self._eo_dzoom_on
        if self._eo_dzoom_on:
            self.enqueue(self.Payload.initiate_EODzoomOn_Raw)
        else:
            self.enqueue(self.Payload.initiate_EODzoomOff_Raw)

    def eo_image_toggle(self):
        """Toggle EO 1 sensor power (ICD 3.8.1.4 Power Control, bits 0-1)."""
        if not self.Payload:
            return
        self._eo_image_on = not self._eo_image_on
        if self._eo_image_on:
            self.enqueue(self.Payload.initiate_EOImagePowerOn_Raw)
        else:
            self.enqueue(self.Payload.initiate_EOImagePowerOff_Raw)

    # ── Laser: power, single-shot, continuous, polling ──────────────────────

    def laser_start(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_LaserPowerOn_Raw)

    def laser_stop(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_LaserPowerOff_Raw)

    def continous_laser_start(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_LaserRangeContinuousStart_Raw)

    def continous_laser_stop(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_LaserRangeStop_Raw)

    def start_laser_polling(self, interval_sec=1.0, response_timeout=1.0):
        if getattr(self, '_laser_poll_thread', None) and self._laser_poll_thread.is_alive():
            return  # already running
        self._laser_poll_stop_event = threading.Event()
        self._laser_poll_thread = threading.Thread(
            target=self._laser_poll_loop,
            args=(interval_sec, response_timeout),
            daemon=True,
            name="LaserRangePoll",
        )
        self._laser_poll_thread.start()
        logger.info(f"Laser range polling started (every {interval_sec}s).")

    def stop_laser_polling(self):
        if getattr(self, '_laser_poll_stop_event', None):
            self._laser_poll_stop_event.set()
        logger.info("Laser range polling stopped.")

    def _laser_poll_loop(self, interval_sec, response_timeout):
        while not self._laser_poll_stop_event.is_set():
            if self.Payload:
                result = self.Payload.poll_status(response_timeout=response_timeout)
                distance = result.get('laser_range_m')
                if distance is not None:
                    logger.info(f"Laser range: {distance}m (new_reading={result.get('laser_range_new')})")
                    self._send_distance_to_gcs(distance)
                elif 'error' in result:
                    logger.debug(f"Laser range poll: {result['error']}")
                else:
                    logger.debug("Laser range poll: reply received but no valid reading yet (0/invalid).")
            self._laser_poll_stop_event.wait(interval_sec)

    def single_laser_range(self, response_timeout=1.0):
        if not self.Payload:
            return
        threading.Thread(
            target=self._single_laser_range_thread,
            args=(response_timeout,),
            daemon=True,
            name="LaserSingleShot",
        ).start()

    def _single_laser_range_thread(self, response_timeout):
        # Laser module must be powered on before a ranging trigger does
        # anything (ICD 3.8.1.3, C2 command 0x74). Sent directly here, not
        # via enqueue(), because this runs on its own thread: enqueuing
        # would only guarantee it lands in command_queue, not that it's
        # drained and sent before the ranging trigger below.
        self.Payload.initiate_LaserPowerOn_Raw()
        result = self.Payload.get_laser_range(response_timeout=response_timeout)
        distance = result.get('laser_range_m')
        if distance is not None:
            logger.info(f"Laser single-shot range: {distance}m")
            self._send_distance_to_gcs(distance)
        elif 'error' in result:
            logger.debug(f"Laser single-shot: {result['error']}")

    def laser_zoom_in(self):
        """Laser's own zoom in (beam divergence) — separate from EO/IR optical zoom."""
        if self.Payload:
            self.enqueue(self.Payload.initiate_LaserZoomIn_Raw)

    def laser_zoom_out(self):
        """Laser's own zoom out (beam divergence) — separate from EO/IR optical zoom."""
        if self.Payload:
            self.enqueue(self.Payload.initiate_LaserZoomOut_Raw)

    def _send_distance_to_gcs(self, distance_m):
        """Relay the laser distance to any connected GCS as NAMED_VALUE_FLOAT."""
        if not self.command_drone:
            return
        try:
            self.command_drone.mav.named_value_float_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                b"LaserRange",
                float(distance_m),
            )
        except Exception as e:
            logger.warning(f"Failed to relay laser distance to GCS: {e}")

    def _send_distance_sensor_to_gcs(self, distance_m):
        """Alternative to _send_distance_to_gcs using DISTANCE_SENSOR. Not
        currently called — see _laser_poll_loop if you want to switch."""
        if not self.command_drone:
            return
        try:
            from pymavlink import mavutil
            self.command_drone.mav.distance_sensor_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                100, 65535,
                min(int(distance_m * 100), 65535),
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                0,
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE,
                0,
            )
        except Exception as e:
            logger.warning(f"Failed to relay laser distance sensor to GCS: {e}")

    # ── Object tracking ───────────────────────────────────────────────────
    # tracking_start locks the tracker onto whatever's currently under the
    # search cross. initiate_TrackingTurnOn_Raw()'s default already puts
    # A1 into SERVO_TRACKING_MODE in the same frame as the E1 lock-on.
    # What tracking still needs standalone is Motor ON — a separate A1
    # state, not bundled into mode-select — hence _ensure_motor_on().

    def tracking_start(self):
        if self.Payload:
            self._ensure_motor_on()
            self.enqueue(self.Payload.initiate_TrackingSetTemplateSize_Raw, 128)
            self.enqueue(self.Payload.initiate_TrackingTurnOn_Raw)
        if self.state:
            self.state.TRACKING_ENGAGED = True

    def tracking_stop(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_TrackingStop_Raw)
        if self.state:
            self.state.TRACKING_ENGAGED = False

    def tracking_search(self, azimuth_nudge, tilt_nudge):
        """Nudge the tracking search cross — used by AnalogInputHandler when JOYSTICK_TRACK_MODE is on."""
        if self.Payload:
            self.enqueue(self.Payload.initiate_TrackingSearch_Raw, azimuth_nudge, tilt_nudge)

    def ai_tracking_on(self):
        if self.Payload:
            self._ensure_motor_on()
            self.enqueue(self.Payload.initiate_TrackingAIOn_Raw)
        if self.state:
            self.state.AI_TRACKING_ENGAGED = True

    def ai_tracking_off(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_TrackingAIOff_Raw)
        if self.state:
            self.state.AI_TRACKING_ENGAGED = False

    def joystick_track_mode_on(self):
        """Joystick2 now nudges the tracking search cross instead of setting gimbal rate."""
        if self.state:
            self.state.JOYSTICK_TRACK_MODE = True
            logger.info("Joystick track mode ON — Joystick2 now nudges the tracking cross.")

    def joystick_track_mode_off(self):
        """Joystick2 back to controlling gimbal rate."""
        if self.state:
            self.state.JOYSTICK_TRACK_MODE = False
            logger.info("Joystick track mode OFF — Joystick2 back to gimbal rate control.")

    def tracking_template_toggle(self):
        """
        Cycle the tracking template size (ICD 3.11's E1 Basic Command
        enum: 0x21 small/32x32, 0x22 medium/64x64, 0x23 big/128x128).
        initiate_TrackingSetTemplateSize_Raw's existing callers (e.g.
        tracking_start()) pass a raw pixel count (128) rather than one of
        these ICD enum codes — worth checking against real hardware
        which one it actually expects; this method uses the ICD's own
        enum codes since that's what the spec documents.
        """
        if not self.Payload:
            return
        if self._tracking_template_cycle is None:
            self._tracking_template_cycle = [0x21, 0x22, 0x23]  # small, medium, big
            self._tracking_template_index = 0
        self._tracking_template_index = (self._tracking_template_index + 1) % len(self._tracking_template_cycle)
        size = self._tracking_template_cycle[self._tracking_template_index]
        self.enqueue(self.Payload.initiate_TrackingSetTemplateSize_Raw, size)

    def tracking_source_toggle(self):
        """
        NOT IMPLEMENTED: ICD 3.11 explicitly marks E2's "Tracking source
        choose" field "(Reserved)" — no confirmed real behavior to wire
        up. Logs and does nothing rather than sending fabricated bytes
        for a field the spec itself says isn't finalized.
        """
        logger.info("tracking_source_toggle: not implemented — ICD marks this field Reserved.")

    # ── Video source toggle (BIT_VIDEO_IP) ──────────────────────────────────
    # Simple two-way EO1<->IR toggle. Expand _video_source_cycle if you
    # want PIP/EO2/Fusion included in the rotation too.

    def video_source_toggle(self):
        if not self.Payload:
            return
        if self._video_source_cycle is None:
            self._video_source_cycle = [GimbalFrameBuilder.VIDEO_EO1, GimbalFrameBuilder.VIDEO_IR_THERMAL]
            self._video_source_index = 0
        self._video_source_index = (self._video_source_index + 1) % len(self._video_source_cycle)
        source = self._video_source_cycle[self._video_source_index]
        self.enqueue(self.Payload.initiate_SwitchVideoSource_Raw, source)

    # ── IR polarity/palette toggle (BIT_IR_POLARITY, "WHITE BTN") ───────────
    # REVERTED to a plain 2-way white-hot/black-hot toggle. An earlier
    # pass folded "rainbow" into this as a 3rd cycle state, on the
    # assumption there was no dedicated control for it — but
    # PayloadCommand actually has its own dedicated `ir_rainbow` bit
    # (its own physical button, per the original bit spec), so rainbow
    # gets its own direct command below instead of living inside this
    # toggle's cycle.

    def ir_polarity_toggle(self):
        if not self.Payload:
            return
        if self._ir_polarity_cycle is None:
            self._ir_polarity_cycle = [
                self.Payload.initiate_IRPolarityWhiteHot_Raw,
                self.Payload.initiate_IRPolarityBlackHot_Raw,
            ]
            self._ir_polarity_index = 0
        self._ir_polarity_index = (self._ir_polarity_index + 1) % len(self._ir_polarity_cycle)
        self.enqueue(self._ir_polarity_cycle[self._ir_polarity_index])

    def ir_rainbow(self):
        """Set IR palette to rainbow — its own dedicated button/bit, not part of ir_polarity_toggle's cycle (see that method's docstring)."""
        if self.Payload:
            self.enqueue(self.Payload.initiate_IRRainbow_Raw)

    # ── Picture / video mode ─────────────────────────────────────────────────

    def take_picture(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_TakePhoto_Raw)

    def start_record(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_StartRecording_Raw)

    def stop_record(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_StopRecording_Raw)

    def picture_record_mode_toggle(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_PicRecordSwitch_Raw)


# ── Registered handlers ──────────────────────────────────────────────────────

@register_handler(FocusPlusCommand)
def _handle_focus_plus(sender, cmd):
    logger.debug("FocusPlusCommand received")
    sender.focus_plus_start()

@register_handler(FocusPlusFallCommand)
def _handle_focus_plus_fall(sender, cmd):
    logger.debug("FocusPlusFallCommand received")
    sender.focus_plus_stop()

@register_handler(FocusMinusCommand)
def _handle_focus_minus(sender, cmd):
    logger.debug("FocusMinusCommand received")
    sender.focus_minus_start()

@register_handler(FocusMinusFallCommand)
def _handle_focus_minus_fall(sender, cmd):
    logger.debug("FocusMinusFallCommand received")
    sender.focus_minus_stop()

@register_handler(VideoSourceToggleCommand)
def _handle_video_source_toggle(sender, cmd):
    logger.debug("VideoSourceToggleCommand received")
    sender.video_source_toggle()

@register_handler(IRPolarityToggleCommand)
def _handle_ir_polarity_toggle(sender, cmd):
    logger.debug("IRPolarityToggleCommand received")
    sender.ir_polarity_toggle()

# "Image Sensor Change" (GREEN BTN) reuses the same video_source_toggle()
# as VIDEO_IP — see the original note this carried in command_sender.py:
# it's plausible one of these two fields means something else entirely
# and this wiring is provisional, not confirmed against real hardware.
@register_handler(ImageSensorChangeCommand)
def _handle_image_sensor_change(sender, cmd):
    logger.debug("ImageSensorChangeCommand received")
    sender.video_source_toggle()

@register_handler(LaserPowerOnCommand)
def _handle_laser_power_on(sender, cmd):
    logger.debug("LaserPowerOnCommand received")
    sender.laser_start()

@register_handler(LaserPowerOffCommand)
def _handle_laser_power_off(sender, cmd):
    logger.debug("LaserPowerOffCommand received")
    sender.laser_stop()

@register_handler(LaserContModeStartCommand)
def _handle_laser_cont_mode_start(sender, cmd):
    logger.debug("LaserContModeStartCommand received")
    sender.continous_laser_start()
    sender.start_laser_polling()

@register_handler(LaserContModeStopCommand)
def _handle_laser_cont_mode_stop(sender, cmd):
    logger.debug("LaserContModeStopCommand received")
    sender.stop_laser_polling()
    sender.continous_laser_stop()

@register_handler(LaserSingleTriggerCommand)
def _handle_laser_single_trigger(sender, cmd):
    logger.debug("LaserSingleTriggerCommand received")
    sender.single_laser_range()

@register_handler(AITrackingOnCommand)
def _handle_ai_tracking_on(sender, cmd):
    logger.debug("AITrackingOnCommand received")
    sender.ai_tracking_on()

@register_handler(AITrackingOffCommand)
def _handle_ai_tracking_off(sender, cmd):
    logger.debug("AITrackingOffCommand received")
    sender.ai_tracking_off()

@register_handler(JoystickTrackModeOnCommand)
def _handle_joystick_track_mode_on(sender, cmd):
    logger.debug("JoystickTrackModeOnCommand received")
    sender.joystick_track_mode_on()

@register_handler(JoystickTrackModeOffCommand)
def _handle_joystick_track_mode_off(sender, cmd):
    logger.debug("JoystickTrackModeOffCommand received")
    sender.joystick_track_mode_off()

@register_handler(ZoomInCommand)
def _handle_zoomin(sender, cmd):
    logger.info("ZoomInCommand received")
    sender.state.ZOOMIN_PRESSED = True

@register_handler(ZoomInFallCommand)
def _handle_zoominFall(sender, cmd):
    logger.info("ZoomInFallCommand received")
    sender.state.ZOOMIN_PRESSED = False

@register_handler(ZoomOutCommand)
def _handle_zoomout(sender, cmd):
    logger.info("ZoomOutCommand received")
    sender.state.ZOOMOUT_PRESSED = True

@register_handler(ZoomOutFallCommand)
def _handle_zoomoutFall(sender, cmd):
    logger.info("ZoomOutFallCommand received")
    sender.state.ZOOMOUT_PRESSED = False

# FOV+/- reuse zoom_out_start/stop and zoom_in_start/stop directly (not
# via the ZOOMIN/OUT_PRESSED state-flag path above) — FOV+ IS zoom out
# and FOV- IS zoom in, per the ICD's own naming.
@register_handler(FOVPlusCommand)
def _handle_fov_plus(sender, cmd):
    logger.info("FOVPlusCommand received")
    sender.zoom_out_start()

@register_handler(FOVPlusFallCommand)
def _handle_fov_plus_fall(sender, cmd):
    logger.info("FOVPlusFallCommand received")
    sender.zoom_out_stop()

@register_handler(FOVMinusCommand)
def _handle_fov_minus(sender, cmd):
    logger.info("FOVMinusCommand received")
    sender.zoom_in_start()

@register_handler(FOVMinusFallCommand)
def _handle_fov_minus_fall(sender, cmd):
    logger.info("FOVMinusFallCommand received")
    sender.zoom_in_stop()

@register_handler(TrackingStartCommand)
def _handle_tracking_start(sender, cmd):
    logger.debug("TrackingStartCommand received")
    sender.tracking_start()

@register_handler(TrackingStopCommand)
def _handle_tracking_stop(sender, cmd):
    logger.debug("TrackingStopCommand received")
    sender.tracking_stop()

# ── New PAYLOAD_COMMAND-only fields ──────────────────────────────────────────

@register_handler(LaserZoomInCommand)
def _handle_laser_zoom_in(sender, cmd):
    logger.debug("LaserZoomInCommand received")
    sender.laser_zoom_in()

@register_handler(LaserZoomOutCommand)
def _handle_laser_zoom_out(sender, cmd):
    logger.debug("LaserZoomOutCommand received")
    sender.laser_zoom_out()

@register_handler(TrackingTemplateToggleCommand)
def _handle_tracking_template_toggle(sender, cmd):
    logger.debug("TrackingTemplateToggleCommand received")
    sender.tracking_template_toggle()

@register_handler(TrackingSourceToggleCommand)
def _handle_tracking_source_toggle(sender, cmd):
    logger.debug("TrackingSourceToggleCommand received")
    sender.tracking_source_toggle()

@register_handler(TakePictureCommand)
def _handle_take_picture(sender, cmd):
    logger.debug("TakePictureCommand received")
    sender.take_picture()

@register_handler(StartRecordCommand)
def _handle_start_record(sender, cmd):
    logger.debug("StartRecordCommand received")
    sender.start_record()

@register_handler(StopRecordCommand)
def _handle_stop_record(sender, cmd):
    logger.debug("StopRecordCommand received")
    sender.stop_record()

@register_handler(PictureRecordModeToggleCommand)
def _handle_picture_record_mode_toggle(sender, cmd):
    logger.debug("PictureRecordModeToggleCommand received")
    sender.picture_record_mode_toggle()

@register_handler(MotorToggleCommand)
def _handle_motor_toggle(sender, cmd):
    logger.debug("MotorToggleCommand received")
    sender.motor_toggle()

@register_handler(NearInfraredToggleCommand)
def _handle_near_infrared_toggle(sender, cmd):
    logger.debug("NearInfraredToggleCommand received")
    sender.near_infrared_toggle()

@register_handler(EODzoomToggleCommand)
def _handle_eo_dzoom_toggle(sender, cmd):
    logger.debug("EODzoomToggleCommand received")
    sender.eo_dzoom_toggle()

@register_handler(IRCameraDzoomPlusCommand)
def _handle_ir_camera_dzoom_plus(sender, cmd):
    logger.debug("IRCameraDzoomPlusCommand received")
    sender.ir_camera_dzoom_plus()

@register_handler(IRCameraDzoomMinusCommand)
def _handle_ir_camera_dzoom_minus(sender, cmd):
    logger.debug("IRCameraDzoomMinusCommand received")
    sender.ir_camera_dzoom_minus()

@register_handler(IRRainbowCommand)
def _handle_ir_rainbow(sender, cmd):
    logger.debug("IRRainbowCommand received")
    sender.ir_rainbow()

@register_handler(EOImageToggleCommand)
def _handle_eo_image_toggle(sender, cmd):
    logger.debug("EOImageToggleCommand received")
    sender.eo_image_toggle()