# mavlink/uav_commads/discrete_command_sender.py

import logging
import queue
import threading
from pymavlink import mavutil
from mavlink.mavlink_worker import MavlinkWorker
from mavlink.uav_commads.commands.arms_commands import Arms
from mavlink.uav_commads.commands.autotakeoff_commands import AutoTakeOff
from mavlink.uav_commads.commands.mode_commands import ModeCommander
from mavlink.uav_commads.commands.speed_commands import Speed_Controller
from mavlink.uav_commads.commands.direction_commands import DirectionCommander, ManualController
from mavlink.uav_commads.commands.payload_commands import Payload
from mavlink.uav_commads.commands.payload_services import GimbalFrameBuilder
from commands.intents import *
from commands.registry import register_handler, dispatch
import time
logger = logging.getLogger(__name__)


class UAVCommandSender(MavlinkWorker):
    """
    Handles discrete button commands arriving via command_bus.
    Owns the MAVLink connection, command queue, and all
    sub-controllers (Manual, Director, ManualCtrl, AutoTakeOff).

    Analog input handling is delegated to AnalogInputHandler,
    which is instantiated here and called each _run_once().
    """

    def __init__(self, connection_string, state, command_bus, watchdog=None, interval=0.05):
        super().__init__(connection_string, state, watchdog, interval)
        self.command_drone       = None
        self.command_queue       = queue.Queue()
        self.command_bus         = command_bus
        self.takeoff_in_progress = False
        self._mav_lock           = threading.Lock()

        self.Payload             = None
        self.Manual              = None
        self.Director            = None
        self.ManualCtrl          = None
        self._ato                = None
        self._analog             = None   # AnalogInputHandler — set after connection

    # ── Connection ────────────────────────────────────────────────────────────

    def _do_connect(self):
        self.command_drone = self._connect_with_retry(label="command-sender")

        if self.is_cancelled():
            self.command_drone.close()
            self.command_drone = None
            return False

        self._heartbeat(self.command_drone)

        self.Manual     = ModeCommander(self.command_drone, self.state, self._mav_lock)
        self.Director   = DirectionCommander(self.command_drone, self.state, self._mav_lock)
        self.ManualCtrl = ManualController(self.command_drone, self.state, self._mav_lock)
        self._ato       = AutoTakeOff(self.command_drone, self._mav_lock)
        self.Payload    = Payload(state = self.state,command_drone =self.command_drone,lock=self._mav_lock  )

        # Analog handler wired to this sender so it can call
        # set_airspeed / turn_left / point_gimbal etc.
        from mavlink.uav_commads.analog_input_handler import AnalogInputHandler
        self._analog = AnalogInputHandler(self.state, self.ManualCtrl, self)

        self.state.update_UAV_Command_Connection(True, self.command_drone)
        logger.info("UAVCommandSender connected.")
        self._start_loop()
        return True

    def _do_disconnect(self):
        if self.ManualCtrl:
            self.ManualCtrl.stop_streaming()
            self.ManualCtrl = None
        if self._ato:
            self._ato.cancel()
            self._ato = None
        if self.command_drone:
            try:
                self.command_drone.close()
            except Exception:
                pass
            self.command_drone = None
        self.Manual   = None
        self.Director = None
        self._analog  = None
        self.state.update_UAV_Command_Connection(False, None)
        logger.info("UAVCommandSender disconnected.")

    def _pet_watchdog(self):
        if self.watchdog:
            self.watchdog.uavCommand_pet()

    def _start_watch(self):
        if self.watchdog:
            self.watchdog.watchCommandsThread()

    def _on_error(self, e):
        logger.error(f"UAVCommandSender loop error: {e}")
        self.state.update_UAV_Command_Connection(False, None)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _run_once(self):
        self.drain_input_commands()
        if self._analog:
            self._analog.process()

        # Execute everything currently queued, not just one item —
        # otherwise producers (discrete commands + gimbal rate) can
        # outpace a single-item drain and the queue backs up under load.
        while True:
            try:
                cmd, args, kwargs = self.command_queue.get_nowait()
            except queue.Empty:
                break
            cmd(*args, **kwargs)
            self.command_queue.task_done()

    def enqueue(self, cmd, *args, **kwargs):
        self.command_queue.put((cmd, args, kwargs))

    # ── Discrete command bus ──────────────────────────────────────────────────

    def drain_input_commands(self):
        while True:
            try:
                cmd = self.command_bus.get_nowait()
            except queue.Empty:
                break
            dispatch(cmd, self)

    def _cancel_takeoff_if_active(self):
        if self._ato:
            self._ato.cancel()
        self.takeoff_in_progress = False

    # ── Public command API ────────────────────────────────────────────────────

    def arm(self):
        if not self.state.is_UAV_Armed:
            self.enqueue(Arms(self.command_drone, self.state, self._mav_lock).initiate_Arm)
        else:
            logger.info("Already armed — skipping")

    def disarm(self):
        self.enqueue(Arms(self.command_drone, self.state, self._mav_lock).initiate_Disarming)

    def set_fbwa(self):
        if self.Manual:
            self.enqueue(self.Manual.initiate_FBWA_Mode)

    def set_fbwb(self):
        if self.Manual:
            self.enqueue(self._switch_to_fbwb)

    def _switch_to_fbwb(self):
        mode_mapping = self.command_drone.mode_mapping()
        if mode_mapping and "FBWB" in mode_mapping:
            with self._mav_lock:
                self.command_drone.mav.set_mode_send(
                    self.command_drone.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_mapping["FBWB"]
                )
            logger.info("Switched to FBWB — joystick active.")
            if self.ManualCtrl:
                self.ManualCtrl.start_streaming()
        else:
            logger.warning("FBWB not available — falling back to FBWA.")
            self.Manual.initiate_FBWA_Mode()
            if self.ManualCtrl:
                self.ManualCtrl.start_streaming()

    def set_guided(self):
        if self.Manual:
            self.enqueue(self.Manual.initiate_Guided_Mode)

    def payloadZoomIn(self):
        self.enqueue(self.Payload.initiate_ZoomIn_Raw)

    def payloadZoomOut(self):
        self.enqueue(self.Payload.initiate_ZoomOut_Raw)




    def set_auto(self):
        if self.Manual:
            self.enqueue(self.Manual.initiate_Auto_Mode)

    def set_rtl(self):
        if self.Manual:
            self.enqueue(self.Manual.initiate_RTL)

    def set_airspeed(self, speed_ms):
        self.enqueue(
            Speed_Controller(self.command_drone, self.state, self._mav_lock).set_airspeed,
            speed_ms
        )

    def turn_left(self, degrees=30):
        if self.Director:
            self.enqueue(self.Director.turn_left, degrees)

    def turn_right(self, degrees=30):
        if self.Director:
            self.enqueue(self.Director.turn_right, degrees)

    def set_altitude(self, target_alt_m):
        if self.Director:
            self.enqueue(self.Director.set_altitude, target_alt_m)

    def point_gimbal(self, pitch_deg, yaw_deg):
        """
        pitch_deg / yaw_deg here are RELATIVE deltas (degrees to move by
        this call), not absolute targets. Still available for one-shot
        nudges (e.g. a UI "+5deg" button) — AnalogInputHandler no longer
        uses this for continuous joystick control; see set_gimbal_rate
        for that.

        Fixed two bugs that were here before:
          1. initiate_PointAngle_Relative_Raw(azimuth_delta_deg, tilt_delta_deg)
             takes azimuth FIRST — this used to pass (pitch_deg, yaw_deg)
             positionally, putting pitch in the azimuth slot and yaw in
             the tilt slot.
          2. This used to build a throwaway `Payload(self.command_drone,
             self.state, self._mav_lock)` with positional args, instead of
             reusing self.Payload (built with keyword args in _do_connect).
             If Payload's real __init__ order doesn't match
             (command_drone, state, lock), state/command_drone get
             silently swapped in that one-off instance.
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
        it. This is what AnalogInputHandler now calls for joystick
        control, instead of repeatedly streaming small relative-angle
        deltas via point_gimbal — see AnalogInputHandler's docstring for
        why this is the better fit for continuous stick input.
        """
        if not self.Payload:
            logger.warning("set_gimbal_rate called with no Payload connection — ignoring.")
            return
        self.enqueue(
            self.Payload.initiate_ManualSpeed_Raw,
            yaw_deg_s,     # azimuth_vel_deg_s
            pitch_deg_s,   # tilt_vel_deg_s
        )

    # ── Zoom / focus — level-triggered raw start/stop ───────────────────────
    # Called by AnalogInputHandler on rising/falling edges of the held
    # zoom/focus flags. Per the ICD, zoom-in/out and focus+/- are "rising
    # edge valid" — one start command keeps the gimbal moving until a
    # matching stop is sent, so these are one-shot per press/release,
    # not something to call repeatedly while held.

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

    # ── Laser range polling — this is how you actually SEE it working ──────
    # laser_start()/continous_laser_start() only ever SEND commands to the
    # gimbal — nothing about sending a command tells you whether ranging
    # is actually happening or what the distance is. This runs
    # Payload.poll_status() on a background thread (poll_status, not
    # get_laser_range, so it doesn't re-trigger single-ranging and
    # potentially interrupt an active continuous session) and logs
    # whatever distance comes back. Runs on its own thread rather than
    # inline in _run_once() because poll_status() blocks for up to
    # response_timeout waiting on a SERIAL_CONTROL reply — doing that on
    # the main loop would stall gimbal/zoom/focus command processing for
    # up to a second on every single poll.

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

    def _send_distance_to_gcs(self, distance_m):
        """
        Relay the laser distance to any connected GCS (QGroundControl,
        Mission Planner, etc.) as a standard MAVLink message, so it's
        visible there without your Python app needing to do anything
        with the value itself. Uses NAMED_VALUE_FLOAT — the simplest
        path to "just show me a number": in QGC, Widgets -> Values ->
        add field -> "LaserRange". See _send_distance_sensor_to_gcs for
        the DISTANCE_SENSOR alternative.

        NOTE: whether this shows up attached to the SAME vehicle panel
        in QGC (rather than as a separate/unassociated system) can
        depend on whether this connection's system_id matches the
        vehicle's own. If it doesn't show where expected, check QGC's
        Analyze -> MAVLink Inspector first to confirm the message is
        arriving at all before troubleshooting display placement.
        """
        if not self.command_drone:
            return
        try:
            self.command_drone.mav.named_value_float_send(
                int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
                b"LaserRange",                          # name, max 10 chars
                float(distance_m),
            )
        except Exception as e:
            logger.warning(f"Failed to relay laser distance to GCS: {e}")

    def _send_distance_sensor_to_gcs(self, distance_m):
        """
        Alternative to _send_distance_to_gcs: sends the semantically
        "proper" DISTANCE_SENSOR message instead of a generic named
        value — more work to get right, potentially picked up by
        built-in rangefinder-aware displays instead of a manually added
        Values widget. Not currently called by the polling loop; swap
        the call in _laser_poll_loop if you want this instead.

        min_distance/max_distance below (cm) are placeholders — the ICD
        doesn't document this Viewpro unit's actual measurement range,
        adjust to the real spec if you know it. Both are uint16 fields
        (max 65535 cm = 655.35m) — max_distance is capped at that limit
        here; if your unit's actual max range is genuinely beyond ~655m,
        this field can't represent it and would need clamping anyway.
        orientation is set to NONE as a simplification: a gimbal-mounted
        laser's true pointing direction changes as the gimbal moves,
        which a fixed orientation value doesn't capture (MAVLink
        supports a quaternion-based ROTATION_CUSTOM orientation for
        exactly this case; not implemented here to keep this simple).
        """
        if not self.command_drone:
            return
        try:
            self.command_drone.mav.distance_sensor_send(
                int(time.time() * 1000) & 0xFFFFFFFF,      # time_boot_ms
                100,                                         # min_distance, cm — placeholder
                65535,                                        # max_distance, cm — placeholder, uint16 max
                min(int(distance_m * 100), 65535),           # current_distance, cm, clamped to uint16
                mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                0,                                            # id
                mavutil.mavlink.MAV_SENSOR_ROTATION_NONE,      # orientation — see docstring
                0,                                            # covariance, 0 = unknown
            )
        except Exception as e:
            logger.warning(f"Failed to relay laser distance sensor to GCS: {e}")

    def zoom_in_start(self, speed=4):
        if self.Payload:
            self.enqueue(self.Payload.initiate_ZoomIn_Raw, speed)

    def zoom_in_stop(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_ZoomStop_Raw)

    def zoom_out_start(self, speed=4):
        if self.Payload:
            self.enqueue(self.Payload.initiate_ZoomOut_Raw, speed)

    def zoom_out_stop(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_ZoomStop_Raw)

    def focus_plus_start(self, speed=4):
        if self.Payload:
            self.enqueue(self.Payload.initiate_FocusPlus_Raw, speed)  # was initiate_FOVPlus_Raw — that's zoom out, not focus

    def focus_plus_stop(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_FocusStop_Raw)

    def focus_minus_start(self, speed=4):
        if self.Payload:
            self.enqueue(self.Payload.initiate_FocusMinus_Raw, speed)  # was initiate_FOVMinus_Raw — that's zoom in, not focus

    def focus_minus_stop(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_FocusStop_Raw)  # was initiate_FOVMinus_Raw — a start command, so this never stopped anything

    # ── Object tracking ───────────────────────────────────────────────────
    # tracking_start locks the tracker onto whatever's currently under the
    # search cross — pair with a search/nudge step (not wired up here yet)
    # if you need to move the cross onto a target first. Note this only
    # engages the image tracker; the gimbal won't physically follow unless
    # A1 servo mode is also put into tracking mode (build_A1_tracking /
    # SERVO_TRACKING_MODE) — not wired here either, ask if you want it.

    def tracking_start(self):
        if self.Payload:
            self.enqueue(self.Payload.initiate_TrackingSetTemplateSize_Raw,128)
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

    # ── Video source toggle (BIT_VIDEO_IP) ──────────────────────────────────
    # Simple two-way EO1<->IR toggle. Expand _video_source_cycle if you
    # want PIP/EO2/Fusion included in the rotation too.

    _video_source_cycle = None  # set lazily on first use, avoids importing GimbalFrameBuilder at class-definition time

    def video_source_toggle(self):
        if not self.Payload:
            return
        if self._video_source_cycle is None:
            self._video_source_cycle = [GimbalFrameBuilder.VIDEO_EO1, GimbalFrameBuilder.VIDEO_IR_THERMAL]
            self._video_source_index = 0
        self._video_source_index = (self._video_source_index + 1) % len(self._video_source_cycle)
        source = self._video_source_cycle[self._video_source_index]
        self.enqueue(self.Payload.initiate_SwitchVideoSource_Raw, source)

    # ── IR polarity toggle (BIT_IR_POLARITY, "WHITE BTN") ───────────────────
    # White-hot <-> black-hot toggle, same lazy-cycle pattern as
    # video_source_toggle above.

    _ir_polarity_cycle = None

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

    # ── Laser: continuous mode (BIT_LASER_CONT_MODE) and single-shot trigger
    # (BIT_LASER_SINGLE_MODE) — separate from laser power (BIT_LASER_ON_OFF,
    # via laser_start/laser_stop above). Continuous mode reuses
    # continous_laser_start/stop; single-shot needs its own background
    # thread for the same reason start_laser_polling does — the query
    # blocks waiting for a reply, and firing it directly in a
    # register_handler (which runs on the main command loop) would stall
    # everything else for up to a second.

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
        result = self.Payload.get_laser_range(response_timeout=response_timeout)
        distance = result.get('laser_range_m')
        if distance is not None:
            logger.info(f"Laser single-shot range: {distance}m")
            self._send_distance_to_gcs(distance)
        elif 'error' in result:
            logger.debug(f"Laser single-shot: {result['error']}")

    def start_takeoff(self):
        if self.takeoff_in_progress:
            logger.info("Takeoff already in progress — ignoring duplicate request.")
            return
        self.takeoff_in_progress = True
        threading.Thread(
            target=self._run_takeoff, daemon=True, name="TakeoffSequence"
        ).start()

    def _run_takeoff(self):
        try:
            self._ato.initiate_takeoff()
        finally:
            self.takeoff_in_progress = False

    def emergency_stop(self):
        if self.state.is_flying:
            logger.critical("EMERGENCY: vehicle is flying — triggering RTL")
            self.set_rtl()
        else:
            logger.critical("EMERGENCY: vehicle on ground — disarming")
            self.disarm()


# ── Registered handlers ───────────────────────────────────────────────────────
'''
@register_handler(ArmCommand)
def _handle_arm(sender, cmd):
    logger.debug("ArmCommand received")
    sender.arm()'''


@register_handler(DisarmCommand)
def _handle_disarm(sender, cmd):
    logger.info("DisarmCommand received")
    sender.disarm()


@register_handler(TakeoffCommand)
def _handle_takeoff(sender, cmd):
    logger.debug("TakeoffCommand received")
    sender.start_takeoff() 

''''
@register_handler(ManualModeCommand)
def _handle_manual(sender, cmd):
    logger.debug("ManualModeCommand received")
    sender._cancel_takeoff_if_active()
    sender.set_fbwb()'''


@register_handler(RTLCommand)
def _handle_rtl(sender, cmd):
    logger.info("RTLCommand received")
    sender._cancel_takeoff_if_active()
    if sender.ManualCtrl:
        sender.ManualCtrl.stop_streaming()
    sender.set_rtl()


@register_handler(EmergencyCommand)
def _handle_emergency(sender, cmd):
    logger.warning("EmergencyCommand received")
    sender._cancel_takeoff_if_active()
    sender.emergency_stop()

@register_handler(AutoModeCommand)
def _handle_auto(sender, cmd):
    logger.debug("AutoModeCommand received")
    sender.set_auto()


@register_handler(LandCommand)
def _handle_land(sender, cmd):
    logger.debug("LandCommand received")



@register_handler(ContiousLaserStartCommand)
def _handle_continous_laser_start(sender, cmd):
    logger.info("ContiousLaserStartCommand received")
    sender.laser_start()
    sender.continous_laser_start()   # was continous_laser_stop() — sent stop immediately after power-on, so ranging never actually began
    sender.start_laser_polling()     # so the distance is actually visible somewhere — see start_laser_polling's docstring

@register_handler(ContiousLaserStopCommand)
def _handle_continous_laser_stop(sender, cmd):
    logger.info("ContiousLaserStopCommand received")
    sender.stop_laser_polling()
    sender.continous_laser_stop()
    sender.laser_stop()
    


@register_handler(SpeedDownCommand)
def _handle_speeddown(sender, cmd):
    logger.info("SpeedDownCommand received")


@register_handler(LaserStartCommand)
def _handle_laser_start(sender, cmd):
    logger.debug("LaserStartCommand received")
    sender.laser_start()


@register_handler(LaserStopCommand)
def _handle_laser_stop(sender, cmd):
    logger.debug("LaserStopCommand received")
    sender.laser_stop()


@register_handler(TrackingStartCommand)
def _handle_tracking_start(sender, cmd):
    logger.debug("TrackingStartCommand received")
    sender.tracking_start()


@register_handler(TrackingStopCommand)
def _handle_tracking_stop(sender, cmd):
    logger.debug("TrackingStopCommand received")
    sender.tracking_stop()


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


# "Image Sensor Change" (GREEN BTN, bit 7) reuses the same
# video_source_toggle() as VIDEO_IP (bit 21) — "image sensor change" is
# a strong naming match for switching between EO/IR sensors, which is
# exactly what video_source_toggle() does, and it's plausible this is
# actually THE real video-source button while VIDEO_IP(21) is something
# else not yet correctly identified. Flagging this rather than silently
# guessing — if VIDEO_IP turns out to mean something different (PIP?
# network target?), this wiring should stay; if it turns out
# VIDEO_IP was already correct and this is a distinct third thing,
# this handler will need to change to call something else instead.
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
    logger.info("CameraZoomInCommand received")
    sender.state.ZOOMIN_PRESSED = True

@register_handler(ZoomInFallCommand)
def _handle_zoominFall(sender, cmd):
    logger.info("CameraZoomInFallCommand received")
    sender.state.ZOOMIN_PRESSED = False

@register_handler(ZoomOutCommand)
def _handle_zoomout(sender, cmd):
    logger.info("CameraZoomOutCommand received")
    sender.state.ZOOMOUT_PRESSED = True

@register_handler(ZoomOutFallCommand)
def _handle_zoomoutFall(sender, cmd):
    logger.info("CameraZoomOutFallCommand received")
    sender.state.ZOOMOUT_PRESSED = False


# "Wide" was previously bridged to Focus here, based on an assumption
# made before main.c was available — turns out "Wide" is FOV terminology
# (the ICD itself names these commands FOV+/FOV-), and real focus now has
# its own dedicated bits (14/15, FocusPlusCommand/FocusMinusCommand
# above). Corrected: widein/wideout now drive FOV+/- directly, reusing
# the existing zoom_out_start/stop and zoom_in_start/stop — FOV+ IS zoom
# out and FOV- IS zoom in, per the ICD's own naming, not a separate
# control. FOCUSIN_PRESSED/FOCUSOUT_PRESSED are no longer set by
# anything as of this change — they're not removed from State, just
# dormant, since the dedicated Focus bits now drive focus_plus/minus
# directly rather than through that state-flag+polling path.

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