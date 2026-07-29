# mavlink/uav_commads/commands/payload_commands.py

from pymavlink import mavutil
from mavlink.uav_commads.commands.base_command import BaseCommand
from mavlink.uav_commads.commands.payload_services import GimbalFrameBuilder
import logging
logger = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────────
# Payload command wrapper
# ─────────────────────────────────────────────────────────────────────────

class Payload(BaseCommand):
    """
    MAVLink commands for controlling a gimbal/camera payload (e.g. Viewpro
    A20KTR) via the Gimbal Protocol v1 (DO_MOUNT_*) and standard camera
    commands. Angle control uses DO_MOUNT_CONTROL rather than the newer
    Gimbal Manager v2 messages, since that's what's broadly supported by
    ArduPilot's mount driver today. Swap in DO_GIMBAL_MANAGER_PITCHYAW if
    you're targeting a v2-only stack.

    Also supports Viewpro's native raw protocol (see GimbalFrameBuilder,
    in payload_services.py) for cases where DO_MOUNT_* doesn't cover a
    feature the payload exposes natively. Raw frames are tunneled to the
    gimbal's dedicated serial port via MAVLink SERIAL_CONTROL — the FC
    does not interpret these bytes.
    """

    # Device the gimbal is wired to, per MAVLink's SERIAL_CONTROL_DEV enum.
    # NOTE: this enum is a small FIXED set (TELEM1, TELEM2, GPS1, GPS2, SHELL,
    # USB1 depending on dialect version) — it is NOT an arbitrary SERIALx
    # port number. Confirm which enum value ArduPilot maps to the physical
    # SERIALx port your Viewpro is wired to before relying on this value.
    VIEWPRO_SERIAL_DEVICE = mavutil.mavlink.SERIAL_CONTROL_DEV_TELEM2

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._gimbal_frames = GimbalFrameBuilder()

    # ── Precondition guard specific to payload commands ────────────────────

    def _requires_payload_connection(self):
        if not self.state.is_UAV_Command_Connection_Available:
            logger.warning(f"{self.__class__.__name__}: blocked — no MAVLink command connection.")
            return False
        return True

    # ── Mount mode / pointing (standard MAVLink DO_MOUNT_*) ─────────────────

    def initiate_Retract(self):
        """Stow the gimbal to its retracted position."""
        if not self._check(self._requires_payload_connection):
            return False

        logger.info("Retracting gimbal...")
        self._send_command(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
            p1=mavutil.mavlink.MAV_MOUNT_MODE_RETRACT
        )
        logger.info("Retract command sent.")
        return True

    def initiate_Neutral(self):
        """Move the gimbal to its neutral (level, forward-facing) position."""
        if not self._check(self._requires_payload_connection):
            return False

        logger.info("Setting gimbal to neutral...")
        self._send_command(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE,
            p1=mavutil.mavlink.MAV_MOUNT_MODE_NEUTRAL
        )
        logger.info("Neutral command sent.")
        return True

    def initiate_PointAngle(self, pitch_deg, yaw_deg, roll_deg=0):
        """
        Point the gimbal at a fixed pitch/roll/yaw (degrees) via standard
        MAVLink DO_MOUNT_CONTROL. Yaw is vehicle-relative: 0 = forward,
        90 = right, -90 = left.
        """
        if not self._check(self._requires_payload_connection):
            return False

        logger.info(f"Pointing gimbal — pitch={pitch_deg}, roll={roll_deg}, yaw={yaw_deg}...")
        self._send_command(
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            p1=pitch_deg,
            p2=roll_deg,
            p3=yaw_deg,
            p7=mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING
        )
        logger.info("Point-angle command sent.")
        return True

    def initiate_PointROI(self, lat, lon, alt):
        """Point the gimbal at a fixed GPS location (region of interest)."""
        if not self._check(self._requires_payload_connection):
            return False

        logger.info(f"Setting gimbal ROI — lat={lat}, lon={lon}, alt={alt}...")
        self._send_command(
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            p5=lat,
            p6=lon,
            p7=alt
        )
        logger.info("ROI command sent.")
        return True

    def initiate_StopROI(self):
        """Cancel ROI tracking and return to the previous mount mode."""
        if not self._check(self._requires_payload_connection):
            return False

        logger.info("Clearing gimbal ROI...")
        self._send_command(mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE)
        logger.info("Stop-ROI command sent.")
        return True

    # ── Camera capture ──────────────────────────────────────────────────────

    def initiate_TakePhoto(self, interval=0, count=1):
        """Trigger a single photo, or a timed burst if interval/count are set."""
        if not self._check(self._requires_payload_connection):
            return False

        logger.info(f"Capturing photo — interval={interval}, count={count}...")
        self._send_command(
            mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
            p2=interval,
            p3=count
        )
        logger.info("Take-photo command sent.")
        return True

    def initiate_StartRecording(self):
        """Start video recording."""
        if not self._check(self._requires_payload_connection):
            return False

        logger.info("Starting video recording...")
        self._send_command(mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE)
        self.state.update_UAV_Recording_Status(True)
        logger.info("Start-recording command sent.")
        return True

    def initiate_StopRecording(self):
        """Stop video recording."""
        if not self._check(self._requires_payload_connection):
            return False

        logger.info("Stopping video recording...")
        self._send_command(mavutil.mavlink.MAV_CMD_VIDEO_STOP_CAPTURE)
        self.state.update_UAV_Recording_Status(False)
        logger.info("Stop-recording command sent.")
        return True

    # ── Zoom / focus (MAVLink-native, absolute level) ───────────────────────

    def initiate_SetZoom(self, zoom_level):
        """
        Set absolute zoom level. zoom_level meaning is camera-dependent —
        for Viewpro this is typically a 0-100 range mapped to optical zoom.
        """
        if not self._check(self._requires_payload_connection):
            return False

        logger.info(f"Setting zoom to {zoom_level}...")
        self._send_command(
            mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM,
            p1=mavutil.mavlink.ZOOM_TYPE_RANGE,
            p2=zoom_level
        )
        logger.info("Set-zoom command sent.")
        return True

    def initiate_SetFocus(self, focus_type=mavutil.mavlink.FOCUS_TYPE_AUTO, focus_value=0):
        """Set camera focus. Defaults to autofocus; pass FOCUS_TYPE_RANGE + a value for manual."""
        if not self._check(self._requires_payload_connection):
            return False

        logger.info(f"Setting focus — type={focus_type}, value={focus_value}...")
        self._send_command(
            mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS,
            p1=focus_type,
            p2=focus_value
        )
        logger.info("Set-focus command sent.")
        return True

    # ── AI object tracking ──────────────────────────────────────────────────

    def initiate_TrackPoint(self, x, y):
        """
        Start AI tracking of the target under normalized image coordinates
        (x, y in [0, 1], origin top-left).
        """
        if not self._check(self._requires_payload_connection):
            return False

        logger.info(f"Starting track-point at ({x}, {y})...")
        self._send_command(
            mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT,
            p1=x,
            p2=y,
            p3=0  # radius, unused for point tracking
        )
        logger.info("Track-point command sent.")
        return True

    def initiate_StopTracking(self):
        """Cancel AI object tracking."""
        if not self._check(self._requires_payload_connection):
            return False

        logger.info("Stopping tracking...")
        self._send_command(mavutil.mavlink.MAV_CMD_CAMERA_STOP_TRACKING)
        logger.info("Stop-tracking command sent.")
        return True

    # ── Raw Viewpro protocol passthrough (bypasses DO_MOUNT_*) ─────────────

    def _send_raw_gimbal_frame(self, frame: bytes, expect_response=False, response_timeout=1.0):
        """
        Tunnel a raw Viewpro protocol frame to the gimbal through the FC's
        dedicated serial port, via MAVLink SERIAL_CONTROL. The FC does not
        parse or validate these bytes — it just writes them out the target
        UART, so this bypasses DO_MOUNT_*/camera-command semantics entirely.

        Returns:
          - True  : sent OK, expect_response=False (fire-and-forget)
          - False : failed precondition or frame too long
          - bytes : the raw reply payload, if expect_response=True and a
                     SERIAL_CONTROL reply arrived within response_timeout
          - None  : expect_response=True but no reply arrived in time
        """
        if not self._check(self._requires_payload_connection):
            return False

        if len(frame) > 70:
            logger.error(f"Raw gimbal frame too long for SERIAL_CONTROL ({len(frame)} bytes, max 70).")
            return False

        flags = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
        if expect_response:
            flags |= mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND

        # Locked: this can now be called from a background polling thread
        # (see UAVCommandSender.start_laser_polling) concurrently with the
        # main command-processing thread's own sends on the same
        # connection — pymavlink connections aren't safe for unsynchronized
        # concurrent send/recv, and the rest of this codebase already
        # locks its other MAVLink sends (see _switch_to_fbwb).
        lock = getattr(self, 'lock', None)
        if lock:
            with lock:
                reply_bytes = self._do_send_and_maybe_recv(frame, flags, expect_response, response_timeout)
        else:
            reply_bytes = self._do_send_and_maybe_recv(frame, flags, expect_response, response_timeout)
        return reply_bytes

    def _do_send_and_maybe_recv(self, frame, flags, expect_response, response_timeout):
        self.command_drone.mav.serial_control_send(
            device=self.VIEWPRO_SERIAL_DEVICE,
            flags=flags,
            timeout=0,
            baudrate=0,  # 0 = don't change the port's configured baud rate
            count=len(frame),
            data=bytes(frame).ljust(70, b'\x00'),
        )
        logger.info(f"Sent raw Viewpro frame ({len(frame)} bytes): {frame.hex(' ')}")

        if not expect_response:
            return True

        reply = self.command_drone.recv_match(
            type='SERIAL_CONTROL', blocking=True, timeout=response_timeout
        )
        if reply is None:
            logger.warning(f"No SERIAL_CONTROL reply within {response_timeout}s.")
            return None

        reply_bytes = bytes(reply.data[:reply.count])
        logger.info(f"Received raw reply ({reply.count} bytes): {reply_bytes.hex(' ')}")
        return reply_bytes

    def initiate_PointAngle_Raw(self, azimuth_deg, tilt_deg, expect_response=False):
        """
        Point the gimbal using Viewpro's native Absolute Angle Mode (0x0B),
        bypassing MAV_CMD_DO_MOUNT_CONTROL entirely. Azimuth/tilt are
        relative to the gimbal's home position (home = 0°).
        """
        frame = self._gimbal_frames.build_A1_absolute_angle(azimuth_deg, tilt_deg)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_PointAngle_Relative_Raw(self, azimuth_delta_deg, tilt_delta_deg,
                                          azimuth_speed_deg_s=0, tilt_speed_deg_s=0,
                                          expect_response=False):
        """
        Nudge the gimbal by a relative angle offset, using the native
        protocol (Relative Angle Mode 0x09). Speeds default to 0 (gimbal's
        own default rate) — pass explicit deg/s values if you want to
        control rate directly through the payload's own velocity fields
        instead of (or in addition to) the per-tick delta size.
        """
        frame = self._gimbal_frames.build_A1_relative_angle(
            azimuth_delta_deg, tilt_delta_deg, azimuth_speed_deg_s, tilt_speed_deg_s
        )
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_MotorPower_Raw(self, on: bool, expect_response=False):
        """Turn the gimbal servo motor on/off, using the native protocol."""
        frame = self._gimbal_frames.build_A1_motor(on)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_ManualSpeed_Raw(self, azimuth_vel_deg_s=0.0, tilt_vel_deg_s=0.0, expect_response=False):
        """
        Set the gimbal moving at a continuous azimuth/tilt rate (Manual
        Speed Mode 0x01) — send once when the desired rate changes, the
        gimbal keeps moving on its own until the next call. Pass (0, 0)
        to stop. This is the preferred way to drive the gimbal from a
        joystick — see AnalogInputHandler, which sends this on rate
        change plus a periodic keepalive, rather than repeatedly
        streaming small relative-angle deltas.
        """
        frame = self._gimbal_frames.build_A1_manual_speed(azimuth_vel_deg_s, tilt_vel_deg_s)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_Home_Raw(self, expect_response=False):
        """Drive the gimbal to its home position, using the native protocol."""
        frame = self._gimbal_frames.build_A1_home()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    # ── Raw zoom / focus / video source / photo / record / LRF (C1) ────────
    # NOTE: zoom and focus here are RATE commands (start moving at `speed`,
    # then call the matching *_Stop_Raw), not absolute levels like the
    # MAVLink-native initiate_SetZoom/initiate_SetFocus above. Per ICD 2.3(b),
    # these run on edge-trigger logic — once started they keep going until a
    # genuinely different enumeration (stop/no-action) arrives. Don't mix
    # this raw path with the MAVLink-native path for the same axis at the
    # same time — same shared-control-conflict concern as DO_MOUNT_* vs raw
    # angle commands.

    def initiate_ZoomIn_Raw(self, speed=4, expect_response=False):
        """Start zooming in (telephoto). speed: 1 (slowest) - 7 (fastest)."""
        frame = self._gimbal_frames.build_C1_zoom_in(speed)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_ZoomOut_Raw(self, speed=4, expect_response=False):
        """Start zooming out (wide). speed: 1 (slowest) - 7 (fastest)."""
        frame = self._gimbal_frames.build_C1_zoom_out(speed)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_ZoomStop_Raw(self, expect_response=False):
        """Stop an in-progress zoom move."""
        frame = self._gimbal_frames.build_C1_zoom_stop()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_FocusPlus_Raw(self, speed=4, expect_response=False):
        """Start focusing far. speed: 1 (slowest) - 7 (fastest)."""
        frame = self._gimbal_frames.build_C1_focus_plus(speed)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_FocusMinus_Raw(self, speed=4, expect_response=False):
        """Start focusing near. speed: 1 (slowest) - 7 (fastest)."""
        frame = self._gimbal_frames.build_C1_focus_minus(speed)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_FocusStop_Raw(self, expect_response=False):
        """Stop an in-progress focus move."""
        frame = self._gimbal_frames.build_C1_focus_stop()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_AutoFocus_Raw(self, expect_response=False):
        """Switch the camera to autofocus mode."""
        frame = self._gimbal_frames.build_C1_auto_focus()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_ManualFocus_Raw(self, expect_response=False):
        """Switch the camera to manual focus mode."""
        frame = self._gimbal_frames.build_C1_manual_focus()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_SwitchVideoSource_Raw(self, source: int, expect_response=False):
        """
        Switch the active video source — this is the IR/EO switch. Use
        GimbalFrameBuilder.VIDEO_EO1 / VIDEO_IR_THERMAL / VIDEO_EO1_IR_PIP /
        VIDEO_IR_EO1_PIP / VIDEO_EO2 / VIDEO_FUSION as `source`.
        """
        frame = self._gimbal_frames.build_C1_switch_video_source(source)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_TakePhoto_Raw(self, expect_response=False):
        """Trigger a single photo capture, using the native protocol."""
        frame = self._gimbal_frames.build_C1_take_picture()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_StartRecording_Raw(self, expect_response=False):
        """Start video recording, using the native protocol."""
        frame = self._gimbal_frames.build_C1_start_record()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_StopRecording_Raw(self, expect_response=False):
        """Stop video recording, using the native protocol."""
        frame = self._gimbal_frames.build_C1_stop_record()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_IRPolarityWhiteHot_Raw(self, expect_response=False):
        """Set IR palette polarity to white-hot."""
        frame = self._gimbal_frames.build_C1_polarity_white_hot()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_IRPolarityBlackHot_Raw(self, expect_response=False):
        """Set IR palette polarity to black-hot."""
        frame = self._gimbal_frames.build_C1_polarity_black_hot()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_LaserRangeSingle_Raw(self, expect_response=False):
        """Trigger a single laser rangefinder measurement. Use get_laser_range() instead if you want the distance back directly."""
        frame = self._gimbal_frames.build_C1_laser_single_range()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_LaserRangeContinuousStart_Raw(self, expect_response=False):
        """Start continuous laser rangefinding."""
        frame = self._gimbal_frames.build_C1_laser_continuous_start()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_LaserRangeStop_Raw(self, expect_response=False):
        """Stop laser rangefinding."""
        frame = self._gimbal_frames.build_C1_laser_stop()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def get_laser_range(self, response_timeout=1.0):
        """
        Trigger a single laser rangefinder measurement AND read the
        distance back, in one call. This is the piece that was completely
        missing before — initiate_LaserRangeSingle_Raw() only ever sent
        the trigger; nothing captured or parsed a reply.

        Returns a dict from GimbalFrameBuilder.parse_reply_frame(), e.g.
        {'cmd_id': 0x40, 'checksum_ok': True, 'laser_range_m': 142,
         'laser_range_new': True}. 'laser_range_m' is None if the reading
        is invalid (raw value 0) or if the payload replied with something
        other than the T1+F1+B1+D1 status frame. Returns
        {'error': ...} if nothing came back — see the error string for
        why (e.g. 'no reply' vs a parse failure).
        """
        frame = self._gimbal_frames.build_C1_laser_single_range()
        reply = self._send_raw_gimbal_frame(frame, expect_response=True, response_timeout=response_timeout)

        if reply is None:
            return {'error': 'no reply received within timeout'}
        if reply is False:
            return {'error': 'send failed (no payload connection or frame too long)'}

        # parse_status_from_buffer scans for a valid frame anywhere in
        # the captured bytes, rather than assuming reply[0:3] is the
        # header — see its docstring for why that assumption fails
        # against a real SERIAL_CONTROL capture.
        parsed = self._gimbal_frames.parse_status_from_buffer(reply)
        if parsed.get('cmd_id') != 0x40:
            parsed.setdefault('error', f"reply was not a T1+F1+B1+D1 status frame (cmd_id={parsed.get('cmd_id')})")
        return parsed

    def poll_status(self, response_timeout=1.0):
        """
        Send a no-op combined A1+C1+E1 frame (servo=no-change, no C1/E1
        action) purely to solicit a T1+F1+B1+D1 status reply — unlike
        get_laser_range(), this does NOT send a new single-ranging
        trigger, so it's safe to call repeatedly (e.g. from a polling
        loop) without repeatedly re-triggering or interrupting an
        already-active continuous ranging session. Returns the same dict
        shape as get_laser_range() — check 'laser_range_m' for the
        latest reading a continuous session has produced.
        """
        frame = self._gimbal_frames.build_combined_A1_C1_E1()  # all defaults = no-op
        reply = self._send_raw_gimbal_frame(frame, expect_response=True, response_timeout=response_timeout)

        if reply is None:
            return {'error': 'no reply received within timeout'}
        if reply is False:
            return {'error': 'send failed (no payload connection or frame too long)'}

        parsed = self._gimbal_frames.parse_status_from_buffer(reply)
        if parsed.get('cmd_id') != 0x40:
            parsed.setdefault('error', f"reply was not a T1+F1+B1+D1 status frame (cmd_id={parsed.get('cmd_id')})")
        return parsed

    # ── C2: laser power / laser-own-zoom / sync mode ────────────────────────
    # Separate from the C1 ranging trigger above — this controls whether the
    # laser module itself is powered on at all. If ranging triggers appear
    # to do nothing, try initiate_LaserPowerOn_Raw() first; some firmware
    # gates the rangefinder on this explicit enable.

    def initiate_LaserPowerOn_Raw(self, expect_response=False):
        """Turn the laser rangefinder module on (ICD 3.8.1.3, C2 command 0x74)."""
        frame = self._gimbal_frames.build_C2_laser_on()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_LaserPowerOff_Raw(self, expect_response=False):
        """Turn the laser rangefinder module off."""
        frame = self._gimbal_frames.build_C2_laser_off()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_LaserPowerOn_Alt_Raw(self, expect_response=False):
        """
        Alternate laser power-on via the Power Control command (0x75)
        instead of 0x74. Try this if initiate_LaserPowerOn_Raw() doesn't
        actually enable the laser on your hardware.
        """
        frame = self._gimbal_frames.build_C2_laser_power(True)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_LaserPowerOff_Alt_Raw(self, expect_response=False):
        """Alternate laser power-off via the Power Control command (0x75)."""
        frame = self._gimbal_frames.build_C2_laser_power(False)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_LaserZoomIn_Raw(self, expect_response=False):
        """Laser's own zoom in (beam divergence) — separate from EO/IR optical zoom."""
        frame = self._gimbal_frames.build_C2_laser_zoom_in()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_LaserZoomOut_Raw(self, expect_response=False):
        """Laser's own zoom out (beam divergence) — separate from EO/IR optical zoom."""
        frame = self._gimbal_frames.build_C2_laser_zoom_out()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_LaserSyncEOMode_Raw(self, expect_response=False):
        """Laser zoom auto-syncs with the EO camera's zoom level."""
        frame = self._gimbal_frames.build_C2_laser_sync_eo_mode()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_LaserManualMode_Raw(self, expect_response=False):
        """Laser zoom under independent manual control (not synced to EO zoom)."""
        frame = self._gimbal_frames.build_C2_laser_manual_mode()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    # ── FOV+/- ────────────────────────────────────────────────────────────
    # ICD naming for the same rate-based zoom commands above — FOV+ =
    # zoom out, FOV- = zoom in. Same "rising edge valid" behavior: call
    # the matching Stop (initiate_ZoomStop_Raw, shared with focus) to halt.

    def initiate_FOVPlus_Raw(self, speed=4, expect_response=False):
        """FOV+ (ICD naming) — zoom out (wide). Same as initiate_ZoomOut_Raw."""
        frame = self._gimbal_frames.build_C1_fov_plus(speed)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_FOVMinus_Raw(self, speed=4, expect_response=False):
        """FOV- (ICD naming) — zoom in (telephoto). Same as initiate_ZoomIn_Raw."""
        frame = self._gimbal_frames.build_C1_fov_minus(speed)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_SetFOV_Raw(self, zoom_times, expect_response=False):
        """
        Directly set an absolute zoom level (e.g. 20.0 for 20x) instead
        of ramping via FOV+/-'s rate control. Verified byte-for-byte
        against the ICD's own worked example.
        """
        frame = self._gimbal_frames.build_C2_set_eo_zoom(zoom_times)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    # ── Picture / record mode, SD card ───────────────────────────────────

    def initiate_PictureMode_Raw(self, expect_response=False):
        """Switch the camera to picture (still-photo) mode."""
        frame = self._gimbal_frames.build_C1_picture_mode()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_RecordMode_Raw(self, expect_response=False):
        """Switch the camera to video-record mode."""
        frame = self._gimbal_frames.build_C1_record_mode()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_PicRecordSwitch_Raw(self, expect_response=False):
        """Toggle between picture and record mode."""
        frame = self._gimbal_frames.build_C1_pic_record_switch()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_FormatSD_Raw(self, expect_response=False):
        """DESTRUCTIVE — formats the SD card, erasing all stored photos/video. No confirmation step here; add one at the call site."""
        frame = self._gimbal_frames.build_C1_format_sd()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_QuerySDStatus_Raw(self, expect_response=True):
        """Query SD card status. Reply parsing isn't implemented yet — see build_C1_query_sd_status's note."""
        frame = self._gimbal_frames.build_C1_query_sd_status()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_QuerySDTotal_Raw(self, expect_response=True):
        """Query SD card total capacity. Reply parsing isn't implemented yet."""
        frame = self._gimbal_frames.build_C1_query_sd_total()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_QuerySDFree_Raw(self, expect_response=True):
        """Query SD card free capacity. Reply parsing isn't implemented yet."""
        frame = self._gimbal_frames.build_C1_query_sd_free()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    # ── Object tracking (E1 common / E2 infrequent) ─────────────────────────
    # Two layers: E1 controls the image tracker itself (find/lock/follow a
    # target in the video); separately, put the gimbal's A1 servo mode into
    # SERVO_TRACKING_MODE (initiate_MotorPower_Raw doesn't do this —
    # build_A1_tracking / servo_mode=0x06 does) for the physical gimbal to
    # actually chase wherever the tracker says the target is. Locking the
    # tracker on without also engaging servo tracking mode won't move the
    # gimbal.

    def initiate_TrackingStop_Raw(self, expect_response=False):
        """Stop tracking."""
        frame = self._gimbal_frames.build_E1_stop_tracking()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_TrackingSearch_Raw(self, azimuth_nudge=0, tilt_nudge=0, expect_response=False):
        """
        Bring up the search cross and nudge it. azimuth_nudge/tilt_nudge
        in [-15, 15] — see GimbalFrameBuilder.build_E1_search for the
        wire-format mapping.
        """
        frame = self._gimbal_frames.build_E1_search(azimuth_nudge, tilt_nudge)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_TrackingTurnOn_Raw(self, expect_response=False):
        """Lock onto whatever's currently under the search cross.""" 
        frame = self._gimbal_frames.build_E1_turn_on_tracking()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_TrackingSwitchToCross_Raw(self, expect_response=False):
        """Switch the active tracking point to the cross position."""
        frame = self._gimbal_frames.build_E1_switch_to_cross()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_TrackingAIOn_Raw(self, expect_response=False):
        """Turn AI-assisted recognition on."""
        frame = self._gimbal_frames.build_E1_ai_on()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_TrackingAIOff_Raw(self, expect_response=False):
        """Turn AI-assisted recognition off."""
        frame = self._gimbal_frames.build_E1_ai_off()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_TrackingAIAutoTrack_Raw(self, expect_response=False):
        """Auto-track whatever the AI recognizes, without manually placing the cross first."""
        frame = self._gimbal_frames.build_E1_ai_auto_track()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)
  
    def initiate_TrackingSetTemplateSize_Raw(self, template: int, expect_response=False):
        """template: one of GimbalFrameBuilder.E1_TEMPLATE_* (32x32 small, 64x64 medium, 128x128 big, or self-adapting combinations)."""
        frame = self._gimbal_frames.build_E1_set_template_size(template)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_TrackingSetVelocity_Raw(self, deg_per_sec: float, expect_response=False):
        """Set the gimbal's absolute tracking (chase) speed. 0 = system default."""
        frame = self._gimbal_frames.build_E1_set_tracking_velocity(deg_per_sec)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_TrackingSetVelocityCoefficient_Raw(self, coefficient: float, expect_response=False):
        """Scale the default tracking speed by a multiplier instead of setting an absolute value. 0 = system default."""
        frame = self._gimbal_frames.build_E1_set_tracking_velocity_coefficient(coefficient)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_TrackPointToPixel_Raw(self, x_pixel: int, y_pixel: int, expect_response=False):
        """Move the tracking point directly to a pixel coordinate — the precise alternative to initiate_TrackingSearch_Raw's cross-nudging."""
        frame = self._gimbal_frames.build_E2_track_point_to_pixel(x_pixel, y_pixel)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_TrackRectAreaPoint1_Raw(self, x_pixel: int, y_pixel: int, expect_response=False):
        """First corner of a rectangular tracking region — send both corners to define a box around a target."""
        frame = self._gimbal_frames.build_E2_rect_area_point1(x_pixel, y_pixel)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_TrackRectAreaPoint2_Raw(self, x_pixel: int, y_pixel: int, expect_response=False):
        """Second (opposite) corner of a rectangular tracking region."""
        frame = self._gimbal_frames.build_E2_rect_area_point2(x_pixel, y_pixel)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_TrackingShowCross_Raw(self, show=True, expect_response=False):
        """Show/hide the center targeting cross overlay."""
        frame = self._gimbal_frames.build_E2_display_cross(show)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_TrackingShowGimbalAngle_Raw(self, show=True, expect_response=False):
        """Show/hide the gimbal angle overlay."""
        frame = self._gimbal_frames.build_E2_gimbal_angle_display(show)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def get_tracking_status(self, response_timeout=1.0):
        """
        Poll the gimbal and return the current tracking sensor + state
        ('stop'/'search'/'tracking'/'lost'), read from the F1 field of
        the T1+F1+B1+D1 status reply — the same reply the laser distance
        comes from, so this is the same dict shape as
        get_laser_range()/poll_status() (which also now includes
        'tracking_sensor'/'tracking_status' — call whichever you already
        have a result from rather than polling twice).
        """
        return self.poll_status(response_timeout=response_timeout)