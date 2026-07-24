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

    def _send_raw_gimbal_frame(self, frame: bytes, expect_response=False):
        """
        Tunnel a raw Viewpro protocol frame to the gimbal through the FC's
        dedicated serial port, via MAVLink SERIAL_CONTROL. The FC does not
        parse or validate these bytes — it just writes them out the target
        UART, so this bypasses DO_MOUNT_*/camera-command semantics entirely.
        """
        if not self._check(self._requires_payload_connection):
            return False

        if len(frame) > 70:
            logger.error(f"Raw gimbal frame too long for SERIAL_CONTROL ({len(frame)} bytes, max 70).")
            return False

        flags = mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
        if expect_response:
            flags |= mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND

        self.state.mav_connection.mav.serial_control_send(
            device=self.VIEWPRO_SERIAL_DEVICE,
            flags=flags,
            timeout=0,
            baudrate=0,  # 0 = don't change the port's configured baud rate
            count=len(frame),
            data=bytes(frame).ljust(70, b'\x00'),
        )
        logger.info(f"Sent raw Viewpro frame ({len(frame)} bytes): {frame.hex(' ')}")
        return True

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

    def initiate_ZoomIn_Raw(self, speed=2, expect_response=False):
        """Start zooming in (telephoto). speed: 1 (slowest) - 7 (fastest)."""
        frame = self._gimbal_frames.build_C1_zoom_in(speed)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_ZoomOut_Raw(self, speed=2, expect_response=False):
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
        """Trigger a single laser rangefinder measurement."""
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