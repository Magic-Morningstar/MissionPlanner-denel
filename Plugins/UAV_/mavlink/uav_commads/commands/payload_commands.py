# mavlink/uav_commads/payload_commands.py

from pymavlink import mavutil
from mavlink.uav_commads.commands.base_command import BaseCommand
from mavlink.uav_commads.commands.payload_services import GimbalFrameBuilder
import logging
logger = logging.getLogger(__name__)





class Payload(BaseCommand):
    """
    MAVLink commands for controlling a gimbal/camera payload (e.g. Viewpro
    A20KTR) via the Gimbal Protocol v1 (DO_MOUNT_*) and standard camera
    commands. Angle control uses DO_MOUNT_CONTROL rather than the newer
    Gimbal Manager v2 messages, since that's what's broadly supported by
    ArduPilot's mount driver today. Swap in DO_GIMBAL_MANAGER_PITCHYAW if
    you're targeting a v2-only stack.

    Also supports Viewpro's native raw protocol (see GimbalFrameBuilder)
    for cases where DO_MOUNT_* doesn't cover a feature the payload exposes
    natively. Raw frames are tunneled to the gimbal's dedicated serial port
    via MAVLink SERIAL_CONTROL — the FC does not interpret these bytes.
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

    # ── Zoom / focus ─────────────────────────────────────────────────────────

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
        
        self.command_drone.mav.serial_control_send(
            device=self.VIEWPRO_SERIAL_DEVICE,
            flags=flags,
            timeout=0,
            baudrate=0,  
            count=len(frame),
            data=bytes(frame).ljust(70, b'\x00')
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

    def initiate_PointAngle_Relative_Raw(self, azimuth_delta_deg, tilt_delta_deg, expect_response=False):
        """Nudge the gimbal by a relative angle offset, using the native protocol."""
        frame = self._gimbal_frames.build_A1_relative_angle(azimuth_delta_deg, tilt_delta_deg)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_MotorPower_Raw(self, on: bool, expect_response=False):
        """Turn the gimbal servo motor on/off, using the native protocol."""
        frame = self._gimbal_frames.build_A1_motor(on)
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)

    def initiate_Home_Raw(self, expect_response=False):
        """Drive the gimbal to its home position, using the native protocol."""
        frame = self._gimbal_frames.build_A1_home()
        return self._send_raw_gimbal_frame(frame, expect_response=expect_response)