# mavlink/uav_commads/payload_commands.py

from pymavlink import mavutil
from mavlink.uav_commads.commands.base_command import BaseCommand
import logging
logger = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────────
# Raw Viewpro gimbal protocol (native frame format, sent as opaque bytes
# tunneled through the FC via MAVLink SERIAL_CONTROL on a dedicated port).
# ─────────────────────────────────────────────────────────────────────────

class GimbalFrameBuilder:
    """
    Builds raw Viewpro gimbal protocol frames (big-endian), per the payload's
    native ICD. These bypass MAV_CMD_DO_MOUNT_* / DO_GIMBAL_MANAGER_* entirely
    — the FC does not parse them, it just forwards the bytes out a dedicated
    UART to the gimbal.

    Frame layout:
        byte0-2   header          0x55 0xAA 0xDC
        byte3     body length + frame counter
                    bit0-5: n = byte count from byte3 through checksum, inclusive
                    bit6-7: rolling frame counter (0-3)
        byte4     CMD_ID_SET (frame ID — e.g. 0x1A for a standalone A1 packet)
        byte5..   DATA (N bytes, meaning depends on CMD_ID)
        last byte CHECKSUM = XOR of byte3 .. last data byte (checksum excluded)
    """

    HEADER = bytes([0x55, 0xAA, 0xDC])

    # A1 "Servo Control" sub-modes (A1 byte1, bits 0-3) — from ICD table 3.3
    SERVO_MOTOR_ON_OFF          = 0x00
    SERVO_MANUAL_SPEED          = 0x01
    SERVO_FOLLOW_YAW            = 0x03
    SERVO_HOME_POSITION         = 0x04
    SERVO_TRACKING_MODE         = 0x06
    SERVO_MANUAL_RELATIVE_ANGLE = 0x09
    SERVO_FOLLOW_YAW_DISABLE    = 0x0A
    SERVO_MANUAL_ABSOLUTE_ANGLE = 0x0B  # home position treated as 0°
    SERVO_MANUAL_RC_MODE        = 0x0D
    SERVO_NO_CHANGE             = 0x0F  # keep current state, params ignored

    # Frame IDs, from the "Data Packet Composition" table
    FRAME_ID_A1       = 0x1A  # A1 alone (9 bytes of data)
    FRAME_ID_C1       = 0x1C  # C1 alone
    FRAME_ID_E1       = 0x1E  # E1 alone
    FRAME_ID_A1_C1_E1 = 0x30  # combined A1+C1+E1

    def __init__(self):
        self._frame_counter = 0

    # ── low-level helpers ───────────────────────────────────────────────

    @staticmethod
    def _angle_to_word(angle_deg):
        """Signed 16-bit word. 1 LSB = 360/65536 degrees (per ICD 3.3.1.5)."""
        return int(round((angle_deg / 360.0) * 65536)) & 0xFFFF

    @staticmethod
    def _word_to_bytes(word):
        """Big-endian 2-byte encoding (MSB first) — ICD specifies big-endian."""
        return bytes([(word >> 8) & 0xFF, word & 0xFF])

    @staticmethod
    def _checksum(payload_bytes):
        """XOR of every byte passed in (byte3 through the last data byte)."""
        cs = 0
        for b in payload_bytes:
            cs ^= b
        return cs

    def _next_frame_counter(self):
        fc = self._frame_counter & 0x03
        self._frame_counter = (self._frame_counter + 1) & 0x03
        return fc

    def _build(self, cmd_id: int, data: bytes) -> bytes:
        body = bytes([cmd_id]) + data
        n = len(body) + 2  # +1 for byte3 itself, +1 for checksum byte
        if n > 0x3F:
            raise ValueError(f"Frame body too large ({n} bytes, protocol max is 63)")
        byte3 = (n & 0x3F) | (self._next_frame_counter() << 6)
        checksum_input = bytes([byte3]) + body
        checksum = self._checksum(checksum_input)
        return self.HEADER + checksum_input + bytes([checksum])

    # ── A1: servo control (mount angle / mode) ──────────────────────────

    def build_A1(self, servo_mode: int, param1=0, param2=0, param3=0, param4=0) -> bytes:
        """
        Standalone A1 servo-control frame (frame ID 0x1A). param1-4 are raw
        16-bit words; their meaning depends on servo_mode (see ICD 3.3.1.x).
        """
        byte1 = servo_mode & 0x0F  # bits 4-7 reserved, left as 0
        data = bytes([byte1])
        for p in (param1, param2, param3, param4):
            data += self._word_to_bytes(p & 0xFFFF)
        return self._build(self.FRAME_ID_A1, data)

    def build_A1_absolute_angle(self, azimuth_deg, tilt_deg) -> bytes:
        """
        A1 frame in Absolute Angle Mode (0x0B) — ICD 3.3.1.5. Points the
        gimbal to a fixed azimuth/tilt relative to home position (home = 0).
        param3/param4 are meaningless in this mode, left as 0.
        """
        return self.build_A1(
            servo_mode=self.SERVO_MANUAL_ABSOLUTE_ANGLE,
            param1=self._angle_to_word(azimuth_deg),
            param2=self._angle_to_word(tilt_deg),
            param3=0,
            param4=0,
        )

    def build_A1_relative_angle(self, azimuth_delta_deg, tilt_delta_deg) -> bytes:
        """A1 frame in Manual Relative Angle Mode (0x09): offset from current position."""
        return self.build_A1(
            servo_mode=self.SERVO_MANUAL_RELATIVE_ANGLE,
            param1=self._angle_to_word(azimuth_delta_deg),
            param2=self._angle_to_word(tilt_delta_deg),
        )

    def build_A1_motor(self, on: bool) -> bytes:
        """A1 frame to turn the gimbal servo motor on/off."""
        return self.build_A1(servo_mode=self.SERVO_MOTOR_ON_OFF, param1=1 if on else 0)

    def build_A1_home(self) -> bytes:
        """A1 frame to drive the gimbal back to its home position."""
        return self.build_A1(servo_mode=self.SERVO_HOME_POSITION)

    def build_A1_tracking(self) -> bytes:
        """A1 frame to switch the gimbal into tracking mode."""
        return self.build_A1(servo_mode=self.SERVO_TRACKING_MODE)


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