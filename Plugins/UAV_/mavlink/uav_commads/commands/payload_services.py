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
