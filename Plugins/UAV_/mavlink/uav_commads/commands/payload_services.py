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
    def _speed_to_word(speed_deg_s):
        """
        Signed 16-bit word for the velocity fields used in Relative Angle
        Mode (0x09), per ICD 3.3.1.4: 1 LSB = 0.1 deg/s. NOTE: this is a
        different scale than Manual Speed Mode's (0x01) 0.01 deg/s fields
        (ICD 3.3.1.2) — don't reuse this for that mode.
        """
        return int(round(speed_deg_s * 10.0)) & 0xFFFF

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

        NOTE: confirmed against real hardware that param1 carries tilt and
        param2 carries azimuth — the reverse of how the ICD text labels
        "Parameter 1 / Parameter 2" — so this maps accordingly rather than
        following the doc's labeling literally.
        """
        return self.build_A1(
            servo_mode=self.SERVO_MANUAL_ABSOLUTE_ANGLE,
            param1=self._angle_to_word(tilt_deg),
            param2=self._angle_to_word(azimuth_deg),
            param3=0,
            param4=0,
        )

    def build_A1_relative_angle(self, azimuth_delta_deg, tilt_delta_deg,
                                 azimuth_speed_deg_s=0, tilt_speed_deg_s=0) -> bytes:
        """
        A1 frame in Manual Relative Angle Mode (0x09) — ICD 3.3.1.4.

        IMPORTANT: this mode's parameter layout is NOT the same two-slot
        (tilt, azimuth) pattern used by build_A1_absolute_angle. It uses
        four distinct fields:
            param1 = azimuth velocity, 1 LSB = 0.1°/s (0 = gimbal's own default speed)
            param2 = azimuth angle delta, 1 LSB = 360/65536°
            param3 = tilt velocity, 1 LSB = 0.1°/s (0 = gimbal's own default speed)
            param4 = tilt angle delta, 1 LSB = 360/65536°
        Previously this only set param1/param2 (copying the absolute-angle
        convention), which left param4 — where the tilt delta actually
        lives in this mode — permanently at 0, so tilt never moved, and
        put the tilt angle's raw word into param1 (read by the gimbal as
        azimuth *speed*, wrong field and wrong units), corrupting azimuth
        rate. Speeds default to 0 (system default speed) unless given.
        """
        return self.build_A1(
            servo_mode=self.SERVO_MANUAL_RELATIVE_ANGLE,
            param1=self._speed_to_word(azimuth_speed_deg_s),
            param2=self._angle_to_word(azimuth_delta_deg),
            param3=self._speed_to_word(tilt_speed_deg_s),
            param4=self._angle_to_word(tilt_delta_deg),
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

    # ── C1: optical control (zoom / focus / video source / photo / record / LRF) ──
    # ICD 3.7 "C1 Optical Control, Commonly Used, 2 Bytes". A single 16-bit,
    # big-endian, bit-packed word:
    #   bits 0-2   Sensor / video-source select
    #   bits 3-5   Operation-command-1 parameter (zoom/focus speed, 0x01-0x07)
    #   bits 6-12  Operation Command 1 (the actual action)
    #   bits 13-15 Laser Rangefinder command
    #
    # NOTE: zoom (FOV+/-) and focus (Focus+/-) are RATE commands, not
    # absolute levels like MAV_CMD_SET_CAMERA_ZOOM's 0-100 range — "rising
    # edge is valid," i.e. they start a continuous zoom/focus move at the
    # given speed until a stop command (0x01) is sent. Plan accordingly if
    # you're driving these from a slider: send start, then send stop when
    # the slider stops moving or a target is reached.

    # Video/sensor source select (bits 0-2)
    VIDEO_NO_ACTION    = 0x00
    VIDEO_EO1          = 0x01
    VIDEO_IR_THERMAL   = 0x02
    VIDEO_EO1_IR_PIP   = 0x03
    VIDEO_IR_EO1_PIP   = 0x04
    VIDEO_EO2          = 0x05
    VIDEO_FUSION       = 0x06

    # Operation Command 1 (bits 6-12)
    OP_NO_ACTION           = 0x00
    OP_STOP_FOCUS_ZOOM     = 0x01
    OP_BRIGHTNESS_PLUS     = 0x02
    OP_BRIGHTNESS_MINUS    = 0x03
    OP_CONTRAST_PLUS       = 0x04  # not supported yet, per ICD
    OP_CONTRAST_MINUS      = 0x05  # not supported yet, per ICD
    OP_APERTURE_PLUS       = 0x06  # not supported yet, per ICD
    OP_APERTURE_MINUS      = 0x07  # not supported yet, per ICD
    OP_ZOOM_OUT            = 0x08  # "FOV+" — zoom out
    OP_ZOOM_IN             = 0x09  # "FOV-" — zoom in
    OP_FOCUS_PLUS          = 0x0A
    OP_FOCUS_MINUS         = 0x0B
    OP_INTERNAL_NUC        = 0x0C  # not supported yet, per ICD
    OP_EXTERNAL_NUC        = 0x0D  # not supported yet, per ICD
    OP_POLARITY_WHITE_HOT  = 0x0E
    OP_POLARITY_BLACK_HOT  = 0x0F
    OP_GAIN_PLUS           = 0x10  # not supported yet, per ICD
    OP_GAIN_MINUS          = 0x11  # not supported yet, per ICD
    OP_IR_RAINBOW          = 0x12
    OP_TAKE_PICTURE        = 0x13
    OP_START_RECORD        = 0x14
    OP_STOP_RECORD         = 0x15
    OP_PICTURE_MODE        = 0x16
    OP_RECORD_MODE         = 0x17
    OP_PIC_RECORD_SWITCH   = 0x18
    OP_AUTO_FOCUS          = 0x19
    OP_MANUAL_FOCUS        = 0x1A
    OP_IR_DZOOM_PLUS       = 0x1B
    OP_IR_DZOOM_MINUS      = 0x1C
    OP_FORMAT_SD           = 0x1D
    OP_QUERY_SD_STATUS     = 0x1E
    OP_QUERY_SD_TOTAL      = 0x1F
    OP_QUERY_SD_FREE       = 0x20

    # Laser Rangefinder command (bits 13-15)
    LRF_NO_ACTION               = 0x00
    LRF_SINGLE_RANGING          = 0x01
    LRF_CONTINUOUS_START        = 0x02
    LRF_LPCL_CONTINUOUS_START   = 0x03  # pro models
    LRF_EXTERNAL_SYNC           = 0x04  # not supported yet, per ICD
    LRF_STOP_RANGING            = 0x05

    def build_C1(self, sensor=0, op_param=0, op_command=0, lrf_command=0) -> bytes:
        """
        Build a standalone C1 optical-control frame (frame ID 0x1C).
        sensor: 3-bit video source select (VIDEO_* constants)
        op_param: 3-bit generic parameter for op_command — for zoom/focus
                  commands this is speed, 0x01 (slowest) to 0x07 (fastest)
        op_command: 7-bit action (OP_* constants)
        lrf_command: 3-bit laser rangefinder action (LRF_* constants)
        """
        word = (
            ((lrf_command & 0x07) << 13)
            | ((op_command & 0x7F) << 6)
            | ((op_param & 0x07) << 3)
            | (sensor & 0x07)
        )
        data = self._word_to_bytes(word)
        return self._build(self.FRAME_ID_C1, data)

    # ── C1 convenience wrappers ──────────────────────────────────────────

    def build_C1_switch_video_source(self, source: int) -> bytes:
        """Switch active video source (EO / IR thermal / PIP / fusion) — VIDEO_* constants."""
        return self.build_C1(sensor=source)

    def build_C1_zoom_in(self, speed=4) -> bytes:
        """Start zooming in (telephoto) at the given speed, 1 (slowest) - 7 (fastest)."""
        return self.build_C1(op_param=speed, op_command=self.OP_ZOOM_IN)

    def build_C1_zoom_out(self, speed=4) -> bytes:
        """Start zooming out (wide) at the given speed, 1 (slowest) - 7 (fastest)."""
        return self.build_C1(op_param=speed, op_command=self.OP_ZOOM_OUT)

    def build_C1_zoom_stop(self) -> bytes:
        """Stop an in-progress zoom move (shared with focus-stop, per ICD 0x01)."""
        return self.build_C1(op_command=self.OP_STOP_FOCUS_ZOOM)

    def build_C1_focus_plus(self, speed=4) -> bytes:
        """Start focusing far at the given speed, 1 (slowest) - 7 (fastest)."""
        return self.build_C1(op_param=speed, op_command=self.OP_FOCUS_PLUS)

    def build_C1_focus_minus(self, speed=4) -> bytes:
        """Start focusing near at the given speed, 1 (slowest) - 7 (fastest)."""
        return self.build_C1(op_param=speed, op_command=self.OP_FOCUS_MINUS)

    def build_C1_focus_stop(self) -> bytes:
        """Stop an in-progress focus move (shared with zoom-stop, per ICD 0x01)."""
        return self.build_C1(op_command=self.OP_STOP_FOCUS_ZOOM)

    def build_C1_auto_focus(self) -> bytes:
        """Switch the camera to autofocus mode."""
        return self.build_C1(op_command=self.OP_AUTO_FOCUS)

    def build_C1_manual_focus(self) -> bytes:
        """Switch the camera to manual focus mode."""
        return self.build_C1(op_command=self.OP_MANUAL_FOCUS)

    def build_C1_take_picture(self) -> bytes:
        """Trigger a single photo capture."""
        return self.build_C1(op_command=self.OP_TAKE_PICTURE)

    def build_C1_start_record(self) -> bytes:
        """Start video recording (native protocol, not MAVLink camera command)."""
        return self.build_C1(op_command=self.OP_START_RECORD)

    def build_C1_stop_record(self) -> bytes:
        """Stop video recording (native protocol, not MAVLink camera command)."""
        return self.build_C1(op_command=self.OP_STOP_RECORD)

    def build_C1_polarity_white_hot(self) -> bytes:
        """Set IR palette polarity to white-hot."""
        return self.build_C1(op_command=self.OP_POLARITY_WHITE_HOT)

    def build_C1_polarity_black_hot(self) -> bytes:
        """Set IR palette polarity to black-hot."""
        return self.build_C1(op_command=self.OP_POLARITY_BLACK_HOT)

    def build_C1_laser_single_range(self) -> bytes:
        """Trigger a single laser rangefinder measurement."""
        return self.build_C1(lrf_command=self.LRF_SINGLE_RANGING)

    def build_C1_laser_continuous_start(self) -> bytes:
        """Start continuous laser rangefinding."""
        return self.build_C1(lrf_command=self.LRF_CONTINUOUS_START)

    def build_C1_laser_stop(self) -> bytes:
        """Stop laser rangefinding."""
        return self.build_C1(lrf_command=self.LRF_STOP_RANGING)