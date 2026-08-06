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
    FRAME_ID_C2       = 0x2C  # C2 alone (3 bytes of data) — infrequently-used optical control
    FRAME_ID_A2       = 0x2A  # A2 alone (2 bytes) — infrequently-used servo control
    FRAME_ID_E2       = 0x2E  # E2 alone (5 bytes) — infrequently-used tracking command
    FRAME_ID_A2_C2_E2 = 0x31  # combined A2+C2+E2
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
        (ICD 3.3.1.2) — don't reuse this for that mode, use
        _manual_speed_to_word instead.
        """
        return int(round(speed_deg_s * 10.0)) & 0xFFFF

    @staticmethod
    def _manual_speed_to_word(speed_deg_s):
        """
        Signed 16-bit word for Manual Speed Mode's (0x01) velocity fields,
        per ICD 3.3.1.2: 1 LSB = 0.01 deg/s — ten times finer resolution
        than Relative Angle Mode's velocity fields. Don't mix these up;
        use _speed_to_word for 0x09 instead.
        """
        return int(round(speed_deg_s * 100.0)) & 0xFFFF

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
        Absolute Angle Mode (0x0B) — ICD 3.3.1.5, sent via the combined
        A1+C1+E1 frame (matches every worked example in ICD section 4.1,
        including "Angle control" — verified byte-for-byte against it).
        Points the gimbal to a fixed azimuth/tilt relative to home
        position (home = 0). param3/param4 are meaningless in this mode,
        left as 0.

        NOTE: confirmed against real hardware that param1 carries tilt and
        param2 carries azimuth — the reverse of how the ICD text labels
        "Parameter 1 / Parameter 2" — so this maps accordingly rather than
        following the doc's labeling literally.
        """
        return self.build_combined_A1_C1_E1(
            a1_servo_mode=self.SERVO_MANUAL_ABSOLUTE_ANGLE,
            a1_param1=self._angle_to_word(tilt_deg),
            a1_param2=self._angle_to_word(azimuth_deg),
            a1_param3=0,
            a1_param4=0,
        )

    def build_A1_relative_angle(self, azimuth_delta_deg, tilt_delta_deg,
                                 azimuth_speed_deg_s=0, tilt_speed_deg_s=0) -> bytes:
        """
        Manual Relative Angle Mode (0x09) — ICD 3.3.1.4, sent via the
        combined A1+C1+E1 frame (see build_A1_absolute_angle's note on why).

        IMPORTANT: this mode's parameter layout is NOT the same two-slot
        (tilt, azimuth) pattern used by build_A1_absolute_angle. It uses
        four distinct fields:
            param1 = azimuth velocity, 1 LSB = 0.1°/s (0 = gimbal's own default speed)
            param2 = azimuth angle delta, 1 LSB = 360/65536°
            param3 = tilt velocity, 1 LSB = 0.1°/s (0 = gimbal's own default speed)
            param4 = tilt angle delta, 1 LSB = 360/65536°
        Speeds default to 0 (system default speed) unless given.
        """
        return self.build_combined_A1_C1_E1(
            a1_servo_mode=self.SERVO_MANUAL_RELATIVE_ANGLE,
            a1_param1=self._speed_to_word(azimuth_speed_deg_s),
            a1_param2=self._angle_to_word(azimuth_delta_deg),
            a1_param3=self._speed_to_word(tilt_speed_deg_s),
            a1_param4=self._angle_to_word(tilt_delta_deg),
        )

    def build_A1_manual_speed(self, azimuth_vel_deg_s=0.0, tilt_vel_deg_s=0.0) -> bytes:
        """
        Manual Speed Mode (0x01) — ICD 3.3.1.2. Pure velocity control: no
        angle/position field at all. Send this ONCE when the desired rate
        changes and the gimbal keeps moving at that rate autonomously
        until a new speed (or mode change) arrives — this is the
        protocol's purpose-built fit for continuous joystick control,
        as opposed to repeatedly streaming small relative-angle deltas
        (build_A1_relative_angle) to fake continuous motion.

        NOTE: the scale here is 1 LSB = 0.01 deg/s — TEN TIMES finer than
        Relative Angle Mode's velocity fields (1 LSB = 0.1 deg/s, ICD
        3.3.1.4). Don't reuse _speed_to_word for this mode; it uses
        _manual_speed_to_word instead.

        param1 = azimuth velocity, param2 = tilt velocity, per ICD's
        literal labeling — this mode's doc example wasn't diagnostic for
        a possible swap the way absolute/relative angle mode's was, so
        this follows the ICD text directly. Verify direction on hardware
        before relying on it.
        """
        return self.build_combined_A1_C1_E1(
            a1_servo_mode=self.SERVO_MANUAL_SPEED,
            a1_param1=self._manual_speed_to_word(azimuth_vel_deg_s),
            a1_param2=self._manual_speed_to_word(tilt_vel_deg_s),
        )

    def build_A1_motor(self, on: bool) -> bytes:
        """
        Turn the gimbal servo motor on/off, via the combined A1+C1+E1
        frame. Verified byte-for-byte against the ICD's own "Motor ON"
        example: param1 = 0x0100 for ON, 0x0001 for OFF (ICD 3.3.1.1).
        """
        return self.build_combined_A1_C1_E1(
            a1_servo_mode=self.SERVO_MOTOR_ON_OFF,
            a1_param1=0x0100 if on else 0x0001,
        )

    def build_A1_home(self) -> bytes:
        """
        Drive the gimbal back to its home position ("Recenter" in the
        ICD's examples), via the combined A1+C1+E1 frame.
        """
        return self.build_combined_A1_C1_E1(a1_servo_mode=self.SERVO_HOME_POSITION)

    def build_A1_tracking(self) -> bytes:
        """Switch the gimbal into tracking mode, via the combined A1+C1+E1 frame."""
        return self.build_combined_A1_C1_E1(a1_servo_mode=self.SERVO_TRACKING_MODE)

    def build_A1_enable_follow_yaw(self) -> bytes:
        """Enable follow-yaw, via the combined A1+C1+E1 frame (ICD example 4.1)."""
        return self.build_combined_A1_C1_E1(a1_servo_mode=self.SERVO_FOLLOW_YAW)

    def build_A1_disable_follow_yaw(self) -> bytes:
        """Disable follow-yaw, via the combined A1+C1+E1 frame (ICD example 4.1)."""
        return self.build_combined_A1_C1_E1(a1_servo_mode=self.SERVO_FOLLOW_YAW_DISABLE)

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
        Build a STANDALONE C1 optical-control frame (frame ID 0x1C).
        sensor: 3-bit video source select (VIDEO_* constants)
        op_param: 3-bit generic parameter for op_command — for zoom/focus
                  commands this is speed, 0x01 (slowest) to 0x07 (fastest)
        op_command: 7-bit action (OP_* constants)
        lrf_command: 3-bit laser rangefinder action (LRF_* constants)

        NOTE: every worked example in the ICD's own section 4 (zoom,
        focus, laser-adjacent, motor, angle control — all of them) sends
        C1 wrapped inside the COMBINED A1+C1+E1 frame (cmd_id 0x30), never
        as this standalone form. Section 2.3(c) also only documents a
        status reply being sent in response to a heartbeat or an
        "A1+C1+E1+(S1)" packet — not a lone C1. Prefer
        build_combined_A1_C1_E1() (used by all the *_C1_* convenience
        methods below) unless you have a specific reason to send C1 alone.
        """
        word = (
            ((lrf_command & 0x07) << 13)
            | ((op_command & 0x7F) << 6)
            | ((op_param & 0x07) << 3)
            | (sensor & 0x07)
        )
        data = self._word_to_bytes(word)
        return self._build(self.FRAME_ID_C1, data)

    def build_combined_A1_C1_E1(self, c1_sensor=0, c1_op_param=0, c1_op_command=0,
                                 c1_lrf_command=0, a1_servo_mode=None,
                                 a1_param1=0, a1_param2=0, a1_param3=0, a1_param4=0,
                                 e1_bytes: bytes = b'\x00\x00\x00') -> bytes:
        """
        Build the COMBINED A1+C1+E1 frame (cmd_id 0x30) — the format every
        worked example in ICD section 4 actually uses, including for pure
        C1-only actions like zoom/focus/photo/laser. In those examples A1
        is set to SERVO_NO_CHANGE (0x0F, a genuine no-op — "do not change
        the servo state, the parameter is meaningless") and E1 is all
        zero, so C1 rides along without disturbing whatever the gimbal is
        currently doing angle-wise. Defaults here match that pattern:
        a1_servo_mode defaults to SERVO_NO_CHANGE if not given.
        """
        if a1_servo_mode is None:
            a1_servo_mode = self.SERVO_NO_CHANGE
        a1_byte1 = a1_servo_mode & 0x0F
        a1_data = bytes([a1_byte1])
        for p in (a1_param1, a1_param2, a1_param3, a1_param4):
            a1_data += self._word_to_bytes(p & 0xFFFF)

        c1_word = (
            ((c1_lrf_command & 0x07) << 13)
            | ((c1_op_command & 0x7F) << 6)
            | ((c1_op_param & 0x07) << 3)
            | (c1_sensor & 0x07)
        )
        c1_data = self._word_to_bytes(c1_word)

        if len(e1_bytes) != 3:
            raise ValueError(f"e1_bytes must be exactly 3 bytes, got {len(e1_bytes)}")

        data = a1_data + c1_data + e1_bytes  # 9 + 2 + 3 = 14 bytes
        return self._build(self.FRAME_ID_A1_C1_E1, data)

    # ── C1 convenience wrappers (sent via the combined A1+C1+E1 frame) ─────

    def build_C1_switch_video_source(self, source: int) -> bytes:
        """Switch active video source (EO / IR thermal / PIP / fusion) — VIDEO_* constants."""
        return self.build_combined_A1_C1_E1(c1_sensor=source)

    def build_C1_zoom_in(self, speed=4) -> bytes:
        """Start zooming in (telephoto) at the given speed, 1 (slowest) - 7 (fastest)."""
        return self.build_combined_A1_C1_E1(c1_op_param=speed, c1_op_command=self.OP_ZOOM_IN)

    def build_C1_zoom_out(self, speed=4) -> bytes:
        """Start zooming out (wide) at the given speed, 1 (slowest) - 7 (fastest)."""
        return self.build_combined_A1_C1_E1(c1_op_param=speed, c1_op_command=self.OP_ZOOM_OUT)

    def build_C1_zoom_stop(self) -> bytes:
        """
        Stop an in-progress zoom move (shared with focus-stop).

        NOTE: table 3.7 names 0x01 ("Stop focus, stop zoom") as the
        explicit stop enumeration, but the ICD's own worked "Stop zoom"
        example (section 4.2) actually sends 0x00 ("No action") instead.
        This uses the table's documented 0x01 by default since it's the
        semantically explicit value; if it doesn't actually halt zoom on
        your hardware, try build_C1_zoom_stop(use_no_action=True) to match
        the vendor's own example instead.
        """
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_STOP_FOCUS_ZOOM)

    def build_C1_focus_plus(self, speed=4) -> bytes:
        """Start focusing far at the given speed, 1 (slowest) - 7 (fastest)."""
        return self.build_combined_A1_C1_E1(c1_op_param=speed, c1_op_command=self.OP_FOCUS_PLUS)

    def build_C1_focus_minus(self, speed=4) -> bytes:
        """Start focusing near at the given speed, 1 (slowest) - 7 (fastest)."""
        return self.build_combined_A1_C1_E1(c1_op_param=speed, c1_op_command=self.OP_FOCUS_MINUS)

    def build_C1_focus_stop(self) -> bytes:
        """Stop an in-progress focus move. See build_C1_zoom_stop's note on 0x01 vs 0x00."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_STOP_FOCUS_ZOOM)

    def build_C1_stop_no_action_variant(self) -> bytes:
        """
        Alternate stop for zoom/focus using op_command=0x00 ("No action"),
        matching the ICD's own worked "Stop zoom" example byte-for-byte
        instead of the table's documented 0x01. Try this if
        build_C1_zoom_stop()/build_C1_focus_stop() don't actually halt
        motion on your hardware.
        """
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_NO_ACTION)

    def build_C1_auto_focus(self) -> bytes:
        """Switch the camera to autofocus mode."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_AUTO_FOCUS)

    def build_C1_manual_focus(self) -> bytes:
        """Switch the camera to manual focus mode."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_MANUAL_FOCUS)

    def build_C1_take_picture(self) -> bytes:
        """Trigger a single photo capture."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_TAKE_PICTURE)

    def build_C1_start_record(self) -> bytes:
        """Start video recording (native protocol, not MAVLink camera command)."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_START_RECORD)

    def build_C1_stop_record(self) -> bytes:
        """Stop video recording (native protocol, not MAVLink camera command)."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_STOP_RECORD)

    def build_C1_polarity_white_hot(self) -> bytes:
        """Set IR palette polarity to white-hot."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_POLARITY_WHITE_HOT)

    def build_C1_polarity_black_hot(self) -> bytes:
        """Set IR palette polarity to black-hot."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_POLARITY_BLACK_HOT)

    def build_C1_ir_rainbow(self) -> bytes:
        """Set IR palette to rainbow (false-color)."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_IR_RAINBOW)

    def build_C1_ir_dzoom_plus(self) -> bytes:
        """IR camera digital zoom in."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_IR_DZOOM_PLUS)

    def build_C1_ir_dzoom_minus(self) -> bytes:
        """IR camera digital zoom out."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_IR_DZOOM_MINUS)

    def build_C1_laser_single_range(self) -> bytes:
        """Trigger a single laser rangefinder measurement."""
        return self.build_combined_A1_C1_E1(c1_lrf_command=self.LRF_SINGLE_RANGING)

    def build_C1_laser_continuous_start(self) -> bytes:
        """Start continuous laser rangefinding."""
        return self.build_combined_A1_C1_E1(c1_lrf_command=self.LRF_CONTINUOUS_START)

    def build_C1_laser_stop(self) -> bytes:
        """Stop laser rangefinding."""
        return self.build_combined_A1_C1_E1(c1_lrf_command=self.LRF_STOP_RANGING)

    # ── FOV+/- (ICD's own naming — same op codes as zoom_in/zoom_out) ──────
    # OP_ZOOM_OUT (0x08) is literally "FOV+" in the ICD; OP_ZOOM_IN (0x09)
    # is "FOV-". These are thin aliases so both naming conventions are
    # discoverable — they produce identical frames to build_C1_zoom_out/in.

    def build_C1_fov_plus(self, speed=4) -> bytes:
        """FOV+ (ICD naming) — zoom OUT (wide). Same as build_C1_zoom_out."""
        return self.build_C1_zoom_out(speed)

    def build_C1_fov_minus(self, speed=4) -> bytes:
        """FOV- (ICD naming) — zoom IN (telephoto). Same as build_C1_zoom_in."""
        return self.build_C1_zoom_in(speed)

    # ── Picture / record mode switching, SD card ────────────────────────────

    def build_C1_picture_mode(self) -> bytes:
        """Switch the camera to picture (still-photo) mode."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_PICTURE_MODE)

    def build_C1_record_mode(self) -> bytes:
        """Switch the camera to video-record mode."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_RECORD_MODE)

    def build_C1_pic_record_switch(self) -> bytes:
        """Toggle between picture and record mode."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_PIC_RECORD_SWITCH)

    def build_C1_format_sd(self) -> bytes:
        """DESTRUCTIVE — formats the SD card, erasing all stored photos/video."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_FORMAT_SD)

    def build_C1_query_sd_status(self) -> bytes:
        """
        Query SD card status (T series). Send with expect_response=True
        to get a reply — NOTE: the reply's byte layout for this specific
        query isn't in the ICD pages available here (likely the separate
        60-byte "N" status packet, cmd_id 0xB2, not yet documented/parsed
        in this codebase), so this can trigger the query but there's no
        parser yet to decode the actual status value from the reply.
        """
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_QUERY_SD_STATUS)

    def build_C1_query_sd_total(self) -> bytes:
        """Query SD card total capacity (T series). See build_C1_query_sd_status's note on reply parsing."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_QUERY_SD_TOTAL)

    def build_C1_query_sd_free(self) -> bytes:
        """Query SD card remaining free capacity (T series). See build_C1_query_sd_status's note on reply parsing."""
        return self.build_combined_A1_C1_E1(c1_op_command=self.OP_QUERY_SD_FREE)

    # ── Combined A2+C2+E2 frame (cmd_id 0x31) — absolute zoom, etc. ─────────
    # Verified byte-for-byte against the ICD's own "Directly zoom to 20x
    # times" example (section 4.2).

    def build_combined_A2_C2_E2(self, c2_command1=0, c2_param_word=0,
                                 a2_bytes: bytes = b'\x00\x00',
                                 e2_bytes: bytes = b'\x00\x00\x00\x00\x00') -> bytes:
        """
        Build the combined A2+C2+E2 frame (cmd_id 0x31). a2_bytes/e2_bytes
        default to all-zero ("no action") — pass explicit bytes if you
        need A2's servo-action/adjustment fields (ICD 3.4) or E2's
        extended tracking display commands (ICD 3.12) alongside a C2
        action in the same frame.
        """
        if len(a2_bytes) != 2:
            raise ValueError(f"a2_bytes must be exactly 2 bytes, got {len(a2_bytes)}")
        if len(e2_bytes) != 5:
            raise ValueError(f"e2_bytes must be exactly 5 bytes, got {len(e2_bytes)}")

        c2_data = bytes([c2_command1 & 0xFF]) + self._word_to_bytes(c2_param_word & 0xFFFF)
        data = a2_bytes + c2_data + e2_bytes  # 2 + 3 + 5 = 10 bytes
        return self._build(self.FRAME_ID_A2_C2_E2, data)

    def build_C2_set_eo_zoom(self, zoom_times: float) -> bytes:
        """
        Directly set the EO camera's absolute zoom level (ICD 3.8.1.2,
        C2 command 0x53) — jumps straight to a zoom multiplier instead of
        ramping via the FOV+/- rate commands. zoom_times: e.g. 20.0 for
        20x. Verified byte-for-byte against the ICD's own "Directly zoom
        to 20x times" example.
        """
        zoom_word = int(round(zoom_times * 10.0)) & 0xFFFF  # 1 LSB = 0.1x, per ICD
        return self.build_combined_A2_C2_E2(
            c2_command1=self.C2_CMD_SET_EO_ZOOM,
            c2_param_word=zoom_word,
        )

    # ── C2: optical control, infrequently used (laser power/zoom/sync) ─────
    # ICD 3.8 "C2 Optical Control Infrequently Used, 3 Bytes": byte1 selects
    # WHICH sub-function this packet performs; bytes2-3 are a parameter
    # whose bit layout depends on byte1. For the laser specifically, byte1
    # = 0x74 ("Laser control command"), ICD 3.8.1.3.

    C2_CMD_LASER_CONTROL = 0x74  # byte1 value selecting laser control
    C2_CMD_POWER_CONTROL = 0x75  # byte1 value selecting the alternate hard-power path
    C2_CMD_SET_EO_ZOOM   = 0x53  # byte1 value selecting "set EO zoom to parameter value" (ICD 3.8.1.2)
    C2_CMD_EO_DZOOM_ON   = 0x06  # ICD 3.8 C2 command table
    C2_CMD_EO_DZOOM_OFF  = 0x07
    C2_CMD_NEAR_IR_ON    = 0x4A  # ICD 3.8 C2 command table
    C2_CMD_NEAR_IR_OFF   = 0x4B

    # Laser control sub-command (bits 0-4 of the 2-byte parameter, when byte1=0x74)
    LASER_CTRL_NO_ACTION            = 0x00
    LASER_CTRL_ON                   = 0x01
    LASER_CTRL_OFF                  = 0x02
    LASER_CTRL_AUTOCHECK            = 0x03  # not supported yet, per ICD
    LASER_CTRL_ZOOM_OUT             = 0x04  # the laser's OWN zoom (beam), not EO/IR zoom
    LASER_CTRL_ZOOM_IN              = 0x05
    LASER_CTRL_SYNC_EO_MODE         = 0x06  # laser zoom auto-syncs with EO zoom
    LASER_CTRL_MANUAL_MODE          = 0x07  # laser zoom under independent manual control
    LASER_CTRL_ELEVATION_PROT_OFF   = 0x0E  # not supported yet, per ICD
    LASER_CTRL_ELEVATION_PROT_ON    = 0x0F  # not supported yet, per ICD

    def build_C2(self, command1: int, param_word: int) -> bytes:
        """
        Build a standalone C2 (Optical Control Infrequently Used) frame,
        frame ID 0x2C. `command1` selects the C2 sub-function (ICD 3.8's
        "Command 1" byte); `param_word` is the 2-byte parameter whose
        meaning depends on command1 (ICD 3.8.1.x).
        """
        data = bytes([command1 & 0xFF]) + self._word_to_bytes(param_word & 0xFFFF)
        return self._build(self.FRAME_ID_C2, data)

    def build_C2_laser_on(self) -> bytes:
        """Turn the laser rangefinder module on (ICD 3.8.1.3)."""
        return self.build_C2(self.C2_CMD_LASER_CONTROL, self.LASER_CTRL_ON)

    def build_C2_laser_off(self) -> bytes:
        """Turn the laser rangefinder module off (ICD 3.8.1.3)."""
        return self.build_C2(self.C2_CMD_LASER_CONTROL, self.LASER_CTRL_OFF)

    def build_C2_laser_zoom_out(self) -> bytes:
        """Laser's own zoom out (beam), separate from EO/IR optical zoom."""
        return self.build_C2(self.C2_CMD_LASER_CONTROL, self.LASER_CTRL_ZOOM_OUT)

    def build_C2_laser_zoom_in(self) -> bytes:
        """Laser's own zoom in (beam), separate from EO/IR optical zoom."""
        return self.build_C2(self.C2_CMD_LASER_CONTROL, self.LASER_CTRL_ZOOM_IN)

    def build_C2_laser_sync_eo_mode(self) -> bytes:
        """Laser zoom auto-syncs with the EO camera's zoom level."""
        return self.build_C2(self.C2_CMD_LASER_CONTROL, self.LASER_CTRL_SYNC_EO_MODE)

    def build_C2_laser_manual_mode(self) -> bytes:
        """Laser zoom under independent manual control (not synced to EO zoom)."""
        return self.build_C2(self.C2_CMD_LASER_CONTROL, self.LASER_CTRL_MANUAL_MODE)

    def build_C2_laser_power(self, on: bool) -> bytes:
        """
        Alternate laser power toggle via the Power Control command (0x75),
        bits 14-15 specifically for laser power (0=no change, 1=ON,
        2=OFF), leaving EO1/IR/EO2 power bits at 0 (no change). Some
        firmware gates the laser on this "hard power" bit separately from
        the 0x74 logical enable — try this if build_C2_laser_on()/off()
        doesn't produce the expected behavior.
        """
        laser_bits = 0b01 if on else 0b10  # 1=ON, 2=OFF in the 2-bit field
        param_word = (laser_bits & 0x03) << 14
        return self.build_C2(self.C2_CMD_POWER_CONTROL, param_word)

    def build_C2_eo_dzoom_on(self) -> bytes:
        """
        Enable EO digital zoom. Per ICD 3.7's note on C1 op 0x08/0x09
        (FOV+/-): "the digital zoom is included when it is enabled" —
        without this, FOV+/- only reaches the optical zoom limit.
        """
        return self.build_C2(self.C2_CMD_EO_DZOOM_ON, 0)

    def build_C2_eo_dzoom_off(self) -> bytes:
        """Disable EO digital zoom — FOV+/- reverts to optical-only range."""
        return self.build_C2(self.C2_CMD_EO_DZOOM_OFF, 0)

    def build_C2_near_ir_on(self) -> bytes:
        """Enable near-infrared mode (ICD 3.8 C2 command table, 0x4A)."""
        return self.build_C2(self.C2_CMD_NEAR_IR_ON, 0)

    def build_C2_near_ir_off(self) -> bytes:
        """Disable near-infrared mode (ICD 3.8 C2 command table, 0x4B)."""
        return self.build_C2(self.C2_CMD_NEAR_IR_OFF, 0)

    # ── E1: tracking command, commonly used, 3 bytes ────────────────────────
    # ICD 3.11 — the section heading in the ICD itself says "E2" but the
    # byte count (3) and the frame-composition table both identify this as
    # E1 (frame ID 0x1E, 3 bytes); the doc's own heading looks like a
    # typo. Embedded in the combined A1+C1+E1 frame via
    # build_combined_A1_C1_E1(e1_bytes=...), same as everything else.
    #
    # Layout:
    #   byte1  bits0-2: tracking source select (TRACK_SOURCE_*)
    #          bits3-7: Parameter1 (5 bits) — meaning depends on basic_command
    #   byte2  Basic Command (E1_CMD_*)
    #   byte3  Parameter2 (1 byte) — meaning depends on basic_command

    TRACK_SOURCE_NONE = 0x00
    TRACK_SOURCE_EO1  = 0x01
    TRACK_SOURCE_IR   = 0x02
    TRACK_SOURCE_EO2  = 0x03

    E1_CMD_NO_ACTION              = 0x00
    E1_CMD_STOP                   = 0x01
    E1_CMD_SEARCH                 = 0x02  # bring up the targeting cross
    E1_CMD_TURN_ON_TRACKING       = 0x03  # lock onto whatever's under the cross
    E1_CMD_SWITCH_TO_CROSS        = 0x04  # switch tracking point to cross position
    E1_CMD_AI_ON_OFF              = 0x05
    E1_CMD_ADJUST_TRACK_VELOCITY  = 0x06
    E1_CMD_ADJUST_TRACK_VEL_COEF  = 0x07
    E1_CMD_AI_AUTO_TRACK          = 0x08  # auto-track once AI recognizes something

    # Tracking template size (E1_CMD_* slot — sent as basic_command itself)
    E1_TEMPLATE_16x16_ULTRA_SMALL   = 0x20  # not supported yet, per ICD
    E1_TEMPLATE_32x32_SMALL         = 0x21
    E1_TEMPLATE_64x64_MEDIUM        = 0x22
    E1_TEMPLATE_128x128_BIG         = 0x23
    E1_TEMPLATE_SMALL_MEDIUM_ADAPT  = 0x24
    E1_TEMPLATE_SMALL_BIG_ADAPT     = 0x25
    E1_TEMPLATE_MEDIUM_BIG_ADAPT    = 0x26
    # 0x27 is not defined in the ICD — it jumps straight to 0x28
    E1_TEMPLATE_SMALL_MEDIUM_BIG_ADAPT = 0x28

    def build_E1(self, tracking_source=0, param1_5bit=0, basic_command=0, param2=0) -> bytes:
        """
        Build the raw 3-byte E1 tracking-command payload, for embedding
        via build_combined_A1_C1_E1(e1_bytes=...). Prefer the specific
        build_E1_* convenience methods below unless you need a
        combination they don't cover.
        """
        byte1 = ((param1_5bit & 0x1F) << 3) | (tracking_source & 0x07)
        return bytes([byte1, basic_command & 0xFF, param2 & 0xFF])

    def build_E1_stop_tracking(self) -> bytes:
        """Stop tracking."""
        e1 = self.build_E1(basic_command=self.E1_CMD_STOP)
        return self.build_combined_A1_C1_E1(e1_bytes=e1)

    def build_E1_search(self, azimuth_nudge=0, tilt_nudge=0) -> bytes:
        """
        Bring up the search cross and nudge it (ICD 3.11.1.1). Unlike a
        normal signed integer, the wire format splits direction across
        two ranges: 1-15 = one direction, 16-31 = the other (right/down
        for the low half, left/up for the high half) — this accepts a
        plain signed value in [-15, 15] and maps it to that scheme:
        positive -> 1-15, negative -> 16-30, 0 -> "no action" on that axis.
        """
        def to_split_range(v):
            v = max(-15, min(15, int(round(v))))
            if v == 0:
                return 0
            return v if v > 0 else 16 + (-v) - 1
        e1 = self.build_E1(
            param1_5bit=to_split_range(azimuth_nudge),
            basic_command=self.E1_CMD_SEARCH,
            param2=to_split_range(tilt_nudge),
        )
        return self.build_combined_A1_C1_E1(e1_bytes=e1)

    def build_E1_turn_on_tracking(self, engage_servo_tracking: bool = True) -> bytes:
        """
        Lock onto whatever's currently under the search cross. By
        default this ALSO sets A1 servo mode to SERVO_TRACKING_MODE
        (0x06) in the same frame, so the gimbal servo explicitly enters
        tracking-follow mode rather than relying on an assumed
        side-effect of the E1 command alone. Pass
        engage_servo_tracking=False to send E1 only, leaving A1 at
        SERVO_NO_CHANGE, if you want to control servo mode separately.

        This matters: previously A1 was left at NO_CHANGE here, and
        nothing else ever explicitly requested tracking servo mode
        either — so tracking-follow likely only ever happened as an
        unconfirmed side-effect of E1 alone, if at all.
        """
        e1 = self.build_E1(basic_command=self.E1_CMD_TURN_ON_TRACKING)
        a1_mode = self.SERVO_TRACKING_MODE if engage_servo_tracking else None
        return self.build_combined_A1_C1_E1(e1_bytes=e1, a1_servo_mode=a1_mode)

    def build_E1_switch_to_cross(self) -> bytes:
        """Switch the active tracking point to the cross position."""
        e1 = self.build_E1(basic_command=self.E1_CMD_SWITCH_TO_CROSS)
        return self.build_combined_A1_C1_E1(e1_bytes=e1)

    def build_E1_ai_on(self) -> bytes:
        """Turn AI-assisted recognition on."""
        e1 = self.build_E1(basic_command=self.E1_CMD_AI_ON_OFF, param2=1)
        return self.build_combined_A1_C1_E1(e1_bytes=e1)

    def build_E1_ai_off(self) -> bytes:
        """Turn AI-assisted recognition off."""
        e1 = self.build_E1(basic_command=self.E1_CMD_AI_ON_OFF, param2=0)
        return self.build_combined_A1_C1_E1(e1_bytes=e1)

    def build_E1_ai_auto_track(self) -> bytes:
        """Auto-track whatever the AI recognizes, without manually placing the cross first."""
        e1 = self.build_E1(basic_command=self.E1_CMD_AI_AUTO_TRACK)
        return self.build_combined_A1_C1_E1(e1_bytes=e1)

    def build_E1_set_template_size(self, template: int) -> bytes:
        """template: one of the E1_TEMPLATE_* constants."""
        e1 = self.build_E1(basic_command=template)
        return self.build_combined_A1_C1_E1(e1_bytes=e1)

    def build_E1_set_tracking_velocity(self, deg_per_sec: float) -> bytes:
        """
        Absolute tracking (gimbal-chase) speed (ICD 3.11.1.4). 1 LSB =
        0.2 deg/s, 0 = system default speed. Clamped to what a single
        unsigned byte can hold (0-51 deg/s in 0.2 steps).
        """
        val = max(0, min(255, int(round(deg_per_sec / 0.2))))
        e1 = self.build_E1(basic_command=self.E1_CMD_ADJUST_TRACK_VELOCITY, param2=val)
        return self.build_combined_A1_C1_E1(e1_bytes=e1)

    def build_E1_set_tracking_velocity_coefficient(self, coefficient: float) -> bytes:
        """
        Scale the gimbal's own default tracking speed by a multiplier
        instead of setting an absolute speed (ICD 3.11.1.3). 1 LSB =
        0.1x, 0 = system default. E.g. 1.5 for 1.5x default speed.
        """
        val = max(0, min(255, int(round(coefficient / 0.1))))
        e1 = self.build_E1(basic_command=self.E1_CMD_ADJUST_TRACK_VEL_COEF, param2=val)
        return self.build_combined_A1_C1_E1(e1_bytes=e1)

    # ── E2: tracking command, infrequently used, 5 bytes ────────────────────
    # ICD 3.12. Embedded via build_combined_A2_C2_E2(e2_bytes=...).
    #   byte1    Extended Command1 (E2_CMD_*)
    #   bytes2-3 Parameter1 (2 bytes) — meaning depends on command1
    #   bytes4-5 Parameter2 (2 bytes) — meaning depends on command1

    E2_CMD_CHAR_DISPLAY_ON          = 0x02
    E2_CMD_CHAR_DISPLAY_OFF         = 0x03
    E2_CMD_DISPLAY_CROSS            = 0x04
    E2_CMD_HIDE_CROSS               = 0x05
    E2_CMD_GPS_DISPLAY_ON           = 0x06
    E2_CMD_GPS_DISPLAY_OFF          = 0x07
    E2_CMD_GIMBAL_ANGLE_DISPLAY_ON  = 0x08
    E2_CMD_GIMBAL_ANGLE_DISPLAY_OFF = 0x09
    E2_CMD_TRACK_POINT_TO_POSITION  = 0x0A  # move tracking point to a pixel coordinate
    E2_CMD_RECT_AREA_POINT_1        = 0x0B  # first corner of a rectangular tracking region
    E2_CMD_RECT_AREA_POINT_2        = 0x0C  # second (opposite) corner

    def build_E2(self, command1=0, param1_word=0, param2_word=0) -> bytes:
        """
        Build the raw 5-byte E2 extended-tracking-command payload, for
        embedding via build_combined_A2_C2_E2(e2_bytes=...). Prefer the
        specific build_E2_* convenience methods below.
        """
        return (bytes([command1 & 0xFF])
                + self._word_to_bytes(param1_word & 0xFFFF)
                + self._word_to_bytes(param2_word & 0xFFFF))

    def build_E2_track_point_to_pixel(self, x_pixel: int, y_pixel: int) -> bytes:
        """
        Move the tracking point directly to a pixel coordinate (ICD
        3.12.1.1) — signed integers, 1 LSB = 1 pixel, origin presumably
        image center per the ICD's sign convention (left/up negative,
        right/down positive is implied by the rectangular-area point
        convention below, though not explicitly restated here). This is
        the direct alternative to build_E1_search's cross-nudging.
        """
        e2 = self.build_E2(
            command1=self.E2_CMD_TRACK_POINT_TO_POSITION,
            param1_word=x_pixel & 0xFFFF,
            param2_word=y_pixel & 0xFFFF,
        )
        return self.build_combined_A2_C2_E2(e2_bytes=e2)

    def build_E2_rect_area_point1(self, x_pixel: int, y_pixel: int) -> bytes:
        """First corner of a rectangular tracking region (ICD 3.12.1.2): left negative/right positive, up negative/down positive, 1 LSB = 1 pixel."""
        e2 = self.build_E2(
            command1=self.E2_CMD_RECT_AREA_POINT_1,
            param1_word=x_pixel & 0xFFFF,
            param2_word=y_pixel & 0xFFFF,
        )
        return self.build_combined_A2_C2_E2(e2_bytes=e2)

    def build_E2_rect_area_point2(self, x_pixel: int, y_pixel: int) -> bytes:
        """Second (opposite) corner of a rectangular tracking region."""
        e2 = self.build_E2(
            command1=self.E2_CMD_RECT_AREA_POINT_2,
            param1_word=x_pixel & 0xFFFF,
            param2_word=y_pixel & 0xFFFF,
        )
        return self.build_combined_A2_C2_E2(e2_bytes=e2)

    def build_E2_display_cross(self, show: bool) -> bytes:
        """Show/hide the center targeting cross overlay."""
        cmd = self.E2_CMD_DISPLAY_CROSS if show else self.E2_CMD_HIDE_CROSS
        e2 = self.build_E2(command1=cmd)
        return self.build_combined_A2_C2_E2(e2_bytes=e2)

    def build_E2_gimbal_angle_display(self, show: bool) -> bytes:
        """Show/hide the gimbal angle overlay."""
        cmd = self.E2_CMD_GIMBAL_ANGLE_DISPLAY_ON if show else self.E2_CMD_GIMBAL_ANGLE_DISPLAY_OFF
        e2 = self.build_E2(command1=cmd)
        return self.build_combined_A2_C2_E2(e2_bytes=e2)

    # ── Reply parsing (T1+F1+B1+D1 status frame, cmd_id 0x40) ──────────────
    # Per ICD 2.3(c), the payload replies with T1+F1+B1+D1 (41 bytes: T1=22,
    # F1=1, B1=6, D1=12) in response to a heartbeat or an A1+C1+E1(+S1)
    # packet. D1 (ICD 3.9) carries the laser rangefinder result.

    @staticmethod
    def find_frames(raw: bytes) -> list:
        """
        Scan an arbitrary byte buffer for complete, checksum-valid
        Viewlink frames, WITHOUT assuming byte 0 is a frame boundary.

        Necessary because MAVLink SERIAL_CONTROL's "respond" flag has no
        concept of the Viewlink application-layer framing — it just
        captures whatever raw bytes happened to be sitting on the
        gimbal's UART during a short window after we wrote. If the
        gimbal is independently streaming its own status (heartbeat, or
        continuous-ranging updates) on a timer not synchronized with our
        poll, the captured window can start mid-frame, land on the tail
        of one frame plus the start of the next, or contain several
        frames back to back — any of which makes raw[0:3] != HEADER even
        though a valid frame IS present somewhere in the buffer. This is
        almost certainly why parse_reply_frame(reply) alone reports "bad
        header" against a real capture, even when the gimbal did reply.

        Returns a list of raw frame byte-strings (header through
        checksum, inclusive), in the order found. Garbage before the
        first header, a truncated trailing frame, or a header-like byte
        sequence that fails its checksum (a false-positive match) are
        all silently skipped rather than aborting the scan.
        """
        frames = []
        i = 0
        while i <= len(raw) - 5:  # minimum: header(3) + byte3(1) + checksum(1)
            if raw[i:i + 3] != GimbalFrameBuilder.HEADER:
                i += 1
                continue
            if i + 4 > len(raw):
                break  # not enough bytes left even for byte3
            n = raw[i + 3] & 0x3F
            frame_len = 3 + n
            if i + frame_len > len(raw):
                i += 1  # incomplete frame here — keep scanning rather than giving up
                continue
            candidate = raw[i:i + frame_len]
            body = candidate[3:3 + n - 1]
            cs = 0
            for b in body:
                cs ^= b
            if cs == candidate[3 + n - 1]:
                frames.append(bytes(candidate))
                i += frame_len  # jump past this valid frame
            else:
                i += 1  # checksum mismatch — false-positive header match, keep scanning

        return frames

    @staticmethod
    def parse_status_from_buffer(raw: bytes) -> dict:
        """
        Find and parse a T1+F1+B1+D1 status frame (cmd_id 0x40) anywhere
        within a raw captured reply buffer, using find_frames() rather
        than assuming the buffer starts exactly on a frame boundary —
        see find_frames()'s docstring for why that assumption doesn't
        hold against a real SERIAL_CONTROL capture. This is the entry
        point get_laser_range()/poll_status() actually use now.
        """
        if not raw:
            return {'error': 'empty reply — nothing was captured on the gimbal UART in the response window',
                    'checksum_ok': False, 'cmd_id': None}

        frames = GimbalFrameBuilder.find_frames(raw)
        if not frames:
            return {'error': f'no valid Viewlink frame found in {len(raw)}-byte reply buffer: {raw.hex(" ")}',
                    'checksum_ok': False, 'cmd_id': None}

        status_frame = next((f for f in frames if f[4] == 0x40), frames[-1])
        return GimbalFrameBuilder.parse_reply_frame(status_frame)

    @staticmethod
    def parse_reply_frame(raw: bytes) -> dict:
        """
        Parse a raw reply frame captured from the payload (e.g. via
        SERIAL_CONTROL with expect_response=True). Verifies header and
        checksum; if cmd_id is 0x40 (the combined T1+F1+B1+D1 status
        frame), also extracts the laser rangefinder fields from D1.

        Returns a dict:
          cmd_id, checksum_ok            — always present
          error                          — present if parsing failed early
          laser_range_m                  — meters (int), or None if the
                                            reading is invalid/zero
          laser_range_new                — bool; toggles each time a fresh
                                            ranging result lands (ICD D1
                                            byte2 bit0) — compare against
                                            the previous call's value to
                                            tell a new reading from a repeat
          tracking_sensor                — 'EO1'/'IR'/'EO2'/'unknown' (F1)
          tracking_status                — 'stop'/'search'/'tracking'/
                                            'lost'/'unknown' (F1)
        """
        result = {'checksum_ok': False, 'cmd_id': None}
        if len(raw) < 5 or raw[0:3] != GimbalFrameBuilder.HEADER:
            result['error'] = 'bad header'
            return result

        n = raw[3] & 0x3F
        if len(raw) < 3 + n:
            result['error'] = 'frame shorter than declared length'
            return result

        body = raw[3:3 + n - 1]  # byte3 .. last data byte, checksum excluded
        checksum = 0
        for b in body:
            checksum ^= b
        result['checksum_ok'] = (checksum == raw[3 + n - 1])
        result['cmd_id'] = raw[4]

        if raw[4] != 0x40:
            return result  # not the T1+F1+B1+D1 status frame — nothing more to extract here

        data = raw[5:5 + 41]  # T1(22) + F1(1) + B1(6) + D1(12) = 41 bytes
        if len(data) < 41:
            result['error'] = 'T1+F1+B1+D1 data shorter than expected (41 bytes)'
            return result

        # F1 (ICD 3.13) — 1 byte, sits right after T1's 22 bytes.
        f1 = data[22]
        tracking_sensor_code = f1 & 0x07
        tracking_status_code = (f1 >> 3) & 0x03
        result['tracking_sensor'] = {0: 'EO1', 1: 'IR', 2: 'EO2'}.get(tracking_sensor_code, 'unknown')
        result['tracking_status'] = {0: 'stop', 1: 'search', 2: 'tracking', 3: 'lost'}.get(tracking_status_code, 'unknown')

        d1 = data[29:41]  # D1 is the final 12 bytes of the combined payload
        laser_range_new = bool(d1[1] & 0x01)    # D1 byte2, bit0
        rlf_raw = (d1[4] << 8) | d1[5]           # D1 bytes5-6, 1 LSB = 1 meter, 0 = invalid
        result['laser_range_new'] = laser_range_new
        result['laser_range_m'] = rlf_raw if rlf_raw != 0 else None

        return result