# state/system_state.py
#
# Holds only two kinds of things now:
#   1. Facts both sides need verbatim (armed, mode, altitude, connection
#      status, joystick position, pot value) — no translation needed,
#      safe for anyone to read.
#   2. Nothing else. The raw incoming serial register is GONE — it was
#      the thing leaking serial's internal representation into mavlink
#      code. Discrete button intent now flows through the command_bus
#      (see commands/translator.py + mavlink/uav_commads/command_sender.py)
#      instead of living here.

import threading


class SystemState:

    def __init__(self):
        # ── MAVLink connections ───────────────────────────────────────────────
        self._UAV_STATE_CONNECTION = None
        self._UAV_COMMAND_CONNECTION = None
        self._UAV_STATE_CONNECTION_STATUS = False
        self._UAV_COMMAND_CONNECTION_STATUS = False
        self.UAV_HEARTBEAT = None

        # ── Serial connection ─────────────────────────────────────────────────
        self._SERIAL_CONNECTION = None
        self._SERIAL_CONNECTION_STATUS = False

        # ── UAV observed state ────────────────────────────────────────────────
        self._UAV_ARMED_STATUS = False
        self._UAV_CURRENT_MODE = None
        self._UAV_ALTITUDE = None
        self._UAV_AIR_SPEED = None
        self._UAV_GROUND_SPEED = None
        self._UAV_CURRENT_ITEM = None
        self._UAV_MISSION_ITEMS = []

        # ── Continuous analog inputs — facts, not events ────────────────────
        self._JOYSTICK_X = 0
        self._JOYSTICK_Y = 0
        self._POT_VALUE = 0
        self._PAYLOAD_JOYSTICK_X = 0
        self._PAYLOAD_JOYSTICK_Y = 0

        # ── Discrete payload button status — ALL 30 PayloadCommand fields,
        # mirrored 1:1, not just the handful this used to have. Naming
        # matches the existing convention exactly (field name uppercased,
        # underscores collapsed, +_PRESSED) — e.g. focus_in -> FOCUSIN_PRESSED
        # was already the pattern for the 4 fields that existed before;
        # every other field below follows the same rule, generated
        # mechanically so there's no risk of a typo diverging from
        # PayloadCommand's actual field names.
        self.ZOOMIN_PRESSED = False
        self.ZOOMOUT_PRESSED = False
        self.WIDEIN_PRESSED = False
        self.WIDEOUT_PRESSED = False
        self.FOCUSIN_PRESSED = False
        self.FOCUSOUT_PRESSED = False
        self.LASERONOFF_PRESSED = False
        self.LASERCONTMODE_PRESSED = False
        self.LASERSINGLEMODE_PRESSED = False
        self.LASERZOOMIN_PRESSED = False
        self.LASERZOOMOUT_PRESSED = False
        self.TRACKINGSEARCHONOFF_PRESSED = False
        self.AITRACKINGONOFF_PRESSED = False
        self.TRACKINGTEMPLATETOGGLE_PRESSED = False
        self.TRACKINGSOURCETOGGLE_PRESSED = False
        self.JOYSTICKTRACK_PRESSED = False
        self.TAKEPICTURE_PRESSED = False
        self.STARTRECORD_PRESSED = False
        self.STOPRECORD_PRESSED = False
        self.PICTURERECORDMODETOGGLE_PRESSED = False
        self.IMAGESENSORCHANGE_PRESSED = False
        self.IRPOLARITY_PRESSED = False
        self.IRCAMERADZOOMPLUS_PRESSED = False
        self.IRCAMERADZOOMMINUS_PRESSED = False
        self.NEARINFRAREDTOGGLE_PRESSED = False
        self.EOIMAGEONOFF_PRESSED = False
        self.MOTORONOFF_PRESSED = False
        self.VIDEOIP_PRESSED = False
        self.EODZOOMTOGGLE_PRESSED = False
        self.IRRAINBOW_PRESSED = False

        self.ZOOMING_IN = False
        self.ZOOMING_OUT =False

        # When True, Joystick2 nudges the tracking search cross instead
        # of setting gimbal rate — set by JoystickTrackModeOnCommand /
        # JoystickTrackModeOffCommand, read by AnalogInputHandler.
        self.JOYSTICK_TRACK_MODE = False

        # When either of these is True, AnalogInputHandler suppresses
        # its joystick-rate gimbal keepalive entirely — that keepalive
        # sends A1=SERVO_MANUAL_SPEED every ~0.3s regardless of stick
        # position, which forces the gimbal servo out of tracking-follow
        # mode almost immediately after tracking engages. Set/cleared by
        # tracking_start/stop and ai_tracking_on/off in command_sender.py.
        self.TRACKING_ENGAGED = False
        self.AI_TRACKING_ENGAGED = False


        # ── Operation flags ───────────────────────────────────────────────────
        self.UAV_STATE_CHANGE = False
        self.ARMED_SWITCH = False
        self.SYSTEM_CHECKED = False

        # ── Speed limits ──────────────────────────────────────────────────────
        self.MIN_SPEED = 5.0
        self.MAX_SPEED = 25.0

        # ── Payload options ──────────────────────────────────────────────────────    
        self.GIMBAL_ZOOM  = 0
        self.GIMBAL_FOCUS = 0

        # ── Discrete control status (edge-detected in translator.py) ─────────
        # The last ButtonState translator.py ran edge-detection against —
        # was previously written via a method that didn't exist here,
        # which crashed InputTranslator.__init__ every time. Fixed by
        # actually defining it.
        self._CONTROL_STATUS = None

        # ── Latest PAYLOAD_COMMAND ────────────────────────────────────────────
        # Full decoded PayloadCommand object, most recent one received.
        # This is the "gateway" — the one thing gui_state_exporter.py and
        # any future menu-dispatch code need, so they don't each need
        # their own path from the wire into state. Individual
        # commonly-read fields are also mirrored onto the plain
        # ZOOMIN_PRESSED-style attributes below by translator.py, for
        # code (PayloadAnalogInputHandler) that already reads those
        # directly rather than going through the full object.
        self._LATEST_PAYLOAD_COMMAND = None

        # ── Current menu selected on the physical unit ────────────────────────
        # 0-3, matching ButtonState.menu_select exactly (0=menu1, 1=menu2,
        # 2=menu3, 3=menu4) — decoded from BUTTON_STATE bits 6-7, set by
        # translator.py on every ButtonState received. This is the piece
        # gui_state_exporter.py's "selected menu" field never had a real
        # source for before now.
        self._CURRENT_MENU = 0



        # ── Thread safety ─────────────────────────────────────────────────────
        self._lock = threading.Lock()

    # ── Derived properties ────────────────────────────────────────────────────

    @property
    def is_flying(self):
        if self._UAV_ALTITUDE is not None:
            return self._UAV_ALTITUDE > 1
        return False

    # ── UAV state properties ──────────────────────────────────────────────────

    @property
    def is_UAV_Armed(self):
        return self._UAV_ARMED_STATUS

    @property
    def get_UAV_Current_Mode(self):
        return self._UAV_CURRENT_MODE

    @property
    def get_UAV_Current_Altitude(self):
        return self._UAV_ALTITUDE

    @property
    def get_UAV_Current_Air_Speed(self):
        return self._UAV_AIR_SPEED

    @property
    def get_UAV_Current_Ground_Speed(self):
        return self._UAV_GROUND_SPEED

    # ── Analog input properties ───────────────────────────────────────────────

    @property
    def get_Joystick_X(self):
        return self._JOYSTICK_X

    @property
    def get_Joystick_Y(self):
        return self._JOYSTICK_Y

    @property
    def get_Pot_Value(self):
        return self._POT_VALUE
    
    @property
    def get_Payload_Joystick_X(self):
        return self._PAYLOAD_JOYSTICK_X

    @property
    def get_Payload_Joystick_Y(self):
        return self._PAYLOAD_JOYSTICK_Y

    # ── Serial properties ─────────────────────────────────────────────────────

    @property
    def is_Serial_Connection_Available(self):
        return self._SERIAL_CONNECTION_STATUS

    @property
    def get_Serial_Connection(self):
        return self._SERIAL_CONNECTION

    # ── MAVLink connection properties ─────────────────────────────────────────

    @property
    def is_UAV_State_Connection_Available(self):
        return self._UAV_STATE_CONNECTION_STATUS

    @property
    def is_UAV_Command_Connection_Available(self):
        return self._UAV_COMMAND_CONNECTION_STATUS

    # ── Serial update methods ─────────────────────────────────────────────────

    def update_Serial_connection(self, connected=False, connection=None):
        with self._lock:
            self._SERIAL_CONNECTION = connection
            self._SERIAL_CONNECTION_STATUS = connected

    # ── MAVLink connection update methods ─────────────────────────────────────

    def update_UAV_State_Connection(self, connected=False, connection=None):
        with self._lock:
            self._UAV_STATE_CONNECTION = connection
            self._UAV_STATE_CONNECTION_STATUS = connected

    def update_UAV_Command_Connection(self, connected=False, connection=None):
        with self._lock:
            self._UAV_COMMAND_CONNECTION = connection
            self._UAV_COMMAND_CONNECTION_STATUS = connected

    # ── UAV state update methods ──────────────────────────────────────────────

    def update_UAV_Armed_Status(self, armed=False):
        with self._lock:
            self._UAV_ARMED_STATUS = armed

    def update_UAV_Current_Mode(self, mode):
        with self._lock:
            self._UAV_CURRENT_MODE = mode

    def update_UAV_Altitude(self, altitude):
        with self._lock:
            self._UAV_ALTITUDE = altitude

    def update_UAV_Air_Speed(self, speed):
        with self._lock:
            self._UAV_AIR_SPEED = speed

    def update_UAV_Ground_Speed(self, speed):
        with self._lock:
            self._UAV_GROUND_SPEED = speed

    def update_UAV_Current_Item(self, item):
        with self._lock:
            self._UAV_CURRENT_ITEM = item

    # ── Analog input update methods ─────────────────────────────────────────

    def update_Joystick(self, x, y):
        with self._lock:
            self._JOYSTICK_X = x
            self._JOYSTICK_Y = y

    def update_Payload_Joystick(self, x, y):
        with self._lock:
            self._PAYLOAD_JOYSTICK_X = x
            self._PAYLOAD_JOYSTICK_Y = y

    def update_Pot_Value(self, value):
        with self._lock:
            self._POT_VALUE = value

    # ── Flag management ───────────────────────────────────────────────────────

    def set_UAV_State_Changed(self):
        with self._lock:
            self.UAV_STATE_CHANGE = True

    def clear_UAV_State_Changed(self):
        with self._lock:
            self.UAV_STATE_CHANGE = False

    # ── Control status (ButtonState edge-detection baseline) ─────────────────

    def update_Control_Status(self, status):
        """status is a ButtonState — the last one translator.py ran
        edge-detection against. Was called from translator.py before
        this method existed, which crashed on every InputTranslator
        construction."""
        with self._lock:
            self._CONTROL_STATUS = status

    @property
    def get_Control_Status(self):
        return self._CONTROL_STATUS

    # ── Payload command (gateway) ─────────────────────────────────────────────

    def update_Latest_Payload_Command(self, payload_command):
        """Called by translator.py whenever a new PayloadCommand decodes
        off the wire. Stores the full object — get_latest_payload_command()
        below is the one thing gui_state_exporter.py needs from this
        class, and it's a plain method (not a @property) to match that
        contract exactly."""
        with self._lock:
            self._LATEST_PAYLOAD_COMMAND = payload_command

    def get_latest_payload_command(self):
        return self._LATEST_PAYLOAD_COMMAND

    # ── Current menu ───────────────────────────────────────────────────────────

    def update_Current_Menu(self, menu_select: int):
        """menu_select is the raw 0-3 value from ButtonState — called by
        translator.py on every ButtonState received, unconditionally
        (this is a level/state value, not something to edge-detect)."""
        with self._lock:
            self._CURRENT_MENU = menu_select

    @property
    def get_Current_Menu(self):
        return self._CURRENT_MENU