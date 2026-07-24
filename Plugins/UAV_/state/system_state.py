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

        self.ZOOMIN_PRESSED = False
        self.ZOOMING_IN = False
        self.ZOOMING_OUT =False
        self.ZOOMOUT_PRESSED = False

        self.FOCUSIN_PRESSED = False
        self.FOCUSOUT_PRESSED = False


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
