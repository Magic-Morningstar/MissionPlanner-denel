# commands/translator.py
#
# The only file allowed to import both serial_controller's decoded types
# AND the shared Command vocabulary. Edge detection lives here.
#
# EDGE_TABLE is the entire mapping from "which switch" to "which Command".
# This is the piece that changes when your button layout changes — and
# now that's a data edit, not a code edit. Adding a button: one new row.
# Reassigning what a button does: change which Command class is in its
# row. Nothing else in this file, or in command_sender.py, changes either
# way — see commands/registry.py for the matching fix on the dispatch side.

import logging
from dataclasses import fields as dataclass_fields
from serial_controller.protocol.messages import ButtonState, Joystick, Joystick2, PayloadCommand  # add Joystick2 to this import
from commands.intents import *
logger = logging.getLogger(__name__)

# (ButtonState field name, rising-edge command, falling-edge command or None)
EDGE_TABLE = [
    ("arm",       ArmCommand,           DisarmCommand),
    ("rtl",       RTLCommand,           None),
    ("takeoff",   TakeoffCommand,       None),
    ("autoland",   LandCommand,         None),
    ("speedup",   SpeedUpCommand,       None),
    ("speeddown",   SpeedDownCommand,   None),
    ("zoomin",   ZoomInCommand,         ZoomInFallCommand),
    ("zoomout",   ZoomOutCommand,       ZoomOutFallCommand),
    ("widein",   FOVPlusCommand,        FOVPlusFallCommand),
    ("wideout",   FOVMinusCommand,      FOVMinusFallCommand),
    ("manual",    LaserStartCommand,    LaserStopCommand),
    ("auto",      ContiousLaserStartCommand,      ContiousLaserStopCommand),
    ("emergency", EmergencyCommand,     None),
    ("tracking",  TrackingStartCommand, TrackingStopCommand),
    ("focus_in",         FocusPlusCommand,          FocusPlusFallCommand),
    ("focus_out",        FocusMinusCommand,         FocusMinusFallCommand),
    ("video_ip",         VideoSourceToggleCommand,  None),
    ("laser_on_off",     LaserPowerOnCommand,       LaserPowerOffCommand),
    ("laser_cont_mode",  LaserContModeStartCommand, LaserContModeStopCommand),
    ("laser_single_mode",LaserSingleTriggerCommand, None),
    ("ai_tracking",      AITrackingOnCommand,       AITrackingOffCommand),
    ("joystick_track",   JoystickTrackModeOnCommand,JoystickTrackModeOffCommand),
    ("ir_polarity",         IRPolarityToggleCommand,   None),
    ("image_sensor_change", ImageSensorChangeCommand,  None),
]


class InputTranslator:

    def __init__(self, command_bus, state):
        self.command_bus = command_bus
        self.state = state
        
        self._prev = ButtonState(
            arm=False, rtl=False, manual=False, takeoff=False,
            emergency=False, autoland = False, auto = False, speedup = False, speeddown = False, 
            zoomin = False, zoomout = False, widein = False, wideout = False,
            tracking = False,
            focus_in = False, focus_out = False, video_ip = False,
            laser_on_off = False, laser_cont_mode = False, laser_single_mode = False,
            ai_tracking = False, joystick_track = False,
            ir_polarity = False, image_sensor_change = False,
        )
        self.state.update_Control_Status(self._prev)

    def handle(self, obj):
        if isinstance(obj, ButtonState):
            self._translate_buttons(obj)
        elif isinstance(obj, Joystick):
            self.state.update_Joystick(obj.x, obj.y)
        elif isinstance(obj, Joystick2):
            self.state.update_Payload_Joystick(obj.x, obj.y) 
        elif isinstance(obj, PayloadCommand):
            self._translate_payload(obj)
        
        

    def _translate_buttons(self, new: ButtonState):
        prev = self._prev
        
        for field, rising_cmd, falling_cmd in EDGE_TABLE:
            new_val = getattr(new, field)
            prev_val = getattr(prev, field)

            if new_val and not prev_val and rising_cmd:
                logger.info(f"{field} rising edge -> {rising_cmd.__name__}")
                self.command_bus.put(rising_cmd())
            elif prev_val and not new_val and falling_cmd:
                logger.info(f"{field} falling edge -> {falling_cmd.__name__}")
                self.command_bus.put(falling_cmd())

        
        self._prev = new
        self.state.update_Control_Status(self._prev)

    def _translate_payload(self, obj: PayloadCommand):
        """
        Stores the full decoded object (for gui_state_exporter.py and
        any future menu-dispatch code), and mirrors EVERY field onto its
        own SystemState attribute — all 30, not a hand-picked subset.
        Uses dataclasses.fields() rather than 30 hardcoded lines, so this
        can't silently drift out of sync if PayloadCommand ever gains or
        loses a field: whatever's actually on the dataclass gets mirrored,
        automatically, using the same name-mangling convention
        system_state.py's __init__ already establishes (field name
        uppercased, underscores collapsed, +_PRESSED — e.g. focus_in ->
        FOCUSIN_PRESSED).

        No edge-detection, no Command dispatch here — unlike
        _translate_buttons above, this isn't wired to an EDGE_TABLE-style
        mapping. That's the menu work being built separately; this just
        gets every field from the wire into state.
        """
        self.state.update_Latest_Payload_Command(obj)

        for f in dataclass_fields(obj):
            attr_name = f.name.upper().replace('_', '') + '_PRESSED'
            setattr(self.state, attr_name, getattr(obj, f.name))