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
from serial_controller.protocol.messages import ButtonState, Joystick, Joystick2  # add Joystick2 to this import
from commands.intents import *
logger = logging.getLogger(__name__)

# (ButtonState field name, rising-edge command, falling-edge command or None)
EDGE_TABLE = [
    ("arm",       TrackingStartCommand,           TrackingStopCommand),
    ("rtl",       RTLCommand,           None),
    ("takeoff",   TakeoffCommand,       None),
    ("autoland",   LandCommand,         None),
    ("speedup",   SpeedUpCommand,       None),
    ("speeddown",   SpeedDownCommand,   None),
    ("zoomin",   ZoomInCommand,         ZoomInFallCommand),
    ("zoomout",   ZoomOutCommand,       ZoomOutFallCommand),
    ("widein",   WideInCommand,         WideInFallCommand),
    ("wideout",   WideOutCommand,       WideOutFallCommand),
    ("manual",    LaserStartCommand,    LaserStopCommand),
    ("auto",      ContiousLaserStartCommand,      ContiousLaserStopCommand),
    ("emergency", EmergencyCommand,     None),
]


class InputTranslator:

    def __init__(self, command_bus, state):
        self.command_bus = command_bus
        self.state = state
        self._prev = ButtonState(
            arm=False, rtl=False, manual=False, takeoff=False,
            emergency=False, autoland = False, auto = False, speedup = False, speeddown = False, 
            zoomin = False, zoomout = False, widein = False, wideout = False,
        )

    def handle(self, obj):
        if isinstance(obj, ButtonState):
            self._translate_buttons(obj)
        elif isinstance(obj, Joystick):
            self.state.update_Joystick(obj.x, obj.y)
        elif isinstance(obj, Joystick2):
            self.state.update_Payload_Joystick(obj.x, obj.y) 
        
        

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