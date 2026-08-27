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
# Flight/status only now — every payload row that used to live here
# (zoomin, focus_in, laser_on_off, ...) moved to PAYLOAD_EDGE_TABLE
# below, since that data comes from PayloadCommand, not ButtonState,
# in the current wire protocol.
#
# FIXED: manual/auto were previously wired to Laser*Command — flight
# mode switches firing laser commands. ManualModeCommand/AutoModeCommand
# already existed in intents.py, completely unused. Fixed to actually
# dispatch the commands their names describe.
FLIGHT_EDGE_TABLE = [
    ("arm",       ArmCommand,           DisarmCommand),
    ("manual",    ManualModeCommand,    None),
    ("auto",      AutoModeCommand,      None),
    ("rtl",       RTLCommand,           None),
    ("takeoff",   TakeoffCommand,       None),
    ("autoland",  LandCommand,          None),
    ("speedup",   SpeedUpCommand,       None),
    ("speeddown", SpeedDownCommand,     None),
    ("emergency", EmergencyCommand,     None),
]

# (PayloadCommand field name, rising-edge command, falling-edge command or None)
# All 30 PayloadCommand fields — every bit in the PAYLOAD_COMMAND
# register now has a row here, in the same order as
# bit_definitions.py's PAYLOAD_*_BIT list. Momentary/rate-style actions
# (zoom, focus, dzoom) get a Fall command so they stop when released,
# matching how zoomin/focus_in already worked; pure toggles get None.
PAYLOAD_EDGE_TABLE = [
    ("zoomin",                     ZoomInCommand,                  ZoomInFallCommand),
    ("zoomout",                    ZoomOutCommand,                 ZoomOutFallCommand),
    ("widein",                     FOVPlusCommand,                 FOVPlusFallCommand),
    ("wideout",                    FOVMinusCommand,                FOVMinusFallCommand),
    ("focus_in",                   FocusPlusCommand,               FocusPlusFallCommand),
    ("focus_out",                  FocusMinusCommand,              FocusMinusFallCommand),
    ("laser_on_off",               LaserPowerOnCommand,            LaserPowerOffCommand),
    ("laser_cont_mode",            LaserContModeStartCommand,      LaserContModeStopCommand),
    ("laser_single_mode",          LaserSingleTriggerCommand,      None),
    ("laser_zoom_in",              LaserZoomInCommand,             LaserZoomInFallCommand),
    ("laser_zoom_out",             LaserZoomOutCommand,            LaserZoomOutFallCommand),
    ("tracking_search_on_off",     TrackingStartCommand,           TrackingStopCommand),
    ("ai_tracking_on_off",         AITrackingOnCommand,            AITrackingOffCommand),
    ("tracking_template_toggle",   TrackingTemplateToggleCommand,  None),
    ("tracking_source_toggle",     TrackingSourceToggleCommand,    None),
    ("joystick_track",             JoystickTrackModeOnCommand,     JoystickTrackModeOffCommand),
    ("take_picture",               TakePictureCommand,             None),
    ("start_record",               StartRecordCommand,             None),
    ("stop_record",                StopRecordCommand,              None),
    ("picture_record_mode_toggle", PictureRecordModeToggleCommand, None),
    ("image_sensor_change",        ImageSensorChangeCommand,       None),
    ("ir_polarity",                IRPolarityToggleCommand,        None),
    ("ir_camera_dzoom_plus",       IRCameraDzoomPlusCommand,       IRCameraDzoomPlusFallCommand),
    ("ir_camera_dzoom_minus",      IRCameraDzoomMinusCommand,      IRCameraDzoomMinusFallCommand),
    ("near_infrared_toggle",       NearInfraredToggleCommand,      None),
    ("eo_image_on_off",            EOImageToggleCommand,           None),
    ("motor_on_off",               MotorToggleCommand,             None),
    ("video_ip",                   VideoSourceToggleCommand,       None),
    ("eo_dzoom_toggle",            EODzoomToggleCommand,           None),
    ("ir_rainbow",                 IRRainbowCommand,               None),
]


class InputTranslator:

    def __init__(self, command_bus, state):
        self.command_bus = command_bus
        self.state = state
        
        self._prev = ButtonState(
            arm=False, rtl=False, manual=False, auto=False, takeoff=False,
            emergency=False, autoland=False, speedup=False, speeddown=False,
            menu_select=0,
        )
        self.state.update_Control_Status(self._prev)

        # Same edge-tracking pattern as ButtonState above, for
        # PayloadCommand — needed because PAYLOAD_EDGE_TABLE dispatches
        # on rising/falling edges too, same as FLIGHT_EDGE_TABLE.
        self._prev_payload = PayloadCommand(**{f.name: False for f in dataclass_fields(PayloadCommand)})

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

        # Level value, not edge-detected — store every time, not just on
        # change, same treatment as update_Joystick/update_Payload_Joystick.
        self.state.update_Current_Menu(new.menu_select)

        for field, rising_cmd, falling_cmd in FLIGHT_EDGE_TABLE:
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
        Now does what _translate_buttons does for ButtonState: rising/
        falling-edge dispatch through PAYLOAD_EDGE_TABLE, onto the same
        command_bus. Still also stores the full object and mirrors every
        field onto its own SystemState attribute (all 30, via
        dataclasses.fields() so it can't drift out of sync) — that part
        is unchanged, since gui_state_exporter.py and anything reading
        the *_PRESSED attributes directly still need it.
        """
        prev = self._prev_payload

        for field, rising_cmd, falling_cmd in PAYLOAD_EDGE_TABLE:
            new_val = getattr(obj, field)
            prev_val = getattr(prev, field)

            if new_val and not prev_val and rising_cmd:
                logger.info(f"{field} rising edge -> {rising_cmd.__name__}")
                self.command_bus.put(rising_cmd())
            elif prev_val and not new_val and falling_cmd:
                logger.info(f"{field} falling edge -> {falling_cmd.__name__}")
                self.command_bus.put(falling_cmd())

        self._prev_payload = obj

        self.state.update_Latest_Payload_Command(obj)

        for f in dataclass_fields(obj):
            attr_name = f.name.upper().replace('_', '') + '_PRESSED'
            setattr(self.state, attr_name, getattr(obj, f.name))