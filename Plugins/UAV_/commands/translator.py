# commands/translator.py
#
# The only file allowed to import both serial_controller's decoded types
# AND the shared Command vocabulary. Edge detection lives here.
#
# Two Commands, two buses now: ButtonState (flight/mode fields — arm,
# rtl, takeoff, autoland, speedup, speeddown, manual, auto, emergency)
# routes to flight_command_bus; PayloadCommand (everything camera/laser/
# gimbal/tracking, TLV 0x04) routes to payload_command_bus. This mirrors
# the split on the wire (BUTTON_STATE vs PAYLOAD_COMMAND) and on the
# dispatch side (FlightCommandSender vs PayloadCommandSender) — a
# Command only ever needs to reach the one sender that actually knows
# what to do with it.
#
# Each EDGE_TABLE is the entire mapping from "which field" to "which
# Command". Adding a button: one new row in the matching table.
# Reassigning what a button does: change which Command class is in its
# row. Nothing else in this file, or in either sender file, changes
# either way — see commands/registry.py for the matching dispatch-side
# mechanism.

import logging
from serial_controller.protocol.messages import ButtonState, PayloadCommand, Joystick, Joystick2
from commands.intents import *
logger = logging.getLogger(__name__)

# ── ButtonState (BUTTON_STATE, 0x01) -> flight_command_bus ──────────────────
# Only the fields ButtonState still actually carries after the
# PAYLOAD_COMMAND split — see TLV_PROTOCOL.md's §4.1 for which fields on
# the dataclass are now dead weight (firmware no longer writes them).
# "manual" and "auto" previously fired LaserStartCommand/
# ContiousLaserStartCommand here — a flight-mode switch triggering a
# payload/laser command, almost certainly leftover wiring from before
# this split existed. Fixed to fire ManualModeCommand/AutoModeCommand,
# which were already defined in intents.py but never used.
FLIGHT_EDGE_TABLE = [
    ("arm",       ArmCommand,        DisarmCommand),
    ("rtl",       RTLCommand,        None),
    ("takeoff",   TakeoffCommand,    None),
    ("autoland",  LandCommand,       None),
    ("speedup",   SpeedUpCommand,    None),
    ("speeddown", SpeedDownCommand,  None),
    ("manual",    ManualModeCommand, None),   # FIXED: was LaserStartCommand
    ("auto",      AutoModeCommand,   None),   # FIXED: was ContiousLaserStartCommand
    ("emergency", EmergencyCommand,  None),
]

# ── PayloadCommand (PAYLOAD_COMMAND, 0x04) -> payload_command_bus ───────────
# All 30 fields. Where a field is a direct continuation of an old
# ButtonState-era control (zoomin, tracking_search_on_off, etc.), it
# reuses that Command class rather than inventing a new one — the
# button's *meaning* didn't change, only which register it's decoded
# from.
PAYLOAD_EDGE_TABLE = [
    ("zoomin",                     ZoomInCommand,               ZoomInFallCommand),
    ("zoomout",                    ZoomOutCommand,               ZoomOutFallCommand),
    ("widein",                     FOVPlusCommand,               FOVPlusFallCommand),
    ("wideout",                    FOVMinusCommand,              FOVMinusFallCommand),
    ("focus_in",                   FocusPlusCommand,             FocusPlusFallCommand),
    ("focus_out",                  FocusMinusCommand,            FocusMinusFallCommand),
    ("laser_on_off",               LaserPowerOnCommand,          LaserPowerOffCommand),
    ("laser_cont_mode",            LaserContModeStartCommand,    LaserContModeStopCommand),
    ("laser_single_mode",          LaserSingleTriggerCommand,    None),
    ("laser_zoom_in",              LaserZoomInCommand,           None),
    ("laser_zoom_out",             LaserZoomOutCommand,          None),
    ("tracking_search_on_off",     TrackingStartCommand,         TrackingStopCommand),
    ("ai_tracking_on_off",         AITrackingOnCommand,          AITrackingOffCommand),
    ("tracking_template_toggle",   TrackingTemplateToggleCommand, None),
    ("tracking_source_toggle",     TrackingSourceToggleCommand,  None),
    ("joystick_track",             JoystickTrackModeOnCommand,   JoystickTrackModeOffCommand),
    ("take_picture",               TakePictureCommand,           None),
    ("start_record",               StartRecordCommand,           None),
    ("stop_record",                StopRecordCommand,            None),
    ("picture_record_mode_toggle", PictureRecordModeToggleCommand, None),
    ("image_sensor_change",        ImageSensorChangeCommand,     None),
    ("ir_polarity",                IRPolarityToggleCommand,      None),
    ("ir_camera_dzoom_plus",       IRCameraDzoomPlusCommand,     None),
    ("ir_camera_dzoom_minus",      IRCameraDzoomMinusCommand,    None),
    ("near_infrared_toggle",       NearInfraredToggleCommand,    None),
    ("eo_image_on_off",            EOImageToggleCommand,         None),
    ("motor_on_off",               MotorToggleCommand,           None),
    ("video_ip",                   VideoSourceToggleCommand,     None),
    ("eo_dzoom_toggle",            EODzoomToggleCommand,         None),
    ("ir_rainbow",                 IRRainbowCommand,              None),
]


class InputTranslator:

    def __init__(self, flight_command_bus, payload_command_bus, state):
        self.flight_command_bus = flight_command_bus
        self.payload_command_bus = payload_command_bus
        self.state = state

        # Fixed: this used to construct ButtonState with laser_on_off=False,
        # a field that no longer exists on the dataclass — that field
        # migrated to PayloadCommand when PAYLOAD_COMMAND was split out.
        # Left as-is it crashed __init__ immediately on every connect.
        self._prev_buttons = ButtonState(
            arm=False, rtl=False, manual=False, takeoff=False,
            emergency=False, autoland=False, auto=False, speedup=False, speeddown=False,
            zoomin=False, zoomout=False, widein=False, wideout=False,
            tracking=False,
            focus_in=False, focus_out=False, video_ip=False,
            laser_cont_mode=False, laser_single_mode=False,
            ai_tracking=False, joystick_track=False,
            ir_polarity=False, image_sensor_change=False,
        )

        # New: PayloadCommand never had edge-detection state at all before
        # this — PayloadCommand objects arrived from serial_handler.py but
        # handle() below had no branch for them, so they were silently
        # dropped. All 30 fields default False (matches PAYLOAD_MESSAGE's
        # post-sync-handshake reset to 0 on the STM32 side — see
        # TLV_PROTOCOL.md §5).
        self._prev_payload = PayloadCommand(
            zoomin=False, zoomout=False, widein=False, wideout=False,
            focus_in=False, focus_out=False,
            laser_on_off=False, laser_cont_mode=False, laser_single_mode=False,
            laser_zoom_in=False, laser_zoom_out=False,
            tracking_search_on_off=False, ai_tracking_on_off=False,
            tracking_template_toggle=False, tracking_source_toggle=False,
            joystick_track=False,
            take_picture=False, start_record=False, stop_record=False,
            picture_record_mode_toggle=False,
            image_sensor_change=False, ir_polarity=False,
            ir_camera_dzoom_plus=False, ir_camera_dzoom_minus=False,
            near_infrared_toggle=False, eo_image_on_off=False,
            motor_on_off=False, video_ip=False,
            eo_dzoom_toggle=False, ir_rainbow=False,
        )

    def handle(self, obj):
        if isinstance(obj, ButtonState):
            self._translate_edges(obj, self._prev_buttons, FLIGHT_EDGE_TABLE, self.flight_command_bus)
            self._prev_buttons = obj
        elif isinstance(obj, PayloadCommand):
            self._translate_edges(obj, self._prev_payload, PAYLOAD_EDGE_TABLE, self.payload_command_bus)
            self._prev_payload = obj
        elif isinstance(obj, Joystick):
            self.state.update_Joystick(obj.x, obj.y)
        elif isinstance(obj, Joystick2):
            self.state.update_Payload_Joystick(obj.x, obj.y)

    def _translate_edges(self, new, prev, edge_table, bus):
        for field, rising_cmd, falling_cmd in edge_table:
            new_val = getattr(new, field)
            prev_val = getattr(prev, field)

            if new_val and not prev_val and rising_cmd:
                logger.info(f"{field} rising edge -> {rising_cmd.__name__}")
                bus.put(rising_cmd())
            elif prev_val and not new_val and falling_cmd:
                logger.info(f"{field} falling edge -> {falling_cmd.__name__}")
                bus.put(falling_cmd())