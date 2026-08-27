# commands/intents.py
#
# The shared vocabulary that crosses the serial <-> mavlink boundary.
# Deliberately knows nothing about buttons, bits, or the STM32 — and
# mavlink/ code is not allowed to know anything about serial internals
# either. This file is the only thing both sides are allowed to import.
#
# Rule of thumb: nothing in serial_controller/ should ever appear in
# mavlink/, and nothing in mavlink/ should ever appear in
# serial_controller/. Only these types cross.
#
# New discrete instruction? Add a dataclass here, add one branch to
# InputTranslator, add one branch to UAVCommandSender.process_command.
# That's the whole cost of a new command, end to end.

from dataclasses import dataclass


class Command:
    """Marker base class — every intent that can cross the boundary."""
    pass


@dataclass
class ArmCommand(Command):
    pass


@dataclass
class DisarmCommand(Command):
    pass


@dataclass
class TakeoffCommand(Command):
    pass


@dataclass
class ManualModeCommand(Command):
    pass


@dataclass
class LandCommand(Command):
    pass


@dataclass
class SpeedUpCommand(Command):
    pass

@dataclass
class SpeedDownCommand(Command):
    pass

@dataclass
class ZoomInCommand(Command):
    pass

@dataclass
class ZoomOutCommand(Command):
    pass

@dataclass
class ZoomInFallCommand(Command):
    pass

@dataclass
class ZoomOutFallCommand(Command):
    pass

@dataclass
class FOVPlusCommand(Command):
    pass

@dataclass
class FOVMinusCommand(Command):
    pass

@dataclass
class FOVPlusFallCommand(Command):
    pass

@dataclass
class FOVMinusFallCommand(Command):
    pass

@dataclass
class RTLCommand(Command):
    pass

@dataclass
class AutoModeCommand(Command):
    pass


@dataclass
class EmergencyCommand(Command):
    pass

@dataclass
class LaserStartCommand(Command):
    pass

@dataclass
class LaserStopCommand(Command):
    pass

class ContiousLaserStopCommand(Command):
    pass

class ContiousLaserStartCommand(Command):
    pass

@dataclass
class TrackingStartCommand(Command):
    pass

@dataclass
class TrackingStopCommand(Command):
    pass

@dataclass
class FocusPlusCommand(Command):
    pass

@dataclass
class FocusPlusFallCommand(Command):
    pass

@dataclass
class FocusMinusCommand(Command):
    pass

@dataclass
class FocusMinusFallCommand(Command):
    pass

@dataclass
class VideoSourceToggleCommand(Command):
    pass

@dataclass
class LaserPowerOnCommand(Command):
    pass

@dataclass
class LaserPowerOffCommand(Command):
    pass

@dataclass
class LaserContModeStartCommand(Command):
    pass

@dataclass
class LaserContModeStopCommand(Command):
    pass

@dataclass
class LaserSingleTriggerCommand(Command):
    pass

@dataclass
class AITrackingOnCommand(Command):
    pass

@dataclass
class AITrackingOffCommand(Command):
    pass

@dataclass
class JoystickTrackModeOnCommand(Command):
    pass

@dataclass
class JoystickTrackModeOffCommand(Command):
    pass

@dataclass
class IRPolarityToggleCommand(Command):
    pass

@dataclass
class ImageSensorChangeCommand(Command):
    pass


# ── Added: the payload concepts that had a PAYLOAD_COMMAND bit but no
# Command class yet — PAYLOAD_EDGE_TABLE in translator.py needed these
# to exist before it could cover all 30 PayloadCommand fields. ──────────

@dataclass
class LaserZoomInCommand(Command):
    pass

@dataclass
class LaserZoomInFallCommand(Command):
    pass

@dataclass
class LaserZoomOutCommand(Command):
    pass

@dataclass
class LaserZoomOutFallCommand(Command):
    pass

@dataclass
class TrackingTemplateToggleCommand(Command):
    pass

@dataclass
class TrackingSourceToggleCommand(Command):
    pass

@dataclass
class TakePictureCommand(Command):
    pass

@dataclass
class StartRecordCommand(Command):
    pass

@dataclass
class StopRecordCommand(Command):
    pass

@dataclass
class PictureRecordModeToggleCommand(Command):
    pass

@dataclass
class IRCameraDzoomPlusCommand(Command):
    pass

@dataclass
class IRCameraDzoomPlusFallCommand(Command):
    pass

@dataclass
class IRCameraDzoomMinusCommand(Command):
    pass

@dataclass
class IRCameraDzoomMinusFallCommand(Command):
    pass

@dataclass
class NearInfraredToggleCommand(Command):
    pass

@dataclass
class EOImageToggleCommand(Command):
    pass

@dataclass
class MotorToggleCommand(Command):
    pass

@dataclass
class EODzoomToggleCommand(Command):
    pass

@dataclass
class IRRainbowCommand(Command):
    pass


# Reserved for when it's wired up — config.py already defines the bit,
# it's just not acted on yet:
#
# @dataclass
# class SystemCheckCommand(Command):
#     pass