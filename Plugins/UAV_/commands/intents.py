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
class WideInCommand(Command):
    pass

@dataclass
class WideOutCommand(Command):
    pass

@dataclass
class WideInFallCommand(Command):
    pass

@dataclass
class WideOutFallCommand(Command):
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

# Reserved for when it's wired up — config.py already defines the bit,
# it's just not acted on yet:
#
# @dataclass
# class SystemCheckCommand(Command):
#     pass