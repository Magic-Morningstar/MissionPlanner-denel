# commands/registry.py
#
# Command -> handler registry. Deliberately the same shape as
# serial_controller/protocol/registry.py — you already validated this
# pattern once for wire message types; this is the second, symmetric
# application of it, one layer up, for Commands.
#
# Adding a new button/command:
#   1. Add the Command dataclass to commands/intents.py
#   2. Write one handler function, decorate it with @register_handler(...)
#   3. Done. process_command() and UAVCommandSender never change.

import logging

logger = logging.getLogger(__name__)

_HANDLERS = {}


def register_handler(command_type):
    """Decorator: @register_handler(ArmCommand) above a function."""
    def decorator(fn):
        if command_type in _HANDLERS:
            raise ValueError(f"Duplicate handler for {command_type.__name__}")
        _HANDLERS[command_type] = fn
        return fn
    return decorator


def dispatch(cmd, sender):
    """sender is the UAVCommandSender instance — handlers get it as their
    first arg so they can call sender.arm(), sender.set_rtl(), etc."""
    handler = _HANDLERS.get(type(cmd))
    if handler is None:
        logger.warning(f"No handler registered for {type(cmd).__name__}")
        return
    handler(sender, cmd)
