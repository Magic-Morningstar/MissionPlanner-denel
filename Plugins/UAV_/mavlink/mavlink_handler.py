# mavlink/mavlink_handler.py

from utils.helper import *
from mavlink.state_update.state_poller import UAVStatePoller
from mavlink.uav_commads.command_sender import UAVCommandSender
from state.system_config import *


class Mavlink_controller:
    """Thin orchestrator — creates the two workers, wires the shared
    command_bus into the command sender. No longer inspects the raw
    serial message at all."""

    def __init__(self, STATE, command_bus, watchdog=None):
        self.state = STATE
        self.command_sender = UAVCommandSender(
            MAVLINK_COMMAND_PORT, STATE, command_bus, watchdog, interval=0.05
        )
        self.state_poller = UAVStatePoller(
            MAVLINK_STATE_PORT, STATE, watchdog, interval=1.0
        )

    def connect(self):
        #self.state_poller.connect()
        self.command_sender.connect()

    def disconnect(self):
        self.state_poller.stop()
        self.command_sender.stop()

    def execute_commands(self):
        """Called every main-loop tick when the command connection is up.
        Draining an empty queue is cheap, so no need to gate this on a
        'did anything change' flag anymore — the queue itself is empty
        when there's nothing to do."""
        if not self.state.is_UAV_Command_Connection_Available:
            return
        self.command_sender.drain_input_commands()