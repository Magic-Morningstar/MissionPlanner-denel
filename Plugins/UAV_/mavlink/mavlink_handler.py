# mavlink/mavlink_handler.py

from utils.helper import *
from mavlink.state_update.state_poller import UAVStatePoller
from mavlink.Flight_controller_commands.flight_command_sender import FlightCommandSender
from mavlink.payload_commands.payload_command_sender import PayloadCommandSender
from config import *


class Mavlink_controller:
    """
    Orchestrator — creates both dispatchers and wires the two command
    buses into them. FlightCommandSender is the one that actually owns
    the MAVLink connection; PayloadCommandSender is attached to it right
    after connecting (see FlightCommandSender._do_connect) rather than
    opening a second connection of its own.
    """

    def __init__(self, STATE, flight_command_bus, payload_command_bus, watchdog=None):
        self.state = STATE
        self.payload_sender = PayloadCommandSender(STATE, payload_command_bus)
        self.flight_sender = FlightCommandSender(
            MAVLINK_COMMAND_PORT, STATE, flight_command_bus, watchdog,
            interval=0.05, payload_sender=self.payload_sender,
        )
        self.state_poller = UAVStatePoller(
            MAVLINK_STATE_PORT, STATE, watchdog, interval=1.0
        )

        # Kept for anything external still referencing the old name —
        # command_sender used to be the single class doing everything
        # flight_sender does now.
        self.command_sender = self.flight_sender

    def connect(self):
        #self.state_poller.connect()
        self.flight_sender.connect()

    def disconnect(self):
        self.state_poller.stop()
        self.flight_sender.stop()

    def execute_commands(self):
        """
        Called every main-loop tick when the command connection is up.
        Draining empty queues is cheap. Note FlightCommandSender's own
        background loop (_run_once(), every `interval` seconds) already
        drains both buses + both command_queues on its own schedule —
        this call is a secondary/redundant drain at the slower main-loop
        cadence, same as it was before the split (see the original
        UAVCommandSender's _run_once for the equivalent double-drain).
        """
        if not self.state.is_UAV_Command_Connection_Available:
            return
        self.flight_sender.drain_input_commands()
        self.payload_sender.drain_input_commands()