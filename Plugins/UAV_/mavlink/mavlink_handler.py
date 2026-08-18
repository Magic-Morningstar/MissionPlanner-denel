# mavlink/mavlink_handler.py

from utils.helper import *
from mavlink.state_update.state_poller import UAVStatePoller
from mavlink.Flight_controller_commands.flight_command_sender import FlightCommandSender
from mavlink.payload_commands.payload_command_sender import PayloadCommandSender
from config import *
import logging
logger = logging.getLogger(__name__)


class Mavlink_controller:
    """
    Orchestrator — creates both dispatchers and wires the two command
    buses into them. FlightCommandSender is the one that actually owns
    the MAVLink connection used for commands; PayloadCommandSender is
    attached to it right after connecting (see
    FlightCommandSender._open_connection) rather than opening a second
    connection of its own.

    UAVStatePoller does open a second, separate connection. The two are
    distinguished by MAVLink identity — poller 200/190, command sender
    200/191 — not by port. Both used to sit on pymavlink's default 255/0,
    which made them one logical GCS at two addresses as far as the
    autopilot and the Herelink router were concerned.

    MAVLINK_STATE_PORT and MAVLINK_COMMAND_PORT must be endpoints that
    both actually carry vehicle traffic. Two different ports pointed
    straight at the Herelink will not work — the ground unit serves one
    endpoint (14552 by default), so either point both connection strings
    at it, or front the link with MAVProxy/mavlink-router and give each
    worker its own local --out.
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
        """
        The state poller connect used to be commented out here. Nothing
        else writes is_UAV_Armed, get_UAV_Current_Mode, is_flying or
        altitude, so with it disabled:

          * emergency_stop() always took the "on the ground" branch and
            disarmed, including in flight
          * arm()'s "already armed — skipping" guard never fired
          * _switch_to_fbwb() read a mode that never updated

        If you need to run without the poller for some reason, note that
        emergency_stop() now fails safe to RTL on unknown state — but the
        other two are still degraded, so don't.
        """
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