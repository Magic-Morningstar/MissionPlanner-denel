# main.py

import threading
import queue
import time
import signal
import os
from state.system_state import SystemState
from state.watchdog import Watchdog
from serial_controller.serial_handler import SerialHandler
from mavlink.mavlink_handler import Mavlink_controller
from commands.translator import InputTranslator
from logging_config import setup_logging
from status_reporter import StatusReporter
import logging
logger = logging.getLogger(__name__)


class Controller:

    def __init__(self):
        self.state = SystemState()
        self.watchdog = Watchdog(state=self.state)

        # Two buses now, not one — the boundary between serial and
        # mavlink split along the same line as BUTTON_STATE/PAYLOAD_COMMAND
        # on the wire and FlightCommandSender/PayloadCommandSender on the
        # dispatch side. InputTranslator routes ButtonState-derived edges
        # onto flight_command_bus and PayloadCommand-derived edges onto
        # payload_command_bus — see commands/translator.py's two
        # EDGE_TABLEs. Both bounded for the same reason the single bus
        # was: cheap backpressure insurance if a stalled mavlink side
        # can't keep up with incoming button presses.
        self.flight_command_bus = queue.Queue(maxsize=256)
        self.payload_command_bus = queue.Queue(maxsize=256)
        self.translator = InputTranslator(self.flight_command_bus, self.payload_command_bus, self.state)
        self.Mavlink_controller = Mavlink_controller(
            self.state, self.flight_command_bus, self.payload_command_bus, self.watchdog
        )
        self.SerialHandler = SerialHandler(self.state, self.translator, self.watchdog)
        self.status_reporter = StatusReporter(self.state)
        self._shutdown = threading.Event()

    def start(self):
        signal.signal(signal.SIGINT, self._on_shutdown)
        signal.signal(signal.SIGTERM, self._on_shutdown)

        self.SerialHandler.connect()
        self.Mavlink_controller.connect()
        self.watchdog.start()
        self.status_reporter.start()
        logger.info("Controller started.")

        while not self._shutdown.is_set():
            time.sleep(2)

        logger.info("Shutting down.")

    def _on_shutdown(self, sig, frame):
        logger.info("Shutdown signal received.")
        self.SerialHandler.disconnect()
        self.Mavlink_controller.disconnect()
        self.status_reporter.stop()
        self._shutdown.set()


if __name__ == "__main__":
    # ProgramData is writable by a standard (non-admin) user, unlike an install
    # under Program Files — controller.log must not be written relative to cwd
    # (this app's cwd when installed) or the write throws PermissionError before
    # any bridge logic runs.
    log_dir = os.path.join(os.environ.get("PROGRAMDATA", "."), "Denel GCS")
    os.makedirs(log_dir, exist_ok=True)
    setup_logging(log_file=os.path.join(log_dir, "controller.log"))   # must run before any other module logs anything
    app = Controller()
    app.start()