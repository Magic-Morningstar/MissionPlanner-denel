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
import logging
logger = logging.getLogger(__name__)


class Controller:

    def __init__(self):
        self.state = SystemState()
        self.watchdog = Watchdog(state=self.state)

        # The boundary itself — the only thing that crosses between
        # serial and mavlink. Bounded so a stalled mavlink side can't
        # let this grow unbounded if button presses come in faster than
        # they're drained (backpressure, not currently hit in practice
        # but cheap insurance).
        self.command_bus = queue.Queue(maxsize=256)

        self.translator = InputTranslator(self.command_bus, self.state)
        self.Mavlink_controller = Mavlink_controller(self.state, self.command_bus, self.watchdog)
        self.SerialHandler = SerialHandler(self.state, self.translator, self.watchdog)
        self._shutdown = threading.Event()

    def start(self):
        signal.signal(signal.SIGINT, self._on_shutdown)
        signal.signal(signal.SIGTERM, self._on_shutdown)

        self.SerialHandler.connect()
        self.Mavlink_controller.connect()
        self.watchdog.start()
        logger.info("Controller started.")

        while not self._shutdown.is_set():

            time.sleep(2)

        logger.info("Shutting down.")

    def _on_shutdown(self, sig, frame):
        logger.info("Shutdown signal received.")
        self.SerialHandler.disconnect()
        self.Mavlink_controller.disconnect()
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
