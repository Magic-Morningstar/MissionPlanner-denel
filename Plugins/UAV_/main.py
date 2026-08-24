# main.py

import threading
import queue
import time
import signal
from state.system_state import SystemState
from state.watchdog import Watchdog
from serial_controller.serial_handler import SerialHandler
from API.json_api_updater import *
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

        # GuiStateExporter only ever reads from self.state — same
        # boundary rule as everything else that touches SystemState.
        # template_path is the static schema (checked into the repo,
        # never overwritten); json_path is the live file the GUI
        # actually polls, rewritten on every update. Adjust these two
        # paths to wherever you actually want them to live.
        self.gui_exporter = GuiStateExporter(
            self.state,
            json_path="menu_info.json",
            template_path="",
        )

        self._shutdown = threading.Event()

    def start(self):
        signal.signal(signal.SIGINT, self._on_shutdown)
        signal.signal(signal.SIGTERM, self._on_shutdown)
        
        self.SerialHandler.connect()
        self.Mavlink_controller.connect()
        self.gui_exporter.start()
        self.watchdog.start()
        logger.info("Controller started.")

        while not self._shutdown.is_set():

            time.sleep(2)

        logger.info("Shutting down.")

    def _on_shutdown(self, sig, frame):
        logger.info("Shutdown signal received.")
        self.SerialHandler.disconnect()
        self.Mavlink_controller.disconnect()
        self.gui_exporter.stop()
        self._shutdown.set()


if __name__ == "__main__":
    setup_logging()   # must run before any other module logs anything
    app = Controller()
    app.start()