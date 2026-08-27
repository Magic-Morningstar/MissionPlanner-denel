# main.py

import threading
import queue
import time
import signal
from state.system_state import SystemState
from state.watchdog import Watchdog
from serial_controller.serial_handler import SerialHandler
from API.panel_sync import PanelStateSync
from mavlink.mavlink_handler import Mavlink_controller
from commands.translator import InputTranslator
from PySide6.QtCore import QTimer
from utils.logging_config import setup_logging
# Assumes gui_main.py lives alongside main.py — adjust if it's elsewhere.
from Menu_UI.gui_main import create_gui
import logging
logger = logging.getLogger(__name__)


class Controller:

    def __init__(self):
        self.state = SystemState()
        self.watchdog = Watchdog(state=self.state)
        self.command_bus = queue.Queue(maxsize=256)
        self.translator = InputTranslator(self.command_bus, self.state)
        self.Mavlink_controller = Mavlink_controller(self.state, self.command_bus, self.watchdog)
        self.SerialHandler = SerialHandler(self.state, self.translator, self.watchdog)
        self.panel_sync = PanelStateSync(self.state)
        self.app = None   # This the Front end object. It it set in the start function
        self._shutdown = threading.Event


    def start(self):
        signal.signal(signal.SIGINT, self._on_shutdown)
        signal.signal(signal.SIGTERM, self._on_shutdown)
        
        self.SerialHandler.connect()
        self.Mavlink_controller.connect()
        self.panel_sync.start()
        self.watchdog.start()
        logger.info("Controller started.")



        self.app, self.gui_window = create_gui()
        signal_pump = QTimer()
        signal_pump.timeout.connect(lambda: None)
        signal_pump.start(200)

        self.app.exec()

        logger.info("Shutting down.")

    def _on_shutdown(self, sig, frame):
        logger.info("Shutdown signal received.")
        self.SerialHandler.disconnect()
        self.Mavlink_controller.disconnect()
        self.panel_sync.stop()
        self._shutdown.set()
        if self.app is not None:
            self.app.quit()   # unblocks app.exec() in start()


if __name__ == "__main__":
    setup_logging()   # must run before any other module logs anything
    app = Controller()
    app.start()