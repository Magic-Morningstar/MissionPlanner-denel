# main.py

import threading
import queue
import time
import signal
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

            # ── MAVLink commands — drain whatever's pending on the bus ────────
            if self.state.is_UAV_Command_Connection_Available:
                self.Mavlink_controller.execute_commands()
            elif not self.Mavlink_controller.command_sender.connection_in_progress:
                logger.info("MAVLink command connection lost — reconnecting...")
                self.Mavlink_controller.command_sender.connect()

            # ── MAVLink state poller reconnect ────────────────────────────────
            if not self.state.is_UAV_State_Connection_Available:
                if not self.Mavlink_controller.state_poller.connection_in_progress:
                    logger.info("MAVLink state connection lost — reconnecting...")
                    self.Mavlink_controller.state_poller.connect()

            # ── Send state back to STM32 only when UAV state changed ──────────
            if self.state.is_Serial_Connection_Available:
                if self.state.UAV_STATE_CHANGE:
                    self.SerialHandler.compile_Send()

            time.sleep(0.01)

        logger.info("Shutting down.")

    def _on_shutdown(self, sig, frame):
        logger.info("Shutdown signal received.")
        self.SerialHandler.disconnect()
        self.Mavlink_controller.disconnect()
        self._shutdown.set()


if __name__ == "__main__":
    setup_logging()   # must run before any other module logs anything
    app = Controller()
    app.start()
