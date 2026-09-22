# main.py

import threading
import queue
import time
import signal
import sys
from state.system_state import SystemState
from state.watchdog import Watchdog
from serial_controller.serial_handler import SerialHandler
from API.panel_sync import PanelStateSync
from mavlink.mavlink_handler import Mavlink_controller
from commands.translator import InputTranslator
from PySide6.QtCore import QTimer
from logging_config import setup_logging
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
        self._shutdown = threading.Event()   # was missing its call — this bound the class, not an instance
        self._shutdown_done = False


    def start(self):
        signal.signal(signal.SIGINT, self._on_shutdown)
        signal.signal(signal.SIGTERM, self._on_shutdown)
        # start(), not connect(): keeps reconnecting if the panel is
        # unplugged, power-cycled, or comes back on a different port.
        self.SerialHandler.start()
        self.Mavlink_controller.connect()
        self.panel_sync.start()
        self.watchdog.start()
        logger.info("Controller started.")

        self._start_stdin_watcher()

        self.app, self.gui_window = create_gui()

        # Runs on the Qt thread every 200 ms. It already existed to let
        # Python signal handlers fire while app.exec() blocks in C++; it
        # now also checks for a shutdown request from another thread.
        # QApplication.quit() must be called from the Qt thread, so the
        # stdin watcher sets an Event and this is what acts on it.
        signal_pump = QTimer()
        signal_pump.timeout.connect(self._poll_shutdown)
        signal_pump.start(200)

        self.app.exec()

        # app.exec() returns when the GUI window is closed, which is the
        # ordinary way this program ends — SIGINT/SIGTERM are the rare
        # path. Without this, closing the window skipped _on_shutdown
        # entirely: no GOODBYE reached the STM32 and the heartbeat thread
        # died with the process, so the panel would blink red instead of
        # returning to amber.
        logger.info("Shutting down.")
        self._shutdown_once()

    def _shutdown_once(self):
        """Safe to call twice — a SIGINT arriving during window close
        would otherwise disconnect an already-disconnected handler."""
        if self._shutdown_done:
            return
        self._shutdown_done = True

        # Serial FIRST, and each step isolated. GOODBYE is the one thing
        # the panel can observe, so it must not be blocked by a MAVLink
        # controller that was never connected (its connect() is commented
        # out in start()) raising or hanging on disconnect.
        for name, step in (
            ("serial",  self.SerialHandler.shutdown),   # sends GOODBYE
            ("mavlink", self.Mavlink_controller.disconnect),
            ("panel",   self.panel_sync.stop),
        ):
            try:
                step()
                logger.info(f"Shutdown: {name} stopped.")
            except Exception:
                logger.exception(f"Shutdown: {name} failed to stop cleanly")

    def _poll_shutdown(self):
        if self._shutdown.is_set() and self.app is not None:
            self.app.quit()

    def _start_stdin_watcher(self):
        """Graceful shutdown when launched by DenelPythonLauncher.

        The C# plugin starts this process with CreateNoWindow, so there is
        no console and no Ctrl+C or console-close event to catch. Instead
        it writes "shutdown" to our stdin and closes the pipe. Either one
        triggers a clean exit here — which is what gets GOODBYE to the
        panel, so it shows amber rather than red when Mission Planner
        closes.

        EOF is the important case: it arrives when the parent closes the
        pipe AND when the parent dies outright, so even a Mission Planner
        crash gets a clean shutdown here.

        Only armed when stdin is a pipe. Run from a terminal, stdin is a
        TTY and Ctrl+C already works, so there's nothing to add."""
        try:
            if sys.stdin is None or sys.stdin.isatty():
                logger.info("StdinWatcher: not armed (stdin is a terminal) — use Ctrl+C.")
                return
        except Exception:
            logger.info("StdinWatcher: not armed (no usable stdin).")
            return

        # If this line is missing from the log, the deployed main.py is an
        # older copy without the watcher — the most common reason graceful
        # shutdown "doesn't work".
        logger.info("StdinWatcher: armed, waiting for launcher shutdown request.")

        def _watch():
            try:
                for line in sys.stdin:
                    if line.strip().lower() == "shutdown":
                        logger.info("Shutdown requested by launcher.")
                        break
                else:
                    logger.info("Launcher closed stdin — shutting down.")
            except Exception:
                logger.info("stdin watcher ended — shutting down.")

            try:
                self._shutdown_once()     # sends GOODBYE while the port is still open
            except Exception:
                # One subsystem failing to stop must not prevent the rest,
                # or prevent Qt quitting — that would hang until the
                # launcher's timeout kills us, which is the exact symptom
                # this whole path exists to avoid.
                logger.exception("StdinWatcher: error during shutdown")
            self._shutdown.set()      # _poll_shutdown quits Qt on its own thread

        threading.Thread(target=_watch, daemon=True, name="StdinWatcher").start()

    def _on_shutdown(self, sig, frame):
        logger.info("Shutdown signal received.")
        self._shutdown_once()
        self._shutdown.set()
        if self.app is not None:
            self.app.quit()   # unblocks app.exec() in start()


if __name__ == "__main__":
    setup_logging()   # must run before any other module logs anything
    app = Controller()
    app.start()