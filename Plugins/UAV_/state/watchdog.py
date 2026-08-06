# state/watchdog.py

import threading
import time
import logging

logger = logging.getLogger(__name__)


class Watchdog:

    def __init__(self, serialTimeout=5, uavStateTimeout=5, uavCommandTimeout=5, state=None):
        self.state = state

        self.serialTimeout = serialTimeout
        self.uavStateTimeout = uavStateTimeout
        self.uavCommandTimeout = uavCommandTimeout

        self.last_serial_heartbeat = time.time()
        self.serial_lock = threading.Lock()

        self.last_uavState_heartbeat = time.time()
        self.uavState_lock = threading.Lock()

        self.last_uavCommand_heartbeat = time.time()
        self.uavCommand_lock = threading.Lock()

        self.watchSerial = False
        self.watchStateUpdate = False
        self.watchCommands = False

        # Optional hook, set via register_serial_timeout_callback(). Lets
        # SerialHandler own its own reconnect policy (backoff, retries,
        # re-handshake) while Watchdog keeps doing only what it's always
        # done: deciding when a thread looks frozen.
        self._serial_timeout_callback = None

    # ── Activation ────────────────────────────────────────────────────────────

    def watchSerialThread(self):
        self.watchSerial = True

    def watchStateUpdateThread(self):
        self.watchStateUpdate = True

    def watchCommandsThread(self):
        self.watchCommands = True

    def register_serial_timeout_callback(self, callback):
        """callback is invoked (with no args) from on_serial_timeout(),
        i.e. from the Watchdog's own monitor thread — callback should not
        block for long, and if it needs to do real work (like a reconnect
        loop) it should hand off to its own thread."""
        self._serial_timeout_callback = callback

    # ── Serial thread ─────────────────────────────────────────────────────────

    def serial_pet(self):
        with self.serial_lock:
            self.last_serial_heartbeat = time.time()

    def is_serial_alive(self):
        with self.serial_lock:
            return (time.time() - self.last_serial_heartbeat) < self.serialTimeout

    # ── UAV state-poll thread ─────────────────────────────────────────────────

    def uavState_pet(self):
        with self.uavState_lock:
            self.last_uavState_heartbeat = time.time()

    def is_uavState_alive(self):
        with self.uavState_lock:
            return (time.time() - self.last_uavState_heartbeat) < self.uavStateTimeout

    # ── UAV command-sender thread ─────────────────────────────────────────────

    def uavCommand_pet(self):
        with self.uavCommand_lock:
            self.last_uavCommand_heartbeat = time.time()

    def is_uavCommand_alive(self):
        with self.uavCommand_lock:
            return (time.time() - self.last_uavCommand_heartbeat) < self.uavCommandTimeout

    # ── Monitor loop ──────────────────────────────────────────────────────────

    def monitor_loop(self):
        while True:
            if self.watchSerial and not self.is_serial_alive():
                logger.error("WATCHDOG: serial thread appears frozen!")
                self.on_serial_timeout()

            if self.watchStateUpdate and not self.is_uavState_alive():
                logger.error("WATCHDOG: UAV state-poll thread appears frozen!")
                self.on_uavState_timeout()

            if self.watchCommands and not self.is_uavCommand_alive():
                logger.error("WATCHDOG: UAV command-sender thread appears frozen!")
                self.on_uavCommand_timeout()

            time.sleep(1)

    def start(self):
        t = threading.Thread(target=self.monitor_loop, daemon=True, name="Watchdog")
        t.start()
        return t

    # ── Recovery hooks ────────────────────────────────────────────────────────

    def on_serial_timeout(self):
        self.state.update_Serial_connection()
        self.serial_pet()
        self.watchSerial = False
        if self._serial_timeout_callback:
            try:
                self._serial_timeout_callback()
            except Exception:
                logger.exception("Watchdog: serial timeout callback raised")

    def on_uavState_timeout(self):
        self.state.update_UAV_State_Connection()
        self.uavState_pet()
        self.watchStateUpdate = False

    def on_uavCommand_timeout(self):
        self.state.update_UAV_Command_Connection()
        self.uavCommand_pet()
        self.watchCommands = False