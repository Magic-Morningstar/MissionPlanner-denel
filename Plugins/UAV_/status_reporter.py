# status_reporter.py
#
# Periodically writes a small JSON snapshot of bridge-side state the C# GCS
# can't otherwise see (currently just STM32 serial connection status) to a
# file the DenelHudOverlay plugin polls. Deliberately file-based rather than
# a socket: the previous GCS-notification channel was a TCP listener on a
# background thread, and a port-bind failure there crashed the whole app on
# startup. A polled file degrades gracefully instead — a missing or stale
# file just reads as "unknown" on the C# side, it can't throw on startup.

import json
import os
import threading
import time
import logging

logger = logging.getLogger(__name__)

STATUS_DIR = os.path.join(os.environ.get("PROGRAMDATA", "."), "Denel GCS")
STATUS_FILE = os.path.join(STATUS_DIR, "bridge_status.json")
WRITE_INTERVAL_SEC = 1.0


class StatusReporter:

    def __init__(self, state):
        self.state = state
        self._stop = threading.Event()

    def start(self):
        os.makedirs(STATUS_DIR, exist_ok=True)
        threading.Thread(target=self._run, daemon=True, name="StatusReporter").start()
        logger.info(f"StatusReporter: writing to {STATUS_FILE} every {WRITE_INTERVAL_SEC}s")

    def stop(self):
        self._stop.set()

    def _run(self):
        while not self._stop.is_set():
            self._write_once()
            time.sleep(WRITE_INTERVAL_SEC)

    def _write_once(self):
        connected = bool(self.state.is_Serial_Connection_Available)
        connection = self.state.get_Serial_Connection
        port = getattr(connection, "port", None)

        payload = {
            "stm32_connected": connected,
            "stm32_port": port,
            "timestamp": time.time(),
        }

        # Write-then-rename so the C# side never reads a half-written file.
        # On Windows, os.replace() can transiently fail with WinError 5
        # (access denied) if the C# side's File.ReadAllText() happens to
        # have the destination open at that exact instant — .NET's default
        # share mode doesn't include FILE_SHARE_DELETE. A short retry
        # absorbs that race without needing to coordinate the two 1Hz
        # loops explicitly.
        tmp_path = STATUS_FILE + ".tmp"
        try:
            with open(tmp_path, "w") as f:
                json.dump(payload, f)
        except OSError as e:
            logger.warning(f"StatusReporter: failed to write temp file: {e}")
            return

        for attempt in range(5):
            try:
                os.replace(tmp_path, STATUS_FILE)
                return
            except OSError as e:
                if attempt == 4:
                    logger.warning(f"StatusReporter: failed to replace status file: {e}")
                else:
                    time.sleep(0.05)
