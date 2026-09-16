# connection_manager.py

import threading


class ConnectionManager:
    def __init__(self):
        self.connection_in_progress = False
        self._cancel_requested = False
        self._lock = threading.Lock()

    def connect(self):
        with self._lock:
            if self.connection_in_progress:
                return
            self._cancel_requested = False
            self.connection_in_progress = True
        t = threading.Thread(target=self._run, daemon=True)
        t.start()
        return t

    def disconnect(self):
        self._cancel_requested = True
        self._do_disconnect()
        with self._lock:
            self.connection_in_progress = False

    def is_cancelled(self):
        return self._cancel_requested

    def _run(self):
        try:
            self._do_connect()
        finally:
            with self._lock:
                self.connection_in_progress = False

    def _do_connect(self):
        raise NotImplementedError

    def _do_disconnect(self):
        raise NotImplementedError
