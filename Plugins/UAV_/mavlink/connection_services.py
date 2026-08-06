import threading
import time
from pymavlink import mavutil
from connection_manager import ConnectionManager
from utils.helper import *
import logging
logger = logging.getLogger(__name__)


def _scan_and_connect(self, label, max_attempts=5):
    """
    Tries ports sequentially starting from self.connection_string.
    Wraps back to PORT_MIN after PORT_MAX.
    Returns a live mavutil connection, or raises ConnectionRefusedError.
    """
    PORT_MIN = 5760
    PORT_MAX = 5769
    PORT_RANGE = PORT_MAX - PORT_MIN + 1

    parts = self.connection_string.rsplit(":", 1)
    prefix = parts[0]
    try:
        start_port = int(parts[1])
    except (IndexError, ValueError):
        raise ValueError(f"Cannot parse port from: {self.connection_string}")

    last_error = None
    for i in range(max_attempts):
        if self.is_cancelled():
            raise InterruptedError("Connection cancelled.")

        port = PORT_MIN + ((start_port - PORT_MIN + i) % PORT_RANGE)
        conn_str = f"{prefix}:{port}"
        logger.info(f"  [{label}] trying {conn_str}...")

        try:
            conn = mavutil.mavlink_connection(conn_str, wait_ready=True)
            logger.info(f"  [{label}] connected on {conn_str}.")
            return conn
        except ConnectionRefusedError as e:
            logger.info(f"  [{label}] port {port} refused — trying next...")
            last_error = e
            time.sleep(0.3)
        except Exception as e:
            logger.info(f"  [{label}] port {port} error: {e} — trying next...")
            last_error = e
            time.sleep(0.3)

    raise ConnectionRefusedError(
        f"{self.__class__.__name__}: no port available after "
        f"{max_attempts} attempts. Last error: {last_error}"
    )