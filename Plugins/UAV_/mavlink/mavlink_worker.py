# mavlink/mavlink_worker.py

import errno
import threading
import time
from pymavlink import mavutil
from utils.connection_manager import ConnectionManager
import logging
logger = logging.getLogger(__name__)


class MavlinkWorker(ConnectionManager):

    def __init__(self, connection_string, state, watchdog=None, interval=1.0):
        super().__init__()
        self.connection_string = connection_string
        self.state = state
        self.watchdog = watchdog
        self.interval = interval
        self._stop_event = threading.Event()
        self._loop_thread = None

    def _connect_with_retry(self, label, retry_delay=2):
        PORT_MIN = 14550 #14551
        PORT_MAX = 14559
        PORT_RANGE = PORT_MAX - PORT_MIN + 1

        # A serial endpoint ("serial:COM7:115200", "/dev/ttyUSB0") has no port to
        # walk. Previously this raised ValueError and the worker never connected
        # at all; now the connection string is simply used as-is and the
        # port-walking below is skipped.
        parts = self.connection_string.rsplit(":", 1)
        prefix = parts[0]
        try:
            start_port = int(parts[1])
        except (IndexError, ValueError):
            logger.info(f"  [{label}] {self.connection_string} has no port to walk; "
                        f"using it as given.")
            return self._connect_fixed(label, retry_delay)

        current_port = start_port

        while not self.is_cancelled():
            conn_str = f"{prefix}:{current_port}"
            logger.info(f"  [{label}] connecting to {conn_str}...")

            try:
                conn = mavutil.mavlink_connection(conn_str, wait_ready=True)

                if not conn_str.startswith("udpin:"):
                    conn.port.bind(('', 0))
                logger.info(f"  [{label}] connected on {conn_str}.")
                return conn

            except ConnectionRefusedError:
                next_port = PORT_MIN + ((current_port - PORT_MIN + 1) % PORT_RANGE)
                logger.info(f"  [{label}] port {current_port} refused (taken) — trying {next_port}...")
                current_port = next_port
                time.sleep(0.3)

            except OSError as e:
                # A UDP bind clash raises EADDRINUSE here, NOT ConnectionRefused,
                # so it never reached the port-walk above and this retried the
                # same taken port forever with a vague message. Name it.
                if getattr(e, "errno", None) in (errno.EADDRINUSE, 10048):
                    logger.error(
                        f"  [{label}] {conn_str} is already in use by another "
                        f"process. Mission Planner's AutoConnect binds 14550 at "
                        f"startup and never releases it; 14551 is reserved for "
                        f"this bridge. Retrying in {retry_delay}s, but it will "
                        f"not free up on its own.")
                else:
                    logger.warning(f"  [{label}] {conn_str} not available "
                                   f"({type(e).__name__}: {e}) — waiting {retry_delay}s...")
                time.sleep(retry_delay)

            except Exception as e:
                logger.warning(f"  [{label}] {conn_str} not available ({type(e).__name__}) — waiting {retry_delay}s...")
                time.sleep(retry_delay)

        raise InterruptedError("Connection cancelled.")

    def _connect_fixed(self, label, retry_delay=2):
        """Retry one connection string unchanged, for endpoints with no port."""
        while not self.is_cancelled():
            try:
                conn = mavutil.mavlink_connection(self.connection_string, wait_ready=True)
                logger.info(f"  [{label}] connected on {self.connection_string}.")
                return conn
            except Exception as e:
                logger.warning(f"  [{label}] {self.connection_string} not available "
                               f"({type(e).__name__}: {e}) — waiting {retry_delay}s...")
                time.sleep(retry_delay)
        raise InterruptedError("Connection cancelled.")

    def _heartbeat(self, connection, timeout=10, retry_delay=5):

        while not self.is_cancelled():
            logger.info(f"{self.__class__.__name__} waiting for heartbeat...")
            msg = connection.wait_heartbeat(timeout=timeout)
            if msg is not None:
                logger.info(
                    f"{self.__class__.__name__} heartbeat received — "
                    f"system {msg.get_srcSystem()} component {msg.get_srcComponent()}"
                )
                return msg

            logger.info(
                f"{self.__class__.__name__}: no heartbeat within {timeout}s on "
                f"{self.connection_string} — will keep trying every {retry_delay}s. "
                f"If this persists on one machine but not others running the "
                f"identical script, check this PC's firewall specifically."
            )
            time.sleep(retry_delay)

        raise InterruptedError("Connection cancelled while waiting for heartbeat.")

    def send_ping(self, connection, target_system=1, target_component=1):
        """
        Sends a MAVLink PING request to a target system (e.g., Herelink/Autopilot).
        """
        # Generate a unique sequence identifier using microsecond timestamps
        ping_id = int(time.time() * 1000000) & 0xFFFFFFFF
        
        logger.info(f"##########Sending PING request (ID: {ping_id}) to System {target_system}...##########")
        
        connection.mav.ping_send(
            ping_id,          # Unique ID (often a timestamp) to match response
            target_system,    # Target System ID (typically 1 for drone/flight controller)
            target_component  # Target Component ID (typically 1 for main autopilot)
        )
        return ping_id

    def _request_message_interval(self, connection, message_id, interval_us=100000):
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0, 0, 0, 0, 0
        )

    def _start_loop(self):
        self._stop_event.clear()
        self._loop_thread = threading.Thread(
            target=self._loop,
            daemon=True,
            name=self.__class__.__name__
        )
        self._loop_thread.start()
        logger.info(f"{self.__class__.__name__} loop started.")

    def getMyThread(self):
        return self._loop_thread

    def stop(self):
        self._stop_event.set()
        if self._loop_thread and self._loop_thread.is_alive():
            self._loop_thread.join(timeout=5)
        self.disconnect()

    def is_running(self):
        return self._loop_thread is not None and self._loop_thread.is_alive()

    def _loop(self):
        self._start_watch()
        while not self._stop_event.is_set():
            try:
                self._pet_watchdog()
                self._run_once()
            except Exception as e:
                logger.error(f"{self.__class__.__name__} error: {e}")
                self._on_error(e)
            self._stop_event.wait(self.interval)

    def _do_connect(self):
        raise NotImplementedError

    def _do_disconnect(self):
        pass

    def _run_once(self):
        raise NotImplementedError

    def _pet_watchdog(self):
        pass

    def _start_watch(self):
        pass

    def _on_error(self, e):
        pass