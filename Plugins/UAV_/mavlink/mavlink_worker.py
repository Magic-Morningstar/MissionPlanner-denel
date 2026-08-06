# mavlink/mavlink_worker.py

import threading
import time
from pymavlink import mavutil
from connection_manager import ConnectionManager
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
        PORT_MIN = 14550
        PORT_MAX = 14559
        PORT_RANGE = PORT_MAX - PORT_MIN + 1

        parts = self.connection_string.rsplit(":", 1)
        prefix = parts[0]
        try:
            start_port = int(parts[1])
        except (IndexError, ValueError):
            raise ValueError(f"Cannot parse port from: {self.connection_string}")

        current_port = start_port

        while not self.is_cancelled():
            conn_str = f"{prefix}:{current_port}"
            logger.info(f"  [{label}] connecting to {conn_str}...")

            try:
                conn = mavutil.mavlink_connection(conn_str, wait_ready=True)
                logger.info(f"  [{label}] connected on {conn_str}.")
                return conn

            except ConnectionRefusedError:
                next_port = PORT_MIN + ((current_port - PORT_MIN + 1) % PORT_RANGE)
                logger.info(f"  [{label}] port {current_port} refused (taken) — trying {next_port}...")
                current_port = next_port
                time.sleep(0.3)

            except Exception as e:
                logger.warning(f"  [{label}] {conn_str} not available ({type(e).__name__}) — waiting {retry_delay}s...")
                time.sleep(retry_delay)

        raise InterruptedError("Connection cancelled.")

    def _heartbeat(self, connection, timeout=10):
        """
        timeout added — this used to block forever with no bound at all.
        A TCP connect() can succeed against a port that never actually
        sends MAVLink data (stale socket, wrong port from a hop, etc),
        and without a timeout that's a silent, permanent hang: the
        connection thread never reaches _start_loop(), so nothing ever
        runs, but nothing ever errors either — indistinguishable from
        "working" until you notice the loop just never logs anything.
        """
        logger.info(f"{self.__class__.__name__} waiting for heartbeat...")
        msg = connection.wait_heartbeat()
        if msg is None:
            logger.warning(f"{self.__class__.__name__}: no heartbeat within {timeout}s — closing and retrying.")
            connection.close()   # free the port rather than leave a dead socket holding it
            raise TimeoutError(f"No heartbeat within {timeout}s on {self.connection_string}")
        logger.info(
            f"{self.__class__.__name__} heartbeat received — "
            f"system {msg.get_srcSystem()} component {msg.get_srcComponent()}"
        )
        return msg

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