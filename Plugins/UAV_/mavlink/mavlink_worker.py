# mavlink/mavlink_worker.py

import threading
import time
from pymavlink import mavutil
from connection_manager import ConnectionManager
import logging
logger = logging.getLogger(__name__)


class MavlinkWorker(ConnectionManager):

    # How often we announce our own presence via HEARTBEAT — both while
    # waiting to connect (see _connect_and_wait_for_heartbeat) and for
    # the life of the connection (see _loop). 1Hz matches the MAVLink
    # spec's expectation for a GCS-role participant.
    HEARTBEAT_SEND_INTERVAL_SEC = 1.0

    def __init__(self, connection_string, state, watchdog=None, interval=1.0):
        super().__init__()
        self.connection_string = connection_string
        self.state = state
        self.watchdog = watchdog
        self.interval = interval
        self._stop_event = threading.Event()
        self._loop_thread = None

        # Subclasses should set this to their live connection object in
        # _do_connect() (in addition to whatever their own named
        # attribute is, e.g. command_drone / state_drone) — this is what
        # lets _loop() send periodic heartbeats generically, without
        # needing to know each subclass's attribute name.
        self._mav_connection = None
        self._last_heartbeat_sent = 0.0

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

    def _send_heartbeat(self, connection):
        """
        Sends our own HEARTBEAT, announcing this side as a GCS. Nothing
        in this class used to call this at all — wait_heartbeat() only
        ever receives. See _connect_and_wait_for_heartbeat's docstring
        for why sending one is what actually fixes needing to open
        QGroundControl/Mission Planner first after a UAV restart.
        """
        try:
            connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
        except Exception as e:
            logger.debug(f"{self.__class__.__name__}: heartbeat send failed: {e}")

    def _heartbeat(self, connection, timeout=10):
        """
        FIXED: previously called connection.wait_heartbeat() with no
        arguments at all, silently ignoring the `timeout` parameter —
        meaning this blocked forever instead of ever timing out, and the
        `if msg is None` branch below could never actually run. Now
        timeout is genuinely enforced.

        Prefer _connect_and_wait_for_heartbeat() over calling this
        directly — this method only WAITS, it doesn't send our own
        heartbeat while waiting, so it's still exposed to the same
        mutual-deadlock risk described there. Kept as a standalone
        method for anything that specifically wants a plain wait.
        """
        logger.info(f"{self.__class__.__name__} waiting for heartbeat...")
        msg = connection.wait_heartbeat(timeout=timeout)
        if msg is None:
            logger.warning(f"{self.__class__.__name__}: no heartbeat within {timeout}s — closing and retrying.")
            connection.close()   # free the port rather than leave a dead socket holding it
            raise TimeoutError(f"No heartbeat within {timeout}s on {self.connection_string}")
        logger.info(
            f"{self.__class__.__name__} heartbeat received — "
            f"system {msg.get_srcSystem()} component {msg.get_srcComponent()}"
        )
        return msg

    def _connect_and_wait_for_heartbeat(self, label, heartbeat_timeout=10):
        """
        Combines _connect_with_retry() + a heartbeat wait into one
        self-healing loop — this is the fix for "needs QGroundControl/
        Mission Planner opened first after a UAV restart":

        1. While waiting for the FC/Herelink's heartbeat, this now
           actively SENDS our own heartbeat every HEARTBEAT_SEND_INTERVAL_SEC
           on a background thread. Previously this class only ever
           listened and never announced itself — if whatever's routing
           telemetry on the other end (Herelink's internal router, in
           particular) gates forwarding on having seen a GCS heartbeat
           first, both sides can end up waiting to hear from the other,
           forever. QGC/Mission Planner "fixed" it by accident, since
           both send a heartbeat the instant they connect. This makes
           the script do that itself, every time, without needing
           another app's help.

        2. If the heartbeat wait genuinely times out anyway, this closes
           the socket and retries the whole connection from scratch,
           instead of letting a TimeoutError propagate up uncaught.
           Previously that's exactly what happened: ConnectionManager's
           _run() has no except clause around _do_connect(), so a timed-
           out _heartbeat() call (once its timeout bug was fixed) would
           have silently killed the connect thread with no retry at all
           — worse than the original infinite-hang behavior, not better.
        """
        while not self.is_cancelled():
            conn = self._connect_with_retry(label=label)
            if conn is None:
                return None

            stop_announcing = threading.Event()

            def _announce():
                while not stop_announcing.is_set():
                    self._send_heartbeat(conn)
                    stop_announcing.wait(self.HEARTBEAT_SEND_INTERVAL_SEC)

            announcer = threading.Thread(
                target=_announce, daemon=True, name=f"{label}-HeartbeatAnnounce"
            )
            announcer.start()

            try:
                msg = conn.wait_heartbeat(timeout=heartbeat_timeout)
            finally:
                stop_announcing.set()

            if msg is not None:
                logger.info(
                    f"{self.__class__.__name__} heartbeat received — "
                    f"system {msg.get_srcSystem()} component {msg.get_srcComponent()}"
                )
                return conn

            logger.warning(
                f"{self.__class__.__name__}: no heartbeat within {heartbeat_timeout}s "
                f"on {label} — retrying connection."
            )
            try:
                conn.close()
            except Exception:
                pass

        return None

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

                # Keep announcing ourselves for the life of the
                # connection, not just while first connecting — the FC/
                # Herelink side can still time out our presence and stop
                # forwarding if it stops seeing heartbeats from us, per
                # the MAVLink spec's ~1Hz expectation.
                now = time.time()
                if self._mav_connection is not None and (now - self._last_heartbeat_sent) >= self.HEARTBEAT_SEND_INTERVAL_SEC:
                    self._send_heartbeat(self._mav_connection)
                    self._last_heartbeat_sent = now

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