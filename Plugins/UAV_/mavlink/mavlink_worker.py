# mavlink/mavlink_worker.py

import threading
import time
from pymavlink import mavutil
from connection_manager import ConnectionManager
import logging
logger = logging.getLogger(__name__)


class MavlinkWorker(ConnectionManager):
    """
    Base class for anything that owns a MAVLink connection and runs a
    periodic background loop against it.

    Subclasses implement:
        _open_connection()  -> bool   open the socket, build helpers, register state
        _do_disconnect()              tear it all down again
        _run_once()                   one tick of work

    Subclasses must NOT override _do_connect() or _loop(); the base class
    handles connect-then-start-loop and loop-then-reconnect so that the
    reconnect path can re-run _open_connection() on the SAME loop thread
    rather than spawning a new one every time.
    """

    # How often we announce our own presence via HEARTBEAT — both while
    # waiting to connect (see _connect_and_wait_for_heartbeat) and for
    # the life of the connection (see _loop). 1Hz matches the MAVLink
    # spec's expectation for a GCS-role participant.
    HEARTBEAT_SEND_INTERVAL_SEC = 1.0

    # How many consecutive _run_once() failures before we assume the link
    # is genuinely dead, tear it down and rebuild it. Previously _loop()
    # marked the connection down via _on_error() but never reconnected,
    # so the worker span at full interval on a socket everyone already
    # believed was dead, forever.
    MAX_CONSECUTIVE_ERRORS = 5

    # Delay between reconnection attempts.
    RECONNECT_BACKOFF_SEC = 3.0

    def __init__(self, connection_string, state, watchdog=None, interval=1.0,
                 source_system=200, source_component=190):
        super().__init__()
        self.connection_string = connection_string
        self.state = state
        self.watchdog = watchdog
        self.interval = interval
        self._stop_event = threading.Event()
        self._loop_thread = None

        # MAVLink identity for THIS worker.
        #
        # Every worker used to leave this at pymavlink's default of
        # system 255 / component 0, and every worker sends MAV_TYPE_GCS
        # heartbeats — so the autopilot and the Herelink router saw one
        # logical GCS reachable at two different addresses. Targeted
        # replies (COMMAND_ACK, PARAM_VALUE, mission protocol) then went
        # to whichever worker had transmitted most recently, which looks
        # exactly like intermittent RF loss but isn't.
        #
        # 255 is also what QGroundControl and Solex use, including the
        # copies running on the Herelink controller itself, so staying
        # off it avoids a three-way collision.
        self.source_system = source_system
        self.source_component = source_component

        # Guards ALL transmits on this worker's connection. pymavlink's
        # MAVLink.send() mutates a shared sequence counter and buffer and
        # is not thread-safe, and this class transmits from its loop
        # thread (heartbeats) while subclasses transmit from theirs
        # (commands). Lives here rather than in FlightCommandSender so
        # that the base class's own heartbeats are covered too.
        self._mav_lock = threading.Lock()

        # Subclasses set this in _open_connection(), in addition to
        # whatever their own named attribute is (command_drone /
        # state_drone) — this is what lets _loop() send periodic
        # heartbeats generically without knowing each subclass's
        # attribute name. It doubles as the "are we connected?" flag:
        # _loop() treats None as "reconnect needed".
        self._mav_connection = None
        self._last_heartbeat_sent = 0.0
        self._ping_seq = 0

    # ── Connection ────────────────────────────────────────────────────────────

    def _connect(self, label, retry_delay=2):
        """
        Opens the MAVLink socket at self.connection_string, retrying on
        failure until cancelled.

        The previous version scanned ports 14550-14559 on
        ConnectionRefusedError, which did not do what it looked like it
        did:

          * udpout: never raises on creation. UDP is connectionless, so
            mavlink_connection() succeeds whether or not anything is
            listening on the far end. The scan branch never fired; the
            worker just sat on a dead port until the heartbeat wait timed
            out.
          * udpin: a port already bound raises OSError/EADDRINUSE, not
            ConnectionRefusedError, so it fell through to the generic
            handler and retried the SAME port forever.

        For Herelink the endpoint is fixed anyway (14552 on the ground
        unit, or whatever local port your MAVProxy/mavlink-router --out
        is bound to), so scanning could only ever waste a heartbeat
        timeout on the wrong port. Fail on the configured endpoint
        instead, loudly.

        Note also that wait_ready=True was being passed here and silently
        discarded: mavlink_connection() only forwards **opts to the
        serial backend, not the UDP/TCP ones. The heartbeat wait in
        _connect_and_wait_for_heartbeat() is what actually establishes
        readiness.
        """
        while not self.is_cancelled() and not self._stop_event.is_set():
            logger.info(f"  [{label}] connecting to {self.connection_string} "
                        f"as sys={self.source_system} comp={self.source_component}...")
            try:
                conn = mavutil.mavlink_connection(
                    self.connection_string,
                    source_system=self.source_system,
                    source_component=self.source_component,
                )
                logger.info(f"  [{label}] socket open on {self.connection_string}.")
                return conn
            except Exception as e:
                logger.warning(
                    f"  [{label}] {self.connection_string} not available "
                    f"({type(e).__name__}: {e}) — retrying in {retry_delay}s..."
                )
                self._stop_event.wait(retry_delay)

        raise InterruptedError("Connection cancelled.")

    def _send_heartbeat(self, connection):
        """
        Sends our own HEARTBEAT, announcing this side as a GCS.

        This is not optional on a udpout link: the Herelink's internal
        router only learns where to send our copy of the telemetry once
        it has seen a packet from us, and it can drop us again if we go
        quiet. Sending one is also what fixes needing to open
        QGroundControl/Mission Planner first after a UAV restart — QGC
        was doing this announce for us by accident.

        Takes _mav_lock because subclasses transmit commands on the same
        connection from a different thread.
        """
        try:
            with self._mav_lock:
                connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
        except Exception as e:
            logger.debug(f"{self.__class__.__name__}: heartbeat send failed: {e}")

    def _wait_for_vehicle_heartbeat(self, conn, timeout):
        """
        Waits for a heartbeat from an actual autopilot and returns it, or
        None on timeout.

        Deliberately NOT conn.wait_heartbeat(): that accepts the first
        HEARTBEAT of any kind. On a link where traffic is broadcast or
        reflected between clients — which is exactly what the Herelink
        router does on its WiFi AP — the first heartbeat you see can
        easily be Solex's, QGC's, or the other worker in this same
        process. You then return "connected" with no vehicle present and
        target_system left at 0, and every subsequent command_long_send
        goes to nobody.

        A real autopilot advertises a concrete autopilot type; GCS-role
        participants send MAV_AUTOPILOT_INVALID. That, plus excluding
        MAV_TYPE_GCS, is the filter.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_cancelled() or self._stop_event.is_set():
                return None

            remaining = max(0.1, deadline - time.time())
            msg = conn.recv_match(type='HEARTBEAT', blocking=True,
                                  timeout=min(1.0, remaining))
            if msg is None:
                continue

            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                logger.debug(
                    f"{self.__class__.__name__}: ignoring GCS heartbeat from "
                    f"system {msg.get_srcSystem()}"
                )
                continue
            if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                logger.debug(
                    f"{self.__class__.__name__}: ignoring non-autopilot heartbeat "
                    f"from system {msg.get_srcSystem()} comp {msg.get_srcComponent()}"
                )
                continue
            if msg.get_srcSystem() == self.source_system:
                continue

            # Pin the routing target explicitly rather than relying on
            # mavutil's own inference, which we've just bypassed by not
            # using wait_heartbeat().
            conn.target_system = msg.get_srcSystem()
            conn.target_component = msg.get_srcComponent()
            return msg

        return None

    def _heartbeat(self, connection, timeout=10):
        """
        Plain blocking wait for a vehicle heartbeat. Prefer
        _connect_and_wait_for_heartbeat() — this one does not announce us
        while it waits, so on a udpout link it can wait forever for
        traffic the router will never send us.

        Kept for anything that specifically wants a bare wait.
        """
        logger.info(f"{self.__class__.__name__} waiting for heartbeat...")
        msg = self._wait_for_vehicle_heartbeat(connection, timeout)
        if msg is None:
            logger.warning(
                f"{self.__class__.__name__}: no vehicle heartbeat within {timeout}s "
                f"— closing and retrying."
            )
            try:
                connection.close()
            except Exception:
                pass
            raise TimeoutError(f"No heartbeat within {timeout}s on {self.connection_string}")

        logger.info(
            f"{self.__class__.__name__} heartbeat received — "
            f"system {msg.get_srcSystem()} component {msg.get_srcComponent()}"
        )
        return msg

    def _connect_and_wait_for_heartbeat(self, label, heartbeat_timeout=10):
        """
        Opens the socket and waits for a real vehicle heartbeat, retrying
        the whole thing from scratch on timeout.

        While waiting, a background thread announces us at
        HEARTBEAT_SEND_INTERVAL_SEC. That is what registers us with the
        Herelink's router so it starts forwarding telemetry to our
        address — without it, both sides can sit waiting to hear from the
        other indefinitely.

        On genuine timeout this closes the socket and loops, rather than
        letting a TimeoutError propagate into ConnectionManager._run(),
        which has no except clause around _do_connect() and would just
        lose the thread with no retry.
        """
        while not self.is_cancelled() and not self._stop_event.is_set():
            conn = self._connect(label=label)
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
                msg = self._wait_for_vehicle_heartbeat(conn, heartbeat_timeout)
            finally:
                stop_announcing.set()
                announcer.join(timeout=2)

            if msg is not None:
                self._last_heartbeat_sent = time.time()
                logger.info(
                    f"[{label}] vehicle heartbeat received — "
                    f"system {msg.get_srcSystem()} component {msg.get_srcComponent()} "
                    f"(target_system={conn.target_system})"
                )
                return conn

            logger.warning(
                f"{self.__class__.__name__}: no vehicle heartbeat within "
                f"{heartbeat_timeout}s on {label} — retrying connection."
            )
            try:
                conn.close()
            except Exception:
                pass

        return None

    # ── Outbound helpers ──────────────────────────────────────────────────────

    def send_ping(self, connection, target_system=1, target_component=1):
        """
        Sends a MAVLink PING request to a target system.

        FIXED: previously passed three arguments to ping_send(), which
        takes four — time_usec, seq, target_system, target_component.
        Any call raised TypeError; nothing called it, so it never
        surfaced.
        """
        self._ping_seq = (self._ping_seq + 1) & 0xFFFFFFFF
        time_usec = int(time.time() * 1e6)

        logger.info(
            f"Sending PING (seq {self._ping_seq}) to system {target_system} "
            f"component {target_component}..."
        )
        with self._mav_lock:
            connection.mav.ping_send(
                time_usec,          # timestamp, microseconds
                self._ping_seq,     # sequence number, echoed back in the reply
                target_system,
                target_component,
            )
        return self._ping_seq

    def _request_message_interval(self, connection, message_id, interval_us=100000):
        """
        Ask the autopilot to emit `message_id` every `interval_us`.

        Note this is link-wide, not per-client: the resulting stream goes
        to every connected GCS, not just to us. Only one worker should
        request a rate for any given message, or the two will overwrite
        each other and both get whatever the loser asked for.
        """
        with self._mav_lock:
            connection.mav.command_long_send(
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                interval_us,
                0, 0, 0, 0, 0
            )

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def _do_connect(self):
        """
        Called by ConnectionManager. Subclasses implement
        _open_connection() instead of overriding this, so that the
        reconnect path in _loop() can reuse the open logic without
        starting a second loop thread each time.
        """
        if not self._open_connection():
            return False
        self._start_loop()
        return True

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

    def _attempt_reconnect(self):
        """
        Rebuilds the connection on the loop thread. Returns False if we
        were stopped or cancelled while trying.
        """
        while not self._stop_event.is_set() and not self.is_cancelled():
            logger.info(f"{self.__class__.__name__}: attempting to reconnect...")
            try:
                if self._open_connection():
                    logger.info(f"{self.__class__.__name__}: reconnected.")
                    return True
            except InterruptedError:
                return False
            except Exception as e:
                logger.error(f"{self.__class__.__name__}: reconnect failed: {e}")
            self._stop_event.wait(self.RECONNECT_BACKOFF_SEC)
        return False

    def _loop(self):
        self._start_watch()
        consecutive_errors = 0

        while not self._stop_event.is_set():
            # _mav_connection is None means either we've never connected
            # or the link was torn down after repeated failures.
            if self._mav_connection is None:
                if not self._attempt_reconnect():
                    break
                consecutive_errors = 0
                continue

            try:
                self._pet_watchdog()

                # Keep announcing ourselves for the life of the
                # connection, not just while first connecting — the FC/
                # Herelink side can time out our presence and stop
                # forwarding if it stops seeing heartbeats from us.
                now = time.time()
                if (now - self._last_heartbeat_sent) >= self.HEARTBEAT_SEND_INTERVAL_SEC:
                    self._send_heartbeat(self._mav_connection)
                    self._last_heartbeat_sent = now

                self._run_once()
                consecutive_errors = 0

            except Exception as e:
                consecutive_errors += 1
                logger.error(
                    f"{self.__class__.__name__} error "
                    f"({consecutive_errors}/{self.MAX_CONSECUTIVE_ERRORS}): {e}"
                )
                self._on_error(e)

                if consecutive_errors >= self.MAX_CONSECUTIVE_ERRORS:
                    logger.error(
                        f"{self.__class__.__name__}: {consecutive_errors} consecutive "
                        f"errors — tearing down link and reconnecting."
                    )
                    try:
                        self._do_disconnect()
                    except Exception as teardown_err:
                        logger.error(
                            f"{self.__class__.__name__}: teardown failed: {teardown_err}"
                        )
                    self._mav_connection = None
                    consecutive_errors = 0
                    continue

            self._stop_event.wait(self.interval)

        logger.info(f"{self.__class__.__name__} loop exited.")

    # ── Subclass hooks ────────────────────────────────────────────────────────

    def _open_connection(self):
        """Open the socket, build helpers, publish state. Return bool."""
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