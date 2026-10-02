# mqtt_bridge/mavlink_endpoint.py
"""MAVLink endpoints used purely as byte pipes. No parsing happens here.

Two implementations:

  MavutilEndpoint  -- aircraft side. Wraps mavutil.mavlink_connection() to get
                      serial / udpin / udpout / tcp connection-string handling for
                      free, then uses only recv/write/select/close. It never calls
                      recv_msg, recv_match or wait_heartbeat, so pymavlink's
                      parser, dialect and CRC logic are never involved.

  UdpPeerEndpoint  -- GCS side. A plain socket, deliberately NOT mavudp.

WHY NOT mavudp ON THE GCS SIDE
------------------------------
mavudp.__init__ takes timeout=0 by default and mavlink_connection() never
forwards a timeout, so self.timeout is always 0. In mavudp.write() the guard
reads:

    if len(self.clients) == 1 or self.timeout <= 0 or <recently alive>:
        self.port.sendto(buf, address)

With timeout <= 0 permanently true, it sends to every peer it has ever heard
from and the stale-peer eviction branch below it is unreachable. Mission Planner
reconnects on a fresh ephemeral port each time, so after a few reconnects every
telemetry frame would be duplicated to several dead sockets. UdpPeerEndpoint
implements an explicit last-peer policy instead.
"""

import logging
import socket
import threading
import time

from pymavlink import mavutil

logger = logging.getLogger(__name__)


class EndpointClosed(Exception):
    """The endpoint is not usable; the caller should reconnect or stop."""


# --------------------------------------------------------------------------
# Aircraft side
# --------------------------------------------------------------------------

class MavutilEndpoint:
    """A pymavlink connection used as a raw byte pipe.

    Thread model: one reader thread calls select()/recv(); any thread may call
    write(), which is serialised by an internal lock. open()/close() are called
    by the reader thread only.
    """

    # Capped exponential backoff. Deliberately NOT the port-walking retry in
    # MavlinkWorker._connect_with_retry: wandering to another port hides the real
    # problem and, for a bridge, silently attaches to the wrong vehicle.
    _BACKOFF = (1, 2, 4, 8, 10)

    def __init__(self, connection_string, name="fc"):
        self.connection_string = connection_string
        self.name = name
        self._conn = None
        self._lock = threading.RLock()
        self._cancel = threading.Event()
        self.reconnects = 0

    def open(self, timeout=None):
        """Connect, retrying with backoff until cancelled. Returns True on success."""
        attempt = 0
        deadline = None if timeout is None else time.monotonic() + timeout
        while not self._cancel.is_set():
            try:
                # wait_ready is deliberately omitted (it defaults to False). With
                # it, mavlink_connection calls wait_heartbeat() and consumes
                # frames through the parser we are avoiding.
                conn = mavutil.mavlink_connection(self.connection_string)
                with self._lock:
                    self._conn = conn
                logger.info("%s endpoint open on %s", self.name, self.connection_string)
                return True
            except Exception as exc:
                delay = self._BACKOFF[min(attempt, len(self._BACKOFF) - 1)]
                attempt += 1
                logger.warning("%s endpoint %s unavailable (%s: %s), retrying in %ds",
                               self.name, self.connection_string,
                               type(exc).__name__, exc, delay)
                if deadline is not None and time.monotonic() + delay > deadline:
                    return False
                if self._cancel.wait(delay):
                    return False
        return False

    def select(self, timeout):
        """True if bytes may be waiting.

        mavfile.select() degrades to time.sleep(min(timeout, 0.5)) when fd is
        None, which is the case for pyserial on Windows -- so this becomes a poll
        rather than a true wait there. Acceptable at a 0.1s period.
        """
        conn = self._conn
        if conn is None:
            raise EndpointClosed(f"{self.name} endpoint is not open")
        try:
            return bool(conn.select(timeout))
        except Exception:
            return True   # let recv() surface the real error

    def recv(self, n=4096):
        """Up to n bytes. Returns b"" when nothing is available.

        mavudp.recv() returns "" -- a str, not bytes -- on EAGAIN, EWOULDBLOCK
        and ECONNREFUSED, so the result is normalised here. ECONNREFUSED is
        routine for udpout against a peer that is not listening yet.
        """
        conn = self._conn
        if conn is None:
            raise EndpointClosed(f"{self.name} endpoint is not open")
        # mavserial.recv() defaults n to self.mav.bytes_needed(), which is tied
        # to a parser we never drive, so n is always explicit.
        data = conn.recv(n)
        if not data:
            return b""
        return data if isinstance(data, (bytes, bytearray)) else data.encode("latin-1")

    def write(self, data):
        with self._lock:
            conn = self._conn
            if conn is None:
                raise EndpointClosed(f"{self.name} endpoint is not open")
            conn.write(data)

    def close(self):
        with self._lock:
            conn, self._conn = self._conn, None
        if conn is not None:
            try:
                conn.close()
            except Exception:
                logger.debug("%s endpoint close raised", self.name, exc_info=True)

    def reopen(self):
        """Close and connect again, counting the reconnect."""
        self.reconnects += 1
        self.close()
        return self.open()

    def cancel(self):
        """Abandon any in-progress retry loop."""
        self._cancel.set()

    @property
    def is_open(self):
        return self._conn is not None

    @property
    def ready_to_write(self):
        """A flight controller link is writable as soon as it is open."""
        return self._conn is not None

    @property
    def is_datagram(self):
        """True for UDP, where a datagram always holds whole frames."""
        return self.connection_string.startswith(("udp", "udpin", "udpout"))

    @property
    def is_passive(self):
        """True for udpin: we bind and wait, so a vanished peer is not our fault.

        Reconnecting a healthy bound socket because the far end went quiet just
        produces flapping. The right response is to report the link down and let
        the peer come back.
        """
        return self.connection_string.startswith("udpin:")


# --------------------------------------------------------------------------
# GCS side
# --------------------------------------------------------------------------

class UdpPeerEndpoint:
    """A bound UDP socket serving one local MAVLink peer, last-peer policy.

    Mission Planner's UDPCl option (UdpSerialConnect) dials out to us from an
    ephemeral port, so the peer address is learned from the first datagram rather
    than configured. Its very first datagram is a lone 0x00 NAT-punch byte, which
    the framer counts as one bad byte -- a useful liveness signal, not an error.
    """

    def __init__(self, host, port, name="mp"):
        self.host = host
        self.port = port
        self.name = name
        self._sock = None
        self._peer = None
        self._lock = threading.RLock()
        self.peer_changes = 0
        self.send_errors = 0

    def open(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # SO_REUSEADDR is deliberately NOT set. On Windows it lets a second
        # process bind the same UDP port successfully while datagrams reach only
        # one socket, which is how the existing 14550 contention fails silently.
        # Without it, a clash raises here and is reported.
        sock.setblocking(False)
        try:
            sock.bind((self.host, self.port))
        except OSError as exc:
            sock.close()
            raise EndpointClosed(
                f"cannot bind UDP {self.host}:{self.port} ({exc}). Another "
                f"process already has it -- Mission Planner's AutoConnect, the "
                f"STM32 bridge, or a previous run of this bridge."
            ) from exc
        with self._lock:
            self._sock = sock
        logger.info("%s endpoint listening on %s:%d (waiting for the first datagram "
                    "to learn the peer)", self.name, self.host, self.port)
        return True

    def select(self, timeout):
        sock = self._sock
        if sock is None:
            raise EndpointClosed(f"{self.name} endpoint is not open")
        import select as _select
        readable, _, _ = _select.select([sock], [], [], timeout)
        return bool(readable)

    def recv(self, n=65535):
        sock = self._sock
        if sock is None:
            raise EndpointClosed(f"{self.name} endpoint is not open")
        try:
            data, addr = sock.recvfrom(n)
        except BlockingIOError:
            return b""
        except ConnectionResetError:
            # Windows raises this on a UDP socket when a previous send met an
            # ICMP port-unreachable -- i.e. the peer went away. Not fatal.
            return b""
        except OSError as exc:
            logger.debug("%s recvfrom: %s", self.name, exc)
            return b""

        if addr != self._peer:
            if self._peer is not None:
                self.peer_changes += 1
                logger.info("%s peer changed %s -> %s (last-peer policy: the old "
                            "one stops receiving)", self.name, self._peer, addr)
            else:
                logger.info("%s peer learned: %s", self.name, addr)
            with self._lock:
                self._peer = addr
        return data

    @property
    def ready_to_write(self):
        """False until a peer is known.

        Distinguishes "nobody has connected yet" from "the send failed", which
        matters: before Mission Planner connects, every inbound telemetry frame
        has nowhere to go. Discarding it is correct, but counting it as a write
        failure would send someone hunting a bug that is not there.
        """
        return self._peer is not None

    def write(self, data):
        """Send to the most recent peer. A no-op before any peer is known."""
        with self._lock:
            sock, peer = self._sock, self._peer
        if sock is None:
            raise EndpointClosed(f"{self.name} endpoint is not open")
        if peer is None:
            return False      # nothing has connected yet; dropping is correct
        try:
            sock.sendto(data, peer)
            return True
        except OSError as exc:
            self.send_errors += 1
            logger.debug("%s sendto %s: %s", self.name, peer, exc)
            return False

    def close(self):
        with self._lock:
            sock, self._sock = self._sock, None
            self._peer = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    @property
    def peer(self):
        return self._peer

    @property
    def is_open(self):
        return self._sock is not None

    @property
    def is_datagram(self):
        return True
