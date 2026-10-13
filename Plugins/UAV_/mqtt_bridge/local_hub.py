# mqtt_bridge/local_hub.py
"""Fan telemetry out to several local MAVLink consumers, and merge their commands.

The GCS machine has more than one thing that talks to the aircraft:

  * Mission Planner  -- the screen
  * main.py          -- the STM32 control panel (arm, RTL, joystick)

A single-peer endpoint feeds only one of them, which makes the panel dead
whenever the aircraft is on the MQTT link. LocalHub presents the same interface
BridgeBase already uses, so the bridge code is unchanged, but writes go to every
peer and reads come from any of them.

WHY THE PANEL NEEDS TELEMETRY, NOT JUST A COMMAND PATH
------------------------------------------------------
It is tempting to think the panel only sends, so it needs no inbound traffic.
That is wrong twice over:

  1. UAVCommandSender._do_connect() blocks in _heartbeat() until a vehicle
     HEARTBEAT arrives. With no inbound traffic it never finishes starting.
  2. Its connection is `udpin:`, i.e. a UDP server. pymavlink's mavudp.write()
     only sends to addresses in self.clients, and that set is populated solely
     inside recv(). With nothing inbound, clients is empty and every command is
     discarded by an empty for-loop -- no error, no log entry.

So feeding it telemetry is precisely what makes its commands work, and the
telemetry must come from the same socket that listens for those commands.

WHY A SEPARATE SOCKET PER PEER
------------------------------
Each peer has its own address and its own liveness. Mission Planner dials in
from an ephemeral port and must be tracked with a last-peer policy; the panel
binds a fixed port and must be dialled out to. Those are different behaviours,
and one socket cannot do both cleanly.

Interleaving datagrams from different peers through the shared framer is safe
because UDP is handled in datagram mode: a datagram always holds whole frames
and any residue is discarded per datagram, so one peer's bytes can never be
spliced onto another's.
"""

import logging
import select as _select
import socket
import threading

from mqtt_bridge.mavlink_endpoint import EndpointClosed, UdpPeerEndpoint

logger = logging.getLogger(__name__)


def parse_peer_specs(text):
    """'bind:127.0.0.1:14570,send:127.0.0.1:14551' -> [(kind, host, port), ...].

    Pure, so the parsing is testable without opening sockets.

    kind is 'bind' (we listen, the peer dials in -- Mission Planner) or 'send'
    (the peer listens, we dial out -- the STM32 panel, which is a udpin server).
    """
    peers = []
    for raw in str(text).split(","):
        spec = raw.strip()
        if not spec:
            continue
        parts = spec.split(":")
        if len(parts) != 3:
            raise ValueError(
                f"bad peer spec {spec!r}: expected kind:host:port, "
                f"e.g. bind:127.0.0.1:14570 or send:127.0.0.1:14551")
        kind, host, port = parts[0].strip().lower(), parts[1].strip(), parts[2].strip()
        if kind not in ("bind", "send"):
            raise ValueError(f"bad peer kind {kind!r} in {spec!r}: use 'bind' or 'send'")
        try:
            port = int(port)
        except ValueError:
            raise ValueError(f"bad port {port!r} in {spec!r}") from None
        # Port 0 is valid for 'bind' -- it asks the OS for any free port, which
        # is useful in tests and for a throwaway listener. It is meaningless for
        # 'send': there is no such destination.
        low = 0 if kind == "bind" else 1
        if not low <= port <= 65535:
            raise ValueError(
                f"port {port} out of range in {spec!r}"
                + ("" if kind == "bind" else " ('send' needs a real destination port)"))
        peers.append((kind, host, port))
    if not peers:
        raise ValueError("no local peers configured")
    return peers


class UdpSendEndpoint:
    """Dial out to a peer that is listening, and receive its replies.

    For the STM32 panel, which is a `udpin:` server. We bind an ephemeral local
    port so the panel learns a concrete loopback address to reply to, and both
    directions then share this one socket -- which is the requirement, since the
    panel only ever replies to an address it has already heard from.
    """

    def __init__(self, host, port, name="peer", bind_host="127.0.0.1"):
        self.host = host
        self.port = port
        self.name = name
        self.bind_host = bind_host
        self._sock = None
        self._lock = threading.RLock()
        self.send_errors = 0
        self.rx_datagrams = 0
        self._warned_unreachable = False

    def open(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        # Bind loopback explicitly rather than letting the OS pick 0.0.0.0 on
        # first send: the peer records our source address and replies there, and
        # a wildcard source can produce a reply path that does not come back.
        sock.bind((self.bind_host, 0))
        with self._lock:
            self._sock = sock
        logger.info("%s endpoint sending to %s:%d from %s",
                    self.name, self.host, self.port, sock.getsockname())
        return True

    def fileno(self):
        sock = self._sock
        return sock.fileno() if sock is not None else -1

    def recv(self, n=65535):
        sock = self._sock
        if sock is None:
            raise EndpointClosed(f"{self.name} endpoint is not open")
        try:
            data, _addr = sock.recvfrom(n)
            self.rx_datagrams += 1
            if self._warned_unreachable:
                # Retract the earlier warning. The bridge usually starts before
                # main.py does, so "not listening" is normal at startup -- but
                # left unretracted it sits in the log looking like a live fault
                # long after the peer has come up.
                self._warned_unreachable = False
                logger.info("%s: %s:%d is responding now (the earlier "
                            "'not listening' warning no longer applies)",
                            self.name, self.host, self.port)
            return data
        except BlockingIOError:
            return b""
        except ConnectionResetError:
            # Windows reports a previous send's ICMP port-unreachable this way:
            # the peer is not listening. Routine when main.py is not running.
            if not self._warned_unreachable:
                self._warned_unreachable = True
                logger.warning("%s: %s:%d is not listening (nothing bound there). "
                               "If this is the STM32 panel, check that main.py is "
                               "running and that state/system_config.py points at "
                               "this port.", self.name, self.host, self.port)
            return b""
        except OSError as exc:
            logger.debug("%s recvfrom: %s", self.name, exc)
            return b""

    def write(self, data):
        with self._lock:
            sock = self._sock
        if sock is None:
            raise EndpointClosed(f"{self.name} endpoint is not open")
        try:
            sock.sendto(data, (self.host, self.port))
            return True
        except OSError as exc:
            self.send_errors += 1
            logger.debug("%s sendto %s:%d: %s", self.name, self.host, self.port, exc)
            return False

    def close(self):
        with self._lock:
            sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    @property
    def is_open(self):
        return self._sock is not None

    @property
    def ready_to_write(self):
        # Always: the destination is configured, not learned. Whether anything
        # is actually listening there is not knowable from a UDP socket.
        return self._sock is not None

    @property
    def peer(self):
        return (self.host, self.port)


class LocalHub:
    """Several local peers behind one endpoint-shaped interface.

    Implements open/select/recv/write/close/is_open/is_datagram/ready_to_write,
    so BridgeBase drives it exactly as it drives a single endpoint.
    """

    def __init__(self, peer_specs, name="hub"):
        self.name = name
        self._peers = []
        self._ready = []
        self._rr = 0                 # round-robin cursor, so one chatty peer
        self._lock = threading.RLock()  # cannot starve the others

        for kind, host, port in peer_specs:
            if kind == "bind":
                self._peers.append(UdpPeerEndpoint(host, port, name=f"{name}:bind{port}"))
            else:
                self._peers.append(UdpSendEndpoint(host, port, name=f"{name}:send{port}"))

    @property
    def peers(self):
        return list(self._peers)

    def open(self):
        """Open every peer.

        A failed bind is fatal -- it means something else holds the port, and
        continuing would leave Mission Planner unable to connect with no clear
        reason. A failed send peer is not fatal: it is an optional consumer, and
        the link to the aircraft should not die because the panel is absent.
        """
        opened = 0
        for peer in self._peers:
            try:
                peer.open()
                opened += 1
            except EndpointClosed:
                if isinstance(peer, UdpPeerEndpoint):
                    self.close()
                    raise
                logger.error("%s: optional peer %s failed to open; continuing without it",
                             self.name, peer.name)
            except OSError as exc:
                if isinstance(peer, UdpPeerEndpoint):
                    self.close()
                    raise EndpointClosed(f"{peer.name}: {exc}") from exc
                logger.error("%s: optional peer %s failed to open (%s); continuing",
                             self.name, peer.name, exc)
        if not opened:
            raise EndpointClosed(f"{self.name}: no local peers could be opened")
        logger.info("%s: %d local peer(s) open", self.name, opened)
        return True

    def select(self, timeout):
        socks = [p for p in self._peers if p.is_open and p.fileno() >= 0]
        if not socks:
            raise EndpointClosed(f"{self.name} has no open peers")
        try:
            readable, _, _ = _select.select(socks, [], [], timeout)
        except (OSError, ValueError):
            return False
        with self._lock:
            self._ready = list(readable)
        return bool(readable)

    def recv(self, n=65535):
        """One datagram from one ready peer, round-robin between them."""
        with self._lock:
            if not self._ready:
                return b""
            self._rr = (self._rr + 1) % len(self._ready)
            peer = self._ready.pop(self._rr % max(1, len(self._ready)))
        return peer.recv(n)

    def write(self, data):
        """Send to every peer. True if at least one accepted it."""
        delivered = False
        for peer in self._peers:
            if not peer.is_open or not peer.ready_to_write:
                continue
            try:
                if peer.write(data) is not False:
                    delivered = True
            except EndpointClosed:
                continue
            except Exception:
                logger.debug("%s: write to %s failed", self.name, peer.name, exc_info=True)
        return delivered

    def close(self):
        for peer in self._peers:
            try:
                peer.close()
            except Exception:
                pass

    @property
    def is_open(self):
        return any(p.is_open for p in self._peers)

    @property
    def is_datagram(self):
        return True

    @property
    def ready_to_write(self):
        """True once ANY peer can receive.

        Not all: before Mission Planner connects, the panel should still get
        telemetry, and vice versa. BridgeBase uses this to tell "nobody is
        listening yet" from "the write failed".
        """
        return any(p.is_open and p.ready_to_write for p in self._peers)

    @property
    def peer(self):
        """Summary for the metrics line."""
        parts = []
        for p in self._peers:
            target = p.peer
            parts.append(f"{p.name}={target if target else 'none'}")
        return " ".join(parts) if parts else None

    def counters(self):
        out = {}
        for p in self._peers:
            if isinstance(p, UdpPeerEndpoint):
                out[f"{p.name}_peer_changes"] = p.peer_changes
            out[f"{p.name}_send_errors"] = p.send_errors
        return out

    def fileno(self):
        # Present so LocalHub can be passed to select() uniformly if ever needed;
        # the hub itself selects over its peers instead.
        return -1
