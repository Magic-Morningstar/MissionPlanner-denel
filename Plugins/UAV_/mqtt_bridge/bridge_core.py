# mqtt_bridge/bridge_core.py
"""The shared machinery of both bridges: threads, queues, shutdown.

Both bridges are the same shape with the publish and subscribe topics swapped:

    reader thread   endpoint -> framer -> MQTT publish
    paho thread     MQTT message -> queue            (must never block)
    writer thread   queue -> endpoint

Shutdown follows the pattern main.py established in the "Working Gracefull
shutdown" commit: an idempotent _shutdown_once() guarded by a flag, with each
step isolated so one subsystem failing to stop cannot block the others. The
stdin watcher is there so a future launcher plugin can ask this process to exit
cleanly -- DenelPythonLauncher starts its child with CreateNoWindow, so there is
no console and no Ctrl+C to catch, and it signals by writing "shutdown" and
closing the pipe. EOF is the case that matters: it also arrives if Mission
Planner dies outright.
"""

import logging
import queue
import signal
import sys
import threading

from mqtt_bridge.framing import MavlinkFramer, frame_header
from mqtt_bridge.mavlink_endpoint import EndpointClosed

logger = logging.getLogger(__name__)


# Resolved once per msgid. Without the cache this runs on every log line, and a
# dialect lookup per message type per interval is pure waste.
_MSGID_NAMES = {}


def _msgid_name(msgid):
    """Message name for a msgid, for LOG LINES ONLY.

    This is the one place the dialect is consulted, and only to make a number
    readable. It must never influence routing or filtering -- that would make the
    tunnel non-transparent, which is the whole point of not decoding frames. An
    unknown msgid falls back to its number rather than being treated as invalid,
    which is what keeps a dialect mismatch from mattering here either.

    Uses .msgname, not .name: pymavlink deprecated .name and emits a multi-line
    DeprecationWarning on every access, which floods the log it is meant to make
    readable.
    """
    name = _MSGID_NAMES.get(msgid)
    if name is not None:
        return name
    name = f"msgid{msgid}"
    try:
        from pymavlink import mavutil
        cls = mavutil.mavlink.mavlink_map.get(msgid)
        if cls is not None:
            name = getattr(cls, "msgname", None) or name
    except Exception:
        pass
    _MSGID_NAMES[msgid] = name
    return name


# How long a reader waits for bytes before looping to re-check the stop flag.
_SELECT_TIMEOUT = 0.1
# How long a writer blocks on an empty queue before re-checking the stop flag.
_QUEUE_TIMEOUT = 0.2


def put_drop_oldest(q, item, on_drop=None):
    """Enqueue, discarding the OLDEST item if full. Returns True if nothing was lost.

    Drop-oldest rather than drop-newest because the realistic cause of a full
    queue is the far endpoint being unreachable while traffic keeps arriving. In
    that situation the newest frame is the one worth having: an old ARM or mode
    change delivered late is precisely the hazard brief section 4 is about.
    """
    try:
        q.put_nowait(item)
        return True
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        if on_drop:
            on_drop()
        try:
            q.put_nowait(item)
        except queue.Full:
            pass
        return False


class BridgeBase:
    """One direction out, one direction in, over one MQTT connection."""

    # Subclasses set these.
    reader_thread_name = "Reader"
    writer_thread_name = "Writer"
    endpoint_label = "endpoint"

    def __init__(self, cfg, topics, endpoint, mqtt_link_factory,
                 publish_topic, publish_qos, publish_expiry,
                 subscribe_topic, subscribe_qos, inbound_queue_size):
        self.cfg = cfg
        self.topics = topics
        self.endpoint = endpoint

        self.publish_topic = publish_topic
        self.publish_qos = publish_qos
        self.publish_expiry = publish_expiry
        self.subscribe_topic = subscribe_topic
        self.subscribe_qos = subscribe_qos

        # One framer per direction. Never shared: see framing.py's docstring.
        self._out_framer = MavlinkFramer(name=f"{self.endpoint_label}-out")
        self._in_framer = MavlinkFramer(name=f"{self.endpoint_label}-in")

        self._inbound = queue.Queue(maxsize=inbound_queue_size)

        self._stop = threading.Event()
        self._shutdown_done = False
        self._threads = []

        self.frames_out = 0
        self.frames_in = 0
        self.bytes_out = 0
        self.bytes_in = 0
        self.inbound_dropped = 0
        self.inbound_malformed = 0
        self.dropped_no_peer = 0
        self.write_failures = 0
        self.publish_failures = 0
        # (sysid, compid) -> count. Dialect-free header arithmetic, diagnostics
        # only. This is the evidence brief open question 4 needs: the STM32
        # bridge presents (255, 0) and Mission Planner (255, 190), and both share
        # this tunnel. Never used for routing -- that would break transparency.
        self.sources = {}
        # msgid -> count for the direction this bridge PUBLISHES. On the GCS side
        # that is the command path, which is low-rate and exactly what an operator
        # needs to see to confirm an arm or a mode change actually left the
        # machine. Names only, never payloads: the brief forbids logging payloads
        # at INFO, and a count answers the question without them.
        self.published_msgids = {}

        self.mqtt = mqtt_link_factory(
            subscriptions=[(subscribe_topic, subscribe_qos)],
            on_frame=self._on_mqtt_frame,
        )

    # ---- lifecycle -------------------------------------------------------

    def start(self):
        self.mqtt.start()
        self._spawn(self.reader_thread_name, self._reader_loop)
        self._spawn(self.writer_thread_name, self._writer_loop)
        if self.cfg.METRICS_INTERVAL > 0:
            self._spawn("BridgeMetrics", self._metrics_loop)
        logger.info("%s bridge started: publishing %s (QoS %d), subscribed %s (QoS %d)",
                    self.endpoint_label, self.publish_topic, self.publish_qos,
                    self.subscribe_topic, self.subscribe_qos)

    def run_forever(self):
        """Install signal handlers, start, and block until asked to stop."""
        signal.signal(signal.SIGINT, self._on_signal)
        try:
            signal.signal(signal.SIGTERM, self._on_signal)
        except (AttributeError, ValueError):
            pass    # SIGTERM is not available everywhere on Windows
        self._start_stdin_watcher()
        self.start()
        try:
            while not self._stop.wait(1.0):
                pass
        finally:
            self._shutdown_once()

    def _spawn(self, name, target):
        t = threading.Thread(target=target, daemon=True, name=name)
        t.start()
        self._threads.append(t)
        return t

    def _on_signal(self, signum, frame):
        logger.info("shutdown signal %s received", signum)
        self._stop.set()

    def _start_stdin_watcher(self):
        """Exit cleanly when a parent process closes our stdin. OPT-IN.

        Enabled only by STDIN_SHUTDOWN (--stdin-shutdown), which the Mission
        Planner launcher plugin will pass. It must NOT be automatic: "stdin is not
        a terminal" is true for a held-open pipe, but equally true for stdin that
        is closed or /dev/null -- and in those cases the watcher reads EOF
        immediately and shuts the bridge down the instant it starts.

        That is not hypothetical. systemd gives a service /dev/null on stdin
        unless told otherwise, so an auto-arming watcher would make the
        aircraft-side service exit on launch, every time, with "launcher closed
        stdin" as the only clue. Detached launches behave the same way. There is
        no reliable way to tell a pipe that will stay open from one that is
        already at EOF without reading it, so this is a flag rather than a guess.
        """
        if not getattr(self.cfg, "STDIN_SHUTDOWN", False):
            logger.info("StdinWatcher: not armed (STDIN_SHUTDOWN is off). "
                        "Use Ctrl+C, SIGTERM, or --stdin-shutdown when a parent "
                        "process holds our stdin open.")
            return
        try:
            if sys.stdin is None or sys.stdin.isatty():
                logger.info("StdinWatcher: not armed (stdin is a terminal) - use Ctrl+C.")
                return
        except Exception:
            logger.info("StdinWatcher: not armed (no usable stdin).")
            return

        logger.info("StdinWatcher: armed, waiting for a launcher shutdown request.")

        def _watch():
            try:
                for line in sys.stdin:
                    if line.strip().lower() == "shutdown":
                        logger.info("shutdown requested by launcher")
                        break
                else:
                    logger.info("launcher closed stdin - shutting down")
            except Exception:
                logger.info("stdin watcher ended - shutting down")
            self._stop.set()

        self._spawn("StdinWatcher", _watch)

    def _shutdown_once(self):
        """Safe to call twice: a signal arriving during shutdown must be harmless."""
        if self._shutdown_done:
            return
        self._shutdown_done = True
        self._stop.set()

        logger.info("shutting down; %s", self.format_counters())

        for name, step in (
            ("mqtt", self.mqtt.stop),
            ("endpoint", self.endpoint.close),
        ):
            try:
                step()
                logger.info("shutdown: %s stopped", name)
            except Exception:
                logger.exception("shutdown: %s failed to stop cleanly", name)

        for t in self._threads:
            if t.is_alive() and t.name != "StdinWatcher":
                t.join(timeout=2.0)

    # ---- endpoint -> MQTT ------------------------------------------------

    def _reader_loop(self):
        """Read bytes, frame them, publish whole frames."""
        if not self.endpoint.is_open:
            try:
                self.endpoint.open()
            except EndpointClosed as exc:
                logger.error("%s: %s", self.endpoint_label, exc)
                self._stop.set()
                return

        datagram_mode = getattr(self.endpoint, "is_datagram", False)

        while not self._stop.is_set():
            try:
                if not self.endpoint.select(_SELECT_TIMEOUT):
                    continue
                data = self.endpoint.recv()
                if not data:
                    continue

                # Datagram transports never split a frame across datagrams, so
                # leftover bytes mean corruption and must not be carried into the
                # next datagram. See framing.feed_datagram().
                frames = (self._out_framer.feed_datagram(data) if datagram_mode
                          else self._out_framer.feed(data))

                for frame in frames:
                    self._note_source(frame)
                    self._note_published(frame)
                    if self.mqtt.publish_frame(self.publish_topic, frame,
                                               self.publish_qos, self.publish_expiry):
                        self.frames_out += 1
                        self.bytes_out += len(frame)
                    else:
                        self.publish_failures += 1
            except EndpointClosed:
                if not self._recover_endpoint():
                    return
            except Exception:
                logger.exception("%s reader error", self.endpoint_label)
                if not self._recover_endpoint():
                    return

    def _recover_endpoint(self):
        """Reconnect the endpoint if that makes sense for its transport.

        A udpin socket stays healthy when its peer disappears, so reopening it
        would just flap. Reporting the link as down and waiting is correct: the
        far end sees the heartbeat stop, which is the proper MAVLink-level signal.
        """
        if self._stop.is_set():
            return False
        self._out_framer.reset()

        if getattr(self.endpoint, "is_passive", False):
            logger.warning("%s: passive endpoint error, not reconnecting a bound "
                           "socket; waiting for the peer to return",
                           self.endpoint_label)
            return not self._stop.wait(1.0)

        logger.warning("%s: reopening endpoint", self.endpoint_label)
        try:
            return bool(self.endpoint.reopen())
        except Exception:
            logger.exception("%s: reopen failed", self.endpoint_label)
            return not self._stop.wait(2.0)

    def _note_published(self, frame):
        header = frame_header(frame)
        if header is None:
            return
        msgid = header[2]
        self.published_msgids[msgid] = self.published_msgids.get(msgid, 0) + 1

    def _note_source(self, frame):
        header = frame_header(frame)
        if header is None:
            return
        key = (header[0], header[1])
        self.sources[key] = self.sources.get(key, 0) + 1

    # ---- MQTT -> endpoint ------------------------------------------------

    def _on_mqtt_frame(self, topic, payload, properties):
        """paho's network thread. One enqueue, then return. Never block here."""
        if self.cfg.VERIFY_INBOUND_FRAMES and not self._verify(topic, payload):
            return
        if not put_drop_oldest(self._inbound, payload, self._note_inbound_drop):
            pass   # counted in _note_inbound_drop

    def _verify(self, topic, payload):
        """Assert the payload is exactly one whole frame.

        Costs microseconds and converts any transport or broker corruption into a
        counter and a log line instead of a confusing downstream symptom.
        """
        frames = self._in_framer.feed(payload)
        ok = len(frames) == 1 and frames[0] == payload and self._in_framer.pending == 0
        if not ok:
            self.inbound_malformed += 1
            logger.error("payload on %s is not exactly one whole frame "
                         "(%d bytes, %d frame(s) parsed, %d pending)",
                         topic, len(payload), len(frames), self._in_framer.pending)
            self._in_framer.reset()
        return ok

    def _note_inbound_drop(self):
        self.inbound_dropped += 1
        logger.log(self.inbound_drop_level,
                   "inbound queue full, dropped the oldest frame (%d so far)",
                   self.inbound_dropped)

    # Subclasses override: on the GCS side a drop is a real defect, because a
    # loopback sendto essentially never blocks.
    inbound_drop_level = logging.WARNING

    def _writer_loop(self):
        while not self._stop.is_set():
            try:
                payload = self._inbound.get(timeout=_QUEUE_TIMEOUT)
            except queue.Empty:
                continue
            # "Nobody has connected yet" is not a failure. Before Mission
            # Planner connects, every telemetry frame legitimately has nowhere to
            # go; counting those as write failures would make a healthy bridge
            # look broken.
            if not getattr(self.endpoint, "ready_to_write", True):
                self.dropped_no_peer += 1
                continue
            try:
                if self.endpoint.write(payload) is False:
                    self.write_failures += 1
                else:
                    self.frames_in += 1
                    self.bytes_in += len(payload)
            except EndpointClosed:
                self.write_failures += 1
            except Exception:
                self.write_failures += 1
                logger.exception("%s write failed", self.endpoint_label)

    def _metrics_loop(self):
        """Log the counters periodically, but only when something changed.

        A heartbeat line every N seconds regardless would bury the interesting
        ones; staying silent while idle means any line you see is a line worth
        reading. The connection state is included because "no traffic" and "not
        connected" look identical from the counters alone.
        """
        previous = None
        while not self._stop.wait(self.cfg.METRICS_INTERVAL):
            current = self.counters()
            if current == previous:
                continue
            previous = current
            state = f"mqtt={'up' if self.mqtt.connected else 'DOWN'}"
            # Only the GCS endpoint learns a peer; saying "peer=none" on the
            # aircraft side would read as a fault rather than "not applicable".
            if hasattr(self.endpoint, "peer"):
                state += f" peer={self.endpoint.peer or 'none'}"
            logger.info("%s | %s", self.format_counters(), state)

    # ---- reporting -------------------------------------------------------

    def counters(self):
        out = {
            "frames_out": self.frames_out,
            "frames_in": self.frames_in,
            "bytes_out": self.bytes_out,
            "bytes_in": self.bytes_in,
            "inbound_dropped": self.inbound_dropped,
            "inbound_malformed": self.inbound_malformed,
            "dropped_no_peer": self.dropped_no_peer,
            "write_failures": self.write_failures,
            "publish_failures": self.publish_failures,
        }
        out.update(self.mqtt.counters())
        out.update({f"out_{k}": v for k, v in self._out_framer.counters().items()})
        return out

    def format_counters(self):
        parts = " ".join(f"{k}={v}" for k, v in self.counters().items())
        sources = " ".join(f"{s}/{c}={n}" for (s, c), n in sorted(self.sources.items()))
        if sources:
            parts = f"{parts} sources[{sources}]"
        if self.published_msgids:
            # Busiest first, capped: the aircraft side publishes ~30 telemetry
            # types and the full list would bury the line.
            top = sorted(self.published_msgids.items(), key=lambda kv: -kv[1])[:8]
            sent = " ".join(f"{_msgid_name(m)}={n}" for m, n in top)
            parts = f"{parts} sent[{sent}]"
        return parts
