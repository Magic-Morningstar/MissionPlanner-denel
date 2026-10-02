# serial_controller/serial_handler.py

import time
import serial
import serial.tools.list_ports
import queue
from utils.helper import system_Print
import threading
import logging
logger = logging.getLogger(__name__)
from utils.connection_manager import ConnectionManager
from serial_controller.protocol.registry import get_decoder, MessageType
from serial_controller.protocol.stream_parser import StreamParser
from serial_controller.protocol.frame_builder import build_frame
from serial_controller.protocol.messages import Hello, Goodbye, Heartbeat
from serial_controller.status_builder import StatusBuilder
from serial_controller import link_health
import state.system_config as system_config

# How often to send HEARTBEAT once the session is open.
#
# This MUST stay comfortably inside main.c's HEARTBEAT_TIMEOUT_MS (500 ms
# at the time of writing). At 100 ms the firmware tolerates four missed
# beats before declaring the link lost; tighten either number and a
# healthy but briefly busy link starts flickering red.
#
# Move this to system_config.py alongside the other timings if you'd
# rather keep all the tunables in one place.
HEARTBEAT_PERIOD_S = 0.1

# How long to wait for a candidate port to prove it's the panel.
#
# Bluetooth needs the longer window: SPP link setup happens on open and
# can take a second or more, and the first bytes after that are often
# slow. USB CDC answers almost immediately.
PROBE_TIMEOUT_USB_S = 1.0
PROBE_TIMEOUT_BT_S  = 3.0

# How long the supervisor waits for a connect attempt to actually finish
# before treating it as failed.
#
# ConnectionManager.connect() can return before _do_connect() has run to
# completion on its own thread. Without this wait the supervisor saw
# is_connected() still False, counted a failure, and started a SECOND
# attempt while the first was still probing — two threads then fought
# over the same COM port, and on Windows the second serial.Serial() on an
# already-open port simply fails.
#
# Must exceed the slowest probe: Bluetooth candidates take
# PROBE_TIMEOUT_BT_S each, and discovery may try several ports in turn.
CONNECT_SETTLE_S = 8.0

# Reconnect backoff, in seconds. Starts quick because most drops are
# transient (a replug, a brief stall), then backs off so a genuinely
# absent panel doesn't burn CPU — and, more to the point, doesn't spend
# 3 s probing every phantom Bluetooth port once per second.
RECONNECT_BACKOFF_S = (1.0, 2.0, 5.0, 10.0)


def _is_usb_cdc(port):
    """STM32 native USB CDC — the Nucleo enumerating directly."""
    return port.vid == 0x0483 and port.pid == 0x5740


def _is_dfrobot(port):
    """CH340 USB-serial bridge, as used by the DFRobot adapter."""
    if port.vid == 0x1A86 and port.pid == 0x7523:
        return True
    return "USB-Enhanced-SERIAL-D" in (port.description or "")


def _is_bluetooth(port):
    """Bluetooth SPP, across the three host platforms.

    There is no VID:PID to match on — the host synthesises these ports, so
    identification is by name only:

      Windows  hwid starts with BTHENUM, description is usually
               "Standard Serial over Bluetooth link"
      Linux    /dev/rfcomm0 and friends, created by rfcomm bind
      macOS    /dev/cu.<device-name>-SPP or -SerialPort

    Windows creates TWO ports per paired SPP device, one incoming and one
    outgoing, and only the outgoing one connects. Both match here, which
    is exactly why every candidate gets probed rather than trusted."""
    device = (port.device or "")
    desc = (port.description or "")
    hwid = (port.hwid or "")

    if hwid.upper().startswith("BTHENUM"):
        return True
    if "bluetooth" in desc.lower():
        return True
    if device.startswith("/dev/rfcomm"):
        return True
    if device.startswith("/dev/cu.") and ("SPP" in device or "SerialPort" in device):
        return True
    return False


# Priority order. USB first because it's faster and can't be a phantom;
# Bluetooth last because probing it is the expensive case.
PORT_MATCHERS = (
    ("usb_cdc",   _is_usb_cdc,   PROBE_TIMEOUT_USB_S),
    ("dfrobot",   _is_dfrobot,   PROBE_TIMEOUT_USB_S),
    ("bluetooth", _is_bluetooth, PROBE_TIMEOUT_BT_S),
)


class SerialHandler(ConnectionManager):
    """
    Owns the serial connection to the STM32.

    Same thread shape as before — a dedicated reader thread and a
    dedicated processor thread connected by a queue, so serial IO is
    never blocked by decode work. What changed: the reader now feeds a
    TLV StreamParser instead of splitting on newlines, and the processor
    dispatches by type through the registry instead of string-prefix
    if/elif branches. Decoded objects are handed to an InputTranslator
    instead of being written to SystemState directly.
    """

    def __init__(self, state, translator, watchdog=None, serial_override=None):
        super().__init__()
        self.ser = None
        self.state = state
        self.translator = translator
        self.watchdog = watchdog
        self._frame_queue = queue.Queue()
        self._reader_stop = threading.Event()
        self._processor_stop = threading.Event()
        self._heartbeat_stop = threading.Event()

        # Supervision. _want_connected separates "the link dropped, get it
        # back" from "we are shutting down, stay down" — without it the
        # supervisor would race the shutdown path and immediately
        # reconnect a port we just said GOODBYE to.
        self._supervisor_stop = threading.Event()
        self._want_connected = False
        # Testing hook only: if set, _do_connect() uses this object
        # directly instead of scanning for real hardware and opening a
        # real serial.Serial. Must support .read(n), .write(data),
        # .is_open, .close() — see testing/fake_serial.py.
        self._serial_override = serial_override

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self):
        """Connect, and keep reconnecting for as long as the app runs.

        Use this instead of connect() from application code. connect() is
        still the single-shot attempt; this is the thing that survives the
        panel being unplugged and plugged back in."""
        self._want_connected = True
        self._supervisor_stop.clear()
        threading.Thread(
            target=self._supervisor_loop,
            daemon=True,
            name="SerialSupervisor"
        ).start()

    def shutdown(self):
        """Stop supervising, then disconnect cleanly (which sends GOODBYE).

        Call this at exit rather than disconnect(), or the supervisor will
        helpfully undo the disconnect a second later."""
        self._want_connected = False
        self._supervisor_stop.set()
        self.disconnect()

    def is_connected(self) -> bool:
        ser = self.ser
        return ser is not None and ser.is_open

    def _supervisor_loop(self):
        """Reconnects whenever the link is down.

        Deliberately dumb: it doesn't try to diagnose why the link
        dropped, it just re-runs discovery. That covers a replug landing
        on a different COM port, the panel being power-cycled, and a
        Bluetooth module coming back into range — all of which change
        which port is correct."""
        logger.info("SerialSupervisor: started.")
        attempt = 0

        while not self._supervisor_stop.is_set():
            if self.is_connected():
                attempt = 0
                self._supervisor_stop.wait(1.0)
                continue

            if attempt == 0:
                logger.info("SerialSupervisor: connecting...")
            else:
                logger.info(f"SerialSupervisor: reconnect attempt {attempt + 1}...")

            try:
                ok = self.connect()
            except Exception:
                logger.exception("SerialSupervisor: connect raised")
                ok = False

            # connect() may be asynchronous — wait for the attempt to
            # settle rather than immediately declaring failure and
            # launching another one onto the same port.
            if ok:
                deadline = time.monotonic() + CONNECT_SETTLE_S
                while time.monotonic() < deadline and not self.is_connected():
                    if self._supervisor_stop.wait(0.1):
                        return

            if ok and self.is_connected():
                logger.info("SerialSupervisor: connected.")
                attempt = 0
                continue

            delay = RECONNECT_BACKOFF_S[min(attempt, len(RECONNECT_BACKOFF_S) - 1)]
            attempt += 1
            logger.info(f"SerialSupervisor: no panel — retrying in {delay}s.")
            self._supervisor_stop.wait(delay)

        logger.info("SerialSupervisor: stopped.")

    # ── ConnectionManager interface ───────────────────────────────────────────

    def _do_connect(self):
        if self.is_cancelled():
            return False

        if self._serial_override is not None:
            new_ser = self._serial_override
            logger.info("SerialHandler: using injected test link (no real hardware).")
        else:
            # find_stm_port() returns an ALREADY-OPEN port, not a name.
            # Reopening would mean a second Bluetooth link setup and would
            # drop the session the probe just established.
            new_ser = self.find_stm_port()

            if self.is_cancelled():
                if new_ser:
                    new_ser.close()
                logger.warning("Serial connection cancelled.")
                return False

            if new_ser is None:
                self.state.update_Serial_connection(False, None)
                return False

        if self.is_cancelled():
            new_ser.close()
            logger.warning("Serial connection cancelled.")
            return False

        self.ser = new_ser
        self.state.update_Serial_connection(True, self.ser)
        logger.info(
            f"STM32 connected on {new_ser.port}." if self._serial_override is None
            else "STM32 connected (test link)."
        )

        # Opens the session. The STM32 clears its stale
        # menu_register/USB_MESSAGE/PAYLOAD_MESSAGE state and moves from
        # WAITING to SYNCED — but it does NOT start sending button frames
        # yet. That needs the first heartbeat, which the thread below
        # provides. Until then the panel shows green.
        #
        # Sent again even though the probe already sent one: a second
        # HELLO simply restarts the handshake, which is harmless, and it
        # keeps this path identical whether the port came from discovery
        # or from an injected test link.
        self.send(build_frame(MessageType.HELLO, Hello()))
        logger.info("PC -> STM32: HELLO (session open)")

        if self.watchdog:
            self.watchdog.watchSerialThread()

        self._reader_stop.clear()
        self._processor_stop.clear()
        self._heartbeat_stop.clear()

        threading.Thread(
            target=self._heartbeat_loop,
            daemon=True,
            name="SerialHeartbeat"
        ).start()

        threading.Thread(
            target=self._reader_loop,
            daemon=True,
            name="SerialReader"
        ).start()

        threading.Thread(
            target=self._processor_loop,
            daemon=True,
            name="SerialProcessor"
        ).start()

        return True

    def _do_disconnect(self):
        # Stop the heartbeat before saying goodbye, so a beat can't slip
        # out after the farewell and leave the STM32 thinking the session
        # is still live.
        self._heartbeat_stop.set()
        self._send_goodbye()

        self._reader_stop.set()
        self._processor_stop.set()

        if self.ser and self.ser.is_open:
            self.ser.close()

        self.ser = None
        self.state.update_Serial_connection(False, None)

        # Stop the watchdog watching a thread that no longer exists —
        # otherwise it logs "serial thread appears frozen" five seconds
        # after every ordinary disconnect. _do_connect re-arms it.
        if self.watchdog:
            self.watchdog.stopWatchingSerial()

        logger.info("STM32 disconnected.")

    def _build_heartbeat(self) -> Heartbeat:
        """Snapshots the state of the chain beyond this PC.

        The two percentages cover different segments and fail
        independently: telemetry_health is the radio hop (Herelink ground
        unit to air unit), uav_health is the autopilot's own view of its
        serial hop to the air unit. Reporting one combined figure would
        hide which half is broken.

        The derivation lives in link_health.py because it's the only
        heuristic in this path — everything else here is a faithful
        translation between representations, whereas "what counts as
        healthy" is a judgement to tune against real flights."""
        return Heartbeat(
            uav_connected    = bool(self.state.is_UAV_State_Connection_Available),
            telemetry_health = link_health.telemetry_health(self.state),
            uav_health       = link_health.uav_health(self.state),
        )

    def _send_goodbye(self):
        """Best-effort, and deliberately NOT via self.send(): that calls
        self.disconnect() on failure and we are already inside the
        disconnect path, so it would recurse.

        Failing is normal and not worth a loud log line — the usual reason
        for reaching _do_disconnect is that the link already died, in
        which case there is nothing left to say goodbye to."""
        if not (self.ser and self.ser.is_open):
            return
        try:
            self.ser.write(build_frame(MessageType.GOODBYE, Goodbye()))
            self.ser.flush()
            logger.info("PC -> STM32: GOODBYE (session closed)")
        except Exception:
            logger.debug("GOODBYE not sent — link already gone.")

    # ── Background threads ────────────────────────────────────────────────────

    def _heartbeat_loop(self):
        """Sends HEARTBEAT every HEARTBEAT_PERIOD_S for as long as the
        session is up.

        The first one completes the STM32's handshake and starts button
        frames flowing; every one after that is the only evidence the
        firmware has that this process is still alive. Stop sending and
        the panel goes red within HEARTBEAT_TIMEOUT_MS.

        Uses Event.wait() rather than sleep() so disconnect takes effect
        immediately instead of after the remainder of the current period."""
        logger.info("SerialHeartbeat: started.")

        while not self._heartbeat_stop.is_set():
            # Rebuilt every beat, not hoisted out of the loop: the whole
            # point of the flags is that they change while the session is
            # up. A cached frame would report the health at connect time
            # forever.
            self.send(build_frame(MessageType.HEARTBEAT, self._build_heartbeat()))
            self._heartbeat_stop.wait(HEARTBEAT_PERIOD_S)

        logger.info("SerialHeartbeat: stopped.")

    def _reader_loop(self):
        """
        Reads raw bytes and feeds them into a TLV StreamParser. Only
        pets the watchdog when real bytes actually arrive — a stalled
        STM32 that keeps the port open but stops transmitting will now
        correctly trip the watchdog (this was a bug in the old version,
        which pet on every loop iteration regardless of data).
        """
        logger.info("SerialReader: started.")
        parser = StreamParser()

        while not self._reader_stop.is_set():
            try:
                ser = self.ser
                if ser is None or not ser.is_open:
                    break

                chunk = ser.read(64)

                if chunk:
                    if self.watchdog:
                        self.watchdog.serial_pet()

                    for msg_type, payload in parser.feed(chunk):
                        self._frame_queue.put((msg_type, payload))

            except serial.SerialException:
                logger.info("SerialReader: connection lost!")
                self.disconnect()
                break

        logger.info("SerialReader: stopped.")

    def _processor_loop(self):
        """
        Drains the frame queue, looks up the decoder for each type,
        decodes into a structured object, and hands it to the
        translator. Unknown types are skipped, not fatal — this is
        what lets you add new message types on the STM32 side without
        requiring the PC side to be updated in lockstep.
        """
        logger.info("SerialProcessor: started.")
        while not self._processor_stop.is_set():
            try:
                msg_type, payload = self._frame_queue.get(timeout=0.1)

                decoder = get_decoder(msg_type)
                if decoder is None:
                    logger.info(f"SerialProcessor: unknown type {msg_type:#x} — skipping")
                    self._frame_queue.task_done()
                    continue

                obj = decoder.decode(payload)
                self.translator.handle(obj)

                self._frame_queue.task_done()
            except queue.Empty:
                continue
            #except Exception as e:
                #logger.error(f"SerialProcessor error: {e}")

        logger.info("SerialProcessor: stopped.")

    # ── Port detection ────────────────────────────────────────────────────────

    def find_stm_candidates(self):
        """Every port that could be the panel, best guess first.

        Returns a list of (device, kind, probe_timeout) rather than a
        single port, because Bluetooth makes the old "first match wins"
        approach unsafe: the host shows an SPP port whether or not the
        module is powered, and Windows shows two per paired device.
        """
        candidates = []
        seen = set()

        ports = serial.tools.list_ports.comports()

        # An explicit SERIAL_PORT in system_config always goes first, so
        # you can pin a specific port and skip discovery entirely.
        forced = getattr(system_config, "SERIAL_PORT", None)
        if forced:
            for port in ports:
                if port.device == forced:
                    candidates.append((port.device, "configured", PROBE_TIMEOUT_BT_S))
                    seen.add(port.device)

        for kind, matches, timeout in PORT_MATCHERS:
            for port in ports:
                if port.device in seen:
                    continue
                try:
                    hit = matches(port)
                except Exception:
                    hit = False
                if hit:
                    candidates.append((port.device, kind, timeout))
                    seen.add(port.device)
                    logger.info(f"Candidate: {port.device} ({kind}) — '{port.description}'")

        if not candidates:
            logger.info("No candidate ports found.")
        return candidates

    def _probe(self, ser, timeout_s) -> bool:
        """True if this port is really the panel.

        Opening a port proves nothing, especially over Bluetooth. So this
        starts a session — HELLO then one HEARTBEAT — and waits for any
        valid TLV frame to come back.

        Sending is necessary rather than just listening: the firmware is
        silent in WAITING and SYNCED, and only starts transmitting once a
        heartbeat has moved it to CONNECTED. A port that stays quiet is
        either not the panel or not powered, and both mean "try the next
        one".
        """
        parser = StreamParser()
        try:
            ser.reset_input_buffer()
            ser.write(build_frame(MessageType.HELLO, Hello()))
            ser.write(build_frame(MessageType.HEARTBEAT, Heartbeat()))
            ser.flush()
        except Exception as e:
            logger.debug(f"Probe write failed: {e}")
            return False

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                chunk = ser.read(64)
            except Exception as e:
                logger.debug(f"Probe read failed: {e}")
                return False

            if chunk and parser.feed(chunk):
                return True

            # Keep the session alive across a slow link — the firmware
            # times out in 500 ms and we may be waiting longer than that.
            try:
                ser.write(build_frame(MessageType.HEARTBEAT, Heartbeat()))
            except Exception:
                return False

        return False

    def find_stm_port(self):
        """Opens and probes each candidate, returning the first live one.

        Returns an already-open serial.Serial, not a port name — reopening
        would mean a second Bluetooth link setup and would drop the
        session the probe just established.
        """
        for device, kind, timeout in self.find_stm_candidates():
            if self.is_cancelled():
                return None

            logger.info(f"Trying {device} ({kind})...")
            try:
                ser = serial.Serial(
                    port=device,
                    baudrate=system_config.BAUDRATE,
                    timeout=system_config.SERIAL_TIMEOUT,
                )
            except Exception as e:
                # Routine for Bluetooth: a paired-but-absent module
                # refuses or times out on open.
                logger.info(f"  {device}: could not open ({e})")
                continue

            if self._probe(ser, timeout):
                logger.info(f"  {device}: panel responded — using this port.")
                return ser

            logger.info(f"  {device}: no response in {timeout}s.")
            try:
                ser.close()
            except Exception:
                pass

        logger.info("No STM32 device responded on any port.")
        return None

    # ── Outgoing — STM32 receives this ───────────────────────────────────────

    def compile_Send(self):
        """Builds the outgoing StatusUpdate from current UAV facts and
        sends it as a TLV frame."""
        status = StatusBuilder(self.state).build()
        frame = build_frame(MessageType.STATUS, status)

        logger.info(f"PC -> STM32: {status}")
        self.send(frame)
        self.state.clear_UAV_State_Changed()

    def send(self, data: bytes):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(data)
        except serial.SerialException:
            logger.error("Send failed — connection lost!")
            self.disconnect()