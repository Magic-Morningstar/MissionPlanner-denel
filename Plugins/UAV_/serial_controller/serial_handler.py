# serial_controller/serial_handler.py

import serial
import serial.tools.list_ports
import queue
from utils.helper import system_Print
import threading
import logging
logger = logging.getLogger(__name__)
from connection_manager import ConnectionManager
from serial_controller.protocol.registry import get_decoder, MessageType
from serial_controller.protocol.stream_parser import StreamParser
from serial_controller.protocol.frame_builder import build_frame, build_sync_frame, SYNC_SENTINEL
from serial_controller.status_builder import StatusBuilder
import struct
import config

# Wire form of SYNC_SENTINEL as it arrives in a BUTTON_STATE payload —
# compared directly against raw bytes so the sentinel is intercepted
# before it ever reaches ButtonStateDecoder (which would otherwise
# decode it as "every button pressed" and hand that to the translator).
_SYNC_SENTINEL_PAYLOAD = struct.pack('<I', SYNC_SENTINEL)


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
        # Set by _processor_loop the moment the STM32 announces the
        # 0xFFFFFFFF sync sentinel (fresh boot, or echoing a resync
        # request). _sync_watchdog waits on this after every connect to
        # decide whether it needs to force a resync itself.
        self._sentinel_seen = threading.Event()
        # Set only by our own disconnect() override — signals "give up
        # retrying," not "tear down for a retry." Watched by
        # _reconnect_loop() between attempts.
        self._reconnect_stop = threading.Event()
        # Testing hook only: if set, _do_connect() uses this object
        # directly instead of scanning for real hardware and opening a
        # real serial.Serial. Must support .read(n), .write(data),
        # .is_open, .close() — see testing/fake_serial.py.
        self._serial_override = serial_override

        if self.watchdog:
            self.watchdog.register_serial_timeout_callback(self._on_watchdog_serial_timeout)

    # ── ConnectionManager interface ───────────────────────────────────────────

    def disconnect(self):
        """
        Overrides ConnectionManager.disconnect() only to also cancel any
        in-flight watchdog-triggered reconnect loop — otherwise shutting
        down while the STM32 happens to be unreachable would leave a
        retry loop spinning in the background after the app has asked
        to stop.
        """
        self._reconnect_stop.set()
        super().disconnect()

    def _do_connect(self):
        # A fresh connect attempt (first connect, or a watchdog-triggered
        # reconnect) means any earlier "please stop retrying" request no
        # longer applies — clear it so a later watchdog timeout can
        # trigger a new reconnect loop.
        self._reconnect_stop.clear()

        if self.is_cancelled():
            return False

        if self._serial_override is not None:
            new_ser = self._serial_override
            logger.info("SerialHandler: using injected test link (no real hardware).")
        else:
            detected_port = self.find_stm_port()

            if self.is_cancelled():
                logger.warning("Serial connection cancelled.")
                return False

            if detected_port is None:
                self.state.update_Serial_connection(False, None)
                return False

            new_ser = serial.Serial(
                port=detected_port,
                baudrate=config.BAUDRATE,
                timeout=config.SERIAL_TIMEOUT
            )

        if self.is_cancelled():
            new_ser.close()
            logger.warning("Serial connection cancelled.")
            return False

        self.ser = new_ser
        self.state.update_Serial_connection(True, self.ser)
        if self._serial_override is None:
            logger.info(f"STM32 connected on {detected_port}.")
        else:
            logger.info("STM32 connected (test link).")

        if self.watchdog:
            self.watchdog.watchSerialThread()

        self._reader_stop.clear()
        self._processor_stop.clear()
        self._sentinel_seen.clear()

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

        threading.Thread(
            target=self._sync_watchdog,
            daemon=True,
            name="SerialSyncWatchdog"
        ).start()

        return True

    def _do_disconnect(self):
        self._reader_stop.set()
        self._processor_stop.set()

        if self.ser and self.ser.is_open:
            self.ser.close()

        self.ser = None
        self.state.update_Serial_connection(False, None)
        logger.info("STM32 disconnected.")

    # ── Watchdog-triggered reconnect ──────────────────────────────────────────

    def _on_watchdog_serial_timeout(self):
        """
        Fires on the Watchdog's own monitor thread — must not block it.
        Hands off to a dedicated thread so the retry loop (which does
        real IO and can sleep for tens of seconds on backoff) never
        stalls the shared watchdog loop that also monitors the mavlink
        threads.
        """
        logger.warning("SerialHandler: watchdog reports the serial link is dead — reconnecting.")
        threading.Thread(target=self._reconnect_loop, daemon=True, name="SerialReconnect").start()

    def _reconnect_loop(self):
        """
        Tears down the dead connection, then retries connect() with
        exponential backoff (capped) until it succeeds or disconnect()
        is called explicitly (which sets _reconnect_stop). Each
        successful connect() goes through the normal _do_connect() path,
        which clears _sentinel_seen — so a reconnect always requires a
        fresh handshake before _processor_loop will dispatch anything to
        the translator again (see the _sentinel_seen gate there).

        Uses _do_disconnect() directly rather than self.disconnect():
        the latter would set _reconnect_stop and cancel this very loop.
        """
        self._do_disconnect()

        delay = 1.0
        max_delay = 30.0

        while not self._reconnect_stop.is_set():
            logger.info("SerialHandler: attempting reconnect...")
            t = self.connect()
            if t is not None:
                t.join()

            if self.state.is_Serial_Connection_Available:
                logger.info("SerialHandler: reconnect successful.")
                return

            logger.info(f"SerialHandler: STM32 not found — retrying in {delay:.0f}s.")
            if self._reconnect_stop.wait(timeout=delay):
                return
            delay = min(delay * 2, max_delay)

    # ── Boot/resync handshake ─────────────────────────────────────────────────

    def _sync_watchdog(self):
        """
        Runs once per connect. We can't tell, right after connecting,
        whether the STM32 just booted (it'll announce the 0xFFFFFFFF
        sentinel on its own within a couple hundred ms) or was already
        running from a previous session (it'll just be sending real
        button data and will never announce the sentinel unprompted).

        Give it a short window to announce itself. If it doesn't, force
        the issue: send it the sentinel ourselves. The STM32 treats a
        STATUS frame carrying 0xFFFFFFFF as "drop back to awaiting-sync"
        and starts announcing the sentinel via BUTTON_STATE, which
        _on_sync_sentinel() below then answers exactly like the fresh-
        boot case.
        """
        if self._sentinel_seen.wait(timeout=2.0):
            return
        if self._reader_stop.is_set():
            return
        logger.info("SerialHandler: STM32 didn't announce a sync sentinel — requesting resync.")
        self.send(build_sync_frame())

    def _on_sync_sentinel(self):
        """
        STM32 is in its awaiting-sync state and just told us so (via an
        all-ones BUTTON_STATE frame) — either it's fresh off boot, or
        it's answering a resync we just forced in _sync_watchdog. Either
        way: send it the real system state now. It'll keep re-announcing
        the sentinel every ~100ms until it gets that, so this is safe to
        call more than once.
        """
        self._sentinel_seen.set()
        logger.info("SerialHandler: STM32 requested sync — sending current state.")
        self.compile_Send()

    # ── Background threads ────────────────────────────────────────────────────

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
                # _do_disconnect(), not self.disconnect(): this is an
                # unexpected error, not the user asking us to stop — the
                # watchdog should still be free to trigger a reconnect
                # once it notices the heartbeat has stopped.
                self._do_disconnect()
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

                # Must be checked before decode/dispatch: as real
                # ButtonState data this would decode as "every button
                # pressed," which is not what it means here.
                if msg_type == MessageType.BUTTON_STATE and payload == _SYNC_SENTINEL_PAYLOAD:
                    self._on_sync_sentinel()
                    self._frame_queue.task_done()
                    continue

                if not self._sentinel_seen.is_set():
                    # Handshake for this connection hasn't completed yet
                    # — hold off on dispatching anything else to the
                    # translator. Matters most right after a reconnect:
                    # _sentinel_seen is cleared on every _do_connect(), so
                    # this guarantees the PC always re-syncs before it
                    # acts on anything the STM32 sends.
                    logger.debug(f"SerialProcessor: sync not complete yet — dropping type {msg_type:#x}")
                    self._frame_queue.task_done()
                    continue

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

    def find_stm_port(self):
        logger.info("Scanning for STM32 on COM ports...")
        ports = serial.tools.list_ports.comports()

        for port in ports:
            if port.vid == 0x0483 and port.pid == 0x5740:
                logger.info(f"Found STM32 (USB CDC) on {port.device} "
                            f"(VID:PID {port.vid:04X}:{port.pid:04X})")
                return port.device

        for port in ports:
            if port.vid == 0x1A86 and port.pid == 0x7523:
                logger.info(f"Found STM32 (DFRobot CH340) on {port.device} "
                            f"(VID:PID {port.vid:04X}:{port.pid:04X})")
                return port.device
            

            if "USB-Enhanced-SERIAL-D" in port.description:
                logger.info(f"Found STM32 (DFRobot CH340, matched by description) "
                            f"on {port.device} — '{port.description}'")
                return port.device

        logger.info("No STM32 device found on any COM port.")
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
            # See _reader_loop's SerialException handler: internal
            # teardown only, must not block the watchdog's reconnect.
            self._do_disconnect()