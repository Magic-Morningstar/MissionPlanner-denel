# serial_controller/serial_handler.py

import time
import serial
import serial.tools.list_ports
import queue
import threading
import logging

from utils.connection_manager import ConnectionManager
from serial_controller.protocol.registry import get_decoder, MessageType
from serial_controller.protocol.stream_parser import StreamParser
from serial_controller.protocol.frame_builder import build_frame
from serial_controller.input_router import InputRouter
import state.system_config as system_config

logger = logging.getLogger(__name__)


class SerialHandler(ConnectionManager):
    """
    Owns the serial connection to the STM32.

    Same thread shape as before — a dedicated reader thread and a
    dedicated processor thread connected by a queue, so serial IO is
    never blocked by decode work.

    What changed with the panel rework: there is no connect handshake at
    all. HELLO existed to reset the STM32 so it wouldn't carry stale
    menu_register / USB_MESSAGE / PAYLOAD_MESSAGE state across a Python
    restart. None of those exist anymore — the firmware holds no
    interpreted state to go stale — so the protocol is connectionless in
    the useful sense: either end can restart without the other noticing.

    The clean start moved into InputRouter, which seeds its edge baseline
    from the first frame and fires no events for it.
    """

    def __init__(self, state, translator, watchdog=None, serial_override=None):
        super().__init__()
        self.ser = None
        self.state = state
        self.translator = translator
        self.watchdog = watchdog

        self.router = InputRouter(state, translator)

        self._frame_queue = queue.Queue()
        self._reader_stop = threading.Event()
        self._processor_stop = threading.Event()

        self._last_led = None
        self._last_led_sent = 0.0

        # Testing hook only: if set, _do_connect() uses this object
        # directly instead of scanning for real hardware. Must support
        # .read(n), .write(data), .is_open, .close().
        self._serial_override = serial_override

    # ── ConnectionManager interface ───────────────────────────────────────────

    def _do_connect(self):
        if self.is_cancelled():
            return False

        detected_port = None

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
                baudrate=system_config.BAUDRATE,
                timeout=system_config.SERIAL_TIMEOUT
            )

        if self.is_cancelled():
            new_ser.close()
            logger.warning("Serial connection cancelled.")
            return False

        self.ser = new_ser
        self.state.update_Serial_connection(True, self.ser)
        logger.info(
            f"STM32 connected on {detected_port}." if detected_port
            else "STM32 connected (test link)."
        )

        if self.watchdog:
            self.watchdog.watchSerialThread()

        self._reader_stop.clear()
        self._processor_stop.clear()

        threading.Thread(
            target=self._reader_loop, daemon=True, name="SerialReader"
        ).start()

        threading.Thread(
            target=self._processor_loop, daemon=True, name="SerialProcessor"
        ).start()

        return True

    def _do_disconnect(self):
        self._reader_stop.set()
        self._processor_stop.set()

        if self.ser and self.ser.is_open:
            self.ser.close()

        self.ser = None
        self._last_led = None
        self.state.update_Serial_connection(False, None)
        logger.info("STM32 disconnected.")

    # ── Background threads ────────────────────────────────────────────────────

    def _reader_loop(self):
        """
        Reads raw bytes and feeds them into the TLV StreamParser. Only
        pets the watchdog when real bytes actually arrive — a stalled
        STM32 that keeps the port open but stops transmitting correctly
        trips the watchdog.
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
        Drains the frame queue, decodes, and hands each PanelState to the
        router. Unknown types are skipped, not fatal.

        The broad exception handler is deliberate and was previously
        commented out: without it, one malformed payload kills this
        thread silently while the reader keeps enqueueing, and the queue
        grows without bound.
        """
        logger.info("SerialProcessor: started.")

        while not self._processor_stop.is_set():
            try:
                msg_type, payload = self._frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                decoder = get_decoder(msg_type)
                if decoder is None:
                    logger.info(f"SerialProcessor: unknown type {msg_type:#x} — skipping")
                    continue

                obj = decoder.decode(payload)

                if msg_type == MessageType.PANEL_STATE:
                    changed = self.router.handle(obj)
                    self._maybe_send_leds(changed)

            except Exception:
                logger.exception(f"SerialProcessor: error on type {msg_type:#x}")
            finally:
                self._frame_queue.task_done()

        logger.info("SerialProcessor: stopped.")

    # ── Outgoing — STM32 receives this ───────────────────────────────────────

    def _maybe_send_leds(self, changed: bool):
        """
        Sends on change, plus a slow heartbeat. Change-triggered alone has
        the same flaw the old STATUS path had: one corrupted frame would
        leave an LED wrong until the next unrelated change. At 23 payload
        bytes every 200 ms the heartbeat costs nothing.

        Driven off the incoming 100 Hz stream rather than its own timer
        thread — if frames stop arriving, there's nothing to light anyway.
        """
        now = time.monotonic()
        stale = (now - self._last_led_sent) >= system_config.LED_HEARTBEAT_S

        if not (changed or stale):
            return

        led = self.router.led_state()
        if led == self._last_led and not stale:
            return

        self.send(build_frame(MessageType.LED_STATE, led))
        self._last_led = led
        self._last_led_sent = now

    def send(self, data: bytes):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(data)
        except serial.SerialException:
            logger.error("Send failed — connection lost!")
            self.disconnect()

    # ── Port detection ────────────────────────────────────────────────────────

    def find_stm_port(self):
        logger.info("Scanning for STM32 on COM ports...")
        ports = serial.tools.list_ports.comports()

        # Native USB CDC. The firmware currently transmits on USART2
        # through a CH340 adapter, so this branch never matches today —
        # but it's viable again now that HELLO is gone. The reason native
        # CDC was a problem before was that NVIC_SystemReset() drops USB
        # enumeration mid-reset and leaves the port handle stale; with no
        # reset in the protocol, that objection disappears.
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