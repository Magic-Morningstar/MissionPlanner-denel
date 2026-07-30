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
from serial_controller.protocol.frame_builder import build_frame
from serial_controller.status_builder import StatusBuilder
import config


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
        # Testing hook only: if set, _do_connect() uses this object
        # directly instead of scanning for real hardware and opening a
        # real serial.Serial. Must support .read(n), .write(data),
        # .is_open, .close() — see testing/fake_serial.py.
        self._serial_override = serial_override

    # ── ConnectionManager interface ───────────────────────────────────────────

    def _do_connect(self):
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
        self._reader_stop.set()
        self._processor_stop.set()

        if self.ser and self.ser.is_open:
            self.ser.close()

        self.ser = None
        self.state.update_Serial_connection(False, None)
        logger.info("STM32 disconnected.")

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
            self.disconnect()
