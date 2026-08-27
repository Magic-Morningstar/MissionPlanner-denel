# serial_controller/protocol/stream_parser.py
#
# Consumes a raw byte stream and yields complete, CRC-validated
# (type, payload) frames. Frame layout:
#
#   SYNC(1) | TYPE(1) | LEN(1) | PAYLOAD(LEN) | CRC8(1) | END(1)
#
# Self-resyncing: any corruption (bad sync, bad CRC, bad end byte) just
# drops that one frame and returns to hunting for the next SYNC byte —
# never raises, never desyncs permanently, never blocks.

from enum import Enum, auto
from state.system_config import START_BYTE, END_BYTE


class _State(Enum):
    WAIT_SYNC = auto()
    READ_TYPE = auto()
    READ_LEN = auto()
    READ_PAYLOAD = auto()
    READ_CRC = auto()
    READ_END = auto()


def _crc8(data: bytes) -> int:
    """Simple XOR checksum. Swap for a real CRC8 polynomial later if
    corruption in practice turns out to need better detection."""
    crc = 0
    for b in data:
        crc ^= b
    return crc


class StreamParser:

    def __init__(self):
        self._state = _State.WAIT_SYNC
        self._type = None
        self._len = None
        self._payload = bytearray()
        self._crc_ok = False

    def feed(self, chunk: bytes):
        """Feed any number of new bytes. Returns a list of complete
        (msg_type: int, payload: bytes) tuples found in this chunk —
        usually 0 or 1, but can be more if several frames arrived at once."""
        frames = []
        for byte in chunk:
            frame = self._feed_byte(byte)
            if frame is not None:
                frames.append(frame)
        return frames

    def _feed_byte(self, byte):
        if self._state == _State.WAIT_SYNC:
            if byte == START_BYTE:
                self._state = _State.READ_TYPE

        elif self._state == _State.READ_TYPE:
            self._type = byte
            self._state = _State.READ_LEN

        elif self._state == _State.READ_LEN:
            self._len = byte
            self._payload = bytearray()
            self._state = _State.READ_PAYLOAD if self._len > 0 else _State.READ_CRC

        elif self._state == _State.READ_PAYLOAD:
            self._payload.append(byte)
            if len(self._payload) == self._len:
                self._state = _State.READ_CRC

        elif self._state == _State.READ_CRC:
            self._crc_ok = (byte == _crc8(bytes(self._payload)))
            self._state = _State.READ_END

        elif self._state == _State.READ_END:
            self._state = _State.WAIT_SYNC
            if byte == END_BYTE and self._crc_ok:
                return (self._type, bytes(self._payload))
            # Bad CRC or bad end byte — silently drop and resync.
            # Caller can add a corruption counter here later if needed.

        return None
