# serial_controller/protocol/frame_builder.py
#
# Mirror of stream_parser.py for the outgoing direction. Wraps any
# registered message type's encoded payload in the same TLV frame.

from serial_controller.protocol.registry import get_decoder
from serial_controller.protocol.stream_parser import _crc8
from config import START_BYTE, END_BYTE


def build_frame(msg_type: int, obj) -> bytes:
    decoder = get_decoder(msg_type)
    if decoder is None:
        raise ValueError(f"No encoder registered for type {msg_type:#x}")

    payload = decoder.encode(obj)
    crc = _crc8(payload)

    return bytes([START_BYTE, msg_type, len(payload)]) + payload + bytes([crc, END_BYTE])
