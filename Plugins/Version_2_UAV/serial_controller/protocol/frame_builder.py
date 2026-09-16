# serial_controller/protocol/frame_builder.py
#
# Mirror of stream_parser.py for the outgoing direction. Wraps any
# registered message type's encoded payload in the same TLV frame.

from serial_controller.protocol.registry import get_decoder
from serial_controller.protocol.stream_parser import _crc8
from state.system_config import START_BYTE, END_BYTE

# main.c's TLV_Send() caps a frame at sizeof(frame[0]) - 5 = 59 payload
# bytes and silently returns 0 above that. Fail loudly on this side
# rather than emitting a frame the firmware will refuse to send back.
MAX_PAYLOAD_LEN = 59


def build_frame(msg_type: int, obj) -> bytes:
    decoder = get_decoder(msg_type)
    if decoder is None:
        raise ValueError(f"No encoder registered for type {msg_type:#x}")

    payload = decoder.encode(obj)
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError(
            f"Payload for type {msg_type:#x} is {len(payload)} bytes, "
            f"max is {MAX_PAYLOAD_LEN}"
        )

    crc = _crc8(payload)
    return bytes([START_BYTE, msg_type, len(payload)]) + payload + bytes([crc, END_BYTE])