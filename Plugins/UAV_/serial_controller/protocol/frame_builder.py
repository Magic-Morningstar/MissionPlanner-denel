# serial_controller/protocol/frame_builder.py
#
# Mirror of stream_parser.py for the outgoing direction. Wraps any
# registered message type's encoded payload in the same TLV frame.

import struct
from serial_controller.protocol.registry import get_decoder, MessageType
from serial_controller.protocol.stream_parser import _crc8
from config import START_BYTE, END_BYTE

SYNC_SENTINEL = 0xFFFFFFFF


def build_frame(msg_type: int, obj) -> bytes:
    decoder = get_decoder(msg_type)
    if decoder is None:
        raise ValueError(f"No encoder registered for type {msg_type:#x}")

    payload = decoder.encode(obj)
    crc = _crc8(payload)

    return bytes([START_BYTE, msg_type, len(payload)]) + payload + bytes([crc, END_BYTE])


def build_sync_frame() -> bytes:
    """
    A STATUS-type (0x10) frame carrying the all-ones sync sentinel
    instead of an encoded StatusUpdate. Used only for the boot/resync
    handshake (see SerialHandler) to tell the STM32 "drop back to
    awaiting-sync." Bypasses StatusUpdateDecoder.encode() on purpose —
    that encoder only ever sets 5 of 32 bits, so 0xFFFFFFFF is never a
    value a real StatusUpdate could produce, and it stays that way as
    long as no one adds a real field on bit 31.
    """
    payload = struct.pack('<I', SYNC_SENTINEL)
    crc = _crc8(payload)
    return bytes([START_BYTE, MessageType.STATUS, len(payload)]) + payload + bytes([crc, END_BYTE])