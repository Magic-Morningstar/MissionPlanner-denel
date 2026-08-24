# serial_controller/protocol/registry.py
#
# Generic type -> decoder lookup. Adding a new message type means writing
# one Decoder subclass and calling register() — nothing in this file,
# stream_parser.py, or serial_handler.py ever needs to change again.


class MessageType:
    BUTTON_STATE     = 0x01
    JOYSTICK         = 0x02   # unchanged — still 4 bytes, x/y, exactly as before
    JOYSTICK2        = 0x03   # NEW — second stick
    PAYLOAD_COMMAND  = 0x04   # NEW — payload/gimbal/camera buttons, own 32-bit register
    STATUS           = 0x10


class Decoder:
    """
    One of these per message type. Owns the byte layout for that type
    completely — nobody outside this class should know how many bytes
    it uses or what order they're in.
    """
    TYPE = None  # set by subclass

    def decode(self, payload: bytes):
        raise NotImplementedError

    def encode(self, obj) -> bytes:
        raise NotImplementedError


_REGISTRY = {}


def register(decoder: Decoder):
    if decoder.TYPE is None:
        raise ValueError(f"{decoder.__class__.__name__} must set TYPE")
    _REGISTRY[decoder.TYPE] = decoder
    return decoder


def get_decoder(msg_type: int):
    """Returns None for unknown types — caller decides how to handle that
    (typically: log and skip, never crash)."""
    return _REGISTRY.get(msg_type)