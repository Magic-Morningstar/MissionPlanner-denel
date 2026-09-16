# serial_controller/protocol/registry.py
#
# Generic type -> codec lookup. Same shape as before; the type list is
# now two entries, one frame each way, both carrying pure physical state.


class MessageType:
    PANEL_STATE = 0x01   # STM32 -> PC: slot levels + pot readings
    LED_STATE   = 0x10   # PC -> STM32: menu index for the indicator strip


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