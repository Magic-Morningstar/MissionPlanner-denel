
from protocol.bit_definitions import *

class PacketBuilder:

    def build(self,message, state):


        if state.is_flying:
            message |= (state.flying_bit<< IS_FLYING_BIT)

        # Convert to bytes for sending
        return f"NUM:{message}\n".encode()