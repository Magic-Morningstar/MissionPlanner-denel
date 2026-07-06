from protocols.bit_definitions import *

class Serializer:

    def serialize(self, state):

        value = 0

        if state.arm:
            value |= (1 << ARM_BIT)

        if state.rtl:
            value |= (1 << RTL_BIT)

        if state.manual_mode:
            value |= (1 << MANUAL_BIT)

        return value