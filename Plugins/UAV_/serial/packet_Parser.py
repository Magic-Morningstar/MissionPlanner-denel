from protocol.bit_definitions import *
class PacketParser:

    def parse(self, line: str):

        if not line:
            return None

        if line.startswith("NUM:"):
            value = int(line.split(":")[1])

            return {
                "raw": value,
                "arm": (value >> ARM_BIT) & 1,
                "rtl": (value >> 2) & 1,
                "manual": (value >> 11) & 1,
                "takeoff": (value >> 6) & 1
            }

        return None