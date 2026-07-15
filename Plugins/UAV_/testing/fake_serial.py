# testing/fake_serial.py
#
# A minimal stand-in for a pyserial Serial object, backed by two plain
# queues instead of a real COM port. Portable — no pty, no com0com, no
# OS-specific virtual serial setup. Works identically on Windows/Mac/Linux.
#
# Two of these, sharing swapped queues, form a duplex link: whatever one
# side writes(), the other side's read() eventually returns.

import queue


class FakeSerialEnd:
    """Enough of pyserial's interface for SerialHandler: read(n), write(data),
    is_open, close(). Deliberately does NOT try to replicate baud-rate
    timing or byte-level chunking quirks — those aren't what we're testing."""

    def __init__(self, inbound: queue.Queue, outbound: queue.Queue, timeout=0.1):
        self._inbound = inbound
        self._outbound = outbound
        self._timeout = timeout
        self.is_open = True

    def read(self, size=1):
        """Real pyserial blocks up to `timeout` waiting for at least one
        byte, then returns whatever's available (not necessarily `size`
        bytes). We replicate that loosely: block briefly for one queued
        chunk, return it whole."""
        if not self.is_open:
            return b''
        try:
            return self._inbound.get(timeout=self._timeout)
        except queue.Empty:
            return b''

    def write(self, data: bytes):
        if not self.is_open:
            return
        self._outbound.put(bytes(data))

    def close(self):
        self.is_open = False


def make_fake_serial_pair(timeout=0.1):
    """
    Returns (stm_side, pc_side). Bytes written on one arrive as reads
    on the other:

        stm_side.write(frame)  ->  pc_side.read(...) returns frame
        pc_side.write(frame)   ->  stm_side.read(...) returns frame

    Give `pc_side` to SerialHandler(serial_override=pc_side).
    Use `stm_side` from your keyboard simulator to act as the STM32.
    """
    stm_to_pc = queue.Queue()
    pc_to_stm = queue.Queue()
    stm_side = FakeSerialEnd(inbound=pc_to_stm, outbound=stm_to_pc, timeout=timeout)
    pc_side  = FakeSerialEnd(inbound=stm_to_pc, outbound=pc_to_stm, timeout=timeout)
    return stm_side, pc_side
