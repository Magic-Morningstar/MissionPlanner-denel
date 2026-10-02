# mqtt_bridge/framing.py
"""MAVLink frame extraction from a byte stream. Pure: no I/O, no sockets.

WHY THIS EXISTS INSTEAD OF pymavlink's PARSER
---------------------------------------------
The obvious implementation is recv_match() then msg.get_msgbuf(). And
get_msgbuf() really does return the exact wire bytes, signature block included,
so that part would be fine. The problem is everything around it:

  * MAVLink.decode() verifies the CRC using crc_extra taken from *our* compiled
    dialect. If the flight controller's ArduPilot build and our pinned pymavlink
    disagree about any message's crc_extra -- a routine consequence of upgrading
    one and not the other -- decode() raises, the frame becomes MAVLink_bad_data,
    and a perfectly valid frame is dropped. Silently, permanently, for that one
    message type. That is the worst possible failure mode for a tunnel whose
    entire job is to be invisible.
  * MAVLink_bad_data also carries a _msgbuf, so the naive loop happily
    republishes garbage; filtering it out is exactly what discards the real
    frames lost to the point above, with no way to tell them apart.
  * mavudp.recv_msg() calls parse_char once per datagram, so a datagram holding
    three frames yields one message and strands the other two until the next
    datagram arrives.

So this framer deliberately knows nothing about message semantics. It does not
look at the msgid, the CRC, or the dialect. It cannot drop a legitimate frame
for any semantic reason, and a dialect upgrade cannot break it. Frame integrity
is the endpoints' job and authenticity is MAV_SIGNING's -- both of which keep
working end to end precisely because we never touch the bytes.

TRADE-OFF, ACCEPTED DELIBERATELY
--------------------------------
Without a CRC check, a garbage byte that happens to equal 0xFD or 0xFE is read
as a frame start, and its bogus length field can swallow the genuine frame behind
it. This is self-correcting, and -- crucially -- visible in bad_bytes, which a
dialect-mismatch drop would not be.

On datagram transports use feed_datagram() rather than feed(): a bogus length of
up to 280 bytes would otherwise carry across datagram boundaries and eat the
heads of several following datagrams, turning one lost frame into a visible
telemetry stall. See feed_datagram() for why that remainder can safely be
discarded.

THREADING
---------
No locks. One instance per direction, fed by exactly one thread. Do not share an
instance between threads or directions; make another one, they are cheap.
"""

import logging

logger = logging.getLogger(__name__)

STX_V1 = 0xFE
STX_V2 = 0xFD

HDR_V1 = 6    # magic, len, seq, sysid, compid, msgid
HDR_V2 = 10   # magic, len, incompat, compat, seq, sysid, compid, msgid(3 bytes)

CRC_LEN = 2
SIG_LEN = 13  # MAVLINK_SIGNATURE_BLOCK_LEN: linkid(1) + timestamp(6) + sig(6)

IFLAG_SIGNED = 0x01  # MAVLINK_IFLAG_SIGNED, the only defined incompat flag

# Largest legal frame: v2 header + 255-byte payload + CRC + signature block.
MAX_FRAME_LEN = HDR_V2 + 255 + CRC_LEN + SIG_LEN   # 280

# Smallest legal frame: v1 header + empty payload + CRC.
MIN_FRAME_LEN = HDR_V1 + 0 + CRC_LEN               # 8

# How many leading bytes must be held before the total length is knowable.
_NEED_V1 = 2  # buf[1] = payload length
_NEED_V2 = 3  # buf[1] = payload length, buf[2] = incompat flags (signed?)


def frame_total_length(buf):
    """Total on-wire length of the frame starting at buf[0], or None.

    None means "not determinable": either too few bytes to read the length
    fields, or buf[0] is not a usable start byte. Callers that have already
    resynced distinguish the two by checking how many bytes they hold.
    """
    if not buf:
        return None
    stx = buf[0]
    if stx == STX_V1:
        if len(buf) < _NEED_V1:
            return None
        return HDR_V1 + buf[1] + CRC_LEN
    if stx == STX_V2:
        if len(buf) < _NEED_V2:
            return None
        incompat = buf[2]
        if incompat & ~IFLAG_SIGNED:
            # Reserved incompat bits set. The MAVLink spec requires a receiver
            # to reject such a frame, so this 0xFD is not a real frame start.
            # This is the one validity check available to us that needs no
            # dialect knowledge -- pymavlink makes exactly the same one.
            return None
        return HDR_V2 + buf[1] + CRC_LEN + (SIG_LEN if incompat & IFLAG_SIGNED else 0)
    return None


def frame_header(frame):
    """(sysid, compid, msgid) from a whole frame. Dialect-free arithmetic.

    Diagnostics only -- never routing or filtering, which would make the tunnel
    non-transparent. Returns None if frame is too short to read.
    """
    if len(frame) < HDR_V1:
        return None
    if frame[0] == STX_V1:
        return (frame[3], frame[4], frame[5])
    if frame[0] == STX_V2 and len(frame) >= HDR_V2:
        msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16)
        return (frame[5], frame[6], msgid)
    return None


class MavlinkFramer:
    """Accumulates bytes, hands back whole MAVLink frames.

    Counters are plain ints touched by the single owning thread. Reading them
    from another thread is fine for logging, where a torn read does not matter.
    """

    def __init__(self, name="framer", max_buffer=MAX_FRAME_LEN * 8):
        self.name = name
        self._buf = bytearray()
        self._max_buffer = max_buffer

        self.frames = 0            # whole frames emitted
        self.frame_bytes = 0       # bytes emitted as part of frames
        self.bad_bytes = 0         # bytes discarded while resyncing
        self.bad_events = 0        # contiguous runs of discarded bytes
        self.residue_events = 0    # datagrams that did not end on a frame boundary
        self.last_bad_sample = b""  # first 16 bytes of the most recent run

    def feed(self, data):
        """Append data; return a list of whole frames now extractable.

        Returns [] when data only partially completes a frame. Never raises on
        malformed input -- bad bytes are counted and discarded, because a tunnel
        that dies on garbage is worse than one that reports it.
        """
        if not data:
            return []
        self._buf.extend(data)

        out = []
        while self._buf:
            # 1. Resync: drop everything before the first plausible start byte.
            skip = 0
            n = len(self._buf)
            while skip < n and self._buf[skip] != STX_V1 and self._buf[skip] != STX_V2:
                skip += 1
            if skip:
                self._note_bad(self._buf[:skip])
                del self._buf[:skip]
                if not self._buf:
                    break

            # 2. How long is this frame?
            total = frame_total_length(self._buf)
            if total is None:
                need = _NEED_V1 if self._buf[0] == STX_V1 else _NEED_V2
                if len(self._buf) < need:
                    break   # genuinely need more bytes to decide
                # Enough bytes were available and it still is not a valid
                # header, so this start byte was noise. Drop just it and resync
                # past it: the real frame may begin inside what is already held.
                self._note_bad(self._buf[:1])
                del self._buf[:1]
                continue

            # 3. Complete?
            if len(self._buf) < total:
                break   # partial frame; wait for the rest

            out.append(bytes(self._buf[:total]))
            del self._buf[:total]
            self.frames += 1
            self.frame_bytes += total

        # Backstop. Unreachable by construction (total <= MAX_FRAME_LEN, and
        # every iteration either emits or discards), but an unbounded buffer fed
        # by a broken or hostile peer is its own bug.
        if len(self._buf) > self._max_buffer:
            logger.error("%s: buffer exceeded %d bytes, discarding %d",
                         self.name, self._max_buffer, len(self._buf))
            self._note_bad(self._buf)
            self._buf.clear()

        return out

    def feed_datagram(self, data):
        """feed() for datagram transports, where a datagram holds whole frames.

        Use this for UDP, never for serial or TCP. MAVLink senders do not split a
        frame across datagrams, so a non-empty buffer after a datagram means that
        datagram was corrupt -- most often a garbage byte equal to 0xFD/0xFE was
        read as a frame start and its bogus length field ate the tail.

        Because the next datagram is independent, the remainder is discarded here
        rather than carried forward. That is what bounds the damage from a
        spurious start byte to a single datagram. Without it, one bogus length
        field (up to 280 bytes) could swallow the heads of several following
        datagrams, and on a telemetry link that is a visible stall rather than a
        single lost frame.
        """
        frames = self.feed(data)
        if self._buf:
            self.residue_events += 1
            logger.debug("%s: datagram did not end on a frame boundary, "
                         "discarding %d trailing byte(s)", self.name, len(self._buf))
            self.reset()        # counts the residue into bad_bytes
        return frames

    def reset(self):
        """Discard partial state, counting it. Call on endpoint reconnect.

        A half-received frame at disconnect is genuinely lost; carrying it
        across a reconnect would splice two unrelated streams together.
        """
        if self._buf:
            self._note_bad(self._buf)
            self._buf.clear()

    @property
    def pending(self):
        """Bytes held awaiting the rest of a frame. Diagnostics only."""
        return len(self._buf)

    def counters(self):
        return {
            "frames": self.frames,
            "frame_bytes": self.frame_bytes,
            "bad_bytes": self.bad_bytes,
            "bad_events": self.bad_events,
            "residue_events": self.residue_events,
        }

    def _note_bad(self, chunk):
        self.bad_bytes += len(chunk)
        self.bad_events += 1
        self.last_bad_sample = bytes(chunk[:16])

    def __repr__(self):
        return (f"<MavlinkFramer {self.name} frames={self.frames} "
                f"bad_bytes={self.bad_bytes} pending={self.pending}>")
