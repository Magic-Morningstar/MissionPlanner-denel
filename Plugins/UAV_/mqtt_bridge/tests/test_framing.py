# mqtt_bridge/tests/test_framing.py
"""Framing tests.

These carry most of the weight for brief Phase 1 acceptance #3 (byte-for-byte
integrity). Framing is the only lossy component in the chain: MQTT payloads are
opaque binary and a UDP datagram is atomic, so both hops preserve bytes by
construction. Proving the framer byte-exact under arbitrary segmentation
therefore proves the tunnel, without needing a broker or SITL.
"""

import pytest

from mqtt_bridge.framing import (
    MAX_FRAME_LEN, MIN_FRAME_LEN, MavlinkFramer, frame_header, frame_total_length,
)

from .conftest import make_v1, make_v2


# --------------------------------------------------------------------------
# Length arithmetic, at both extremes of both versions
# --------------------------------------------------------------------------

@pytest.mark.parametrize("frame,expected", [
    (make_v1(payload_len=0), 8),
    (make_v1(payload_len=255), 263),
    (make_v2(payload_len=0), 12),
    (make_v2(payload_len=255), 267),
    (make_v2(payload_len=0, signed=True), 25),
    (make_v2(payload_len=255, signed=True), 280),
])
def test_frame_total_length(frame, expected):
    assert len(frame) == expected
    assert frame_total_length(frame) == expected


def test_length_constants():
    assert MAX_FRAME_LEN == 280
    assert MIN_FRAME_LEN == 8


def test_signature_block_is_counted():
    """A signed frame is 13 bytes longer than the same frame unsigned.

    Getting this wrong truncates every signed frame, which would break
    MAV_SIGNING end to end -- the thing the raw tunnel exists to preserve.
    """
    unsigned = make_v2(payload_len=20, signed=False)
    signed = make_v2(payload_len=20, signed=True)
    assert len(signed) - len(unsigned) == 13


# --------------------------------------------------------------------------
# The core property: byte-exact reassembly under arbitrary segmentation
# --------------------------------------------------------------------------

def test_single_frames_round_trip(frame_corpus):
    for frame in frame_corpus:
        f = MavlinkFramer("single")
        assert f.feed(frame) == [frame]
        assert f.bad_bytes == 0
        assert f.pending == 0


def test_three_frames_in_one_feed():
    """One UDP datagram can hold several frames; all must come out at once.

    This is the case pymavlink's recv_msg() gets wrong -- it calls parse_char
    once per datagram and strands the rest.
    """
    frames = [make_v2(payload_len=9), make_v1(payload_len=4), make_v2(payload_len=0, signed=True)]
    f = MavlinkFramer("multi")
    assert f.feed(b"".join(frames)) == frames
    assert f.frames == 3
    assert f.bad_bytes == 0


@pytest.mark.parametrize("chunk", [1, 2, 3, 7, 11, 64, 4096])
def test_byte_split_reassembly(frame_stream, frame_corpus, chunk):
    """Feed the whole corpus in fixed-size chunks; output must be identical.

    chunk=1 is the serial worst case (a byte at a time, splitting every header
    field); chunk=4096 is the UDP case (many frames per read). Both must produce
    exactly the same frames in exactly the same order.
    """
    f = MavlinkFramer(f"split{chunk}")
    collected = []
    for i in range(0, len(frame_stream), chunk):
        collected.extend(f.feed(frame_stream[i:i + chunk]))

    assert collected == frame_corpus
    assert b"".join(collected) == frame_stream
    assert f.bad_bytes == 0
    assert f.bad_events == 0
    assert f.pending == 0


def test_payload_containing_start_bytes():
    """0xFD/0xFE inside a payload must not be mistaken for a frame start.

    The framer is length-driven, not scan-driven. The corpus payloads already
    contain both start bytes; this asserts it explicitly.
    """
    frame = make_v2(payload_len=32)
    assert b"\xfd" in frame[10:] and b"\xfe" in frame[10:]
    f = MavlinkFramer("stx-in-payload")
    assert f.feed(frame) == [frame]
    assert f.bad_bytes == 0


def test_partial_frame_is_withheld_then_completed():
    frame = make_v2(payload_len=50)
    f = MavlinkFramer("partial")
    assert f.feed(frame[:-1]) == []
    assert f.pending == len(frame) - 1
    assert f.bad_bytes == 0          # withheld, not discarded
    assert f.feed(frame[-1:]) == [frame]
    assert f.pending == 0


# --------------------------------------------------------------------------
# Bad input is counted, never silently dropped
# --------------------------------------------------------------------------

def test_leading_garbage_is_counted():
    frame = make_v2(payload_len=8)
    f = MavlinkFramer("garbage")
    assert f.feed(b"\x01\x02\x03" + frame) == [frame]
    assert f.bad_bytes == 3
    assert f.bad_events == 1
    assert f.last_bad_sample == b"\x01\x02\x03"


def test_lone_nat_punch_byte_counts_exactly_one():
    """Mission Planner's UdpSerialConnect sends a single 0x00 before any MAVLink.

    AutoConnect.ProcessEntry does the same on its outbound paths. So on a real
    connect, bad_bytes should read exactly 1 -- which makes it a live check that
    the accounting works, rather than a number nobody ever looks at.
    """
    f = MavlinkFramer("nat-punch")
    assert f.feed(b"\x00") == []
    assert f.bad_bytes == 1
    assert f.pending == 0            # not held, it can never start a frame

    frame = make_v2(payload_len=9)
    assert f.feed(frame) == [frame]
    assert f.bad_bytes == 1          # unchanged by the good frame


def test_reserved_incompat_flag_is_rejected():
    """incompat bits other than SIGNED mean the frame must be rejected.

    The spec requires it, and it is the only validity check available without the
    dialect -- pymavlink makes the same one.
    """
    bogus = make_v2(payload_len=8, incompat=0x02)
    assert frame_total_length(bogus) is None

    f = MavlinkFramer("reserved")
    assert f.feed(bogus) == []
    assert f.bad_bytes + f.pending == len(bogus)   # every byte accounted for


def test_resync_after_noise():
    """Noise containing no start byte is skipped and the next frame recovered."""
    good = make_v1(payload_len=4)
    f = MavlinkFramer("resync")
    assert f.feed(b"\x11\x22\x33\x44\x55" + good) == [good]
    assert f.bad_bytes == 5


def test_spurious_start_byte_can_swallow_the_next_frame():
    """The documented cost of not validating CRCs, pinned so it stays known.

    A garbage 0xFD/0xFE is read as a frame start, and its bogus length field can
    consume the genuine frame behind it. The guarantees are only that the framer
    never deadlocks, never raises, and accounts for every byte -- NOT that the
    following frame survives.

    If a future change makes recovery better, this test should be updated rather
    than deleted; if it makes it worse, this is where it shows up.
    """
    bogus = make_v2(payload_len=8, incompat=0x02)   # its payload contains 0xFE
    good = make_v1(payload_len=4)
    f = MavlinkFramer("swallow")
    out = f.feed(bogus + good)

    assert out == []                                 # the good frame was eaten
    assert f.bad_bytes + f.pending == len(bogus) + len(good)
    assert f.frames == 0

    # Still healthy afterwards: once enough bytes arrive to satisfy the bogus
    # length, framing resumes normally.
    filler = make_v2(payload_len=255) * 2
    recovered = f.feed(filler)
    assert f.pending < MAX_FRAME_LEN
    assert isinstance(recovered, list)


def test_datagram_mode_bounds_the_damage_to_one_datagram():
    """feed_datagram() must not let a bogus length eat the NEXT datagram.

    MAVLink senders never split a frame across datagrams, so leftover bytes mean
    this datagram was corrupt. Discarding them is what keeps the blast radius at
    one datagram -- with plain feed(), the bogus 239-byte length below would eat
    the whole of the following datagram too.
    """
    bogus = make_v2(payload_len=8, incompat=0x02)
    good = make_v1(payload_len=4)
    nxt = make_v2(payload_len=9)

    streaming = MavlinkFramer("stream")
    streaming.feed(bogus + good)
    assert streaming.pending > 0                     # carries corruption forward
    assert streaming.feed(nxt) == []                 # ...and eats the next datagram

    datagrams = MavlinkFramer("datagram")
    assert datagrams.feed_datagram(bogus + good) == []
    assert datagrams.pending == 0                    # corruption did not survive
    assert datagrams.residue_events == 1
    assert datagrams.feed_datagram(nxt) == [nxt]     # next datagram is clean


def test_feed_datagram_is_transparent_for_clean_input(frame_corpus):
    """Well-formed datagrams must behave exactly as feed() does."""
    f = MavlinkFramer("datagram-clean")
    for frame in frame_corpus:
        assert f.feed_datagram(frame) == [frame]
    assert f.bad_bytes == 0
    assert f.residue_events == 0


def test_reset_counts_the_partial():
    frame = make_v2(payload_len=100)
    f = MavlinkFramer("reset")
    f.feed(frame[:30])
    assert f.pending == 30
    f.reset()
    assert f.pending == 0
    assert f.bad_bytes == 30         # accounted for, not vanished


def test_buffer_backstop():
    f = MavlinkFramer("bounded", max_buffer=512)
    f.feed(b"\xfd\xff" + b"\x00" * 2000)   # a frame header promising more than arrives
    assert f.pending <= 512
    assert f.bad_bytes > 0


def test_empty_feed_is_a_noop():
    f = MavlinkFramer("empty")
    assert f.feed(b"") == []
    assert f.feed(None) == []
    assert f.bad_bytes == 0


# --------------------------------------------------------------------------
# Transparency: things a semantic parser would reject must still pass through
# --------------------------------------------------------------------------

def test_unknown_msgid_passes_through():
    """A msgid in no dialect is still a frame. Forwarding it is correct.

    pymavlink yields MAVLink_unknown here; a tunnel that filtered on msgid would
    break any dialect extension the FC has and we do not.
    """
    frame = make_v2(payload_len=16, msgid=0xFFFFF0)
    f = MavlinkFramer("unknown-msgid")
    assert f.feed(frame) == [frame]
    assert frame_header(frame)[2] == 0xFFFFF0


def test_bad_crc_passes_through_unchanged():
    """INTENTIONAL: the framer does not validate CRCs. Pinned by this test.

    If someone later "fixes" the framer by adding CRC validation, this fails and
    they read the docstring in framing.py explaining why a dialect-dependent
    check would silently drop valid frames.
    """
    frame = make_v2(payload_len=9, msgid=0, crc=0x0000)
    f = MavlinkFramer("bad-crc")
    assert f.feed(frame) == [frame]
    assert f.bad_bytes == 0


def test_bad_crc_interop():
    """The design rationale, demonstrated: pymavlink rejects it, we forward it.

    A CRC mismatch is exactly what a dialect-version disagreement looks like
    from the receiving side. pymavlink turns it into BAD_DATA; our tunnel must
    carry it byte-for-byte and let the real endpoint decide.
    """
    from pymavlink.dialects.v20 import ardupilotmega as dialect

    mav = dialect.MAVLink(None, srcSystem=1, srcComponent=1)
    mav.robust_parsing = True
    good = mav.heartbeat_encode(6, 3, 81, 0, 4).pack(mav)

    # Corrupt only the CRC, leaving the header and payload intact.
    corrupt = bytearray(good)
    corrupt[-1] ^= 0xFF
    corrupt = bytes(corrupt)

    # pymavlink refuses it...
    parser = dialect.MAVLink(None)
    parser.robust_parsing = True
    msgs = [m for m in (parser.parse_char(bytes([b])) for b in corrupt) if m is not None]
    assert any(m.get_type() == "BAD_DATA" for m in msgs), \
        "expected pymavlink to reject a corrupted CRC"

    # ...and the framer carries it through unchanged, which is the point.
    f = MavlinkFramer("interop")
    assert f.feed(corrupt) == [corrupt]
    assert f.bad_bytes == 0

    # Sanity: the uncorrupted frame is accepted by pymavlink, so the only
    # difference between the two cases really is the CRC.
    ok = dialect.MAVLink(None)
    ok.robust_parsing = True
    good_msgs = [m for m in (ok.parse_char(bytes([b])) for b in good) if m is not None]
    assert any(m.get_type() == "HEARTBEAT" for m in good_msgs)


def test_real_pymavlink_frames_round_trip():
    """Frames produced by pymavlink, including a signed one, survive the framer.

    Guards the arithmetic against a real encoder rather than only our own
    hand-built fixtures.
    """
    from pymavlink.dialects.v20 import ardupilotmega as dialect

    mav = dialect.MAVLink(None, srcSystem=1, srcComponent=1)
    unsigned = mav.heartbeat_encode(6, 3, 81, 0, 4).pack(mav)

    signer = dialect.MAVLink(None, srcSystem=1, srcComponent=1)
    signer.signing.secret_key = bytes(range(32))
    signer.signing.link_id = 1
    signer.signing.timestamp = 1
    signer.signing.sign_outgoing = True
    signed = signer.heartbeat_encode(6, 3, 81, 0, 4).pack(signer)

    assert len(signed) - len(unsigned) == 13, "signed frame should carry 13 extra bytes"

    f = MavlinkFramer("pymavlink-real")
    assert f.feed(unsigned + signed) == [unsigned, signed]
    assert f.bad_bytes == 0


# --------------------------------------------------------------------------
# Header accessor (diagnostics only)
# --------------------------------------------------------------------------

def test_frame_header_v1_and_v2():
    assert frame_header(make_v1(sysid=42, compid=7, msgid=33)) == (42, 7, 33)
    assert frame_header(make_v2(sysid=255, compid=190, msgid=0x4E2)) == (255, 190, 0x4E2)
    assert frame_header(b"\xfd\x00") is None
