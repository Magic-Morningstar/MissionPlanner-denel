# mqtt_bridge/tests/test_endpoint_udp.py
"""UdpPeerEndpoint behaviour, exercised against real loopback sockets."""

import socket

import pytest

from mqtt_bridge.mavlink_endpoint import EndpointClosed, UdpPeerEndpoint

from .conftest import make_v2


@pytest.fixture
def endpoint():
    ep = UdpPeerEndpoint("127.0.0.1", 0, name="test")   # port 0 = pick one free
    ep.open()
    # Port 0 means the real port is only known after bind.
    ep.port = ep._sock.getsockname()[1]
    yield ep
    ep.close()


@pytest.fixture
def peer():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    s.settimeout(2.0)
    yield s
    s.close()


def test_peer_is_learned_from_the_first_datagram(endpoint, peer):
    assert endpoint.peer is None
    frame = make_v2(payload_len=9)
    peer.sendto(frame, ("127.0.0.1", endpoint.port))

    assert endpoint.select(2.0)
    assert endpoint.recv() == frame
    assert endpoint.peer == peer.getsockname()


def test_write_before_a_peer_is_known_is_a_noop(endpoint):
    """Returning False rather than raising: nothing has connected yet."""
    assert endpoint.write(b"\xfd\x00") is False


def test_write_reaches_the_learned_peer(endpoint, peer):
    peer.sendto(b"\x00", ("127.0.0.1", endpoint.port))   # the NAT-punch byte
    endpoint.select(2.0)
    endpoint.recv()

    frame = make_v2(payload_len=16)
    assert endpoint.write(frame) is True
    assert peer.recv(4096) == frame


def test_last_peer_policy_replaces_the_old_peer(endpoint):
    """Mission Planner reconnects on a new ephemeral port; the old one must stop.

    This is the behaviour mavudp gets wrong: with its timeout permanently 0 it
    sends to every peer it has ever heard from and never evicts any.
    """
    first = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    second = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Bind explicitly: an unbound socket auto-binds to 0.0.0.0 on first send, so
    # getsockname() would not match the 127.0.0.1 address the endpoint sees.
    first.bind(("127.0.0.1", 0))
    second.bind(("127.0.0.1", 0))
    first.settimeout(0.3)
    second.settimeout(2.0)
    try:
        first.sendto(b"\x00", ("127.0.0.1", endpoint.port))
        endpoint.select(2.0)
        endpoint.recv()
        assert endpoint.peer == first.getsockname()

        second.sendto(b"\x00", ("127.0.0.1", endpoint.port))
        endpoint.select(2.0)
        endpoint.recv()
        assert endpoint.peer == second.getsockname()
        assert endpoint.peer_changes == 1

        frame = make_v2(payload_len=8)
        endpoint.write(frame)
        assert second.recv(4096) == frame
        with pytest.raises(socket.timeout):
            first.recv(4096)        # the replaced peer gets nothing
    finally:
        first.close()
        second.close()


def test_multiple_frames_in_one_datagram_come_back_whole(endpoint, peer):
    """A datagram may hold several frames; recv returns the datagram verbatim."""
    frames = [make_v2(payload_len=9), make_v1(4), make_v2(payload_len=0, signed=True)]
    blob = b"".join(frames)
    peer.sendto(blob, ("127.0.0.1", endpoint.port))
    endpoint.select(2.0)
    assert endpoint.recv() == blob


def test_recv_on_a_closed_endpoint_raises():
    ep = UdpPeerEndpoint("127.0.0.1", 0)
    with pytest.raises(EndpointClosed):
        ep.recv()
    with pytest.raises(EndpointClosed):
        ep.write(b"x")


def test_close_is_idempotent(endpoint):
    endpoint.close()
    endpoint.close()
    assert endpoint.is_open is False


def test_bind_clash_raises_rather_than_silently_sharing():
    """SO_REUSEADDR is deliberately not set, so a clash is reported.

    With it set -- as .NET's UdpClient and pymavlink's mavudp both do -- a second
    bind on Windows succeeds while datagrams reach only one socket. That is how
    the existing 14550 contention fails silently, and the whole point of not
    repeating it here.
    """
    first = UdpPeerEndpoint("127.0.0.1", 0)
    first.open()
    port = first._sock.getsockname()[1]
    try:
        clash = UdpPeerEndpoint("127.0.0.1", port)
        with pytest.raises(EndpointClosed, match="cannot bind"):
            clash.open()
    finally:
        first.close()


from .conftest import make_v1  # noqa: E402  (used by one test above)
