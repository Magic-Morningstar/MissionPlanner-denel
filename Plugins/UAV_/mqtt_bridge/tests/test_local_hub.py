# mqtt_bridge/tests/test_local_hub.py
"""LocalHub: fan telemetry to every local consumer, merge their commands."""

import socket

import pytest

from mqtt_bridge.local_hub import LocalHub, UdpSendEndpoint, parse_peer_specs
from mqtt_bridge.mavlink_endpoint import EndpointClosed

from .conftest import make_v1, make_v2


# --------------------------------------------------------------------------
# Spec parsing (pure)
# --------------------------------------------------------------------------

def test_bind_port_zero_is_allowed():
    """'any free port' is a real request for a listener, unlike for a target."""
    assert parse_peer_specs("bind:127.0.0.1:0") == [("bind", "127.0.0.1", 0)]


def test_parse_both_kinds():
    assert parse_peer_specs("bind:127.0.0.1:14570,send:127.0.0.1:14551") == [
        ("bind", "127.0.0.1", 14570),
        ("send", "127.0.0.1", 14551),
    ]


def test_parse_tolerates_whitespace_and_blanks():
    assert parse_peer_specs(" bind:127.0.0.1:14570 , , send:10.0.0.5:14551 ") == [
        ("bind", "127.0.0.1", 14570),
        ("send", "10.0.0.5", 14551),
    ]


@pytest.mark.parametrize("bad", [
    "",                       # nothing configured
    "127.0.0.1:14570",        # missing kind
    "listen:127.0.0.1:14570",  # unknown kind
    "bind:127.0.0.1",         # missing port
    "bind:127.0.0.1:notaport",
    "send:127.0.0.1:0",       # no such destination
    "bind:127.0.0.1:70000",
])
def test_parse_rejects_bad_specs(bad):
    with pytest.raises(ValueError):
        parse_peer_specs(bad)


# --------------------------------------------------------------------------
# Fan-out behaviour
# --------------------------------------------------------------------------

@pytest.fixture
def panel():
    """Stands in for main.py: a udpin server that only replies to who it heard."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    s.settimeout(2.0)
    yield s
    s.close()


@pytest.fixture
def hub(panel):
    """A hub with one bind peer (Mission Planner) and one send peer (the panel)."""
    panel_port = panel.getsockname()[1]
    h = LocalHub(parse_peer_specs(f"bind:127.0.0.1:0,send:127.0.0.1:{panel_port}"))
    h.open()
    yield h
    h.close()


def _bind_peer_port(hub):
    return hub.peers[0]._sock.getsockname()[1]


def test_telemetry_reaches_both_consumers(hub, panel):
    """The point of the whole exercise: one frame in, every consumer gets it."""
    mp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    mp.bind(("127.0.0.1", 0))
    mp.settimeout(2.0)
    try:
        # Mission Planner dials in so the bind peer learns its address.
        mp.sendto(b"\x00", ("127.0.0.1", _bind_peer_port(hub)))
        assert hub.select(2.0)
        hub.recv()

        frame = make_v2(payload_len=9)
        assert hub.write(frame) is True
        assert mp.recv(4096) == frame        # the screen got it
        assert panel.recv(4096) == frame     # and so did the panel
    finally:
        mp.close()


def test_panel_gets_telemetry_before_mission_planner_connects(hub, panel):
    """The panel must not wait on Mission Planner.

    main.py blocks in wait_heartbeat() until telemetry arrives, so if the hub
    withheld writes until every peer was known, the panel would never start.
    """
    assert hub.ready_to_write is True        # true even with no MP peer yet
    frame = make_v2(payload_len=9)
    assert hub.write(frame) is True
    assert panel.recv(4096) == frame


def test_commands_from_either_consumer_are_received(hub, panel):
    """Commands merge: whichever consumer sends, the bridge reads it."""
    mp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    mp.bind(("127.0.0.1", 0))
    mp.settimeout(2.0)
    try:
        # The panel can only reply to an address it has heard from, which is
        # exactly why telemetry has to go out on the same socket first.
        hub.write(make_v2(payload_len=4))
        _, hub_addr = panel.recvfrom(4096)

        from_panel = make_v1(payload_len=6, sysid=255, compid=0)
        from_mp = make_v2(payload_len=7, sysid=255, compid=190)
        panel.sendto(from_panel, hub_addr)
        mp.sendto(from_mp, ("127.0.0.1", _bind_peer_port(hub)))

        seen = []
        for _ in range(8):
            if hub.select(2.0):
                data = hub.recv()
                if data:
                    seen.append(data)
            if len(seen) >= 2:
                break
        assert from_panel in seen, "command from the STM32 panel was not received"
        assert from_mp in seen, "command from Mission Planner was not received"
    finally:
        mp.close()


def test_one_chatty_peer_does_not_starve_the_other(hub, panel):
    """Round-robin: a flood from one consumer must not block the other."""
    mp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    mp.bind(("127.0.0.1", 0))
    try:
        hub.write(make_v2(payload_len=4))
        _, hub_addr = panel.recvfrom(4096)

        for _ in range(20):
            panel.sendto(make_v1(payload_len=2), hub_addr)
        quiet = make_v2(payload_len=11)
        mp.sendto(quiet, ("127.0.0.1", _bind_peer_port(hub)))

        seen = []
        for _ in range(40):
            if hub.select(0.5):
                data = hub.recv()
                if data:
                    seen.append(data)
            if quiet in seen:
                break
        assert quiet in seen, "the quiet peer was starved by the chatty one"
    finally:
        mp.close()


def test_missing_panel_is_not_fatal():
    """An absent optional consumer must not take the aircraft link down.

    main.py is frequently not running. The bind peer is different: if Mission
    Planner's port cannot be bound, that is fatal and reported.
    """
    h = LocalHub(parse_peer_specs("bind:127.0.0.1:0,send:127.0.0.1:9"))
    try:
        assert h.open() is True
        assert h.is_open is True
        assert h.write(make_v2(payload_len=4)) is True
    finally:
        h.close()


def test_bind_clash_is_fatal():
    """A port clash must raise, not silently half-work."""
    first = LocalHub(parse_peer_specs("bind:127.0.0.1:0"))
    first.open()
    port = first.peers[0]._sock.getsockname()[1]
    try:
        clash = LocalHub(parse_peer_specs(f"bind:127.0.0.1:{port}"))
        with pytest.raises(EndpointClosed):
            clash.open()
    finally:
        first.close()


def test_hub_presents_the_endpoint_interface():
    """BridgeBase drives the hub exactly as it drives a single endpoint."""
    h = LocalHub(parse_peer_specs("bind:127.0.0.1:0"))
    for attr in ("open", "select", "recv", "write", "close"):
        assert callable(getattr(h, attr))
    for attr in ("is_open", "is_datagram", "ready_to_write", "peer"):
        assert hasattr(h, attr)
    assert h.is_datagram is True


def test_send_endpoint_binds_loopback_not_wildcard():
    """The peer replies to our source address, so it must be a real one.

    Letting the OS auto-bind on first send can yield a 0.0.0.0 source, and the
    reply path then may not come back.
    """
    ep = UdpSendEndpoint("127.0.0.1", 9, name="t")
    ep.open()
    try:
        assert ep._sock.getsockname()[0] == "127.0.0.1"
    finally:
        ep.close()


def test_closed_hub_raises_on_select():
    h = LocalHub(parse_peer_specs("bind:127.0.0.1:0"))
    with pytest.raises(EndpointClosed):
        h.select(0.1)


def test_unreachable_warning_is_retracted_when_the_peer_appears(caplog):
    """The bridge normally starts before main.py, so the warning is expected.

    Leaving it unretracted makes a healthy log look faulty: "check that main.py
    is running" stays on screen long after main.py came up.
    """
    import logging as _logging

    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(("127.0.0.1", 0))
    port = listener.getsockname()[1]
    ep = UdpSendEndpoint("127.0.0.1", port, name="panel")
    ep.open()
    try:
        ep._warned_unreachable = True            # as if it had warned at startup
        listener.sendto(b"\xfd\x00", ep._sock.getsockname())
        for _ in range(20):
            if ep.recv():
                break
        with caplog.at_level(_logging.INFO):
            ep._warned_unreachable = True
            listener.sendto(b"\xfd\x00", ep._sock.getsockname())
            for _ in range(20):
                if ep.recv():
                    break
        assert ep._warned_unreachable is False
        assert "responding now" in caplog.text
    finally:
        ep.close()
        listener.close()
