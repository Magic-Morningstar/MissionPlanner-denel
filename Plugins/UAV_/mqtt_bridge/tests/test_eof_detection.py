# mqtt_bridge/tests/test_eof_detection.py
"""A stream endpoint that hits EOF must be noticed and reconnected.

Regression test for a bug Phase 2 acceptance 2 exposed: when the flight
controller died, the TCP socket reached EOF, and because select() reports a
closed socket as readable forever while pymavlink's recv() returns b"" rather
than raising, the reader loop span at full speed and never noticed. The bridge
sat there looking healthy with a dead link, and did not reconnect when the FC
came back.

The distinction that matters: empty-on-a-datagram-socket is normal and must NOT
trigger a reconnect; empty-on-a-stream-socket after readable is EOF.
"""

import threading
import time

import pytest

from mqtt_bridge import bridge_core
from mqtt_bridge.bridge_config import build_config
from mqtt_bridge.topics import TopicSet


class _FakeEndpoint:
    """Always readable, never returns data -- a socket at EOF."""

    def __init__(self, datagram):
        self.is_datagram = datagram
        self.is_open = True
        self.ready_to_write = True
        self.reopen_calls = 0
        self.is_passive = False

    def open(self):
        return True

    def select(self, timeout):
        time.sleep(0.01)      # keep the spin from pegging a core during the test
        return True           # readable, exactly as a closed socket reports

    def recv(self, n=4096):
        return b""            # ...but with nothing to read

    def write(self, data):
        return True

    def close(self):
        self.is_open = False

    def reopen(self):
        self.reopen_calls += 1
        return True


class _FakeMqtt:
    connected = True

    def __init__(self):
        self.published = []

    def start(self):
        pass

    def stop(self):
        pass

    def publish_frame(self, topic, payload, qos, expiry_s=None):
        self.published.append(payload)
        return True

    def counters(self):
        return {}


def _bridge(endpoint, monkeypatch):
    monkeypatch.setattr(bridge_core, "_EOF_GRACE", 0.2)   # keep the test quick
    cfg = build_config(cli={"METRICS_INTERVAL": 0})
    topics = TopicSet("uav01")
    return bridge_core.BridgeBase(
        cfg=cfg, topics=topics, endpoint=endpoint,
        mqtt_link_factory=lambda subscriptions, on_frame: _FakeMqtt(),
        publish_topic=topics.from_vehicle, publish_qos=0, publish_expiry=None,
        subscribe_topic=topics.to_vehicle, subscribe_qos=1,
        inbound_queue_size=16,
    )


def test_stream_endpoint_at_eof_triggers_reconnect(monkeypatch):
    ep = _FakeEndpoint(datagram=False)
    bridge = _bridge(ep, monkeypatch)
    t = threading.Thread(target=bridge._reader_loop, daemon=True)
    t.start()
    try:
        deadline = time.monotonic() + 5
        while ep.reopen_calls == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        assert ep.reopen_calls >= 1, "EOF on a stream endpoint was not detected"
    finally:
        bridge._stop.set()
        t.join(timeout=3)


def test_datagram_endpoint_does_not_reconnect_on_empty_reads(monkeypatch):
    """A quiet UDP socket is not a broken one.

    Reconnecting here would flap a perfectly healthy bound socket every time the
    peer went briefly quiet.
    """
    ep = _FakeEndpoint(datagram=True)
    bridge = _bridge(ep, monkeypatch)
    t = threading.Thread(target=bridge._reader_loop, daemon=True)
    t.start()
    try:
        time.sleep(2.0)       # ten times the shortened EOF grace
        assert ep.reopen_calls == 0, "a quiet datagram endpoint was reconnected"
    finally:
        bridge._stop.set()
        t.join(timeout=3)
