# mqtt_bridge/tests/test_health.py
"""The aircraft health report: warning logic, mmcli parsing, the summary line.

The probes themselves read /sys, /proc and run ip/mmcli/timedatectl; those are
exercised on the real unit. What is pinned here is what the report SAYS.
"""

import json
import os

import pytest

from mqtt_bridge.health import (
    HOT_CPU_C, WEAK_SIGNAL_PCT, build_health, parse_mmcli_keyvalue, probe_system,
)

MMCLI_SAMPLE = """modem.generic.state                              : connected
modem.generic.signal-quality.value               : 61
modem.generic.signal-quality.recent              : yes
modem.generic.access-technologies.length         : 1
modem.generic.access-technologies.value[1]       : lte
modem.3gpp.operator-name                         : Vodacom
modem.3gpp.registration-state                    : home
"""

GOOD_LINK = {"mqtt_up": True, "fc_open": True, "fc_frames_per_s": 64.5,
             "cmd_frames_per_s": 1.2, "wakeup_repairs": 0}


def system(ipv4="10.1.2.3", signal=61, synced=True, temp=50.0):
    return {"host": "xblink", "uptime_s": 100, "cpu_temp_c": temp,
            "clock_synced": synced,
            "lte": {"iface": "wwan0", "ipv4": ipv4,
                    "modem": {"signal_pct": signal, "tech": "lte"}}}


def test_parse_mmcli():
    m = parse_mmcli_keyvalue(MMCLI_SAMPLE)
    assert m == {"state": "connected", "signal_pct": 61, "tech": "lte",
                 "operator": "Vodacom", "registration": "home"}


def test_parse_mmcli_tolerates_missing_and_placeholder_values():
    m = parse_mmcli_keyvalue("modem.generic.signal-quality.value : --\n")
    assert m["signal_pct"] is None and m["state"] is None
    assert parse_mmcli_keyvalue(None)["signal_pct"] is None


def test_healthy_report_has_no_warnings():
    r = build_health(system(), dict(GOOD_LINK), now=1000)
    assert r["ok"] is True and r["warnings"] == []
    assert r["ts"] == 1000 and r["link"]["fc_frames_per_s"] == 64.5
    json.dumps(r)                                    # must be publishable


def test_connected_but_no_cube_data_is_flagged():
    """The 2026-11-12 failure: online at the broker, nothing from the Cube."""
    r = build_health(system(), dict(GOOD_LINK, fc_frames_per_s=0.0))
    assert "no_cube_data" in r["warnings"] and r["ok"] is False


def test_closed_fc_port_is_flagged_instead_of_no_data():
    r = build_health(system(), dict(GOOD_LINK, fc_open=False, fc_frames_per_s=0))
    assert "fc_port_closed" in r["warnings"]
    assert "no_cube_data" not in r["warnings"]


@pytest.mark.parametrize("kwargs,code", [
    ({"signal": WEAK_SIGNAL_PCT - 1}, "weak_signal"),
    ({"synced": False}, "clock_not_synced"),
    ({"temp": HOT_CPU_C}, "cpu_hot"),
])
def test_system_warnings(kwargs, code):
    assert code in build_health(system(**kwargs), dict(GOOD_LINK))["warnings"]


@pytest.mark.skipif(os.name == "nt", reason="LTE address check is Linux-only")
def test_missing_lte_address_is_flagged():
    assert "no_lte_address" in build_health(system(ipv4=None), dict(GOOD_LINK))["warnings"]


def test_unknown_values_do_not_warn():
    r = build_health(system(signal=None, synced=None, temp=None), dict(GOOD_LINK))
    assert r["warnings"] == []


def test_wakeup_repair_is_reported():
    r = build_health(system(), dict(GOOD_LINK, wakeup_repairs=2))
    assert "mqtt_wakeup_repaired" in r["warnings"]


def test_probe_system_never_raises():
    s = probe_system("no-such-iface", read_modem=False)
    assert s["lte"]["ipv4"] is None and s["lte"]["modem"] is None


def test_status_watch_summary_line():
    from mqtt_bridge.tools.status_watch import summarise_health
    line = summarise_health(json.dumps(build_health(system(), dict(GOOD_LINK))))
    assert line.startswith("health: OK") and "signal 61% lte" in line
    bad = summarise_health(json.dumps(build_health(system(), dict(GOOD_LINK, fc_frames_per_s=0))))
    assert "WARN no_cube_data" in bad
    assert summarise_health(b"not json") == "health: (not JSON)"


def test_aircraft_link_fields_from_snapshots():
    pytest.importorskip("paho.mqtt.client")
    from mqtt_bridge.aircraft_bridge import AircraftBridge
    from mqtt_bridge.bridge_config import build_config
    b = AircraftBridge(build_config(cli={"AIRCRAFT_MAVLINK": "udpin:127.0.0.1:0"}))
    prev = {"fc_frames": 100, "cmd_frames": 10, "wakeup_repairs": 1}
    cur = {"fc_frames": 1100, "cmd_frames": 40, "wakeup_repairs": 1}
    f = b.health_link_fields(prev, cur, 10.0)
    assert f["fc_frames_per_s"] == 100.0 and f["cmd_frames_per_s"] == 3.0
    assert f["wakeup_repairs"] == 0 and f["fc_open"] is False


def test_health_switches_itself_off_when_the_broker_refuses_it(caplog):
    """An unauthorised health publish gets the client kicked; stop after one."""
    pytest.importorskip("paho.mqtt.client")
    import logging
    import threading
    import time as _t
    from mqtt_bridge.aircraft_bridge import AircraftBridge
    from mqtt_bridge.bridge_config import build_config
    b = AircraftBridge(build_config(cli={"AIRCRAFT_MAVLINK": "udpin:127.0.0.1:0",
                                         "HEALTH_INTERVAL": 1}))
    calls = []

    class Stub:
        connected = True
        wakeup_repairs = 0
        disconnects = 0
        last_disconnect_reason = None
        last_disconnect_at = None

        def publish_status(self, *a, **k):
            calls.append(1)
            self.last_disconnect_reason = "Not authorized"
            self.last_disconnect_at = _t.monotonic()
            return True

    b.mqtt = Stub()
    with caplog.at_level(logging.ERROR, logger="mqtt_bridge.aircraft_bridge"):
        t = threading.Thread(target=b._health_loop, daemon=True)
        t.start()
        t.join(timeout=8)
    b._stop.set()
    assert not t.is_alive(), "health loop should have stopped itself"
    assert len(calls) == 1
    assert any("Health reporting is OFF" in r.getMessage() for r in caplog.records)
