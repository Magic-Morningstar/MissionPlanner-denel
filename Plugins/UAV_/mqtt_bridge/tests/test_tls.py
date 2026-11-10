# mqtt_bridge/tests/test_tls.py
"""TLS and credential handling (tls.py, and its use in cli and mqtt_link).

What is pinned here is the safety posture, not just the plumbing:

- TLS on never degrades to plaintext, and certificate settings without TLS
  refuse to start rather than quietly connecting unencrypted.
- Verification of the broker certificate and hostname cannot be switched off.
- A CA file REPLACES the system trust store instead of adding to it.
- A password is never accepted on the command line and never logged.

The real handshake against a real broker is proved by mqtt_smoke, not here.
"""

import argparse
import logging
import ssl

import pytest

from mqtt_bridge import tls
from mqtt_bridge.bridge_config import build_config

# A throwaway self-signed CA certificate (public half only, no key anywhere),
# valid until 2126. Used to prove a CA file replaces the system store.
TEST_CA_PEM = """-----BEGIN CERTIFICATE-----
MIIBkTCCATegAwIBAgIUZ+0h8rbPSvD47b1WKFH+PsjAR8wwCgYIKoZIzj0EAwIw
HTEbMBkGA1UEAwwSZGVuZWwtbXF0dC10ZXN0LWNhMCAXDTI2MTAyODExNDY0MloY
DzIxMjYxMDA0MTE0NjQyWjAdMRswGQYDVQQDDBJkZW5lbC1tcXR0LXRlc3QtY2Ew
WTATBgcqhkjOPQIBBggqhkjOPQMBBwNCAASlZfQ7V/MFMHS7Lu9gpV6JjGKjxBkN
4JWm1abJ8TDqr3RlkH4/NSqZUlQIuFLmqlK0402YqggEs+SwBek5nXo8o1MwUTAd
BgNVHQ4EFgQUbbwGE05q8RLIDYgSdCn6NL9Js4EwHwYDVR0jBBgwFoAUbbwGE05q
8RLIDYgSdCn6NL9Js4EwDwYDVR0TAQH/BAUwAwEB/zAKBggqhkjOPQQDAgNIADBF
AiBmfarlPDy6Cn1jr+DJ3LjcwDfFXiGstKzTwMWKgMZ+nwIhAMyqIIpBuPWtVfHj
2r1I2JgOcJfPJdE6Y7UZoHRiTUGX
-----END CERTIFICATE-----
"""


def cfg(**overrides):
    return build_config(cli=overrides)


@pytest.fixture
def ca_file(tmp_path):
    path = tmp_path / "ca.pem"
    path.write_text(TEST_CA_PEM)
    return str(path)


# ---- the ssl context ------------------------------------------------------

def test_tls_off_gives_no_context():
    assert tls.build_ssl_context(cfg()) is None


def test_tls_on_always_verifies_certificate_and_hostname():
    ctx = tls.build_ssl_context(cfg(TLS_ENABLED=True, BROKER_PORT=8883))
    assert ctx.verify_mode == ssl.CERT_REQUIRED
    assert ctx.check_hostname is True
    assert ctx.minimum_version >= ssl.TLSVersion.TLSv1_2


def test_ca_file_replaces_the_system_store(ca_file):
    ctx = tls.build_ssl_context(cfg(TLS_ENABLED=True, TLS_CA_FILE=ca_file))
    # Exactly our one CA: nothing from the OS store was mixed in.
    assert ctx.cert_store_stats()["x509_ca"] == 1


def test_malformed_ca_file_raises_rather_than_degrading(tmp_path):
    bad = tmp_path / "bad.pem"
    bad.write_text("not a certificate")
    with pytest.raises((ssl.SSLError, OSError)):
        tls.build_ssl_context(cfg(TLS_ENABLED=True, TLS_CA_FILE=str(bad)))


# ---- problems: these stop a bridge from starting ------------------------

def test_valid_tls_settings_have_no_problems(ca_file):
    assert tls.tls_config_problems(cfg(TLS_ENABLED=True, TLS_CA_FILE=ca_file)) == []
    assert tls.tls_config_problems(cfg(TLS_ENABLED=True)) == []
    assert tls.tls_config_problems(cfg()) == []


def test_missing_ca_file_is_a_problem(tmp_path):
    problems = tls.tls_config_problems(
        cfg(TLS_ENABLED=True, TLS_CA_FILE=str(tmp_path / "nope.pem")))
    assert len(problems) == 1 and "TLS_CA_FILE" in problems[0]


def test_missing_client_cert_and_key_are_problems(tmp_path):
    problems = tls.tls_config_problems(cfg(
        TLS_ENABLED=True, TLS_CERT_FILE=str(tmp_path / "c.pem"),
        TLS_KEY_FILE=str(tmp_path / "k.pem")))
    assert any("TLS_CERT_FILE" in p for p in problems)
    assert any("TLS_KEY_FILE" in p for p in problems)


def test_key_without_cert_is_a_problem(tmp_path):
    key = tmp_path / "k.pem"
    key.write_text("x")
    problems = tls.tls_config_problems(cfg(TLS_ENABLED=True, TLS_KEY_FILE=str(key)))
    assert any("without TLS_CERT_FILE" in p for p in problems)


def test_certificate_settings_without_tls_refuse_plaintext(ca_file):
    """A forgotten --tls must not become a silent plaintext connection."""
    problems = tls.tls_config_problems(cfg(TLS_CA_FILE=ca_file))
    assert len(problems) == 1 and "plaintext" in problems[0]


# ---- warnings: logged, but the bridge still starts ----------------------

def test_tls_on_plaintext_port_warns():
    warnings = tls.tls_config_warnings(cfg(TLS_ENABLED=True, BROKER_PORT=1883))
    assert any("8883" in w for w in warnings)


def test_tls_port_without_tls_warns():
    warnings = tls.tls_config_warnings(cfg(BROKER_PORT=8883))
    assert any("--tls" in w for w in warnings)


@pytest.mark.parametrize("host", ["10.190.190.12", "abc.s1.eu.hivemq.cloud"])
def test_plaintext_password_to_remote_host_warns(host):
    warnings = tls.tls_config_warnings(cfg(
        BROKER_HOST=host, MQTT_USERNAME="u", MQTT_PASSWORD="p"))
    assert any("PLAINTEXT" in w for w in warnings)


@pytest.mark.parametrize("host", ["localhost", "127.0.0.1", "::1", "[::1]"])
def test_plaintext_password_to_loopback_is_fine(host):
    warnings = tls.tls_config_warnings(cfg(
        BROKER_HOST=host, MQTT_USERNAME="u", MQTT_PASSWORD="p"))
    assert not any("PLAINTEXT" in w for w in warnings)


def test_tls_password_to_remote_host_is_fine():
    warnings = tls.tls_config_warnings(cfg(
        BROKER_HOST="abc.s1.eu.hivemq.cloud", BROKER_PORT=8883,
        TLS_ENABLED=True, MQTT_USERNAME="u", MQTT_PASSWORD="p"))
    assert warnings == []


def test_password_without_username_warns():
    warnings = tls.tls_config_warnings(cfg(MQTT_PASSWORD="p"))
    assert any("MQTT_USERNAME is empty" in w for w in warnings)


# ---- never leak the password --------------------------------------------

def test_describe_transport_never_contains_the_password():
    secret = "hunter2-very-secret"
    for tls_on in (False, True):
        line = tls.describe_transport(cfg(
            TLS_ENABLED=tls_on, MQTT_USERNAME="u", MQTT_PASSWORD=secret))
        assert secret not in line
        assert "password set" in line


def test_describe_transport_says_plaintext_when_it_is():
    assert "PLAINTEXT" in tls.describe_transport(cfg())
    assert "TLS" in tls.describe_transport(cfg(TLS_ENABLED=True))


# ---- the command line ---------------------------------------------------

def _common_parser():
    from mqtt_bridge import cli
    return cli.add_common_arguments(argparse.ArgumentParser())


def test_cli_flags_map_onto_config_keys(ca_file):
    args = _common_parser().parse_args([
        "--tls", "--tls-ca-file", ca_file, "--username", "xblink-test",
        "--broker-port", "8883"])
    overrides = {k: v for k, v in vars(args).items() if v is not None}
    c = build_config(cli=overrides)
    assert c.TLS_ENABLED is True
    assert c.TLS_CA_FILE == ca_file
    assert c.MQTT_USERNAME == "xblink-test"
    assert c.BROKER_PORT == 8883


def test_tls_flag_absent_is_not_an_override():
    args = _common_parser().parse_args([])
    assert args.TLS_ENABLED is None


def test_there_is_no_password_flag():
    """Command lines are visible to every user and land in shell history."""
    with pytest.raises(SystemExit):
        _common_parser().parse_args(["--password", "x"])


# ---- MqttLink wiring (needs paho) ---------------------------------------

def _link(c):
    pytest.importorskip("paho.mqtt.client")
    from mqtt_bridge.mqtt_link import MqttLink
    return MqttLink(c, "test-client", [], lambda *a: None)


def test_mqtt_link_applies_tls_only_when_enabled():
    # paho 2.1.0 records tls_set_context() in Client._ssl (pinned version).
    assert _link(cfg()).client._ssl is False
    assert _link(cfg(TLS_ENABLED=True, BROKER_PORT=8883)).client._ssl is True


def test_connect_failures_are_counted_and_logged_sparingly(caplog):
    link = _link(cfg(TLS_ENABLED=True, BROKER_PORT=8883))
    with caplog.at_level(logging.WARNING, logger="mqtt_bridge.mqtt_link"):
        for _ in range(25):
            link._cb_connect_fail(link.client, None)
    lines = [r for r in caplog.records if "could not reach" in r.getMessage()]
    assert len(lines) == 3                       # attempts 1, 10 and 20
    assert "certificate" in lines[0].getMessage()
    assert link.counters()["mqtt_connect_failures"] == 25


# ---- broker Receive Maximum (found on the first desk test via Azure) -----

class _Props:
    def __init__(self, receive_maximum=None):
        if receive_maximum is not None:
            self.ReceiveMaximum = receive_maximum


def test_default_inflight_fits_hivemq_receive_maximum():
    assert 0 < cfg().MAX_INFLIGHT_MESSAGES <= 10


def test_receive_maximum_check(caplog):
    link = _link(cfg())
    assert link._check_receive_maximum(None) is True
    assert link._check_receive_maximum(_Props()) is True
    assert link._check_receive_maximum(_Props(10)) is True
    with caplog.at_level(logging.ERROR, logger="mqtt_bridge.mqtt_link"):
        assert _link(cfg(MAX_INFLIGHT_MESSAGES=20))._check_receive_maximum(_Props(10)) is False
    assert any("Receive Maximum" in r.getMessage() for r in caplog.records)
