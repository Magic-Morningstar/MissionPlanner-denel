# mqtt_bridge/tls.py
"""TLS and broker-credential settings: validation, warnings and the ssl context.

Standard library only (ssl, ipaddress, pathlib), so like bridge_config it imports
and unit-tests without paho or pymavlink installed.

Each rule below exists because the failure it prevents is silent or misleading
if left to paho:

1. TLS on NEVER falls back to plaintext. A missing CA, certificate or key file
   is a startup error, not a warning. Equally, certificate files configured
   while TLS is off are an error: that is almost certainly a forgotten --tls,
   and quietly connecting in plaintext would leave the operator believing the
   link is encrypted.

2. Server certificate AND hostname verification are always on. There is
   deliberately no "insecure" switch. A test that only passes with
   verification off has not tested TLS.

3. TLS 1.2 is the floor.

4. TLS_CA_FILE empty means "use the operating system's trust store", which is
   what a publicly-trusted broker such as HiveMQ Cloud needs. When TLS_CA_FILE
   is set it REPLACES the system store rather than adding to it, so a private
   broker CA cannot be satisfied by some unrelated public certificate.

5. A password sent without TLS to anything other than loopback is warned about
   at startup, every time.

Clock note for the aircraft: certificate validity is checked against the local
clock. A Raspberry Pi has no RTC and gets the time from NTP, so a Pi that boots
without network time fails every TLS handshake with "certificate is not yet
valid". Check `date` before suspecting the broker.
"""

import ipaddress
import ssl
from pathlib import Path

PLAIN_DEFAULT_PORT = 1883
TLS_DEFAULT_PORT = 8883

_FILE_KEYS = ("TLS_CA_FILE", "TLS_CERT_FILE", "TLS_KEY_FILE")


def is_loopback(host):
    """True for localhost, 127.0.0.0/8 and ::1. Anything unparseable is remote."""
    host = str(host).strip().strip("[]").lower()
    if host == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def tls_config_problems(cfg):
    """Settings that must stop a bridge from starting. Returns a list of str.

    Touches the filesystem (Path.is_file) but nothing else, so it can run in
    cli.bootstrap before any network activity.
    """
    problems = []

    if not cfg.TLS_ENABLED:
        stray = [key for key in _FILE_KEYS if getattr(cfg, key)]
        if stray:
            problems.append(
                f"{', '.join(stray)} set but TLS_ENABLED is off. Refusing to "
                "connect in plaintext while certificate settings are present -- "
                "add --tls (or DENEL_MQTT_TLS_ENABLED=true), or remove them.")
        return problems

    for key in _FILE_KEYS:
        path = getattr(cfg, key)
        if path and not Path(path).is_file():
            problems.append(f"{key}={path!r} does not exist or is not a file")

    if cfg.TLS_KEY_FILE and not cfg.TLS_CERT_FILE:
        problems.append("TLS_KEY_FILE is set without TLS_CERT_FILE; a client key "
                        "is only used together with a client certificate")

    return problems


def tls_config_warnings(cfg):
    """Settings that are probably wrong but not dangerous enough to refuse."""
    warnings = []

    if cfg.TLS_ENABLED and cfg.BROKER_PORT == PLAIN_DEFAULT_PORT:
        warnings.append(
            f"TLS is on but BROKER_PORT is {PLAIN_DEFAULT_PORT}, the usual "
            f"PLAINTEXT port. TLS brokers normally listen on {TLS_DEFAULT_PORT} "
            "(HiveMQ Cloud does); against a plaintext listener the handshake "
            "fails or hangs.")

    if not cfg.TLS_ENABLED and cfg.BROKER_PORT == TLS_DEFAULT_PORT:
        warnings.append(
            f"BROKER_PORT is {TLS_DEFAULT_PORT}, the usual TLS port, but TLS is "
            "off. Add --tls.")

    if cfg.MQTT_PASSWORD and not cfg.MQTT_USERNAME:
        warnings.append("MQTT_PASSWORD is set but MQTT_USERNAME is empty, so "
                        "no credentials will be sent at all.")

    if (cfg.MQTT_PASSWORD and cfg.MQTT_USERNAME and not cfg.TLS_ENABLED
            and not is_loopback(cfg.BROKER_HOST)):
        warnings.append(
            f"a password is configured and TLS is off: it will cross the "
            f"network to {cfg.BROKER_HOST} in PLAINTEXT, readable by anyone on "
            "the path. Use --tls for any broker that is not on this machine.")

    return warnings


def build_ssl_context(cfg):
    """An ssl.SSLContext for paho's tls_set_context(), or None when TLS is off.

    Raises (ssl.SSLError, OSError) if a configured file is unreadable or
    malformed. Callers let that propagate: not starting is better than
    starting unencrypted.
    """
    if not cfg.TLS_ENABLED:
        return None

    ctx = ssl.create_default_context(ssl.Purpose.SERVER_AUTH,
                                     cafile=cfg.TLS_CA_FILE or None)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    # create_default_context already sets both of these. They are asserted
    # here rather than assumed, so a future edit cannot quietly relax them.
    ctx.check_hostname = True
    ctx.verify_mode = ssl.CERT_REQUIRED

    if cfg.TLS_CERT_FILE:
        # Mutual TLS: the broker verifies us as well. keyfile None means the
        # private key is inside the certificate file.
        ctx.load_cert_chain(certfile=cfg.TLS_CERT_FILE,
                            keyfile=cfg.TLS_KEY_FILE or None)
    return ctx


def describe_transport(cfg):
    """One human-readable line for the startup log. Never includes a secret."""
    if not cfg.TLS_ENABLED:
        transport = "PLAINTEXT (no TLS)"
    else:
        trust = cfg.TLS_CA_FILE or "the system trust store"
        client = (f"client certificate {cfg.TLS_CERT_FILE}" if cfg.TLS_CERT_FILE
                  else "no client certificate")
        transport = f"TLS 1.2+, server verified against {trust}, {client}"
    if cfg.MQTT_USERNAME:
        auth = f"username {cfg.MQTT_USERNAME!r}" + (
            ", password set" if cfg.MQTT_PASSWORD else ", NO password")
    else:
        auth = "no credentials"
    return f"{transport}; {auth}"
