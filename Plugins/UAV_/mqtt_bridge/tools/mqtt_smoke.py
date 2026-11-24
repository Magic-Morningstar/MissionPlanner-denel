"""Phase 0 check: can this machine actually talk MQTT 5 to the broker?

A TCP port check is not enough — it proves a socket opens, not that an MQTT 5
CONNECT is accepted, that the broker's current security extension lets us
publish and subscribe, or that binary payloads survive the hop. This does all
four, using the same paho configuration the real bridges use.

Run it from Windows AND from WSL before building anything on top:

    python -m mqtt_bridge.tools.mqtt_smoke --host localhost
    python -m mqtt_bridge.tools.mqtt_smoke --host 172.21.32.1   # WSL -> Windows host

Against a TLS broker with credentials (HiveMQ Cloud, for example). The port
defaults to 8883 with --tls. The password is read from DENEL_MQTT_MQTT_PASSWORD
or prompted for -- never passed on the command line:

    python -m mqtt_bridge.tools.mqtt_smoke --host <cluster>.hivemq.cloud --tls --username xblink-test

Unlike the bridges, this connects synchronously, so a TLS or certificate
failure is printed with its exact reason. It is the tool to reach for when a
bridge logs "MQTT could not reach ...".

Exit code 0 = the broker is usable from here. Anything else is a hard stop.
"""

import argparse
import getpass
import os
import socket
import ssl
import sys
import time
import uuid
from types import SimpleNamespace

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.properties import Properties

from mqtt_bridge.tls import (
    PLAIN_DEFAULT_PORT, TLS_DEFAULT_PORT, build_ssl_context, describe_transport,
    tls_config_problems, tls_config_warnings,
)

PASSWORD_ENV = "DENEL_MQTT_MQTT_PASSWORD"

# A payload chosen to catch the things text payloads hide: a NUL, a 0xFF, the
# MAVLink v1/v2 start bytes, and a byte sequence that is not valid UTF-8. If
# anything in the path tries to treat payloads as text, this will not survive.
PROBE_PAYLOAD = bytes([0x00, 0xFF, 0xFD, 0xFE, 0x55, 0xAA]) + b"\x80\x81\x82" + os.urandom(8)


def main(argv=None):
    ap = argparse.ArgumentParser(description="MQTT 5 round-trip smoke test.")
    ap.add_argument("--host", default="localhost", help="broker host (default: localhost)")
    ap.add_argument("--port", type=int, default=None,
                    help=f"broker port (default: {PLAIN_DEFAULT_PORT}, or "
                         f"{TLS_DEFAULT_PORT} with --tls)")
    ap.add_argument("--tls", action="store_true",
                    help="connect with TLS; the server certificate and hostname "
                         "are always verified")
    ap.add_argument("--ca-file", default="",
                    help="CA certificate (PEM) to verify the broker against "
                         "(default: the system trust store)")
    ap.add_argument("--cert-file", default="", help="client certificate (PEM), mutual TLS")
    ap.add_argument("--key-file", default="", help="client private key (PEM), mutual TLS")
    ap.add_argument("--username", default="",
                    help=f"broker username. The password comes from {PASSWORD_ENV} "
                         "or an interactive prompt")
    ap.add_argument("--topic", default="denel/uav/_smoke/probe", help="topic to round-trip on")
    ap.add_argument("--qos", type=int, default=1, choices=(0, 1, 2))
    ap.add_argument("--timeout", type=float, default=10.0, help="seconds to wait (default: 10)")
    args = ap.parse_args(argv)

    port = args.port or (TLS_DEFAULT_PORT if args.tls else PLAIN_DEFAULT_PORT)
    password = os.environ.get(PASSWORD_ENV, "")
    if args.username and not password and sys.stdin.isatty():
        password = getpass.getpass(f"password for {args.username!r}: ")

    # Same shape as BridgeConfig, so the smoke test applies exactly the rules
    # the bridges do.
    cfg = SimpleNamespace(
        BROKER_HOST=args.host, BROKER_PORT=port, TLS_ENABLED=args.tls,
        TLS_CA_FILE=args.ca_file, TLS_CERT_FILE=args.cert_file,
        TLS_KEY_FILE=args.key_file, MQTT_USERNAME=args.username,
        MQTT_PASSWORD=password,
    )
    problems = tls_config_problems(cfg)
    if problems:
        for problem in problems:
            print(f"FAIL  {problem}")
        return 6
    for warning in tls_config_warnings(cfg):
        print(f"WARN  {warning}")

    # Unique, because a duplicate client ID makes the broker kick the earlier
    # session — which looks exactly like a broker fault if two people run this
    # at once.
    client_id = f"smoke-{socket.gethostname()}-{uuid.uuid4().hex[:8]}"

    received = []
    connect_result = {}

    client = mqtt.Client(CallbackAPIVersion.VERSION2, client_id=client_id, protocol=mqtt.MQTTv5)
    if args.username:
        client.username_pw_set(args.username, password or None)
    try:
        ssl_context = build_ssl_context(cfg)
    except (ssl.SSLError, OSError) as exc:
        print(f"FAIL  could not load the TLS files: {type(exc).__name__}: {exc}")
        return 6
    if ssl_context is not None:
        client.tls_set_context(ssl_context)

    def on_connect(c, userdata, connect_flags, reason_code, properties):
        connect_result["reason_code"] = reason_code
        if reason_code.is_failure:
            return
        c.subscribe(args.topic, qos=args.qos)

    def on_subscribe(c, userdata, mid, reason_codes, properties):
        # Publish only once the SUBACK is in, otherwise the message can be
        # routed before our subscription exists and the test fails spuriously.
        props = Properties(PacketTypes.PUBLISH)
        props.MessageExpiryInterval = 10
        c.publish(args.topic, PROBE_PAYLOAD, qos=args.qos, properties=props)

    def on_message(c, userdata, message):
        received.append(message)

    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_message = on_message

    print(f"connecting to {args.host}:{port} as {client_id} (MQTT 5, QoS {args.qos})")
    print(f"      over {describe_transport(cfg)}")
    try:
        client.connect(args.host, port, keepalive=20, clean_start=True)
    except ssl.SSLCertVerificationError as exc:
        print(f"FAIL  the broker's certificate was rejected: {exc.verify_message}")
        print("      Check that --host is exactly the name on the certificate (the")
        print("      full cluster URL, not an IP), that the clock is right (run")
        print("      `date`; a Pi has no RTC), and --ca-file if the broker uses a")
        print("      private CA.")
        return 2
    except ssl.SSLError as exc:
        print(f"FAIL  TLS handshake failed: {exc}")
        print(f"      Is {port} really the broker's TLS port? A plaintext listener")
        print("      (usually 1883) cannot complete a TLS handshake.")
        return 2
    except Exception as exc:
        print(f"FAIL  could not connect: {type(exc).__name__}: {exc}")
        return 2
    client.loop_start()

    deadline = time.monotonic() + args.timeout
    while not received and time.monotonic() < deadline:
        rc = connect_result.get("reason_code")
        if rc is not None and rc.is_failure:
            print(f"FAIL  broker refused the connection: {rc}")
            if args.username or "auth" in str(rc).lower() or "password" in str(rc).lower():
                print("      Check the username, the password, and that the credential")
                print("      has permission to publish and subscribe.")
            client.loop_stop()
            return 3
        time.sleep(0.05)

    client.loop_stop()
    client.disconnect()

    if not received:
        print(f"FAIL  no message came back within {args.timeout}s "
              f"(connected={connect_result.get('reason_code')})")
        if connect_result.get("reason_code") is None:
            print("      The broker never answered the CONNECT (no CONNACK): the")
            print("      network or TLS layer worked, but nothing MQTT replied. Check")
            print("      the host and port point at an MQTT listener.")
        else:
            print("      Connected but nothing routed usually means the broker's security")
            print("      extension is denying publish or subscribe on this topic.")
        return 4

    msg = received[0]
    if msg.payload != PROBE_PAYLOAD:
        print(f"FAIL  payload corrupted: sent {PROBE_PAYLOAD.hex()}, got {msg.payload.hex()}")
        return 5

    print(f"PASS  {len(msg.payload)} bytes round-tripped on {msg.topic} byte-for-byte")
    return 0


if __name__ == "__main__":
    sys.exit(main())
