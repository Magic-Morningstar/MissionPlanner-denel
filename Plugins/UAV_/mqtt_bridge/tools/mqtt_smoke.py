"""Phase 0 check: can this machine actually talk MQTT 5 to the broker?

A TCP port check is not enough — it proves a socket opens, not that an MQTT 5
CONNECT is accepted, that the broker's current security extension lets us
publish and subscribe, or that binary payloads survive the hop. This does all
four, using the same paho configuration the real bridges use.

Run it from Windows AND from WSL before building anything on top:

    python -m mqtt_bridge.tools.mqtt_smoke --host localhost
    python -m mqtt_bridge.tools.mqtt_smoke --host 172.21.32.1   # WSL -> Windows host

Exit code 0 = the broker is usable from here. Anything else is a hard stop.
"""

import argparse
import os
import socket
import sys
import time
import uuid

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.properties import Properties

# A payload chosen to catch the things text payloads hide: a NUL, a 0xFF, the
# MAVLink v1/v2 start bytes, and a byte sequence that is not valid UTF-8. If
# anything in the path tries to treat payloads as text, this will not survive.
PROBE_PAYLOAD = bytes([0x00, 0xFF, 0xFD, 0xFE, 0x55, 0xAA]) + b"\x80\x81\x82" + os.urandom(8)


def main(argv=None):
    ap = argparse.ArgumentParser(description="MQTT 5 round-trip smoke test.")
    ap.add_argument("--host", default="localhost", help="broker host (default: localhost)")
    ap.add_argument("--port", type=int, default=1883, help="broker port (default: 1883)")
    ap.add_argument("--topic", default="denel/uav/_smoke/probe", help="topic to round-trip on")
    ap.add_argument("--qos", type=int, default=1, choices=(0, 1, 2))
    ap.add_argument("--timeout", type=float, default=10.0, help="seconds to wait (default: 10)")
    args = ap.parse_args(argv)

    # Unique, because a duplicate client ID makes the broker kick the earlier
    # session — which looks exactly like a broker fault if two people run this
    # at once.
    client_id = f"smoke-{socket.gethostname()}-{uuid.uuid4().hex[:8]}"

    received = []
    connect_result = {}

    client = mqtt.Client(CallbackAPIVersion.VERSION2, client_id=client_id, protocol=mqtt.MQTTv5)

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

    print(f"connecting to {args.host}:{args.port} as {client_id} (MQTT 5, QoS {args.qos})")
    try:
        client.connect(args.host, args.port, keepalive=20, clean_start=True)
    except Exception as exc:
        print(f"FAIL  could not connect: {type(exc).__name__}: {exc}")
        return 2
    client.loop_start()

    deadline = time.monotonic() + args.timeout
    while not received and time.monotonic() < deadline:
        rc = connect_result.get("reason_code")
        if rc is not None and rc.is_failure:
            print(f"FAIL  broker refused the connection: {rc}")
            client.loop_stop()
            return 3
        time.sleep(0.05)

    client.loop_stop()
    client.disconnect()

    if not received:
        print(f"FAIL  no message came back within {args.timeout}s "
              f"(connected={connect_result.get('reason_code')})")
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
