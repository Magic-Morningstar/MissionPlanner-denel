"""Watch the retained status topics: aircraft up, GCS up, aircraft health.

    python -m mqtt_bridge.tools.status_watch --vehicle-id uav01
    python -m mqtt_bridge.tools.status_watch --broker-host <cluster> --tls --username uav01-gcs

Prints every change, with a timestamp. Because the topics are retained, the
current state arrives immediately on subscribe rather than at the next change --
which is the point of retaining them. Health reports (JSON from the aircraft
bridge, see health.py) are shown as one summary line each.

With --tls the port defaults to 8883 and the password comes from
DENEL_MQTT_MQTT_PASSWORD or a prompt, never the command line. On the Azure test
broker use a user allowed to read status/+, e.g. uav01-gcs.

Also the acceptance harness for brief Phase 2 tests 3 and 5: kill the aircraft
bridge ungracefully and 'offline' should arrive via the Last Will; stop it
cleanly and 'offline' should arrive because the bridge publishes it itself (a
graceful DISCONNECT suppresses the Will).
"""

import argparse
import getpass
import json
import os
import sys
import time
from types import SimpleNamespace

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion

if __package__ in (None, ""):
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from mqtt_bridge.tls import (  # noqa: E402
    PLAIN_DEFAULT_PORT, TLS_DEFAULT_PORT, build_ssl_context, tls_config_problems,
)
from mqtt_bridge.topics import TopicSet  # noqa: E402

PASSWORD_ENV = "DENEL_MQTT_MQTT_PASSWORD"


def summarise_health(payload):
    """One line from a health JSON payload. Pure, so it is testable."""
    try:
        h = json.loads(payload)
    except (ValueError, TypeError):
        return "health: (not JSON)"
    link = h.get("link") or {}
    lte = h.get("lte") or {}
    modem = lte.get("modem") or {}
    state = "OK" if h.get("ok") else "WARN " + ",".join(h.get("warnings") or [])
    parts = [
        state,
        f"cube {link.get('fc_frames_per_s')} fr/s",
        f"cmds {link.get('cmd_frames_per_s')} fr/s",
        f"lte {lte.get('ipv4') or 'no address'}",
    ]
    if modem.get("signal_pct") is not None:
        parts.append(f"signal {modem['signal_pct']}% {modem.get('tech') or ''}".rstrip())
    if h.get("cpu_temp_c") is not None:
        parts.append(f"cpu {h['cpu_temp_c']}C")
    return "health: " + " | ".join(parts)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Watch bridge status topics.")
    ap.add_argument("--broker-host", default="localhost")
    ap.add_argument("--broker-port", type=int, default=None,
                    help=f"default {PLAIN_DEFAULT_PORT}, or {TLS_DEFAULT_PORT} with --tls")
    ap.add_argument("--tls", action="store_true")
    ap.add_argument("--ca-file", default="")
    ap.add_argument("--username", default="")
    ap.add_argument("--vehicle-id", default="uav01")
    ap.add_argument("--topic-prefix", default="denel/uav")
    ap.add_argument("--raw", action="store_true", help="print health JSON in full")
    ap.add_argument("--seconds", type=float, default=0,
                    help="exit after this long (0 = run until Ctrl+C)")
    ap.add_argument("--expect", help="exit 0 as soon as this value is seen "
                                     "(e.g. 'offline'), else exit 1 at timeout")
    args = ap.parse_args(argv)

    port = args.broker_port or (TLS_DEFAULT_PORT if args.tls else PLAIN_DEFAULT_PORT)
    password = os.environ.get(PASSWORD_ENV, "")
    if args.username and not password and sys.stdin.isatty():
        password = getpass.getpass(f"password for {args.username!r}: ")
    cfg = SimpleNamespace(TLS_ENABLED=args.tls, TLS_CA_FILE=args.ca_file,
                          TLS_CERT_FILE="", TLS_KEY_FILE="")
    for problem in tls_config_problems(cfg):
        print(f"FAIL  {problem}")
        return 6

    topics = TopicSet(args.vehicle_id, args.topic_prefix)
    seen = []

    client = mqtt.Client(CallbackAPIVersion.VERSION2,
                         client_id=f"status-watch-{int(time.time())}",
                         protocol=mqtt.MQTTv5)
    if args.username:
        client.username_pw_set(args.username, password or None)
    ctx = build_ssl_context(cfg)
    if ctx is not None:
        client.tls_set_context(ctx)

    watched = (topics.status_link, topics.status_gcs, topics.status_health)

    def on_connect(c, userdata, flags, rc, props):
        if rc.is_failure:
            print(f"broker refused the connection: {rc}")
            return
        for t in watched:
            c.subscribe(t, qos=1)
        print("watching " + ", ".join(watched))

    def on_message(c, userdata, msg):
        stamp = time.strftime("%H:%M:%S")
        retained = "   (retained)" if msg.retain else ""
        if msg.topic == topics.status_health:
            text = (msg.payload.decode("utf-8", "replace") if args.raw
                    else summarise_health(msg.payload))
            print(f"{stamp}  aircraft  {text}{retained}", flush=True)
            return
        value = msg.payload.decode("utf-8", "replace")
        who = "aircraft" if msg.topic.endswith("/link") else "gcs     "
        print(f"{stamp}  {who}  {value}{retained}", flush=True)
        seen.append((msg.topic, value))

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.broker_host, port, keepalive=20)
    client.loop_start()

    deadline = time.monotonic() + (args.seconds or 86400)
    try:
        while time.monotonic() < deadline:
            if args.expect and any(v == args.expect for _, v in seen):
                print(f"\nPASS  saw {args.expect!r}")
                return 0
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nstopped")
        return 0
    finally:
        client.loop_stop()
        client.disconnect()

    if args.expect:
        print(f"\nFAIL  never saw {args.expect!r} (saw: "
              f"{[v for _, v in seen] or 'nothing'})")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
