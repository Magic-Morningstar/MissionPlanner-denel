"""Watch the retained status topics: is the aircraft bridge up, is the GCS up?

    python -m mqtt_bridge.tools.status_watch --vehicle-id uav01

Prints every change, with a timestamp. Because the topics are retained, the
current state arrives immediately on subscribe rather than at the next change --
which is the point of retaining them.

Also the acceptance harness for brief Phase 2 tests 3 and 5: kill the aircraft
bridge ungracefully and 'offline' should arrive via the Last Will; stop it
cleanly and 'offline' should arrive because the bridge publishes it itself (a
graceful DISCONNECT suppresses the Will).
"""

import argparse
import sys
import time

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion

if __package__ in (None, ""):
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from mqtt_bridge.topics import TopicSet


def main(argv=None):
    ap = argparse.ArgumentParser(description="Watch bridge status topics.")
    ap.add_argument("--broker-host", default="localhost")
    ap.add_argument("--broker-port", type=int, default=1883)
    ap.add_argument("--vehicle-id", default="uav01")
    ap.add_argument("--topic-prefix", default="denel/uav")
    ap.add_argument("--seconds", type=float, default=0,
                    help="exit after this long (0 = run until Ctrl+C)")
    ap.add_argument("--expect", help="exit 0 as soon as this value is seen "
                                     "(e.g. 'offline'), else exit 1 at timeout")
    args = ap.parse_args(argv)

    topics = TopicSet(args.vehicle_id, args.topic_prefix)
    seen = []

    client = mqtt.Client(CallbackAPIVersion.VERSION2,
                         client_id=f"status-watch-{int(time.time())}",
                         protocol=mqtt.MQTTv5)

    def on_connect(c, userdata, flags, rc, props):
        for t in (topics.status_link, topics.status_gcs):
            c.subscribe(t, qos=1)
        print(f"watching {topics.status_link} and {topics.status_gcs}")

    def on_message(c, userdata, msg):
        value = msg.payload.decode("utf-8", "replace")
        who = "aircraft" if msg.topic.endswith("/link") else "gcs     "
        stamp = time.strftime("%H:%M:%S")
        print(f"{stamp}  {who}  {value}"
              + ("   (retained)" if msg.retain else ""), flush=True)
        seen.append((msg.topic, value))

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.broker_host, args.broker_port, keepalive=20)
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
