"""Publish a command straight to to_vehicle, bypassing the GCS bridge.

The harness for brief Phase 2 acceptance test 4 -- the stale-command test:

    1. stop the aircraft bridge
    2. inject a command here
    3. wait past its expiry
    4. start the aircraft bridge again
    5. the command must NOT reach the vehicle

It publishes a real, encodable MAVLink frame so that if the protection ever
fails, the consequence is visible at the vehicle rather than merely logged.

    python -m mqtt_bridge.tools.inject_command --command arm --expiry 3

SAFETY: --command arm sends MAV_CMD_COMPONENT_ARM_DISARM. Only ever point this
at a simulator.
"""

import argparse
import sys
import time

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.properties import Properties
from pymavlink.dialects.v20 import ardupilotmega as dialect

if __package__ in (None, ""):
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from mqtt_bridge.mqtt_link import USER_PROP_ORIGINAL_EXPIRY
from mqtt_bridge.topics import TopicSet


def build_frame(command, target_system=1, target_component=1):
    """A raw MAVLink frame, encoded exactly as a GCS would send it."""
    mav = dialect.MAVLink(None, srcSystem=255, srcComponent=190)
    if command == "arm":
        msg = mav.command_long_encode(
            target_system, target_component,
            dialect.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    elif command == "disarm":
        msg = mav.command_long_encode(
            target_system, target_component,
            dialect.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    elif command == "heartbeat":
        msg = mav.heartbeat_encode(dialect.MAV_TYPE_GCS,
                                   dialect.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    else:
        raise ValueError(f"unknown command {command!r}")
    return msg.pack(mav)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Publish a raw command to to_vehicle.")
    ap.add_argument("--broker-host", default="localhost")
    ap.add_argument("--broker-port", type=int, default=1883)
    ap.add_argument("--vehicle-id", default="uav01")
    ap.add_argument("--topic-prefix", default="denel/uav")
    ap.add_argument("--command", default="heartbeat",
                    choices=("arm", "disarm", "heartbeat"))
    ap.add_argument("--expiry", type=int, default=3,
                    help="MessageExpiryInterval seconds (default 3)")
    ap.add_argument("--qos", type=int, default=1, choices=(0, 1, 2))
    ap.add_argument("--repeat", type=int, default=1)
    args = ap.parse_args(argv)

    topics = TopicSet(args.vehicle_id, args.topic_prefix)
    frame = build_frame(args.command)

    client = mqtt.Client(CallbackAPIVersion.VERSION2,
                         client_id=f"inject-{int(time.time())}", protocol=mqtt.MQTTv5)
    client.connect(args.broker_host, args.broker_port, keepalive=20)
    client.loop_start()
    time.sleep(0.5)

    props = Properties(PacketTypes.PUBLISH)
    props.MessageExpiryInterval = args.expiry
    props.UserProperty = [(USER_PROP_ORIGINAL_EXPIRY, str(args.expiry))]

    for i in range(args.repeat):
        info = client.publish(topics.to_vehicle, frame, qos=args.qos,
                              retain=False, properties=props)
        info.wait_for_publish(timeout=5)
        print(f"published {args.command} ({len(frame)} bytes) to "
              f"{topics.to_vehicle} qos={args.qos} expiry={args.expiry}s"
              + (f"  [{i + 1}/{args.repeat}]" if args.repeat > 1 else ""))
        if args.repeat > 1:
            time.sleep(0.2)

    client.loop_stop()
    client.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
