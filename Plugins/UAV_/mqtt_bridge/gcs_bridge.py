# mqtt_bridge/gcs_bridge.py
"""GCS-side bridge: MQTT broker <-> Mission Planner.

    MP <--(UDP 14570)-- this <--subscribe-- denel/uav/{vid}/mavlink/from_vehicle
    MP --(UDP 14570)--> this --publish-->   denel/uav/{vid}/mavlink/to_vehicle

Connect Mission Planner with the UDPCl option (host 127.0.0.1, port 14570), not
UDP. UdpSerialConnect.Open() returns immediately, whereas UdpSerial.Open() blocks
in a 500ms poll loop until a datagram arrives and would hang the Connect button
until telemetry happened to be flowing.

Start this BEFORE Mission Planner. UdpSerialConnect.Open() never blocks and never
fails -- VerifyConnected() is an empty method -- so connecting MP to a bridge that
is not running gives a cheerfully "connected" MP with no data, which is a
confusing state to debug.

Run:
    python -u -m mqtt_bridge.gcs_bridge --broker-host localhost --vehicle-id uav01
"""

import argparse
import logging
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mqtt_bridge import cli  # noqa: E402
from mqtt_bridge.bridge_core import BridgeBase  # noqa: E402
from mqtt_bridge.mavlink_endpoint import EndpointClosed, UdpPeerEndpoint  # noqa: E402
from mqtt_bridge.mqtt_link import MqttLink  # noqa: E402
from mqtt_bridge.topics import TopicSet  # noqa: E402

logger = logging.getLogger(__name__)

LOG_FILENAME = "denel_mqtt_bridge_gcs.log"

# Larger than the aircraft's inbound queue because this is not pure telemetry:
# parameter and mission DOWNLOADS also arrive on from_vehicle, and dropping those
# breaks a parameter download outright. A loopback sendto essentially never
# blocks, so this queue should never fill -- any drop here is a real defect, which
# is why it is logged at ERROR rather than WARNING.
FROM_VEHICLE_QUEUE_SIZE = 2048


class GcsBridge(BridgeBase):
    reader_thread_name = "MpReader"
    writer_thread_name = "MpWriter"
    endpoint_label = "gcs"
    inbound_drop_level = logging.ERROR

    def __init__(self, cfg):
        topics = TopicSet(cfg.VEHICLE_ID, cfg.TOPIC_PREFIX)
        host, port = cfg.gcs_bind_address()
        endpoint = UdpPeerEndpoint(host, port, name="mp")

        super().__init__(
            cfg=cfg,
            topics=topics,
            endpoint=endpoint,
            mqtt_link_factory=lambda subscriptions, on_frame: MqttLink(
                cfg=cfg,
                client_id=cli.gcs_client_id(cfg.VEHICLE_ID),
                subscriptions=subscriptions,
                on_frame=on_frame,
            ),
            publish_topic=topics.to_vehicle,
            publish_qos=cfg.QOS_TO_VEHICLE,
            publish_expiry=cfg.TO_VEHICLE_EXPIRY,
            subscribe_topic=topics.from_vehicle,
            subscribe_qos=cfg.QOS_FROM_VEHICLE,
            inbound_queue_size=FROM_VEHICLE_QUEUE_SIZE,
        )


def build_parser():
    p = argparse.ArgumentParser(
        description="GCS-side MAVLink-over-MQTT bridge (connect Mission Planner "
                    "with UDPCl to the --udp-bind address).",
        epilog="Every setting also accepts a DENEL_MQTT_<KEY> environment variable.")
    cli.add_common_arguments(p)
    p.add_argument("--udp-bind", dest="GCS_UDP_BIND",
                   help="host:port to bind for Mission Planner (default "
                        "127.0.0.1:14570; avoid 14550-14559)")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    cfg = cli.bootstrap(args, LOG_FILENAME, "gcs")
    if cfg is None:
        return 2

    host, port = cfg.gcs_bind_address()
    logger.info("Mission Planner should connect with UDPCl to %s:%d "
                "(start this bridge before MP)", host, port)
    try:
        GcsBridge(cfg).run_forever()
    except EndpointClosed as exc:
        logger.error("%s", exc)
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
