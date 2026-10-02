# mqtt_bridge/aircraft_bridge.py
"""Aircraft-side bridge: flight controller <-> MQTT broker.

    FC --(serial/UDP/TCP)--> this --publish--> denel/uav/{vid}/mavlink/from_vehicle
    FC <--(serial/UDP/TCP)-- this <--subscribe-- denel/uav/{vid}/mavlink/to_vehicle

Host-agnostic by design (brief open question 1 is still open): the MAVLink
endpoint is any pymavlink connection string, so this runs wherever there is a
route to the flight controller and to the broker. Nothing here assumes a
companion computer, an Ethernet link, or a serial port.

Run:
    python -u -m mqtt_bridge.aircraft_bridge --mavlink udpin:127.0.0.1:14571 \
        --broker-host localhost --vehicle-id uav01
"""

import argparse
import logging
import sys
from pathlib import Path

# Allow `python mqtt_bridge/aircraft_bridge.py` as well as `python -m`.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mqtt_bridge import cli  # noqa: E402
from mqtt_bridge.bridge_core import BridgeBase  # noqa: E402
from mqtt_bridge.mavlink_endpoint import MavutilEndpoint  # noqa: E402
from mqtt_bridge.mqtt_link import MqttLink  # noqa: E402
from mqtt_bridge.topics import TopicSet  # noqa: E402

logger = logging.getLogger(__name__)

LOG_FILENAME = "denel_mqtt_bridge_aircraft.log"

# Commands arriving while the FC link is down are deliberately not buffered
# beyond this. Holding them would recreate the stale-command hazard inside our
# own process, which QoS 1 does nothing to protect against -- it guarantees the
# broker hop only.
TO_VEHICLE_QUEUE_SIZE = 256


class AircraftBridge(BridgeBase):
    reader_thread_name = "FcReader"
    writer_thread_name = "FcWriter"
    endpoint_label = "aircraft"

    def __init__(self, cfg):
        topics = TopicSet(cfg.VEHICLE_ID, cfg.TOPIC_PREFIX)
        endpoint = MavutilEndpoint(cfg.AIRCRAFT_MAVLINK, name="fc")

        super().__init__(
            cfg=cfg,
            topics=topics,
            endpoint=endpoint,
            mqtt_link_factory=lambda subscriptions, on_frame: MqttLink(
                cfg=cfg,
                client_id=f"aircraft-bridge-{cfg.VEHICLE_ID}",
                subscriptions=subscriptions,
                on_frame=on_frame,
            ),
            publish_topic=topics.from_vehicle,
            publish_qos=cfg.QOS_FROM_VEHICLE,
            publish_expiry=cfg.FROM_VEHICLE_EXPIRY,
            subscribe_topic=topics.to_vehicle,
            subscribe_qos=cfg.QOS_TO_VEHICLE,
            inbound_queue_size=TO_VEHICLE_QUEUE_SIZE,
        )


def build_parser():
    p = argparse.ArgumentParser(
        description="Aircraft-side MAVLink-over-MQTT bridge.",
        epilog="Every setting also accepts a DENEL_MQTT_<KEY> environment variable.")
    cli.add_common_arguments(p)
    p.add_argument("--mavlink", dest="AIRCRAFT_MAVLINK",
                   help="pymavlink connection string for the flight controller "
                        "(serial:/dev/ttyS1:57600, udpin:0.0.0.0:14571, tcp:host:5760)")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    cfg = cli.bootstrap(args, LOG_FILENAME, "aircraft")
    if cfg is None:
        return 2

    logger.info("flight controller endpoint: %s", cfg.AIRCRAFT_MAVLINK)
    AircraftBridge(cfg).run_forever()
    return 0


if __name__ == "__main__":
    sys.exit(main())
