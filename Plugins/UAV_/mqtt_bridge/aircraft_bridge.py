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
import json
import logging
import sys
import time
from pathlib import Path

# Allow `python mqtt_bridge/aircraft_bridge.py` as well as `python -m`.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mqtt_bridge import cli  # noqa: E402
from mqtt_bridge.bridge_core import BridgeBase  # noqa: E402
from mqtt_bridge.health import build_health, probe_system  # noqa: E402
from mqtt_bridge.led import SysfsLed, run_status_led  # noqa: E402
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
    # This side receives commands, so it is the side that checks their age.
    check_staleness = True

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
                status_topic=topics.status_link,
            ),
            publish_topic=topics.from_vehicle,
            publish_qos=cfg.QOS_FROM_VEHICLE,
            publish_expiry=cfg.FROM_VEHICLE_EXPIRY,
            subscribe_topic=topics.to_vehicle,
            subscribe_qos=cfg.QOS_TO_VEHICLE,
            inbound_queue_size=TO_VEHICLE_QUEUE_SIZE,
        )

    def start(self):
        super().start()
        if self.cfg.HEALTH_INTERVAL > 0:
            self._spawn("Health", self._health_loop)
        if self.cfg.STATUS_LED:
            self._spawn("StatusLed", self._status_led_loop)

    def _status_led_loop(self):
        """Blink the on-board LED with the link state (see led.py)."""
        try:
            run_status_led(
                SysfsLed(self.cfg.STATUS_LED), self._stop,
                frame_count=lambda: self._out_framer.counters()["frames"],
                broker_up=lambda: self.mqtt.connected)
        except Exception:
            logger.exception("status LED stopped (the link is unaffected)")

    def _health_snapshot(self):
        return {
            "fc_frames": self._out_framer.counters()["frames"],
            "cmd_frames": self.frames_in,
            "wakeup_repairs": self.mqtt.wakeup_repairs,
        }

    def health_link_fields(self, prev, cur, seconds):
        """Link section of the health report, from two counter snapshots."""
        seconds = max(seconds, 0.001)
        return {
            "mqtt_up": self.mqtt.connected,
            "fc_open": self.endpoint.is_open,
            "fc_frames_per_s": round((cur["fc_frames"] - prev["fc_frames"]) / seconds, 1),
            "cmd_frames_per_s": round((cur["cmd_frames"] - prev["cmd_frames"]) / seconds, 2),
            "wakeup_repairs": cur["wakeup_repairs"] - prev["wakeup_repairs"],
            "mqtt_disconnects": self.mqtt.disconnects,
            "publish_failures": self.publish_failures,
            "dropped_stale": self.dropped_stale,
        }

    def _health_refused(self, sent_at, grace=3.0):
        """Stop health reports if the broker kicked us for sending one.

        HiveMQ answers an unauthorised PUBLISH by DISCONNECTING the client, so a
        missing status/health permission would drop the whole link every
        interval (seen 2026-11-17 before the broker ACL was updated). A
        monitoring feature must never cost the link: on a "Not authorized"
        disconnect within `grace` seconds of our publish, health reporting is
        switched off until the bridge restarts. Returns True if it was.
        """
        if self._stop.wait(grace):
            return True
        at = self.mqtt.last_disconnect_at
        reason = (self.mqtt.last_disconnect_reason or "").lower()
        if at is not None and sent_at <= at <= sent_at + grace and "authori" in reason:
            logger.error("broker refused the health report (%s): the aircraft user "
                         "needs PUBLISH on %s. Health reporting is OFF until the "
                         "bridge restarts; the MAVLink link is unaffected.",
                         self.mqtt.last_disconnect_reason, self.topics.status_health)
            return True
        return False

    def _health_loop(self):
        """Publish a retained health report every HEALTH_INTERVAL seconds.

        Never allowed to affect the link: every failure is logged and the loop
        carries on. Warnings are logged when they change, so the local log also
        shows when the Cube went quiet or the signal dropped.
        """
        interval = self.cfg.HEALTH_INTERVAL
        prev, prev_t = self._health_snapshot(), time.monotonic()
        last_warnings = None
        while not self._stop.wait(interval):
            try:
                cur, now_t = self._health_snapshot(), time.monotonic()
                link = self.health_link_fields(prev, cur, now_t - prev_t)
                prev, prev_t = cur, now_t
                report = build_health(
                    probe_system(self.cfg.HEALTH_LTE_IFACE, self.cfg.HEALTH_READ_MODEM),
                    link)
                if report["warnings"] != last_warnings:
                    last_warnings = report["warnings"]
                    if last_warnings:
                        logger.warning("health: %s", ", ".join(last_warnings))
                    else:
                        logger.info("health: ok")
                sent_at = time.monotonic()
                self.mqtt.publish_status(
                    self.topics.status_health,
                    json.dumps(report, separators=(",", ":")).encode("utf-8"),
                    expiry_s=interval * 3)
                if self._health_refused(sent_at):
                    return
            except Exception:
                logger.exception("health report failed (the link is unaffected)")


def build_parser():
    p = argparse.ArgumentParser(
        description="Aircraft-side MAVLink-over-MQTT bridge.",
        epilog="Every setting also accepts a DENEL_MQTT_<KEY> environment variable.")
    cli.add_common_arguments(p)
    p.add_argument("--mavlink", dest="AIRCRAFT_MAVLINK",
                   help="pymavlink connection string for the flight controller "
                        "(/dev/ttyACM0,115200 for USB or /dev/ttyS1,57600 for serial, udpin:0.0.0.0:14571, tcp:host:5760; pymavlink has no 'serial:' prefix)")
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
