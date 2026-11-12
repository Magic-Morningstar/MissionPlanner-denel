# mqtt_bridge/topics.py
"""MQTT topic construction and vehicle-id validation. Pure: no I/O.

Topic layout (brief section 4), namespaced by vehicle from day one so that
multi-aircraft support and per-vehicle access control need no rename later:

    denel/uav/{vehicle_id}/mavlink/from_vehicle   aircraft -> GCS, QoS 0
    denel/uav/{vehicle_id}/mavlink/to_vehicle     GCS -> aircraft, QoS 1
    denel/uav/{vehicle_id}/status/link            aircraft bridge, retained
    denel/uav/{vehicle_id}/status/gcs             GCS bridge, retained

Separate from_vehicle and to_vehicle topics are the primary loop-prevention
mechanism: a bridge publishes to one and subscribes to the other, never both.
"""

import re

DEFAULT_PREFIX = "denel/uav"

# Deliberately stricter than MQTT requires. MQTT itself only forbids wildcards
# in a published topic, but the HiveMQ File RBAC extension earmarked for Phase 3
# disallows '#' and '+' in client IDs and usernames, and the vehicle_id appears
# in both (aircraft-bridge-{vid}). Enforcing it now means a vehicle id that
# works in development cannot become unusable the day auth is switched on --
# renaming vehicles already in service is the thing to avoid.
#
# '/' is excluded too: it would silently inject extra topic levels and break
# per-vehicle ACL patterns.
_VEHICLE_ID_RE = re.compile(r"^[A-Za-z0-9_-]{1,32}$")


class InvalidVehicleId(ValueError):
    """Raised for a vehicle_id that cannot safely be used in a topic or client id."""


def validate_vehicle_id(vehicle_id):
    """Return vehicle_id unchanged, or raise InvalidVehicleId explaining why not."""
    if not isinstance(vehicle_id, str):
        raise InvalidVehicleId(f"vehicle_id must be a string, got {type(vehicle_id).__name__}")
    if not _VEHICLE_ID_RE.match(vehicle_id):
        raise InvalidVehicleId(
            f"invalid vehicle_id {vehicle_id!r}: use 1-32 characters from "
            "A-Z a-z 0-9 _ - only. MQTT wildcards (# +), '/' and whitespace are "
            "rejected because the id is used in both topic paths and MQTT client "
            "ids, and the Phase 3 broker auth extension forbids them there."
        )
    return vehicle_id


class TopicSet:
    """The four topics for one vehicle, plus the QoS each carries."""

    __slots__ = ("vehicle_id", "prefix", "from_vehicle", "to_vehicle",
                 "status_link", "status_gcs")

    def __init__(self, vehicle_id, prefix=DEFAULT_PREFIX):
        self.vehicle_id = validate_vehicle_id(vehicle_id)
        self.prefix = prefix.rstrip("/")
        base = f"{self.prefix}/{self.vehicle_id}"
        self.from_vehicle = f"{base}/mavlink/from_vehicle"
        self.to_vehicle = f"{base}/mavlink/to_vehicle"
        self.status_link = f"{base}/status/link"
        self.status_gcs = f"{base}/status/gcs"

    def all_topics(self):
        return (self.from_vehicle, self.to_vehicle, self.status_link, self.status_gcs)

    def __repr__(self):
        return f"<TopicSet {self.prefix}/{self.vehicle_id}>"

    def __eq__(self, other):
        return (isinstance(other, TopicSet)
                and self.vehicle_id == other.vehicle_id
                and self.prefix == other.prefix)


def parse_mavlink_topic(topic, prefix=DEFAULT_PREFIX):
    """(vehicle_id, direction) for a mavlink topic, else None.

    direction is "from_vehicle" or "to_vehicle". Used to attribute an inbound
    message when one client subscribes to several vehicles via a wildcard.
    """
    prefix = prefix.rstrip("/")
    if not topic.startswith(prefix + "/"):
        return None
    parts = topic[len(prefix) + 1:].split("/")
    if len(parts) != 3 or parts[1] != "mavlink":
        return None
    vehicle_id, _, direction = parts
    if direction not in ("from_vehicle", "to_vehicle"):
        return None
    if not _VEHICLE_ID_RE.match(vehicle_id):
        return None
    return (vehicle_id, direction)


def wildcard_from_vehicle(prefix=DEFAULT_PREFIX):
    """Subscription covering from_vehicle for every vehicle. Phase 4 fan-out."""
    return f"{prefix.rstrip('/')}/+/mavlink/from_vehicle"


def topic_overhead_bytes(topic):
    """MQTT framing cost of publishing on `topic`, excluding the payload.

    2 bytes of fixed header + 2 bytes of topic length + the topic itself. Worth
    measuring rather than guessing: on a 21-byte HEARTBEAT this dominates, which
    is the whole of brief open question 5 (whether MQTT 5 topic aliases are
    needed over LTE).
    """
    return 2 + 2 + len(topic.encode("utf-8"))
