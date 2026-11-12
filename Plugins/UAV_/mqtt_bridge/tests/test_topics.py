# mqtt_bridge/tests/test_topics.py

import pytest

from mqtt_bridge.topics import (
    DEFAULT_PREFIX, InvalidVehicleId, TopicSet, parse_mavlink_topic,
    topic_overhead_bytes, validate_vehicle_id, wildcard_from_vehicle,
)


def test_topic_layout_matches_the_brief():
    t = TopicSet("uav01")
    assert t.from_vehicle == "denel/uav/uav01/mavlink/from_vehicle"
    assert t.to_vehicle == "denel/uav/uav01/mavlink/to_vehicle"
    assert t.status_link == "denel/uav/uav01/status/link"
    assert t.status_gcs == "denel/uav/uav01/status/gcs"


def test_from_and_to_are_distinct():
    """Separate topics per direction are the primary loop-prevention mechanism."""
    t = TopicSet("uav01")
    assert t.from_vehicle != t.to_vehicle
    assert len(set(t.all_topics())) == 4


def test_prefix_override_and_trailing_slash():
    assert TopicSet("uav01", prefix="x/y").from_vehicle == "x/y/uav01/mavlink/from_vehicle"
    assert TopicSet("uav01", prefix="x/y/").from_vehicle == "x/y/uav01/mavlink/from_vehicle"


@pytest.mark.parametrize("vid", ["uav01", "a", "A-Z_0-9", "x" * 32, "250-FW-VTOL"])
def test_valid_vehicle_ids(vid):
    assert validate_vehicle_id(vid) == vid


@pytest.mark.parametrize("vid", [
    "",             # empty
    "x" * 33,       # too long
    "uav/01",       # extra topic level
    "uav+01",       # MQTT single-level wildcard
    "uav#01",       # MQTT multi-level wildcard
    "#",
    "+",
    "uav 01",       # whitespace
    "uav\t01",
    "uav\n01",
    "uav.01",       # '.' is not in the allowed set
    "uavé01",  # non-ASCII
])
def test_rejected_vehicle_ids(vid):
    with pytest.raises(InvalidVehicleId):
        validate_vehicle_id(vid)


def test_wildcards_are_rejected_for_the_stated_reason():
    """'#' and '+' must fail now, not when Phase 3 auth is switched on.

    The id goes into the MQTT client id as well as the topic, and the HiveMQ File
    RBAC extension forbids both characters there. Catching it at construction
    avoids renaming vehicles that are already in service.
    """
    with pytest.raises(InvalidVehicleId, match="wildcard"):
        TopicSet("uav+01")


def test_non_string_rejected():
    with pytest.raises(InvalidVehicleId):
        validate_vehicle_id(1)
    with pytest.raises(InvalidVehicleId):
        validate_vehicle_id(None)


def test_parse_round_trip():
    t = TopicSet("uav01")
    assert parse_mavlink_topic(t.from_vehicle) == ("uav01", "from_vehicle")
    assert parse_mavlink_topic(t.to_vehicle) == ("uav01", "to_vehicle")


def test_parse_rejects_non_mavlink_and_malformed():
    t = TopicSet("uav01")
    assert parse_mavlink_topic(t.status_link) is None          # status, not mavlink
    assert parse_mavlink_topic("other/uav01/mavlink/to_vehicle") is None
    assert parse_mavlink_topic("denel/uav/uav01/mavlink/sideways") is None
    assert parse_mavlink_topic("denel/uav/uav01/mavlink") is None
    assert parse_mavlink_topic("denel/uav/uav01/telem/position") is None
    assert parse_mavlink_topic("") is None


def test_parse_honours_custom_prefix():
    assert parse_mavlink_topic("x/y/uav01/mavlink/to_vehicle", prefix="x/y") == ("uav01", "to_vehicle")
    assert parse_mavlink_topic("x/y/uav01/mavlink/to_vehicle") is None


def test_wildcard_subscription_shape():
    assert wildcard_from_vehicle() == "denel/uav/+/mavlink/from_vehicle"


def test_topic_overhead_is_the_number_open_question_5_needs():
    """Record the real MQTT framing cost; it dominates small MAVLink frames.

    A v2 HEARTBEAT is 21 bytes on the wire. If the topic framing exceeds that,
    MQTT 5 topic aliases become the obvious lever over a metered LTE link -- and
    that is a measurement, not an opinion.
    """
    topic = TopicSet("uav01").from_vehicle
    assert len(topic) == 36
    overhead = topic_overhead_bytes(topic)
    assert overhead == 4 + 36 == 40

    # 40 bytes of MQTT framing to carry a 21-byte frame: the hop costs ~190% more
    # than the data. Properties (MessageExpiryInterval + the 'exp' user property)
    # add ~8 more on to_vehicle.
    heartbeat_v2_len = 21
    assert overhead > heartbeat_v2_len
    assert round(overhead / heartbeat_v2_len, 1) == 1.9


def test_default_prefix_constant():
    assert DEFAULT_PREFIX == "denel/uav"
