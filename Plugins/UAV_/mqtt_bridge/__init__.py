# mqtt_bridge — MAVLink-over-MQTT bridge for the Denel GCS.
#
# Two mirrored processes carry raw MAVLink frames through an MQTT broker as
# opaque payloads: aircraft_bridge (next to the flight controller) and
# gcs_bridge (next to Mission Planner). Nothing here decodes or re-encodes
# MAVLink, so v2 message signing survives the hop untouched.
#
# See mavlink-mqtt-bridge-brief.md in the parent directory for requirements.

__all__ = []
