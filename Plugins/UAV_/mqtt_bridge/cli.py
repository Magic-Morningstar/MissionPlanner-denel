# mqtt_bridge/cli.py
"""Shared command-line plumbing and startup sequence for both bridges."""

import logging
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from logging_config import setup_logging  # noqa: E402  (UAV_ package root)

from mqtt_bridge.bridge_config import (  # noqa: E402
    load_config, resolve_log_path, warn_about_contested_port,
)
from mqtt_bridge.topics import InvalidVehicleId, TopicSet  # noqa: E402

logger = logging.getLogger(__name__)


def gcs_client_id(vehicle_id):
    """Unique per machine: two ground stations watching one vehicle is normal.

    A duplicate MQTT client id makes the broker disconnect the earlier session,
    and the kicked client reconnects immediately, producing a flapping loop that
    looks like a broker fault rather than a naming collision.
    """
    import socket
    host = socket.gethostname()
    # Keep it to characters the Phase 3 File RBAC extension tolerates.
    safe = "".join(c if (c.isalnum() or c in "-_") else "-" for c in host)[:32]
    return f"gcs-bridge-{vehicle_id}-{safe}"


def add_common_arguments(parser):
    """Arguments both bridges share. Dest names match config keys exactly."""
    parser.add_argument("--vehicle-id", dest="VEHICLE_ID",
                        help="vehicle id used in topics and the MQTT client id")
    parser.add_argument("--topic-prefix", dest="TOPIC_PREFIX",
                        help="topic namespace (default denel/uav)")
    parser.add_argument("--broker-host", dest="BROKER_HOST")
    parser.add_argument("--broker-port", dest="BROKER_PORT", type=int)
    parser.add_argument("--keepalive", dest="KEEPALIVE", type=int,
                        help="MQTT keepalive seconds; an ungraceful link loss is "
                             "noticed after roughly 1.5x this")
    parser.add_argument("--stdin-shutdown", dest="STDIN_SHUTDOWN",
                        action="store_true", default=None,
                        help="exit when stdin reaches EOF. Only for a parent that "
                             "holds our stdin open (the Mission Planner launcher). "
                             "Do NOT use under systemd, which supplies /dev/null "
                             "and would make the bridge exit immediately.")
    parser.add_argument("--log-level", dest="LOG_LEVEL",
                        choices=("DEBUG", "INFO", "WARNING", "ERROR"))
    parser.add_argument("--log-dir", dest="LOG_DIR")
    return parser


def bootstrap(args, log_filename, role):
    """Set up logging, resolve config, validate. Returns a config or None.

    Logging is configured FIRST, before config parsing or any precondition check,
    because an early failure that is not logged is the failure mode the brief
    specifically warns about: on the GCS machine a relative log path has
    previously resolved into system32 and the process died silently.
    """
    cli_overrides = {k: v for k, v in vars(args).items() if v is not None}

    log_path = resolve_log_path(log_filename, cli_overrides.get("LOG_DIR", ""))
    level = getattr(logging, cli_overrides.get("LOG_LEVEL", "INFO"), logging.INFO)
    setup_logging(file_level=logging.DEBUG, console_level=level, log_file=log_path)
    logger.info("=== Denel MAVLink-over-MQTT bridge (%s) starting ===", role)
    logger.info("log file: %s", log_path)

    cfg = load_config(cli=cli_overrides)

    try:
        TopicSet(cfg.VEHICLE_ID, cfg.TOPIC_PREFIX)
    except InvalidVehicleId as exc:
        logger.error("%s", exc)
        return None

    for key, value in sorted(cfg.loggable().items()):
        logger.debug("config %s=%r", key, value)
    logger.info("vehicle=%s broker=%s:%d keepalive=%ds (link loss detected in ~%ds)",
                cfg.VEHICLE_ID, cfg.BROKER_HOST, cfg.BROKER_PORT,
                cfg.KEEPALIVE, cfg.link_loss_detect_seconds)

    if role == "gcs":
        _, port = cfg.gcs_bind_address()
        warn_about_contested_port(port, what="the GCS bridge")

    return cfg
