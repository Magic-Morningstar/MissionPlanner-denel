# mqtt_bridge/bridge_config.py
"""Bridge configuration: flat defaults, env overrides, CLI overrides.

Shape follows state/system_config.py -- UPPER_SNAKE module-level constants -- so
it is greppable and so the csproj's recursive `plugins\\UAV_\\**\\*.py` glob
deploys it with no extra build step. A .yaml or .ini file would need its own
csproj entry and would silently never reach an operator machine.

Precedence, lowest to highest:

    DEFAULTS  <  secrets file  <  DENEL_MQTT_* environment  <  CLI flag

build_config() is a pure function of plain dicts. It never reads os.environ or
touches the filesystem, so precedence is unit-testable without monkeypatching
the process environment. load_config() is the thin impure wrapper that gathers
the dicts and calls it.

A bad value logs an ERROR naming the key and falls back to the default rather
than raising. A typo in one environment variable must not stop an aircraft
bridge from starting.
"""

import logging
import os
import shlex
from dataclasses import dataclass, field, fields
from pathlib import Path

logger = logging.getLogger(__name__)

ENV_PREFIX = "DENEL_MQTT_"

# Secrets live outside the repo tree in a file matching this name, gitignored.
# Only credentials belong here; everything else should be visible in config.
SECRETS_FILENAME = "secrets.env"


# --------------------------------------------------------------------------
# Defaults. Every key here is overridable by DENEL_MQTT_<KEY> and by a CLI flag.
# --------------------------------------------------------------------------

DEFAULTS = {
    # Identity -------------------------------------------------------------
    "VEHICLE_ID": "uav01",
    "TOPIC_PREFIX": "denel/uav",

    # Broker ---------------------------------------------------------------
    "BROKER_HOST": "localhost",
    "BROKER_PORT": 1883,

    # An ungraceful drop (LTE dying mid-flight, not a clean disconnect) is only
    # noticed after roughly 1.5x keepalive, so this number IS the link-loss
    # detection time: ~30 s at 20. Lower it for faster detection at the cost of
    # more PINGREQ/PINGRESP traffic on a metered link. The bridges log the
    # derived detection time at startup so the trade-off is visible.
    "KEEPALIVE": 20,

    # QoS ------------------------------------------------------------------
    # 0 for telemetry is load-bearing for SAFETY, not just bandwidth: paho drops
    # QoS 0 publishes outright when offline, whereas QoS >= 1 enters its
    # out-queue and is RE-SENT on reconnect. Raising this to 1 turns a dropped
    # telemetry frame into a burst of stale telemetry after every reconnect.
    "QOS_FROM_VEHICLE": 0,
    "QOS_TO_VEHICLE": 1,      # commands, params, missions must not be lost
    "QOS_STATUS": 1,

    # Message expiry, seconds.
    "FROM_VEHICLE_EXPIRY": 5,

    # 30 s, raised from 3 s on 2026-10-09 (Taariq's call, interim).
    #
    # 3 s was too tight for cellular: Phase 1c found commands intermittently
    # discarded at the broker on a stalling link, and the symptom was silence
    # rather than an error -- the vehicle simply never received the request. See
    # LINK-TESTING.md finding 3.
    #
    # Understand the trade before changing it. This value is the ONLY thing
    # currently bounding how stale a command can be when it reaches the aircraft,
    # because the receive-side dwell check (staleness layer 6) is not built yet.
    # At 30 s a command delayed by up to half a minute can still execute. That is
    # acceptable for simulator and bench work; it is NOT a flight setting, and it
    # should be revisited together with TO_VEHICLE_FAIL_OPEN once the dwell check
    # exists and can reject on measured age instead of on a blunt timeout.
    "TO_VEHICLE_EXPIRY": 30,

    # Session. clean_start + session expiry 0 on the aircraft is the primary
    # stale-command defence: with no session there is nothing for the broker to
    # queue commands into while the aircraft is offline.
    "CLEAN_START": True,
    "SESSION_EXPIRY": 0,

    # Bound paho's own outgoing queue. Its default is 0, meaning UNLIMITED, and
    # QoS 1 messages published while disconnected are retained for resend. On
    # to_vehicle that is an unbounded stale-command backlog, so this is set from
    # the very first commit rather than left for a robustness phase.
    "MAX_QUEUED_MESSAGES": 256,
    "MAX_INFLIGHT_MESSAGES": 20,

    # MAVLink endpoints ----------------------------------------------------
    # Any pymavlink connection string: serial:, udpin:, udpout:, tcp:. The
    # aircraft bridge makes no assumption about where it runs (brief open
    # question 1) -- only that it can reach the flight controller somehow.
    "AIRCRAFT_MAVLINK": "udpin:127.0.0.1:14571",

    # Mission Planner connects here with the UDPCl option. 14570 avoids
    # 14550-14559 entirely: 14550 is bound at startup by AutoConnect and never
    # released, 14551 is reserved for the STM32 bridge, and 14552-14559 are
    # reachable by MavlinkWorker._connect_with_retry's cyclic port walk.
    # Loopback, never 0.0.0.0 -- Mission Planner's UDP inbound paths do no
    # heartbeat validation, so a wildcard bind on a ground station is a
    # LAN-facing MAVLink injection surface.
    "GCS_UDP_BIND": "127.0.0.1:14570",

    # Behaviour ------------------------------------------------------------
    # Re-frame each received payload to assert it is exactly one whole frame.
    # Costs microseconds and turns transport corruption into a counter instead
    # of a mystery.
    "VERIFY_INBOUND_FRAMES": True,

    # Exit when a parent process closes our stdin. OFF by default and deliberately
    # not auto-detected: stdin that is closed or /dev/null is indistinguishable
    # from a held-open pipe without reading it, and systemd hands a service
    # /dev/null, which would make the bridge exit the moment it starts. The
    # Mission Planner launcher plugin passes --stdin-shutdown explicitly.
    "STDIN_SHUTDOWN": False,

    "RECONNECT_MIN_DELAY": 1,
    "RECONNECT_MAX_DELAY": 30,

    # Seconds between periodic counter lines. 0 disables them. Without this the
    # counters only appear at shutdown, which is no use while watching a live
    # test. Quiet by design: a line is only emitted when something changed.
    "METRICS_INTERVAL": 10,

    # Logging --------------------------------------------------------------
    "LOG_LEVEL": "INFO",
    "LOG_DIR": "",            # empty = resolve_log_path() decides

    # TLS. Declared now so Phase 3 adds no new keys, but unused until then.
    "TLS_ENABLED": False,
    "TLS_CA_FILE": "",
    "TLS_CERT_FILE": "",
    "TLS_KEY_FILE": "",
    "MQTT_USERNAME": "",
    "MQTT_PASSWORD": "",      # from the secrets file or env, never committed
}

# Keys whose values must never be written to a log line.
SECRET_KEYS = frozenset({"MQTT_PASSWORD"})

# Ports a bridge must not bind. Binding one of these does not fail loudly: a
# .NET UdpClient and pymavlink's mavudp both set SO_REUSEADDR, so on Windows a
# second bind succeeds and datagrams reach only one socket. Silent half-dead
# telemetry is much worse than a startup warning.
CONTESTED_PORTS = range(14550, 14560)


@dataclass(frozen=True)
class BridgeConfig:
    """Resolved configuration. Frozen so nothing mutates it after startup."""

    VEHICLE_ID: str
    TOPIC_PREFIX: str
    BROKER_HOST: str
    BROKER_PORT: int
    KEEPALIVE: int
    QOS_FROM_VEHICLE: int
    QOS_TO_VEHICLE: int
    QOS_STATUS: int
    FROM_VEHICLE_EXPIRY: int
    TO_VEHICLE_EXPIRY: int
    CLEAN_START: bool
    SESSION_EXPIRY: int
    MAX_QUEUED_MESSAGES: int
    MAX_INFLIGHT_MESSAGES: int
    AIRCRAFT_MAVLINK: str
    GCS_UDP_BIND: str
    VERIFY_INBOUND_FRAMES: bool
    STDIN_SHUTDOWN: bool
    RECONNECT_MIN_DELAY: int
    RECONNECT_MAX_DELAY: int
    METRICS_INTERVAL: int
    LOG_LEVEL: str
    LOG_DIR: str
    TLS_ENABLED: bool
    TLS_CA_FILE: str
    TLS_CERT_FILE: str
    TLS_KEY_FILE: str
    MQTT_USERNAME: str
    MQTT_PASSWORD: str = field(repr=False)   # keep it out of repr() by default

    @property
    def link_loss_detect_seconds(self):
        """Roughly how long an ungraceful disconnect goes unnoticed."""
        return round(self.KEEPALIVE * 1.5)

    def gcs_bind_address(self):
        """(host, port) parsed from GCS_UDP_BIND."""
        return _parse_host_port(self.GCS_UDP_BIND)

    def loggable(self):
        """A dict safe to log: secrets replaced with a placeholder."""
        out = {}
        for f in fields(self):
            value = getattr(self, f.name)
            out[f.name] = "<set>" if (f.name in SECRET_KEYS and value) else value
        return out


# --------------------------------------------------------------------------
# Coercion
# --------------------------------------------------------------------------

_TRUE = {"1", "true", "yes", "on"}
_FALSE = {"0", "false", "no", "off"}


def coerce(key, raw, default):
    """Coerce `raw` to the type of `default`. Returns default on bad input.

    Logs an ERROR naming the key rather than raising: one mistyped environment
    variable must not prevent a bridge from starting.
    """
    if raw is None:
        return default
    if isinstance(raw, type(default)) and not isinstance(raw, str):
        return raw
    text = str(raw).strip()
    try:
        if isinstance(default, bool):
            low = text.lower()
            if low in _TRUE:
                return True
            if low in _FALSE:
                return False
            raise ValueError(f"expected a boolean, got {text!r}")
        if isinstance(default, int):
            return int(text, 10)
        return text
    except (TypeError, ValueError) as exc:
        logger.error("config %s=%r is invalid (%s); using default %r",
                     key, raw, exc, default)
        return default


# --------------------------------------------------------------------------
# The pure resolver
# --------------------------------------------------------------------------

def build_config(defaults=None, secrets=None, env=None, cli=None):
    """Resolve configuration from plain dicts. Pure: no env, no filesystem.

    `env` is the whole environment; only DENEL_MQTT_-prefixed keys are consulted
    and the prefix is stripped. `secrets` and `cli` use bare key names. Unknown
    keys in any source are ignored with a DEBUG note -- an unrecognised
    DENEL_MQTT_* variable is far more likely a typo than a reason to refuse to
    start, and the ERROR path is reserved for values we cannot use.
    """
    defaults = dict(DEFAULTS if defaults is None else defaults)
    merged = dict(defaults)

    def apply(source, label, strip_prefix=False):
        if not source:
            return
        for raw_key, raw_value in source.items():
            key = raw_key[len(ENV_PREFIX):] if strip_prefix else raw_key
            if strip_prefix and not raw_key.startswith(ENV_PREFIX):
                continue
            key = key.upper()
            if key not in defaults:
                logger.debug("ignoring unknown %s key %s", label, raw_key)
                continue
            if raw_value is None:
                continue        # an unset CLI flag is not an override
            merged[key] = coerce(key, raw_value, defaults[key])

    apply(secrets, "secrets")
    apply(env, "environment", strip_prefix=True)
    apply(cli, "command line")

    return BridgeConfig(**merged)


# --------------------------------------------------------------------------
# Impure helpers
# --------------------------------------------------------------------------

def parse_secrets_file(text):
    """Parse KEY=value lines. Pure, so it is testable without a file.

    Blank lines and # comments are skipped. Values may be quoted, which matters
    for passwords containing spaces or '#'.
    """
    out = {}
    for lineno, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            logger.error("%s line %d is not KEY=value, ignoring", SECRETS_FILENAME, lineno)
            continue
        key, _, value = line.partition("=")
        try:
            parts = shlex.split(value.strip())
            value = parts[0] if parts else ""
        except ValueError:
            value = value.strip()
        out[key.strip().upper()] = value
    return out


def load_secrets(directory=None):
    """Read secrets.env next to this module, or {} if absent."""
    directory = Path(directory) if directory else Path(__file__).resolve().parent
    path = directory / SECRETS_FILENAME
    if not path.is_file():
        return {}
    try:
        secrets = parse_secrets_file(path.read_text(encoding="utf-8"))
        logger.info("loaded %d setting(s) from %s", len(secrets), path)
        return secrets
    except OSError as exc:
        logger.error("could not read %s: %s", path, exc)
        return {}


def load_config(cli=None, directory=None, env=None):
    """Gather defaults, secrets file and environment, then resolve."""
    return build_config(
        defaults=DEFAULTS,
        secrets=load_secrets(directory),
        env=os.environ if env is None else env,
        cli=cli,
    )


def resolve_log_path(filename, log_dir=""):
    """Absolute log file path, never dependent on the working directory.

    The brief is emphatic about this: on the GCS machine a relative path has
    previously resolved into system32 and failed silently. logging_config.py's
    DEFAULT_LOG_FILE is a bare relative name for exactly that reason, so the
    bridges pass an absolute path instead of inheriting it. That shared module is
    left alone -- main.py uses it too.

    Windows gets %PROGRAMDATA%\\Denel GCS, matching where DenelPythonLauncher
    already puts denel_python.log: always writable wherever the release ZIP was
    extracted, and it survives replacing the app folder on an upgrade.
    """
    explicit = log_dir or os.environ.get(ENV_PREFIX + "LOG_DIR", "")
    if explicit:
        directory = Path(explicit)
    elif os.name == "nt":
        directory = Path(os.environ.get("PROGRAMDATA", r"C:\ProgramData")) / "Denel GCS"
    else:
        directory = Path(__file__).resolve().parent / "logs"

    try:
        directory.mkdir(parents=True, exist_ok=True)
    except OSError:
        # Never let an unwritable log directory stop the bridge. Fall back next
        # to this module, which is writable in every deployment seen so far.
        fallback = Path(__file__).resolve().parent / "logs"
        fallback.mkdir(parents=True, exist_ok=True)
        directory = fallback

    return str(directory / filename)


def warn_about_contested_port(port, what="bridge"):
    """Log a WARNING if `port` is one a bind will silently share. Returns bool."""
    if port in CONTESTED_PORTS:
        logger.warning(
            "%s is binding UDP %d, which is in the contested range %d-%d. "
            "Mission Planner's AutoConnect binds 14550 at startup and never "
            "releases it, 14551 is reserved for the STM32 bridge, and "
            "MavlinkWorker._connect_with_retry walks this whole range. "
            "SO_REUSEADDR means the bind will appear to succeed while datagrams "
            "reach only one socket. Use 14570 or another port outside the range.",
            what, port, CONTESTED_PORTS.start, CONTESTED_PORTS.stop - 1)
        return True
    return False


def _parse_host_port(text, default_host="127.0.0.1"):
    """'host:port' or bare 'port' -> (host, port)."""
    text = str(text).strip()
    if ":" in text:
        host, _, port = text.rpartition(":")
        return (host or default_host, int(port))
    return (default_host, int(text))
