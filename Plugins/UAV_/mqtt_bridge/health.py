# mqtt_bridge/health.py
"""Aircraft health report: what the XBLink knows about itself, as JSON.

Published by the aircraft bridge, retained, on denel/uav/<vid>/status/health
every HEALTH_INTERVAL seconds. Built because status/link only says
online/offline: on 2026-11-12 the aircraft showed "online" for an hour while
publishing no Cube data at all, and nothing outside the unit could tell.

Design rules:

- Standard library only, and every probe is optional. A probe that is
  unavailable (not Linux, no ModemManager, no wwan0) reports None and never
  stops the report. A broken health report must never affect the link itself.
- The modem is only READ, through ModemManager (`mmcli -K`), which is built to
  share the modem. Never raw AT commands here: the same modem carries LTE.
- build_health() is pure, so the warning logic is unit-tested without a Pi.
"""

import os
import socket
import subprocess
import time

# Thresholds for warnings. Signal quality is ModemManager's 0-100 %.
WEAK_SIGNAL_PCT = 25
HOT_CPU_C = 75.0


def _run(args, timeout=5.0):
    """stdout of a command, or None if it is missing, fails or times out."""
    try:
        out = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout if out.returncode == 0 else None


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def cpu_temp_c():
    raw = _read("/sys/class/thermal/thermal_zone0/temp")
    try:
        return round(int(raw) / 1000.0, 1)
    except (TypeError, ValueError):
        return None


def uptime_s():
    raw = _read("/proc/uptime")
    try:
        return int(float(raw.split()[0]))
    except (AttributeError, ValueError, IndexError):
        return None


def interface_ipv4(name):
    """First IPv4 address of an interface, e.g. wwan0, or None."""
    out = _run(["ip", "-4", "-o", "addr", "show", "dev", name], timeout=2.0)
    if not out:
        return None
    for token in out.split():
        if "/" in token and token[0].isdigit():
            return token.split("/")[0]
    return None


def clock_synced():
    out = _run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"], timeout=2.0)
    if out is None:
        return None
    return out.strip() == "yes"


def parse_mmcli_keyvalue(text):
    """Pick the useful fields out of `mmcli -m any -K` output. Pure."""
    fields = {}
    for line in (text or "").splitlines():
        key, sep, value = line.partition(":")
        if sep:
            fields[key.strip()] = value.strip()

    def get(key):
        v = fields.get(key)
        return None if v in (None, "", "--") else v

    signal = get("modem.generic.signal-quality.value")
    try:
        signal = int(signal) if signal is not None else None
    except ValueError:
        signal = None
    return {
        "state": get("modem.generic.state"),
        "signal_pct": signal,
        "tech": get("modem.generic.access-technologies.value[1]"),
        "operator": get("modem.3gpp.operator-name"),
        "registration": get("modem.3gpp.registration-state"),
    }


def modem_status():
    """Read-only modem summary from ModemManager, or None if unavailable."""
    out = _run(["mmcli", "-m", "any", "-K"], timeout=5.0)
    return parse_mmcli_keyvalue(out) if out else None


def probe_system(lte_iface="wwan0", read_modem=True):
    """Everything the unit can tell about itself. Never raises."""
    return {
        "host": socket.gethostname(),
        "uptime_s": uptime_s(),
        "cpu_temp_c": cpu_temp_c(),
        "clock_synced": clock_synced(),
        "lte": {
            "iface": lte_iface,
            "ipv4": interface_ipv4(lte_iface) if os.name != "nt" else None,
            "modem": modem_status() if read_modem else None,
        },
    }


def build_health(system, link, now=None):
    """The health message. Pure.

    `system` comes from probe_system(); `link` is a dict the bridge fills in:
    mqtt_up, fc_open, fc_frames_per_s, cmd_frames_per_s, wakeup_repairs (the
    increase since the last report), mqtt_disconnects, publish_failures,
    dropped_stale. Warnings are short codes a display or LED can act on.
    """
    warnings = []
    if not link.get("fc_open"):
        warnings.append("fc_port_closed")
    elif not link.get("fc_frames_per_s"):
        warnings.append("no_cube_data")
    modem = (system.get("lte") or {}).get("modem") or {}
    signal = modem.get("signal_pct")
    if signal is not None and signal < WEAK_SIGNAL_PCT:
        warnings.append("weak_signal")
    if (system.get("lte") or {}).get("ipv4") is None and os.name != "nt":
        warnings.append("no_lte_address")
    if system.get("clock_synced") is False:
        warnings.append("clock_not_synced")
    temp = system.get("cpu_temp_c")
    if temp is not None and temp >= HOT_CPU_C:
        warnings.append("cpu_hot")
    if link.get("wakeup_repairs"):
        warnings.append("mqtt_wakeup_repaired")

    return {
        "ts": int(now if now is not None else time.time()),
        "ok": not warnings,
        "warnings": warnings,
        "link": link,
        **system,
    }
