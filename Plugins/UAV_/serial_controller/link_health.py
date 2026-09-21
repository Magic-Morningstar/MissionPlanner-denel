# serial_controller/link_health.py
#
# Turns the raw link figures SystemState already collects into the two
# 0-100 percentages the HEARTBEAT frame carries.
#
# This lives in its own file because it is the only genuinely *heuristic*
# code in the serial path. Everything else is a faithful translation of
# one representation into another; these two functions make a judgement
# call about what "healthy" means. Keeping that judgement in one place
# means it can be tuned against real flights without touching the
# protocol, the builder, or the firmware.
#
# Both functions are deliberately total: they return a number for any
# state, including one where nothing has ever been reported. They never
# raise, because they run inside the heartbeat thread and an exception
# there would stop the beat and make the panel think the PC had died.

import logging

logger = logging.getLogger(__name__)


# ── Tuning ────────────────────────────────────────────────────────────────────
# RSSI on a SiK-style RADIO_STATUS is a 0-255 figure, not dBm, and the
# usable range in practice is narrower than the full scale. These two
# bound the mapping to percent; readings at or below FLOOR count as 0 %,
# at or above CEILING as 100 %.
RSSI_FLOOR   = 50
RSSI_CEILING = 190

# Weight given to packet loss when RADIO_STATUS is available. Loss is
# the more meaningful signal of the two — a strong carrier that drops
# frames is worse than a weak one that doesn't — so it gets the larger
# share.
RSSI_WEIGHT = 0.4
LOSS_WEIGHT = 0.6


def _pct(value, lo, hi) -> int:
    """Linear map into 0-100, clamped at both ends."""
    if hi <= lo:
        return 0
    scaled = (float(value) - lo) * 100.0 / (hi - lo)
    return int(max(0.0, min(100.0, scaled)))


def telemetry_health(state) -> int:
    """Health of the radio hop — Herelink ground unit to air unit.

    Two sources, and which one is authoritative depends on what the link
    actually reports:

      RADIO_STATUS present   combine the weaker of the two RSSI ends with
                             the rolling packet-loss figure. The weaker
                             end is the one that matters; a strong
                             downlink doesn't help if the uplink is
                             marginal.
      RADIO_STATUS absent    fall back to packet loss alone. Plenty of
                             digital links (Herelink included, depending
                             on firmware) never emit msg 109, and
                             is_Radio_Status_Available is what
                             distinguishes "no reading" from "a genuine
                             zero".

    A link that is down entirely reports 0 rather than falling back to
    loss, since packet loss on a dead link is meaningless.
    """
    try:
        if not state.is_UAV_State_Connection_Available:
            return 0

        loss = float(state.get_UAV_Link_Packet_Loss or 0.0)
        loss_health = int(max(0.0, min(100.0, 100.0 - loss)))

        if not state.is_Radio_Status_Available:
            # No RSSI to work with — loss is the only real evidence.
            return loss_health

        rssi = state.get_UAV_Link_RSSI or 0
        remrssi = state.get_UAV_Link_Remote_RSSI or 0
        weakest = min(rssi, remrssi)
        rssi_health = _pct(weakest, RSSI_FLOOR, RSSI_CEILING)

        combined = (rssi_health * RSSI_WEIGHT) + (loss_health * LOSS_WEIGHT)
        return int(max(0.0, min(100.0, combined)))

    except Exception:
        logger.exception("telemetry_health: falling back to 0")
        return 0


def uav_health(state) -> int:
    """The vehicle's own reported health.

    Built from SYS_STATUS's comm drop rate, which is the autopilot's view
    of ITS OWN link — the FC-to-air-unit serial hop, not the radio hop
    above. The two are genuinely different segments of the chain and can
    fail independently, which is why both are reported rather than one
    combined number.

    Gated on the heartbeat being current: an autopilot that stopped
    talking has no health to report, and continuing to send its last
    known figure would be worse than sending zero.
    """
    try:
        if not state.is_UAV_State_Connection_Available:
            return 0

        # The link-state machine already decides whether heartbeats are
        # current; RED means they are not.
        link_state = getattr(state, "_UAV_LINK_STATE", "RED")
        if link_state == "RED":
            return 0

        drop = float(state._UAV_DROP_RATE_COMM or 0.0)
        health = int(max(0.0, min(100.0, 100.0 - drop)))

        # AMBER means heartbeats are late but not absent. Cap rather than
        # zero, so the panel can distinguish "degraded" from "gone".
        if link_state == "AMBER":
            health = min(health, 50)

        return health

    except Exception:
        logger.exception("uav_health: falling back to 0")
        return 0
