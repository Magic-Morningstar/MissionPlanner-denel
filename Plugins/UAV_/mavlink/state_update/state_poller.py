# mavlink/state_update/state_poller.py

from pymavlink import mavutil
from mavlink.mavlink_worker import MavlinkWorker
from utils.helper import *
import logging
logger = logging.getLogger(__name__)


class UAVStatePoller(MavlinkWorker):
    """
    Owns its own MAVLink state-poll connection. Connects itself via
    _open_connection(), then polls the message cache every interval — no
    blocking recv_match anywhere.

    Runs as a distinct MAVLink component (200/190) from
    FlightCommandSender (200/191). Both used to default to 255/0, which
    made them indistinguishable to the autopilot and the Herelink router:
    targeted replies went to whichever had transmitted most recently. In
    practice FlightCommandSender won, because it heartbeats at 1Hz and
    polls at 20Hz, so this poller quietly lost any ACK or PARAM_VALUE it
    ever asked for.
    """

    SOURCE_SYSTEM = 200
    SOURCE_COMPONENT = 190

    def __init__(self, connection_string, state, watchdog=None, interval=1.0):
        super().__init__(
            connection_string, state, watchdog, interval,
            source_system=self.SOURCE_SYSTEM,
            source_component=self.SOURCE_COMPONENT,
        )
        self.state_drone = None

    # ── Connection ────────────────────────────────────────────────────────────

    def _open_connection(self):
        """
        Opens the link and publishes it to shared state. Called both on
        initial connect (via MavlinkWorker._do_connect) and on every
        reconnect (via MavlinkWorker._loop) — so it must be safe to run
        more than once, and must not start a loop thread itself.
        """
        self.state_drone = self._connect_and_wait_for_heartbeat(label="state-poll")

        if self.state_drone is None:
            logger.warning("UAVStatePoller: connection cancelled or failed.")
            return False

        if self.is_cancelled():
            try:
                self.state_drone.close()
            except Exception:
                pass
            self.state_drone = None
            return False

        self._mav_connection = self.state_drone  # lets _loop() heartbeat generically
        self.state.UAV_HEARTBEAT = self.state_drone.messages.get('HEARTBEAT')

        # This worker is the only one that requests stream rates. Rates
        # are link-wide, not per-client, so if FlightCommandSender also
        # requested any of these the two would overwrite each other.
        self._request_message_interval(
            self.state_drone,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            interval_us=100000,     # 10 Hz
        )
        # VFR_HUD and MISSION_CURRENT are read below in _run_once but were
        # never requested — they were arriving only because ArduPilot's
        # default stream rates happened to include them. Ask explicitly
        # so the poller doesn't depend on the autopilot's SRx_ params.
        self._request_message_interval(
            self.state_drone,
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
            interval_us=200000,     # 5 Hz
        )
        self._request_message_interval(
            self.state_drone,
            mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT,
            interval_us=1000000,    # 1 Hz
        )

        self.state.update_UAV_State_Connection(True, self.state_drone)
        logger.info("UAVStatePoller connected.")
        return True

    def _do_disconnect(self):
        if self.state_drone:
            try:
                self.state_drone.close()
            except Exception:
                pass
            self.state_drone = None
        self._mav_connection = None
        self.state.update_UAV_State_Connection(False, None)
        logger.info("UAVStatePoller disconnected.")

    # ── Watchdog ──────────────────────────────────────────────────────────────

    def _pet_watchdog(self):
        if self.watchdog:
            self.watchdog.uavState_pet()

    def _start_watch(self):
        if self.watchdog:
            self.watchdog.watchStateUpdateThread()

    def _on_error(self, e):
        logger.error(f"UAVStatePoller error: {e}")
        self.state.update_UAV_State_Connection(False, None)

    # ── Poll loop ─────────────────────────────────────────────────────────────

    def _run_once(self):
        # Guard: _loop() can call us in the window where a teardown has
        # nulled the connection but the reconnect hasn't completed.
        if self.state_drone is None:
            return

        while self.state_drone.recv_match(blocking=False) is not None:
            pass

        heartbeat       = self.state_drone.messages.get('HEARTBEAT')
        position        = self.state_drone.messages.get('GLOBAL_POSITION_INT')
        current_mission = self.state_drone.messages.get('MISSION_CURRENT')
        vfr_hud         = self.state_drone.messages.get('VFR_HUD')

        if heartbeat is None:
            logger.debug("UAVStatePoller: no HEARTBEAT in cache yet — waiting...")
            return

        changed = False

        # ── Armed status ──────────────────────────────────────────────────────
        armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if armed != self.state.is_UAV_Armed:
            logger.info(
                f"[STATE] Armed: {self.state.is_UAV_Armed} -> {armed}"
            )
            self.state.update_UAV_Armed_Status(armed)
            changed = True

        # ── Flight mode ───────────────────────────────────────────────────────
        mode_mapping = self.state_drone.mode_mapping()
        if mode_mapping:
            id_to_name = {v: k for k, v in mode_mapping.items()}
            mode = id_to_name.get(heartbeat.custom_mode)
            if mode != self.state.get_UAV_Current_Mode:
                logger.info(
                    f"[STATE] Mode: {self.state.get_UAV_Current_Mode} -> {mode}"
                )
                self.state.update_UAV_Current_Mode(mode)
                changed = True

        # ── Altitude ──────────────────────────────────────────────────────────
        if position is not None:
            altitude = position.relative_alt / 1000.0
            if altitude != self.state.get_UAV_Current_Altitude:
                logger.info(
                    f"[STATE] Altitude: "
                    f"{self.state.get_UAV_Current_Altitude}m -> {altitude:.2f}m"
                )
                self.state.update_UAV_Altitude(altitude)
                changed = True
        else:
            logger.debug("Cant update ALTITUDE position is None")

        # ── Current mission item ──────────────────────────────────────────────
        if current_mission is not None:
            if current_mission.seq != self.state._UAV_CURRENT_ITEM:
                logger.info(
                    f"[STATE] Mission item: "
                    f"{self.state._UAV_CURRENT_ITEM} -> {current_mission.seq}"
                )
                self.state.update_UAV_Current_Item(current_mission.seq)
                changed = True
        else:
            logger.debug("Cant update Mission item, current_mission is None")

        # ── Ground speed ──────────────────────────────────────────────────────
        if vfr_hud is not None:
            if vfr_hud.groundspeed != self.state.get_UAV_Current_Ground_Speed:
                logger.info(
                    f"[STATE] Ground speed: "
                    f"{self.state.get_UAV_Current_Ground_Speed} -> "
                    f"{vfr_hud.groundspeed:.2f} m/s"
                )
                self.state.update_UAV_Ground_Speed(vfr_hud.groundspeed)
                changed = True

            # ── Air speed ─────────────────────────────────────────────────────
            if vfr_hud.airspeed != self.state.get_UAV_Current_Air_Speed:
                logger.info(
                    f"[STATE] Air speed: "
                    f"{self.state.get_UAV_Current_Air_Speed} -> "
                    f"{vfr_hud.airspeed:.2f} m/s"
                )
                self.state.update_UAV_Air_Speed(vfr_hud.airspeed)
                changed = True
        else:
            logger.debug("Cant update Ground speed and AIR SPEED, vfr_hud is None")

        if changed:
            self.state.set_UAV_State_Changed()