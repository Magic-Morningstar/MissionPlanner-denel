# mavlink/state_update/state_poller.py

from pymavlink import mavutil
from mavlink.mavlink_worker import MavlinkWorker
from utils.helper import *


class UAVStatePoller(MavlinkWorker):
    """
    Owns its own MAVLink state-poll connection.
    Connects itself via _do_connect, then polls the
    message cache every interval — no blocking recv_match anywhere.
    """

    def __init__(self, connection_string, state, watchdog=None, interval=1.0):
        super().__init__(connection_string, state, watchdog, interval)
        self.state_drone = None

    # ── Connection ────────────────────────────────────────────────────────────

    def _do_connect(self):
        self.state_drone = self._connect_with_retry(label="state-poll")

        if self.state_drone is None:
            system_Print("UAVStatePoller: connection cancelled or failed.")
            return False

        if self.is_cancelled():
            self.state_drone.close()
            self.state_drone = None
            return False

        msg = self._heartbeat(self.state_drone)
        self.state.UAV_HEARTBEAT = msg

        self._request_message_interval(
            self.state_drone,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            interval_us=100000
        )

        self.state.update_UAV_State_Connection(True, self.state_drone)
        system_Print("UAVStatePoller connected.")
        self._start_loop()
        return True

    def _do_disconnect(self):
        if self.state_drone:
            try:
                self.state_drone.close()
            except Exception:
                pass
            self.state_drone = None
        self.state.update_UAV_State_Connection(False, None)
        system_Print("UAVStatePoller disconnected.")

    # ── Watchdog ──────────────────────────────────────────────────────────────

    def _pet_watchdog(self):
        if self.watchdog:
            self.watchdog.uavState_pet()

    def _start_watch(self):
        if self.watchdog:
            self.watchdog.watchStateUpdateThread()

    def _on_error(self, e):
        system_Print(f"UAVStatePoller error: {e}")
        self.state.update_UAV_State_Connection(False, None)

    # ── Poll loop ─────────────────────────────────────────────────────────────

    def _run_once(self):
        while self.state_drone.recv_match(blocking=False) is not None:
            pass
        heartbeat       = self.state_drone.messages.get('HEARTBEAT')
        position        = self.state_drone.messages.get('GLOBAL_POSITION_INT')
        current_mission = self.state_drone.messages.get('MISSION_CURRENT')
        vfr_hud         = self.state_drone.messages.get('VFR_HUD')

        if heartbeat is None:
            system_Print("UAVStatePoller: no HEARTBEAT in cache yet — waiting...")
            return

        changed = False

        # ── Armed status ──────────────────────────────────────────────────────
        armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if armed != self.state.is_UAV_Armed:
            system_Print(
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
                system_Print(
                    f"[STATE] Mode: {self.state.get_UAV_Current_Mode} -> {mode}"
                )
                self.state.update_UAV_Current_Mode(mode)
                changed = True

        # ── Altitude ──────────────────────────────────────────────────────────
        if position is not None:
            altitude = position.relative_alt / 1000.0
            if altitude != self.state.get_UAV_Current_Altitude:
                system_Print(
                    f"[STATE] Altitude: "
                    f"{self.state.get_UAV_Current_Altitude}m -> {altitude:.2f}m"
                )
                self.state.update_UAV_Altitude(altitude)
                changed = True
        else:
            system_Print("Cant update ALTITUDE position is None")

        # ── Current mission item ──────────────────────────────────────────────
        if current_mission is not None:
            if current_mission.seq != self.state._UAV_CURRENT_ITEM:
                system_Print(
                    f"[STATE] Mission item: "
                    f"{self.state._UAV_CURRENT_ITEM} -> {current_mission.seq}"
                )
                self.state.update_UAV_Current_Item(current_mission.seq)
                changed = True
        else:
            system_Print("Cant update Mission item, current_mission is None")

        # ── Ground speed ──────────────────────────────────────────────────────
        if vfr_hud is not None:
            if vfr_hud.groundspeed != self.state.get_UAV_Current_Ground_Speed:
                system_Print(
                    f"[STATE] Ground speed: "
                    f"{self.state.get_UAV_Current_Ground_Speed} -> "
                    f"{vfr_hud.groundspeed:.2f} m/s"
                )
                self.state.update_UAV_Ground_Speed(vfr_hud.groundspeed)
                changed = True
        

            # ── Air speed ─────────────────────────────────────────────────────
            if vfr_hud.airspeed != self.state.get_UAV_Current_Air_Speed:
                system_Print(
                    f"[STATE] Air speed: "
                    f"{self.state.get_UAV_Current_Air_Speed} -> "
                    f"{vfr_hud.airspeed:.2f} m/s"
                )
                self.state.update_UAV_Air_Speed(vfr_hud.airspeed)
                changed = True
        else:
            system_Print("Cant update Ground speed and AIR SPEED, vfr_hud is None")

        if changed:
            self.state.set_UAV_State_Changed()