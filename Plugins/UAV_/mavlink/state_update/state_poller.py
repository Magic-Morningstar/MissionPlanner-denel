# mavlink/state_update/state_poller.py

from pymavlink import mavutil
from mavlink.mavlink_worker import MavlinkWorker
import logging
logger = logging.getLogger(__name__)


class UAVStatePoller(MavlinkWorker):

    def __init__(self, connection_string, state, watchdog=None, interval=1.0):
        super().__init__(connection_string, state, watchdog, interval)
        self.state_drone = None

    def _do_connect(self):
        self.state_drone = self._connect_with_retry(label="state-poll")

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
        logger.info("UAVStatePoller connected.")
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
        logger.info("UAVStatePoller disconnected.")

    def _pet_watchdog(self):
        if self.watchdog:
            self.watchdog.uavState_pet()

    def _start_watch(self):
        if self.watchdog:
            self.watchdog.watchStateUpdateThread()

    def _on_error(self, e):
        self.state.update_UAV_State_Connection(False, None)

    def _run_once(self):
        heartbeat       = self.state_drone.messages.get('HEARTBEAT')
        position        = self.state_drone.messages.get('GLOBAL_POSITION_INT')
        current_mission = self.state_drone.messages.get('MISSION_CURRENT')
        vfr_hud         = self.state_drone.messages.get('VFR_HUD')

        if heartbeat is None:
            logger.info("No HEARTBEAT in cache yet — vehicle not fully connected")
            return

        changed = False

        # NOTE: the leftover `time.sleep(2)` that used to be here is gone.
        # It was blocking this thread's loop for 2s on every single tick —
        # meaning every fact below (armed, mode, altitude, mission item,
        # speed) was delayed by at least 2s, on top of the 1s poll
        # interval. This is the exact class of bug the message-cache
        # design was originally built to avoid.
        armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if armed != self.state.is_UAV_Armed:
            logger.info(f"Armed: {self.state.is_UAV_Armed} -> {armed}")
            self.state.update_UAV_Armed_Status(armed)
            changed = True

        mode_mapping = self.state_drone.mode_mapping()
        if mode_mapping:
            id_to_name = {v: k for k, v in mode_mapping.items()}
            mode = id_to_name.get(heartbeat.custom_mode)
            if mode != self.state.get_UAV_Current_Mode:
                logger.info(f"Mode: {self.state.get_UAV_Current_Mode} -> {mode}")
                self.state.update_UAV_Current_Mode(mode)
                changed = True

        if position is not None:
            altitude = position.relative_alt / 1000.0
            if altitude != self.state.get_UAV_Current_Altitude:
                self.state.update_UAV_Altitude(altitude)
                changed = True

        if current_mission is not None:
            if current_mission.seq != self.state._UAV_CURRENT_ITEM:
                logger.info(f"Mission item: {self.state._UAV_CURRENT_ITEM} -> {current_mission.seq}")
                self.state.update_UAV_Current_Item(current_mission.seq)
                changed = True

        if vfr_hud is not None:
            if vfr_hud.groundspeed != self.state.get_UAV_Current_Ground_Speed:
                self.state.update_UAV_Ground_Speed(vfr_hud.groundspeed)
                changed = True
            if vfr_hud.airspeed != self.state.get_UAV_Current_Air_Speed:
                self.state.update_UAV_Air_Speed(vfr_hud.airspeed)
                changed = True

        if changed:
            self.state.set_UAV_State_Changed()
