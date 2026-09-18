# mavlink/uav_commads/autotakeoff_commands.py

from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect
import logging
logger = logging.getLogger(__name__)
import time
import threading


class AutoTakeOff:
    """
    Fixed-altitude VTOL takeoff, then hold. 3-item mission: home
    placeholder, NAV_VTOL_TAKEOFF, NAV_LOITER_UNLIM. The loiter item is
    required — without it, ArduPlane's documented behavior is to
    auto-RTL the instant the takeoff item (the last item) completes,
    since a mission with no loiter/landing at the end is considered
    finished. Confirmed this was happening: vehicle went straight to
    RTL right after climbing, matching this exact documented behavior.
    """

    TARGET_ALTITUDE_M = 5.0

    def __init__(self, drone, lock: threading.Lock):
        self.drone = drone
        self.lock = lock
        self._cancel_flag = threading.Event()

    def initiate_takeoff(self, timeout=90, poll_interval=0.2):
        self._cancel_flag.clear()

        logger.info("=" * 40)
        logger.info(f"Initiating VTOL takeoff to {self.TARGET_ALTITUDE_M}m...")

        if not self._upload_takeoff_mission(self.TARGET_ALTITUDE_M):
            return False

        if not self._start_auto():
            return False

        completed = self._wait_for_mission_item_complete(timeout, poll_interval)

        if self._cancel_flag.is_set():
            logger.warning("Takeoff cancelled during climb.")
            return False

        if not completed:
            logger.error(f"Takeoff timed out after {timeout}s.")
            return False

        logger.info("Takeoff complete — now loitering at altitude.")
        logger.info("=" * 40)
        return True

    def cancel(self):
        logger.warning("AutoTakeOff: cancel requested.")
        self._cancel_flag.set()

    def _upload_takeoff_mission(self, altitude_m, timeout=5):
        with self.lock:
            self.drone.mav.mission_count_send(
                self.drone.target_system,
                self.drone.target_component,
                3,   # was 2 — home, takeoff, loiter
                dialect.MAV_MISSION_TYPE_MISSION,
            )

        for _ in range(3):
            req = self.drone.recv_match(
                type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'],
                blocking=True, timeout=timeout
            )
            if req is None:
                logger.error("No MISSION_REQUEST received — upload failed.")
                return False

            if req.seq == 0:
                item = dialect.MAVLink_mission_item_int_message(
                    target_system=self.drone.target_system,
                    target_component=self.drone.target_component,
                    seq=0,
                    frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    current=0, autocontinue=1,
                    param1=0, param2=0, param3=0, param4=0,
                    x=0, y=0, z=0,
                    mission_type=dialect.MAV_MISSION_TYPE_MISSION,
                )
            elif req.seq == 1:
                item = dialect.MAVLink_mission_item_int_message(
                    target_system=self.drone.target_system,
                    target_component=self.drone.target_component,
                    seq=1,
                    frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    command=mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
                    current=0, autocontinue=1,
                    param1=0, param2=0, param3=0, param4=0,
                    x=0, y=0, z=altitude_m,
                    mission_type=dialect.MAV_MISSION_TYPE_MISSION,
                )
            else:  # seq == 2 — NEW: unlimited loiter, prevents auto-RTL
                item = dialect.MAVLink_mission_item_int_message(
                    target_system=self.drone.target_system,
                    target_component=self.drone.target_component,
                    seq=2,
                    frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    command=mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                    current=0, autocontinue=1,
                    param1=0, param2=0, param3=0, param4=0,
                    x=0, y=0, z=altitude_m,
                    mission_type=dialect.MAV_MISSION_TYPE_MISSION,
                )

            with self.lock:
                self.drone.mav.send(item)

        ack = self.drone.recv_match(type='MISSION_ACK', blocking=True, timeout=timeout)
        if ack is None or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            logger.error(f"Mission upload not accepted (ack={ack}).")
            return False

        logger.info(f"Takeoff+loiter mission uploaded ({altitude_m}m) and accepted.")
        return True

    def _start_auto(self):
        logger.info("Switching to AUTO mode...")
        mode_mapping = self.drone.mode_mapping()
        if mode_mapping is None or "AUTO" not in mode_mapping:
            logger.error("AUTO mode not available.")
            return False
        with self.lock:
            self.drone.mav.set_mode_send(
                self.drone.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_mapping["AUTO"]
            )
        logger.info("AUTO mode set — takeoff beginning.")
        return True

    def _wait_for_mission_item_complete(self, timeout=90, poll_interval=0.2):
        logger.info(f"Waiting for takeoff item (timeout={timeout}s)...")
        start = time.time()

        while time.time() - start < timeout:
            if self._cancel_flag.is_set():
                return False

            current_item = self.get_current_mission_item()
            elapsed = time.time() - start

            if current_item is not None:
                logger.info(f"  [{elapsed:.1f}s] MISSION_CURRENT seq: {current_item}")
                if current_item > 1:   # advanced onto the loiter item — takeoff climb is done
                    logger.info(f"Takeoff item complete — now on seq {current_item} (loiter).")
                    return True

            time.sleep(poll_interval)

        return False

    def get_current_mission_item(self):
        msg = self.drone.recv_match(type='MISSION_CURRENT', blocking=True, timeout=1)
        return msg.seq if msg else None

    def get_altitude(self):
        msg = self.drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg is None:
            return None
        return msg.relative_alt / 1000.0