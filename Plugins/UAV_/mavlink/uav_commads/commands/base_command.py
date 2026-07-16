# mavlink/uav_commads/base_command.py

from pymavlink import mavutil
import logging
logger = logging.getLogger(__name__)
import threading
import time


class BaseCommand:
    """
    Shared foundation for all MAVLink command classes.

    `lock` MUST be the same threading.Lock instance shared by every
    object that writes to `command_drone.mav` — including ManualController
    and AutoTakeOff, which don't extend this class but write to the same
    connection from their own threads. This is what fixes the
    "two threads writing to the same socket at once" bug: every write,
    from every thread, funnels through this one lock.
    """

    def __init__(self, command_drone, state, lock: threading.Lock):
        self.command_drone = command_drone
        self.state = state
        self.lock = lock

    # ── Precondition guards ───────────────────────────────────────────────────

    def _requires_connection(self):
        if not self.state.is_UAV_Command_Connection_Available:
            logger.warning(f"{self.__class__.__name__}: blocked — no MAVLink command connection.")
            return False
        return True

    def _requires_armed(self):
        if not self.state.is_UAV_Armed:
            logger.warning(f"{self.__class__.__name__}: blocked — vehicle is not armed.")
            return False
        return True

    def _requires_disarmed(self):
        if self.state.is_UAV_Armed:
            logger.warning(f"{self.__class__.__name__}: blocked — vehicle is already armed.")
            return False
        return True

    def _requires_not_flying(self):
        if self.state.is_flying:
            logger.warning(f"{self.__class__.__name__}: blocked — vehicle is currently flying.")
            return False
        return True

    def _requires_flying(self):
        if not self.state.is_flying:
            logger.warning(f"{self.__class__.__name__}: blocked — vehicle is not flying.")
            return False
        return True

    def _requires_mode(self, mode_name):
        if self.state.get_UAV_Current_Mode != mode_name:
            system_Print(
                f"{self.__class__.__name__}: blocked — "
                f"mode is {self.state.get_UAV_Current_Mode}, expected {mode_name}."
            )
            return False
        return True

    def _requires_mode_in(self, *mode_names):
        """Like _requires_mode but for commands valid in more than one
        mode (e.g. speed control, valid in both FBWA and FBWB)."""
        if self.state.get_UAV_Current_Mode not in mode_names:
            system_Print(
                f"{self.__class__.__name__}: blocked — "
                f"mode is {self.state.get_UAV_Current_Mode}, expected one of {mode_names}."
            )
            return False
        return True

    def _check(self, *guards):
        for guard in guards:
            if not guard():
                return False
        return True

    # ── Shared MAVLink helpers — all writes go through self.lock ───────────

    def _send_command(self, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        with self.lock:
            self.command_drone.mav.command_long_send(
                self.command_drone.target_system,
                self.command_drone.target_component,
                command,
                0,
                p1, p2, p3, p4, p5, p6, p7
            )

    def _set_mode(self, mode_name):
        mode_mapping = self.command_drone.mode_mapping()
        if mode_mapping is None:
            logger.warning(f"{self.__class__.__name__}: could not retrieve mode mapping.")
            return False
        if mode_name not in mode_mapping:
            logger.warning(f"{self.__class__.__name__}: {mode_name} not available on this vehicle.")
            return False
        with self.lock:
            self.command_drone.mav.set_mode_send(
                self.command_drone.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_mapping[mode_name]
            )
        logger.info(f"{self.__class__.__name__}: switched to {mode_name}.")
        return True

    def _wait_for_ack(self, command, timeout=3):
        """Read-only — not lock-protected. Concurrent reads alongside
        writes from other threads are fine on a full-duplex TCP socket;
        it's concurrent WRITES that corrupt the stream."""
        start = time.time()
        while time.time() - start < timeout:
            msg = self.command_drone.recv_match(
                type="COMMAND_ACK", blocking=True, timeout=timeout
            )
            if msg and msg.command == command:
                accepted = msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
                system_Print(
                    f"{self.__class__.__name__}: ACK "
                    f"{'ACCEPTED' if accepted else 'REJECTED'}"
                )
                return accepted
        logger.warning(f"{self.__class__.__name__}: no ACK within {timeout}s.")
        return False
