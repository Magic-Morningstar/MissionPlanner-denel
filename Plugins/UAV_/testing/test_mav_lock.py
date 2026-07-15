"""
testing/test_mav_lock.py

Doesn't reason about the concurrency fix — runs it under real thread
contention and detects overlapping writes directly.

FakeMav simulates a slow write (a few ms) and flags it as a violation
if a second thread starts writing while the first hasn't finished —
that's exactly what an interleaved/corrupted MAVLink frame looks like
on a real socket.

Two scenarios:
  1. SHARED lock  (the fix)         -> expect zero violations
  2. SEPARATE locks (the old bug)   -> expect violations, to prove this
     test would actually have caught the original bug, not just pass
     trivially.
"""

import sys
import os
import time
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mavlink.uav_commads.arms_commands import Arms
from mavlink.uav_commads.mode_commands import ModeCommander
from mavlink.uav_commads.direction_commands import ManualController
from utils.helper import system_Print
from logging_config import setup_logging
import logging

setup_logging(console_level=logging.WARNING, log_file="test_mav_lock.log")  # quiet console, full detail in the log file


class FakeMav:
    def __init__(self, violations):
        self._busy = False
        self._violations = violations
        self.calls = 0

    def _simulate_write(self, name):
        if self._busy:
            self._violations.append(f"CONCURRENT ACCESS during {name}")
        self._busy = True
        time.sleep(0.005)   # wide enough window that a real race gets caught
        self._busy = False
        self.calls += 1

    def command_long_send(self, *a, **kw):
        self._simulate_write("command_long_send")

    def manual_control_send(self, *a, **kw):
        self._simulate_write("manual_control_send")

    def set_mode_send(self, *a, **kw):
        self._simulate_write("set_mode_send")


class FakeDrone:
    def __init__(self, violations):
        self.mav = FakeMav(violations)
        self.target_system = 1
        self.target_component = 1

    def mode_mapping(self):
        return {"FBWA": 5, "FBWB": 6, "GUIDED": 4, "AUTO": 3, "QLOITER": 11}

    def recv_match(self, *a, **kw):
        return None


class FakeState:
    def __init__(self):
        self._armed = False
        self.MIN_SPEED, self.MAX_SPEED = 5.0, 25.0

    @property
    def is_UAV_Command_Connection_Available(self): return True
    @property
    def is_UAV_Armed(self): return self._armed
    @property
    def is_flying(self): return False
    @property
    def get_UAV_Current_Mode(self): return "FBWB"
    @property
    def get_Joystick_X(self): return 3000   # deflected, away from deadzone
    @property
    def get_Joystick_Y(self): return 3000

    def update_UAV_Armed_Status(self, armed=False):
        self._armed = armed


def run_scenario(shared_lock: bool, duration=1.5):
    violations = []
    drone = FakeDrone(violations)
    state = FakeState()

    lock_a = threading.Lock()
    lock_b = lock_a if shared_lock else threading.Lock()

    manual_ctrl = ManualController(drone, state, lock_b)
    manual_ctrl.start_streaming()   # its own thread, ~10Hz manual_control_send

    stop = threading.Event()

    def hammer_commands():
        # Main-thread-equivalent: fires a burst of unrelated MAVLink
        # writes (arm/disarm/mode switch) as fast as possible, exactly
        # the kind of contention that exposed the original bug.
        while not stop.is_set():
            Arms(drone, state, lock_a)._send_command(1)  # raw command, bypass guards for stress purposes
            ModeCommander(drone, state, lock_a)._set_mode("FBWA")

    hammer_thread = threading.Thread(target=hammer_commands, daemon=True)
    hammer_thread.start()

    time.sleep(duration)
    stop.set()
    manual_ctrl.stop_streaming()
    hammer_thread.join(timeout=1)

    return violations, drone.mav.calls


def main():
    system_Print("Scenario 1: SHARED lock (the actual fix)")
    violations, calls = run_scenario(shared_lock=True)
    system_Print(f"  {calls} total writes, {len(violations)} violations")
    assert len(violations) == 0, f"FAIL: shared lock still had violations: {violations[:3]}"
    system_Print("  PASS — zero overlapping writes under contention.")

    print()

    system_Print("Scenario 2: SEPARATE locks (reproducing the old bug, to prove this test is meaningful)")
    violations, calls = run_scenario(shared_lock=False)
    system_Print(f"  {calls} total writes, {len(violations)} violations")
    assert len(violations) > 0, "Test is not meaningful — even separate locks produced no violations"
    system_Print(f"  PASS — confirmed this test WOULD have caught the original bug ({len(violations)} violations detected).")


if __name__ == "__main__":
    main()
