# mavlink/uav_commads/command_sender.py

import logging
import queue
import threading
from pymavlink import mavutil
from mavlink.mavlink_worker import MavlinkWorker
from mavlink.uav_commads.arms_commands import Arms
from mavlink.uav_commads.autotakeoff_commands import AutoTakeOff
from mavlink.uav_commads.mode_commands import ModeCommander
from mavlink.uav_commads.speed_commands import Speed_Controller
from mavlink.uav_commads.direction_commands import DirectionCommander, ManualController
from commands.intents import (
    ArmCommand, DisarmCommand, TakeoffCommand, ManualModeCommand, RTLCommand,
    EmergencyCommand,
)
from commands.registry import register_handler, dispatch

logger = logging.getLogger(__name__)


class UAVCommandSender(MavlinkWorker):

    def __init__(self, connection_string, state, command_bus, watchdog=None, interval=0.05):
        super().__init__(connection_string, state, watchdog, interval)
        self.command_drone       = None
        self.command_queue       = queue.Queue()
        self.command_bus         = command_bus
        self.takeoff_in_progress = False
        self._mav_lock           = threading.Lock()

        self.Manual     = None
        self.Director   = None
        self.ManualCtrl = None
        self._ato       = None

        self._last_pot_value = None
        self._last_turn_degrees = 0.0
        self._last_altitude_delta = 0.0

    # ── Connection ────────────────────────────────────────────────────────────

    def _do_connect(self):
        self.command_drone = self._connect_with_retry(label="command-sender")

        if self.is_cancelled():
            self.command_drone.close()
            self.command_drone = None
            return False

        self._heartbeat(self.command_drone)

        self.Manual     = ModeCommander(self.command_drone, self.state, self._mav_lock)
        self.Director   = DirectionCommander(self.command_drone, self.state, self._mav_lock)
        self.ManualCtrl = ManualController(self.command_drone, self.state, self._mav_lock)
        self._ato       = AutoTakeOff(self.command_drone, self._mav_lock)

        self.state.update_UAV_Command_Connection(True, self.command_drone)
        logger.info("UAVCommandSender connected.")
        self._start_loop()
        return True

    def _do_disconnect(self):
        if self.ManualCtrl:
            self.ManualCtrl.stop_streaming()
            self.ManualCtrl = None
        if self._ato:
            self._ato.cancel()
            self._ato = None
        if self.command_drone:
            try:
                self.command_drone.close()
            except Exception:
                pass
            self.command_drone = None
        self.Manual   = None
        self.Director = None
        self.state.update_UAV_Command_Connection(False, None)
        logger.info("UAVCommandSender disconnected.")

    def _pet_watchdog(self):
        if self.watchdog:
            self.watchdog.uavCommand_pet()

    def _start_watch(self):
        if self.watchdog:
            self.watchdog.watchCommandsThread()

    def _on_error(self, e):
        logger.error(f"UAVCommandSender loop error: {e}")
        self.state.update_UAV_Command_Connection(False, None)

    def _run_once(self):
        self._process_analog_inputs()
        try:
            cmd, args, kwargs = self.command_queue.get(timeout=self.interval)
            cmd(*args, **kwargs)
            self.command_queue.task_done()
        except queue.Empty:
            pass

    def enqueue(self, cmd, *args, **kwargs):
        self.command_queue.put((cmd, args, kwargs))

    # ── Inbound dispatch — drains the Command bus ───────────────────────────

    def drain_input_commands(self):
        while True:
            try:
                cmd = self.command_bus.get_nowait()
            except queue.Empty:
                break
            dispatch(cmd, self)   # was a hand-written if/elif chain — see commands/registry.py

    def _cancel_takeoff_if_active(self):
        if self._ato:
            self._ato.cancel()
        self.takeoff_in_progress = False

    # ── Continuous analog inputs — read as facts, not Commands ─────────────

    def _process_analog_inputs(self):
        current_mode = self.state.get_UAV_Current_Mode

        pot_value = self.state.get_Pot_Value
        if pot_value and pot_value != self._last_pot_value and current_mode in ("FBWA", "FBWB"):
            speed = self._pot_to_speed(pot_value)
            logger.debug(f"POT: {pot_value} -> {speed:.1f} m/s")
            self.set_airspeed(speed)
            self._last_pot_value = pot_value

        joystick_x = self.state.get_Joystick_X
        joystick_y = self.state.get_Joystick_Y

        if current_mode in ("FBWA", "FBWB"):
            if self.ManualCtrl and not self.ManualCtrl._streaming:
                if self.state.is_flying:
                    self.ManualCtrl.start_streaming()

        elif current_mode == "GUIDED":
            if self.ManualCtrl and self.ManualCtrl._streaming:
                self.ManualCtrl.stop_streaming()
                self.ManualCtrl.set_neutral()

            degrees = self._joystick_to_degrees(joystick_x)
            if degrees != 0 and abs(degrees - self._last_turn_degrees) > 2.0:
                if degrees > 0:
                    self.turn_right(abs(degrees))
                else:
                    self.turn_left(abs(degrees))
                self._last_turn_degrees = degrees

            alt_delta = self._joystick_to_altitude_delta(joystick_y)
            if alt_delta != 0 and abs(alt_delta - self._last_altitude_delta) > 1.0:
                current_alt = self.state.get_UAV_Current_Altitude or 0
                target_alt = max(5.0, current_alt + alt_delta)
                self.set_altitude(target_alt)
                self._last_altitude_delta = alt_delta

        else:
            if self.ManualCtrl and self.ManualCtrl._streaming:
                self.ManualCtrl.stop_streaming()
                self.ManualCtrl.set_neutral()

    # ── Public command API ────────────────────────────────────────────────────

    def arm(self):
        if not self.state.is_UAV_Armed:
            self.enqueue(Arms(self.command_drone, self.state, self._mav_lock).initiate_Arm)
        else:
            logger.info("Already armed — skipping")

    def disarm(self):
        self.enqueue(Arms(self.command_drone, self.state, self._mav_lock).initiate_Disarming)

    def set_fbwa(self):
        if self.Manual:
            self.enqueue(self.Manual.initiate_FBWA_Mode)

    def set_fbwb(self):
        if self.Manual:
            self.enqueue(self._switch_to_fbwb)

    def _switch_to_fbwb(self):
        mode_mapping = self.command_drone.mode_mapping()
        if mode_mapping and "FBWB" in mode_mapping:
            with self._mav_lock:
                self.command_drone.mav.set_mode_send(
                    self.command_drone.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_mapping["FBWB"]
                )
            logger.info("Switched to FBWB — joystick active.")
            if self.ManualCtrl:
                self.ManualCtrl.start_streaming()
        else:
            logger.warning("FBWB not available — falling back to FBWA.")
            self.Manual.initiate_FBWA_Mode()
            if self.ManualCtrl:
                self.ManualCtrl.start_streaming()

    def set_guided(self):
        if self.Manual:
            self.enqueue(self.Manual.initiate_Guided_Mode)

    def set_auto(self):
        if self.Manual:
            self.enqueue(self.Manual.initiate_Auto_Mode)

    def set_rtl(self):
        if self.Manual:
            self.enqueue(self.Manual.initiate_RTL)

    def set_airspeed(self, speed_ms):
        self.enqueue(Speed_Controller(self.command_drone, self.state, self._mav_lock).set_airspeed, speed_ms)

    def turn_left(self, degrees=30):
        if self.Director:
            self.enqueue(self.Director.turn_left, degrees)

    def turn_right(self, degrees=30):
        if self.Director:
            self.enqueue(self.Director.turn_right, degrees)

    def set_altitude(self, target_alt_m):
        if self.Director:
            self.enqueue(self.Director.set_altitude, target_alt_m)

    def start_takeoff(self):
        if self.takeoff_in_progress:
            logger.info("Takeoff already in progress — ignoring duplicate request.")
            return
        self.takeoff_in_progress = True
        threading.Thread(target=self._run_takeoff, daemon=True, name="TakeoffSequence").start()

    def _run_takeoff(self):
        try:
            self._ato.initiate_takeoff()
        finally:
            self.takeoff_in_progress = False

    def emergency_stop(self):
        if self.state.is_flying:
            logger.critical("EMERGENCY: vehicle is flying — triggering RTL")
            self.set_rtl()
        else:
            logger.critical("EMERGENCY: vehicle on ground — disarming")
            self.disarm()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _pot_to_speed(self, pot_value):
        min_speed = getattr(self.state, 'MIN_SPEED', 5.0)
        max_speed = getattr(self.state, 'MAX_SPEED', 25.0)
        return min_speed + ((pot_value - 1) / 99.0) * (max_speed - min_speed)

    def _joystick_to_degrees(self, joystick_x):
        DEAD_LOW  = int(0.45 * 4095)
        DEAD_HIGH = int(0.55 * 4095)
        if DEAD_LOW <= joystick_x <= DEAD_HIGH:
            return 0
        if joystick_x < DEAD_LOW:
            return -(((DEAD_LOW - joystick_x) / DEAD_LOW) * 45.0)
        return +(((joystick_x - DEAD_HIGH) / (4095 - DEAD_HIGH)) * 45.0)

    def _joystick_to_altitude_delta(self, joystick_y):
        DEAD_LOW  = int(0.45 * 4095)
        DEAD_HIGH = int(0.55 * 4095)
        if DEAD_LOW <= joystick_y <= DEAD_HIGH:
            return 0
        if joystick_y < DEAD_LOW:
            return -(((DEAD_LOW - joystick_y) / DEAD_LOW) * 20.0)
        return +(((joystick_y - DEAD_HIGH) / (4095 - DEAD_HIGH)) * 20.0)
    
    def _joystick2_y_to_speed(self, raw_adc):
        min_speed = getattr(self.state, 'MIN_SPEED', 5.0)
        max_speed = getattr(self.state, 'MAX_SPEED', 25.0)
        ratio = raw_adc / 4095.0
        return min_speed + ratio * (max_speed - min_speed)


# ── Registered handlers — this list IS the dispatch table ──────────────────
# Adding a command: write one function here, decorate it. process_command
# (now just dispatch(), called from drain_input_commands above) never
# changes, and neither does UAVCommandSender.

@register_handler(ArmCommand)
def _handle_arm(sender, cmd):
    logger.info("ArmCommand received")
    sender.arm()


@register_handler(DisarmCommand)
def _handle_disarm(sender, cmd):
    logger.info("DisarmCommand received")
    sender.disarm()


@register_handler(TakeoffCommand)
def _handle_takeoff(sender, cmd):
    logger.info("TakeoffCommand received")
    sender.start_takeoff()


@register_handler(ManualModeCommand)
def _handle_manual(sender, cmd):
    logger.info("ManualModeCommand received")
    sender._cancel_takeoff_if_active()
    sender.set_fbwb()


@register_handler(RTLCommand)
def _handle_rtl(sender, cmd):
    logger.info("RTLCommand received")
    sender._cancel_takeoff_if_active()
    if sender.ManualCtrl:
        sender.ManualCtrl.stop_streaming()
    sender.set_rtl()


@register_handler(EmergencyCommand)
def _handle_emergency(sender, cmd):
    logger.warning("EmergencyCommand received")
    sender._cancel_takeoff_if_active()
    sender.emergency_stop()
