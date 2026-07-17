# mavlink/uav_commads/discrete_command_sender.py

import logging
import queue
import threading
from pymavlink import mavutil
from mavlink.mavlink_worker import MavlinkWorker
from mavlink.uav_commads.commands.arms_commands import Arms
from mavlink.uav_commads.commands.autotakeoff_commands import AutoTakeOff
from mavlink.uav_commads.commands.mode_commands import ModeCommander
from mavlink.uav_commads.commands.speed_commands import Speed_Controller
from mavlink.uav_commads.commands.direction_commands import DirectionCommander, ManualController
from commands.intents import (
    ArmCommand, DisarmCommand, TakeoffCommand, ManualModeCommand,
    RTLCommand, EmergencyCommand,
)
from commands.registry import register_handler, dispatch

logger = logging.getLogger(__name__)


class UAVCommandSender(MavlinkWorker):
    """
    Handles discrete button commands arriving via command_bus.
    Owns the MAVLink connection, command queue, and all
    sub-controllers (Manual, Director, ManualCtrl, AutoTakeOff).

    Analog input handling is delegated to AnalogInputHandler,
    which is instantiated here and called each _run_once().
    """

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
        self._analog    = None   # AnalogInputHandler — set after connection

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

        # Analog handler wired to this sender so it can call
        # set_airspeed / turn_left / set_altitude etc.
        from mavlink.uav_commads.analog_input_handler import AnalogInputHandler
        self._analog = AnalogInputHandler(self.state, self.ManualCtrl, self)

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
        self._analog  = None
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

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _run_once(self):
        # 1. Drain discrete commands from the bus
        self.drain_input_commands()

        # 2. Process continuous analog inputs
        if self._analog:
            self._analog.process()

        # 3. Execute one MAVLink command from the queue
        try:
            cmd, args, kwargs = self.command_queue.get(timeout=self.interval)
            cmd(*args, **kwargs)
            self.command_queue.task_done()
        except queue.Empty:
            pass

    def enqueue(self, cmd, *args, **kwargs):
        self.command_queue.put((cmd, args, kwargs))

    # ── Discrete command bus ──────────────────────────────────────────────────

    def drain_input_commands(self):
        while True:
            try:
                cmd = self.command_bus.get_nowait()
            except queue.Empty:
                break
            dispatch(cmd, self)

    def _cancel_takeoff_if_active(self):
        if self._ato:
            self._ato.cancel()
        self.takeoff_in_progress = False

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
        self.enqueue(
            Speed_Controller(self.command_drone, self.state, self._mav_lock).set_airspeed,
            speed_ms
        )

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
        threading.Thread(
            target=self._run_takeoff, daemon=True, name="TakeoffSequence"
        ).start()

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


# ── Registered handlers ───────────────────────────────────────────────────────

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

@register_handler(AutoModeCommand)
def _handle_auto(sender, cmd):
    logger.info("AutoModeCommand received")
    sender.set_auto()


@register_handler(LandCommand)
def _handle_land(sender, cmd):
    logger.info("LandCommand received")



@register_handler(SpeedUpCommand)
def _handle_speedup(sender, cmd):
    logger.info("SpeedUpCommand received")
    

@register_handler(SpeedDownCommand)
def _handle_speeddown(sender, cmd):
    logger.info("SpeedDownCommand received")


@register_handler(ZoomInCommand)
def _handle_zoomin(sender, cmd):
    logger.info("CameraZoomInCommand received")


@register_handler(ZoomOutCommand)
def _handle_zoomout(sender, cmd):
    logger.info("CameraZoomOutCommand received")


@register_handler(WideInCommand)
def _handle_wideout(sender, cmd):
    logger.info("CameraWideOutCommand received")

@register_handler(WideOutCommand)
def _handle_widein(sender, cmd):
    logger.info("CameraWideInCommand received")

