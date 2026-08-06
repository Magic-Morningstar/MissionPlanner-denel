# mavlink/Flight_controller_commands/flight_command_sender.py

import logging
import queue
import threading
from pymavlink import mavutil
from mavlink.mavlink_worker import MavlinkWorker
from mavlink.Flight_controller_commands.commands.arms_commands import Arms
from mavlink.Flight_controller_commands.commands.autotakeoff_commands import AutoTakeOff
from mavlink.Flight_controller_commands.commands.mode_commands import ModeCommander
from mavlink.Flight_controller_commands.commands.speed_commands import Speed_Controller
from mavlink.Flight_controller_commands.commands.direction_commands import DirectionCommander, ManualController
from commands.intents import *
from commands.registry import register_handler, dispatch
logger = logging.getLogger(__name__)


class FlightCommandSender(MavlinkWorker):
    """
    Arm/disarm, flight mode, speed, heading, manual-control streaming,
    autotakeoff. Owns the MAVLink connection (command_drone + _mav_lock)
    and hands both to `payload_sender` right after connecting — see
    PayloadCommandSender.attach(). This is the only one of the two
    dispatchers that actually opens a socket; PayloadCommandSender has no
    connection lifecycle of its own.

    payload_sender's queues are drained from THIS class's _run_once(),
    same background loop/interval as this class's own — one shared
    connection, one shared loop, two logical dispatchers.
    """

    def __init__(self, connection_string, state, flight_command_bus, watchdog=None,
                 interval=0.05, payload_sender=None):
        super().__init__(connection_string, state, watchdog, interval)
        self.command_drone       = None
        self.command_queue       = queue.Queue()
        self.command_bus         = flight_command_bus
        self.takeoff_in_progress = False
        self._mav_lock           = threading.Lock()

        self.Manual     = None
        self.Director   = None
        self.ManualCtrl = None
        self._ato       = None
        self._analog    = None   # AnalogInputHandler — set after connection
        self._payload_analog = None   # PayloadAnalogInputHandler — set after connection

        self.payload_sender = payload_sender

    # ── Connection ────────────────────────────────────────────────────────────

    def _do_connect(self):
        self.command_drone = self._connect_with_retry(label="flight-command-sender")

        if self.is_cancelled():
            self.command_drone.close()
            self.command_drone = None
            return False

        self._heartbeat(self.command_drone)

        self.Manual     = ModeCommander(self.command_drone, self.state, self._mav_lock)
        self.Director   = DirectionCommander(self.command_drone, self.state, self._mav_lock)
        self.ManualCtrl = ManualController(self.command_drone, self.state, self._mav_lock)
        self._ato       = AutoTakeOff(self.command_drone, self._mav_lock)

        if self.payload_sender:
            self.payload_sender.attach(self.command_drone, self._mav_lock)

        # Wired here rather than in __init__: needs a live ManualCtrl
        # (flight side) to exist first, which only happens post-connect.
        # AnalogInputHandler is now flight-only (joystick -> FBWA/FBWB
        # streaming) — no longer takes `sender` at all, since it never
        # actually used it for anything payload-related; the payload half
        # moved to PayloadAnalogInputHandler, driven off payload_sender.
        from mavlink.Flight_controller_commands.analog_input_handler import AnalogInputHandler
        from mavlink.payload_commands.payload_analog_input_handler import PayloadAnalogInputHandler
        self._analog = AnalogInputHandler(self.state, self.ManualCtrl)
        self._payload_analog = (
            PayloadAnalogInputHandler(self.state, self.payload_sender)
            if self.payload_sender else None
        )

        self.state.update_UAV_Command_Connection(True, self.command_drone)
        logger.info("FlightCommandSender connected.")
        self._start_loop()
        return True

    def _do_disconnect(self):
        if self.ManualCtrl:
            self.ManualCtrl.stop_streaming()
            self.ManualCtrl = None
        if self._ato:
            self._ato.cancel()
            self._ato = None
        if self.payload_sender:
            self.payload_sender.detach()
        if self.command_drone:
            try:
                self.command_drone.close()
            except Exception:
                pass
            self.command_drone = None
        self.Manual   = None
        self.Director = None
        self._analog  = None
        self._payload_analog = None
        self.state.update_UAV_Command_Connection(False, None)
        logger.info("FlightCommandSender disconnected.")

    def _pet_watchdog(self):
        if self.watchdog:
            self.watchdog.uavCommand_pet()

    def _start_watch(self):
        if self.watchdog:
            self.watchdog.watchCommandsThread()

    def _on_error(self, e):
        logger.error(f"FlightCommandSender loop error: {e}")
        self.state.update_UAV_Command_Connection(False, None)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _run_once(self):
        self.drain_input_commands()
        if self.payload_sender:
            self.payload_sender.drain_input_commands()
            self.payload_sender.process_queue()
        if self._analog:
            self._analog.process()
        if self._payload_analog:
            self._payload_analog.process()

        while True:
            try:
                cmd, args, kwargs = self.command_queue.get_nowait()
            except queue.Empty:
                break
            cmd(*args, **kwargs)
            self.command_queue.task_done()

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
    """
    FIXED: this handler previously existed only inside an accidental
    triple-quoted string literal (`'''...sender.arm()'''`) — meaning
    @register_handler(ArmCommand) never actually executed, and arming
    via the STM had no live handler at all. Re-enabled.
    """
    logger.debug("ArmCommand received")
    sender.arm()

@register_handler(DisarmCommand)
def _handle_disarm(sender, cmd):
    logger.info("DisarmCommand received")
    sender.disarm()

@register_handler(TakeoffCommand)
def _handle_takeoff(sender, cmd):
    logger.debug("TakeoffCommand received")
    sender.start_takeoff()

@register_handler(ManualModeCommand)
def _handle_manual(sender, cmd):
    """
    FIXED: same class of bug as ArmCommand above — this handler was
    inside a stray `''''...'''` string block and never actually
    registered. Also now correctly reachable: translator.py used to fire
    LaserStartCommand (not this) for the "manual" field; that's fixed
    too, so this handler is both re-enabled AND now actually the thing
    that gets called when the manual-mode switch flips.
    """
    logger.debug("ManualModeCommand received")
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
    """Same fix as ManualModeCommand: translator.py used to fire
    ContiousLaserStartCommand for the "auto" field instead of this."""
    logger.debug("AutoModeCommand received")
    sender.set_auto()

@register_handler(LandCommand)
def _handle_land(sender, cmd):
    logger.debug("LandCommand received")
    # No-op, pre-existing — nothing has ever called anything here for
    # the autoland field. Left as-is; flag if you want this wired to a
    # real landing sequence.

@register_handler(SpeedUpCommand)
def _handle_speedup(sender, cmd):
    logger.info("SpeedUpCommand received")
    # No-op — this Command previously had NO handler registered at all
    # (every press logged "No handler registered for SpeedUpCommand").
    # Added as a stub for symmetry with SpeedDownCommand below, which
    # was already a no-op; neither actually adjusts airspeed yet, even
    # though set_airspeed() exists and is fully wired.

@register_handler(SpeedDownCommand)
def _handle_speeddown(sender, cmd):
    logger.info("SpeedDownCommand received")
    # No-op, pre-existing — see SpeedUpCommand's note above.