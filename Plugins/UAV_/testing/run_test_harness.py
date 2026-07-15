"""
testing/run_test_harness.py

A keyboard-driven stand-in for the STM32, wired to the REAL, unmodified
serial pipeline: SerialHandler -> StreamParser -> registry -> decoders
-> InputTranslator -> command_bus. No MAVLink/SITL required for this —
it prints the Command objects that WOULD be dispatched, which is exactly
the boundary we designed (command_sender.py never needs to be running
for this to prove the serial side works).

Run interactively:
    python testing/run_test_harness.py

Run a scripted sequence (useful for CI / no real terminal available):
    python testing/run_test_harness.py --script "attmreq"

Keys:
  a  toggle ARM switch          r  toggle RTL switch
  t  toggle TAKEOFF switch      m  toggle MANUAL switch
  e  toggle EMERGENCY switch
  +/-  pot value up/down (0-255, clamped)
  i/k  joystick Y up/down       j/l  joystick X left/right
  c  center joystick
  p  toggle a simulated UAV-armed FACT and push it out as an outgoing
     STATUS frame (tests the PC -> STM32 direction)
  h  help
  q  quit

Buttons are simulated as TOGGLE SWITCHES (matching your real hardware)
and are re-transmitted every 300ms in the background even when unchanged
— this is deliberate: it's what proves duplicate frames from a held
switch don't re-fire commands (the exact bug the edge-detection design
fixes). Watch the [COMMAND] lines: you should see exactly one per press,
never one per resend.
"""

import sys
import os
import time
import queue
import threading
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from state.system_state import SystemState
from commands.translator import InputTranslator
from serial_controller.serial_handler import SerialHandler
from serial_controller.protocol.messages import ButtonState, Joystick
from serial_controller.protocol.registry import MessageType, get_decoder
from serial_controller.protocol.frame_builder import build_frame
from serial_controller.protocol.stream_parser import StreamParser
from testing.fake_serial import make_fake_serial_pair
from utils.helper import system_Print
from logging_config import setup_logging
import logging


HELP = """
────────────────────────────────────────────────────────────
  a  toggle ARM        r  toggle RTL
  t  toggle TAKEOFF     m  toggle MANUAL
  e  toggle EMERGENCY
  +/-  pot value up/down
  i/k  joystick Y up/down     j/l  joystick X left/right
  c  center joystick
  p  simulate UAV armed-fact change -> send outgoing STATUS
  h  help                q  quit
────────────────────────────────────────────────────────────
"""


class STMState:
    """The 'physical switch positions' the simulator is currently holding."""
    def __init__(self):
        self.arm = False
        self.rtl = False
        self.manual = False
        self.takeoff = False
        self.emergency = False
        self.system_check = False
        self.pot_value = 0
        self.joy_x = 2048
        self.joy_y = 2048

    def to_button_state(self) -> ButtonState:
        return ButtonState(
            arm=self.arm, rtl=self.rtl, manual=self.manual,
            takeoff=self.takeoff, emergency=self.emergency,
            system_check=self.system_check, pot_value=self.pot_value,
        )

    def to_joystick(self) -> Joystick:
        return Joystick(x=self.joy_x, y=self.joy_y)


def send_button_state(stm_side, stm_state):
    stm_side.write(build_frame(MessageType.BUTTON_STATE, stm_state.to_button_state()))


def send_joystick(stm_side, stm_state):
    stm_side.write(build_frame(MessageType.JOYSTICK, stm_state.to_joystick()))


# ── Background threads ────────────────────────────────────────────────────

def heartbeat_loop(stm_side, stm_state, stop_event, interval=0.3):
    """Mimics real hardware continuously transmitting current switch/ADC
    state, whether or not anything changed. This is what stress-tests
    the edge-detection dedup."""
    while not stop_event.is_set():
        send_button_state(stm_side, stm_state)
        send_joystick(stm_side, stm_state)
        stop_event.wait(interval)


def command_consumer_loop(command_bus, stop_event):
    """Stand-in for what UAVCommandSender.drain_input_commands() would do —
    prints each Command instead of dispatching to MAVLink."""
    while not stop_event.is_set():
        try:
            cmd = command_bus.get(timeout=0.1)
        except queue.Empty:
            continue
        system_Print(f"[COMMAND -> would dispatch to MAVLink]  {cmd}")


def stm_receive_loop(stm_side, stop_event):
    """The 'STM32' listening for outgoing STATUS frames from the PC —
    proves the PC -> STM32 direction round-trips correctly too."""
    parser = StreamParser()
    while not stop_event.is_set():
        chunk = stm_side.read(64)
        if chunk:
            for msg_type, payload in parser.feed(chunk):
                decoder = get_decoder(msg_type)
                if decoder:
                    obj = decoder.decode(payload)
                    system_Print(f"[STM32 <- received from PC]  {obj}")
                else:
                    system_Print(f"[STM32 <- received unknown type {msg_type:#x}]")


def status_send_loop(serial_handler, state, stop_event):
    """Mirrors the two lines in your real main.py loop that trigger
    compile_Send() whenever UAV_STATE_CHANGE is set."""
    while not stop_event.is_set():
        if state.is_Serial_Connection_Available and state.UAV_STATE_CHANGE:
            serial_handler.compile_Send()
        stop_event.wait(0.05)


# ── Key handling ──────────────────────────────────────────────────────────

def handle_key(ch, stm_state, stm_side, state):
    if ch == 'q':
        return False
    elif ch == 'a':
        stm_state.arm = not stm_state.arm
        system_Print(f"[KEY] arm switch -> {stm_state.arm}")
        send_button_state(stm_side, stm_state)
    elif ch == 'r':
        stm_state.rtl = not stm_state.rtl
        system_Print(f"[KEY] rtl switch -> {stm_state.rtl}")
        send_button_state(stm_side, stm_state)
    elif ch == 'm':
        stm_state.manual = not stm_state.manual
        system_Print(f"[KEY] manual switch -> {stm_state.manual}")
        send_button_state(stm_side, stm_state)
    elif ch == 't':
        stm_state.takeoff = not stm_state.takeoff
        system_Print(f"[KEY] takeoff switch -> {stm_state.takeoff}")
        send_button_state(stm_side, stm_state)
    elif ch == 'e':
        stm_state.emergency = not stm_state.emergency
        system_Print(f"[KEY] emergency switch -> {stm_state.emergency}")
        send_button_state(stm_side, stm_state)
    elif ch in ('+', '='):
        stm_state.pot_value = min(255, stm_state.pot_value + 10)
        system_Print(f"[KEY] pot -> {stm_state.pot_value}")
        send_button_state(stm_side, stm_state)
    elif ch == '-':
        stm_state.pot_value = max(0, stm_state.pot_value - 10)
        system_Print(f"[KEY] pot -> {stm_state.pot_value}")
        send_button_state(stm_side, stm_state)
    elif ch == 'i':
        stm_state.joy_y = min(4095, stm_state.joy_y + 200)
        send_joystick(stm_side, stm_state)
    elif ch == 'k':
        stm_state.joy_y = max(0, stm_state.joy_y - 200)
        send_joystick(stm_side, stm_state)
    elif ch == 'j':
        stm_state.joy_x = max(0, stm_state.joy_x - 200)
        send_joystick(stm_side, stm_state)
    elif ch == 'l':
        stm_state.joy_x = min(4095, stm_state.joy_x + 200)
        send_joystick(stm_side, stm_state)
    elif ch == 'c':
        stm_state.joy_x, stm_state.joy_y = 2048, 2048
        send_joystick(stm_side, stm_state)
        system_Print("[KEY] joystick centered")
    elif ch == 'p':
        armed = not state.is_UAV_Armed
        state.update_UAV_Armed_Status(armed)
        state.set_UAV_State_Changed()
        system_Print(f"[KEY] simulated UAV armed FACT -> {armed} (sending STATUS...)")
    elif ch == 'h':
        print(HELP)
    return True


# ── Cross-platform single-keypress input, with a scripted fallback ────────

def make_key_source(script=None):
    if script is not None:
        chars = iter(script)
        def get_key():
            time.sleep(0.35)
            return next(chars, None)
        return get_key

    if sys.stdin.isatty():
        if sys.platform == "win32":
            import msvcrt
            def get_key():
                return msvcrt.getch().decode(errors="ignore")
            return get_key
        else:
            import termios
            import tty
            def get_key():
                fd = sys.stdin.fileno()
                old = termios.tcgetattr(fd)
                try:
                    tty.setraw(fd)
                    return sys.stdin.read(1)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old)
            return get_key

    # No real terminal attached and no script given — run a short demo
    # so the tool is still useful non-interactively.
    demo = "attmreq"
    system_Print(f"No TTY detected — running built-in demo script: {demo!r}")
    return make_key_source(script=demo)


# ── Main ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--script", default=None,
                         help="Feed a fixed sequence of keys instead of reading the keyboard, e.g. --script 'attmreq'")
    args = parser.parse_args()

    setup_logging(console_level=logging.DEBUG, log_file="test_harness.log")

    stm_side, pc_side = make_fake_serial_pair()

    state = SystemState()
    command_bus = queue.Queue(maxsize=256)
    translator = InputTranslator(command_bus, state)
    serial_handler = SerialHandler(state, translator, watchdog=None, serial_override=pc_side)

    stop_event = threading.Event()
    stm_state = STMState()

    serial_handler.connect()
    time.sleep(0.2)  # let _do_connect's thread finish spinning up

    threading.Thread(target=heartbeat_loop, args=(stm_side, stm_state, stop_event), daemon=True).start()
    threading.Thread(target=command_consumer_loop, args=(command_bus, stop_event), daemon=True).start()
    threading.Thread(target=stm_receive_loop, args=(stm_side, stop_event), daemon=True).start()
    threading.Thread(target=status_send_loop, args=(serial_handler, state, stop_event), daemon=True).start()

    print(HELP)
    get_key = make_key_source(script=args.script)

    try:
        while True:
            ch = get_key()
            if ch is None:
                break
            if not handle_key(ch, stm_state, stm_side, state):
                break
    except KeyboardInterrupt:
        pass
    finally:
        system_Print("Shutting down test harness...")
        stop_event.set()
        time.sleep(0.3)
        serial_handler.disconnect()


if __name__ == "__main__":
    main()
