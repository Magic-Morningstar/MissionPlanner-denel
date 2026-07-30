"""
joystick_test.py
────────────────
Streams and displays both joysticks' X/Y live, decoding the real TLV
binary protocol via StreamParser + registry — not ASCII lines.
"""

import sys
import time
import argparse
import serial
import serial.tools.list_ports

from serial_controller.protocol.stream_parser import StreamParser
from serial_controller.protocol.registry import get_decoder
from serial_controller.protocol.messages import Joystick, Joystick2

STM_VID, STM_PID = 0x0483, 0x5740
STM_BAUDRATE = 115200
ADC_MAX = 4095

def find_stm_port():

    ports = serial.tools.list_ports.comports()

    for port in ports:
        if port.vid == 0x0483 and port.pid == 0x5740:
            return port.device

    for port in ports:
        if port.vid == 0x1A86 and port.pid == 0x7523:
            return port.device
        

        if "USB-Enhanced-SERIAL-D" in port.description:
            return port.device


    return None
def as_percent(raw_value):
    return (raw_value / ADC_MAX) * 100.0


def make_bar(percent, width=20):
    filled = max(0, min(width, int((percent / 100.0) * width)))
    return "[" + "#" * filled + "-" * (width - filled) + "]"


def print_joystick_frame(j1_x, j1_y, j2_x, j2_y):
    line = (
        f"J1 X:{j1_x:4d} {make_bar(as_percent(j1_x))} "
        f"J1 Y:{j1_y:4d} {make_bar(as_percent(j1_y))}   "
        f"J2 X:{j2_x:4d} {make_bar(as_percent(j2_x))} "
        f"J2 Y:{j2_y:4d} {make_bar(as_percent(j2_y))}"
    )
    sys.stdout.write("\r" + line + " " * 5)
    sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=STM_BAUDRATE)
    parser.add_argument("--raw", action="store_true", help="print every decoded frame instead of the bar display")
    args = parser.parse_args()

    port = args.port or find_stm_port()
    if port is None:
        print("STM32 not found — pass --port explicitly.")
        sys.exit(1)

    ser = serial.Serial(port, args.baud, timeout=0.1)
    print(f"\nConnected to {port} at {args.baud} baud.")
    print("Move both joysticks to test. Press Ctrl+C to stop.\n")

    stream = StreamParser()
    j1_x = j1_y = j2_x = j2_y = 0
    frame_count = 0

    try:
        while True:
            chunk = ser.read(64)
            if not chunk:
                continue

            for msg_type, payload in stream.feed(chunk):
                decoder = get_decoder(msg_type)
                if decoder is None:
                    continue
                obj = decoder.decode(payload)

                if args.raw:
                    print(obj)

                if isinstance(obj, Joystick):
                    j1_x, j1_y = obj.x, obj.y
                    frame_count += 1
                elif isinstance(obj, Joystick2):
                    j2_x, j2_y = obj.x, obj.y
                    frame_count += 1
                else:
                    continue  # button state — ignored here

                if not args.raw:
                    print_joystick_frame(j1_x, j1_y, j2_x, j2_y)

    except KeyboardInterrupt:
        print(f"\n\nStopped. Received {frame_count} joystick frames.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()