"""
btn_message_reader.py
──────────────────────
Reads BUTTON_STATE frames from the STM32 and prints:
  1. The TRUE raw 32-bit register, unpacked directly from the wire bytes
     (independent of ButtonState's field-splitting logic).
  2. The decoded "intent" using the project's real ButtonState fields
     (arm/rtl/manual/takeoff/emergency/system_check/pot_value) — this is
     what commands/translator.py actually acts on.

NOTE: as of writing, main.c's BIT_* defines and
serial_controller/protocol/bit_definitions.py do NOT agree on bit
positions. The raw value below is ground truth; the decoded intent will
be wrong until the two sides are reconciled. Compare them side by side.
"""

import sys
import argparse
import struct
import serial
import serial.tools.list_ports

from serial_controller.protocol.stream_parser import StreamParser
from serial_controller.protocol.registry import get_decoder, MessageType
from serial_controller.protocol.messages import ButtonState

STM_BAUDRATE = 115200


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


def describe_intent(bs: ButtonState) -> str:
    """Mirrors the fields commands/translator.py actually reads."""
    parts = []
    if bs.arm:           parts.append("ARM")
    if bs.rtl:            parts.append("RTL")
    if bs.manual:         parts.append("MANUAL")
    if bs.takeoff:        parts.append("TAKEOFF")
    if bs.emergency:      parts.append("EMERGENCY")
    if bs.system_check:   parts.append("SYSTEM_CHECK")
    label = ", ".join(parts) if parts else "IDLE"
    return f"{label}  (pot={bs.pot_value})"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=STM_BAUDRATE)
    args = parser.parse_args()

    port = args.port or find_stm_port()
    if port is None:
        print("STM32 not found — pass --port explicitly.")
        sys.exit(1)

    ser = serial.Serial(port, args.baud, timeout=0.1)
    print(f"\nConnected to {port} at {args.baud} baud.")
    print("Press buttons to test. Press Ctrl+C to stop.\n")

    stream = StreamParser()
    frame_count = 0

    try:
        while True:
            chunk = ser.read(64)
            if not chunk:
                continue

            for msg_type, payload in stream.feed(chunk):
                if msg_type != MessageType.BUTTON_STATE:
                    continue  # ignore joystick frames here

                # Ground truth: unpack the raw register ourselves,
                # bypassing ButtonState's (currently mismatched) bit map.
                raw_value = struct.unpack('<I', payload)[0]

                decoder = get_decoder(msg_type)
                bs: ButtonState = decoder.decode(payload)

                frame_count += 1
                line = (
                    f"RAW  HEX:0x{raw_value:08X}  BIN:{raw_value:032b}  DEC:{raw_value:>10d}  |  "
                    f"DECODED INTENT: {describe_intent(bs)}"
                )
                sys.stdout.write("\r" + line + " " * 5)
                sys.stdout.flush()

    except KeyboardInterrupt:
        print(f"\n\nStopped. Received {frame_count} button-state frames.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()