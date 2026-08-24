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
import threading
import queue
import serial
import serial.tools.list_ports

from serial_controller.protocol.stream_parser import StreamParser
from serial_controller.protocol.registry import get_decoder, MessageType
from serial_controller.protocol.messages import ButtonState

STM_BAUDRATE = 115200

# Must match main.c: #define TLV_SYNC 0xAA / #define TLV_END 0x55
TLV_SYNC = 0xAA
TLV_END = 0x55


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
    if bs.arm:            parts.append("ARM")
    if bs.rtl:            parts.append("RTL")
    if bs.manual:         parts.append("MANUAL")
    if bs.takeoff:        parts.append("TAKEOFF")
    if bs.emergency:      parts.append("EMERGENCY")
    if bs.auto:           parts.append("AUTO MODE")
    if bs.autoland:       parts.append("AUTOLAND")
    if bs.speedup:        parts.append("SPEEDUP")
    if bs.speeddown:      parts.append("SPEED DOWN")
    if bs.zoomin:         parts.append("ZOOM IN")
    if bs.zoomout:        parts.append("ZOOM OUT")
    if bs.widein:         parts.append("WIDE IN")
    if bs.wideout:        parts.append("WIDE OUT")

    label = ", ".join(parts) if parts else "IDLE"
    pot = getattr(bs, "pot_value", "N/A")
    return f"{label}  (pot={pot})"


def build_tlv_frame(msg_type: int, payload: bytes) -> bytes:
    """Mirrors TLV_Send() in main.c: SYNC | TYPE | LEN | PAYLOAD | CRC8(payload) | END
    CRC8 there is a plain XOR over the payload bytes only (not type/len)."""
    crc = 0
    for b in payload:
        crc ^= b
    return bytes([TLV_SYNC, msg_type & 0xFF, len(payload) & 0xFF]) + payload + bytes([crc, TLV_END])


def parse_hex_bytes(text: str) -> bytes:
    """Accepts 'AA 01 04', 'AA0104', or 'AA,01,04'."""
    cleaned = text.replace(",", " ").split()
    return bytes(int(tok, 16) for tok in cleaned)


def handle_command_line(ser: serial.Serial, line: str) -> bool:
    """Parses one typed command and sends it. Returns False to request exit."""
    line = line.strip()
    if not line:
        return True

    parts = line.split(maxsplit=1)
    cmd = parts[0].lower()
    rest = parts[1] if len(parts) > 1 else ""

    if cmd in ("quit", "exit", "q"):
        return False

    if cmd == "help":
        print(
            "\nCommands:\n"
            "  hex AA 01 04 00 00 00 00 04 55   send those exact raw bytes\n"
            "  tlv <type_hex> [payload hex bytes...]\n"
            "                                    build+send a TLV frame the way TLV_Send()\n"
            "                                    in main.c does — SYNC/LEN/CRC8/END are\n"
            "                                    added for you, e.g. 'tlv 01 00 00 00 00'\n"
            "  quit / exit                       stop the tool\n"
        )
        return True

    if cmd == "hex":
        try:
            data = parse_hex_bytes(rest)
        except ValueError as e:
            print(f"\n  bad hex ({e}) — type 'help'")
            return True
        ser.write(data)
        print(f"\n  sent {len(data)} bytes: {data.hex(' ').upper()}")
        return True

    if cmd == "tlv":
        tokens = rest.split()
        if not tokens:
            print("\n  usage: tlv <type_hex> [payload hex bytes...]")
            return True
        try:
            msg_type = int(tokens[0], 16)
            payload = bytes(int(tok, 16) for tok in tokens[1:])
        except ValueError as e:
            print(f"\n  bad hex ({e}) — type 'help'")
            return True
        frame = build_tlv_frame(msg_type, payload)
        ser.write(frame)
        print(f"\n  sent TLV frame: {frame.hex(' ').upper()}")
        return True

    print(f"\n  unknown command '{cmd}' — type 'help'")
    return True


def stdin_reader(cmd_queue: "queue.Queue[str]"):
    """Runs in a daemon thread: blocking input() here keeps the main thread
    free to keep reading/printing incoming frames without stalling."""
    while True:
        try:
            line = input()
        except EOFError:
            cmd_queue.put("quit")
            return
        cmd_queue.put(line)


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
    print("Press buttons to test, or type a command ('help' lists them). Ctrl+C to stop.\n")

    cmd_queue: "queue.Queue[str]" = queue.Queue()
    input_thread = threading.Thread(target=stdin_reader, args=(cmd_queue,), daemon=True)
    input_thread.start()

    stream = StreamParser()
    frame_count = 0
    running = True

    try:
        while running:
            # Send anything typed since the last loop iteration.
            # ser is only ever touched from this thread, so no lock is needed.
            while True:
                try:
                    typed = cmd_queue.get_nowait()
                except queue.Empty:
                    break
                running = handle_command_line(ser, typed)
                if not running:
                    break
            if not running:
                break

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

                try:
                    intent = describe_intent(bs)
                except AttributeError as e:
                    # This is exactly the BIT_* / bit_definitions.py mismatch
                    # the module docstring warns about — surface it, don't crash.
                    intent = f"<field mismatch: {e}>"

                frame_count += 1
                line = (
                    f"RAW  HEX:0x{raw_value:08X}  BIN:{raw_value:032b}  DEC:{raw_value:>10d}  |  "
                    f"DECODED INTENT: {intent}"
                )
                sys.stdout.write("\r" + line + " " * 5)
                sys.stdout.flush()

    except KeyboardInterrupt:
        print(f"\n\nStopped. Received {frame_count} button-state frames.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()