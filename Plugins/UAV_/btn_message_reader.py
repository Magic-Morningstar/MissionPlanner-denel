"""
btn_message_reader.py
──────────────────────
Reads BUTTON_STATE and PAYLOAD_COMMAND frames from the STM32 and prints:
  1. The TRUE raw 32-bit register, unpacked directly from the wire bytes
     (independent of the decoders' field-splitting logic) for each type.
  2. The decoded "intent" using the real ButtonState/PayloadCommand
     fields — this is what commands/translator.py actually acts on.

BUTTON_STATE and PAYLOAD_COMMAND both print as change-only lines — one
line per frame that actually differs from the last one of that type,
nothing per-frame otherwise. Both registers get sent every ~10ms
unconditionally by main.c regardless of whether anything changed, so
without this the terminal fills with identical repeats.

JOYSTICK/JOYSTICK2 frames are intentionally still ignored — this is a
*button* message reader; the two register types above are what
commands/translator.py's edge-detection actually reads.
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
from serial_controller.protocol.messages import ButtonState, PayloadCommand

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
    """Mirrors FLIGHT_EDGE_TABLE's fields in translator.py — flight/
    status only. menu_select (0-3) is shown separately since it's a
    level value, not something translator.py edge-detects."""
    parts = []
    if bs.arm:            parts.append("ARM")
    if bs.rtl:            parts.append("RTL")
    if bs.manual:         parts.append("MANUAL")
    if bs.auto:           parts.append("AUTO")
    if bs.takeoff:        parts.append("TAKEOFF")
    if bs.emergency:      parts.append("EMERGENCY")
    if bs.autoland:       parts.append("AUTOLAND")
    if bs.speedup:        parts.append("SPEED UP")
    if bs.speeddown:      parts.append("SPEED DOWN")

    label = ", ".join(parts) if parts else "IDLE"
    return f"{label}  (menu={bs.menu_select})"


def describe_payload_intent(pc: PayloadCommand) -> str:
    """Mirrors PAYLOAD_EDGE_TABLE's field order in translator.py — all
    30 PayloadCommand fields, camera/laser/tracking/gimbal."""
    parts = []
    if pc.zoomin:                     parts.append("ZOOM IN")
    if pc.zoomout:                    parts.append("ZOOM OUT")
    if pc.widein:                     parts.append("FOV NARROW")
    if pc.wideout:                    parts.append("FOV WIDE")
    if pc.focus_in:                   parts.append("FOCUS NEAR")
    if pc.focus_out:                  parts.append("FOCUS FAR")
    if pc.laser_on_off:               parts.append("LASER ON/OFF")
    if pc.laser_cont_mode:            parts.append("LASER CONT MODE")
    if pc.laser_single_mode:          parts.append("LASER SINGLE MODE")
    if pc.laser_zoom_in:              parts.append("LASER ZOOM IN")
    if pc.laser_zoom_out:             parts.append("LASER ZOOM OUT")
    if pc.tracking_search_on_off:     parts.append("TRACKING SEARCH")
    if pc.ai_tracking_on_off:         parts.append("AI TRACKING")
    if pc.tracking_template_toggle:   parts.append("TRACKING TEMPLATE")
    if pc.tracking_source_toggle:     parts.append("TRACKING SOURCE")
    if pc.joystick_track:             parts.append("JOYSTICK TRACK")
    if pc.take_picture:               parts.append("TAKE PICTURE")
    if pc.start_record:               parts.append("START RECORD")
    if pc.stop_record:                parts.append("STOP RECORD")
    if pc.picture_record_mode_toggle: parts.append("PIC/REC MODE")
    if pc.image_sensor_change:        parts.append("IMAGE SENSOR")
    if pc.ir_polarity:                parts.append("IR POLARITY")
    if pc.ir_camera_dzoom_plus:       parts.append("IR DZOOM +")
    if pc.ir_camera_dzoom_minus:      parts.append("IR DZOOM -")
    if pc.near_infrared_toggle:       parts.append("NEAR IR")
    if pc.eo_image_on_off:            parts.append("EO IMAGE")
    if pc.motor_on_off:               parts.append("MOTOR")
    if pc.video_ip:                   parts.append("VIDEO IP")
    if pc.eo_dzoom_toggle:            parts.append("EO DZOOM")
    if pc.ir_rainbow:                 parts.append("IR RAINBOW")

    return ", ".join(parts) if parts else "IDLE"


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
            "                                    (type 01 = BUTTON_STATE, 04 = PAYLOAD_COMMAND)\n"
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
    print("Press buttons to test, or type a command ('help' lists them). Ctrl+C to stop.")
    print("BUTTON_STATE updates live on one line; PAYLOAD_COMMAND logs new lines on change.\n")

    cmd_queue: "queue.Queue[str]" = queue.Queue()
    input_thread = threading.Thread(target=stdin_reader, args=(cmd_queue,), daemon=True)
    input_thread.start()

    stream = StreamParser()
    button_frame_count = 0
    payload_frame_count = 0
    last_button_raw = None    # for change-only logging
    last_payload_raw = None   # for change-only logging
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

                if msg_type == MessageType.BUTTON_STATE:
                    raw_value = struct.unpack('<I', payload)[0]

                    # Change-only, same treatment as PAYLOAD_COMMAND below —
                    # main.c sends this every ~10ms regardless of whether
                    # anything actually changed.
                    if raw_value == last_button_raw:
                        continue
                    last_button_raw = raw_value

                    decoder = get_decoder(msg_type)
                    bs: ButtonState = decoder.decode(payload)

                    try:
                        intent = describe_intent(bs)
                    except AttributeError as e:
                        # Safety net in case ButtonState's field set ever
                        # drifts from what this tool expects again —
                        # surface it, don't crash.
                        intent = f"<field mismatch: {e}>"

                    button_frame_count += 1
                    print(
                        f"BUTTON_STATE  HEX:0x{raw_value:08X}  BIN:{raw_value:032b}  |  "
                        f"{intent}"
                    )

                elif msg_type == MessageType.PAYLOAD_COMMAND:
                    raw_value = struct.unpack('<I', payload)[0]

                    # Change-only: skip if identical to the last PAYLOAD_COMMAND
                    # frame seen — this register is idle most of the time,
                    # and logging every 10ms frame regardless would be far
                    # noisier than useful.
                    if raw_value == last_payload_raw:
                        continue
                    last_payload_raw = raw_value

                    decoder = get_decoder(msg_type)
                    pc: PayloadCommand = decoder.decode(payload)

                    try:
                        intent = describe_payload_intent(pc)
                    except AttributeError as e:
                        intent = f"<field mismatch: {e}>"

                    payload_frame_count += 1
                    print(
                        f"PAYLOAD_COMMAND  HEX:0x{raw_value:08X}  BIN:{raw_value:032b}  |  "
                        f"{intent}"
                    )

                # JOYSTICK/JOYSTICK2 intentionally ignored — see module docstring.

    except KeyboardInterrupt:
        print(
            f"\n\nStopped. Received {button_frame_count} BUTTON_STATE frames, "
            f"{payload_frame_count} PAYLOAD_COMMAND changes."
        )
    finally:
        ser.close()


if __name__ == "__main__":
    main()