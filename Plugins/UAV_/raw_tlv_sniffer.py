"""
raw_tlv_sniffer.py
────────────────────
Bypasses serial_controller entirely and parses the TLV frames byte-by-byte,
exactly matching the firmware's TLV_Send() framing:

    [0xAA SYNC][TYPE][LEN][PAYLOAD...][CRC8][0x55 END]

CRC8 = XOR of all payload bytes (matches tlv_crc8() in main.c).

Use this to check ground truth: is the STM32 actually sending a nonzero
button-state payload, or is the problem in how the Python side decodes it?
"""

import sys
import argparse
import serial
import serial.tools.list_ports

STM_VID, STM_PID = 0x0483, 0x5740
STM_BAUDRATE = 115200

TLV_SYNC = 0xAA
TLV_END = 0x55

TYPE_NAMES = {
    0x01: "BUTTON_STATE",
    0x02: "JOYSTICK",
    0x03: "JOYSTICK2",
}


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


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc


def parse_stream(ser):
    """Generator: yields (msg_type, payload_bytes, crc_ok) as frames are found."""
    buf = bytearray()

    while True:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)

        # Try to find and consume complete frames in buf
        while True:
            sync_idx = buf.find(bytes([TLV_SYNC]))
            if sync_idx == -1:
                buf.clear()
                break
            if sync_idx > 0:
                del buf[:sync_idx]  # drop garbage before sync

            # Need at least SYNC, TYPE, LEN = 3 bytes to know the length
            if len(buf) < 3:
                break

            msg_type = buf[1]
            length = buf[2]
            frame_len = 3 + length + 1 + 1  # sync+type+len + payload + crc + end

            if len(buf) < frame_len:
                break  # wait for more bytes

            payload = bytes(buf[3:3 + length])
            recv_crc = buf[3 + length]
            end_byte = buf[3 + length + 1]

            if end_byte != TLV_END:
                # Not a valid frame at this sync byte — drop just the sync
                # and rescan, in case 0xAA showed up inside payload data.
                del buf[0:1]
                continue

            crc_ok = (crc8(payload) == recv_crc)
            del buf[:frame_len]

            yield msg_type, payload, crc_ok


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
    print("Raw TLV sniffer — press buttons and watch BUTTON_STATE payloads. Ctrl+C to stop.\n")

    try:
        for msg_type, payload, crc_ok in parse_stream(ser):
            name = TYPE_NAMES.get(msg_type, f"UNKNOWN(0x{msg_type:02X})")
            hex_bytes = " ".join(f"{b:02X}" for b in payload)

            line = f"[{name:13s}] len={len(payload)} bytes=[{hex_bytes}] crc={'OK' if crc_ok else 'BAD'}"

            if msg_type == 0x01 and len(payload) == 4:
                value = int.from_bytes(payload, byteorder="little")
                line += f"  -> uint32 = 0x{value:08X} ({value:>10d}) bin={value:032b}"

            print(line)

    except KeyboardInterrupt:
        print("\n\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()