"""
raw_serial_read.py
───────────────────
Bare-minimum serial smoke test — no TLV parsing, no protocol
assumptions. Just proves bytes are physically arriving.

Usage:
    python raw_serial_read.py --port COM5
    python raw_serial_read.py --port COM5 --baud 115200
"""

import sys
import time
import argparse
import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
 

    try:
        ser = serial.Serial('COM67', 115200, timeout=0.005)
    except serial.SerialException as e:
        print(f"Failed to open {'COM67'}: {e}")
        sys.exit(1)

    print(f"Connected to {'COM67'} at {115200} baud.")
    print("Reading raw bytes. Press Ctrl+C to stop.\n")

    total_bytes = 0

    try:
        while True:
            chunk = ser.read(16)
            if not chunk:
                continue

            total_bytes += len(chunk)
            ts = time.strftime("%H:%M:%S")
            hex_str = " ".join(f"{b:02X}" for b in chunk)
            ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
            print(f"[{ts}] ({len(chunk):3d} bytes)  {hex_str}   |{ascii_str}|")

    except KeyboardInterrupt:
        print(f"\n\nStopped. Total bytes received: {total_bytes}")
    finally:
        ser.close()


if __name__ == "__main__":
    main()