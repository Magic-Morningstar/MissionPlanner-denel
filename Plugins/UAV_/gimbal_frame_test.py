"""
gimbal_frame_test.py
─────────────────────
Builds every supported GimbalFrameBuilder frame and prints the raw output
bytes, with a full breakdown of the protocol fields (header, body-length +
frame-counter byte, CMD_ID, DATA, CHECKSUM) and an independent checksum
re-verification — so you can eyeball exactly what would be sent to the
payload before it ever goes out over SERIAL_CONTROL / a real UART.

Two modes:
  1. Default: runs a fixed suite of test cases (absolute angle at several
     azimuth/tilt values including negative angles, relative angle, motor
     on/off, home, tracking) and prints each frame.
  2. --port: also opens a real serial port and transmits each frame
     directly to the gimbal, for bench testing outside the vehicle
     (bypasses MAVLink/SERIAL_CONTROL entirely — useful to confirm the
     Viewpro accepts the frames before wiring the FC into the loop).

Usage:
    python gimbal_frame_test.py
    python gimbal_frame_test.py --azimuth 45 --tilt -30
    python gimbal_frame_test.py --port COM5 --baud 115200 --send
"""

import sys
import argparse

from mavlink.uav_commads.commands.payload_commands import GimbalFrameBuilder


def hex_bytes(frame: bytes) -> str:
    return " ".join(f"{b:02X}" for b in frame)


def verify_checksum(frame: bytes) -> tuple[bool, int, int]:
    """
    Independently recomputes the checksum from the raw frame bytes
    (byte3 through the byte before checksum) and compares it against
    the checksum byte actually present at the end of the frame.
    Returns (matches, expected, actual).
    """
    body = frame[3:-1]   # byte3 .. last data byte (excludes header, excludes checksum)
    expected = 0
    for b in body:
        expected ^= b
    actual = frame[-1]
    return expected == actual, expected, actual


def decode_byte3(byte3: int) -> tuple[int, int]:
    """Splits byte3 into (body_length_n, frame_counter)."""
    n = byte3 & 0x3F
    frame_counter = (byte3 >> 6) & 0x03
    return n, frame_counter


def print_frame(label: str, frame: bytes):
    n, frame_counter = decode_byte3(frame[3])
    ok, expected, actual = verify_checksum(frame)
    checksum_status = "OK" if ok else f"MISMATCH (expected 0x{expected:02X}, got 0x{actual:02X})"

    print(f"\n── {label} " + "─" * max(1, 60 - len(label)))
    print(f"  RAW BYTES   : {hex_bytes(frame)}")
    print(f"  LENGTH      : {len(frame)} bytes")
    print(f"  HEADER      : {hex_bytes(frame[0:3])}")
    print(f"  BYTE3       : 0x{frame[3]:02X}  (body_length n={n}, frame_counter={frame_counter})")
    print(f"  CMD_ID      : 0x{frame[4]:02X}")
    print(f"  DATA        : {hex_bytes(frame[5:-1])}")
    print(f"  CHECKSUM    : 0x{frame[-1]:02X}  [{checksum_status}]")


def run_test_suite(builder: GimbalFrameBuilder, azimuth: float, tilt: float):
    print_frame(
        f"Absolute angle  (azimuth={azimuth:+.1f}°, tilt={tilt:+.1f}°)",
        builder.build_A1_absolute_angle(azimuth, tilt),
    )
    print_frame(
        "Absolute angle  (azimuth=0°, tilt=0°) — home-relative zero",
        builder.build_A1_absolute_angle(0, 0),
    )
    print_frame(
        "Absolute angle  (azimuth=-179.9°, tilt=-89.9°) — near negative extremes",
        builder.build_A1_absolute_angle(-179.9, -89.9),
    )
    print_frame(
        "Absolute angle  (azimuth=179.9°, tilt=44.9°) — near positive extremes",
        builder.build_A1_absolute_angle(179.9, 44.9),
    )
    print_frame(
        "Relative angle  (delta_azimuth=+10°, delta_tilt=-5°)",
        builder.build_A1_relative_angle(10, -5),
    )
    print_frame("Motor ON", builder.build_A1_motor(True))
    print_frame("Motor OFF", builder.build_A1_motor(False))
    print_frame("Home position", builder.build_A1_home())
    print_frame("Tracking mode", builder.build_A1_tracking())


def send_frame(ser, frame: bytes, label: str):
    ser.write(frame)
    print(f"  → sent {len(frame)} bytes to {ser.port} for '{label}'")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--azimuth", type=float, default=30.0, help="Azimuth angle (deg) for the custom test frame")
    parser.add_argument("--tilt", type=float, default=-15.0, help="Tilt angle (deg) for the custom test frame")
    parser.add_argument("--port", default=None, help="Serial port to also transmit frames to (e.g. COM5, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate for --port")
    parser.add_argument("--send", action="store_true", help="Actually transmit frames out --port (otherwise just print)")
    args = parser.parse_args()

    builder = GimbalFrameBuilder()

    print("=" * 70)
    print("GimbalFrameBuilder — raw frame test suite")
    print("=" * 70)

    run_test_suite(builder, args.azimuth, args.tilt)

    print("\n" + "=" * 70)
    print("Frame counter check — should roll 0→1→2→3→0 across calls above")
    print(f"Next frame counter value: {builder._frame_counter}")
    print("=" * 70)

    if args.port:
        if not args.send:
            print(f"\n--port given but --send not set — not transmitting. Pass --send to actually write to {args.port}.")
            return

        import serial  # local import so the print-only path has no pyserial dependency

        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        print(f"\nConnected to {args.port} at {args.baud} baud. Sending test frames...\n")

        try:
            send_frame(ser, builder.build_A1_absolute_angle(args.azimuth, args.tilt), "custom absolute angle")
            send_frame(ser, builder.build_A1_home(), "home")
        finally:
            ser.close()
            print("\nSerial port closed.")


if __name__ == "__main__":
    main()