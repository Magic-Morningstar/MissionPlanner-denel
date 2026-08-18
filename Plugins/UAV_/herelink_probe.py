#!/usr/bin/env python3
"""
herelink_probe.py — standalone MAVLink endpoint diagnostic.

Opens one endpoint, announces itself as a GCS at 1Hz, and prints every
HEARTBEAT it sees along with the exact fields the worker's filter checks
(type, autopilot, srcSystem, srcComponent). Also counts every other
message type so you can tell "nothing arriving" apart from "arriving but
rejected".

Run it with NOTHING else connected to the endpoint first, then again with
your app running, and compare.

Examples:
    python3 herelink_probe.py udpout:192.168.43.1:14552
    python3 herelink_probe.py udpin:0.0.0.0:14552
    python3 herelink_probe.py udpout:127.0.0.1:14561 --seconds 20
"""

import argparse
import collections
import sys
import threading
import time

from pymavlink import mavutil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("endpoint",
                    help="pymavlink connection string, e.g. udpout:192.168.43.1:14552")
    ap.add_argument("--seconds", type=int, default=15,
                    help="how long to listen (default 15)")
    ap.add_argument("--sysid", type=int, default=200)
    ap.add_argument("--compid", type=int, default=195,
                    help="use something neither worker uses (190/191)")
    ap.add_argument("--no-announce", action="store_true",
                    help="listen only, never transmit")
    args = ap.parse_args()

    if args.endpoint.startswith(("udpin:", "udp:")) and not args.no_announce:
        print("!! WARNING: this is an INPUT (server) socket.")
        print("!! pymavlink cannot transmit on it until it has RECEIVED something,")
        print("!! and it swallows the send error silently. If the far end only")
        print("!! unicasts to registered clients, this will never connect.")
        print("!! Try the same test with udpout: if you see nothing below.\n")

    print(f"opening {args.endpoint} as sys={args.sysid} comp={args.compid} ...")
    try:
        conn = mavutil.mavlink_connection(
            args.endpoint,
            source_system=args.sysid,
            source_component=args.compid,
        )
    except Exception as e:
        print(f"FAILED to open: {type(e).__name__}: {e}")
        return 1
    print("socket open.\n")

    stop = threading.Event()
    sent = [0]

    def announce():
        while not stop.is_set():
            try:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0,
                )
                sent[0] += 1
            except Exception as e:
                print(f"  [announce] send raised: {type(e).__name__}: {e}")
            stop.wait(1.0)

    if not args.no_announce:
        threading.Thread(target=announce, daemon=True, name="announce").start()

    counts = collections.Counter()
    seen_nodes = {}
    deadline = time.time() + args.seconds

    while time.time() < deadline:
        msg = conn.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        mtype = msg.get_type()
        counts[mtype] += 1
        if mtype == "BAD_DATA":
            continue

        if mtype == "HEARTBEAT":
            key = (msg.get_srcSystem(), msg.get_srcComponent())
            if key not in seen_nodes:
                seen_nodes[key] = msg
                is_gcs = msg.type == mavutil.mavlink.MAV_TYPE_GCS
                is_invalid_ap = msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID
                verdict = "ACCEPTED as vehicle"
                if is_gcs:
                    verdict = "REJECTED (MAV_TYPE_GCS)"
                elif is_invalid_ap:
                    verdict = "REJECTED (MAV_AUTOPILOT_INVALID)"
                elif msg.get_srcSystem() == args.sysid:
                    verdict = "REJECTED (that's us)"
                print(f"HEARTBEAT sys={key[0]:<4} comp={key[1]:<4} "
                      f"type={msg.type:<3} autopilot={msg.autopilot:<3} "
                      f"-> {verdict}")

    stop.set()
    time.sleep(0.2)

    print(f"\n--- {args.seconds}s summary ---")
    print(f"heartbeats sent: {sent[0]}")
    if not counts:
        print("NOTHING RECEIVED AT ALL.")
        print("  -> wrong host/port, wrong direction, or the router never")
        print("     registered us. Verify with:")
        print(f"     mavproxy.py --master={args.endpoint}")
        return 2

    print("messages received by type:")
    for mtype, n in counts.most_common(15):
        print(f"  {mtype:<28} {n}")

    vehicles = [k for k, m in seen_nodes.items()
                if m.type != mavutil.mavlink.MAV_TYPE_GCS
                and m.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID]
    print(f"\ndistinct nodes seen: {len(seen_nodes)}")
    print(f"nodes that look like a real autopilot: {len(vehicles)} {vehicles}")
    if not vehicles:
        print("  -> traffic is arriving but NO autopilot heartbeat among it.")
        print("     This is exactly what makes the worker hang.")
    return 0


if __name__ == "__main__":
    sys.exit(main())