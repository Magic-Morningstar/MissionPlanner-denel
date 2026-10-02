"""Phase 1 acceptance #1: a minimal MAVLink client that proves frames arrive.

Stands in for Mission Planner so the tunnel can be verified before involving the
GUI. Unlike the bridge itself this DOES use pymavlink's parser -- that is the
point: if this decodes a HEARTBEAT, the bytes crossing the broker are genuinely
well-formed MAVLink and not merely the right length.

    python -m mqtt_bridge.tools.mav_probe --connect udpout:127.0.0.1:14570

udpout is right for talking to the GCS bridge: the bridge binds the port and
learns our address from the first datagram we send, exactly as Mission Planner's
UDPCl option does.
"""

import argparse
import sys
import time
from collections import Counter

from pymavlink import mavutil


def main(argv=None):
    ap = argparse.ArgumentParser(description="Minimal MAVLink probe for the bridge.")
    ap.add_argument("--connect", default="udpout:127.0.0.1:14570",
                    help="pymavlink connection string (default udpout:127.0.0.1:14570)")
    ap.add_argument("--seconds", type=float, default=15.0,
                    help="how long to listen (default 15)")
    ap.add_argument("--expect", default="HEARTBEAT",
                    help="message type that must be seen (default HEARTBEAT)")
    ap.add_argument("--quiet", action="store_true", help="only print the summary")
    ap.add_argument("--request-streams", action="store_true",
                    help="ask the vehicle for all data streams. Proves the "
                         "to_vehicle direction end to end: a command goes out "
                         "through the tunnel and the extra message types coming "
                         "back are the vehicle's response to it. Without this a "
                         "bare SITL only emits HEARTBEAT and TIMESYNC, because "
                         "nothing has asked it for more.")
    args = ap.parse_args(argv)

    print(f"connecting {args.connect}")
    conn = mavutil.mavlink_connection(args.connect)

    # The bridge learns our address from the first datagram, so announce
    # ourselves rather than waiting silently. A GCS-role heartbeat is the
    # conventional way to do it, and it also exercises the to_vehicle direction.
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    seen = Counter()
    sources = Counter()
    bad_data = 0
    first_hit = None
    deadline = time.monotonic() + args.seconds
    last_beat = 0.0
    streams_requested = False

    while time.monotonic() < deadline:
        # Keep announcing: if the bridge restarts it must relearn our address.
        now = time.monotonic()
        if now - last_beat > 1.0:
            try:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                pass
            last_beat = now

        msg = conn.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "BAD_DATA":
            bad_data += 1
            continue
        seen[kind] += 1
        sources[(msg.get_srcSystem(), msg.get_srcComponent())] += 1
        if kind == args.expect and first_hit is None:
            first_hit = time.monotonic()
            if not args.quiet:
                print(f"got {kind} from system {msg.get_srcSystem()} "
                      f"component {msg.get_srcComponent()}")

        # Only once a heartbeat has arrived, because target_system is taken from
        # it -- requesting before that would address system 0.
        if args.request_streams and not streams_requested and kind == "HEARTBEAT":
            conn.mav.request_data_stream_send(
                msg.get_srcSystem(), msg.get_srcComponent(),
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
            streams_requested = True
            if not args.quiet:
                print("requested all data streams at 4 Hz through the tunnel")

    print()
    print(f"message types seen : {len(seen)}")
    for kind, count in seen.most_common(12):
        print(f"  {kind:<28} {count}")
    print(f"sources (sysid/compid): "
          f"{', '.join(f'{s}/{c}={n}' for (s, c), n in sorted(sources.items())) or 'none'}")
    print(f"BAD_DATA frames    : {bad_data}")

    if first_hit is None:
        print(f"\nFAIL  no {args.expect} within {args.seconds}s")
        print("      Check, in order: is the aircraft bridge publishing (its log "
              "shows frames_out climbing)? is the GCS bridge subscribed? did this "
              "probe's first datagram reach the bridge so it learned our address?")
        return 1
    if bad_data:
        # Not a failure on its own: MAVProxy and MP both emit a little noise, and
        # the probe's own first datagram can land mid-stream. A large count
        # alongside few good messages does mean something is wrong.
        print(f"\nnote: {bad_data} BAD_DATA frame(s) seen")

    print(f"\nPASS  {args.expect} received; the tunnel carries valid MAVLink")
    return 0


if __name__ == "__main__":
    sys.exit(main())
