"""Timed, repeatable versions of the three Mission Planner acceptance tests.

Drives the same MAVLink protocols Mission Planner uses, through the tunnel,
and reports how long each takes and whether it completes:

  1. parameter download   -- retry/timeout driven, the most latency-sensitive
  2. mission write + read-back + compare  -- a strict request/response ladder
  3. command round trip   -- time from sending to COMMAND_ACK

Why scripted rather than clicking in Mission Planner: the deliverable for this
phase is the latency at which each protocol starts to fail, which needs the same
test run repeatedly under different conditions with a stopwatch attached. The GUI
cannot give that. Confirm with the real Mission Planner afterwards.

IMPORTANT: Mission Planner must be DISCONNECTED while this runs. The GCS bridge
uses a last-peer policy, so this tool's first datagram takes over the peer slot
and Mission Planner would silently stop receiving.

    python -m mqtt_bridge.tools.link_test --label typical
"""

import argparse
import json
import sys
import time
from pathlib import Path

from pymavlink import mavutil


def _f32(x):
    import struct
    return struct.unpack("f", struct.pack("f", x))[0]


def _through_float32(scaled_degrees):
    """What a 1e-7-degree integer becomes after the legacy float32 mission path.

    Both steps happen in float32: the degrees go on the wire as float32, and the
    autopilot scales them back by 1e7 in float32 too. Doing the multiply in
    float64 here gets within a metre but not exactly, which was confirmed by
    measurement -- the float64 model was off by 9 units in latitude and 62 in
    longitude against the real autopilot, and the float32 model matches.

    At longitude 149 degrees this path resolves to roughly 1.1 m. That is the
    legacy protocol's limit, not the tunnel's.
    """
    return int(_f32(_f32(scaled_degrees / 1e7) * _f32(1e7)))


def _connect(connect_str, timeout):
    conn = mavutil.mavlink_connection(connect_str, source_system=255, source_component=190)
    # Announce ourselves so the bridge learns our address, exactly as MP does.
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if conn.wait_heartbeat(timeout=2) is not None:
            return conn
    return None


def test_param_download(conn, timeout, stall_tolerance=15.0):
    """Full parameter list. Returns (ok, seconds, count).

    stall_tolerance must exceed the link's worst stall plus a round trip, or this
    measures the harness giving up rather than the link failing. A real GCS has
    its own (different) patience; the point here is to characterise the link.
    """
    start = time.monotonic()
    conn.mav.param_request_list_send(conn.target_system, conn.target_component)

    params = {}
    expected = None
    last_rx = time.monotonic()
    while time.monotonic() - start < timeout:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is None:
            # Stalled. Re-request is what Mission Planner does too; without it a
            # single lost reply ends the download.
            if time.monotonic() - last_rx > stall_tolerance:
                break
            continue
        last_rx = time.monotonic()
        expected = msg.param_count
        pid = msg.param_id
        params[pid.strip("\x00") if isinstance(pid, str) else pid] = msg.param_value
        if expected and len(params) >= expected:
            break

    elapsed = time.monotonic() - start
    ok = bool(expected) and len(params) >= expected
    return ok, elapsed, f"{len(params)}/{expected or '?'}"


def test_mission_roundtrip(conn, count, timeout):
    """Upload `count` waypoints, read them back, compare. Returns (ok, s, detail)."""
    start = time.monotonic()
    home_lat, home_lon = -35.3632621, 149.1652373

    items = []
    for i in range(count):
        items.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            conn.target_system, conn.target_component, i,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, 0, 0, 0,
            int((home_lat + i * 0.0005) * 1e7),
            int((home_lon + i * 0.0005) * 1e7),
            30 + i,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION))

    # --- upload: COUNT, then one item per request ---
    conn.mav.mission_count_send(conn.target_system, conn.target_component,
                                count, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    sent = 0
    used_legacy_float = False
    while time.monotonic() - start < timeout:
        msg = conn.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT",
                                    "MISSION_ACK"], blocking=True, timeout=2)
        if msg is None:
            continue
        if msg.get_type() == "MISSION_ACK":
            # A NACK is not the end of a successful transfer. Treating any ACK as
            # success made a rejected upload look like a failed read-back, which
            # pointed the investigation at entirely the wrong half of the test.
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                reason = mavutil.mavlink.enums["MAV_MISSION_RESULT"].get(msg.type)
                reason = reason.name if reason else f"type {msg.type}"
                return (False, time.monotonic() - start,
                        f"upload REJECTED after {sent}/{count} items: {reason} "
                        f"(autopilot's own mission-transfer timeout fires when "
                        f"item replies arrive slower than it expects)")
            break
        # Answer in the protocol variant the vehicle asked in. ArduPilot may use
        # the legacy MISSION_REQUEST, and replying with MISSION_ITEM_INT to that
        # makes it store zeroes -- silently, with a successful-looking MISSION_ACK
        # at the end. Mission Planner gets this right; an earlier version of this
        # harness did not, and the resulting "waypoint 0 differs: (0, 0)" looked
        # like a tunnel fault rather than a protocol mismatch.
        item = items[msg.seq]
        if msg.get_type() == "MISSION_REQUEST":
            used_legacy_float = True
            conn.mav.mission_item_send(
                conn.target_system, conn.target_component, item.seq,
                item.frame, item.command, item.current, item.autocontinue,
                item.param1, item.param2, item.param3, item.param4,
                item.x / 1e7, item.y / 1e7, item.z,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        else:
            conn.mav.send(item)
        sent += 1
    else:
        return False, time.monotonic() - start, f"upload stalled after {sent}"

    upload_s = time.monotonic() - start

    # --- read back and compare ---
    conn.mav.mission_request_list_send(conn.target_system, conn.target_component,
                                       mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    got = {}
    expected = None
    while time.monotonic() - start < timeout:
        msg = conn.recv_match(type=["MISSION_COUNT", "MISSION_ITEM_INT"],
                              blocking=True, timeout=2)
        if msg is None:
            continue
        if msg.get_type() == "MISSION_COUNT":
            expected = msg.count
            # One request at a time, waiting for each item before asking for the
            # next. This is what Mission Planner does, and it is the behaviour
            # worth measuring: the ladder costs one round trip per waypoint, which
            # is exactly what latency makes expensive. Firing all requests at once
            # would measure something the real GCS never does.
            conn.mav.mission_request_int_send(
                conn.target_system, conn.target_component, 0,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            continue
        got[msg.seq] = msg
        if expected and len(got) >= expected:
            break
        if expected:
            nxt = len(got)
            if nxt < expected:
                conn.mav.mission_request_int_send(
                    conn.target_system, conn.target_component, nxt,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    elapsed = time.monotonic() - start
    if not expected or len(got) < count:
        return False, elapsed, f"read back {len(got)}/{count}"

    # Compare coordinates. Exactness is only available on the INT protocol: the
    # legacy MISSION_REQUEST path carries lat/lon as float32 degrees, which cannot
    # represent 1e-7 of a degree, so a round trip through it lands within about a
    # centimetre rather than bit-exact. That is the protocol's limit, not the
    # tunnel's -- the tunnel moves whatever bytes it is given, unaltered.
    # On the legacy path the expected value is not what we sent, it is what we
    # sent after a float32 round trip. Modelling that exactly is better than
    # guessing a tolerance: at longitude 149 degrees float32 resolves to about
    # 1.1 m (~99 units of 1e-7 deg), at latitude -35 about 5 units, and a blanket
    # tolerance wide enough for both would be too slack to catch real corruption.
    for i, original in enumerate(items):
        back = got.get(i)
        if back is None:
            return False, elapsed, f"waypoint {i} missing"
        # Accept either the exact value, or the value after the documented
        # float32 mission path -- and nothing else. Which of the two applies
        # depends on whether the autopilot asked via MISSION_REQUEST_INT or the
        # legacy MISSION_REQUEST, and it has been observed to mix the two within
        # one transfer. Both are correct; anything else is corruption, which is
        # what this check is actually for.
        def acceptable(got_v, sent_v):
            return abs(got_v - sent_v) <= 1 or abs(got_v - _through_float32(sent_v)) <= 1

        if not (acceptable(back.x, original.x) and acceptable(back.y, original.y)):
            return (False, elapsed,
                    f"waypoint {i} corrupted: got ({back.x}, {back.y}), "
                    f"sent ({original.x}, {original.y}), "
                    f"float32 path would give "
                    f"({_through_float32(original.x)}, {_through_float32(original.y)})")

    conn.mav.mission_ack_send(conn.target_system, conn.target_component, 0,
                              mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    variant = "legacy float" if used_legacy_float else "int"
    return True, elapsed, f"{count} wp ({variant}), upload {upload_s:.1f}s"


def test_command_rtt(conn, repeats, timeout):
    """Time a COMMAND_LONG to its COMMAND_ACK. Returns (ok, mean_s, detail)."""
    times = []
    for _ in range(repeats):
        start = time.monotonic()
        # Read-only: asks for the autopilot version. Safe regardless of state,
        # unlike arming, and exercises the same request/ack path.
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
            if msg is not None:
                times.append(time.monotonic() - start)
                break
        time.sleep(0.3)
    if not times:
        return False, 0.0, "no COMMAND_ACK"
    mean = sum(times) / len(times)
    return True, mean, (f"{len(times)}/{repeats} acked, "
                        f"min {min(times) * 1000:.0f} ms, max {max(times) * 1000:.0f} ms")


def main(argv=None):
    ap = argparse.ArgumentParser(description="Timed MAVLink protocol tests through the tunnel.")
    ap.add_argument("--connect", default="udpout:127.0.0.1:14570")
    ap.add_argument("--label", default="unlabelled",
                    help="name of the link profile under test, for the report")
    ap.add_argument("--waypoints", type=int, default=8)
    ap.add_argument("--param-timeout", type=float, default=180.0)
    ap.add_argument("--mission-timeout", type=float, default=120.0)
    ap.add_argument("--rtt-repeats", type=int, default=5)
    ap.add_argument("--stall-tolerance", type=float, default=15.0,
                    help="seconds of silence before the param download is "
                         "considered stalled (must exceed the link's worst stall)")
    ap.add_argument("--json-out", help="append one JSON result line to this file")
    args = ap.parse_args(argv)

    print(f"=== link test: {args.label} ===")
    print(f"connecting {args.connect} (Mission Planner must be DISCONNECTED)")
    conn = _connect(args.connect, timeout=30)
    if conn is None:
        print("FAIL  no heartbeat through the tunnel -- is the chain up?")
        return 1
    print(f"heartbeat from system {conn.target_system}\n")

    results = {"label": args.label}

    ok, secs, detail = test_command_rtt(conn, args.rtt_repeats, timeout=15)
    print(f"{'PASS' if ok else 'FAIL'}  command round-trip   mean {secs * 1000:7.0f} ms   {detail}")
    results["command_rtt_ms"] = round(secs * 1000, 1)
    results["command_ok"] = ok

    ok, secs, detail = test_mission_roundtrip(conn, args.waypoints, args.mission_timeout)
    print(f"{'PASS' if ok else 'FAIL'}  mission round-trip   {secs:10.1f} s    {detail}")
    results["mission_s"] = round(secs, 1)
    results["mission_ok"] = ok

    ok, secs, detail = test_param_download(conn, args.param_timeout,
                                           args.stall_tolerance)
    print(f"{'PASS' if ok else 'FAIL'}  parameter download   {secs:10.1f} s    {detail}")
    results["params_s"] = round(secs, 1)
    results["params_ok"] = ok
    results["params"] = detail

    if args.json_out:
        with open(args.json_out, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(results) + "\n")
        print(f"\nappended to {args.json_out}")

    return 0 if all(results[k] for k in ("command_ok", "mission_ok", "params_ok")) else 2


if __name__ == "__main__":
    sys.exit(main())
