"""A degraded-link emulator for the bridge-to-broker hop. No privileges needed.

Sits between a bridge and the broker as a plain TCP proxy and adds latency,
jitter and stalls:

    aircraft_bridge --> link_shaper (:18830) --> HiveMQ (:1883)

Run the bridge with --broker-port 18830 and everything else is unchanged.

WHY A PROXY RATHER THAN tc netem
--------------------------------
netem needs root, and it shapes a whole interface rather than one connection.
This shapes exactly the hop we care about, needs no sudo, and can be retuned
while a test is running.

WHAT THIS CANNOT DO, AND WHY IT MOSTLY DOES NOT MATTER
------------------------------------------------------
This sits ABOVE TCP, so it cannot drop segments and therefore cannot reproduce
congestion-window collapse or real retransmit timers. What it reproduces is what
the application actually experiences on a lossy link: added round-trip time, and
occasional stalls while TCP recovers. Since MQTT runs over TCP, a lossy link
never presents lost MAVLink frames to the bridge -- it presents delay. That is
the effect being measured here.

For the real thing, including congestion control, use tc netem on the WSL side
(needs sudo):

    sudo tc qdisc add dev eth0 root netem delay 75ms 20ms loss 1%
    sudo tc qdisc del dev eth0 root

ORDERING
--------
Release times are forced monotonic. Jitter applied naively would let a later
chunk overtake an earlier one, which cannot happen on a real TCP connection and
would make the emulation lie in a way that flatters the system under test.
"""

import argparse
import random
import socket
import sys
import threading
import time

PROFILES = {
    #              one-way ms, jitter ms, stall every N s, stall ms
    "perfect":    (0,    0,   0,    0),
    "good":       (25,   5,   0,    0),      # strong LTE
    "typical":    (75,   20,  30,   400),    # ~150 ms RTT, occasional recovery stall
    "bad":        (200,  80,  10,   1200),   # edge of coverage
    "awful":      (400,  150, 5,    3000),   # barely usable
}


class Direction:
    """One way of the proxy: read from src, release to dst after a delay.

    A queue plus a dedicated sender thread, rather than sleeping in the reader,
    so that a stall delays delivery without also stopping us reading from the
    socket -- which is what a real network does.
    """

    def __init__(self, src, dst, shaper, label):
        self.src = src
        self.dst = dst
        self.shaper = shaper
        self.label = label
        self.pending = []           # (release_monotonic, bytes)
        self.lock = threading.Condition()
        self.closed = False
        self.last_release = 0.0
        self.bytes_moved = 0

    def reader(self):
        try:
            while not self.closed:
                data = self.src.recv(65536)
                if not data:
                    break
                now = time.monotonic()
                delay = self.shaper.delay_for()
                release = now + delay
                with self.lock:
                    # Monotonic: a real TCP stream cannot reorder.
                    release = max(release, self.last_release)
                    self.last_release = release
                    self.pending.append((release, data))
                    self.lock.notify()
        except OSError:
            pass
        finally:
            self.close()

    def sender(self):
        try:
            while True:
                with self.lock:
                    while not self.pending and not self.closed:
                        self.lock.wait(0.1)
                    if self.closed and not self.pending:
                        return
                    release, data = self.pending[0]
                    wait = release - time.monotonic()
                    if wait > 0:
                        self.lock.wait(wait)
                        continue
                    self.pending.pop(0)
                try:
                    self.dst.sendall(data)
                    self.bytes_moved += len(data)
                except OSError:
                    return
        finally:
            self.close()

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.lock.notify_all()
        for s in (self.src, self.dst):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


class Shaper:
    def __init__(self, delay_ms, jitter_ms, stall_every_s, stall_ms):
        self.delay_ms = delay_ms
        self.jitter_ms = jitter_ms
        self.stall_every_s = stall_every_s
        self.stall_ms = stall_ms
        self._next_stall = (time.monotonic() + stall_every_s) if stall_every_s else None

    def delay_for(self):
        d = self.delay_ms + (random.uniform(-self.jitter_ms, self.jitter_ms)
                             if self.jitter_ms else 0.0)
        d = max(0.0, d)
        # A stall models TCP recovering from a loss: everything behind it waits.
        if self._next_stall is not None and time.monotonic() >= self._next_stall:
            self._next_stall = time.monotonic() + self.stall_every_s
            d += self.stall_ms
            print(f"  [shaper] stall {self.stall_ms} ms", flush=True)
        return d / 1000.0


def handle(client, upstream_addr, shaper, conn_id):
    try:
        upstream = socket.create_connection(upstream_addr, timeout=10)
    except OSError as exc:
        print(f"  [shaper] conn {conn_id}: upstream unreachable: {exc}", flush=True)
        client.close()
        return
    upstream.settimeout(None)
    client.settimeout(None)
    print(f"  [shaper] conn {conn_id} established", flush=True)

    up = Direction(client, upstream, shaper, "up")
    down = Direction(upstream, client, shaper, "down")
    threads = [
        threading.Thread(target=up.reader, daemon=True, name=f"up-r{conn_id}"),
        threading.Thread(target=up.sender, daemon=True, name=f"up-s{conn_id}"),
        threading.Thread(target=down.reader, daemon=True, name=f"dn-r{conn_id}"),
        threading.Thread(target=down.sender, daemon=True, name=f"dn-s{conn_id}"),
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    print(f"  [shaper] conn {conn_id} closed "
          f"(up {up.bytes_moved} B, down {down.bytes_moved} B)", flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Degraded-link emulator for the broker hop.")
    ap.add_argument("--listen-port", type=int, default=18830)
    ap.add_argument("--broker-host", default="127.0.0.1")
    ap.add_argument("--broker-port", type=int, default=1883)
    ap.add_argument("--profile", choices=sorted(PROFILES), default="typical")
    ap.add_argument("--delay-ms", type=float, help="override the profile's one-way delay")
    ap.add_argument("--jitter-ms", type=float, help="override the profile's jitter")
    args = ap.parse_args(argv)

    delay, jitter, stall_every, stall_ms = PROFILES[args.profile]
    if args.delay_ms is not None:
        delay = args.delay_ms
    if args.jitter_ms is not None:
        jitter = args.jitter_ms

    shaper = Shaper(delay, jitter, stall_every, stall_ms)
    print(f"profile {args.profile}: one-way {delay} ms +/- {jitter} ms "
          f"(~{delay * 2:.0f} ms RTT)"
          + (f", {stall_ms} ms stall every {stall_every} s" if stall_every else ""))

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", args.listen_port))
    listener.listen(8)
    print(f"listening on 127.0.0.1:{args.listen_port} -> "
          f"{args.broker_host}:{args.broker_port}\nCtrl+C to stop")

    conn_id = 0
    try:
        while True:
            client, _ = listener.accept()
            conn_id += 1
            threading.Thread(target=handle, daemon=True,
                             args=(client, (args.broker_host, args.broker_port),
                                   shaper, conn_id),
                             name=f"conn{conn_id}").start()
    except KeyboardInterrupt:
        print("\nshaper stopped")
    finally:
        listener.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
