# Phase 1c — behaviour under LTE-like conditions

**Date:** 2026-10-09 · ArduCopter SITL · impairment on the aircraft→broker hop only

Everything in Phase 1 ran at loopback latency, which cannot show how Mission
Planner's protocols behave on a real cellular link. This measures that.

## Method

`tools/link_shaper.py` sits between the aircraft bridge and the broker as a TCP
proxy, adding one-way delay, jitter and periodic stalls. Only that hop is
affected: SITL↔bridge is loopback, and the GCS bridge talks to the broker
directly, which mirrors reality (the ground side is ordinary internet).

`tools/link_test.py` drives the same three protocols Mission Planner uses and
times them. Scripted rather than clicked, because the deliverable is a
repeatable comparison across profiles.

```
aircraft_bridge --> link_shaper (impaired) --> broker <-- gcs_bridge <-- link_test
```

## Results

| Profile | One-way | RTT | Command ack | Mission 8 wp | Param download (1386) |
|---|---|---|---|---|---|
| direct | 0 | ~0 | **16 ms** | **0.8 s** | **0.7 s** |
| good | 25 ms ±5 | 50 ms | 76 ms | 1.3 s | 0.8 s |
| typical | 75 ms ±20 | 150 ms | 175 ms | 3.3 s | 1.1 s |
| bad | 200 ms ±80 | 400 ms | 707 ms | 9.1 s | 1.6 s |
| awful | 400 ms ±150 | 800 ms | 2345 ms (4/5) | **FAIL** | **FAIL / intermittent** |

`bad` and `awful` also inject stalls (1.2 s every 10 s, 3 s every 5 s) to stand
in for TCP recovering from loss.

## Findings

### 1. The mission protocol is the first thing to break, at ~800 ms RTT

It fails with `MAV_MISSION_INVALID_SEQUENCE`, and the reason matters: ArduPilot
re-requests an item when its own mission-transfer timeout fires, and on a slow
link the original reply is still in flight. The delayed reply then arrives
against the wrong sequence number and the transfer desynchronises. It is not
throughput, and not the tunnel dropping anything — it is a protocol whose
timeouts assume a fast link.

Up to 400 ms RTT it completes reliably, costing roughly one round trip per
waypoint: 8 waypoints took 0.8 s direct and 9.1 s at 400 ms. **Budget ~1.1 × RTT
per waypoint**, in each direction.

### 2. Parameter download is barely latency-sensitive

(Measured later with real loss: it is sensitive to loss **multiplied by** RTT,
not to loss on its own. See the tc netem section.)

0.7 s direct, 1.6 s at 400 ms RTT, for 1386 parameters. The autopilot streams
them one-way, so latency adds a fixed offset rather than a per-parameter cost.
This is the opposite of the usual assumption that the param download is the
fragile one.

What does break it is a lost or discarded request, which is finding 3.

### 3. Short message expiry may be discarding commands on a stalling link

At `awful`, `PARAM_REQUEST_LIST` sometimes never reaches the vehicle, and the
symptom is silence rather than an error. The aircraft bridge showed
`PARAM_VALUE=2` published in total, with `inbound_dropped=0`,
`inbound_malformed=0` and no disconnects — so the tunnel did not lose anything.
The vehicle was simply never asked.

`TO_VEHICLE_EXPIRY` is 3 s, and that profile stalls for 3 s, so a command can
plausibly exceed its expiry at the broker and be discarded by design.

**Evidence is suggestive, not conclusive.** Three runs each at `awful`:

| `TO_VEHICLE_EXPIRY` | Result |
|---|---|
| 3 s (default) | pass, pass, **fail** |
| 30 s | pass, pass, pass |

One failure in three against none in three is not proof. The mechanism is sound
and the mitigation is a single config value, but it warrants more trials before
being treated as established.

This also sharpens the open `TO_VEHICLE_FAIL_OPEN` decision: expiry is the
safety feature that stops a stale ARM arriving late, and the same feature can
silently drop a legitimate command on a degraded link. That trade-off is a team
call, not a default to pick quietly.

### 4. Command round-trip tracks link RTT, as expected

16 ms → 76 → 175 → 707 → 2345. Baseline plus RTT, with stalls inflating the mean
at the worst profiles. No surprises, which is itself worth knowing.

## Recommendations

1. **The link is comfortable to ~400 ms RTT** and unusable for missions beyond
   roughly 800 ms. Typical LTE (50–150 ms RTT) has ample margin.
2. **`TO_VEHICLE_EXPIRY` raised 3 s -> 30 s** on 2026-10-09, interim. 3 s was
   tight for cellular. This is a bench/simulator setting: with the dwell check
   still unbuilt, expiry is the only bound on delivered command age, so 30 s
   means a half-minute-old command can execute. Revisit alongside
   `TO_VEHICLE_FAIL_OPEN` once the dwell check can reject on measured age.
3. **Upload missions before relying on a degraded link**, or expect to retry.
   Mission transfer is the fragile protocol, not telemetry or parameters.
4. **MQTT framing overhead is ~190 % on small frames** — a 36-byte topic plus
   4 bytes of header to carry a 21-byte HEARTBEAT. MQTT 5 topic aliases reduce
   the topic to 4 bytes after the first publish. Worth doing before flight on a
   metered link (brief §9 Q5).

## Real packet loss (tc netem)

Run separately with `tc netem` on WSL `eth0`, with the aircraft bridge running
*inside* WSL against the Windows host so its broker traffic crossed the shaped
interface. This is below TCP, so it reproduces genuine retransmits and
congestion control, which the proxy cannot.

Baselines differ from the table above because the aircraft bridge sat in WSL
rather than on Windows (66 ms command ack rather than 16 ms), so compare within
this table, not across.

| netem profile | Command ack | Mission 8 wp | Param download |
|---|---|---|---|
| baseline (none) | 66 ms | 1.0 s | **0.8 s** |
| `delay 75ms 20ms loss 1%` | 201 ms | 3.7 s | 1.1 s |
| `delay 200ms 80ms loss 3%` | 447 ms | 8.5 s | **8.9 s** |
| `delay 25ms 5ms loss 5%` | 100 ms | 2.3 s | **1.2 s** |

**Nothing failed, even at 5 % loss.** TCP absorbs the loss and the application
sees delay, exactly as predicted — the bridges never saw a lost frame.

### Loss costs are proportional to round-trip time

This is the useful result, and it refines finding 2. Compare the last two rows:

- 3 % loss at 200 ms one-way → parameter download **8.9 s**, an 11x degradation
  on the 0.8 s baseline.
- **5 % loss** at 25 ms one-way → parameter download **1.2 s**, barely affected,
  despite nearly twice the loss rate.

Every retransmission costs one round trip, so loss is close to free on a short
link and expensive on a long one. Loss rate alone does not predict behaviour;
loss multiplied by RTT does.

The parameter download is the clearest example because it is a long one-way
burst with many chances to lose something. The mission protocol barely changed
(8.5 s here versus 9.1 s for comparable latency without loss), because it is
already round-trip bound and the retransmits hide inside the existing waits.

### Practical conclusion

The tunnel tolerates realistic cellular loss comfortably. **Latency, not loss,
is the thing to design against** — and the one hard limit found anywhere in this
work remains mission upload desynchronising at around 800 ms RTT.

## What this did NOT test

The shaper sits **above** TCP, so it can delay and stall but cannot drop
segments. It therefore does not reproduce congestion-window collapse or real
retransmit timers — only their visible effect, which is delay.

Real packet loss was tested separately with `tc netem` -- see the section above.
To reproduce it, run the aircraft bridge **inside WSL** pointed at the Windows
host (so the traffic crosses `eth0`), then in a WSL terminal:

```bash
# aircraft bridge, inside WSL, so its broker traffic crosses eth0:
cd /mnt/c/MissionPlanner-denel/Plugins/UAV_
~/venv-ardupilot/bin/python -u -m mqtt_bridge.aircraft_bridge \
    --mavlink tcp:127.0.0.1:5760 --broker-host $(ip route show default | awk '{print $3}') \
    --vehicle-id uav01

# in another WSL terminal (needs your sudo password):
sudo tc qdisc add dev eth0 root netem delay 75ms 20ms loss 1%
sudo tc qdisc change dev eth0 root netem delay 200ms 80ms loss 3%
sudo tc qdisc del dev eth0 root        # always clear up afterwards
```

Then re-run `tools/link_test.py` from Windows as above. Expect the parameter
download to degrade far more than it does here, since that is the loss-sensitive
protocol and loss is exactly what the proxy cannot emulate.

## Harness caveats, for whoever repeats this

- Measurements were taken on a machine with 7.9 GB RAM while SITL, WSL, Docker
  and the bridges all ran. Means varied by tens of percent between runs; the
  shape of the curve is reliable, individual numbers less so.
- `link_test.py` compares waypoints against either the exact value sent or the
  value after the legacy float32 mission path, because ArduPilot may request
  items via either `MISSION_REQUEST_INT` or the legacy `MISSION_REQUEST`, and has
  been observed to mix both within one transfer. At longitude 149° the float path
  resolves to about 1.1 m.
- `--stall-tolerance` must exceed the link's worst stall plus a round trip, or
  the harness gives up before the link does.
