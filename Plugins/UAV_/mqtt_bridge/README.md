# MAVLink-over-MQTT bridge

Carries MAVLink between the aircraft and the GCS through an MQTT broker, as a
long-range / BVLOS **backup** path over the aircraft's LTE uplink. It does not
replace the Herelink handheld link.

The MQTT hop is meant to be invisible. To Mission Planner the GCS-side bridge is
an ordinary MAVLink UDP peer; to the flight controller the aircraft-side bridge is
an ordinary MAVLink link. MQTT payloads are **raw MAVLink frame bytes** — nothing
here decodes or re-encodes, so MAVLink v2 signing survives end to end.

Requirements are in `../mavlink-mqtt-bridge-brief.md`. This README covers setup,
configuration, how to run the end-to-end test, and the known limitations.

```
 AIRCRAFT                        BROKER                        GCS

 Flight controller                                      Mission Planner
   |  ^                                                      ^  |
   v  |  MAVLink (serial/UDP/TCP)              MAVLink (UDP, loopback) |  v
 +-----------------+   MQTT 5   +-----------+   MQTT 5   +-----------------+
 | aircraft_bridge |<---------->|  HiveMQ   |<---------->|   gcs_bridge    |
 | pub from_vehicle|            | topic     |            | pub to_vehicle  |
 | sub to_vehicle  |            | routing   |            | sub from_vehicle|
 +-----------------+            +-----------+            +-----------------+
```

---

## Status

Phase 1 acceptance passed against ArduCopter SITL on 2026-10-02, with Mission
Planner unmodified:

- Telemetry, arm/disarm, mode changes, GUIDED takeoff, Fly-To-Here and RTL.
- Mission **write, read-back and AUTO execution** — the strictest protocol the
  tunnel carries, and the one that proves fidelity at the application level
  rather than only at the frame level.
- ~26,000 frames / 1 MB with `inbound_malformed=0`, `inbound_dropped=0` and
  **zero BAD_DATA** reported by a pymavlink client, i.e. every frame arrived
  byte-exact and CRC-valid.
- SITL emitted `msgid11030`, which is in no dialect either side knows, and the
  tunnel carried all of it — the no-CRC-validation design working as intended.

**Not yet implemented** (designed, deferred on purpose): Last Will / `status/*`
topics, broker-dwell staleness rejection, the local fan-out that feeds the STM32
bridge, TLS and broker authentication, and the Mission Planner launcher plugin.

**This is not flight-ready.** Everything above ran at loopback latency against a
simulator, through a broker with the Allow-All extension enabled — any client on
the machine can connect and send commands to the vehicle. Passing Phase 1 proves
the tunnel is transparent; it does not make the link safe for an aircraft that can
hurt someone.

---

## Install

```
pip install -r requirements.txt
```

Needs `paho-mqtt` 2.x specifically — the code uses `CallbackAPIVersion.VERSION2`
and MQTT 5 properties, neither of which exists in 1.x. The aircraft host does
**not** need `../requirements.txt`: that pulls in PySide6 for the operator GUI,
which a headless aircraft computer has no use for.

Files the aircraft needs, and nothing else:

```
mqtt_bridge/              logging_config.py        utils/connection_manager.py
```

### Phase 0: check the target machine before relying on any of this

A dependency probe tells you what is missing, not whether you are *allowed* to
fix it. Run these on the real GCS machine, as the operator's own account, before
building anything on top:

1. `python --version` — must print a version. If it offers to open the Microsoft
   Store, that is the Store stub, not a usable Python.
2. `pip install -r requirements.txt` — and note whether it actually succeeds
   under this machine's policies: outbound PyPI access, a proxy, TLS inspection,
   admin rights, AppLocker, antivirus quarantining newly written `.pyd` files.
3. `python -m mqtt_bridge.tools.mqtt_smoke --host <broker>` — proves an MQTT 5
   CONNECT is accepted, that publish and subscribe are permitted, and that binary
   payloads survive. A TCP port check proves none of those.
4. Expect a Windows Firewall prompt the first time the GCS bridge binds its UDP
   port. A non-admin operator may not be able to accept it.

If step 2 is blocked, vendor the wheels instead:

```
pip download -r requirements.txt -d packaging/offline-wheels      # on a machine with access
pip install --no-index --find-links packaging/offline-wheels -r requirements.txt
```

---

## Running the end-to-end test

Three terminals. **Start the GCS bridge before Mission Planner** — see the
Mission Planner note below for why.

```bash
# 1. WSL: SITL, serving MAVLink on tcp:5760
source ~/venv-ardupilot/bin/activate
cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter --no-mavproxy

# 2. WSL: aircraft bridge
cd /mnt/c/MissionPlanner-denel/Plugins/UAV_
~/venv-ardupilot/bin/python -u -m mqtt_bridge.aircraft_bridge \
    --mavlink tcp:127.0.0.1:5760 --broker-host localhost --vehicle-id uav01
```
```powershell
# 3. Windows: GCS bridge
cd C:\MissionPlanner-denel\Plugins\UAV_
python -u -m mqtt_bridge.gcs_bridge --broker-host localhost --vehicle-id uav01

# 4. Windows: prove frames arrive (acceptance #1)
python -m mqtt_bridge.tools.mav_probe --connect udpout:127.0.0.1:14570 --request-streams
```

`--request-streams` is what makes this a *bidirectional* test. A bare SITL emits
only HEARTBEAT and TIMESYNC until something asks it for more, so without the flag
you prove one direction. With it, the stream request travels GCS → broker →
aircraft → SITL and the extra message types coming back are the vehicle's response
to a command that crossed the tunnel. A healthy run looks like:

```
message types seen : 30
sources (sysid/compid): 1/1=1511
BAD_DATA frames    : 0
```

`BAD_DATA frames: 0` is the number that matters — it means every frame arrived
byte-exact *and* CRC-valid, verified by pymavlink's parser rather than by us.

### Two places a nonzero `bad_bytes` is expected, not a fault

The counters are printed on clean shutdown. `out_bad_bytes` is normally 0, with
two legitimate exceptions, both one-off at connect:

- **A connect driven by `AutoConnect`** contributes exactly **1** — the lone
  `0x00` NAT-punch byte that `AutoConnect.ProcessEntry()` sends on its outbound
  paths. A *manual* UDPCl connect from the Mission Planner UI does **not** do
  this: `UdpSerialConnect.Open()` sends no priming byte, so a hand-connected MP
  leaves `out_bad_bytes` at 0 (confirmed in testing).
- **Connecting to SITL over TCP** contributes a few dozen in a single
  `bad_events=1` run (35 bytes in testing). Attaching to `tcp:5760` lands
  mid-stream, so the partial frame in flight is discarded on resync. This is the
  framer accounting for bytes it could not use, which is exactly its job.

A `bad_events` count that keeps climbing during steady operation is the thing to
investigate.

### Shutting a bridge down

`Ctrl+C`, or `SIGTERM`. **Not** stdin EOF unless you passed `--stdin-shutdown`:
that is off by default on purpose, because "stdin is not a terminal" is equally
true of a held-open pipe and of `/dev/null`. systemd supplies `/dev/null`, so an
auto-arming stdin watcher makes the service exit the instant it starts — which is
what happened the first time this was launched detached, with "launcher closed
stdin" as the only clue. Only the Mission Planner launcher plugin, which holds the
pipe open, should pass the flag.

To use MAVProxy instead of a direct TCP connection, give it an output the bridge
can bind against: `sim_vehicle.py -v ArduCopter --console --out=udpout:127.0.0.1:14571`
and run the bridge with `--mavlink udpin:127.0.0.1:14571`.

Broker health and message rates: HiveMQ Control Center at <http://localhost:8080>.

### Connecting Mission Planner

Use **UDPCl**, not UDP: host `127.0.0.1`, port `14570`.

- `UDPCl` (`UdpSerialConnect`) dials out and returns immediately.
- `UDP` (`UdpSerial`) *binds* a local port and blocks in a 500 ms poll loop until
  a datagram arrives, so the Connect button hangs unless telemetry already flows.

Connecting by hand, Mission Planner's first datagram is a real MAVLink frame, so
`out_bad_bytes` stays at 0. (The single `0x00` priming byte belongs to
`AutoConnect.ProcessEntry()`, not to the UI's connect path — so do not treat a 1
there as expected, nor a 0 as a fault.)

`UdpSerialConnect.Open()` never blocks **and never fails** (`VerifyConnected()` is
an empty method), so pointing Mission Planner at a bridge that is not running
produces a confidently "connected" MP with no data. Start the bridge first.

---

## Configuration

Defaults live in `bridge_config.py` as plain `UPPER_SNAKE` constants. Precedence:

```
defaults  <  secrets.env  <  DENEL_MQTT_<KEY> environment  <  command-line flag
```

So `DENEL_MQTT_BROKER_HOST=10.0.0.5` overrides the default, and `--broker-host`
overrides that. An unparseable value logs an ERROR naming the key and falls back
to the default rather than refusing to start — one mistyped variable must not
ground an aircraft bridge.

Settings worth understanding rather than just reading:

| Key | Default | Why it is set this way |
|---|---|---|
| `GCS_UDP_BIND` | `127.0.0.1:14570` | Loopback, never `0.0.0.0`: Mission Planner's UDP inbound paths do no heartbeat validation, so a wildcard bind on a ground station is a LAN-facing MAVLink injection surface. 14570 avoids the contested range below. |
| `QOS_FROM_VEHICLE` | `0` | **Safety, not just bandwidth.** paho *discards* a QoS 0 publish while offline but *stores and re-sends* QoS ≥ 1 after reconnect. At QoS 1 every reconnect would deliver a burst of stale telemetry. |
| `QOS_TO_VEHICLE` | `1` | Commands, parameters and mission items must not be silently lost. |
| `CLEAN_START` / `SESSION_EXPIRY` | `True` / `0` | The primary stale-command defence: with no persistent session, the broker has nowhere to queue commands while the aircraft is offline. |
| `MAX_QUEUED_MESSAGES` | `256` | paho's own default is `0`, meaning **unlimited**. On `to_vehicle` that is an unbounded backlog of stale commands. |
| `KEEPALIVE` | `20` | An *ungraceful* drop is only noticed after roughly 1.5 × keepalive, so this is effectively the link-loss detection time (~30 s). Lower it for faster detection at the cost of more PINGREQ traffic on a metered link. Logged at startup. |
| `TO_VEHICLE_EXPIRY` | `3` | Short, so a command that sat at the broker expires rather than arriving late. |

Credentials go in `secrets.env` next to `bridge_config.py` (gitignored, see
`secrets.env.example`) or in the environment. Never commit a credential, key or
certificate.

### Ports: avoid 14550–14559 entirely

| Port | Owner |
|---|---|
| 14550 | Mission Planner's `AutoConnect` binds it at startup and never releases it. Also what `state/system_config.py` currently asks the STM32 bridge to bind — a known conflict, tracked separately. |
| 14551 | Reserved for the STM32 bridge (`AutoConnect` has a one-time migration that disables its own 14551 entry). |
| 14552–14559 | Reachable by `MavlinkWorker._connect_with_retry`, which walks the whole range when a port is refused. |
| **14570** | This bridge. |

This matters more than it looks: both .NET's `UdpClient` and pymavlink's `mavudp`
set `SO_REUSEADDR` before binding, so on Windows a **second bind succeeds** and
unicast datagrams reach only one socket. The result is silently half-dead
telemetry rather than a clean error. `UdpPeerEndpoint` deliberately does *not* set
`SO_REUSEADDR`, so a clash here raises and is reported, and the bridge warns at
startup if its configured port falls in the contested range.

---

## Tests

```
cd Plugins/UAV_ && python -m pytest
```

`test_framing.py` carries most of the weight. Framing is the only lossy component
in the whole chain — MQTT payloads are opaque binary and a UDP datagram is atomic,
so both hops preserve bytes by construction. Proving the framer byte-exact under
arbitrary segmentation (chunk sizes 1, 2, 3, 7, 11, 64 and 4096 across a mixed
corpus) therefore demonstrates tunnel integrity without needing a broker or SITL.

Two tests exist specifically to stop someone "improving" the framer:
`test_bad_crc_passes_through_unchanged` and `test_bad_crc_interop`, the latter
showing pymavlink rejecting a frame that the tunnel must still carry.

---

## Design notes

### Why the framer does not validate CRCs

Verifying a MAVLink CRC requires `crc_extra` from the compiled dialect. If the
flight controller's ArduPilot build and our pinned pymavlink disagree about any
message's `crc_extra` — a routine consequence of upgrading one and not the other —
every frame of that one message type would be silently discarded, permanently.

A raw tunnel must have no opinion about payload semantics. The framer reads only
the start byte, the length byte, and the incompat flags; it rejects reserved
incompat bits (the one validity check that needs no dialect) and nothing else.
Integrity is the endpoints' business, and authenticity is MAV_SIGNING's — both of
which keep working precisely because the bytes are never touched.

The cost: a garbage byte equal to `0xFD`/`0xFE` is read as a frame start and its
bogus length can swallow the genuine frame behind it. That is self-correcting and
visible in `bad_bytes`, whereas a dialect-mismatch drop is neither. On datagram
transports `feed_datagram()` discards any leftover at the end of each datagram, so
one bogus length cannot eat the following datagrams too.

### Why not `mavudp` on the GCS side

`mavudp.__init__` defaults `timeout=0` and `mavlink_connection()` never forwards a
timeout, so `self.timeout` is always 0. In `mavudp.write()` the guard
`self.timeout <= 0` is then permanently true: it sends to **every peer it has ever
heard from**, and the stale-peer eviction branch below it is unreachable. Mission
Planner reconnects on a fresh ephemeral port each time, so telemetry would be
duplicated to every dead socket forever. `UdpPeerEndpoint` uses an explicit
last-peer policy and logs peer changes.

---

## Known limitations

- **No staleness rejection yet.** `clean_start` plus a short message expiry are in
  place, but the receive-side broker-dwell check is not. Do not rely on this build
  to reject an old command.
- **Measured age is not end-to-end age.** When the dwell check lands, it will
  measure broker dwell only. Time spent in paho's own out-queue, and delay from
  TCP retransmission over LTE, are not visible to any MQTT property. Measuring
  true end-to-end age needs clock synchronisation between aircraft and GCS, which
  is not available. The queue is bounded instead.
- **The STM32 bridge is not fed.** `main.py` is a second MAVLink consumer on the
  GCS and it currently gets nothing from this path. It needs telemetry *and*
  commands, not either alone: `UAVCommandSender._do_connect()` blocks in
  `_heartbeat()` until a vehicle HEARTBEAT arrives, and because its connection is
  `udpin:`, `mavudp.write()` only sends to peers learned in `recv()` — so with no
  inbound traffic its commands vanish into an empty client set with no error and
  no log. Local fan-out is the next piece of work.
- **A dead tunnel is invisible to the Python side.** Its watchdog monitors thread
  liveness, not MAVLink liveness (`MavlinkWorker._loop` pets it unconditionally),
  so `is_UAV_Command_Connection_Available` stays `True` while nothing flows.
- **QoS 1 can deliver duplicates.** Not deduplicated by design: a naive dedupe
  drops legitimately repeated frames. QoS is configurable.
- **MQTT framing costs ~190% on small frames.** The topic
  `denel/uav/uav01/mavlink/from_vehicle` is 36 bytes; with the fixed header and
  topic-length field that is 40 bytes of framing to carry a 21-byte HEARTBEAT.
  MQTT 5 topic aliases reduce this to 4 bytes after the first publish. Measure
  over real LTE before optimising.
- **No TLS and no authentication.** The dev broker runs the Allow-All extension:
  any client may connect and publish or subscribe to anything. Dev machines only.
- **Untested under latency.** Everything above runs at loopback speed, which
  cannot show how Mission Planner's parameter and mission protocols behave at
  150 ms+ round trips with jitter and loss.

## Open questions carried from the brief

- **Q1, where the aircraft bridge runs:** no assumption made. `AIRCRAFT_MAVLINK`
  accepts any pymavlink connection string, and both the serial and the UDP failure
  modes are handled. Only the systemd unit's `After=` depends on the answer.
- **Q4, two GCS paths to one flight controller:** partly answered already. The
  STM32 bridge presents `(sysid 255, compid 0)` and Mission Planner
  `(255, 190)` — pymavlink defaults, never overridden — and neither sends a
  HEARTBEAT of its own. The bridge counts frames per `(sysid, compid)` so the
  decision can be made on data. It deliberately does **not** filter: that would
  break transparency.
- **Q2, Q3, Q5, Q6** (HiveMQ edition and security path, hosting and sovereignty,
  LTE bandwidth budget, which commands the backup link may carry) remain open.
