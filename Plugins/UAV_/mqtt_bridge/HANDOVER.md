# MAVLink-over-MQTT bridge — handoff

**Date:** 2026-10-30 · **Branch:** `MQTT` · **Head:** `b381d162a` plus uncommitted Phase 3 work
**Status:** Phases 0, 1, 1b, 1c, 2 complete and committed. **Phase 3 (security) in progress:**
client-side TLS/credentials done, secured test broker running on Azure, 2 of 5 negative tests passed.
Bridge installed and tested on the real XBLink. **Uncommitted:** `tls.py`, `tests/test_tls.py`,
changes to `mqtt_link.py`, `cli.py`, `bridge_config.py`, `tools/mqtt_smoke.py`, docs, and `broker/azure/`.
**Tests:** 186 pass, 1 skipped on Linux (`cd Plugins/UAV_ && python -m pytest`)

Read `README.md` for day-to-day usage and the config reference, and
`LINK-TESTING.md` for measured behaviour under latency and loss. This file is
for picking the work up cold.

---

## 1. What this is

A long-range / BVLOS **backup** MAVLink link for the Denel FW-VTOL 250, carrying
MAVLink between the aircraft and the ground station through an MQTT broker over
the aircraft's XBLink LTE uplink. It does **not** replace the Herelink handheld
link. The authoritative spec is `../mavlink-mqtt-bridge-brief.md`.

Two mirrored processes. The broker only routes by topic and never decodes
MAVLink.

```
 AIRCRAFT                          BROKER                       GCS

 Flight controller                                       Mission Planner
   |  ^                                                        ^  |
   |  | MAVLink (serial/UDP/TCP)            MAVLink (UDP, loopback) |
   v  |                                                        |  v
 +------------------+  MQTT 5  +-----------+  MQTT 5  +------------------+
 | aircraft_bridge  |<-------->|  HiveMQ   |<-------->|   gcs_bridge     |
 | pub from_vehicle |          | topic     |          | pub to_vehicle   |
 | sub to_vehicle   |          | routing   |          | sub from_vehicle |
 +------------------+          +-----------+          +--------+---------+
                                                                |
                                           LocalHub fans out to every local
                                           consumer and merges their commands:
                                                      |              |
                                            Mission Planner     main.py
                                            (UDPCl 14570)       (STM32 panel)
```

---

## 2. Architecture decisions, and why

These were deliberate and are pinned by tests. Do not reverse them without
reading the reasoning.

### The framer does not validate CRCs

`framing.py` reads only the start byte, the length byte and the incompat flags.
It never checks the CRC, the msgid, or the dialect.

Verifying a MAVLink CRC needs `crc_extra` from the compiled dialect. If the FC's
ArduPilot build and our pinned pymavlink disagree about any message's
`crc_extra` — routine when one is upgraded and not the other — every frame of
that one message type would be silently discarded, permanently. A raw tunnel
must have no opinion about payload meaning.

Pinned by `test_bad_crc_passes_through_unchanged` and `test_bad_crc_interop`
(the latter shows pymavlink rejecting a frame the tunnel must still carry). If
someone "fixes" the framer, those fail and explain why.

Observed in practice: SITL emits `msgid11030`, which is in no dialect either end
knows, and the tunnel carried all of it.

The cost, accepted: a garbage byte equal to `0xFD`/`0xFE` is read as a frame
start and its bogus length can swallow the real frame behind it. Self-correcting
and visible in `bad_bytes`. On datagram transports `feed_datagram()` discards
per-datagram residue so one bogus length cannot eat following datagrams.

### Our own UDP socket on the GCS side, not `mavudp`

`mavudp.__init__` defaults `timeout=0` and `mavlink_connection()` never forwards
a timeout, so `self.timeout` is always 0. In `mavudp.write()` the guard
`self.timeout <= 0` is then permanently true: it sends to **every peer it has
ever heard from**, and the eviction branch below is unreachable. Mission Planner
reconnects on a fresh ephemeral port each time, so telemetry would be duplicated
to every dead socket forever. `UdpPeerEndpoint` uses an explicit last-peer
policy and logs peer changes.

### `UdpPeerEndpoint` deliberately does NOT set `SO_REUSEADDR`

Both .NET's `UdpClient` and pymavlink's `mavudp` do. On Windows that makes a
second bind on a taken UDP port **succeed** while datagrams reach only one
socket, arbitrarily, with no error anywhere. Without the flag a clash raises and
is reported.

### `QOS_FROM_VEHICLE = 0` is a safety setting, not a bandwidth one

Verified in paho 2.1.0 `client.py`: a QoS 0 publish while offline returns
`MQTT_ERR_NO_CONN` and is **discarded** (line 1772); a QoS >= 1 publish enters
`_out_messages` and is explicitly re-sent after reconnect (lines 1779-1807).
Raising this to 1 would turn a dropped telemetry frame into a burst of stale
telemetry after every reconnect.

### paho's out-queue is bounded explicitly

Its default `max_queued_messages` is **0, meaning unlimited**. On `to_vehicle`
(QoS 1) that is an unbounded backlog of stale commands. Set to 256 queued and
10 in-flight (lowered from 20 on 2026-11-10: HiveMQ's Receive Maximum is 10,
paho 2.1 ignores it, and exceeding it disconnects the client).

### The stdin shutdown watcher is opt-in

"stdin is not a terminal" is equally true of a held-open pipe and of closed
stdin or `/dev/null`. **systemd gives a service `/dev/null`**, so an auto-arming
watcher reads EOF and the bridge exits the instant it starts. Found the hard
way. Only the Mission Planner launcher, which holds the pipe open, passes
`--stdin-shutdown`.

### Stream EOF detection

A closed TCP socket reports as *readable* forever and pymavlink's `recv()`
returns empty rather than raising, so a naive reader loop spins and never
notices the FC died. Treated as EOF after 2 s on **stream** endpoints only — on
a datagram endpoint an empty read is unremarkable and reconnecting would flap a
healthy socket. Pinned by `test_eof_detection.py`.

### Stale-command posture: availability-first (Taariq's call, 2026-10-13)

`TO_VEHICLE_MAX_AGE=30`, `TO_VEHICLE_FAIL_OPEN=True`,
`REQUIRE_LINK_ONLINE_FOR_COMMANDS=False`. **Interim, not a flight setting.**

The real protection is layer 1: the aircraft connects `clean_start=True` with
session expiry 0, so the broker has no session to queue into and a command
published while the aircraft is offline is discarded at publish time. Verified —
acceptance test 4 passes with `dropped_stale=0`, i.e. the dwell check never
fires.

The dwell check measures **broker dwell only**: `dwell = original_expiry -
remaining_expiry`, using MQTT 5's requirement that the broker decrement
`MessageExpiryInterval`, with the original carried in an `exp` user property. No
clock sync needed, because none is available. It does **not** see time in our
own paho out-queue, nor TCP retransmission delay.

---

## 3. Broker setup

### Local dev broker (Docker on the dev PC)

| | |
|---|---|
| Image | `hivemq/hivemq4:latest` (HiveMQ 4 Community, free "Lab" licence: 100 connections, 100 msg/s) |
| Container name | `hivemq` |
| Runtime | Docker Desktop on Windows, WSL2 backend |
| MQTT port | **1883**, plain TCP, no TLS |
| Control Center | **8080** — <http://localhost:8080> |
| Reachable from | Windows `localhost:1883`; WSL `localhost:1883` **and** the host IP (was `172.21.32.1` — re-check with `ip route show default \| awk '{print $3}'`, it changes between WSL restarts) |
| Protocol | MQTT **5** only (`protocol=mqtt.MQTTv5`) |

The local dev broker is **open**: Allow-All extension, plaintext 1883, no
certificates. It is reachable from the whole dev network (no Windows Firewall
rule was needed for the XBLink to reach it), so stop it when not testing.

### The secured test broker (Azure), since 2026-10-29

Full build runbook and config files: `broker/azure/`. Summary:

| | |
|---|---|
| Host | Azure VM `TDC-Cloud`, resource group `BVLOS-COMS`, South Africa North, Ubuntu 24.04, 1 GiB RAM + 2 GB swap |
| Name | `hivemq-broker.southafricanorth.cloudapp.azure.com` |
| Subscription | Azure for Students (personal, Taariq). **Test data only.** |
| Broker | `hivemq/hivemq4` 4.56.1 in Docker, container `hivemq`, trial licence (25 connections), heap 512 MB |
| Listener | **TLS only, 8883**, TLS 1.3/1.2. No 1883. |
| Certificate | Let's Encrypt for the DNS name, **expires 2027-01-27**, converted to `~/hivemq/certs/broker.jks`. Renewal is manual. |
| Auth | File RBAC extension 4.6.19, hashed passwords (PBKDF2-SHA512, 100,000 iterations). Allow-All removed. If the extension fails, HiveMQ denies everyone. |
| Users | `uav01-aircraft`, `uav01-gcs`, `smoke-test`. Permissions in `broker/azure/credentials.xml.template`. |
| Firewall | Inbound 8883 from anywhere; SSH 22 from the office IP only. Control Center on the VM's localhost only (SSH tunnel). |

**Gotcha:** HiveMQ substitutes `${ENV:...}` placeholders inside XML comments
too. An unknown name stops the broker starting.

### Client-side TLS and credentials, since 2026-10-28

`tls.py` (stdlib only) holds the rules; `MqttLink` applies them via
`tls_set_context()`; `cli.bootstrap()` refuses to start on a TLS problem.

- Certificate and hostname verification always on; no insecure switch. TLS 1.2 floor.
- `TLS_CA_FILE` empty = system trust store (right for Let's Encrypt / public
  brokers). When set it **replaces** the system store.
- Missing TLS files, or certificate settings with TLS off, refuse to start. Never
  falls back to plaintext.
- Flags `--tls --tls-ca-file --tls-cert-file --tls-key-file --username`.
  **No `--password` flag** by design; password from `secrets.env` or
  `DENEL_MQTT_MQTT_PASSWORD`.
- paho swallows TLS errors in its retry loop (`ssl.SSLError` is an `OSError`),
  so `on_connect_fail` logs `MQTT could not reach ...` on the 1st, 10th, 20th...
  failure. `mqtt_smoke` connects synchronously and prints the exact reason.
- Startup logs one `broker transport:` line; never the password.
- `secrets.env` is gitignored. No credential has ever been committed.

File RBAC disallows `#` and `+` in client ids and usernames, which
`validate_vehicle_id()` already rejects. The Enterprise Security Extension
remains an alternative for production; that decision is open.

### Topics

Namespaced by vehicle from day one, for multi-aircraft support and per-vehicle
ACLs. With `VEHICLE_ID=uav01` and `TOPIC_PREFIX=denel/uav`:

| Topic | Direction | QoS | Retained |
|---|---|---|---|
| `denel/uav/uav01/mavlink/from_vehicle` | aircraft to GCS | 0 | no |
| `denel/uav/uav01/mavlink/to_vehicle` | GCS to aircraft | 1 | no |
| `denel/uav/uav01/status/link` | aircraft bridge | 1 | **yes** |
| `denel/uav/uav01/status/gcs` | GCS bridge | 1 | **yes** |

Separate from/to topics are the primary loop prevention; `noLocal=True` on every
subscription is a second guard.

Status topics carry `online`/`offline`, retained so a late subscriber learns the
state immediately. **A graceful MQTT DISCONNECT suppresses the Last Will**, so
`MqttLink.stop()` publishes `offline` itself and waits for the PUBACK before
disconnecting — without that a clean shutdown would leave the status reading
`online` forever.

### Measured MQTT overhead

`denel/uav/uav01/mavlink/from_vehicle` is 36 bytes; plus a 2-byte fixed header
and a 2-byte topic-length field = **40 bytes of framing to carry a 21-byte v2
HEARTBEAT, about 190% overhead**. MQTT 5 topic aliases cut the topic to 4 bytes
after the first publish. Worth doing before flight on a metered link (brief Q5).

---

## 4. Ports — who owns what

| Port | Owner | Notes |
|---|---|---|
| 1883 | HiveMQ (local dev) | plain MQTT, Allow-All. The Azure broker uses 8883 TLS only. |
| 8080 | HiveMQ Control Center | web UI |
| 5760 | ArduPilot SITL | TCP; also serves 5762/5763 |
| **14550** | Mission Planner `AutoConnect` | **CONTESTED — never use.** Bound at MP startup and never released. Its UDP inbound path does **no** heartbeat validation, so anything sent there can promote the socket into a phantom extra vehicle. |
| **14551** | STM32 panel (`main.py`) | Reserved. `AutoConnect.cs` disables MP's own 14551 entry to keep it free. `state/system_config.py` binds it. |
| 14552-14559 | nobody | Avoid: `MavlinkWorker._connect_with_retry` walks this range. |
| **14570** | MQTT GCS bridge | Mission Planner connects with **UDPCl**, not UDP. |
| 14571 | aircraft bridge (dev default) | `udpin:127.0.0.1:14571`, only when using MAVProxy rather than a direct TCP connection to SITL |

`bridge_config.warn_about_contested_port()` logs a WARNING if a bound port lands
in 14550-14559.

**Why UDPCl and not UDP in Mission Planner:** `UdpSerialConnect.Open()` dials out
and returns immediately. `UdpSerial.Open()` *binds* and blocks in a 500 ms poll
loop until a datagram arrives, hanging the Connect button. Also note
`UdpSerialConnect.Open()` never blocks **and never fails** (`VerifyConnected()`
is an empty method), so pointing MP at a bridge that is not running gives a
confidently "connected" MP with no data. **Start the bridge first.**

---

## 5. Files

All under `Plugins/UAV_/mqtt_bridge/` — about 40 files, roughly 5,600 lines including
tests.

### Pure logic (no I/O, fully unit-tested)

| File | Purpose |
|---|---|
| `framing.py` | **The core.** Byte stream to whole MAVLink frames. `MavlinkFramer.feed()` for streams, `feed_datagram()` for UDP. v1 total = `6+len+2`; v2 = `10+len+2` (+13 when signed); max 280. Counts `bad_bytes`/`bad_events`, never silently drops. |
| `topics.py` | `TopicSet`, `parse_mavlink_topic()`, `validate_vehicle_id()` (rejects `#`, `+`, `/`, over 32 chars — the Phase 3 RBAC constraint), `topic_overhead_bytes()`. |
| `staleness.py` | `age_from_expiry()`, `is_stale()`, `describe_posture()`. Broker-dwell only; documents what it cannot see. |
| `tls.py` | TLS and credential rules: `tls_config_problems()` (refuse to start), `tls_config_warnings()`, `build_ssl_context()`, `describe_transport()`. Standard library only. |
| `bridge_config.py` | Defaults plus a pure `build_config(defaults, secrets, env, cli)`. Precedence: defaults < `secrets.env` < `DENEL_MQTT_*` < CLI. Bad values log ERROR and fall back. Also `resolve_log_path()` and `warn_about_contested_port()`. |

### I/O

| File | Purpose |
|---|---|
| `mqtt_link.py` | paho 2.x wrapper, `CallbackAPIVersion.VERSION2` + MQTTv5. Connect/reconnect, re-subscribe in `on_connect` every time, Last Will, retained status, bounded out-queue, duplicate-client-id flap detection. |
| `mavlink_endpoint.py` | `MavutilEndpoint` (aircraft; mavutil used as a byte pipe only — never `recv_msg`/`recv_match`/`wait_heartbeat`) and `UdpPeerEndpoint` (GCS; own socket, last-peer policy). |
| `local_hub.py` | `LocalHub` + `UdpSendEndpoint` + `parse_peer_specs()`. Fans telemetry to every local consumer, merges their commands, round-robins reads so one chatty peer cannot starve another. |
| `bridge_core.py` | `BridgeBase`: reader/writer/metrics threads, bounded queues, EOF detection, idempotent shutdown, counters, per-`(sysid,compid)` and per-msgid tallies. |
| `cli.py` | Shared arguments and `bootstrap()` — logging is configured **first**, before config parsing, so early failures are captured. |
| `aircraft_bridge.py`, `gcs_bridge.py` | Entry points. Differ only in which topic they publish to versus subscribe to, and the aircraft sets `check_staleness = True`. |

### Tools (`tools/`, all ship to operator machines)

| File | Purpose |
|---|---|
| `mqtt_smoke.py` | **Run this first when something is wrong.** Proves an MQTT 5 CONNECT is accepted and a binary payload round-trips byte-for-byte. A TCP port check proves none of that. |
| `mav_probe.py` | Stand-in for Mission Planner. `--request-streams` makes it bidirectional. Identifies as 255/190 like MP. |
| `status_watch.py` | Watches the retained status topics. `--expect offline` for scripted acceptance checks. |
| `inject_command.py` | Publishes a real encodable frame straight to `to_vehicle`. The stale-command test harness. `--command arm` — simulator only. |
| `link_shaper.py` | TCP proxy adding delay/jitter/stalls to the broker hop. No privileges needed. Profiles: `perfect`, `good`, `typical`, `bad`, `awful`. |
| `link_test.py` | Timed versions of the three Mission Planner protocol tests, for comparing link profiles. |

### Other files in the package

`packaging/denel-mqtt-aircraft-bridge.service` (systemd template, two variants,
passes `systemd-analyze verify`) · `requirements.txt` (`pymavlink==2.4.49`,
`paho-mqtt==2.1.0` — deliberately no PySide6, the aircraft is headless) ·
`requirements-dev.txt` (`pytest==8.4.2`) · `secrets.env.example` ·
`README.md` · `LINK-TESTING.md` · `tests/` (9 files, 187 tests) ·
`broker/azure/` (HiveMQ config, File RBAC template, VM runbook for the secured test broker).

### Outside the package

- `Plugins/DenelMqttBridgeLauncher.cs` — starts the GCS bridge with Mission
  Planner. **Default OFF**; enable by creating `mqtt_bridge/enabled.txt`.
  Self-contained rather than sharing code with `DenelPythonLauncher` (whose
  single-instance `_stopped` flag would orphan one of two children). Probes that
  `paho.mqtt.client` and `pymavlink` actually *import*, not just that python
  runs. **Plugins are compiled at RUNTIME by `CodeGenRoslyn`, so the solution
  build does not check them** — verify with the Roslyn csc, see section 6.
- `MissionPlanner.csproj` — `plugins\UAV_\**\*.py` (recursive, `tests/`
  excluded), plus explicit items for both `requirements.txt` files, the plugin
  `.cs`, and the `.service`. Non-`.py` files need their own entry or they never
  reach the release ZIP.
- `Plugins/UAV_/state/system_config.py` — binds **14551** (moved off 14550).
- `Plugins/UAV_/logging_config.py` — `controller.log` anchored to the module
  directory rather than the CWD.
- Root `CLAUDE.md` — has an MQTT section with the port-ownership table.

---

## 6. How to run and test

### Unit tests

```
cd C:\MissionPlanner-denel\Plugins\UAV_
python -m pytest            # 186 pass (1 Windows-only skip elsewhere), about 6 s
```

`pytest.ini` lives at `Plugins/UAV_` so rootdir does not walk `ExtLibs/`.

### Full stack

```powershell
# 0. Broker (Docker Desktop must be running)
docker start hivemq
python -m mqtt_bridge.tools.mqtt_smoke --host localhost     # sanity: expect PASS
```

```bash
# 1. SITL, in WSL  (a clean build takes about 5 minutes)
cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter --no-mavproxy
```

```powershell
# 2. Aircraft bridge  (Windows reaches WSL's SITL on localhost)
cd C:\MissionPlanner-denel\Plugins\UAV_
python -u -m mqtt_bridge.aircraft_bridge --mavlink tcp:127.0.0.1:5760 --broker-host localhost --vehicle-id uav01

# 3. GCS bridge, second window — START BEFORE MISSION PLANNER
cd C:\MissionPlanner-denel\Plugins\UAV_
python -u -m mqtt_bridge.gcs_bridge --broker-host localhost --vehicle-id uav01

# 4. Verify without the GUI
python -m mqtt_bridge.tools.mav_probe --connect udpout:127.0.0.1:14570 --request-streams
```

Then Mission Planner: dropdown **UDPCl**, CONNECT, host `127.0.0.1`, port
`14570`.

**Start exactly one of each bridge.** Two instances share an MQTT client id, the
broker kicks one, it reconnects and kicks the other — a flap loop that reads as
a broker fault. The bridge detects three or more disconnects a minute and names
the cause. Check first:

```powershell
Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Where-Object { $_.CommandLine -match 'mqtt_bridge' }
```

### Degraded-link testing

```powershell
python -u -m mqtt_bridge.tools.link_shaper --profile typical --listen-port 18830
# then point the aircraft bridge at --broker-host 127.0.0.1 --broker-port 18830
python -u -m mqtt_bridge.tools.link_test --label typical
```

Real packet loss needs `tc netem` in WSL (root): run the aircraft bridge
*inside* WSL against the Windows host IP so traffic crosses `eth0`, then
`sudo tc qdisc add dev eth0 root netem delay 75ms 20ms loss 1%`. **Always
`sudo tc qdisc del dev eth0 root` afterwards** or all WSL networking stays
degraded. Full recipe in `LINK-TESTING.md`.

### Verifying the C# plugin compiles

The solution build does **not** check it. Use the same compiler MP does:

```powershell
cd C:\MissionPlanner-denel\bin\Release\net461
& "C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\Roslyn\csc.exe" `
  /nologo /target:library /langversion:latest /out:"$env:TEMP\pc.dll" `
  /r:MissionPlanner.exe /r:System.Windows.Forms.dll /r:System.dll /r:System.Core.dll /r:System.Drawing.dll `
  plugins\DenelMqttBridgeLauncher.cs
```

Do **not** use `C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe` — it is
C# 5 and chokes on expression-bodied members, failing on known-good files too
and sending you after a non-existent bug.

### What healthy looks like

A metrics line every 10 s, only when something changed:

```
frames_out=1911 frames_in=193 ... inbound_dropped=0 inbound_malformed=0
dropped_stale=0 write_failures=0 mqtt_connects=1 mqtt_disconnects=0
out_bad_bytes=0 sources[1/1=1913] sent[HEARTBEAT=91 ...] | mqtt=up peer=...
```

Watch: `inbound_malformed` and `inbound_dropped` must stay 0; `mqtt_disconnects`
must not climb; `peer=` shows an address once MP connects. `out_bad_events`
ticking up by one per TCP reconnect is normal — attaching to a TCP stream
mid-frame discards a partial. Climbing during steady flight is not.

Logs: `C:\ProgramData\Denel GCS\denel_mqtt_bridge_gcs.log` and
`denel_mqtt_bridge_aircraft.log`. Override the directory with
`DENEL_MQTT_LOG_DIR`.

---

## 7. Status

| Phase | State |
|---|---|
| 0 — environment feasibility | done |
| 1 — raw tunnel, Mission Planner works | done, all 4 acceptance tests |
| 1b — STM32 fan-out | done, verified against the real `main.py` |
| 1c — LTE characterisation | done, see `LINK-TESTING.md` |
| 2 — robustness | done, all 6 acceptance tests |
| 1d — first run on the XBLink | done 2026-10-28/29: smoke test and aircraft bridge on the Pi; TLS round trip over LTE to Azure |
| **3 — security (TLS, auth, ACLs)** | **in progress**: client side done; Azure broker with File RBAC done; 2 of 5 negative tests passed |
| 4 — semantic JSON fan-out | optional, after 3 |

Verified against ArduCopter SITL with Mission Planner **unmodified**: telemetry,
arm/disarm, mode changes, GUIDED takeoff, Fly-To-Here, RTL, mission
write/read-back/AUTO execution, parameter download (1386 params), broker-restart
recovery, FC-loss recovery, Last Will, clean-shutdown status, and the
stale-command test with a positive control. Zero BAD_DATA across about 26,000
frames.

Fan-out verified with the real panel software: it logged `UAVCommandSender
heartbeat received - system 1 component 1` and `UAVCommandSender connected.`
with telemetry arriving only through the tunnel, then sent 296 commands up it
while a Mission Planner stand-in ran concurrently — `sources[255/0=296
255/190=5]`, `inbound_dropped=0`, `write_failures=0`.

### Measured limits

- **Mission upload breaks at about 800 ms RTT** with
  `MAV_MISSION_INVALID_SEQUENCE`: ArduPilot re-requests an item before the reply
  arrives and the transfer desynchronises. Reliable to 400 ms. Budget roughly
  1.1 x RTT per waypoint.
- **Packet loss is tolerated to at least 5%** — TCP absorbs it and the bridges
  never see a lost frame. The cost scales with RTT, not with loss rate: 3% at
  200 ms made the parameter download 11x slower, while 5% at 25 ms barely
  registered.
- **Parameter download is barely latency-sensitive** (0.7 s direct, 1.6 s at
  400 ms RTT for 1386 parameters) — the autopilot streams them one-way.

---

## 8. What is left, and what is not safe yet

### NOT FLIGHT READY. Three reasons.

1. **Broker security is only partly proven.** The Azure test broker has TLS,
   logins and topic ACLs, but three negative tests remain, the dev broker is
   still open, and the test broker is on a personal subscription.
2. **Staleness protection is availability-first and interim.** With
   `TO_VEHICLE_EXPIRY=30` and fail-open, a command delayed up to 30 s can still
   execute. Deliberate, and flagged for the team to revisit.
3. **No real flight controller yet.** MAVLink has only flowed with SITL. The
   XBLink and real LTE have carried the smoke test and an idle aircraft bridge,
   but not MAVLink from a Cube.

### Phase 3 — the remaining work

Done: client-side TLS and credentials; broker-side File RBAC with per-vehicle
ACLs on the Azure test broker; negative test 1 (wrong credentials refused) and a
topic-outside-role refusal, both run from the XBLink over LTE on 2026-10-29.

Remaining negative tests: an aircraft-role client cannot publish to
`to_vehicle`; a GCS-role client cannot publish to `from_vehicle`; vehicle A
cannot reach vehicle B (needs `uav02` users); untrusted CA refused against the
real broker (already shown against a local test server). Tests 2-3 need TLS and
`--username` support in `inject_command.py`; `status_watch.py` and
`link_test.py` need the same.

Also: automate certificate renewal before 2027-01-27; change the Control Center
default login; mount the Lab licence; run the GCS bridge against the Azure broker.

Plus short threat-model notes: MQTT auth/ACL protects the broker path,
MAV_SIGNING protects frame authenticity end to end, and the two are
complementary.

### Open questions — not for a Claude session to decide

- `TO_VEHICLE_FAIL_OPEN` default (safety trade-off, team call).
- Brief Q1: the bridge runs on the XBLink's Pi (verified). Still open: how it
  receives the FC's MAVLink, since XBFirm most likely owns that link (local UDP
  forward, mavlink-router, or replacing XBFirm's role). Needs XBStation. Decides
  `--mavlink` and the systemd unit's `After=`.
- XBLink: the vendor cellular service did not restart after a power cut on
  2026-10-28 (reboot fixed it). Debian 11 past end of support, 243 pending
  updates, own Wi-Fi hotspot on `wlan0`, root SSH. Raise with XBStation.
- Moving the broker off the personal student subscription for anything beyond
  test data.
- Brief Q2/Q3: HiveMQ edition, hosting, and sovereignty/export-control
  implications for a defence programme.
- Brief Q6: which commands the backup link may carry.
- Whether to re-enable `UAVStatePoller` — **needs a second port** if so, since
  `MAVLINK_STATE_PORT` and `MAVLINK_COMMAND_PORT` currently name the same one
  and would double-bind.

### Known gaps and pre-existing defects (documented, not fixed)

- **The UAV_ watchdog does not detect a dead MAVLink link.** `state/watchdog.py`
  monitors *thread* liveness and `MavlinkWorker._loop` pets it unconditionally,
  so `is_UAV_Command_Connection_Available` stays `True` while commands vanish.
  Fixing it is a behaviour change needing a spec, not a bug fix.
- `UAVStatePoller.connect()` is commented out, so `is_flying`, mode and altitude
  have no writer and several panel guards never pass.
- QoS 1 can deliver duplicates. Not deduplicated by design — a naive dedupe
  drops legitimately repeated frames. QoS is configurable.
- Brief Q4 is partly answered with data: the STM32 panel presents
  `(sysid 255, compid 0)` and Mission Planner `(255, 190)`, and neither sends its
  own HEARTBEAT. The bridge counts frames per `(sysid, compid)`; it deliberately
  does **not** filter, which would break transparency.
