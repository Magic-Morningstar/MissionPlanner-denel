# MAVLink-over-MQTT Bridge: Implementation Brief

**Programme:** Denel FW-VTOL 250 UAV (24W0039TB0001), cloud comms prototype
**Owner:** Taariq (GCS integration + security)
**Status:** Prototype. Production architecture is still being finalised by the team. Some details come from a brief meeting recollection and are marked *assumption* or listed in section 9.

---

## 0. How to work

1. **Read this whole brief, then explore the existing repo (the GCS Python infrastructure) before writing any code.**
2. Come back with a short plan: where the bridge lives, which existing modules you will reuse (config loading, logging, process launching, packaging), and the proposed file layout. Wait for approval, then implement.
3. Follow the codebase's existing conventions. Do not reinvent config, logging, or launcher plumbing that already exists.
4. Work in the phases in section 7. Commit per phase. Each phase has acceptance tests that must pass before moving on.
5. Taariq is new to MQTT. Comment the non-obvious MQTT/MAVLink logic, keep the code readable, and explain design decisions briefly in commit messages.
6. Do not guess on anything in section 9. Surface it in your plan.

---

## 1. Goal

Carry MAVLink between the aircraft and the GCS over MQTT, through a HiveMQ broker, using the aircraft's XBLink cellular (LTE) uplink. This is a **long-range / BVLOS backup path** for when the aircraft is beyond Herelink RF range. It does not replace the Herelink handheld link.

**Design goal: the MQTT hop must be invisible to both ends.** To Mission Planner, the GCS-side bridge looks like an ordinary MAVLink UDP peer. To the flight controller, the aircraft-side bridge looks like an ordinary MAVLink link. Mission Planner must keep full functionality (parameters, mission upload/download, calibration, tuning) with no modification.

SRS context (as referenced in the SRS; verify against the SRS text): SRS-UAS-0415-00 (XBLink as backup air-ground comms over Ethernet for extended range), SRS-UAS-0290-00 (LTE as back-up control link), SRS-UAS-0410-00 (MAVLink as primary protocol).

---

## 2. Architecture

Two separate bridge processes with mirrored roles. Both are MQTT publishers **and** subscribers. The broker only routes by topic and never decodes MAVLink.

```
 AIRCRAFT SIDE                          CLOUD / LAN                     GCS SIDE

 Flight controller                                                      Mission Planner
 (Cube Orange+,                                                         (MAVLink over UDP)
  ArduPilot)                                                                  ^
      ^  |                                                                     |
      |  | MAVLink (serial/UDP/TCP)                                            | MAVLink (UDP, local)
      |  v                                                                     v
 +-----------------+   MQTT/TLS    +----------------+   MQTT/TLS    +-----------------+
 | AIRCRAFT BRIDGE |<------------->| HiveMQ broker  |<------------->|   GCS BRIDGE    |
 | pub: from_vehicle| via XBLink   | routes by topic|  ordinary     | pub: to_vehicle |
 | sub: to_vehicle |   (LTE)       | no MAVLink     |  internet     | sub: from_vehicle|
 +-----------------+               +----------------+               +-----------------+
```

- Telemetry path: FC -> aircraft bridge -> `from_vehicle` -> broker -> GCS bridge -> Mission Planner
- Command path: Mission Planner -> GCS bridge -> `to_vehicle` -> broker -> aircraft bridge -> FC
- Both paths run continuously and simultaneously.
- LTE applies only to the aircraft's hop. The GCS reaches the broker over whatever connection the ground station has.

---

## 3. Decisions already made

| Decision | Detail |
|---|---|
| Tunnel style | **Raw frame tunnel.** MQTT payload = raw MAVLink frame bytes. The bridge does not decode or re-encode. This preserves MAVLink v2 signing end to end (MAV_SIGNING): the bridge never signs or verifies. |
| Semantic telemetry | JSON topics per message type are **Phase 4, cloud-side only**, fed from the raw tunnel. They are not part of the control path. |
| MQTT version | MQTT 5.0 |
| Libraries | `pymavlink`, `paho-mqtt >= 2.0` (needs `CallbackAPIVersion.VERSION2` and `protocol=MQTTv5`) |
| Broker | HiveMQ Platform (`hivemq/hivemq4`) in Docker Desktop (WSL2 backend), free "Lab" licence (100 connections, 100 msg/s). MQTT on `1883`, Control Center on `8080`. Production is expected to default to HiveMQ. |
| Broker-agnostic messaging | Use standard MQTT 5 only in the bridges. No HiveMQ-proprietary features, so the broker can be swapped without a rewrite. |
| Current broker security | The **Allow-All extension is active**: any client can connect and pub/sub anything. Dev only. Must be replaced before this touches anything beyond a dev machine (Phase 3). |

**Secrets:** The HiveMQ licence key belongs to the broker container's environment only. The bridges do not need it. Never commit any credential, key, or certificate. Load secrets from environment variables or an untracked secrets file.

---

## 4. Topics, QoS and staleness

Namespace by vehicle from day one. This enables multi-aircraft support and per-vehicle access control.

| Topic | Direction | QoS | Retained | Notes |
|---|---|---|---|---|
| `denel/uav/{vehicle_id}/mavlink/from_vehicle` | aircraft -> GCS | 0 (configurable) | no | Raw frames. Short message expiry (default 5 s). Telemetry-dominated; MAVLink's own retries cover lost ACKs. |
| `denel/uav/{vehicle_id}/mavlink/to_vehicle` | GCS -> aircraft | 1 (configurable) | no | Raw frames. Commands, params, missions must not be lost. Short message expiry (default 3 s, configurable). |
| `denel/uav/{vehicle_id}/status/link` | aircraft bridge | 1 | **yes** | `online` on connect. **Last Will** publishes `offline`. |
| `denel/uav/{vehicle_id}/status/gcs` | GCS bridge | 1 | **yes** | Same pattern (online/offline). |

**Stale-command safety (important):** QoS 1 with a persistent session could queue commands while the aircraft is offline and deliver them on reconnect, so an old ARM, mode change, or waypoint could execute unexpectedly. Prevent this with all of:
- Aircraft bridge connects with `clean_start=True` and session expiry 0.
- GCS bridge sets a short `MessageExpiryInterval` on every `to_vehicle` publish.
- Aircraft bridge re-subscribes in `on_connect` on every reconnect (a clean session forgets subscriptions).
- Add a test for this (section 7, Phase 2).

**Duplicates:** QoS 1 can deliver duplicates. Do not dedupe in v1 (a naive dedupe can drop legitimate repeated frames). Document the risk and make QoS configurable.

---

## 5. Bridge specifications

### 5.1 Aircraft bridge
- Connects to the FC's MAVLink source. The connection string is configurable (serial, `udpin:`/`udpout:`, `tcp:`). Do not hardcode.
- Reads MAVLink, publishes **whole raw frames** to `from_vehicle`.
- Subscribes to `to_vehicle` and writes the bytes to the FC.
- Registers an MQTT Last Will on `status/link` (`offline`, retained) and publishes `online` (retained) after connecting.
- Must be **host-agnostic**: runs anywhere that has a route to the FC's MAVLink and to the broker. Where it physically runs is undecided (section 9, Q1).

### 5.2 GCS bridge
- Presents a local MAVLink UDP endpoint that Mission Planner connects to. Default suggestion: send to `127.0.0.1:14550` and receive Mission Planner's replies on the same socket, but **verify against how the existing GCS connects** and make it configurable.
- Subscribes to `from_vehicle` and forwards frames to Mission Planner.
- Publishes Mission Planner's outgoing frames to `to_vehicle` (with message expiry).
- Same `status/gcs` online/offline handling.
- Integrate with the existing GCS Python launcher and packaging conventions found in the repo.

### 5.3 Shared requirements
- **Whole frames only.** UDP datagrams may contain more than one frame; serial is a byte stream and can split frames. Frame-parse (for example via pymavlink and `msg.get_msgbuf()`, or a lightweight framer) so no MQTT message ever contains a partial frame. Count and log bad or unparseable data, do not silently drop.
- **Byte-for-byte transparency.** What goes in one side comes out the other unchanged (including signature bytes).
- **Threading.** `paho` runs its network loop with `loop_start()`. `on_message` must not block: push to a queue and write from a dedicated thread. MAVLink reads run in their own thread. Make writes thread-safe.
- **Unique client IDs.** A duplicate client ID makes the broker kick the earlier session, causing a flapping loop. Use e.g. `aircraft-bridge-{vehicle_id}` and `gcs-bridge-{vehicle_id}-{hostname}`.
- **Reconnect.** Automatic MQTT reconnect with backoff (`reconnect_delay_set`). Also handle the FC/MAVLink source dropping and returning. Neither failure may crash the process.
- **Loop prevention.** Separate `from_vehicle` and `to_vehicle` topics, and a bridge never publishes what it received from the broker back to the broker.
- **Config** (file plus env overrides): `vehicle_id`, broker host/port, TLS settings, credentials source, topic prefix, QoS per topic, message expiry, MAVLink endpoint, log level.
- **Logging.** Start logging at process start, before any precondition check, so early failures are captured. Anchor all file paths to `Path(__file__).resolve().parent` or an explicit configured directory, never the CWD. The GCS machine is Windows and has previously failed silently, with relative paths resolving to `system32`. Use the same log location/convention as the existing GCS code (confirm; existing logs live under `C:\ProgramData\Denel GCS\`).
- **Metrics** (logged periodically): frames and bytes up/down, bad frames, dropped/expired, reconnect count, MQTT round-trip latency estimate if feasible.
- **Graceful shutdown** on SIGINT/SIGTERM and service stop, publishing `offline` cleanly.
- Do not log full payloads at INFO. Optional hex dump at DEBUG.

---

## 6. Environment

- **Dev machine:** Windows with WSL2 (Ubuntu). Python venv `venv-ardupilot` in WSL. ArduPilot **SITL** runs in WSL and is fronted by **MAVProxy**, which outputs MAVLink on UDP `14550` (the port conventionally used for a GCS-role client). This is dev-only plumbing and will not exist on the aircraft.
- **Broker:** Docker Desktop, container name `hivemq`, reachable at `localhost:1883` (should be reachable from both WSL and Windows; verify both).
- **GCS:** Windows, Mission Planner plus the existing Python launcher/plugin infrastructure. Inspect the repo for it.
- **Aircraft:** Cube Orange+ running ArduPilot. Comms hardware is XBLink (cellular). Onboard compute for the bridge is undecided.

---

## 7. Phases and acceptance tests

### Phase 1: Raw tunnel, local, no TLS
Both bridges against the local HiveMQ, SITL as the aircraft.

Acceptance:
1. SITL -> aircraft bridge -> HiveMQ -> GCS bridge -> a `pymavlink` test client sees `HEARTBEAT`.
2. Mission Planner connected to the GCS bridge endpoint shows live telemetry, completes a **parameter download**, does a **mission write and read-back**, and can **arm/disarm and change mode** in SITL.
3. **Integrity test:** replay a recorded `.tlog` (or generated frames) through the tunnel and assert byte-for-byte equality in both directions.
4. Both directions carry traffic concurrently without loss of the control path.

### Phase 2: Robustness
Acceptance:
1. Kill and restart the broker: both bridges reconnect and traffic resumes.
2. Drop and restore the FC/MAVLink source: the bridge recovers without restart.
3. Kill the aircraft bridge: `status/link` flips to `offline` via the Last Will and is retained.
4. **Stale-command test:** with the aircraft bridge offline, publish a command to `to_vehicle`, wait past expiry, reconnect the aircraft bridge: the command must **not** be delivered.
5. Clean shutdown publishes `offline`. Metrics are logged.
6. Packaging: a sample service definition for the aircraft side (for example systemd) and integration with the GCS launcher on the Windows side.

### Phase 3: Security (client support, broker config, tests)
Client side:
- TLS (broker port `8883`), configurable CA, optional client certificate (mutual TLS), username/password. Secrets from env or an untracked file.

Broker side (deliverable is a proposal plus a dev config, not a production commitment):
- Replace the Allow-All extension with the **HiveMQ File RBAC extension** (Apache 2.0, HiveMQ-verified) as the candidate: username/password auth plus topic-level permissions with clientId/username substitution. Note it disallows `#` and `+` in client IDs and usernames. The team may instead choose the Enterprise Security Extension (section 9, Q2).
- Per-vehicle ACL intent: the aircraft user may **publish** `.../from_vehicle` and `.../status/link` and **subscribe** `.../to_vehicle` only. The GCS user is the inverse.

Acceptance (negative tests):
1. Wrong credentials are refused.
2. An aircraft-role client cannot publish to `to_vehicle`.
3. A GCS-role client cannot publish to `from_vehicle`.
4. A client for vehicle A cannot read or write vehicle B's topics.
5. TLS handshake fails against an untrusted CA.

Write short threat-model notes: MQTT auth/ACL protects the broker path, MAV_SIGNING protects frame authenticity end to end, and they are complementary.

### Phase 4 (optional, after 1-3 pass): Semantic fan-out
A cloud-side service subscribes to `from_vehicle`, parses with pymavlink, and republishes structured JSON to `.../telem/position`, `telem/attitude`, `telem/battery`, and retained `status/mode`. Keep it fully isolated from the control path. Parsing happens cloud-side so the LTE uplink only carries lean raw frames.

---

## 8. Non-goals

AWS/Google Cloud deployment, replacing Herelink, modifying Mission Planner, writing HiveMQ Java extensions, GCS UI changes, any change to flight code, message dedupe, bandwidth optimisation (measure first).

---

## 9. Open questions (do not guess; raise in your plan)

1. **Where does the aircraft bridge run?** XBLink is normally a MAVLink datalink product. Does it expose an IP network/Ethernet to an onboard companion computer, or does it connect straight to the FC's telemetry port? Is there a companion computer at all? This determines the aircraft bridge's host and how it reaches the FC.
2. **HiveMQ production edition and security path:** Community vs Platform/Enterprise, and File RBAC vs the Enterprise Security Extension.
3. **Hosting:** AWS vs Google Cloud (later). Sovereignty and export-control implications of a managed/foreign-hosted broker for a defence programme. A self-hosted broker is assumed for now.
4. **Two GCS paths to one FC:** the Herelink-connected GCS and the MQTT-connected GCS may both talk to the same flight controller. How are system IDs, heartbeats, and GCS-loss failsafe behaviour handled, and how do we avoid two sources commanding at once?
5. **LTE bandwidth budget and stream rates**, including whether MQTT topic aliases or frame coalescing are needed.
6. **Which commands are permitted over the backup link**, and the latency the requirements allow.

---

## 10. Implementation notes

- `pymavlink`: `mavutil.mavlink_connection(...)`, `recv_match(...)`, and `msg.get_msgbuf()` for the raw frame bytes. Raw writes go through the connection's write method, or use a plain socket/serial port for full control.
- `paho-mqtt` 2.x: `mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=..., protocol=mqtt.MQTTv5)`. v5 callbacks receive `reason_code` and `properties`. Use `Properties(PacketTypes.PUBLISH)` for `MessageExpiryInterval`, and `will_set` for the Last Will.
- MQTT topic strings add per-message overhead over LTE. Measure before optimising; MQTT 5 topic aliases and short coalescing windows are the later levers.
- Keep a small, well-tested core (framing, topic mapping, config) separate from I/O so it can be unit-tested without a broker or SITL.

---

## 11. Definition of done

- Phases 1-3 acceptance tests pass and are automated where practical (unit tests for framing/integrity, an integration script for the SITL loop).
- Mission Planner works through the bridge with no modification.
- No secrets in the repo.
- A short README covering setup, config reference, how to run the SITL end-to-end test, and known limitations.
- Open questions from section 9 are recorded with the assumptions the code currently makes.
