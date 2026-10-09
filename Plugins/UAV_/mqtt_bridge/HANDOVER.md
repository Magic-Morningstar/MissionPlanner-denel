# MAVLink-over-MQTT bridge — session handover

**Date:** 2026-10-02 · **Branch:** `MQTT` · **State:** Phase 1 passed, work uncommitted

Read `README.md` for how to *use* the bridge. This file is for picking the work
back up: what was decided, what was found, and what to do next.

---

## 1. Where things stand

Phase 0 (environment feasibility) and Phase 1 (raw tunnel, Mission Planner works)
are **done and verified against ArduCopter SITL**. 120 unit tests pass.

Verified on 2026-10-02, with Mission Planner **unmodified**:

| Brief §7 Phase 1 acceptance | Result |
|---|---|
| 1. SITL → tunnel → client sees HEARTBEAT | pass (`tools/mav_probe.py`) |
| 2. MP: params, mission write + read-back, arm/disarm, mode change | pass — plus GUIDED takeoff, Fly-To-Here, RTL and full **AUTO mission execution** |
| 3. Byte-for-byte integrity both directions | pass — zero BAD_DATA across ~26,000 frames / 1 MB, plus the automated framing tests |
| 4. Both directions concurrently, control path intact | pass |

Supporting evidence from a live run: `inbound_malformed=0`, `inbound_dropped=0`,
`write_failures=0`, `mqtt_refused_queue_full=0`. SITL emitted `msgid11030` — a
message in no dialect either end knows — and the tunnel carried all 1,616 of them,
which is the no-CRC-validation design doing exactly its job.

### This is NOT flight-ready

Three reasons, all deliberate scope decisions rather than oversights:

1. Everything ran at **loopback latency**. MP's parameter and mission protocols
   are retry/timeout driven and behave differently at 150 ms with jitter and loss.
2. The broker runs the **Allow-All extension** — any process that can reach it
   can publish to `to_vehicle` and command the aircraft.
3. **No stale-command rejection.** `clean_start` covers the main case; the
   receive-side dwell check is not built.

---

## 2. What was built

`Plugins/UAV_/mqtt_bridge/` — 24 files, ~3,300 lines including tests.

| File | Purpose |
|---|---|
| `framing.py` | **The core.** Byte stream → whole MAVLink frames. Pure, no I/O. |
| `topics.py` | Topic construction, `validate_vehicle_id()`. Pure. |
| `bridge_config.py` | Defaults + env + CLI resolution, log-path anchoring. Pure resolver. |
| `mqtt_link.py` | paho 2.x `VERSION2` + MQTT 5 wrapper. |
| `mavlink_endpoint.py` | `MavutilEndpoint` (aircraft), `UdpPeerEndpoint` (GCS). |
| `bridge_core.py` | Threads, queues, shutdown, counters. |
| `aircraft_bridge.py` / `gcs_bridge.py` / `cli.py` | Entry points and shared CLI. |
| `tools/mqtt_smoke.py` | Phase 0 broker check. Ships to operator machines deliberately. |
| `tools/mav_probe.py` | Acceptance #1; `--request-streams` makes it bidirectional. |
| `tests/` | 120 tests. `test_framing.py` carries the integrity argument. |

Modified outside the package: `.gitignore`, `MissionPlanner.csproj` (a copy item
for the bridge's `requirements.txt`, and excluding `tests/` from deployment),
`Plugins/UAV_/requirements.txt` (added `paho-mqtt==2.1.0`), plus new
`Plugins/UAV_/pytest.ini`.

---

## 3. Bugs found during the work, and fixed

Each was found by testing, not by review — worth knowing they existed:

1. **stdin watcher killed the bridge at launch.** Auto-arming on "stdin is not a
   terminal" is wrong: that is equally true of closed stdin and `/dev/null`, and
   **systemd hands a service `/dev/null`**. The bridge exited instantly with
   `launcher closed stdin` as the only clue. Now opt-in via `--stdin-shutdown`.
   *This would have shipped as a silently dead aircraft service.*
2. **A bogus frame length could eat following datagrams.** The framer buffer
   persisted across reads, so one garbage `0xFD` with a 239-byte length could
   swallow several subsequent datagrams. Added `feed_datagram()`, which discards
   per-datagram residue — UDP never splits a frame, so leftovers mean corruption.
3. **`write_failures` was a lie.** Frames arriving before MP connected were
   counted as write failures. Split out as `dropped_no_peer`.
4. **Log flooded by a pymavlink deprecation.** `_msgid_name()` used `cls.name`,
   deprecated in 2.4.49, warning on every lookup. Now `.msgname`, cached.
5. **Duplicate client IDs flapped the connection.** Two bridge instances sharing
   one MQTT client id kick each other in a loop; the symptom reads as a flaky
   broker. The bridge now detects ≥3 disconnects/minute and names the cause.

---

## 4. Decisions made (don't re-litigate without reason)

- **The framer does not validate CRCs.** Verifying needs `crc_extra` from the
  compiled dialect; a drift between the FC's ArduPilot build and our pinned
  pymavlink would silently drop every frame of one message type, permanently.
  Two tests pin this: `test_bad_crc_passes_through_unchanged` and
  `test_bad_crc_interop`. If someone "fixes" it, those fail and explain why.
- **Our own UDP socket on the GCS side, not `mavudp`.** `mavudp.write()` has a
  permanently-true `self.timeout <= 0` guard, so it sends to every peer it has
  ever heard from and its eviction branch is dead code.
- **Port 14570**, clear of 14550–14559 (AutoConnect owns 14550; 14551 is reserved
  for the STM32 bridge; `MavlinkWorker` walks the whole range). Bound to
  `127.0.0.1`, never `0.0.0.0`.
- **`QOS_FROM_VEHICLE=0` is a safety setting, not a bandwidth one.** paho discards
  QoS 0 when offline but stores and re-sends QoS ≥ 1 after reconnect.
- **`DenelPythonLauncher.cs` untouched** — it is the suspected silent-failure path
  on the GCS machine, and mixing it with new work would tangle two problems.

---

## 5. Open — needs someone else's input

| # | Question | Why it is not mine to decide |
|---|---|---|
| 1 | `TO_VEHICLE_FAIL_OPEN` default | A command of unknown age reaching an aircraft, versus one broker quirk disabling the whole command path. Both implemented; it is a config flag, logged at startup. **Team decision, still open.** Related: `TO_VEHICLE_EXPIRY` was raised 3 s -> 30 s on 2026-10-09 as an interim measure, which makes this decision more pressing rather than less — expiry is currently the only bound on delivered command age. |
| 2 | Brief §9 Q1 — where the aircraft bridge runs | Decides the systemd unit's `After=`. Code assumes nothing: any pymavlink connection string works. |
| 3 | Brief §9 Q2/Q3 — HiveMQ edition, hosting, sovereignty | Phase 3. |
| 4 | Brief §9 Q4 — two GCS paths to one FC | Partly answered with data: the STM32 bridge presents `(255, 0)` and MP `(255, 190)`, and neither sends its own HEARTBEAT. The bridge counts frames per `(sysid, compid)`. |
| 5 | Re-enable `UAVStatePoller`? | Its `connect()` is commented out, so the STM32 panel's state fields have no writer. Teammate's file. |

### Pre-existing defects raised, deliberately not fixed

- `state/system_config.py:8,10` points both MAVLink ports at **14550**, which
  AutoConnect binds at startup and never releases. Same race the
  `AutoConnect_denel14551` migration was written to kill.
- Root `CLAUDE.md` is stale: says 14551.
- `logging_config.py:27` `DEFAULT_LOG_FILE` is a bare relative path.
- `mavlink_worker.py:22-58` walks ports, raises `ValueError` on serial strings,
  and `send_ping` is missing `self`.
- The UAV_ watchdog monitors **thread** liveness, not MAVLink liveness, so a dead
  link leaves `is_UAV_Command_Connection_Available = True` while commands vanish.
- `GCSViews/FlightData.cs:1969` — the Actions **Do Action** dropdown throws
  `KeyNotFoundException` for every entry (indexer instead of `TryGetValue`;
  `CustomActions` is plugin-only and empty). Looks like upstream MP. Workaround:
  the dedicated **RTL** / **Set Mode** buttons use a different path and work.

---

## 6. Next steps, in order

1. **Phase 1b — STM32 fan-out** (`local_hub.py`). The Python panel bridge is a
   second MAVLink consumer on the GCS and currently gets nothing. It needs
   telemetry **and** commands, not either alone: `UAVCommandSender._do_connect()`
   blocks in `_heartbeat()` until a vehicle HEARTBEAT arrives, and because its
   connection is `udpin:`, `mavudp.write()` only sends to peers learned in
   `recv()` — so with no inbound traffic its commands vanish silently. "MP works"
   is not "the Denel GCS works".
2. **Phase 1c — LTE conditions.** `tc netem` on WSL `eth0`, aircraft bridge
   pointed at the host IP so only that hop degrades. Re-run the three MP tests at
   ~25/75/200 ms with jitter and loss. Deliverable: the latency at which each
   starts failing. Note MQTT is over TCP, so loss presents as retransmit delay and
   head-of-line blocking, not lost frames.
3. **Phase 2 — robustness.** Last Will + clean `offline` (a graceful DISCONNECT
   *suppresses* the Will, so `stop()` must publish it and `wait_for_publish`),
   reconnect tests, staleness layers, the opt-in launcher plugin.

---

## 7. Resuming on Monday

```bash
# 1. Broker (Docker Desktop must be running)
docker start hivemq          # already up? docker ps

# 2. SITL, in WSL
cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter --no-mavproxy
```
```powershell
# 3. Aircraft bridge  (Windows reaches WSL's SITL on localhost)
cd C:\MissionPlanner-denel\Plugins\UAV_
python -u -m mqtt_bridge.aircraft_bridge --mavlink tcp:127.0.0.1:5760 --broker-host localhost --vehicle-id uav01

# 4. GCS bridge, in a second window
cd C:\MissionPlanner-denel\Plugins\UAV_
python -u -m mqtt_bridge.gcs_bridge --broker-host localhost --vehicle-id uav01

# 5. Mission Planner -> UDPCl -> 127.0.0.1 -> 14570
```

**Start exactly one of each bridge.** Two instances share a client id and flap.
Check first:

```powershell
Get-CimInstance Win32_Process -Filter "Name='python.exe'" | Where-Object { $_.CommandLine -match 'mqtt_bridge' }
```

Sanity checks: `python -m pytest` from `Plugins/UAV_` → 120 passed.
`python -m mqtt_bridge.tools.mqtt_smoke --host localhost` → PASS.

Healthy counters: `inbound_malformed=0`, `inbound_dropped=0`, `mqtt=up`,
`mqtt_disconnects` not climbing, and `peer=` showing an address once MP connects.
`out_bad_events` ticking up by one per SITL reconnect is normal; climbing during
steady flight is not.
