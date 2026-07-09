# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

**Primary build (Visual Studio 2022 required):**
```
msbuild -v:m -restore -t:Build -p:Configuration=Release MissionPlanner.sln
```

**Debug build:**
```
msbuild -v:m -restore -t:Build -p:Configuration=Debug MissionPlanner.sln
```

**Before first build**, initialize git submodules (includes `mono`, MAVLink definitions, etc.):
```
git submodule update --init --depth 2 --no-single-branch
```

Build output lands in `bin\Release\net461\` (Release) or `bin\Debug\net461\` (Debug). The main project targets `net472`; output path does not append the framework name (`AppendTargetFrameworkToOutputPath=false`).

**Run tests (MSTest):**
```
dotnet test MissionPlannerTests\MissionPlannerTests.csproj
```
To run a specific test class or method, use `--filter`:
```
dotnet test MissionPlannerTests\MissionPlannerTests.csproj --filter "FullyQualifiedName~FirmwareTests"
```

## Architecture

Mission Planner is a Windows Forms ground control station (GCS) for ArduPilot autopilots. The solution (`MissionPlanner.sln`) compiles many projects; the main executable is `MissionPlanner.csproj`.

### Key namespaces and projects

| Project | Location | Purpose |
|---|---|---|
| `MissionPlanner` (exe) | root `.cs` files | Main WinForms app, entry point `Program.cs`, main window `MainV2.cs` |
| `MissionPlanner.ArduPilot` | `ExtLibs/ArduPilot/` | Vehicle state (`CurrentState`), firmware, parameter packs, mavlink wrappers |
| `MAVLink` | `ExtLibs/Mavlink/` | Auto-generated MAVLink protocol bindings and CRC; regenerated via `regenerate.bat` |
| `MissionPlanner.Comms` | `ExtLibs/Comms/` | All transport layers: serial, TCP, UDP, BLE, WebSocket, pipe (netstandard2.0) |
| `MissionPlanner.Utilities` | `ExtLibs/Utilities/` | Shared utilities: settings, firmware fetching, GDAL, coordinate transforms, etc. |
| `MissionPlanner.Controls` | `ExtLibs/Controls/` | Reusable WinForms controls |
| `GMap.NET.Core/WindowsForms` | `ExtLibs/GMap.NET.*` | Embedded map library |
| `MissionPlannerTests` | `MissionPlannerTests/` | MSTest test project |

### View layer (`GCSViews/`)

The five top-level screen tabs each have a corresponding `MyUserControl` in `GCSViews/`:

- `FlightData.cs` — live telemetry HUD, map, gauges
- `FlightPlanner.cs` — mission/waypoint editor with GMap integration
- `InitialSetup.cs` — first-time setup wizard
- `SoftwareConfig.cs` — parameter tuning UI
- `SITL.cs` — software-in-the-loop simulator launcher

`ConfigurationView/` contains sub-pages loaded inside `SoftwareConfig`.

### MAVLink / vehicle communication

`MAVLinkInterface` (`ExtLibs/ArduPilot/Mavlink/MAVLinkInterface.cs`) is the central communications class. It wraps an `ICommsSerial` transport and handles packet parsing, command sending, parameter reads, and mission upload/download. It exposes an `OnPacketReceived` event and maintains a `MAVList` of connected vehicles.

Each connected vehicle is a `MAVState` (`ExtLibs/ArduPilot/Mavlink/MAVState.cs`) which holds per-vehicle state. Live telemetry values are stored in `CurrentState` (`ExtLibs/ArduPilot/CurrentState.cs`).

`MainV2.comPort` is the global `MAVLinkInterface` instance. `MainV2.comPort.MAV` is the currently active `MAVState`.

### Settings

`Settings.Instance` (singleton, XML-backed) is the persistent key/value store. Platform paths:
- Data directory: `C:\ProgramData\Mission Planner\`
- User data: `C:\Users\USERNAME\Documents\Mission Planner\`

### Plugin system

Plugins are compiled `.dll` or `.cs` files placed in the `plugins/` subdirectory. They implement the abstract `Plugin` base class (`Plugin/Plugin.cs`) with `Init()`, `Loaded()`, `Loop()`, and `Exit()`. `PluginLoader` discovers and loads them at startup. Plugins access the app via `PluginHost`.

### Maps

Custom `GMap.NET` providers are registered at startup in `Program.cs`. The map cache is stored in `C:\ProgramData\Mission Planner\gmapcache`. Optional GDAL support is loaded if a `gdal/` directory exists next to the executable.

### Multi-platform notes

The codebase compiles for `net472` (Windows exe) and `netstandard2.0` (library projects). Android and iOS builds use Xamarin; macOS uses Mono. The `#if !LIB` and `#if MONO` guards separate platform-specific code. `Program.MONO` is set at runtime when running under Mono.

### Localization

`.resx` files per language are alongside each view file (e.g., `FlightData.zh-Hans.resx`). The `L10N.cs` helper handles runtime string lookup.

---

## Denel Customisations

This fork contains Denel Aeronautics branding and GCS-specific changes on top of upstream Mission Planner. All changes are isolated so they do not affect core ArduPilot functionality.

### MSBuild path (this machine)
MSBuild is not in PATH. Use the full path:
```
"C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" -v:m -restore -t:Build -p:Configuration=Debug MissionPlanner.sln
```
Kill any running `MissionPlanner.exe` first — it locks DLLs and breaks the copy step.

---

### Branding

| File | Change |
|---|---|
| `Program.cs` (~line 205) | `name = "GCS - By Denel"` — sets window title and splash title |
| `Splash.cs` | `label1.Text = "By Denel Aerospace"` + `ForeColor = Color.FromArgb(0x00,0xBF,0xFF)` (cyan) |
| `MainV2.cs` `MenuArduPilot_Click` (~line 4678) | URL changed to `http://www.denelaeronautics.co.za/` |
| `MainV2.cs` (~line 1092) | MenuArduPilot toolbar button image set from `Properties.Resources` directly (no logo2.png at runtime) |
| `mpdesktop.ico` | Replaced with multi-size (16×16 / 32×32 / 48×48) Denel icon (PNG-in-ICO format) |
| `denel_cyan.mpsystheme` (repo root) | Denel theme — dark background (`#16181A`), cyan buttons (`#00BFFF`), desert-sand HUD |
| `MissionPlanner.csproj` | `denel_cyan.mpsystheme` registered as `<None CopyToOutputDirectory=PreserveNewest>` |

**Activating the theme on first run:** Config → Planner → Theme → select `denel_cyan`. After that it persists in `Settings.Instance["theme"]`.

### Dark title bar (DWM)

`NativeMethods.cs` has a `DwmSetWindowAttribute` P/Invoke with `DWMWA_USE_IMMERSIVE_DARK_MODE = 20`.
`MainV2.cs` (~line 1092) calls it on startup to give the window a dark title bar on Windows 10 build 19041+.

### Theme system

`.mpsystheme` files are XML. `ThemeManager.GetThemesList()` auto-discovers any `.mpsystheme` in the running directory. Key fields in the Denel theme:

- `iconSet` = `BurnKermitIconSet` (no trailing 't') — loads `light_*` icon PNGs from `Resources\`
- `HudGroundTop/Bot` — desert sand (`#C8A96E` / `#8B6914`)
- `HudSkyTop/Bot` — light desert haze (`#87CEEB` / `#D4EAF7`)
- `HudText` — warm cream (`#F5ECD7`)

**HUD colours on startup fix:** `ThemeManager.SetTheme()` skips HUD colour assignment when `FlightData.myhud == null` (form not yet created). Fixed in `GCSViews/FlightData.cs` constructor — immediately after `myhud = hud1`, the five HUD colour properties are explicitly read from `ThemeManager` and applied to `hud1`.

---

### Layout (`GCSViews/FlightData.cs` constructor, ~line 244)

The designer layout is restructured at runtime — do not rely on the designer view:

```
MainH (top/bottom SplitContainer — FixedPanel.Panel2 keeps tabs fixed)
├── Panel1 (top, resizable)
│   └── SubMainLeft (left/right SplitContainer — FixedPanel.None, both sides resize)
│       ├── Panel1 (left) = hud1  [Dock=Fill]
│       └── Panel2 (right) = tableMap (map)
└── Panel2 (bottom, full-width) = tabControlactions + panel_persistent
```

**HUD/map split:** `SubMainLeft.SplitterDistance` starts at a true 50/50 split on first run (set in `FlightData_Load` when no saved value exists yet). The user's manually-dragged split position is persisted across restarts via `Settings.Instance["FlightSplitterHudMap"]` — saved in `Dispose()`, restored in `FlightData_Load`, mirroring the pre-existing pattern already used for the outer `MainH.SplitterDistance` (`Settings.Instance["FlightSplitter"]`). `hud1_Resize` nudges `SubMainLeft.SplitterDistance` back to `hud1.Width` if it drifts by more than 5px — a stability guard against internal HUD layout recalculation, not an active resize policy.

**Removed (do not re-add without discussion):** an earlier version of this fork had `AutoUndockHUD()` — automatically moving the HUD into a borderless window on a second monitor at startup when one was detected — plus a `SubMainLeft_Resize` handler that forced the HUD panel to stay square (`SplitterDistance = Math.Min(SubMainLeft.Height, SubMainLeft.Width / 2)`). Both were deliberately removed; the HUD no longer auto-undocks on startup, and the HUD/map split is a plain, user-adjustable, persisted splitter instead of a square-forcing one. Manual double-click-to-undock (`hud1_DoubleClick` / `dropout_FormClosed`) and "swap HUD and map" (`SwapHud1AndMap`, persisted via `Settings.Instance["HudSwap"]`) are unaffected and still work.

---

### Tab strip (`GCSViews/FlightData.cs`)

`tabControlactions` (full-width bottom, holds Quick / Actions / Messages / PreFlight / Gauges / etc.) uses owner-draw mode:
```csharp
tabControlactions.DrawMode = TabDrawMode.OwnerDrawFixed;
tabControlactions.DrawItem += tabControlactions_DrawItem;
```
Selected tabs paint in `ThemeManager.ButBG` (cyan); unselected in `ThemeManager.ControlBGColor`. Right-clicking the tab strip shows the undock context menu.

### Gauges tab (`GCSViews/FlightData.cs`)

Seven gauges are present — four from the designer, three added programmatically in the constructor:

| Field | Label | Binding | Scale |
|---|---|---|---|
| `Gvspeed` | VSI | `verticalspeed` | designer |
| `Gspeed` | Speed | `airspeed`/`groundspeed` | designer |
| `Galt` | Alt | `alt` | designer |
| `Gheading` | Heading (HSI) | compass | designer |
| `Gwpdist` | WP Dist | `wp_dist` | 0–1000 m |
| `GdistHome` | Home | `DistToHome` | 0–2000 m |
| `GbattRemaining` | Batt % | `battery_remaining` | 0–100 % |

**Background:** The four designer gauges had embedded JPEG face images in `FlightData.resx` that painted over `BackColor`. These are nulled at runtime (`BackgroundImage = null`). All AGauge controls use `BackColor = Color.Transparent` (renders black via their UserPaint offscreen bitmap). The HSI (`Gheading`) does not use UserPaint so it gets `BackColor = Color.Black` explicitly. The tab itself gets a `Paint` event that force-fills black to override WinForms visual-style rendering.

**Scale line radii:** All gauges use `ScaleLinesMajorOuterRadius = 60`, `ScaleLinesInterOuterRadius = 60`, `ScaleLinesMinorOuterRadius = 60` — matching the designer gauges. Do not set these to 70 (BaseArcRadius) or the tick marks will be overlong.

**Resize (`tabPage1_Resize`):** Null-guarded on `Gwpdist`. Three layout branches:
- Wide (≥500px): all 7 in a row — `mywidth = Math.Min(Width/7, Height)`
- Narrow (<500px): 3 visible (Gspeed, Galt, Gheading) — `mywidth = Math.Min(Width/3, Height)`
- Square (aspect ratio 0.5–1.9): 4+3 grid — top row (Gvspeed, Gspeed, Galt, Gheading), bottom row (Gwpdist, GdistHome, GbattRemaining); `myheight = Math.Min(Height/2, Width/4)`

### Undock — Gauges and Messages (`GCSViews/FlightData.cs`)

Right-click context menu → "Undock Gauges" / "Undock Messages" moves the `TabPage` into a wrapper `TabControl` inside a new `Form`. The tab header row is hidden by offsetting the `TabControl` 22px above the form edge so content fills the window:

```csharp
dropout.Controls.Add(tab);
tab.Location = new Point(0, -22);
tab.Size = new Size(dropout.ClientSize.Width, dropout.ClientSize.Height + 22);
tab.Anchor = AnchorStyles.Top | AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right;
```

**Important:** Set `dropout.Size` BEFORE `dropout.Controls.Add(tab)` so `ClientSize` is correct when computing `tab.Size`. Closing the undocked window re-attaches the `TabPage` to `tabControlactions`.

---

### Python controller integration

A custom STM32 hardware controller communicates with the drone via a Python script launched automatically by the `DenelPythonLauncher` plugin.

**Current connection chain (UAV_ scripts):**
```
STM32 → COM7 (binary serial packets) → main.py → tcp:127.0.0.1:5763 → ArduPilot
```

**Startup ordering / connection resilience:** `DenelPythonLauncher.cs` auto-starts `main.py` as soon as the GCS plugin loads — this is typically *before* the operator has manually connected MissionPlanner to the vehicle, so nothing may be listening on `MAVLINK_CONNECTION` yet. `Mavlink_controller.initiate_Connection()` (`plugins/UAV_/mavlink/mavlink_contoller.py`) wraps `mavutil.mavlink_connection()` in a `try`/`except` retry loop (1s interval, `system_Print`-logged) instead of letting a refused connection raise uncaught — matching the existing STM32 serial-retry pattern in `main.py`'s read loop. There is no process-level watchdog (`DenelPythonLauncher.cs` does not restart `main.py` if it exits), so this in-script retry is the only thing standing between a cold-boot race and a permanently-dead bridge for the rest of the GCS session — do not remove it without adding an equivalent safeguard.

**Files:**

| File | Purpose |
|---|---|
| `plugins/DenelPythonLauncher.cs` | C# plugin — auto-starts `plugins/UAV_/main.py` on GCS startup, kills it on exit, logs to `denel_python.log` |
| `plugins/UAV_/main.py` | Entry point — modular Controller class; connects MAVLink + STM32 serial |
| `plugins/UAV_/config.py` | Central config — `MAVLINK_CONNECTION`, `SERIAL_PORT`, bitmask bits, timeouts, debug flags |
| `plugins/UAV_/mavlink/` | Flight controllers: arms, RTL, auto-takeoff, manual, speed control (`mavlink/Services/Pre_Flight_Checks/` holds battery/flight/GPS pre-flight checks) |
| `plugins/UAV_/serial_controller/` | Binary packet protocol: `serial_handler.py`, `packet_Parser.py`, `packet_builder.py` |
| `plugins/UAV_/serial_controller/protocol/bit_definitions.py` | Centralised bitmask definitions |
| `plugins/UAV_/notifications.py` | Sends `BUZZER_ALERT` messages to the GCS (see Buzzer notification system below) |
| `plugins/UAV_/state/system_state.py` | SystemState class — tracks all vehicle flags and computed properties |
| `plugins/Misson_PlannerScripts/` | Legacy scripts (Heart.py) — kept for reference, no longer launched |

**Binary packet format:** START_BYTE=0xAA, END_BYTE=0x55, PACKET_SIZE=6 bytes (different from old "NUM:X" text format).

**STM32 bitmask:**

| Bit | Command |
|---|---|
| 0 | Emergency |
| 2 | RTL |
| 4 | Arm |
| 6 | Auto takeoff |
| 8 | Pre-flight system check |
| 10 | Disarm |
| 11 | Manual mode |
| 12 | RF signal |
| 13 | LTE signal |
| 15 | Is flying (state read-back) |

**MAVLink connection for SITL testing:**
The script connects to `tcp:127.0.0.1:5763`. For ArduPilot SITL (sim_vehicle.py), change `config.py` line:
```python
MAVLINK_CONNECTION = 'tcp:127.0.0.1:5762'
```
Port 5762 is SITL's built-in secondary TCP port — no MAVProxy change needed.

**Python dependencies (install once per machine):**
```
pip install pymavlink pyserial
```

**Monitoring the script:**
```powershell
Get-Content -Path "denel_python.log" -Wait   # PowerShell tail -f equivalent
```
Log is created in the same folder as `MissionPlanner.exe` on first run.

---

### Release packaging

UAV_ Python scripts live in `plugins/UAV_/` in the repo. `MissionPlanner.csproj` has a `<None Include="plugins\UAV_\**\*.py">` item with `CopyToOutputDirectory=PreserveNewest`, so every `.py` file under `plugins/UAV_/` (including new ones) is copied to `bin\<Config>\net461\plugins\UAV_\` automatically on build — no manual copy step needed. Build and ZIP:
```powershell
# 1. Build
"C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" -v:m -restore -t:Build -p:Configuration=Release MissionPlanner.sln

# 2. ZIP for distribution
Compress-Archive -Path "bin\Release\net461\*" -DestinationPath "DenelGCS_Release.zip" -Force
```

**Note:** `python_runtime\` (bundled Python) lives in `bin\Release\net461\` and is excluded from the repo (`bin/` is gitignored). It must be set up once per dev machine. It is included in the ZIP automatically.

**Target machine requirements:** Windows 10/11 + .NET 4.7.2. No Python install needed — bundled in `python_runtime\`. No installer — unzip and run `MissionPlanner.exe`. Apply Denel theme on first launch via Config → Planner → Theme → `denel_cyan`.

**Auto-connect (future):** `ExtLibs/Utilities/AutoConnect.cs` already supports TCP auto-connect. To enable it, set `Enabled = true` on the relevant `ConnectionInfo` entry and set the target IP/port. No other code changes needed.

---

### Buzzer notification system (GCS side — COMPLETE)

A hardware buzzer attached to the STM32 is controlled by the Python script. When a flight condition is detected, the Python script sends an alert to the GCS; the GCS shows a dismissible dialog; after the operator dismisses it, the Python script stops the buzzer.

**Communication channel:** TCP socket on `127.0.0.1:5764` (separate from MAVLink on 5763).

**Protocol (simple text lines):**

| Direction | Message | Meaning |
|---|---|---|
| Python → GCS | `BUZZER_ALERT:some message\n` | Show alert dialog |
| GCS → Python | `BUZZER_ACK\n` | Operator dismissed — stop buzzer |

**C# side — `plugins/DenelPythonLauncher.cs` (implemented):**
- `_notifThread` starts unconditionally at the top of `Loaded()` — server always runs even if UAV_ scripts are absent
- `TcpListener` on `127.0.0.1:5764`; each connection handled in its own background thread
- On `BUZZER_ALERT:msg`: calls `MainV2.instance.Invoke(...)` → `ShowBuzzerAlert(msg)` — synchronous, blocks until operator dismisses
- After dismiss: writes `BUZZER_ACK\n` back to client
- `_notifRunning = false` + listener stopped in `Exit()`

**Alert dialog (`ShowBuzzerAlert` in `DenelPythonLauncher.cs`):**
- 500×280px borderless form; 3px border = `frm.BackColor`
- Flashes cyan (`#00BFFF`) ↔ red (`#FF3300`) at 500ms using `System.Windows.Forms.Timer`
- Cyan title strip "GCS ALERT" at top
- Windows warning icon (48×48) left of message text
- Cyan OK button, dark text; Enter/Escape also close the dialog
- Always centred on `Screen.PrimaryScreen` regardless of HUD/secondary screen

**Python side — `plugins/UAV_/notifications.py` (implemented):**
```python
from config import GCS_NOTIFICATION_PORT  # = 5764

def send_buzzer_alert(message: str) -> bool:
    with socket.create_connection(('127.0.0.1', GCS_NOTIFICATION_PORT), timeout=5) as s:
        s.sendall(f"BUZZER_ALERT:{message}\n".encode())
        ack = s.makefile().readline().strip()
        return ack == "BUZZER_ACK"
```

**Test without hardware (MissionPlanner must be running):**
```powershell
python -c "import socket; s=socket.create_connection(('127.0.0.1',5764)); s.sendall(b'BUZZER_ALERT:Test alert\n'); print(s.makefile().readline())"
```
Expected: themed flashing dialog appears; terminal prints `BUZZER_ACK` after OK is clicked.

---

### Buzzer hardware wiring (PENDING — Python + STM32 side)

Everything below still needs to be done when the STM32 hardware and buzzer are available.  
Requirements sourced from `C:\Users\denel\Desktop\VTOL\Copy of vtol_control.xlsx`.

#### Severity scale (from spec)

| Scale | Buzzer behaviour | GCS popup? |
|---|---|---|
| 10 | Very loud buzzer for X seconds | Yes |
| 8–9 | Very loud buzzer | Yes |
| 7 | Loud 3-beep sound | Yes |
| 6 | 2 high beeps | Yes |
| 5 | 1 high beep | Yes |
| 4 | Medium beep | Yes |
| 3 | Low beep | Yes |
| 2 | No buzzer | Error msg only |
| 1 | Nothing | No notification |

#### All alerts requiring buzzer + GCS popup

**Performance (sheet 4.3.1):**

| Req | Condition | Scale | MAVLink source | Threshold |
|---|---|---|---|---|
| PF001 | Airspeed too high | 10 | `VFR_HUD.airspeed` | ≥ 33.33 m/s (120 km/h) |
| PF002 | Airspeed too low / near stall | 10 | `VFR_HUD.airspeed` | ≤ 16.0 m/s (57.6 km/h) |
| PF003 | Altitude > service ceiling | 10 | `VFR_HUD.alt` (ASL) | ≥ 3000 m |
| PF007 | Height > standard operating ceiling | 4 | AGL estimate | > 120 m AGL |
| PF008 | Height > extended operating ceiling | 6 | AGL estimate | > 300 m AGL |
| PF009 | Climb rate excessive | **1** | `VFR_HUD.climb` | > 4 m/s — **no action** |

**Environmental (sheet 4.3.2):**

| Req | Condition | Scale | MAVLink source | Threshold |
|---|---|---|---|---|
| ENV001 | Wind speed too high | 8 | `WIND.speed` | > 12 m/s |

**Navigation (sheet 4.3.6):**

| Req | Condition | Scale | MAVLink source | Threshold |
|---|---|---|---|---|
| NAV002 | Geo-fence breach | 10 | `FENCE_STATUS.breach_status` | != 0 |
| NAV004 | Air traffic within 500 m | 8 | `ADSB_VEHICLE` | distance < 500 m |
| NAV003 | Altitude too low — ground proximity | 7 | AGL estimate | < 10 m AGL |
| NAV001 | Deviation from flight path | 5 | `NAV_CONTROLLER_OUTPUT.xtrack_error` | > 3 m |

**AGL note:** ArduPilot does not expose a direct AGL field. Best approaches in order of preference:
1. `TERRAIN_REPORT.current_height` (requires terrain data loaded on vehicle)
2. `RANGEFINDER.distance` if a rangefinder is fitted
3. `VFR_HUD.alt` minus home altitude (least accurate — ASL-based estimate)

#### Protocol change required

The current protocol `BUZZER_ALERT:message\n` carries no severity. Change to:
```
BUZZER_ALERT:<scale>:<message>\n
```
e.g. `BUZZER_ALERT:10:Airspeed critical — 125 km/h\n`

**Files to update:**
- `plugins/UAV_/notifications.py` — add `scale` parameter to `send_buzzer_alert(scale, message)`
- `plugins/DenelPythonLauncher.cs` — parse scale from the message; vary dialog flash speed for scale ≥ 8 vs lower
- `plugins/UAV_/config.py` — add threshold constants (speeds, altitudes) alongside existing `SAFE_BATTERY_LEVEL`

#### Python implementation steps

1. **Define a buzzer bit in the STM32 bitmask** — agree bit number with STM32 firmware developer, add `BUZZER_BIT` to `plugins/UAV_/protocol/bit_definitions.py`
2. **Add buzzer on/off helpers** to `plugins/UAV_/serial/packet_builder.py` (same pattern as arm/disarm)
3. **Add threshold constants** to `config.py`:
   - `MAX_AIRSPEED_MS = 33.33` (120 km/h)
   - `MIN_AIRSPEED_MS = 16.0` (57.6 km/h)
   - `MAX_ALT_ASL = 3000`
   - `MAX_ALT_AGL_STD = 120`, `MAX_ALT_AGL_EXT = 300`, `MIN_ALT_AGL = 10`
   - `MAX_WIND_MS = 12`
   - `MAX_XTRACK_ERROR = 3`
   - `ADSB_DANGER_DIST = 500`
4. **Wire each condition** in the MAVLink monitoring loop with:
   - Per-condition `_active` flag (debounce)
   - `threading.Thread(target=..., daemon=True).start()` so alerts don't block the loop
   - Call `send_buzzer_alert(scale, message)` from the thread
   - Send buzzer-off to STM32 after ACK received
5. **Implementation priority** (highest severity first):
   1. NAV002 — Geo-fence breach (scale 10)
   2. PF001/PF002 — Overspeed / stall (scale 10)
   3. PF003 — Altitude ceiling (scale 10)
   4. NAV004 — ADS-B traffic (scale 8)
   5. ENV001 — High wind (scale 8)
   6. NAV003 — Low altitude (scale 7)
   7. NAV001 — Path deviation (scale 5)
   8. PF007/PF008 — Height ceiling warnings (scale 4/6)

#### Config changes on the day

| Setting | File | What to change |
|---|---|---|
| `SERIAL_PORT` | `config.py` | `COM7` → actual STM32 COM port |
| `MAVLINK_CONNECTION` | `config.py` | Keep `tcp:127.0.0.1:5763` for hardware; use `5762` for SITL only |
| `BUZZER_BIT` | `bit_definitions.py` | New — confirm bit number with STM32 firmware developer |

#### Potential issues to watch for

- **Threading:** Always call `send_buzzer_alert()` from a background thread — never the main MAVLink loop or it will pause heartbeats during the dialog.
- **Debounce:** Each condition needs its own `_active` flag. Resetting only after the operator dismisses prevents re-alerts while the condition still exists.
- **Multiple simultaneous alerts:** Queue or prioritise by scale — do not fire several popups at once.
- **STM32 buzzer timeout:** If serial drops while buzzer is on it stays on forever. STM32 firmware should auto-stop if no packet received within a few seconds.
