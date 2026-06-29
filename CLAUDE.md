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
│       ├── Panel1 (left) = hud1  [Dock=Fill — fills to square via SubMainLeft_Resize]
│       └── Panel2 (right) = tableMap (map)
└── Panel2 (bottom, full-width) = tabControlactions + panel_persistent
```

**HUD proportional resize:** `SubMainLeft_Resize` handler sets `SplitterDistance = Math.Min(SubMainLeft.Height, SubMainLeft.Width / 2)` — keeps the HUD panel square, map gets the rest.

**HUD auto-undock on startup:** When two monitors are detected (`Screen.AllScreens.Length > 1`), `FlightData_Load` calls `AutoUndockHUD()` via `BeginInvoke` (deferred until after first paint). `AutoUndockHUD()` collapses `SubMainLeft.Panel1`, creates a borderless `Form`, moves `hud1` (Dock=Fill) into it, positions it at `Screen.AllScreens[1].Bounds.Location`, and maximizes it. `dropout_FormClosed` re-attaches `hud1` and un-collapses the panel. The `huddropout` bool prevents double-undocking.

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

**Files:**

| File | Purpose |
|---|---|
| `plugins/DenelPythonLauncher.cs` | C# plugin — auto-starts `plugins/UAV_/main.py` on GCS startup, kills it on exit, logs to `denel_python.log` |
| `plugins/UAV_/main.py` | Entry point — modular Controller class; connects MAVLink + STM32 serial |
| `plugins/UAV_/config.py` | Central config — `MAVLINK_CONNECTION`, `SERIAL_PORT`, bitmask bits, timeouts, debug flags |
| `plugins/UAV_/mavlink/` | Flight controllers: arms, RTL, auto-takeoff, manual, speed control |
| `plugins/UAV_/serial/` | Binary packet protocol: `serial_handler.py`, `packet_Parser.py`, `packet_builder.py` |
| `plugins/UAV_/protocol/bit_definitions.py` | Centralised bitmask definitions |
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

Build Release, copy UAV_ scripts, ZIP:
```bash
# 1. Build
MSBuild.exe -v:m -restore -t:Build -p:Configuration=Release MissionPlanner.sln

# 2. Copy new Python scripts into output
xcopy /E /I "C:\Users\denel\Desktop\VTOL\UAV_" "bin\Release\net461\plugins\UAV_"

# 3. ZIP for distribution (PowerShell)
Compress-Archive -Path "bin\Release\net461\*" -DestinationPath "DenelGCS_Release.zip" -Force
```

**Target machine requirements:** Windows 10/11, Python 3 + `pip install pymavlink pyserial`. No installer needed — unzip and run `MissionPlanner.exe`. Apply Denel theme on first launch via Config → Planner → Theme → `denel_cyan`.

**Auto-connect (future):** `ExtLibs/Utilities/AutoConnect.cs` already supports TCP auto-connect. To enable it, set `Enabled = true` on the relevant `ConnectionInfo` entry and set the target IP/port. No other code changes needed.

---

### Buzzer notification system (planned — not yet implemented)

A hardware buzzer attached to the STM32 will be controlled by the Python script. When the buzzer fires, the GCS must show a dismissible alert; when the operator dismisses it, the Python script stops the buzzer.

**Communication channel:** TCP socket on `127.0.0.1:5764` (separate from MAVLink on 5763).

**Protocol (simple text lines):**

| Direction | Message | Meaning |
|---|---|---|
| Python → GCS | `BUZZER_ALERT:some message\n` | Activate buzzer + show dialog |
| GCS → Python | `BUZZER_ACK\n` | Operator dismissed — stop buzzer |

**C# side (`plugins/DenelPythonLauncher.cs`):**
- Start a `TcpListener` on `127.0.0.1:5764` in a background thread launched from `Loaded()`
- On connection: read lines; on `BUZZER_ALERT:msg` call `MainV2.instance.Invoke(...)` to show `CustomMessageBox.Show(msg, "GCS Alert", OK, Warning)` — `Invoke` (synchronous) blocks until user clicks OK
- After `Invoke` returns, write `BUZZER_ACK\n` back to the client
- Set `_notifRunning = false` and stop listener in `Exit()`

**Python side (`plugins/UAV_/main.py` or a new `notifications.py`):**
```python
import socket
from config import GCS_NOTIFICATION_PORT  # = 5764

def send_buzzer_alert(message: str) -> bool:
    with socket.create_connection(('127.0.0.1', GCS_NOTIFICATION_PORT), timeout=5) as s:
        s.sendall(f"BUZZER_ALERT:{message}\n".encode())
        ack = s.makefile().readline().strip()
        return ack == "BUZZER_ACK"
```
Call `send_buzzer_alert(...)` when the buzzer condition is detected; after it returns `True`, send the stop-buzzer command to the STM32.

**Testing without hardware:**
```powershell
python -c "import socket; s=socket.create_connection(('127.0.0.1',5764)); s.sendall(b'BUZZER_ALERT:Test alert\n'); print(s.makefile().readline())"
```
Expected: dialog appears in GCS; terminal prints `BUZZER_ACK` after OK is clicked.

**Note:** The specific flight condition that triggers the buzzer (low battery, GPS loss, STM32 bitmask bit, etc.) is a flight-operations decision to be confirmed before implementation.
