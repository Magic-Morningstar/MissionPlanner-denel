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
STM32 → COM7 (binary serial packets) → main.py → udpin:0.0.0.0:14551 (state + command) → ArduPilot
```
As of the `hardware` branch merge, `MAVLINK_STATE_PORT` and `MAVLINK_COMMAND_PORT` in `state/system_config.py` both point at the same shared UDP endpoint (`udpin:0.0.0.0:14551`) instead of the previous two separate TCP ports (`5762`/`5763`) — an intentional architecture change (enables Herelink digital-link compatibility). `SERIAL_PORT` (STM32 COM port) also now lives in `state/system_config.py` rather than being set elsewhere.

**Startup ordering / connection resilience:** `DenelPythonLauncher.cs` auto-starts `main.py` as soon as the GCS plugin loads — this is typically *before* the operator has manually connected MissionPlanner to the vehicle, so nothing may be listening on `MAVLINK_CONNECTION` yet. `Mavlink_controller.initiate_Connection()` (`plugins/UAV_/mavlink/mavlink_contoller.py`) wraps `mavutil.mavlink_connection()` in a `try`/`except` retry loop (1s interval, `system_Print`-logged) instead of letting a refused connection raise uncaught — matching the existing STM32 serial-retry pattern in `main.py`'s read loop. There is no process-level watchdog (`DenelPythonLauncher.cs` does not restart `main.py` if it exits), so this in-script retry is the only thing standing between a cold-boot race and a permanently-dead bridge for the rest of the GCS session — do not remove it without adding an equivalent safeguard.

**Files:**

| File | Purpose |
|---|---|
| `plugins/DenelPythonLauncher.cs` | C# plugin — auto-starts `plugins/UAV_/main.py` on GCS startup, kills it on exit, logs to `denel_python.log` |
| `plugins/UAV_/main.py` | Entry point — modular Controller class; connects MAVLink + STM32 serial |
| `plugins/UAV_/state/system_config.py` | Central config — `MAVLINK_STATE_PORT`, `MAVLINK_COMMAND_PORT`, `SERIAL_PORT`, bitmask bits, timeouts, debug flags (was `plugins/UAV_/config.py`, which no longer exists) |
| `plugins/UAV_/API/` | Panel state sync + API-side config/model |
| `plugins/UAV_/Menu_UI/gui_main.py` | PySide6 (Qt) operator GUI — `create_gui()`, driven from `main.py` |
| `plugins/UAV_/commands/` | Input translation: `intents.py`, `registry.py`, `translator.py` |
| `plugins/UAV_/requirements.txt` | Pinned Python dependencies — ships in the release ZIP |
| `plugins/UAV_/mavlink/` | Flight controllers: arms, RTL, auto-takeoff, manual, speed control (`mavlink/Services/Pre_Flight_Checks/` holds battery/flight/GPS pre-flight checks) |
| `plugins/UAV_/serial_controller/` | Binary packet protocol: `serial_handler.py`, `packet_Parser.py`, `packet_builder.py` |
| `plugins/UAV_/serial_controller/protocol/bit_definitions.py` | Centralised bitmask definitions |
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
The script connects via `udpin:0.0.0.0:14551` (both `MAVLINK_STATE_PORT` and `MAVLINK_COMMAND_PORT` in `state/system_config.py`). For ArduPilot SITL (sim_vehicle.py), add a matching output so SITL forwards MAVLink to that port, e.g. `--out udp:127.0.0.1:14551`, or add `output add 127.0.0.1:14551` in the MAVProxy console.

**UDP 14551 is reserved for the bridge — do not re-enable it in AutoConnect.** Mission Planner's own
auto-connect used to listen on 14551 as well (`ExtLibs/Utilities/AutoConnect.cs`, `"Mavlink alt
port"`, enabled upstream by default). `AutoConnect.ProcessEntry()` opens a `UdpClient` on the port
and never disposes it, so whichever process binds first owns 14551 for the whole session — a
startup race against `main.py`. Both outcomes fail quietly: when the GCS won, the bridge logged
`not available (PermissionError)` and retried every 2s forever, leaving the STM32 controller dead;
when the bridge won, `ProcessEntry` swallowed the bind error in its `catch`. Adding PySide6 made
the bridge slower to start and tilted the race toward the GCS.

Fixed in two places, both needed: the source default is now `false`, **and** `AutoConnect.Start()`
carries a one-time migration that disables the entry in already-saved profiles. The migration is
not optional — `Start()` only consults the default list when `Settings.Instance["AutoConnect"]` is
null, so on any machine that has run Mission Planner before, the default alone does nothing. It
runs once and sets `AutoConnect_denel14551=done` so a deliberate re-enable is not overridden every
start. Port 14550 is untouched and still auto-connects normally.

**Python requirement:** `DenelPythonLauncher.cs` resolves the interpreter via `ResolvePythonExe()`,
which normally just uses `"python.exe"` from `PATH`. Target machines are expected to have Python and
the bridge's dependencies already installed (see "Release packaging" below).

As an **optional manual override**, it first looks for `plugins\UAV_\python_path.txt` — a one-line
text file holding the full path to a `python.exe`. If present and valid, that interpreter is used
instead. This is the escape hatch for a machine with several Pythons installed, where `PATH` order
decides which one wins and the dependencies may only be present in one of them. Nothing creates this
file; you write it by hand when you need it.

If Python is missing or unusable at launch, `IsPythonAvailable()` shows a warning dialog instead of
failing silently.

**Python dependencies:** pinned in `plugins/UAV_/requirements.txt` (source of truth). It is copied
into the build output, so it ships inside the release ZIP:
```
pip install -r plugins\UAV_\requirements.txt
```
`PySide6` is in there because `main.py` now builds a Qt GUI (`Menu_UI/gui_main.py`) — it is not an
optional extra; the bridge will not start without it.

**Monitoring the script:**
```powershell
Get-Content -Path "C:\ProgramData\Denel GCS\denel_python.log" -Wait   # PowerShell tail -f equivalent
```
`denel_python.log` (the C# launcher's capture of `main.py`'s stdout/stderr) is written to
`C:\ProgramData\Denel GCS\` — **not** next to `MissionPlanner.exe`. Keep it there: it is always
writable regardless of where the ZIP was extracted, and it survives replacing the app folder on an
upgrade. Do not "simplify" it back to the exe directory.

**Known wart:** `controller.log` (Python's own `logging_config.py` output) does *not* follow this.
`utils/logging_config.py` uses a bare relative `DEFAULT_LOG_FILE = "controller.log"`, so it lands in
the working directory — `plugins\UAV_\` — and gets committed by accident. Owned by the UAV_ Python
author; left alone deliberately.

---

### Release packaging

UAV_ Python scripts live in `plugins/UAV_/` in the repo. `MissionPlanner.csproj` has a `<None Include="plugins\UAV_\**\*.py">` item with `CopyToOutputDirectory=PreserveNewest`, so every `.py` file under `plugins/UAV_/` (including new ones) is copied to `bin\<Config>\net461\plugins\UAV_\` automatically on build — no manual copy step needed. A sibling item does the same for `plugins\UAV_\requirements.txt`, so the dependency list ships alongside the scripts it describes.

Distribution is a **plain ZIP of the Release build output** — there is no installer. Build chain:
```powershell
# 1. Build the app (Release). Kill any running MissionPlanner.exe first — it locks DLLs.
"C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe" -v:m -restore -t:Build -p:Configuration=Release MissionPlanner.sln

# 2. Zip the output folder — that folder IS the release.
Compress-Archive -Path bin\Release\net461\* -DestinationPath DenelGCS-<version>.zip
```
The ZIP is extracted anywhere writable on the target machine and run in place. Nothing is
registered with Windows, so "upgrading" is just replacing the folder.

**Target machine requirements** — nothing provisions these automatically any more, so they must
already be true before the GCS is handed over:

- Windows 10/11 and .NET Framework 4.7.2 (near-universal on modern Windows).
- A real Python on `PATH` — **not** the Windows Store stub. Check with `python --version`; if it
  prints an "install from the Store" prompt instead of a version number, install from python.org
  or via `py`/`winget`.
- The bridge's Python dependencies: `pip install -r plugins\UAV_\requirements.txt` (that file
  ships inside the ZIP — see the csproj item above).

Apply the Denel theme on first launch via Config → Planner → Theme → `denel_cyan`.

If Python or its dependencies are missing, `DenelPythonLauncher` shows a warning dialog naming the
`pip install` command at startup rather than failing silently — that dialog is the only signal a
non-technical operator gets, so do not remove it.

**Auto-connect (future):** `ExtLibs/Utilities/AutoConnect.cs` already supports TCP auto-connect. To enable it, set `Enabled = true` on the relevant `ConnectionInfo` entry and set the target IP/port. No other code changes needed.
