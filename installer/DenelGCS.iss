; DenelGCS.iss — Inno Setup script for the Denel GCS installer.
;
; Bundles the Release build output plus an offline Python installer and
; pre-downloaded wheels for pymavlink/pyserial, so the target machine needs
; no internet access at install time. See installer\fetch-offline-deps.ps1
; for how redist\ and wheels\ are (re)generated before compiling this script.
;
; Build with:
;   & "C:\Program Files (x86)\Inno Setup 6\ISCC.exe" installer\DenelGCS.iss

#define MyAppName "Denel GCS"
#define MyAppVersion "1.0.0"
#define MyAppPublisher "Denel Aeronautics"
#define MyAppExeName "MissionPlanner.exe"
#define PythonVersion "3.13.14"
#define PythonInstaller "python-" + PythonVersion + "-amd64.exe"
; Registry key the official python.org installer writes for this version —
; used post-install to resolve the real python.exe path without relying on
; a PATH refresh inside this Setup process.
#define PythonRegKey "SOFTWARE\Python\PythonCore\3.13\InstallPath"

[Setup]
; This GUID must never change across releases — it's how Windows identifies
; upgrades/uninstalls for this app. Generated once for this project.
AppId={{19DF7B9A-1B96-46F5-9042-CE9FFAE28739}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
DefaultDirName={autopf}\{#MyAppName}
DefaultGroupName={#MyAppName}
DisableProgramGroupPage=yes
OutputDir=output
OutputBaseFilename=DenelGCS-Setup-{#MyAppVersion}
Compression=lzma2/max
SolidCompression=yes
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
PrivilegesRequired=admin
SetupIconFile=..\mpdesktop.ico
LicenseFile=..\COPYING.txt
UninstallDisplayIcon={app}\{#MyAppExeName}
WizardStyle=modern

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "Create a &desktop icon"; GroupDescription: "Additional icons:"

[UninstallDelete]
; Sweeps up runtime-generated files Inno's normal uninstall doesn't know about —
; plugins\UAV_\python_path.txt (written by CurStepChanged, not part of [Files]),
; Python __pycache__ directories, denel_python.log if ever written next to the exe
; by an older build. Safe as a full-directory wipe: {app} is exclusively this app's
; install directory, nothing else should ever be there.
Type: filesandordirs; Name: "{app}"

[Files]
; Main application payload — everything from the Release build output.
Source: "..\bin\Release\net461\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

; Offline prerequisites — extracted to {tmp} only, never left in the installed app dir.
Source: "redist\{#PythonInstaller}"; DestDir: "{tmp}"; Flags: deleteafterinstall
Source: "wheels\*"; DestDir: "{tmp}\wheels"; Flags: deleteafterinstall recursesubdirs createallsubdirs
Source: "..\Plugins\UAV_\requirements.txt"; DestDir: "{tmp}"; Flags: deleteafterinstall

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; IconFilename: "{app}\mpdesktop.ico"
Name: "{group}\Uninstall {#MyAppName}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; IconFilename: "{app}\mpdesktop.ico"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "Launch {#MyAppName}"; Flags: nowait postinstall skipifsilent

[Code]

var
  ProgressPage: TOutputProgressWizardPage;

procedure InitializeWizard();
begin
  ProgressPage := CreateOutputProgressPage('Setting Up Python',
    'Please wait while Denel GCS finishes setting up its Python components. ' +
    'This can take several minutes on the first install.');
end;

// ── .NET Framework 4.7.2+ check — warn, don't block. Windows 10 1803+ and ──────────
// Windows 11 ship with this already, so we don't bundle a ~60MB .NET installer.
function IsDotNet472OrLater(): Boolean;
var
  Release: Cardinal;
begin
  Result := RegQueryDWordValue(HKLM, 'SOFTWARE\Microsoft\NET Framework Setup\NDP\v4\Full', 'Release', Release)
            and (Release >= 461808); // 4.7.2 minimum release value
end;

function InitializeSetup(): Boolean;
begin
  Result := True;
  if not IsDotNet472OrLater() then
    MsgBox('Denel GCS requires .NET Framework 4.7.2 or later. Setup will continue, ' +
           'but the application may not run correctly until it is installed/updated ' +
           'via Windows Update.', mbInformation, MB_OK);
end;

// ── Python provisioning ─────────────────────────────────────────────────────────────
// We deliberately do NOT try to reuse whatever Python happens to already be on PATH:
// our offline wheels (fastcrc/lxml have compiled, ABI-specific builds) are pinned to
// the exact bundled Python version below, so a pre-existing Python of a different
// minor version would pass a "python --version" check but then fail the offline pip
// install (this was caught by real-machine testing — an existing 3.14 install broke
// the offline install of wheels built for 3.13). Always installing our own bundled
// Python (side-by-side; the official installer no-ops/repairs if already present) is
// slightly more install time/disk, but fully deterministic. PrependPath=0 — we don't
// want to silently take over the system-wide "python" command for anything other
// than this app; DenelPythonLauncher.cs is told the exact resolved path (see below)
// rather than relying on ambient PATH order.
function GetBundledPythonPath(var PythonExe: String): Boolean;
var
  InstallPath: String;
begin
  Result := False;
  if RegQueryStringValue(HKLM64, '{#PythonRegKey}', '', InstallPath) then
  begin
    PythonExe := InstallPath + 'python.exe';
    Result := FileExists(PythonExe);
    if Result then
      Log('GetBundledPythonPath: found bundled Python at ' + PythonExe)
    else
      Log('GetBundledPythonPath: registry key present but python.exe missing at ' + PythonExe);
  end
  else
    Log('GetBundledPythonPath: registry key not found.');
end;

// ── Post-install: install our bundled Python, offline-install pymavlink/pyserial,
// and record the resolved python.exe path for the app to use at runtime.
// Failures here are non-fatal to the overall app install — the GCS itself must still
// install even if this step has a problem. Plugins\DenelPythonLauncher.cs has a
// runtime check that surfaces a visible warning if Python turns out to be missing
// when the app actually launches, as a fallback safety net for this case.
procedure CurStepChanged(CurStep: TSetupStep);
var
  ResultCode: Integer;
  PythonExe, WheelsDir, ReqFile, PathFile: String;
begin
  if CurStep = ssPostInstall then
  begin
    Log('CurStepChanged(ssPostInstall): starting Python provisioning.');
    ProgressPage.Show();
    try
      ProgressPage.SetText('Installing Python runtime (this may take a few minutes)...', '');
      ProgressPage.SetProgress(1, 3);

      Log('Installing bundled Python: {#PythonInstaller}');
      if not (Exec(ExpandConstant('{tmp}\{#PythonInstaller}'),
                   '/quiet InstallAllUsers=1 PrependPath=0 Include_test=0',
                   '', SW_HIDE, ewWaitUntilTerminated, ResultCode)
              and (ResultCode = 0)) then
        Log('Bundled Python installer exited with code ' + IntToStr(ResultCode) +
            ' (continuing — Python may already be installed at this version).');

      if not GetBundledPythonPath(PythonExe) then
      begin
        Log('Python still not found after running bundled installer — aborting provisioning.');
        MsgBox('Python installation did not complete successfully. The STM32/joystick ' +
               'controller bridge will not work until Python is installed manually. See ' +
               'https://www.python.org/downloads/ or re-run this installer.',
               mbError, MB_OK);
        Exit;
      end;

      ProgressPage.SetText('Installing Python dependencies...', '');
      ProgressPage.SetProgress(2, 3);

      // Offline pip install of pymavlink/pyserial (+ transitive deps) from bundled wheels.
      WheelsDir := ExpandConstant('{tmp}\wheels');
      ReqFile   := ExpandConstant('{tmp}\requirements.txt');
      Log('Running offline pip install via: ' + PythonExe);
      if Exec(PythonExe,
              '-m pip install --no-index --find-links="' + WheelsDir + '" -r "' + ReqFile + '"',
              '', SW_HIDE, ewWaitUntilTerminated, ResultCode)
         and (ResultCode = 0) then
        Log('pip install completed successfully (exit code 0).')
      else
      begin
        Log('pip install failed (exit code ' + IntToStr(ResultCode) + ').');
        MsgBox('Installing the Python dependencies (pymavlink/pyserial) failed (exit code ' +
               IntToStr(ResultCode) + '). Run manually: "' + PythonExe + '" -m pip install -r "' +
               ExpandConstant('{app}') + '\plugins\UAV_\requirements.txt"',
               mbError, MB_OK);
      end;

      ProgressPage.SetText('Finishing up...', '');
      ProgressPage.SetProgress(3, 3);

      // Record the resolved python.exe path for DenelPythonLauncher.cs — it reads this
      // instead of trusting a bare "python.exe" PATH lookup, which is unpredictable if
      // another Python is also installed on this machine.
      PathFile := ExpandConstant('{app}\plugins\UAV_\python_path.txt');
      if not SaveStringToFile(PathFile, PythonExe, False) then
        Log('WARNING: failed to write ' + PathFile);
    finally
      ProgressPage.Hide();
    end;
  end;
end;

// NOTE for future editors: do NOT add Python removal to the uninstaller below.
// The system Python this installer sets up (or reuses) may be shared with other
// software on the machine — uninstalling Denel GCS must only remove its own
// application directory and shortcuts, which is Inno Setup's default behavior.
