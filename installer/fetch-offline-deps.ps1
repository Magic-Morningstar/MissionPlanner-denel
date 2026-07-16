<#
.SYNOPSIS
  Regenerates installer\redist\ and installer\wheels\ - the offline build inputs for
  DenelGCS.iss. Not run automatically as part of the app build; run manually whenever
  Plugins\UAV_\requirements.txt or $PythonVersion below changes.

.NOTES
  Output is gitignored (see .gitignore) - this script is the source of truth for
  regenerating it, analogous to `git submodule update --init` for the mono/mavlink
  submodules.
#>

param(
    [string]$PythonVersion = "3.13.14"
)

$ErrorActionPreference = "Stop"

$RepoRoot        = Split-Path -Parent $PSScriptRoot
$InstallerDir    = $PSScriptRoot
$RequirementsTxt = Join-Path $RepoRoot "Plugins\UAV_\requirements.txt"
$WheelsDir       = Join-Path $InstallerDir "wheels"
$RedistDir       = Join-Path $InstallerDir "redist"
$WheelBuildDir   = Join-Path $InstallerDir ".wheelbuild"

if (-not (Test-Path $RequirementsTxt)) {
    throw "requirements.txt not found at $RequirementsTxt"
}

New-Item -ItemType Directory -Force -Path $WheelsDir, $RedistDir | Out-Null

# 1. Scratch venv for a clean, reproducible download environment.
Write-Host "Creating scratch venv at $WheelBuildDir ..."
if (Test-Path $WheelBuildDir) {
    Remove-Item -Recurse -Force $WheelBuildDir
}
py -3 -m venv $WheelBuildDir
$VenvPython = Join-Path $WheelBuildDir "Scripts\python.exe"
& $VenvPython -m pip install --upgrade pip --quiet

# 2. Download wheels only - fail loudly on anything that would need a source build.
Write-Host "Downloading wheels for Python $PythonVersion / win_amd64 (binary-only) ..."
$ShortVer = ($PythonVersion -split '\.')[0..1] -join ''   # "3.13.14" -> "313"

& $VenvPython -m pip download `
    -r $RequirementsTxt `
    -d $WheelsDir `
    --only-binary=":all:" `
    --python-version $ShortVer `
    --implementation cp `
    --abi "cp$ShortVer" `
    --platform win_amd64

if ($LASTEXITCODE -ne 0) {
    throw "pip download failed (exit $LASTEXITCODE) - see output above for the offending package."
}

# 3. Assert every downloaded file is an installable wheel, not a source dist.
$badFiles = Get-ChildItem $WheelsDir -File | Where-Object { $_.Extension -ne ".whl" }
if ($badFiles) {
    $names = ($badFiles | ForEach-Object { $_.Name }) -join ", "
    throw ("Non-wheel file(s) downloaded (would need a C compiler on the target machine " +
           "to install offline): $names. A binary wheel may not be published for this " +
           "package/Python/platform combination - investigate before building the installer.")
}
Write-Host "OK: $((Get-ChildItem $WheelsDir -File).Count) wheel(s) in $WheelsDir"

# 4. Fetch the python.org offline installer.
$PythonInstallerName = "python-$PythonVersion-amd64.exe"
$PythonInstallerPath = Join-Path $RedistDir $PythonInstallerName

if (Test-Path $PythonInstallerPath) {
    Write-Host "Python installer already present at $PythonInstallerPath - skipping download."
} else {
    $url = "https://www.python.org/ftp/python/$PythonVersion/$PythonInstallerName"
    Write-Host "Downloading $url ..."
    Invoke-WebRequest -Uri $url -OutFile $PythonInstallerPath
    Write-Host "Saved to $PythonInstallerPath"
    Write-Host ("NOTE: verify the SHA256 of this file against the hash published on the " +
                "python.org downloads page before shipping an installer built from it.")
}

Write-Host ""
Write-Host "Done. installer\wheels\ and installer\redist\ are ready for DenelGCS.iss."
