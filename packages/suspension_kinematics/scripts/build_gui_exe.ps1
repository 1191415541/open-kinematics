param(
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

$RepoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$VenvPath = Join-Path $RepoRoot ".venv-build-gui"
$PythonPath = Join-Path $VenvPath "Scripts\python.exe"
$ExePath = Join-Path $RepoRoot "dist\KinematicsWorkbench\KinematicsWorkbench.exe"

Set-Location $RepoRoot

if ($Clean) {
    Remove-Item -Recurse -Force -ErrorAction SilentlyContinue `
        $VenvPath, `
        (Join-Path $RepoRoot "build"), `
        (Join-Path $RepoRoot "dist")
}

if (-not (Test-Path $PythonPath)) {
    python -m venv $VenvPath
}

& $PythonPath -m pip install --upgrade pip

# Install only the GUI runtime set. Avoid installing the project's CLI/parquet/dev
# dependencies so PyInstaller has less to discover and bundle.
& $PythonPath -m pip install `
    pyinstaller `
    numpy `
    scipy `
    pydantic `
    pyyaml `
    cma `
    matplotlib `
    python-docx `
    tksheet
& $PythonPath -m pip install --no-deps --editable .

& $PythonPath -m PyInstaller --noconfirm --clean .\packaging\kinematics_gui.spec

if (-not (Test-Path $ExePath)) {
    throw "Executable was not created: $ExePath"
}

$SmokeProcess = Start-Process -FilePath $ExePath -ArgumentList "--smoke-test" -PassThru
if (-not $SmokeProcess.WaitForExit(30000)) {
    Stop-Process -Id $SmokeProcess.Id -Force -ErrorAction SilentlyContinue
    throw "Executable smoke test timed out"
}
if ($SmokeProcess.ExitCode -ne 0) {
    throw "Executable smoke test failed with exit code $($SmokeProcess.ExitCode)"
}

$sizeBytes = (Get-ChildItem -Recurse (Split-Path $ExePath -Parent) | Measure-Object -Property Length -Sum).Sum
$sizeMiB = [math]::Round($sizeBytes / 1MB, 1)

Write-Host "Built: $ExePath"
Write-Host "Bundle size: $sizeMiB MiB"
