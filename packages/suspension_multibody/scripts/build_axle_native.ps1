param(
    [ValidateSet("Release", "Debug")]
    [string]$Configuration = "Release",
    [string]$Compiler = ""
)

$ErrorActionPreference = "Stop"
$packageRoot = Split-Path -Parent $PSScriptRoot
$repositoryRoot = Split-Path -Parent (Split-Path -Parent $packageRoot)
$source = Join-Path $repositoryRoot "cpp\axle_dynamics\axle_kernel.cpp"
$include = Join-Path $repositoryRoot "cpp\axle_dynamics"
$nativeDir = Join-Path $packageRoot "src\suspension_multibody\native"
$output = Join-Path $nativeDir "axle_dynamics_native.dll"
$metadata = Join-Path $nativeDir "native_build.json"
$temporaryOutput = Join-Path $env:TEMP "axle_dynamics_native.dll"

New-Item -ItemType Directory -Force -Path $nativeDir | Out-Null

if ($Compiler) {
    $compiler = (Resolve-Path -LiteralPath $Compiler).Path
} else {
    $compiler = $null
    $candidate = Get-Command x86_64-w64-mingw32-g++ -ErrorAction SilentlyContinue
    if ($candidate) {
        $compiler = $candidate.Source
    }
    if (-not $compiler) {
        $wingetRoot = Join-Path $env:LOCALAPPDATA "Microsoft\WinGet\Packages"
        $candidate = Get-ChildItem -LiteralPath $wingetRoot -Filter "x86_64-w64-mingw32-g++.exe" `
            -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($candidate) {
            $compiler = $candidate.FullName
        }
    }
    if (-not $compiler) {
        $candidate = Get-Command g++ -ErrorAction SilentlyContinue
        if ($candidate) {
            $compiler = $candidate.Source
        }
    }
    if (-not $compiler) {
        throw "No C++ compiler found; install a 64-bit MinGW-w64 toolchain"
    }
}

$machine = (& $compiler -dumpmachine).Trim()
if ($machine -notmatch "x86_64|amd64") {
    throw "The axle native kernel requires a 64-bit compiler; detected target '$machine'"
}
$flags = @(
    "-std=c++17",
    "-shared",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-fno-fast-math",
    "-fopenmp",
    "-static",
    "-static-libgcc",
    "-static-libstdc++",
    "-I$include"
)
if ($Configuration -eq "Release") {
    # Keep IEEE-sensitive transformations disabled while allowing the compiler
    # to optimize the small matrix and directional-derivative kernels.
    $flags += @("-O3", "-flto")
} else {
    $flags += @("-O0", "-g")
}

Remove-Item -LiteralPath $temporaryOutput -Force -ErrorAction SilentlyContinue
& $compiler @flags $source "-o" $temporaryOutput
if ($LASTEXITCODE -ne 0) {
    throw "C++ build failed with exit code $LASTEXITCODE"
}
Copy-Item -LiteralPath $temporaryOutput -Destination $output -Force

$version = (& $compiler --version | Select-Object -First 1)
$record = [ordered]@{
    abi_version = 14
    compiler = $compiler
    compiler_version = $version
    configuration = $Configuration
    flags = $flags
    source = "cpp/axle_dynamics/axle_kernel.cpp"
}
$record | ConvertTo-Json -Depth 4 | Set-Content -Encoding utf8 $metadata
Write-Output $output
