# Compatibility wrapper for the axle native build.
#
# The real build is `packages/suspension_kernel` (CMake + Ninja).  This script
# exists because the axle package's documentation, CI and justfile referred to
# it for years; it delegates and then mirrors the product into this package's
# `native` directory, where the axle ctypes boundary loads it from.
#
# PGO build modes were dropped in K1: CMake cannot express `-fprofile-generate`
# and `-fprofile-use` as one option (they are two configure+build passes), so
# the capability is retired rather than emulated per platform.

param(
    [ValidateSet("Release", "Debug")]
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$repositoryRoot = Resolve-Path (Join-Path $PSScriptRoot "..\..\..")
& python (Join-Path $PSScriptRoot "build_axle_native.py") --configuration $Configuration
if ($LASTEXITCODE -ne 0) {
    throw "the kernel build failed with exit code $LASTEXITCODE"
}
