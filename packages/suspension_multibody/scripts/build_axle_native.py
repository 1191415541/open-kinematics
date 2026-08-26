"""Build the axle native library for the current host platform."""

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

ABI_VERSION = 14


def main() -> int:
    """Build the host-native shared library and metadata."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--configuration",
        choices=("Release", "Debug"),
        default="Release",
    )
    args = parser.parse_args()
    script = Path(__file__).resolve()
    package_root = script.parents[1]
    repository_root = script.parents[3]
    if struct.calcsize("P") != 8:
        raise RuntimeError("the axle native kernel requires a 64-bit host")
    if sys.platform == "win32":
        powershell = shutil.which("pwsh") or shutil.which("powershell")
        if powershell is None:
            raise RuntimeError("PowerShell is required for the Windows build")
        subprocess.run(
            [
                powershell,
                "-NoProfile",
                "-File",
                str(script.with_suffix(".ps1")),
                "-Configuration",
                args.configuration,
            ],
            cwd=repository_root,
            check=True,
        )
        return 0

    compiler = (
        os.environ.get("CXX")
        or shutil.which("c++")
        or shutil.which("g++")
        or shutil.which("clang++")
    )
    if compiler is None:
        raise RuntimeError("no C++ compiler found in CXX or PATH")
    source = repository_root / "cpp" / "axle_dynamics" / "axle_kernel.cpp"
    include = source.parent
    native_dir = package_root / "src" / "suspension_multibody" / "native"
    native_dir.mkdir(parents=True, exist_ok=True)
    if sys.platform == "darwin":
        output_name = "libaxle_dynamics_native.dylib"
        shared_flag = "-dynamiclib"
    else:
        output_name = "libaxle_dynamics_native.so"
        shared_flag = "-shared"
    output = native_dir / output_name
    flags = [
        "-std=c++17",
        shared_flag,
        "-fPIC",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-fno-fast-math",
        # The Jacobian assembly loop carries no reduction, so results are
        # identical for any thread count; without this the pragma is ignored
        # and the kernel simply runs single-threaded.
        "-fopenmp",
        f"-I{include}",
        "-O2" if args.configuration == "Release" else "-O0",
    ]
    if args.configuration == "Debug":
        flags.append("-g")
    with tempfile.TemporaryDirectory(prefix="axle-native-") as temp:
        temporary_output = Path(temp) / output_name
        subprocess.run(
            [compiler, *flags, str(source), "-o", str(temporary_output)],
            cwd=repository_root,
            check=True,
        )
        shutil.copy2(temporary_output, output)
    version = subprocess.run(
        [compiler, "--version"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()[0]
    metadata = {
        "abi_version": ABI_VERSION,
        "compiler": compiler,
        "compiler_version": version,
        "configuration": args.configuration,
        "flags": flags,
        "platform": platform.platform(),
        "architecture": platform.machine(),
        "source": "cpp/axle_dynamics/axle_kernel.cpp",
    }
    (native_dir / "native_build.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
