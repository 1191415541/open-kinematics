from __future__ import annotations

import json
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).parents[4]


def test_wheel_contains_current_platform_native_kernel(tmp_path: Path) -> None:
    if sys.platform == "win32":
        library_name = "axle_dynamics_native.dll"
    elif sys.platform == "darwin":
        library_name = "libaxle_dynamics_native.dylib"
    else:
        library_name = "libaxle_dynamics_native.so"
    native_dir = (
        ROOT
        / "packages"
        / "suspension_multibody"
        / "src"
        / "suspension_multibody"
        / "native"
    )
    assert (native_dir / library_name).is_file()
    assert (native_dir / "native_build.json").is_file()
    subprocess.run(
        [
            "uv",
            "build",
            "--package",
            "suspension-multibody",
            "--wheel",
            "--out-dir",
            str(tmp_path),
        ],
        cwd=ROOT,
        check=True,
    )
    wheel = next(tmp_path.glob("*.whl"))
    library_path = f"suspension_multibody/native/{library_name}"
    metadata_path = "suspension_multibody/native/native_build.json"
    with zipfile.ZipFile(wheel) as archive:
        assert library_path in archive.namelist()
        assert metadata_path in archive.namelist()
        metadata = json.loads(archive.read(metadata_path))
    assert metadata["abi_version"] == 14
    assert metadata["vehicle_abi_version"] == 21
    assert metadata["source"] == "cpp/axle_dynamics/axle_kernel.cpp"
