from __future__ import annotations

import json
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).parents[4]


def test_wheel_contains_current_platform_native_kernel(tmp_path: Path) -> None:
    # The kernel is built by `packages/suspension_kernel` and mirrored into this
    # package's `native` directory, which is the path the wheel packages and the
    # ctypes boundary loads.  K1 renamed the library from `axle_dynamics_native`.
    if sys.platform == "win32":
        library_name = "suspension_kernel.dll"
    elif sys.platform == "darwin":
        library_name = "libsuspension_kernel.dylib"
    else:
        library_name = "libsuspension_kernel.so"
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
    assert metadata["abi_version"] == 15
    assert metadata["vehicle_abi_version"] == 30
    assert metadata["source"] == "cpp/axle_dynamics/axle_kernel.cpp"




