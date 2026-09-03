"""Local Adams/Car installation and field discovery."""

from __future__ import annotations

import csv
import os
import re
import shutil
import subprocess
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path

DEFAULT_PROFILE = "adams-car-2024.1"
DEFAULT_HOME = Path(r"C:\Program Files\MSC.Software\Adams\2024_1")


@dataclass(frozen=True)
class AdamsProfile:
    """Discovered non-proprietary metadata for one Adams installation."""

    name: str
    home: str | None
    executable: str | None
    version: str | None
    license_file: str | None
    template_id: str | None
    subsystem_id: str | None
    database_path: str | None
    report_dictionary: str | None
    export_fields: tuple[str, ...]
    available: bool
    license_probe: str
    message: str

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


def discover_profile(
    name: str = DEFAULT_PROFILE, home: str | Path | None = None
) -> AdamsProfile:
    """Discover paths and report fields without starting the Adams GUI."""
    if name != DEFAULT_PROFILE:
        return AdamsProfile(
            name=name,
            home=None,
            executable=None,
            version=None,
            license_file=os.environ.get("MSC_LICENSE_FILE"),
            template_id=None,
            subsystem_id=None,
            database_path=None,
            report_dictionary=None,
            export_fields=(),
            available=False,
            license_probe="unknown-profile",
            message=f"unknown Adams profile {name!r}",
        )
    candidates: list[Path] = []
    env_home = os.environ.get("SUSPENSION_MULTIBODY_ADAMS_HOME")
    if home is not None:
        candidates.append(Path(home))
    if env_home:
        candidates.append(Path(env_home))
    path_executable = shutil.which("adams2024_1.bat")
    if path_executable:
        candidates.append(Path(path_executable).parent.parent)
    candidates.extend(_registry_homes())
    candidates.extend(
        [
            DEFAULT_HOME,
            Path(r"C:\Program Files\MSC.Software\Adams\2024_1"),
            Path(r"C:\MSC.Software\Adams\2024_1"),
            Path(r"G:\MSC.Software\Adams\2024_1"),
        ]
    )
    candidates = _unique_paths(candidates)
    root = next((candidate for candidate in candidates if candidate.is_dir()), None)
    if root is None:
        return _unavailable(name, "Adams/Car 2024.1 installation was not found")

    executable = root / "bin" / "adams2024_1.bat"
    database = root / "acar" / "shared_car_database.cdb"
    template = database / "templates.tbl" / "_double_wishbone.tpl"
    subsystem = database / "subsystems.tbl" / "TR_Front_Suspension.sub"
    dictionary = root / "acar" / "acar_report_dictionary.csv"
    fields = _read_report_fields(dictionary)
    version, version_detail = _run_version_probe(executable)
    license_probe, license_detail = _run_license_probe(executable)
    detail = "; ".join(value for value in (version_detail, license_detail) if value)
    available = (
        executable.is_file()
        and template.is_file()
        and subsystem.is_file()
        and version == "2024.1"
        and license_probe == "passed"
    )
    if available:
        message = f"Adams/Car {version} discovered at {root}"
    else:
        failures = []
        if not executable.is_file():
            failures.append("executable")
        if not template.is_file():
            failures.append("template")
        if not subsystem.is_file():
            failures.append("subsystem")
        if version != "2024.1":
            failures.append("version probe")
        if license_probe != "passed":
            failures.append(f"license probe ({license_probe})")
        message = (
            f"Adams installation at {root} failed: {', '.join(failures)}; {detail}"
        )
    return AdamsProfile(
        name=name,
        home=str(root),
        executable=str(executable) if executable.is_file() else None,
        version=version,
        license_file=os.environ.get("MSC_LICENSE_FILE"),
        template_id="_double_wishbone.tpl" if template.is_file() else None,
        subsystem_id="TR_Front_Suspension.sub" if subsystem.is_file() else None,
        database_path=str(database) if database.is_dir() else None,
        report_dictionary=str(dictionary) if dictionary.is_file() else None,
        export_fields=fields,
        available=available,
        license_probe=license_probe,
        message=message,
    )


def probe_profile(name: str = DEFAULT_PROFILE) -> AdamsProfile:
    """Alias used by API and CLI callers."""
    return discover_profile(name)


def write_profile(profile: AdamsProfile, path: str | Path) -> None:
    """Write a non-proprietary JSON profile cache."""
    import json

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(profile.as_dict(), indent=2, sort_keys=True), encoding="utf-8"
    )


def _adams_environment(runtime: Path) -> dict[str, str]:
    """Use a run-local writable database without changing the user's config."""
    home = runtime / "adams_home"
    private_database = home / "private.cdb"
    private_database.mkdir(parents=True, exist_ok=True)
    (home / ".acar.cfg").write_text(
        "! Open Kinematics run-local Adams/Car configuration\n"
        f"DATABASE private {private_database.as_posix()}\n"
        "DEFAULT_WRITE_DB private\n",
        encoding="ascii",
    )
    environment = os.environ.copy()
    environment["HOME"] = str(home)
    environment.pop("DEFAULT_WRITE_DB", None)
    return environment


def _unavailable(name: str, message: str) -> AdamsProfile:
    return AdamsProfile(
        name=name,
        home=None,
        executable=None,
        version=None,
        license_file=os.environ.get("MSC_LICENSE_FILE"),
        template_id=None,
        subsystem_id=None,
        database_path=None,
        report_dictionary=None,
        export_fields=(),
        available=False,
        license_probe="not-run",
        message=message,
    )


def _read_report_fields(path: Path) -> tuple[str, ...]:
    if not path.is_file():
        return ()
    fields: list[str] = []
    try:
        with path.open(newline="", encoding="utf-8-sig") as stream:
            for row in csv.reader(stream):
                if len(row) >= 2 and row[0].strip().lower() == "channel":
                    field = row[1].strip()
                    if field and field not in fields:
                        fields.append(field)
    except (OSError, UnicodeError, csv.Error):
        return ()
    return tuple(fields)


def _unique_paths(paths: list[Path]) -> list[Path]:
    unique: list[Path] = []
    seen: set[str] = set()
    for path in paths:
        key = os.path.normcase(os.path.abspath(path))
        if key not in seen:
            seen.add(key)
            unique.append(path)
    return unique


def _registry_homes() -> tuple[Path, ...]:
    """Return Adams install roots recorded by Windows uninstall entries."""
    if os.name != "nt":
        return ()
    try:
        import winreg
    except ImportError:
        return ()

    roots: list[Path] = []
    uninstall = r"SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall"
    views = (
        0,
        getattr(winreg, "KEY_WOW64_64KEY", 0),
        getattr(winreg, "KEY_WOW64_32KEY", 0),
    )
    for hive in (winreg.HKEY_LOCAL_MACHINE, winreg.HKEY_CURRENT_USER):
        for view in views:
            try:
                with winreg.OpenKey(
                    hive, uninstall, 0, winreg.KEY_READ | view
                ) as parent:
                    for index in range(winreg.QueryInfoKey(parent)[0]):
                        try:
                            with winreg.OpenKey(
                                parent, winreg.EnumKey(parent, index)
                            ) as entry:
                                display_name = str(
                                    winreg.QueryValueEx(entry, "DisplayName")[0]
                                )
                                if "adams 2024.1" not in display_name.lower():
                                    continue
                                location = str(
                                    winreg.QueryValueEx(entry, "InstallLocation")[0]
                                ).strip()
                                if location:
                                    roots.append(Path(location))
                        except OSError:
                            continue
            except OSError:
                continue
    return tuple(_unique_paths(roots))


def _run_version_probe(executable: Path) -> tuple[str | None, str]:
    if not executable.is_file():
        return None, "adams executable is missing"
    try:
        with tempfile.TemporaryDirectory(prefix="suspension_multibody_adams_") as cwd:
            completed = subprocess.run(
                [str(executable), "-v"],
                cwd=cwd,
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
    except (OSError, subprocess.SubprocessError) as exc:
        return None, str(exc)
    output = f"{completed.stdout}\n{completed.stderr}"
    match = re.search(
        r"Version\s*=\s*([0-9]+(?:[_\.][0-9]+))(?:[_\.][A-Za-z0-9]+)?",
        output,
        flags=re.IGNORECASE,
    )
    version = match.group(1).replace("_", ".") if match else None
    return version, output.strip()[-500:]


def _run_license_probe(executable: Path) -> tuple[str, str]:
    """Start the Adams/Car product in batch mode and require a command marker."""
    if not executable.is_file():
        return "not-run", "adams executable is missing"
    try:
        with tempfile.TemporaryDirectory(
            prefix="suspension_multibody_adams_license_"
        ) as cwd:
            working_dir = Path(cwd)
            command_file = working_dir / "license_probe.cmd"
            marker = working_dir / "license_probe_status.txt"
            command_file.write_text(
                "defaults command_file echo_commands=off\n"
                'file text open file="license_probe_status.txt" open=overwrite\n'
                'file text write format="ok"\n'
                "file text close\n"
                "exit confirm=yes\n",
                encoding="ascii",
            )
            completed = subprocess.run(
                [str(executable), "acar", "ru-acar", "b", str(command_file)],
                cwd=working_dir,
                env=_adams_environment(working_dir),
                capture_output=True,
                text=True,
                timeout=120,
                check=False,
            )
            marker_written = (
                marker.is_file()
                and marker.read_text(encoding="utf-8", errors="replace").strip() == "ok"
            )
    except (OSError, subprocess.SubprocessError) as exc:
        return "error", str(exc)
    output = f"{completed.stdout}\n{completed.stderr}".strip()
    if completed.returncode == 0 and marker_written:
        return "passed", output[-500:]
    return f"exit-{completed.returncode}", output[
        -500:
    ] or "product start marker was not written"
