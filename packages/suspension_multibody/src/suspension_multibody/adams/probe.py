"""
Local Adams/Car installation and field discovery.

Discovery is release-agnostic on purpose: an installation is identified by its
layout (``bin/adams<release>.bat`` plus ``acar/shared_car_database.cdb``), not by
one pinned release number. A caller that needs a specific release asks for it by
name (``adams-car-2025.1.1``) and fails closed when it is absent, instead of
silently validating against a different Adams build.
"""

from __future__ import annotations

import csv
import os
import re
import shutil
import subprocess
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path

DEFAULT_PROFILE = "adams-car"
VERSIONED_PROFILE_PREFIX = "adams-car-"
HOME_ENVIRONMENT_VARIABLE = "SUSPENSION_MULTIBODY_ADAMS_HOME"

# Adams/Car has shipped into several roots over the years. Each root holds one
# directory per release, which is what keeps this discovery version-independent.
INSTALL_ROOTS = (
    Path(r"C:\Program Files\MSC.Software\Adams"),
    Path(r"C:\Program Files (x86)\MSC.Software\Adams"),
    Path(r"C:\MSC.Software\Adams"),
    Path(r"D:\MSC.Software\Adams"),
    Path(r"E:\MSC.Software\Adams"),
    Path(r"F:\MSC.Software\Adams"),
    Path(r"G:\MSC.Software\Adams"),
)

_VERSION_TOKEN = re.compile(r"^[0-9]{4}(?:[_\.][0-9]+)+$")
_UNKNOWN_PROFILE = object()


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


def normalize_version(text: str) -> str | None:
    """Convert an Adams release token such as ``2025_1_1`` to ``2025.1.1``."""
    token = str(text).strip()
    if not _VERSION_TOKEN.match(token):
        return None
    return token.replace("_", ".")


def installation_version(home: str | Path) -> str | None:
    """Return the release recorded by an installation directory name."""
    return normalize_version(Path(home).name)


def adams_executable(home: str | Path) -> Path | None:
    """Return the batch launcher of one installation, whatever its release."""
    bin_directory = Path(home) / "bin"
    version = installation_version(home)
    if version is not None:
        named = bin_directory / f"adams{version.replace('.', '_')}.bat"
        if named.is_file():
            return named
    try:
        launchers = sorted(
            path for path in bin_directory.glob("adams*.bat") if path.is_file()
        )
    except OSError:
        return None
    return launchers[0] if launchers else None


def adams_database(home: str | Path) -> Path | None:
    """Return the shared Adams/Car database of one installation."""
    database = Path(home) / "acar" / "shared_car_database.cdb"
    return database if database.is_dir() else None


def is_installation(home: str | Path) -> bool:
    """Report whether a directory holds a usable Adams/Car installation."""
    return adams_executable(home) is not None and adams_database(home) is not None


def producer_id(product: str, version: str | None) -> str:
    """Name an evidence producer by the release that actually ran the solve."""
    return f"msc.{product}.{version}" if version else f"msc.{product}"


def resolve_adams_home(
    home: str | Path | None = None, *, version: str | None = None
) -> Path | None:
    """
    Return the best matching installation directory, or ``None``.

    An explicit ``home`` and ``SUSPENSION_MULTIBODY_ADAMS_HOME`` always win over
    auto-discovery, so a caller can point at a non-standard layout.
    """
    explicit: list[Path] = []
    if home is not None:
        explicit.append(Path(home))
    configured = os.environ.get(HOME_ENVIRONMENT_VARIABLE)
    if configured:
        explicit.append(Path(configured))

    discovered: list[Path] = [
        *_path_homes(),
        *_registry_homes(),
        *_versioned_homes(),
    ]
    if version is not None:
        discovered = [
            candidate
            for candidate in discovered
            if installation_version(candidate) == version
        ]

    candidates = _unique_paths([*explicit, *discovered])
    for candidate in candidates:
        if is_installation(candidate):
            return candidate
    # A partially installed or fully absent Adams still has to be reported with
    # the directory the caller should look at, so keep the first existing one.
    return next((candidate for candidate in candidates if candidate.is_dir()), None)


def resolve_adams_executable(
    home: str | Path | None = None, *, version: str | None = None
) -> Path | None:
    """Return the launcher of the best matching installation."""
    root = resolve_adams_home(home, version=version)
    return adams_executable(root) if root is not None else None


def resolve_adams_database(
    home: str | Path | None = None, *, version: str | None = None
) -> Path | None:
    """
    Return the shared database of the best matching installation.

    This is deliberately filesystem-only: importers need a path, not a license
    probe, so resolving a database must not start Adams.
    """
    root = resolve_adams_home(home, version=version)
    return adams_database(root) if root is not None else None


def discover_profile(
    name: str = DEFAULT_PROFILE, home: str | Path | None = None
) -> AdamsProfile:
    """Discover paths and report fields without starting the Adams GUI."""
    requested = _requested_version(name)
    if requested is _UNKNOWN_PROFILE:
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

    root = resolve_adams_home(home, version=requested)
    if root is None:
        label = f" {requested}" if requested is not None else ""
        return _unavailable(name, f"Adams/Car{label} installation was not found")

    executable = adams_executable(root)
    database = adams_database(root)
    template = (
        database / "templates.tbl" / "_double_wishbone.tpl"
        if database is not None
        else None
    )
    subsystem = (
        database / "subsystems.tbl" / "TR_Front_Suspension.sub"
        if database is not None
        else None
    )
    dictionary = root / "acar" / "acar_report_dictionary.csv"
    fields = _read_report_fields(dictionary)

    version, version_detail = _run_version_probe(executable)
    license_probe, license_detail = _run_license_probe(executable)
    detail = "; ".join(value for value in (version_detail, license_detail) if value)

    # A pinned request and the release named by the installation directory both
    # have to agree with what the launcher reports; otherwise the evidence would
    # name an Adams build that never ran.
    expected = requested if requested is not None else installation_version(root)
    version_matches = version is not None and (
        expected is None or version == expected
    )
    available = (
        executable is not None
        and template is not None
        and template.is_file()
        and subsystem is not None
        and subsystem.is_file()
        and version_matches
        and license_probe == "passed"
    )
    if available:
        message = f"Adams/Car {version} discovered at {root}"
    else:
        failures: list[str] = []
        if executable is None:
            failures.append("executable")
        if template is None or not template.is_file():
            failures.append("template")
        if subsystem is None or not subsystem.is_file():
            failures.append("subsystem")
        if version is None:
            failures.append("version probe")
        elif not version_matches:
            failures.append(f"version mismatch (expected {expected}, got {version})")
        if license_probe != "passed":
            failures.append(f"license probe ({license_probe})")
        message = (
            f"Adams installation at {root} failed: {', '.join(failures)}; {detail}"
        )
    return AdamsProfile(
        name=name,
        home=str(root),
        executable=str(executable) if executable is not None else None,
        version=version,
        license_file=os.environ.get("MSC_LICENSE_FILE"),
        template_id="_double_wishbone.tpl"
        if template is not None and template.is_file()
        else None,
        subsystem_id="TR_Front_Suspension.sub"
        if subsystem is not None and subsystem.is_file()
        else None,
        database_path=str(database) if database is not None else None,
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


def _requested_version(name: str) -> object:
    """Return the release pinned by a profile name, ``None``, or unknown."""
    if name == DEFAULT_PROFILE:
        return None
    if name.startswith(VERSIONED_PROFILE_PREFIX):
        version = normalize_version(name[len(VERSIONED_PROFILE_PREFIX) :])
        return version if version is not None else _UNKNOWN_PROFILE
    return _UNKNOWN_PROFILE


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


def _version_sort_key(version: str) -> tuple[int, ...]:
    return tuple(int(part) for part in version.split("."))


def _versioned_homes() -> tuple[Path, ...]:
    """Return installed releases found under the known roots, newest first."""
    homes: list[tuple[tuple[int, ...], Path]] = []
    for root in INSTALL_ROOTS:
        try:
            children = sorted(root.iterdir())
        except OSError:
            continue
        for child in children:
            version = normalize_version(child.name)
            if version is None or not child.is_dir():
                continue
            homes.append((_version_sort_key(version), child))
    homes.sort(key=lambda item: item[0], reverse=True)
    return tuple(home for _key, home in homes)


def _path_homes() -> tuple[Path, ...]:
    """Return installations whose launcher is on ``PATH``, newest first."""
    homes: list[Path] = []
    for candidate in _versioned_homes():
        executable = adams_executable(candidate)
        if executable is None:
            continue
        found = shutil.which(executable.name)
        if found:
            homes.append(Path(found).parent.parent)
    return tuple(_unique_paths(homes))


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
                                if "adams" not in display_name.lower():
                                    continue
                                location = str(
                                    winreg.QueryValueEx(entry, "InstallLocation")[0]
                                ).strip()
                                if location:
                                    roots.extend(_installation_paths(location))
                        except OSError:
                            continue
            except OSError:
                continue
    return tuple(_unique_paths(roots))


def _installation_paths(location: str) -> tuple[Path, ...]:
    """Expand one registry location into the releases it contains."""
    path = Path(location)
    if is_installation(path):
        return (path,)
    expanded: list[Path] = []
    try:
        children = sorted(path.iterdir())
    except OSError:
        return (path,)
    for child in children:
        if child.is_dir() and installation_version(child) is not None:
            expanded.append(child)
    return tuple(expanded) if expanded else (path,)


def _run_version_probe(executable: Path | None) -> tuple[str | None, str]:
    if executable is None or not executable.is_file():
        return None, "adams executable is missing"
    try:
        with tempfile.TemporaryDirectory(prefix="suspension_multibody_adams_") as cwd:
            completed = subprocess.run(
                [str(executable), "-v"],
                cwd=cwd,
                capture_output=True,
                text=True,
                # See `adams/adapter.py`: the locale codec cannot read every byte
                # an external tool prints, and this path is non-ASCII.
                encoding="utf-8",
                errors="replace",
                timeout=30,
                check=False,
            )
    except (OSError, subprocess.SubprocessError) as exc:
        return None, str(exc)
    output = f"{completed.stdout}\n{completed.stderr}"
    match = re.search(
        r"Version\s*=\s*([0-9]+(?:[_\.][0-9]+)+)(?:[_\.][A-Za-z0-9]+)?",
        output,
        flags=re.IGNORECASE,
    )
    version = normalize_version(match.group(1)) if match else None
    return version, output.strip()[-500:]


def _run_license_probe(executable: Path | None) -> tuple[str, str]:
    """Start the Adams/Car product in batch mode and require a command marker."""
    if executable is None or not executable.is_file():
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
                encoding="utf-8",
                errors="replace",
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
