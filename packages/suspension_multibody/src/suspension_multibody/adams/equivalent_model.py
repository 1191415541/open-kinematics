"""Generate task-owned Adams/Car sources from a frozen equivalence manifest."""

from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, Mapping

from .probe import AdamsProfile

GeneratedMode = Literal["K", "C"]


@dataclass(frozen=True)
class GeneratedAdamsSources:
    """Paths and hashes for one isolated Adams/Car source set."""

    suspension: Path
    steering: Path
    assembly: Path
    hashes: dict[str, str]


def write_equivalent_sources(
    profile: AdamsProfile,
    manifest: Mapping[str, Any],
    runtime: str | Path,
    *,
    mode: GeneratedMode,
) -> GeneratedAdamsSources:
    """Emit an isolated suspension-class assembly without changing Adams data."""
    if mode not in ("K", "C"):
        raise ValueError("mode must be K or C")
    destination = Path(runtime)
    destination.mkdir(parents=True, exist_ok=True)
    database = _database(profile)
    source_suspension = database / "subsystems.tbl" / str(profile.subsystem_id)
    source_steering = database / "subsystems.tbl" / "TR_Steering.sub"
    source_assembly = database / "assemblies.tbl" / "mdi_front_vehicle.asy"
    for source in (source_suspension, source_steering, source_assembly):
        if not source.is_file():
            raise FileNotFoundError(f"required Adams source is unavailable: {source}")

    suspension = destination / "strict_equivalent_front_suspension.sub"
    steering = destination / "strict_equivalent_steering.sub"
    assembly = destination / "strict_suspension.asy"

    suspension_text = source_suspension.read_text(encoding="utf-8", errors="strict")
    for name, values in _template_hardpoints(manifest).items():
        suspension_text = _replace_hardpoint(suspension_text, name, values)
    suspension_text = _set_parameter(
        suspension_text,
        "kinematic_flag",
        "1" if mode == "K" else "0",
    )
    suspension_text = _set_parameter(suspension_text, "camber_angle", "0.0")
    suspension.write_text(suspension_text, encoding="utf-8")

    steering_text = source_steering.read_text(encoding="utf-8", errors="strict")
    steering_text = _set_parameter(
        steering_text,
        "kinematic_flag",
        "1" if mode == "K" else "0",
    )
    steering.write_text(steering_text, encoding="utf-8")

    assembly_text = source_assembly.read_text(encoding="utf-8", errors="strict")
    assembly_text = _replace_exact(
        assembly_text,
        "mdids://acar_shared/subsystems.tbl/MDI_FRONT_SUSPENSION.sub",
        suspension.as_posix(),
        "suspension usage",
    )
    assembly_text = _replace_exact(
        assembly_text,
        "mdids://acar_shared/subsystems.tbl/MDI_FRONT_STEERING.sub",
        steering.as_posix(),
        "steering usage",
    )
    assembly_text = _set_parameter(assembly_text, "compliance_matrix_flag", "0")
    assembly_text = _set_parameter(assembly_text, "compliance_objects_flag", "0")
    assembly.write_text(assembly_text, encoding="utf-8")

    return GeneratedAdamsSources(
        suspension=suspension,
        steering=steering,
        assembly=assembly,
        hashes={
            "suspension_sha256": _sha256(suspension),
            "steering_sha256": _sha256(steering),
            "assembly_sha256": _sha256(assembly),
            "source_suspension_sha256": _sha256(source_suspension),
            "source_steering_sha256": _sha256(source_steering),
            "source_assembly_sha256": _sha256(source_assembly),
        },
    )


def _template_hardpoints(
    manifest: Mapping[str, Any],
) -> dict[str, tuple[float, float, float]]:
    try:
        raw = manifest["physical_input"]["adams_template_hardpoints_mm"]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "equivalence manifest lacks Adams template hardpoints"
        ) from exc
    if not isinstance(raw, Mapping) or not raw:
        raise ValueError("Adams template hardpoints must be a nonempty mapping")
    points: dict[str, tuple[float, float, float]] = {}
    for name, values in raw.items():
        if not isinstance(name, str) or not isinstance(values, (list, tuple)):
            raise ValueError("Adams template hardpoints must use named triples")
        if len(values) != 3:
            raise ValueError(f"Adams hardpoint {name!r} must contain three values")
        point = tuple(float(value) for value in values)
        if not all(value == value and abs(value) != float("inf") for value in point):
            raise ValueError(f"Adams hardpoint {name!r} must be finite")
        points[name] = point
    return points


def _replace_hardpoint(text: str, name: str, values: tuple[float, float, float]) -> str:
    number = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][-+]?\d+)?"
    pattern = re.compile(
        rf"(?m)^(?P<prefix>\s*'{re.escape(name)}\s*'\s+'left/right'\s+)"
        rf"{number}\s+{number}\s+{number}\s*$"
    )
    matches = tuple(pattern.finditer(text))
    if len(matches) != 1:
        raise ValueError(f"unexpected Adams hardpoint layout for {name!r}")
    x, y, z = values
    return pattern.sub(
        lambda match: f"{match.group('prefix')}{x:12.6f} {y:12.6f} {z:12.6f}",
        text,
        count=1,
    )


def _set_parameter(text: str, name: str, value: str) -> str:
    """Set exactly one base parameter, excluding optional variant overrides."""
    base, separator, variants = text.partition("(VARIANTS)")
    pattern = re.compile(
        rf"(?m)^(?P<prefix>\s*'{re.escape(name)}\s*'\s+"
        r"'(?:single|left/right)\s*'\s+'(?:integer|real)'\s+)"
        r"[^\r\n]+$"
    )
    matches = tuple(pattern.finditer(base))
    if len(matches) != 1:
        raise ValueError(f"unexpected Adams parameter layout for {name!r}")
    updated = pattern.sub(
        lambda match: f"{match.group('prefix')}{value}", base, count=1
    )
    return updated + separator + variants


def _replace_exact(text: str, old: str, new: str, label: str) -> str:
    if text.count(old) != 1:
        raise ValueError(f"unexpected Adams {label} layout")
    return text.replace(old, new)


def _database(profile: AdamsProfile) -> Path:
    database = Path(profile.database_path or "")
    if not database.is_dir():
        raise FileNotFoundError("Adams database is unavailable")
    return database


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
