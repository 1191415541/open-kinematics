"""
Import a real Adams Car suspension subsystem into the SI axle model.

Adams Car stores a suspension as a template (topology) plus a subsystem file
(the numbers).  This module reads the subsystem file and the property files it
references and produces an :class:`AxleDynamicsModel` in strict SI units, so a
correlation run uses the same masses, inertias, hardpoints, bushings, spring and
damper as Adams rather than a hand-transcribed approximation.

Only what the frozen contract needs is imported.  Anything the kernel cannot
represent exactly is reported through :func:`import_blockers` instead of being
silently approximated.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

# Adams Car writes mm / kg / newton / degrees; the kernel is strict SI.
_MM_PER_M = 1000.0
_KG_MM2_PER_KG_M2 = 1.0e6
_N_PER_MM_PER_N_PER_M = 1000.0


@dataclass(frozen=True)
class AdamsPart:
    """One rigid body as Adams Car stores it, already converted to SI."""

    name: str
    mass_kg: float
    centre_m: tuple[float, float, float]
    inertia_kg_m2: tuple[float, float, float]
    sprung_fraction: float


@dataclass(frozen=True)
class AdamsBushing:
    """A six-axis bushing with its diagonal rates in SI."""

    name: str
    translational_stiffness_n_per_m: tuple[float, float, float]
    rotational_stiffness_n_m_per_rad: tuple[float, float, float]
    translational_damping_n_s_per_m: tuple[float, float, float]
    rotational_damping_n_m_s_per_rad: tuple[float, float, float]


@dataclass(frozen=True)
class AdamsSuspension:
    """Everything imported from one Adams Car suspension subsystem."""

    name: str
    template: str
    hardpoints_m: dict[str, tuple[float, float, float]]
    parts: tuple[AdamsPart, ...]
    bushings: tuple[AdamsBushing, ...]
    spring_rate_n_per_m: float
    spring_free_length_m: float
    damper_velocity_m_per_s: tuple[float, ...]
    damper_force_n: tuple[float, ...]
    bumpstop_clearance_m: float
    reboundstop_clearance_m: float
    tire_unloaded_radius_m: float
    tire_stiffness_n_per_m: float
    nonlinear_notes: tuple[str, ...] = field(default=())


def _blocks(text: str) -> list[tuple[str, str]]:
    """Split an Adams ASCII file into its bracketed sections."""
    out: list[tuple[str, str]] = []
    current: str | None = None
    lines: list[str] = []
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            if current is not None:
                out.append((current, "\n".join(lines)))
            current = stripped[1:-1]
            lines = []
        elif current is not None:
            lines.append(line)
    if current is not None:
        out.append((current, "\n".join(lines)))
    return out


def _fields(block: str) -> dict[str, str]:
    return {
        key: value.strip().strip("'").strip()
        for key, value in re.findall(r"^\s*(\w+)\s+=\s+(.+)$", block, re.M)
    }


def _table(block: str) -> list[list[float]]:
    """Read the numeric rows of an Adams `{header}` table."""
    rows: list[list[float]] = []
    for line in block.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith(("$", "{", "!")):
            continue
        try:
            rows.append([float(value) for value in stripped.split()])
        except ValueError:
            continue
    return rows


def _resolve(reference: str, root: Path) -> Path | None:
    """Resolve an Adams `<db>/table/file` or `mdids://db/table/file` path."""
    cleaned = reference.strip().strip("'")
    match = re.match(r"^(?:mdids://|<)([\w.]+)(?:>)?[/\\](.+)$", cleaned)
    if match is None:
        candidate = root / cleaned
        return candidate if candidate.exists() else None
    tail = match.group(2)
    for base in (root, *root.parent.glob("*.cdb")):
        candidate = base / tail
        if candidate.exists():
            return candidate
    return None


def _xml_attributes(text: str, tag: str) -> dict[str, str]:
    match = re.search(rf"<{tag}\b([^>]*)>", text)
    if match is None:
        return {}
    return dict(re.findall(r'(\w+)="([^"]*)"', match.group(1)))


def _spring_properties(path: Path) -> tuple[float, float, list[str]]:
    """Return spring rate (N/m), free length (m), and any nonlinear notes."""
    text = path.read_text(encoding="utf-8", errors="replace")
    attributes = _xml_attributes(text, "SpringProperties")
    method = attributes.get("method", "linear")
    notes: list[str] = []
    if method != "linear":
        notes.append(
            f"spring {path.name} uses method {method!r}, not a linear rate"
        )
    rate = float(attributes.get("rate", "0")) * _N_PER_MM_PER_N_PER_M
    free_length = float(attributes.get("freeLength", "0")) / _MM_PER_M
    return rate, free_length, notes


def _damper_curve(path: Path) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Return the measured force-velocity curve in SI, sorted by velocity."""
    text = path.read_text(encoding="utf-8", errors="replace")
    match = re.search(
        r'<Spline name="spline_data".*?<!\[CDATA\[(.*?)\]\]>', text, re.S
    )
    if match is None:
        return (), ()
    points: list[tuple[float, float]] = []
    for line in match.group(1).splitlines():
        parts = line.split()
        if len(parts) != 2:
            continue
        try:
            velocity_mm_s, force_n = float(parts[0]), float(parts[1])
        except ValueError:
            continue
        points.append((velocity_mm_s / _MM_PER_M, force_n))
    points.sort()
    deduped: list[tuple[float, float]] = []
    for velocity, force in points:
        if deduped and velocity <= deduped[-1][0]:
            continue
        deduped.append((velocity, force))
    return (
        tuple(point[0] for point in deduped),
        tuple(point[1] for point in deduped),
    )


def _component_block(text: str, axis: str) -> str:
    """Return just one axis component, so a search cannot span components."""
    start = re.search(rf'<ConnectorDirectionComponent name="{axis}"', text)
    if start is None:
        return ""
    end = text.find("</ConnectorDirectionComponent>", start.end())
    return text[start.end() : end if end >= 0 else len(text)]


def _connector_rate(text: str, axis: str, kind: str) -> float:
    """Read one diagonal rate of an XML bushing, in Adams units."""
    block = _component_block(text, axis)
    pattern = (
        rf'<Connector{kind} name="\w+"[^>]*type="(\w+)".*?'
        rf'<ConnectorLinearData[^>]*rate="([-\d.eE+]+)"'
    )
    match = re.search(pattern, block, re.S)
    if match is None:
        return 0.0
    return float(match.group(2))


def _connector_is_linear(text: str, axis: str, kind: str) -> bool:
    block = _component_block(text, axis)
    match = re.search(rf'<Connector{kind} name="\w+"[^>]*type="(\w+)"', block)
    return match is None or match.group(1) == "linear"


def _bushing_from_xml(name: str, path: Path) -> tuple[AdamsBushing, list[str]]:
    text = path.read_text(encoding="utf-8", errors="replace")
    notes: list[str] = []
    for axis in ("x", "y", "z", "rx", "ry", "rz"):
        for kind in ("Stiffness", "Damping"):
            if not _connector_is_linear(text, axis, kind):
                notes.append(
                    f"bushing {path.name} axis {axis} {kind.lower()} "
                    "is not a linear rate"
                )
    translational = tuple(
        _connector_rate(text, axis, "Stiffness") * _N_PER_MM_PER_N_PER_M
        for axis in ("x", "y", "z")
    )
    # Adams stores rotational rates in N*mm/deg; SI wants N*m/rad.
    rotational = tuple(
        _connector_rate(text, axis, "Stiffness")
        / _MM_PER_M
        * (180.0 / math.pi)
        for axis in ("rx", "ry", "rz")
    )
    translational_damping = tuple(
        _connector_rate(text, axis, "Damping") * _N_PER_MM_PER_N_PER_M
        for axis in ("x", "y", "z")
    )
    rotational_damping = tuple(
        _connector_rate(text, axis, "Damping")
        / _MM_PER_M
        * (180.0 / math.pi)
        for axis in ("rx", "ry", "rz")
    )
    bushing = AdamsBushing(
        name=name,
        translational_stiffness_n_per_m=translational,  # type: ignore[arg-type]
        rotational_stiffness_n_m_per_rad=rotational,  # type: ignore[arg-type]
        translational_damping_n_s_per_m=translational_damping,  # type: ignore[arg-type]
        rotational_damping_n_m_s_per_rad=rotational_damping,  # type: ignore[arg-type]
    )
    return bushing, notes


def _bushing_from_ascii(name: str, path: Path) -> tuple[AdamsBushing, list[str]]:
    """Read a legacy `.bus` file, which stores curves rather than rates."""
    text = path.read_text(encoding="utf-8", errors="replace")
    sections = dict(_blocks(text))
    damping = _fields(sections.get("DAMPING", ""))
    notes: list[str] = []

    def slope(curve_name: str, scale: float) -> float:
        rows = _table(sections.get(curve_name, ""))
        if len(rows) < 2:
            return 0.0
        # A linear curve has a constant slope; report anything else.
        slopes = [
            (b[1] - a[1]) / (b[0] - a[0])
            for a, b in zip(rows, rows[1:])
            if b[0] != a[0]
        ]
        if not slopes:
            return 0.0
        if max(slopes) - min(slopes) > 1e-9 * max(1.0, abs(max(slopes))):
            notes.append(f"bushing {path.name} {curve_name} is nonlinear")
        return slopes[0] * scale

    translational = tuple(
        slope(f"F{axis}_CURVE", _N_PER_MM_PER_N_PER_M) for axis in ("X", "Y", "Z")
    )
    rotational = tuple(
        slope(f"T{axis}_CURVE", (180.0 / math.pi) / _MM_PER_M)
        for axis in ("X", "Y", "Z")
    )
    translational_damping = tuple(
        float(damping.get(f"F{axis}_DAMPING", "0")) * _N_PER_MM_PER_N_PER_M
        for axis in ("X", "Y", "Z")
    )
    rotational_damping = tuple(
        float(damping.get(f"T{axis}_DAMPING", "0"))
        / _MM_PER_M
        * (180.0 / math.pi)
        for axis in ("X", "Y", "Z")
    )
    bushing = AdamsBushing(
        name=name,
        translational_stiffness_n_per_m=translational,  # type: ignore[arg-type]
        rotational_stiffness_n_m_per_rad=rotational,  # type: ignore[arg-type]
        translational_damping_n_s_per_m=translational_damping,  # type: ignore[arg-type]
        rotational_damping_n_m_s_per_rad=rotational_damping,  # type: ignore[arg-type]
    )
    return bushing, notes


def read_adams_suspension(
    subsystem_path: str | Path,
    *,
    tire_unloaded_radius_m: float,
    tire_stiffness_n_per_m: float,
) -> AdamsSuspension:
    """Read one Adams Car suspension subsystem and convert it to SI."""
    path = Path(subsystem_path)
    root = path.parent.parent
    text = path.read_text(encoding="utf-8", errors="replace")
    sections = _blocks(text)
    by_name: dict[str, list[str]] = {}
    for name, block in sections:
        by_name.setdefault(name, []).append(block)

    units = _fields(by_name.get("UNITS", [""])[0])
    if units.get("LENGTH") != "mm" or units.get("MASS") != "kg":
        raise ValueError(
            f"{path.name} uses unsupported units {units!r}; "
            "the importer converts from mm/kg only"
        )
    header = _fields(by_name.get("SUBSYSTEM_HEADER", [""])[0])

    hardpoints: dict[str, tuple[float, float, float]] = {}
    for row in by_name.get("HARDPOINT", []):
        for line in row.splitlines():
            match = re.match(
                r"\s*'([\w ]+?)\s*'\s+'([\w/ ]+?)\s*'\s+"
                r"([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)",
                line,
            )
            if match is None:
                continue
            name = match.group(1)
            hardpoints[name] = (
                float(match.group(3)) / _MM_PER_M,
                float(match.group(4)) / _MM_PER_M,
                float(match.group(5)) / _MM_PER_M,
            )

    parts: list[AdamsPart] = []
    for block in by_name.get("PART_ASSEMBLY", []):
        data = _fields(block)
        parts.append(
            AdamsPart(
                name=data.get("USAGE", ""),
                mass_kg=float(data.get("MASS", "0")),
                centre_m=(
                    float(data.get("PART_LOC_X", "0")) / _MM_PER_M,
                    float(data.get("PART_LOC_Y", "0")) / _MM_PER_M,
                    float(data.get("PART_LOC_Z", "0")) / _MM_PER_M,
                ),
                inertia_kg_m2=(
                    float(data.get("IXX", "0")) / _KG_MM2_PER_KG_M2,
                    float(data.get("IYY", "0")) / _KG_MM2_PER_KG_M2,
                    float(data.get("IZZ", "0")) / _KG_MM2_PER_KG_M2,
                ),
                sprung_fraction=float(data.get("SPRUNG_PERCENTAGE", "0")) / 100.0,
            )
        )

    notes: list[str] = []
    bushings: list[AdamsBushing] = []
    for block in by_name.get("BUSHING_ASSEMBLY", []):
        data = _fields(block)
        reference = data.get("PROPERTY_FILE_LIST", "")
        resolved = _resolve(reference, root)
        if resolved is None:
            notes.append(f"bushing property file not found: {reference}")
            continue
        name = data.get("USAGE", resolved.stem)
        if resolved.suffix.lower() == ".xml":
            bushing, extra = _bushing_from_xml(name, resolved)
        else:
            bushing, extra = _bushing_from_ascii(name, resolved)
        bushings.append(bushing)
        notes.extend(extra)

    spring_rate = 0.0
    spring_free_length = 0.0
    for block in by_name.get("NSPRING_ASSEMBLY", []):
        resolved = _resolve(_fields(block).get("PROPERTY_FILE_LIST", ""), root)
        if resolved is None:
            notes.append("spring property file not found")
            continue
        spring_rate, spring_free_length, extra = _spring_properties(resolved)
        notes.extend(extra)

    damper_velocity: tuple[float, ...] = ()
    damper_force: tuple[float, ...] = ()
    for block in by_name.get("DAMPER_ASSEMBLY", []):
        resolved = _resolve(_fields(block).get("PROPERTY_FILE_LIST", ""), root)
        if resolved is None:
            notes.append("damper property file not found")
            continue
        damper_velocity, damper_force = _damper_curve(resolved)

    def clearance(section: str) -> float:
        for block in by_name.get(section, []):
            data = _fields(block)
            if data.get("DISTANCE_TYPE") == "clearance":
                return float(data.get("USER_DISTANCE", "0")) / _MM_PER_M
        return 0.0

    return AdamsSuspension(
        name=path.stem,
        template=header.get("TEMPLATE_NAME", ""),
        hardpoints_m=hardpoints,
        parts=tuple(parts),
        bushings=tuple(bushings),
        spring_rate_n_per_m=spring_rate,
        spring_free_length_m=spring_free_length,
        damper_velocity_m_per_s=damper_velocity,
        damper_force_n=damper_force,
        bumpstop_clearance_m=clearance("BUMPSTOP_ASSEMBLY"),
        reboundstop_clearance_m=clearance("REBOUNDSTOP_ASSEMBLY"),
        tire_unloaded_radius_m=tire_unloaded_radius_m,
        tire_stiffness_n_per_m=tire_stiffness_n_per_m,
        nonlinear_notes=tuple(notes),
    )


def import_blockers(suspension: AdamsSuspension) -> tuple[str, ...]:
    """Report anything that cannot be represented exactly."""
    blockers: list[str] = list(suspension.nonlinear_notes)
    if suspension.spring_rate_n_per_m <= 0.0:
        blockers.append("spring rate was not imported")
    if not suspension.damper_velocity_m_per_s:
        blockers.append("damper curve was not imported")
    for part in suspension.parts:
        if part.mass_kg <= 0.0:
            blockers.append(f"part {part.name!r} has non-positive mass")
        if min(part.inertia_kg_m2) <= 0.0:
            blockers.append(f"part {part.name!r} has non-positive inertia")
    return tuple(blockers)


def suspension_summary(suspension: AdamsSuspension) -> dict[str, Any]:
    """Return a compact, self-describing record of what was imported."""
    return {
        "name": suspension.name,
        "template": suspension.template,
        "hardpoint_count": len(suspension.hardpoints_m),
        "part_count": len(suspension.parts),
        "bushing_count": len(suspension.bushings),
        "spring_rate_n_per_m": suspension.spring_rate_n_per_m,
        "spring_free_length_m": suspension.spring_free_length_m,
        "damper_point_count": len(suspension.damper_velocity_m_per_s),
        "unsprung_mass_kg": sum(
            part.mass_kg * (1.0 - part.sprung_fraction)
            for part in suspension.parts
        ),
        "blockers": import_blockers(suspension),
    }
