"""Independent suspension_multibody reference for the bundled Adams/Car demo model."""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np

from ..api import validate
from ..authoring.migration import migrate_v1_kc_case
from ..report.kc_evidence import k_records
from ..results.envelope import ResultEnvelope
from ..schema import MassSpec
from ..schema.model import AxleDeclaration
from ..schema.solver import AxleSolverSettings
from ..simulation import run_compiled
from .probe import AdamsProfile

#: The K case time grid and solver settings the native contract expands.  They
#: restate the case layer's defaults so this gate authors its own documents
#: instead of borrowing the legacy workflow runner.
_KC_TIMES_S = tuple(float(value) for value in np.linspace(0.0, 2e-3, 9))
_KC_SETTINGS = AxleSolverSettings(internal_step_s=2.5e-4)

_POINT_RE = re.compile(
    r"^\s*'(?P<name>[^']+)'\s+'[^']+'\s+"
    r"(?P<x>[-+0-9.Ee]+)\s+(?P<y>[-+0-9.Ee]+)\s+(?P<z>[-+0-9.Ee]+)"
)


def build_default_reference(profile: AdamsProfile) -> dict[str, dict[str, float]]:
    """Solve the Adams demo hardpoints with suspension_multibody, without Adams results."""
    if not profile.database_path:
        raise ValueError("Adams database path is unavailable")
    subsystem = (
        Path(profile.database_path)
        / "subsystems.tbl"
        / str(profile.subsystem_id or "TR_Front_Suspension.sub")
    )
    hardpoints = _read_hardpoints(subsystem)
    required = {
        "uca_front",
        "uca_rear",
        "uca_outer",
        "lca_front",
        "lca_rear",
        "lca_outer",
        "tierod_inner",
        "tierod_outer",
        "wheel_center",
    }
    missing = sorted(required - hardpoints.keys())
    if missing:
        raise ValueError(f"Adams subsystem is missing hardpoints: {missing}")

    # Adams/Car uses +X forward; suspension_multibody uses +X rearward.
    mapped = {
        name: [-point[0], point[1], point[2]]
        for name, point in hardpoints.items()
        if name in required
    }
    tie_inner = mapped["tierod_inner"]
    mapped["rack_center"] = [tie_inner[0], 0.0, tie_inner[2]]
    model = AxleDeclaration(
        name="adams_car_demo_equivalent",
        hardpoints=mapped,
        mass=MassSpec(sprung_mass=1200.0),
    )
    states = {
        (float(state["wheel_travel_mm"]), float(state["rack_displacement_mm"])): state
        for state in _k_grid_states(
            model, wheel_values_mm=(-10.0, 10.0), rack_values_mm=(0.0,)
        )
    }
    rebound = states[(-10.0, 0.0)]
    bump = states[(10.0, 0.0)]

    def change(field: str) -> float:
        return abs(float(bump[field]) - float(rebound[field]))

    static_wheel_load = 1200.0 * 9.81 / 4.0
    return {
        "K_geometry": {
            "left_toe_change_deg": change("left_toe_deg"),
            "right_toe_change_deg": change("right_toe_deg"),
            "left_camber_change_deg": change("left_camber_deg"),
        },
        "C_compliance": {
            "converging_lateral_steer_symmetry_deg_per_kn": 0.0,
            "converging_lateral_camber_symmetry_deg_per_kn": 0.0,
        },
        "static_load": {
            "left_wheel_force_n": static_wheel_load,
            "right_wheel_force_n": static_wheel_load,
        },
    }


def _read_hardpoints(path: Path) -> dict[str, tuple[float, float, float]]:
    if not path.is_file():
        raise FileNotFoundError(path)
    points: dict[str, tuple[float, float, float]] = {}
    in_section = False
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.strip() == "[HARDPOINT]":
            in_section = True
            continue
        if in_section and line.startswith("["):
            break
        if in_section and line.startswith("$") and "HARDPOINT" not in line:
            break
        match = _POINT_RE.match(line)
        if match:
            points[match.group("name").strip()] = tuple(
                float(match.group(axis)) for axis in ("x", "y", "z")
            )
    return points


def _k_grid_states(
    model: AxleDeclaration,
    *,
    wheel_values_mm: tuple[float, ...],
    rack_values_mm: tuple[float, ...],
) -> list[dict[str, object]]:
    """Submit ordinary subsystem documents and query declared wheel frames."""
    assembly, case = migrate_v1_kc_case(
        model,
        mode="K",
        name="kc-k",
        wheel_values_mm=wheel_values_mm,
        rack_values_mm=rack_values_mm,
        times_s=_KC_TIMES_S,
        settings=_KC_SETTINGS,
    )
    wheel = next(entry for entry in assembly.entries if entry.functional_role == "wheel")
    frames = {side: wheel.ref+".wheel_center_"+side for side in ("L", "R")}
    result = run_compiled(validate(assembly, case)).result
    if not isinstance(result, ResultEnvelope):
        raise TypeError("ordinary documents must return a ResultEnvelope")
    return k_records(result, frames=frames, wheel_values=wheel_values_mm, rack_values=rack_values_mm)
