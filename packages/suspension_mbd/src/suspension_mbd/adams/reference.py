"""Independent suspension_mbd reference for the bundled Adams/Car demo model."""

from __future__ import annotations

import re
from pathlib import Path

from ..analysis.k_mode import KModeSolver
from ..model import build_front_axle
from ..schema import FrontAxleModel, MassSpec
from .probe import AdamsProfile

_POINT_RE = re.compile(
    r"^\s*'(?P<name>[^']+)'\s+'[^']+'\s+"
    r"(?P<x>[-+0-9.Ee]+)\s+(?P<y>[-+0-9.Ee]+)\s+(?P<z>[-+0-9.Ee]+)"
)


def build_default_reference(profile: AdamsProfile) -> dict[str, dict[str, float]]:
    """Solve the Adams demo hardpoints with suspension_mbd, without Adams results."""
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

    # Adams/Car uses +X forward; suspension_mbd uses +X rearward.
    mapped = {
        name: [-point[0], point[1], point[2]]
        for name, point in hardpoints.items()
        if name in required
    }
    tie_inner = mapped["tierod_inner"]
    mapped["rack_center"] = [tie_inner[0], 0.0, tie_inner[2]]
    model = FrontAxleModel(
        name="adams_car_demo_equivalent",
        hardpoints=mapped,
        mass=MassSpec(sprung_mass=1200.0),
    )
    assembly = build_front_axle(model, "K")
    solver = KModeSolver()
    rebound = solver.solve(assembly, wheel_travel_left=-10.0, wheel_travel_right=-10.0)
    bump = solver.solve(assembly, wheel_travel_left=10.0, wheel_travel_right=10.0)

    def change(field: str) -> float:
        return abs(bump.metrics[field] - rebound.metrics[field])

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
