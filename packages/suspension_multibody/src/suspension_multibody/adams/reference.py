"""Independent suspension_multibody reference for the bundled Adams/Car demo model."""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np

from ..analysis._geometry import _wheel_geometry
from ..axle_dynamics import AxleSolverSettings
from ..cases.kc_quasi_static.contract import case_document, model_document
from ..cases.kc_quasi_static.convert import MM, NativeKcError, quaternion_to_rotation
from ..model import build_front_axle
from ..schema import FrontAxleModel, MassSpec
from ..simulation import SimulationRequest, run_request
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
    model = FrontAxleModel(
        name="adams_car_demo_equivalent",
        hardpoints=mapped,
        mass=MassSpec(sprung_mass=1200.0),
    )
    assembly = build_front_axle(model, "K")
    states = {
        (float(state["wheel_travel_mm"]), float(state["rack_displacement_mm"])): state
        for state in _k_grid_states(
            assembly, wheel_values_mm=(-10.0, 10.0), rack_values_mm=(0.0,)
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


def _side_fields(assembly, side: str, position_m, quaternion) -> dict[str, float]:
    """Return the wheel-centre and alignment fields a K/C case reports."""
    rotation = quaternion_to_rotation(quaternion)
    local = np.asarray(assembly.point(f"upright_{side}", "wheel_center"), dtype=float)
    geometry = _wheel_geometry(
        np.asarray(position_m, dtype=float) / MM,
        rotation,
        local,
        side=side,
    )
    name = "left" if side == "L" else "right"
    return {
        f"{name}_wheel_center_x_mm": float(geometry.center[0]),
        f"{name}_wheel_center_y_mm": float(geometry.center[1]),
        f"{name}_wheel_center_z_mm": float(geometry.center[2]),
        f"{name}_camber_deg": geometry.camber_deg,
        f"{name}_toe_deg": geometry.toe_deg,
    }


def _k_grid_states(
    assembly,
    *,
    wheel_values_mm: tuple[float, ...],
    rack_values_mm: tuple[float, ...],
) -> list[dict[str, object]]:
    """Solve one K grid by authoring its documents and running the service."""
    model = model_document(assembly, name="native-k", drive_wheels=True)
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="kc-k",
        wheel_values_mm=wheel_values_mm,
        rack_values_mm=rack_values_mm,
        times_s=_KC_TIMES_S,
        settings=_KC_SETTINGS,
        drive_wheels=True,
    )
    run = run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=model,
            case=case,
        )
    ).raw
    left_states = run.body_state("upright_L")
    right_states = run.body_state("upright_R")
    records: list[dict[str, object]] = []
    for index, entry in enumerate(run.cases()):
        wheel = wheel_values_mm[index // len(rack_values_mm)]
        rack = rack_values_mm[index % len(rack_values_mm)]
        case_id = f"k-w{wheel:+.0f}-r{rack:+.0f}"
        # The case layer expands the grid in document order; checking the name
        # it reported turns a silent reordering into a failure.
        if str(entry["name"]) != case_id:
            raise NativeKcError(
                f"the kernel expanded {entry['name']!r} where {case_id!r} was expected"
            )
        last = int(entry["sample_offset"]) + int(entry["sample_count"]) - 1
        record: dict[str, object] = {
            "case_id": case_id,
            "wheel_travel_mm": float(wheel),
            "rack_displacement_mm": float(rack),
        }
        record.update(
            _side_fields(assembly, "L", left_states[last, :3], left_states[last, 3:7])
        )
        record.update(
            _side_fields(assembly, "R", right_states[last, :3], right_states[last, 3:7])
        )
        records.append(record)
    return records
