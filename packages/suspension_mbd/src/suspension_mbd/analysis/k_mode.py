"""Quasi-static K-mode drives and single-state solves."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Literal

import numpy as np

from ..core import CoordinateDrive
from ..elements import BushingElement, VerticalTireElement
from ..model import FrontAxleAssembly
from ..solver import EquilibriumResult, EquilibriumSettings, EquilibriumSolver
from .metrics import compute_k_metrics

DriveKind = Literal["contact_point", "wheel_center"]


@dataclass(frozen=True)
class KState:
    """One solved K-mode state."""

    case_id: str
    wheel_travel_left: float
    wheel_travel_right: float
    rack_displacement: float
    drive: DriveKind
    equilibrium: EquilibriumResult
    metrics: dict[str, float]
    tire_compression: dict[str, float]


class KModeSolver:
    """Solve wheel travel and rack displacement with ideal joints."""

    def __init__(self, settings: EquilibriumSettings | None = None) -> None:
        self.equilibrium = EquilibriumSolver(settings)

    def solve(
        self,
        assembly: FrontAxleAssembly,
        *,
        wheel_travel_left: float = 0.0,
        wheel_travel_right: float | None = None,
        rack_displacement: float = 0.0,
        drive: DriveKind = "wheel_center",
        road_z: float = 0.0,
        tire_radius: float = 0.0,
        road_z_left: float | None = None,
        road_z_right: float | None = None,
        external_wrenches_global: dict[str, np.ndarray] | None = None,
        case_id: str = "k-0",
        initial_state=None,
    ) -> KState:
        """Solve one state using contact-point or wheel-center drive."""
        if drive not in ("contact_point", "wheel_center"):
            raise ValueError("drive must be contact_point or wheel_center")
        right_travel = (
            wheel_travel_left if wheel_travel_right is None else wheel_travel_right
        )
        state = initial_state or assembly.state
        reference_state = assembly.state
        base_left = reference_state.point_world(
            "upright_L", assembly.point("upright_L", "wheel_center")
        )[2]
        base_right = reference_state.point_world(
            "upright_R", assembly.point("upright_R", "wheel_center")
        )[2]
        base_rack = reference_state.point_world(
            "rack", assembly.point("rack", "center")
        )[1]
        constraints = list(assembly.ideal_constraints or assembly.constraints)
        tires = tuple(
            element for element in assembly.elements if isinstance(element, VerticalTireElement)
        )
        contact_with_tires = drive == "contact_point" and bool(tires)
        if drive == "wheel_center" or not contact_with_tires:
            constraints.extend(
                (
                    CoordinateDrive(
                        "upright_L",
                        assembly.point("upright_L", "wheel_center"),
                        np.array([0.0, 0.0, 1.0]),
                        base_left + wheel_travel_left,
                        name="wheel_drive_L",
                    ),
                    CoordinateDrive(
                        "upright_R",
                        assembly.point("upright_R", "wheel_center"),
                        np.array([0.0, 0.0, 1.0]),
                        base_right + right_travel,
                        name="wheel_drive_R",
                    ),
                )
            )
        constraints.append(
            CoordinateDrive(
                "rack",
                assembly.point("rack", "center"),
                np.array([0.0, 1.0, 0.0]),
                base_rack + rack_displacement,
                name="rack_drive",
            )
        )
        elements = tuple(
            replace(
                element,
                road_z=(
                    road_z_left
                    if element.name.endswith("_L") and road_z_left is not None
                    else road_z_right
                    if element.name.endswith("_R") and road_z_right is not None
                    else road_z
                ),
            )
            if isinstance(element, VerticalTireElement) and contact_with_tires
            else element
            for element in assembly.elements
            if not (assembly.mode == "C" and isinstance(element, BushingElement))
            if drive != "wheel_center" or not isinstance(element, VerticalTireElement)
        )
        result = self.equilibrium.solve(
            state,
            constraints=constraints,
            elements=elements,
            external_wrenches_global=external_wrenches_global,
        )
        if contact_with_tires and not result.converged:
            # A tire-only contact path can start outside the active unilateral
            # set when no preload or gravity is supplied.  Recover the state
            # with the road-plane kinematic boundary, preserving the tire
            # element so its compression and event are still reported.
            fallback_constraints = list(constraints)
            fallback_constraints.extend(
                (
                    CoordinateDrive(
                        "upright_L",
                        assembly.point("upright_L", "wheel_center"),
                        np.array([0.0, 0.0, 1.0]),
                        (road_z_left if road_z_left is not None else road_z)
                        + tire_radius
                        + wheel_travel_left,
                        name="contact_recovery_drive_L",
                    ),
                    CoordinateDrive(
                        "upright_R",
                        assembly.point("upright_R", "wheel_center"),
                        np.array([0.0, 0.0, 1.0]),
                        (road_z_right if road_z_right is not None else road_z)
                        + tire_radius
                        + right_travel,
                        name="contact_recovery_drive_R",
                    ),
                )
            )
            result = self.equilibrium.solve(
                state,
                constraints=fallback_constraints,
                elements=elements,
                external_wrenches_global=external_wrenches_global,
            )
        centers = {
            side: result.state.point_world(
                f"upright_{side}", assembly.point(f"upright_{side}", "wheel_center")
            )[2]
            for side in ("L", "R")
        }
        left_road = road_z if road_z_left is None else road_z_left
        right_road = road_z if road_z_right is None else road_z_right
        tire_compression = {
            "left": float(max(0.0, left_road + tire_radius - centers["L"])),
            "right": float(max(0.0, right_road + tire_radius - centers["R"])),
        }
        return KState(
            case_id,
            wheel_travel_left,
            right_travel,
            rack_displacement,
            drive,
            result,
            compute_k_metrics(result.state, assembly),
            tire_compression,
        )
