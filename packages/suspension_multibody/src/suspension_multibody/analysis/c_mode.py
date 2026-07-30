"""C-mode load paths and physical compliant static response."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Literal

import numpy as np

from ..core import CoordinateDrive, RigidBodyState, quaternion_to_rotation_vector
from ..elements import PointWrenchElement
from ..model import FrontAxleAssembly
from ..schema import SixVector
from ..solver import EquilibriumResult, EquilibriumSettings, EquilibriumSolver
from .compliance import secant_compliance, validate_compliance
from .k_mode import KState
from .k_reference import KReferenceCache
from .metrics import compute_k_metrics

LoadAxis = Literal["fx", "fy", "fz", "mx", "my", "mz"]
LeftRightMode = Literal["single", "symmetric", "opposite"]


@dataclass(frozen=True)
class LoadPath:
    """One scalar six-component load path."""

    name: str
    axis: LoadAxis
    maximum: float
    levels: int = 11

    def values(self) -> tuple[float, ...]:
        if self.levels < 2 or self.levels % 2 == 0:
            raise ValueError("load path levels must be an odd number >= 3")
        return tuple(
            float(value)
            for value in np.linspace(-self.maximum, self.maximum, self.levels)
        )

    @classmethod
    def standard(cls) -> tuple[LoadPath, ...]:
        return tuple(
            cls(name=axis, axis=axis, maximum=1.0)
            for axis in ("fx", "fy", "fz", "mx", "my", "mz")
        )


@dataclass(frozen=True)
class CState:
    """One C-mode load response relative to its ideal K reference."""

    case_id: str
    path: str
    level: float
    side_mode: LeftRightMode
    load_left: SixVector
    load_right: SixVector
    deformation_left: SixVector
    deformation_right: SixVector
    tangent_compliance: np.ndarray | None
    secant_compliance_left: np.ndarray
    secant_compliance_right: np.ndarray
    c_minus_k: dict[str, float]
    k_reference_case: str
    metrics: dict[str, float]
    equilibrium: EquilibriumResult | None
    solver_kind: Literal["equilibrium", "linear_proxy"]
    wheel_travel_left: float = 0.0
    wheel_travel_right: float = 0.0
    rack_displacement: float = 0.0


def _vector(axis: LoadAxis, value: float) -> SixVector:
    return SixVector(**{axis: value})


def _six(values: np.ndarray) -> SixVector:
    return SixVector(
        fx=float(values[0]),
        fy=float(values[1]),
        fz=float(values[2]),
        mx=float(values[3]),
        my=float(values[4]),
        mz=float(values[5]),
    )


class CModeSolver:
    """
    Solve compliant static load cases against an ideal-joint K reference.

    Passing ``compliance`` explicitly enables the legacy linear proxy used only
    by the synthetic performance benchmark.  It is never selected implicitly
    and therefore cannot masquerade as a physical C solve.
    """

    def __init__(
        self,
        compliance: np.ndarray | None = None,
        *,
        settings: EquilibriumSettings | None = None,
    ) -> None:
        self.compliance = (
            validate_compliance(compliance) if compliance is not None else None
        )
        self.equilibrium = EquilibriumSolver(settings)

    def solve(
        self,
        assembly: FrontAxleAssembly,
        load: SixVector,
        *,
        side_mode: LeftRightMode = "single",
        k_cache: KReferenceCache | None = None,
        wheel_left: float = 0.0,
        wheel_right: float = 0.0,
        rack: float = 0.0,
        case_id: str = "c-0",
    ) -> CState:
        if side_mode not in ("single", "symmetric", "opposite"):
            raise ValueError("side_mode must be single, symmetric or opposite")
        cache = k_cache or KReferenceCache()
        k_reference = cache.get_or_solve(
            assembly,
            wheel_left=wheel_left,
            wheel_right=wheel_right,
            rack=rack,
        )
        load_left = np.asarray(load.as_tuple(), dtype=float)
        if side_mode == "single":
            load_right = np.zeros(6)
        elif side_mode == "symmetric":
            load_right = load_left.copy()
        else:
            load_right = -load_left
        if self.compliance is not None:
            return self._solve_linear_proxy(
                load_left,
                load_right,
                k_reference,
                side_mode,
                case_id,
                wheel_left,
                wheel_right,
                rack,
            )
        return self._solve_multibody(
            assembly,
            load_left,
            load_right,
            k_reference,
            side_mode,
            case_id,
            wheel_left,
            wheel_right,
            rack,
        )

    def _solve_linear_proxy(
        self,
        load_left: np.ndarray,
        load_right: np.ndarray,
        k_reference: KState,
        side_mode: LeftRightMode,
        case_id: str,
        wheel_left: float,
        wheel_right: float,
        rack: float,
    ) -> CState:
        """Return an explicitly requested nonphysical benchmark response."""
        assert self.compliance is not None
        deformation_left = self.compliance @ load_left
        deformation_right = self.compliance @ load_right
        c_minus_k = {
            key: float(deformation_left[2]) if key.endswith("wheel_center_z") else 0.0
            for key in k_reference.metrics
        }
        return CState(
            case_id,
            "custom",
            float(np.linalg.norm(load_left)),
            side_mode,
            _six(load_left),
            _six(load_right),
            _six(deformation_left),
            _six(deformation_right),
            None,
            secant_compliance(load_left, deformation_left),
            secant_compliance(load_right, deformation_right),
            c_minus_k,
            k_reference.case_id,
            {},
            None,
            "linear_proxy",
            wheel_left,
            wheel_right,
            rack,
        )

    def _solve_multibody(
        self,
        assembly: FrontAxleAssembly,
        load_left: np.ndarray,
        load_right: np.ndarray,
        k_reference: KState,
        side_mode: LeftRightMode,
        case_id: str,
        wheel_left: float,
        wheel_right: float,
        rack: float,
    ) -> CState:
        """Solve one physically represented compliant static state."""
        if assembly.mode != "C":
            raise ValueError("physical C solving requires build_front_axle(model, 'C')")
        bushings = tuple(
            element
            for element in assembly.bushings
            if float(np.linalg.norm(element.stiffness)) > 0.0
        )
        if not bushings:
            raise ValueError("physical C solving requires at least one nonzero bushing")
        reference_equilibrium = k_reference.equilibrium
        if not reference_equilibrium.converged:
            raise RuntimeError("C reference K state did not converge")
        applied_wrenches = tuple(
            PointWrenchElement(
                name=f"wheel_center_wrench_{side}",
                body=f"upright_{side}",
                point_local=assembly.point(f"upright_{side}", "wheel_center"),
                force_global=load[:3],
                moment_global=load[3:],
            )
            for side, load in (("L", load_left), ("R", load_right))
        )
        rack_center = assembly.point("rack", "center")
        rack_target = float(
            reference_equilibrium.state.point_world("rack", rack_center)[1]
        )
        constraints = (
            *assembly.constraints,
            CoordinateDrive(
                "rack",
                rack_center,
                np.array([0.0, 1.0, 0.0]),
                rack_target,
                name="c_rack_neutral_drive",
            ),
        )
        result = self.equilibrium.solve(
            reference_equilibrium.state,
            constraints=constraints,
            elements=(*assembly.elements, *applied_wrenches),
        )
        if not result.converged:
            raise RuntimeError(
                f"C equilibrium did not converge for {case_id}: "
                f"constraint={result.constraint_residual:.3e}, "
                f"force={result.force_residual:.3e}, "
                f"moment={result.moment_residual:.3e}"
            )
        reference_metrics = k_reference.metrics
        metrics = compute_k_metrics(result.state, assembly)
        c_minus_k = {
            key: float(value - reference_metrics[key])
            for key, value in metrics.items()
            if key in reference_metrics
        }
        left = _wheel_response(result.state, reference_equilibrium.state, assembly, "L")
        right = _wheel_response(
            result.state, reference_equilibrium.state, assembly, "R"
        )
        return CState(
            case_id,
            "custom",
            float(np.linalg.norm(load_left)),
            side_mode,
            _six(load_left),
            _six(load_right),
            _six(left),
            _six(right),
            None,
            secant_compliance(load_left, left),
            secant_compliance(load_right, right),
            c_minus_k,
            k_reference.case_id,
            metrics,
            result,
            "equilibrium",
            wheel_left,
            wheel_right,
            rack,
        )

    def run_path(
        self,
        assembly: FrontAxleAssembly,
        path: LoadPath,
        *,
        side_mode: LeftRightMode = "single",
        k_cache: KReferenceCache | None = None,
    ) -> tuple[CState, ...]:
        """Run all levels of one standard path."""
        cache = k_cache or KReferenceCache()
        results: list[CState] = []
        for index, value in enumerate(path.values()):
            result = self.solve(
                assembly,
                _vector(path.axis, value),
                side_mode=side_mode,
                k_cache=cache,
                case_id=f"c-{path.name}-{index:02d}",
            )
            results.append(replace(result, path=path.name, level=value))
        return tuple(results)


def _wheel_response(
    state: RigidBodyState,
    reference_state: RigidBodyState,
    assembly: FrontAxleAssembly,
    side: Literal["L", "R"],
) -> np.ndarray:
    """Return global wheel-center translation and rotation-vector response."""
    body = f"upright_{side}"
    local_center = assembly.point(body, "wheel_center")
    current_center = state.point_world(body, local_center)
    reference_center = reference_state.point_world(body, local_center)
    reference_pose = reference_state.pose(body)
    relative = reference_pose.inverse().compose(state.pose(body))
    rotation = reference_pose.rotation @ quaternion_to_rotation_vector(
        relative.quaternion
    )
    return np.concatenate((current_center - reference_center, rotation))
