"""C-mode load paths and linear local compliance response."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

from ..model import FrontAxleAssembly
from ..schema import SixVector
from ..solver import EquilibriumSolver
from .compliance import secant_compliance, validate_compliance
from .k_reference import KReferenceCache

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
    """One C-mode load response."""

    case_id: str
    path: str
    level: float
    side_mode: LeftRightMode
    load_left: SixVector
    load_right: SixVector
    deformation_left: SixVector
    deformation_right: SixVector
    tangent_compliance: np.ndarray
    secant_compliance_left: np.ndarray
    c_minus_k: dict[str, float]
    k_reference_case: str


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
    """Evaluate C-mode loads against a K reference and local compliance."""

    def __init__(self, compliance: np.ndarray | None = None) -> None:
        self.compliance = validate_compliance(
            compliance if compliance is not None else np.eye(6) * 1e-3
        )

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
        deformation_left = self.compliance @ load_left
        deformation_right = self.compliance @ load_right
        c_minus_k = {
            key: float(deformation_left[2]) if key.endswith("wheel_center_z") else 0.0
            for key in k_reference.metrics
        }
        runtime = self._solve_multibody(
            assembly,
            load_left,
            load_right,
            k_reference,
            side_mode,
            case_id,
        )
        if runtime is not None:
            return runtime
        return CState(
            case_id,
            "custom",
            float(np.linalg.norm(load_left)),
            side_mode,
            _six(load_left),
            _six(load_right),
            _six(deformation_left),
            _six(deformation_right),
            self.compliance,
            secant_compliance(load_left, deformation_left),
            c_minus_k,
            k_reference.case_id,
        )

    def _solve_multibody(
        self,
        assembly: FrontAxleAssembly,
        load_left: np.ndarray,
        load_right: np.ndarray,
        k_reference: object,
        side_mode: LeftRightMode,
        case_id: str,
    ) -> CState | None:
        """Use the KKT path when a nonzero schema bushing is installed."""
        bushings = tuple(
            element
            for element in assembly.bushings
            if float(np.linalg.norm(element.stiffness)) > 0.0
        )
        if not bushings:
            return None
        external = {
            "upright_L": np.asarray(load_left, dtype=float),
            "upright_R": np.asarray(load_right, dtype=float),
        }
        result = EquilibriumSolver().solve(
            assembly.state,
            constraints=assembly.constraints,
            elements=assembly.elements,
            external_wrenches_global=external,
        )
        if not result.converged:
            return None
        left = next(
            (item.deformation(result.state) for item in bushings if item.name.endswith("_L")),
            np.zeros(6),
        )
        right = next(
            (item.deformation(result.state) for item in bushings if item.name.endswith("_R")),
            np.zeros(6),
        )
        reference_metrics = getattr(k_reference, "metrics", {})
        metrics = {
            key: float(value - reference_metrics.get(key, 0.0))
            for key, value in reference_metrics.items()
        }
        return CState(
            case_id,
            "custom",
            float(np.linalg.norm(load_left)),
            side_mode,
            _six(load_left),
            _six(load_right),
            _six(left),
            _six(right),
            self.compliance,
            secant_compliance(load_left, left),
            metrics,
            getattr(k_reference, "case_id", "k-reference"),
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
            results.append(
                CState(
                    result.case_id,
                    path.name,
                    value,
                    result.side_mode,
                    result.load_left,
                    result.load_right,
                    result.deformation_left,
                    result.deformation_right,
                    result.tangent_compliance,
                    result.secant_compliance_left,
                    result.c_minus_k,
                    result.k_reference_case,
                )
            )
        return tuple(results)
