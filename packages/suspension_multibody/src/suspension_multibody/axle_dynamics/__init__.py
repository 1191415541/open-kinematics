"""Physical SI axle dynamics implemented by the native C++ kernel."""

from ..kernel.native import NativeKernelUnavailableError, native_build_metadata

#: The shared integration settings live in the neutral schema layer; re-exported
#: here so an import that names this package keeps working.
from ..schema.solver import AxleSolverSettings  # noqa: E402
from .errors import NativeAxleError
from .io import load_axle_dynamics_case
from .result import (
    ANTI_ROLL_OUTPUT_COLUMNS,
    BODY_STATE_COLUMNS,
    BUMP_STOP_OUTPUT_COLUMNS,
    BUSHING_OUTPUT_COLUMNS,
    CONSTRAINT_WRENCH_COLUMNS,
    DAMPER_OUTPUT_COLUMNS,
    DIAGNOSTIC_COLUMNS,
    ENERGY_COLUMNS,
    PERFORMANCE_COLUMNS,
    SPRING_OUTPUT_COLUMNS,
    TIRE_OUTPUT_COLUMNS,
    AxleContactEventRecord,
    AxleDynamicsResult,
    AxleRunDiagnostics,
    AxleRunPerformance,
)
from .schema import (
    AxleAerodynamicDrag,
    AxleAntiRollBar,
    AxleBody,
    AxleBumpStop,
    AxleBushing,
    AxleCoordinateCoupler,
    AxleDamper,
    AxleDrivenCoordinate,
    AxleDynamicsCase,
    AxleHarmonicRoad,
    AxleJoint,
    AxleSpring,
    AxleTire,
)

__all__ = [
    "AxleAerodynamicDrag",
    "AxleBody",
    "AxleBumpStop",
    "AxleBushing",
    "AxleCoordinateCoupler",
    "AxleDamper",
    "AxleDrivenCoordinate",
    "AxleAntiRollBar",
    "AxleDynamicsCase",
    "AxleContactEventRecord",
    "AxleHarmonicRoad",
    "AxleDynamicsResult",
    "AxleJoint",
    "AxleRunDiagnostics",
    "AxleRunPerformance",
    "AxleSolverSettings",
    "AxleSpring",
    "AxleTire",
    "ANTI_ROLL_OUTPUT_COLUMNS",
    "BODY_STATE_COLUMNS",
    "BUMP_STOP_OUTPUT_COLUMNS",
    "BUSHING_OUTPUT_COLUMNS",
    "DAMPER_OUTPUT_COLUMNS",
    "CONSTRAINT_WRENCH_COLUMNS",
    "DIAGNOSTIC_COLUMNS",
    "ENERGY_COLUMNS",
    "PERFORMANCE_COLUMNS",
    "NativeAxleError",
    "NativeKernelUnavailableError",
    "SPRING_OUTPUT_COLUMNS",
    "TIRE_OUTPUT_COLUMNS",
    "load_axle_dynamics_case",
    "native_build_metadata",
]
