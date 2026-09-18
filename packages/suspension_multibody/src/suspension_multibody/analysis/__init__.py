"""
C&K reporting and vehicle-level analysis.

What is left here after the solver takeover is *reporting*: the geometry metrics
a K/C state is summarised by, the compliance numbers derived from a load and its
response, and the vehicle-level load and roll-centre algebra.  The solvers that
used to live beside them -- `k_mode`, `c_mode`, `k_reference` and the KKT
equilibrium -- are gone; K and C states come from the native kernel through
`suspension_multibody.kernel`.  So is the independent 14/15-DOF correlation model
and the tire laws it needed: that family of Adams gates was retired rather than
turned into a kernel-versus-kernel comparison.
"""

from .benchmarks import benchmark_grid, benchmark_model
from .compliance import secant_compliance, tangent_compliance, validate_compliance
from .metrics import compute_k_metrics, wheel_metrics
from .time_domain_physics import (
    DynamicLoadTransferResult,
    DynamicLoadTransferSample,
    diagnose_dynamic_load_transfer,
)
from .vehicle_kc_time_domain import VehicleKCTimeDomainSolver
from .vehicle_physics import (
    RollCenterResult,
    StaticWheelLoadResult,
    WheelLoadSummary,
    compute_static_wheel_loads,
    compute_vehicle_roll_centers,
    summarize_wheel_loads,
    wheel_load_metrics,
)

__all__ = [
    "DynamicLoadTransferResult",
    "DynamicLoadTransferSample",
    "RollCenterResult",
    "StaticWheelLoadResult",
    "VehicleKCTimeDomainSolver",
    "WheelLoadSummary",
    "benchmark_grid",
    "benchmark_model",
    "compute_k_metrics",
    "compute_static_wheel_loads",
    "compute_vehicle_roll_centers",
    "diagnose_dynamic_load_transfer",
    "secant_compliance",
    "summarize_wheel_loads",
    "tangent_compliance",
    "validate_compliance",
    "wheel_load_metrics",
    "wheel_metrics",
]
