"""
Report and derived metrics.

Every module here consumes something that already exists: a decoded native fact,
a formal result object, or a value a caller measured.  The report does not call
the kernel or a solver, does not prepare an input, and does not recompute a
constitutive law -- it publishes what came back.  ``report`` takes over the
reporting half of ``analysis`` and ``metrics``, both of which 08 deletes.

The boundary is enforced, not merely documented:
``tests/architecture/legacy_surface_gate.py`` fails when a module under this
directory imports native/kernel/solver code, calls a solve entry point, runs
preparation, or evaluates a force law.

``report.geometry`` (the wheel-centre and wheel-angle convention) is imported
from its module rather than re-exported here, because its only callers are two
production modules that already name the path they want.
"""

from .compliance import secant_compliance, tangent_compliance, validate_compliance
from .metrics import (
    axle_metrics,
    case_metric_for,
    compute_axle_metrics,
    compute_case_metrics,
    compute_common_metrics,
    compute_k_metrics,
    compute_vehicle_metrics,
    contact_metrics,
    convergence_metrics,
    not_applicable_metrics,
    peak,
    register_case_metric,
    registered_case_metric_families,
    residual_norm,
    rms,
    time_metrics,
    vehicle_metrics,
    wheel_load_metrics,
    wheel_metrics,
)
from .time_domain_physics import (
    DynamicLoadTransferResult,
    DynamicLoadTransferSample,
    diagnose_dynamic_load_transfer,
)
from .wheel_loads import WheelLoadSummary, summarize_wheel_loads

__all__ = [
    "DynamicLoadTransferResult",
    "DynamicLoadTransferSample",
    "WheelLoadSummary",
    "axle_metrics",
    "case_metric_for",
    "compute_axle_metrics",
    "compute_case_metrics",
    "compute_common_metrics",
    "compute_k_metrics",
    "compute_vehicle_metrics",
    "contact_metrics",
    "convergence_metrics",
    "diagnose_dynamic_load_transfer",
    "not_applicable_metrics",
    "peak",
    "register_case_metric",
    "registered_case_metric_families",
    "residual_norm",
    "rms",
    "secant_compliance",
    "summarize_wheel_loads",
    "tangent_compliance",
    "time_metrics",
    "validate_compliance",
    "vehicle_metrics",
    "wheel_load_metrics",
    "wheel_metrics",
]
