"""
The fixed, non-proprietary front-axle geometry the K/C gates share.

This is a *fixture*, not analysis: one reproducible axle the parity gates, the
native probes and the tests all measure against, so a difference between two
gate results is a difference in the solvers and not in the model.  It used to
carry the Python performance benchmarks too; those are gone with the Python
solver, and the surviving performance gate is `scripts/kc_perf_gate.py`, which
measures the native workloads against a budget of their own.
"""

from __future__ import annotations

import numpy as np

from ..schema import FrontAxleModel, MassSpec

#: The 10x10 work-point grid both performance workloads use.
_WHEEL_VALUES_MM = tuple(float(value) for value in np.linspace(-40.0, 40.0, 10))
_RACK_VALUES_MM = tuple(float(value) for value in np.linspace(-10.0, 10.0, 10))


def benchmark_grid() -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Return the fixed 10x10 wheel-travel x rack work-point grid."""
    return _WHEEL_VALUES_MM, _RACK_VALUES_MM

def benchmark_model() -> FrontAxleModel:
    """Return the non-proprietary fixed geometry used by both gates."""
    return FrontAxleModel(
        name="benchmark_front_double_wishbone",
        hardpoints={
            "uca_front": [-100, -500, 400],
            "uca_rear": [100, -500, 400],
            "uca_outer": [0, -700, 450],
            "lca_front": [-120, -500, 150],
            "lca_rear": [120, -500, 150],
            "lca_outer": [0, -700, 150],
            "tierod_inner": [100, -400, 250],
            "tierod_outer": [50, -700, 250],
            "wheel_center": [0, -700, 300],
            "rack_center": [0, 0, 250],
        },
        mass=MassSpec(sprung_mass=1000),
    )
