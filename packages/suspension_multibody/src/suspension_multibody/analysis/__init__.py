"""
Static vehicle-level load analysis retained in Python (A2).

Only ``vehicle_physics`` is left: the static wheel-load solve and the roll-centre
geometry.  The reporting half of this package moved to ``report``, the K/C signal
helpers to ``preparation/signals.py`` and the replay orchestration to
``simulation/replay.py``; the wheel-load aggregation lives in
``report/wheel_loads.py`` and is imported from there, not re-exported here.
"""

from .vehicle_physics import (
    RollCenterResult,
    StaticWheelLoadResult,
    compute_static_wheel_loads,
    compute_vehicle_roll_centers,
)

__all__ = [
    "RollCenterResult",
    "StaticWheelLoadResult",
    "compute_static_wheel_loads",
    "compute_vehicle_roll_centers",
]
