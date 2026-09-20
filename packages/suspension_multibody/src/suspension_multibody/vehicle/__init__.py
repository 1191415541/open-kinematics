"""
High-level vehicle simulation services.

The package owns the boundary between a caller's domain objects and the unified
simulation lifecycle: it builds one ``SimulationRequest``, runs it through the
shared runner, and adds the vehicle-specific metrics and error evidence.  It
authors no contract and submits no native call of its own.
"""

from .service import run_vehicle_dynamics

__all__ = ["run_vehicle_dynamics"]
