"""
The time-integration settings every family shares.

These settings used to live in the axle-dynamics schema, which made them look
like a property of that one family.  They are not: the integrator, the step bounds
and the Newton tolerances describe *how a run is advanced*, and the quasi-static
and vehicle families read them too.  Living beside one family's element types is
what made a shared setting look private, and it is why a family that never
mentions an axle still had to import the axle package to say how it integrates.

The fields and their defaults are unchanged -- they are what the frozen numbers
were produced with -- so this is a move rather than a redesign.  The axle schema
re-exports the class so an existing import keeps working while the callers are
brought over.
"""
from __future__ import annotations

from typing import Literal

from pydantic import Field, model_validator

from .common import StrictModel

__all__ = ["AxleSolverSettings"]


class AxleSolverSettings(StrictModel):
    """
    Native time-integration settings.

    ``ggl_generalized_alpha`` remains the native default.  The explicit HHT
    mode is used when a comparison manifest pins the same Adams HHT alpha.
    """

    integrator: Literal["ggl_generalized_alpha", "hht"] = (
        "ggl_generalized_alpha"
    )
    rho_inf: float = Field(default=0.8, gt=0, le=1)
    hht_alpha: float = Field(default=-0.3, ge=-1.0 / 3.0, le=0)
    initialization_mode: Literal[
        "static_equilibrium", "provided_consistent_state"
    ] = "static_equilibrium"
    adaptive_step: bool = True
    internal_step_s: float = Field(default=0.00025, gt=0)
    minimum_step_s: float = Field(default=1e-6, gt=0)
    maximum_step_s: float = Field(default=0.001, gt=0)
    local_relative_tolerance: float = Field(default=1e-5, gt=0)
    local_position_tolerance_m: float = Field(default=1e-7, gt=0)
    local_angle_tolerance_rad: float = Field(default=1e-7, gt=0)
    local_velocity_tolerance_m_per_s: float = Field(default=1e-6, gt=0)
    local_angular_velocity_tolerance_rad_per_s: float = Field(
        default=1e-6, gt=0
    )
    local_brush_tolerance_m: float = Field(default=1e-7, gt=0)
    contact_event_tolerance_s: float = Field(default=1e-6, gt=0)
    max_newton_iterations: int = Field(default=20, ge=1, le=100)
    max_line_search_iterations: int = Field(default=10, ge=1, le=30)
    position_tolerance_m: float = Field(default=1e-8, gt=0)
    velocity_tolerance_m_per_s: float = Field(default=1e-7, gt=0)
    dynamics_tolerance: float = Field(default=1e-8, gt=0)
    increment_tolerance: float = Field(default=1e-8, gt=0)

    @model_validator(mode="after")
    def _step_bounds(self) -> AxleSolverSettings:
        if self.minimum_step_s > self.internal_step_s:
            raise ValueError("minimum_step_s must not exceed internal_step_s")
        if self.internal_step_s > self.maximum_step_s:
            raise ValueError("internal_step_s must not exceed maximum_step_s")
        return self
