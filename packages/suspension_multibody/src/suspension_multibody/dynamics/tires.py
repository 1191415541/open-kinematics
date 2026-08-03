"""Dynamic tire model interfaces."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..schema import TireModelSpec


@dataclass(frozen=True)
class TireKinematics:
    """Tire-road kinematic state used by force models."""

    normal_load: float
    slip_angle: float = 0.0
    slip_ratio: float = 0.0
    camber: float = 0.0
    vertical_deflection: float = 0.0


@dataclass(frozen=True)
class TireForces:
    """Tire forces in the contact frame."""

    fx: float
    fy: float
    fz: float
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0


class TireModel:
    """Common tire force interface."""

    def evaluate(self, state: TireKinematics) -> TireForces:
        """Return tire forces for one state."""
        raise NotImplementedError


@dataclass(frozen=True)
class VerticalLinearTireModel(TireModel):
    """Compression-only vertical tire."""

    stiffness: float

    def evaluate(self, state: TireKinematics) -> TireForces:
        normal = max(0.0, state.normal_load + self.stiffness * state.vertical_deflection)
        return TireForces(0.0, 0.0, normal)


@dataclass(frozen=True)
class FialaTireModel(TireModel):
    """Steady-state Fiala lateral tire with clipped longitudinal stiffness."""

    cornering_stiffness: float
    longitudinal_stiffness: float
    friction_coefficient: float

    def evaluate(self, state: TireKinematics) -> TireForces:
        normal = max(0.0, state.normal_load)
        limit = self.friction_coefficient * normal
        longitudinal = _clip(self.longitudinal_stiffness * state.slip_ratio, limit)
        if normal <= 0.0:
            lateral = 0.0
        else:
            tangent = math.tan(state.slip_angle)
            critical = math.atan(3.0 * limit / self.cornering_stiffness)
            if abs(state.slip_angle) < critical:
                lateral = (
                    -self.cornering_stiffness * tangent
                    + self.cornering_stiffness**2
                    * abs(tangent)
                    * tangent
                    / (3.0 * limit)
                    - self.cornering_stiffness**3
                    * tangent**3
                    / (27.0 * limit**2)
                )
            else:
                lateral = -limit * math.copysign(1.0, state.slip_angle)
        return TireForces(longitudinal, _clip(lateral, limit), normal)


@dataclass(frozen=True)
class Pac2002TireModel(TireModel):
    """PAC2002-compatible interface using Magic-Formula-style core terms."""

    cornering_stiffness: float
    longitudinal_stiffness: float
    friction_coefficient: float
    parameter_source: str = "adams_builtin"

    def evaluate(self, state: TireKinematics) -> TireForces:
        normal = max(0.0, state.normal_load)
        limit = self.friction_coefficient * normal
        longitudinal_shape = 1.4
        longitudinal = limit * math.sin(
            longitudinal_shape
            * math.atan(
                self.longitudinal_stiffness
                * state.slip_ratio
                / max(longitudinal_shape * limit, 1e-12)
            )
        )
        lateral_shape = 1.3
        lateral = -limit * math.sin(
            lateral_shape
            * math.atan(
                self.cornering_stiffness
                * state.slip_angle
                / max(lateral_shape * limit, 1e-12)
            )
        )
        return TireForces(_clip(longitudinal, limit), _clip(lateral, limit), normal)


def tire_model_from_spec(spec: TireModelSpec) -> TireModel:
    """Create a tire model from schema configuration."""
    if spec.kind == "vertical_linear":
        return VerticalLinearTireModel(spec.vertical_stiffness)
    if spec.kind == "fiala":
        return FialaTireModel(
            spec.cornering_stiffness,
            spec.longitudinal_stiffness,
            spec.friction_coefficient,
        )
    return Pac2002TireModel(
        spec.cornering_stiffness,
        spec.longitudinal_stiffness,
        spec.friction_coefficient,
        spec.parameter_source,
    )


def _clip(value: float, limit: float) -> float:
    if limit <= 0.0:
        return 0.0
    return min(limit, max(-limit, value))
