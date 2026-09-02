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
    """Combined-slip steady-state Fiala tire with aligning moment."""

    cornering_stiffness: float
    longitudinal_stiffness: float
    friction_coefficient: float
    pneumatic_trail: float = 0.0

    def evaluate(self, state: TireKinematics) -> TireForces:
        normal = max(0.0, state.normal_load)
        limit = self.friction_coefficient * normal
        if normal <= 0.0 or limit <= 0.0:
            return TireForces(0.0, 0.0, 0.0)
        longitudinal = _clip(self.longitudinal_stiffness * state.slip_ratio, limit)
        if normal > 0.0:
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
        longitudinal, lateral = _friction_ellipse(longitudinal, lateral, limit)
        return TireForces(
            longitudinal,
            lateral,
            normal,
            mz=-self.pneumatic_trail * lateral,
        )


@dataclass(frozen=True)
class Pac2002TireModel(TireModel):
    """PAC2002-style combined-slip Magic Formula tire interface."""

    cornering_stiffness: float
    longitudinal_stiffness: float
    friction_coefficient: float
    parameter_source: str = "adams_builtin"
    pneumatic_trail: float = 0.0
    coefficients: dict[str, float] | None = None

    def evaluate(self, state: TireKinematics) -> TireForces:
        normal = max(0.0, state.normal_load)
        limit = self.friction_coefficient * normal
        if normal <= 0.0 or limit <= 0.0:
            return TireForces(0.0, 0.0, 0.0)
        coefficients = self.coefficients or {}
        if coefficients:
            longitudinal = _pac2002_force(
                state.slip_ratio,
                normal,
                coefficients,
                prefix="X",
                fallback_stiffness=self.longitudinal_stiffness,
            )
            lateral = -_pac2002_force(
                state.slip_angle,
                normal,
                coefficients,
                prefix="Y",
                fallback_stiffness=self.cornering_stiffness,
            )
        else:
            longitudinal = _magic_formula(
                state.slip_ratio,
                self.longitudinal_stiffness,
                limit,
                shape=1.65,
                curvature=0.97,
            )
            lateral = -_magic_formula(
                state.slip_angle,
                self.cornering_stiffness,
                limit,
                shape=1.30,
                curvature=0.97,
            )
        longitudinal, lateral = _friction_ellipse(longitudinal, lateral, limit)
        return TireForces(
            longitudinal,
            lateral,
            normal,
            mz=-self.pneumatic_trail * lateral,
        )


def tire_model_from_spec(spec: TireModelSpec) -> TireModel:
    """Create a tire model from schema configuration."""
    if spec.kind == "native_brush":
        raise ValueError("native_brush is available only through the native vehicle solver")
    if spec.kind == "vertical_linear":
        return VerticalLinearTireModel(spec.vertical_stiffness)
    if spec.kind == "fiala":
        return FialaTireModel(
            spec.cornering_stiffness,
            spec.longitudinal_stiffness,
            spec.friction_coefficient,
            spec.pneumatic_trail,
        )
    return Pac2002TireModel(
        spec.cornering_stiffness,
        spec.longitudinal_stiffness,
        spec.friction_coefficient,
        spec.parameter_source,
        spec.pneumatic_trail,
        spec.pac2002_coefficients or None,
    )


def _clip(value: float, limit: float) -> float:
    if limit <= 0.0:
        return 0.0
    return min(limit, max(-limit, value))


def _friction_ellipse(longitudinal: float, lateral: float, limit: float) -> tuple[float, float]:
    """Project pure-slip forces onto the available combined-slip ellipse."""
    utilization = math.hypot(longitudinal / limit, lateral / limit)
    if utilization <= 1.0:
        return longitudinal, lateral
    return longitudinal / utilization, lateral / utilization


def _magic_formula(
    slip: float,
    stiffness: float,
    limit: float,
    *,
    shape: float,
    curvature: float,
) -> float:
    """Return a load-scaled Magic Formula core with zero-load protection."""
    if limit <= 0.0 or stiffness <= 0.0:
        return 0.0
    peak_slope = stiffness / max(shape * limit, 1e-12)
    x = peak_slope * slip
    return limit * math.sin(shape * math.atan(x - curvature * (x - math.atan(x))))


def _pac2002_force(
    slip: float,
    normal_load: float,
    coefficients: dict[str, float],
    *,
    prefix: str,
    fallback_stiffness: float,
) -> float:
    """Evaluate pure-slip PAC2002 terms parsed from an Adams .tir file."""
    nominal = max(coefficients.get("FNOMIN", 4850.0), 1e-9)
    dfz = (normal_load - nominal) / nominal
    if prefix == "X":
        c = coefficients.get("PCX1", 1.65)
        mu = coefficients.get("PDX1", 1.0) + coefficients.get("PDX2", 0.0) * dfz
        d = max(0.0, mu * normal_load)
        bcd = normal_load * (
            coefficients.get("PKX1", fallback_stiffness / nominal)
            + coefficients.get("PKX2", 0.0) * dfz
        )
        e = coefficients.get("PEX1", 0.0) + coefficients.get("PEX2", 0.0) * dfz
        sh = coefficients.get("PHX1", 0.0) + coefficients.get("PHX2", 0.0) * dfz
        sv = normal_load * (
            coefficients.get("PVX1", 0.0) + coefficients.get("PVX2", 0.0) * dfz
        )
    else:
        c = coefficients.get("PCY1", 1.3)
        mu = coefficients.get("PDY1", 1.0) + coefficients.get("PDY2", 0.0) * dfz
        d = max(0.0, mu * normal_load)
        bcd = normal_load * (
            coefficients.get("PKY1", -fallback_stiffness / nominal)
            + coefficients.get("PKY2", 0.0) * dfz
        )
        e = coefficients.get("PEY1", 0.0) + coefficients.get("PEY2", 0.0) * dfz
        sh = coefficients.get("PHY1", 0.0) + coefficients.get("PHY2", 0.0) * dfz
        sv = normal_load * (
            coefficients.get("PVY1", 0.0) + coefficients.get("PVY2", 0.0) * dfz
        )
    if d <= 0.0:
        return 0.0
    b = bcd / max(c * d, 1e-12)
    def raw(value: float) -> float:
        argument = b * (value + sh)
        return d * math.sin(
            c * math.atan(argument - e * (argument - math.atan(argument)))
        ) + sv

    # A vehicle initialized in straight rolling has zero tangential contact
    # force.  PAC2002 shift/vertical terms describe a calibrated reference
    # offset; subtract that reference so it cannot inject an artificial drive
    # or lateral impulse before a non-zero slip is applied.
    return raw(slip) - raw(0.0)
