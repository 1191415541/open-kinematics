"""Time-domain roll-center consistency diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from ..dynamics.contact import TireContactResult
from ..schema import VehicleModel
from .vehicle_physics import RollCenterResult, compute_vehicle_roll_centers

if TYPE_CHECKING:
    from .full_vehicle_dynamic import FullVehicleDynamicRun, FullVehicleDynamicSample


_AXLES = ("front", "rear")


@dataclass(frozen=True)
class DynamicRollCenterSample:
    """One axle-level effective roll-center estimate at one time sample."""

    time: float
    axle: str
    geometric_height: float
    effective_height: float
    lateral_force: float
    track: float
    left_vertical_load: float
    right_vertical_load: float
    valid: bool
    reason: str | None
    force_source: str

    @property
    def height_difference(self) -> float:
        """Return effective minus geometric height for valid samples."""
        if not self.valid:
            return float("nan")
        return self.effective_height - self.geometric_height


@dataclass(frozen=True)
class DynamicRollCenterResult:
    """Effective roll-center history and the nominal geometric reference."""

    geometric_centers: dict[str, RollCenterResult]
    samples: tuple[DynamicRollCenterSample, ...]
    valid_sample_count: int
    invalid_sample_count: int
    ignored_sample_count: int

    @property
    def valid_samples(self) -> tuple[DynamicRollCenterSample, ...]:
        """Return only samples that passed contact and force gates."""
        return tuple(sample for sample in self.samples if sample.valid)

    def samples_for_axle(self, axle: str) -> tuple[DynamicRollCenterSample, ...]:
        """Return samples for one axle, including invalid samples."""
        if axle not in _AXLES:
            raise ValueError("axle must be front or rear")
        return tuple(sample for sample in self.samples if sample.axle == axle)

    def mean_effective_height(self, axle: str, *, ignore_before: float = 0.0) -> float:
        """Return the mean valid effective height after a time gate."""
        if not np.isfinite(ignore_before) or ignore_before < 0.0:
            raise ValueError("ignore_before must be finite and non-negative")
        values = [
            sample.effective_height
            for sample in self.samples_for_axle(axle)
            if sample.valid and sample.time >= ignore_before
        ]
        if not values:
            raise ValueError("no valid roll-center samples remain after ignore_before")
        return float(np.mean(values))


def diagnose_dynamic_roll_centers(
    run: FullVehicleDynamicRun,
    vehicle: VehicleModel,
    *,
    road_z: float = 0.0,
    lateral_force_epsilon: float = 1.0,
    ignore_before: float = 0.0,
) -> DynamicRollCenterResult:
    """
    Compare nominal geometric and effective dynamic roll-center heights.

    The effective height is inferred per axle as
    ``track * (Fz_right - Fz_left) / (2 * Fy_axle)``.  Contact forces are
    reconstructed in the global frame.  If the tire lateral-force sum is too
    small, a controlled lateral force applied through the two wheel-body
    external-wrench channels is used instead.  Samples that cannot identify a
    height are retained with an explicit reason rather than being discarded.
    """
    if not run.samples:
        raise ValueError("dynamic run has no samples")
    if not np.isfinite(road_z):
        raise ValueError("road_z must be finite")
    if not np.isfinite(lateral_force_epsilon) or lateral_force_epsilon <= 0.0:
        raise ValueError("lateral_force_epsilon must be finite and positive")
    if not np.isfinite(ignore_before) or ignore_before < 0.0:
        raise ValueError("ignore_before must be finite and non-negative")

    geometric = compute_vehicle_roll_centers(vehicle, road_z=road_z)
    diagnostics: list[DynamicRollCenterSample] = []
    for sample in run.samples:
        for axle in _AXLES:
            diagnostics.append(
                _diagnose_sample(
                    sample,
                    axle,
                    geometric[axle],
                    vehicle,
                    lateral_force_epsilon,
                )
            )
    considered = [sample for sample in diagnostics if sample.time >= ignore_before]
    valid_count = sum(sample.valid for sample in considered)
    invalid_count = sum(not sample.valid for sample in considered)
    return DynamicRollCenterResult(
        geometric_centers=geometric,
        samples=tuple(diagnostics),
        valid_sample_count=valid_count,
        invalid_sample_count=invalid_count,
        ignored_sample_count=len(diagnostics) - len(considered),
    )


def _diagnose_sample(
    sample: FullVehicleDynamicSample,
    axle: str,
    geometric: RollCenterResult,
    vehicle: VehicleModel,
    lateral_force_epsilon: float,
) -> DynamicRollCenterSample:
    left_name = f"{axle}_left"
    right_name = f"{axle}_right"
    left = sample.contacts.get(left_name)
    right = sample.contacts.get(right_name)
    geometric_height = float(geometric.center[1])
    if left is None or right is None:
        return _invalid_sample(
            sample.time,
            axle,
            geometric_height,
            "missing_contact",
        )
    if not left.active or not right.active:
        return _invalid_sample(
            sample.time,
            axle,
            geometric_height,
            "contact_inactive",
        )

    left_force = _global_contact_force(left)
    right_force = _global_contact_force(right)
    left_load = float(left_force[2])
    right_load = float(right_force[2])
    track = float(right.contact_point[1] - left.contact_point[1])
    if not np.isfinite(track) or track <= 1e-9:
        return _invalid_sample(
            sample.time,
            axle,
            geometric_height,
            "invalid_track",
            left_load,
            right_load,
            track,
        )

    contact_lateral_force = float(left_force[1] + right_force[1])
    if abs(contact_lateral_force) >= lateral_force_epsilon:
        lateral_force = contact_lateral_force
        source = "contact"
    else:
        lateral_force = _wheel_external_lateral_force(sample, vehicle, axle)
        source = "wheel_external"
    if not np.isfinite(lateral_force) or abs(lateral_force) < lateral_force_epsilon:
        return _invalid_sample(
            sample.time,
            axle,
            geometric_height,
            "lateral_force_too_small",
            left_load,
            right_load,
            track,
            source,
            lateral_force,
        )
    effective_height = track * (right_load - left_load) / (2.0 * lateral_force)
    if not np.isfinite(effective_height):
        return _invalid_sample(
            sample.time,
            axle,
            geometric_height,
            "effective_height_not_finite",
            left_load,
            right_load,
            track,
            source,
            lateral_force,
        )
    return DynamicRollCenterSample(
        time=float(sample.time),
        axle=axle,
        geometric_height=geometric_height,
        effective_height=float(effective_height),
        lateral_force=lateral_force,
        track=track,
        left_vertical_load=left_load,
        right_vertical_load=right_load,
        valid=True,
        reason=None,
        force_source=source,
    )


def _invalid_sample(
    time: float,
    axle: str,
    geometric_height: float,
    reason: str,
    left_load: float = float("nan"),
    right_load: float = float("nan"),
    track: float = float("nan"),
    force_source: str = "none",
    lateral_force: float = 0.0,
) -> DynamicRollCenterSample:
    return DynamicRollCenterSample(
        time=float(time),
        axle=axle,
        geometric_height=geometric_height,
        effective_height=float("nan"),
        lateral_force=float(lateral_force),
        track=float(track),
        left_vertical_load=float(left_load),
        right_vertical_load=float(right_load),
        valid=False,
        reason=reason,
        force_source=force_source,
    )


def _global_contact_force(contact: TireContactResult) -> np.ndarray:
    """Reconstruct a contact force from its road-frame components."""
    return (
        float(contact.forces.fx) * np.asarray(contact.longitudinal, dtype=float)
        + float(contact.forces.fy) * np.asarray(contact.lateral, dtype=float)
        + float(contact.forces.fz) * np.asarray(contact.normal, dtype=float)
    )


def _wheel_external_lateral_force(
    sample: FullVehicleDynamicSample,
    vehicle: VehicleModel,
    axle: str,
) -> float:
    names = {f"{axle}_left", f"{axle}_right"}
    body_names = {
        wheel.body for wheel in vehicle.wheels if wheel.name in names
    }
    total = 0.0
    for body in body_names:
        wrench = sample.external_wrenches_global.get(body)
        if wrench is None:
            continue
        value = np.asarray(wrench, dtype=float)
        if value.shape != (6,) or not np.all(np.isfinite(value)):
            raise ValueError("external wrench must contain six finite values")
        total += float(value[1])
    return total
