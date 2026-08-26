"""Time-domain force-balance and load-transfer diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from ..model import VehicleAssembly
from .vehicle_physics import WheelLoadSummary, summarize_wheel_loads

if TYPE_CHECKING:
    from typing import Any

    FullVehicleDynamicRun = Any
    FullVehicleDynamicSample = Any


@dataclass(frozen=True)
class DynamicLoadTransferSample:
    """One time-domain load-transfer and momentum-balance sample."""

    time: float
    wheel_loads: dict[str, float]
    summary: WheelLoadSummary
    center_of_mass: np.ndarray
    external_force: np.ndarray
    external_moment_about_com: np.ndarray
    force_balance_residual: float
    moment_balance_residual: float
    has_interval_balance: bool


@dataclass(frozen=True)
class DynamicLoadTransferResult:
    """Time history of contact loads and whole-vehicle balance residuals."""

    samples: tuple[DynamicLoadTransferSample, ...]
    balanced_sample_count: int
    max_force_balance_residual: float
    max_moment_balance_residual: float

    @property
    def final(self) -> DynamicLoadTransferSample:
        """Return the final diagnostic sample."""
        if not self.samples:
            raise ValueError("dynamic load-transfer result has no samples")
        return self.samples[-1]


def diagnose_dynamic_load_transfer(
    run: FullVehicleDynamicRun,
    *,
    gravity: np.ndarray = np.array([0.0, 0.0, -9810.0]),
    ignore_before: float = 0.0,
) -> DynamicLoadTransferResult:
    """
    Diagnose integrated contact loads against whole-vehicle momentum balance.

    Contact forces are reconstructed in the global frame from the tire basis.
    The residual is evaluated over each sample interval using finite-difference
    linear and angular momentum, so startup transients remain visible instead
    of being mistaken for a static load result.
    """
    gravity_array = np.asarray(gravity, dtype=float)
    if gravity_array.shape != (3,) or not np.all(np.isfinite(gravity_array)):
        raise ValueError("gravity must contain three finite values")
    if not np.isfinite(ignore_before) or ignore_before < 0.0:
        raise ValueError("ignore_before must be finite and non-negative")
    if not run.samples:
        raise ValueError("dynamic run has no samples")

    assembly = run.assembly
    centers = tuple(_center_of_mass(assembly, sample) for sample in run.samples)
    external = tuple(
        _external_wrench(assembly, sample, center, gravity_array)
        for sample, center in zip(run.samples, centers)
    )

    diagnostics: list[DynamicLoadTransferSample] = []
    balanced_count = 0
    max_force = 0.0
    max_moment = 0.0
    for index, (sample, center) in enumerate(zip(run.samples, centers)):
        loads = _contact_normal_loads(sample)
        summary = summarize_wheel_loads(loads)
        force_residual = 0.0
        moment_residual = 0.0
        has_interval = index > 0
        if has_interval:
            previous = run.samples[index - 1]
            dt = float(sample.time - previous.time)
            if not np.isfinite(dt) or dt <= 0.0:
                raise ValueError("dynamic sample times must be strictly increasing")
            reference = 0.5 * (centers[index - 1] + center)
            linear_momentum_previous, angular_momentum_previous = _momentum(
                assembly, previous, reference
            )
            linear_momentum, angular_momentum = _momentum(
                assembly, sample, reference
            )
            force_rate = (linear_momentum - linear_momentum_previous) / dt
            moment_rate = (angular_momentum - angular_momentum_previous) / dt
            force_external = 0.5 * (external[index - 1][0] + external[index][0])
            moment_external = 0.5 * (external[index - 1][1] + external[index][1])
            force_residual = float(np.linalg.norm(force_rate - force_external))
            moment_residual = float(np.linalg.norm(moment_rate - moment_external))
            if sample.time >= ignore_before:
                balanced_count += 1
                max_force = max(max_force, force_residual)
                max_moment = max(max_moment, moment_residual)
        diagnostics.append(
            DynamicLoadTransferSample(
                time=float(sample.time),
                wheel_loads=loads,
                summary=summary,
                center_of_mass=center,
                external_force=external[index][0],
                external_moment_about_com=external[index][1],
                force_balance_residual=force_residual,
                moment_balance_residual=moment_residual,
                has_interval_balance=has_interval,
            )
        )
    if balanced_count == 0:
        raise ValueError("ignore_before excludes every dynamic sample interval")
    return DynamicLoadTransferResult(
        samples=tuple(diagnostics),
        balanced_sample_count=balanced_count,
        max_force_balance_residual=max_force,
        max_moment_balance_residual=max_moment,
    )


def _contact_normal_loads(sample: FullVehicleDynamicSample) -> dict[str, float]:
    loads: dict[str, float] = {}
    for name, contact in sample.contacts.items():
        value = float(contact.forces.fz)
        if not np.isfinite(value) or value < -1e-9:
            raise ValueError(f"contact {name!r} has an invalid normal load")
        loads[name] = max(0.0, value)
    return loads


def _center_of_mass(assembly: VehicleAssembly, sample: FullVehicleDynamicSample) -> np.ndarray:
    weighted = np.zeros(3)
    total_mass = 0.0
    for name, body in assembly.bodies.items():
        if body.fixed or body.mass <= 0.0:
            continue
        position = sample.state.pose_state.point_world(name, body.center_of_mass)
        weighted += body.mass * position
        total_mass += body.mass
    if total_mass <= 0.0:
        raise ValueError("dynamic vehicle has no movable mass")
    return weighted / total_mass


def _momentum(
    assembly: VehicleAssembly,
    sample: FullVehicleDynamicSample,
    reference: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    linear = np.zeros(3)
    angular = np.zeros(3)
    for name, body in assembly.bodies.items():
        if body.fixed or body.mass <= 0.0:
            continue
        pose = sample.state.pose_state.pose(name)
        center = sample.state.pose_state.point_world(name, body.center_of_mass)
        velocity = sample.state.point_velocity_global(name, body.center_of_mass)
        angular_velocity = pose.rotation @ sample.state.velocity(name)[3:]
        inertia_global = pose.rotation @ body.inertia @ pose.rotation.T
        body_linear = body.mass * velocity
        linear += body_linear
        angular += inertia_global @ angular_velocity
        angular += np.cross(center - reference, body_linear)
    return linear, angular


def _external_wrench(
    assembly: VehicleAssembly,
    sample: FullVehicleDynamicSample,
    reference: np.ndarray,
    gravity: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    force = np.zeros(3)
    moment = np.zeros(3)
    for name, body in assembly.bodies.items():
        if body.fixed or body.mass <= 0.0:
            continue
        center = sample.state.pose_state.point_world(name, body.center_of_mass)
        body_force = body.mass * gravity
        force += body_force
        moment += np.cross(center - reference, body_force)

    for wrench in sample.external_wrenches_global.values():
        value = np.asarray(wrench, dtype=float)
        if value.shape != (6,) or not np.all(np.isfinite(value)):
            raise ValueError("external wrench must contain six finite values")
        body_force = value[:3]
        force += body_force
        moment += value[3:] - np.cross(reference, body_force)

    for contact in sample.contacts.values():
        contact_force = (
            contact.forces.fx * contact.longitudinal
            + contact.forces.fy * contact.lateral
            + contact.forces.fz * contact.normal
        )
        contact_moment = (
            contact.forces.mx * contact.longitudinal
            + contact.forces.my * contact.lateral
            + contact.forces.mz * contact.normal
        )
        force += contact_force
        moment += np.cross(contact.contact_point - reference, contact_force)
        moment += contact_moment
    return force, moment
