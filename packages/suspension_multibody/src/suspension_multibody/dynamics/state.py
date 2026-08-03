"""Dynamic rigid-body state."""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping

import numpy as np

from ..core import RigidBodyState


def _copy_twists(values: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    copied: dict[str, np.ndarray] = {}
    for body, value in values.items():
        array = np.asarray(value, dtype=float)
        if array.shape != (6,) or not np.all(np.isfinite(array)):
            raise ValueError("dynamic twists must contain six finite values")
        copied[body] = array.copy()
    return copied


@dataclass(frozen=True)
class DynamicRigidBodyState:
    """Rigid-body poses plus local spatial velocities and accelerations."""

    pose_state: RigidBodyState
    velocities: Mapping[str, np.ndarray] = field(default_factory=dict)
    accelerations: Mapping[str, np.ndarray] = field(default_factory=dict)
    multipliers: np.ndarray = field(default_factory=lambda: np.zeros(0))
    internal_states: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        velocities = {
            name: np.zeros(6)
            for name, body in self.pose_state.bodies.items()
            if not body.fixed
        }
        velocities.update(_copy_twists(self.velocities))
        accelerations = {
            name: np.zeros(6)
            for name, body in self.pose_state.bodies.items()
            if not body.fixed
        }
        accelerations.update(_copy_twists(self.accelerations))
        multipliers = np.asarray(self.multipliers, dtype=float)
        if multipliers.ndim != 1 or not np.all(np.isfinite(multipliers)):
            raise ValueError("multipliers must be a finite vector")
        object.__setattr__(self, "velocities", MappingProxyType(velocities))
        object.__setattr__(self, "accelerations", MappingProxyType(accelerations))
        object.__setattr__(self, "multipliers", multipliers.copy())
        object.__setattr__(
            self, "internal_states", MappingProxyType(dict(self.internal_states))
        )

    @classmethod
    def from_rigid_body_state(
        cls,
        pose_state: RigidBodyState,
        velocities: Mapping[str, np.ndarray] | None = None,
    ) -> DynamicRigidBodyState:
        """Create a dynamic state from a quasi-static pose state."""
        return cls(pose_state, velocities or {})

    @property
    def bodies(self) -> dict[str, object]:
        """Return runtime bodies from the embedded pose state."""
        return self.pose_state.bodies

    def body_order(self) -> tuple[str, ...]:
        """Return movable body order used by vector assemblies."""
        return tuple(name for name, body in self.pose_state.bodies.items() if not body.fixed)

    def velocity(self, body: str) -> np.ndarray:
        """Return a local body-origin spatial velocity."""
        try:
            return self.velocities[body].copy()
        except KeyError as exc:
            raise KeyError(f"unknown dynamic body {body!r}") from exc

    def point_velocity_global(self, body: str, point_local: np.ndarray) -> np.ndarray:
        """Return global velocity of a body-local point."""
        pose = self.pose_state.pose(body)
        twist = self.velocity(body)
        point = np.asarray(point_local, dtype=float)
        if point.shape != (3,):
            raise ValueError("point must contain three values")
        linear = pose.rotation @ twist[:3]
        angular = pose.rotation @ twist[3:]
        return linear + np.cross(angular, pose.rotation @ point)

    def retract(
        self,
        increments: Mapping[str, np.ndarray],
        velocity_updates: Mapping[str, np.ndarray] | None = None,
        acceleration_updates: Mapping[str, np.ndarray] | None = None,
        multipliers: np.ndarray | None = None,
        internal_states: Mapping[str, object] | None = None,
    ) -> DynamicRigidBodyState:
        """Apply pose and optional velocity/state updates."""
        velocities = dict(self.velocities)
        if velocity_updates:
            velocities.update(_copy_twists(velocity_updates))
        accelerations = dict(self.accelerations)
        if acceleration_updates:
            accelerations.update(_copy_twists(acceleration_updates))
        return DynamicRigidBodyState(
            self.pose_state.retract(dict(increments)),
            velocities,
            accelerations,
            self.multipliers if multipliers is None else multipliers,
            self.internal_states if internal_states is None else internal_states,
        )
