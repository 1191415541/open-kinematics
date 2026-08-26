"""Prescribed vehicle-body K/C time-history replay."""

from __future__ import annotations

import uuid
from datetime import datetime, timezone

import numpy as np

from .. import __version__
from ..core import rotation_vector_to_quaternion
from ..io import canonical_hash
from ..schema import (
    DynamicCaseSpec,
    DynamicManifest,
    DynamicResultBundle,
    DynamicTimeSample,
    FrontAxleModel,
    Pose,
    Provenance,
    Quaternion,
    TimeSignal,
    Vec3,
)
from .time_signals import loads_at_time, time_grid


class VehicleKCTimeDomainSolver:
    """Replay prescribed body motion without dynamics integration."""

    def run(
        self, model: FrontAxleModel, case: DynamicCaseSpec
    ) -> DynamicResultBundle:
        if case.mode != "vehicle_kc_dynamic":
            raise ValueError(
                "VehicleKCTimeDomainSolver requires mode='vehicle_kc_dynamic'"
            )
        if case.vehicle is None:
            raise ValueError("vehicle-level K/C replay requires vehicle data")
        samples = [
            DynamicTimeSample(
                time=time,
                body=case.vehicle.name,
                pose=_pose_from_angles(
                    roll=_motion(case, "body_roll").value_at(time),
                    pitch=_motion(case, "body_pitch").value_at(time),
                    yaw=_motion(case, "body_yaw").value_at(time),
                    heave=_motion(case, "body_heave").value_at(time),
                ),
                loads=loads_at_time(case, time),
                metrics=_vehicle_metrics(case, time),
            )
            for time in time_grid(case)
        ]
        return DynamicResultBundle(
            manifest=DynamicManifest(
                run_id=uuid.uuid4().hex,
                mode=case.mode,
                sample_count=len(samples),
                provenance=Provenance(
                    package_version=__version__,
                    model_hash=canonical_hash(model.model_dump(mode="json")),
                    case_hash=canonical_hash(case.model_dump(mode="json")),
                    created_at=datetime.now(timezone.utc).isoformat(),
                ),
            ),
            samples=tuple(samples),
        )


def _vehicle_metrics(case: DynamicCaseSpec, time: float) -> dict[str, float]:
    assert case.vehicle is not None
    roll = _motion(case, "body_roll").value_at(time)
    pitch = _motion(case, "body_pitch").value_at(time)
    yaw = _motion(case, "body_yaw").value_at(time)
    heave = _motion(case, "body_heave").value_at(time)
    return {
        "degrees_of_freedom": float(case.vehicle.degrees_of_freedom),
        "steering_input": _motion(case, "steering").value_at(time),
        "rack_displacement": _motion(case, "rack").value_at(time),
        "roll_angle": roll,
        "body_roll": roll,
        "body_pitch": pitch,
        "body_yaw": yaw,
        "body_heave": heave,
    }


def _motion(case: DynamicCaseSpec, target: str) -> TimeSignal:
    aliases = {
        "roll": "body_roll",
        "pitch": "body_pitch",
        "yaw": "body_yaw",
        "heave": "body_heave",
        "rack_displacement": "rack",
        "steer": "steering",
    }
    normalized = aliases.get(target, target)
    for prescribed in case.prescribed_motions:
        current = aliases.get(prescribed.target, prescribed.target)
        if current == normalized:
            return prescribed.displacement
    return TimeSignal(constant=0.0)


def _pose_from_angles(roll: float, pitch: float, yaw: float, heave: float) -> Pose:
    quaternion = rotation_vector_to_quaternion(np.array([roll, pitch, yaw]))
    return Pose(
        translation=Vec3(x=0.0, y=0.0, z=heave),
        rotation=Quaternion(
            w=float(quaternion[0]),
            x=float(quaternion[1]),
            y=float(quaternion[2]),
            z=float(quaternion[3]),
        ),
    )
