"""
Prescribed-motion replay orchestration.

``VehicleKCTimeDomainSolver`` replays a *prescribed* body motion: every sample
is the value of the case's own motion signal at that time, so there is no
integrator here and no state is propagated between samples.  The orchestration
belongs to ``simulation`` because it turns a case into samples; the aggregation
of those samples into one immutable result belongs to
``results/timeseries.py`` (``aggregate_replay_samples``).

Moved here from ``analysis/vehicle_kc_time_domain.py``, which 08 deletes.  The
time grid, the sample contents, the provenance and the aggregate metric are
unchanged, and so is the ``mode`` check that refuses anything but
``vehicle_kc_dynamic``.
"""

from __future__ import annotations

import numpy as np

from .. import __version__
from ..io import canonical_hash
from ..preparation.geometry import rotation_vector_to_quaternion
from ..preparation.signals import loads_at_time, time_grid
from ..results import TimeSeriesResult, TimeSeriesSample
from ..results.timeseries import aggregate_replay_samples
from ..schema import (
    DynamicCaseSpec,
    FrontAxleModel,
    Pose,
    Provenance,
    Quaternion,
    TimeSignal,
    Vec3,
)


class VehicleKCTimeDomainSolver:
    """Replay prescribed body motion without dynamics integration."""

    def run(
        self, model: FrontAxleModel, case: DynamicCaseSpec
    ) -> TimeSeriesResult:
        if case.mode != "vehicle_kc_dynamic":
            raise ValueError(
                "VehicleKCTimeDomainSolver requires mode='vehicle_kc_dynamic'"
            )
        if case.vehicle is None:
            raise ValueError("vehicle-level K/C replay requires vehicle data")
        samples = tuple(
            TimeSeriesSample(
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
        )
        provenance = Provenance(
            package_version=__version__,
            model_hash=canonical_hash(model.model_dump(mode="json")),
            case_hash=canonical_hash(case.model_dump(mode="json")),
        ).model_dump(mode="json")
        return aggregate_replay_samples(
            samples,
            mode=case.mode,
            provenance=provenance,
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


__all__ = ["VehicleKCTimeDomainSolver"]
