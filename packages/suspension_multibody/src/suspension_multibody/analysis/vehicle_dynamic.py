"""Vehicle-level time-domain analysis skeleton."""

from __future__ import annotations

import uuid
from datetime import datetime, timezone

import numpy as np

from .. import __version__
from ..core import SE3, RigidBody, RigidBodyState, rotation_vector_to_quaternion
from ..dynamics import DynamicIntegrator, DynamicRigidBodyState
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
    SixVector,
    TimeSignal,
    Vec3,
)
from .axle_dynamic import _loads_at_time, _time_grid, _wrenches_at_time


class VehicleTimeDomainSolver:
    """Run fixed-body K&C replay or free-body vehicle dynamics skeleton."""

    def run(
        self, model: FrontAxleModel, case: DynamicCaseSpec
    ) -> DynamicResultBundle:
        """Run one vehicle-level dynamic case."""
        if case.vehicle is None:
            raise ValueError("vehicle-level dynamic cases require vehicle data")
        if case.mode == "vehicle_kc_dynamic":
            return self._run_vehicle_kc(model, case)
        if case.mode == "vehicle_dynamic":
            return self._run_vehicle_dynamic(model, case)
        raise ValueError("VehicleTimeDomainSolver requires a vehicle mode")

    def _run_vehicle_kc(
        self, model: FrontAxleModel, case: DynamicCaseSpec
    ) -> DynamicResultBundle:
        samples = [
            DynamicTimeSample(
                time=time,
                body=case.vehicle.name,  # type: ignore[union-attr]
                pose=_pose_from_angles(
                    roll=_motion(case, "body_roll").value_at(time),
                    pitch=_motion(case, "body_pitch").value_at(time),
                    yaw=_motion(case, "body_yaw").value_at(time),
                    heave=_motion(case, "body_heave").value_at(time),
                ),
                loads=_loads_at_time(case, time),
                metrics=_vehicle_metrics(case, time),
            )
            for time in _time_grid(case)
        ]
        return _bundle(model, case, samples)

    def _run_vehicle_dynamic(
        self, model: FrontAxleModel, case: DynamicCaseSpec
    ) -> DynamicResultBundle:
        assert case.vehicle is not None
        body = RigidBody(
            case.vehicle.name,
            pose=SE3.identity(),
            mass=case.vehicle.mass,
            inertia=np.asarray(case.vehicle.inertia, dtype=float),
            center_of_mass=case.vehicle.center_of_mass.as_array(),
        )
        initial = DynamicRigidBodyState(
            RigidBodyState({case.vehicle.name: body}),
            velocities={
                state.body: np.asarray(state.velocity.as_tuple(), dtype=float)
                for state in case.initial_states
                if state.body == case.vehicle.name
            },
        )
        integrator = DynamicIntegrator(case.solver)
        results = integrator.integrate(
            initial,
            external_wrenches=lambda time, _state: _wrenches_at_time(case, time),
        )
        samples = [
            DynamicTimeSample(
                time=result.time,
                body=case.vehicle.name,
                pose=_schema_pose(result.state.pose_state.pose(case.vehicle.name)),
                velocity=_six(result.state.velocity(case.vehicle.name)),
                acceleration=_six(result.state.accelerations[case.vehicle.name]),
                loads=_loads_at_time(case, result.time),
                metrics={
                    **_vehicle_metrics(case, result.time),
                    "constraint_residual": result.constraint_residual,
                    "velocity_residual": result.velocity_residual,
                },
                events=result.events,
            )
            for result in results
        ]
        return _bundle(model, case, samples)


def _bundle(
    model: FrontAxleModel,
    case: DynamicCaseSpec,
    samples: list[DynamicTimeSample],
) -> DynamicResultBundle:
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
    steering = _motion(case, "steering").value_at(time)
    rack = _motion(case, "rack").value_at(time)
    roll = _motion(case, "body_roll").value_at(time)
    pitch = _motion(case, "body_pitch").value_at(time)
    yaw = _motion(case, "body_yaw").value_at(time)
    heave = _motion(case, "body_heave").value_at(time)
    return {
        "degrees_of_freedom": float(case.vehicle.degrees_of_freedom),
        "steering_input": steering,
        "rack_displacement": rack,
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
    for motion in case.prescribed_motions:
        current = aliases.get(motion.target, motion.target)
        if current == normalized:
            return motion.displacement
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


def _schema_pose(value: SE3) -> Pose:
    return Pose(
        translation=Vec3(
            x=float(value.translation[0]),
            y=float(value.translation[1]),
            z=float(value.translation[2]),
        ),
        rotation=Quaternion(
            w=float(value.quaternion[0]),
            x=float(value.quaternion[1]),
            y=float(value.quaternion[2]),
            z=float(value.quaternion[3]),
        ),
    )


def _six(value: np.ndarray) -> SixVector:
    return SixVector(
        fx=float(value[0]),
        fy=float(value[1]),
        fz=float(value[2]),
        mx=float(value[3]),
        my=float(value[4]),
        mz=float(value[5]),
    )
