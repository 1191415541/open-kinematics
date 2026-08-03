"""Axle-level time-domain analysis."""

from __future__ import annotations

import uuid
from datetime import datetime, timezone

import numpy as np

from .. import __version__
from ..core import SE3
from ..io import canonical_hash
from ..model import build_front_axle
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
    WrenchInput,
)
from .k_mode import KModeSolver


class AxleTimeDomainSolver:
    """Run first-pass axle time histories from scalar drives and wrenches."""

    def run(
        self, model: FrontAxleModel, case: DynamicCaseSpec
    ) -> DynamicResultBundle:
        """Run one axle dynamic case."""
        if case.mode != "axle_dynamic":
            raise ValueError("AxleTimeDomainSolver requires mode='axle_dynamic'")
        assembly = build_front_axle(model, "K")
        solver = KModeSolver()
        samples: list[DynamicTimeSample] = []
        initial_state = None
        for index, time in enumerate(_time_grid(case)):
            left = _motion(case, "wheel_travel_left").value_at(time)
            right = _motion(case, "wheel_travel_right").value_at(time)
            rack = _motion(case, "rack").value_at(time)
            result = solver.solve(
                assembly,
                wheel_travel_left=left,
                wheel_travel_right=right,
                rack_displacement=rack,
                external_wrenches_global=_wrenches_at_time(case, time),
                case_id=f"{case.name}-{index:04d}",
                initial_state=initial_state,
            )
            initial_state = result.equilibrium.state
            metrics = {
                **result.metrics,
                "wheel_travel_left": result.wheel_travel_left,
                "wheel_travel_right": result.wheel_travel_right,
                "rack_displacement": result.rack_displacement,
                "constraint_residual": result.equilibrium.constraint_residual,
                "force_residual": result.equilibrium.force_residual,
                "moment_residual": result.equilibrium.moment_residual,
            }
            samples.append(
                DynamicTimeSample(
                    time=time,
                    body="axle",
                    pose=Pose(),
                    loads=_loads_at_time(case, time),
                    metrics=metrics,
                    events=result.equilibrium.active_events,
                    converged=result.equilibrium.converged,
                )
            )
            for body in ("upright_L", "upright_R", "rack"):
                samples.append(
                    DynamicTimeSample(
                        time=time,
                        body=body,
                        pose=_schema_pose(result.equilibrium.state.pose(body)),
                        converged=result.equilibrium.converged,
                    )
                )
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


def _time_grid(case: DynamicCaseSpec) -> tuple[float, ...]:
    start = case.solver.start_time
    end = case.solver.end_time
    step = case.solver.output_step or case.solver.step_size
    count = int(np.floor((end - start) / step + 1e-12))
    values = [start + index * step for index in range(count + 1)]
    if values[-1] < end - 1e-12:
        values.append(end)
    return tuple(float(value) for value in values)


def _motion(case: DynamicCaseSpec, target: str) -> TimeSignal:
    aliases = {
        "left_wheel_travel": "wheel_travel_left",
        "right_wheel_travel": "wheel_travel_right",
        "rack_displacement": "rack",
    }
    normalized = aliases.get(target, target)
    for motion in case.prescribed_motions:
        current = aliases.get(motion.target, motion.target)
        if current == normalized:
            return motion.displacement
    return TimeSignal(constant=0.0)


def _loads_at_time(case: DynamicCaseSpec, time: float) -> dict[str, SixVector]:
    return {item.target: item.wrench.value_at(time) for item in case.wrench_inputs}


def _wrenches_at_time(case: DynamicCaseSpec, time: float) -> dict[str, np.ndarray]:
    totals: dict[str, np.ndarray] = {}
    for item in case.wrench_inputs:
        body = _target_body(item)
        value = item.wrench.value_at(time).as_array()
        force = value[:3]
        moment = value[3:].copy()
        if item.moment_reference == "application_point":
            moment += np.cross(item.application_point.as_array(), force)
        wrench = np.concatenate((force, moment))
        totals[body] = totals.get(body, np.zeros(6)) + wrench
    return totals


def _target_body(item: WrenchInput) -> str:
    target = item.target.lower().replace("-", "_")
    aliases = {
        "left": "upright_L",
        "wheel_left": "upright_L",
        "wheel_travel_left": "upright_L",
        "left_wheel": "upright_L",
        "right": "upright_R",
        "wheel_right": "upright_R",
        "wheel_travel_right": "upright_R",
        "right_wheel": "upright_R",
        "rack": "rack",
    }
    return aliases.get(target, item.target)


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
