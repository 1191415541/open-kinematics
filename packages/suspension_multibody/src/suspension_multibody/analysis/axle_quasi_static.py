"""Sampled quasi-static axle analysis."""

from __future__ import annotations

import uuid
from datetime import datetime, timezone

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
    Vec3,
)
from .k_mode import KModeSolver
from .time_signals import loads_at_time, motion, time_grid, wrenches_at_time


class AxleTimeDomainSolver:
    """Replay sampled motion and load inputs through independent K equilibria."""

    def run(
        self, model: FrontAxleModel, case: DynamicCaseSpec
    ) -> DynamicResultBundle:
        if case.mode != "axle_dynamic":
            raise ValueError("AxleTimeDomainSolver requires mode='axle_dynamic'")
        if case.solver.integrator != "quasi_static":
            raise ValueError("AxleTimeDomainSolver only supports quasi_static")
        assembly = build_front_axle(model, "K")
        solver = KModeSolver()
        samples: list[DynamicTimeSample] = []
        initial_state = None
        for index, time in enumerate(time_grid(case)):
            result = solver.solve(
                assembly,
                wheel_travel_left=motion(case, "wheel_travel_left").value_at(time),
                wheel_travel_right=motion(case, "wheel_travel_right").value_at(time),
                rack_displacement=motion(case, "rack").value_at(time),
                external_wrenches_global=wrenches_at_time(case, time),
                case_id=f"{case.name}-{index:04d}",
                initial_state=initial_state,
            )
            initial_state = result.equilibrium.state
            samples.append(
                DynamicTimeSample(
                    time=time,
                    body="axle",
                    pose=Pose(),
                    loads=loads_at_time(case, time),
                    metrics={
                        **result.metrics,
                        "wheel_travel_left": result.wheel_travel_left,
                        "wheel_travel_right": result.wheel_travel_right,
                        "rack_displacement": result.rack_displacement,
                        "constraint_residual": result.equilibrium.constraint_residual,
                        "force_residual": result.equilibrium.force_residual,
                        "moment_residual": result.equilibrium.moment_residual,
                    },
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
