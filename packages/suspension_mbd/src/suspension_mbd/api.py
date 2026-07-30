"""Public Python run API shared by the CLI."""

from __future__ import annotations

import uuid
from datetime import datetime, timezone
from itertools import product
from pathlib import Path
from typing import Iterable

import numpy as np

from . import __version__
from .analysis import CModeSolver, CState, KModeSolver, KState
from .core import SE3, wrench_global_to_local
from .elements import BushingElement
from .io import CheckpointStore, canonical_hash, write_bundle
from .model import FrontAxleAssembly, build_front_axle
from .schema import (
    BushingResult,
    CaseSpec,
    ComponentLoad,
    CResponse,
    Diagnostic,
    FrontAxleModel,
    Manifest,
    Pose,
    Provenance,
    Quaternion,
    ResultBundle,
    SixVector,
    StateResult,
    Vec3,
    WheelResponse,
)
from .schema.case import DisplacementControl, LoadControl
from .solver import evaluate_generalized_forces


def run_case(
    model: FrontAxleModel, case: CaseSpec, output_dir: str | Path | None = None
) -> ResultBundle:
    """Run one validated model/case and optionally write result files."""
    assembly = build_front_axle(model, case.mode)
    states: list[StateResult] = []
    component_loads: list[ComponentLoad] = []
    bushing_results: list[BushingResult] = []
    model_hash = canonical_hash(model.model_dump(mode="json"))
    case_hash = canonical_hash(case.model_dump(mode="json"))
    solver_hash = canonical_hash({"package": __version__, "mode": case.mode})
    checkpoint = (
        CheckpointStore(case.checkpoint_path)
        if case.checkpoint_path is not None
        else None
    )
    if case.mode == "K":
        controls = [
            control
            for control in case.controls
            if isinstance(control, DisplacementControl)
        ]
        axes = [_k_control_axis(control.target) for control in controls]
        values = [control.expanded() for control in controls]
        combinations = product(*values) if values else [()]
        engine = KModeSolver()
        for index, combination in enumerate(combinations):
            settings = dict(zip(axes, combination))
            result = engine.solve(
                assembly,
                wheel_travel_left=settings.get("left", 0.0),
                wheel_travel_right=settings.get("right"),
                rack_displacement=settings.get("rack", 0.0),
                drive=_case_drive(case),
                external_wrenches_global={
                    body: np.asarray(load.as_tuple(), dtype=float)
                    for body, load in case.external_loads.items()
                },
                case_id=f"{case.name}-{index:04d}",
            )
            states.append(_state_from_k(result))
            loads, bushings = _collect_element_results(assembly, result)
            component_loads.extend(loads)
            bushing_results.extend(bushings)
            _checkpoint(checkpoint, result.case_id, model_hash, case_hash, solver_hash)
    else:
        controls = [
            control for control in case.controls if isinstance(control, LoadControl)
        ]
        loads = _c_control_loads(controls, case.external_loads)
        engine = CModeSolver()
        for index, load in enumerate(loads):
            result = engine.solve(
                assembly,
                load,
                side_mode=case.left_right_mode,
                case_id=f"{case.name}-{index:04d}",
            )
            states.append(_state_from_c(result))
            loads, bushings = _collect_element_results(assembly, result)
            component_loads.extend(loads)
            bushing_results.extend(bushings)
            _checkpoint(checkpoint, result.case_id, model_hash, case_hash, solver_hash)
    states.sort(key=lambda state: state.state_id)
    provenance = Provenance(
        package_version=__version__,
        model_hash=model_hash,
        case_hash=case_hash,
        created_at=datetime.now(timezone.utc).isoformat(),
    )
    manifest = Manifest(
        run_id=uuid.uuid4().hex,
        mode=case.mode,
        state_count=len(states),
        provenance=provenance,
    )
    bundle = ResultBundle(
        manifest=manifest,
        states=tuple(states),
        component_loads=tuple(component_loads),
        bushings=tuple(bushing_results),
        diagnostics=tuple(),
    )
    if output_dir is not None:
        write_bundle(bundle, output_dir)
    return bundle


def _k_control_axis(target: str) -> str:
    normalized = target.lower().replace("-", "_")
    if normalized in {"wheel_travel_left", "left_wheel_travel", "wheel_left"}:
        return "left"
    if normalized in {"wheel_travel_right", "right_wheel_travel", "wheel_right"}:
        return "right"
    if normalized in {"rack", "rack_displacement", "rack_travel"}:
        return "rack"
    raise ValueError(f"unsupported K displacement target {target!r}")


def _case_drive(case: CaseSpec) -> str:
    for control in case.controls:
        target = control.target.lower()
        if "contact" in target:
            return "contact_point"
    return "wheel_center"


def _c_control_loads(
    controls: list[LoadControl], external_loads: dict[str, SixVector]
) -> tuple[SixVector, ...]:
    if not controls:
        return (next(iter(external_loads.values()), SixVector()),)
    loads: list[SixVector] = []
    for control in controls:
        if control.values is not None:
            loads.extend(control.values)
        elif control.sweep is not None:
            axis = control.target.lower()
            if axis not in {"fx", "fy", "fz", "mx", "my", "mz"}:
                raise ValueError(
                    f"load sweep target must be a six-vector axis: {axis!r}"
                )
            loads.extend(SixVector(**{axis: value}) for value in control.sweep.values())
    return tuple(loads)


def _state_from_k(result: KState) -> StateResult:
    return StateResult(
        state_id=result.case_id,
        mode="K",
        drives={
            "wheel_travel_left": result.wheel_travel_left,
            "wheel_travel_right": result.wheel_travel_right,
            "rack_displacement": result.rack_displacement,
        },
        metrics=result.metrics,
        tire_compression=result.tire_compression,
        constraint_residual=result.equilibrium.constraint_residual,
        force_residual=result.equilibrium.force_residual,
        moment_residual=result.equilibrium.moment_residual,
        converged=result.equilibrium.converged,
    )


def _state_from_c(result: CState) -> StateResult:
    if result.equilibrium is None:
        raise ValueError(
            "linear C proxy results cannot be written as physical C states"
        )
    equilibrium = result.equilibrium
    return StateResult(
        state_id=result.case_id,
        mode="C",
        drives={
            "wheel_travel_left": result.wheel_travel_left,
            "wheel_travel_right": result.wheel_travel_right,
            "rack_displacement": result.rack_displacement,
        },
        external_loads={"left": result.load_left, "right": result.load_right},
        metrics=result.c_minus_k,
        poses={
            "upright_left": _schema_pose(equilibrium.state.pose("upright_L")),
            "upright_right": _schema_pose(equilibrium.state.pose("upright_R")),
        },
        c_response=CResponse(
            wheel_left=_wheel_response(result.deformation_left),
            wheel_right=_wheel_response(result.deformation_right),
            secant_compliance_left=_matrix(result.secant_compliance_left),
            secant_compliance_right=_matrix(result.secant_compliance_right),
        ),
        constraint_residual=equilibrium.constraint_residual,
        force_residual=equilibrium.force_residual,
        moment_residual=equilibrium.moment_residual,
        converged=equilibrium.converged,
        diagnostics=tuple(
            Diagnostic(
                code="c_equilibrium",
                severity="warning",
                message=message,
                state_id=result.case_id,
            )
            for message in equilibrium.diagnostics
        ),
    )


def _checkpoint(
    store: CheckpointStore | None,
    state_id: str,
    model_hash: str,
    case_hash: str,
    solver_hash: str,
) -> None:
    if store is not None:
        store.add(
            state_id,
            model_hash=model_hash,
            case_hash=case_hash,
            solver_hash=solver_hash,
        )


def _collect_element_results(
    assembly: FrontAxleAssembly, result: KState | CState
) -> tuple[tuple[ComponentLoad, ...], tuple[BushingResult, ...]]:
    equilibrium = result.equilibrium
    if equilibrium is None:
        raise ValueError("element loads require an equilibrium-backed result")
    loads: list[ComponentLoad] = []
    bushings: list[BushingResult] = []
    _force, evaluations = evaluate_generalized_forces(
        equilibrium.state,
        assembly.elements,
        body_order=tuple(
            name for name, body in equilibrium.state.bodies.items() if not body.fixed
        ),
    )
    for evaluation in evaluations:
        for body, global_array in evaluation.body_wrenches_global.items():
            local_array = wrench_global_to_local(
                equilibrium.state.pose(body), global_array
            )
            loads.append(
                ComponentLoad(
                    state_id=result.case_id,
                    component=evaluation.name,
                    endpoint=body,
                    global_load=_six_vector(global_array),
                    local_load=_six_vector(local_array),
                )
            )
    for element in assembly.elements:
        if not isinstance(element, BushingElement):
            continue
        deformation = element.deformation(equilibrium.state)
        bushings.append(
            BushingResult(
                state_id=result.case_id,
                bushing=element.name,
                deformation=_six_vector(deformation),
                load=_six_vector(-element.stiffness @ deformation + element.preload),
                strain_energy=0.5
                * float(deformation @ element.stiffness @ deformation),
                stiffness_id=element.name,
                zero_load_pose=_schema_pose(element.local_pose_a),
            )
        )
    return tuple(loads), tuple(bushings)


def _wheel_response(value: SixVector) -> WheelResponse:
    return WheelResponse(
        x_mm=value.fx,
        y_mm=value.fy,
        z_mm=value.fz,
        rx_rad=value.mx,
        ry_rad=value.my,
        rz_rad=value.mz,
    )


def _matrix(values: np.ndarray) -> tuple[tuple[float, ...], ...]:
    return tuple(tuple(float(value) for value in row) for row in values)


def _six_vector(values: Iterable[float]) -> SixVector:
    array = tuple(values)
    return SixVector(
        fx=float(array[0]),
        fy=float(array[1]),
        fz=float(array[2]),
        mx=float(array[3]),
        my=float(array[4]),
        mz=float(array[5]),
    )


def _schema_pose(pose: SE3) -> Pose:
    return Pose(
        translation=Vec3(
            x=float(pose.translation[0]),
            y=float(pose.translation[1]),
            z=float(pose.translation[2]),
        ),
        rotation=Quaternion(
            w=float(pose.quaternion[0]),
            x=float(pose.quaternion[1]),
            y=float(pose.quaternion[2]),
            z=float(pose.quaternion[3]),
        ),
    )
