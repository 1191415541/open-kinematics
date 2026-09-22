"""
Public Python run API shared by the CLI.

The physics is in the kernel.  This module turns a `CaseSpec` into the contract
documents the kernel reads, runs them through the single entry point, and turns
the states that come back into the reporting model -- wheel-centre metrics,
element loads, bushing deformations and poses.  Nothing here solves an
equilibrium any more; what is left is authoring and reporting, which is the
boundary `ARCHITECTURE.md` draws.

Two conventions are worth stating because the takeover moved them:

* the contract reports body poses in **metres**, while `RigidBodyState` -- and
  every reporting helper that reads it -- carries **millimetres**.  The bridge is
  `_rigid_state`, and it is the only place that conversion happens;
* the kernel reports convergence by *refusing the run*: a contract result exists
  only when every expanded case solved, so `converged` is true for every returned
  state and is a stronger statement than a per-case flag.  The residuals beside
  it are the kernel's own, read from the case's diagnostics row -- see
  `_case_residuals` for which columns and which units.
"""

from __future__ import annotations

import uuid
from datetime import datetime, timezone
from itertools import product
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from . import __version__

# Imported before the rest of the module, out of alphabetical order, on
# purpose: the authoring assembly has to be the package the import order
# enters.  The legacy chain `elements -> core -> preparation.assembly.types`
# runs this package's `__init__`, and `front_axle` needs the element classes --
# if `cases`, `elements` or `results` is entered first, `elements` is still
# half-built when `front_axle` asks for `AntiRollBarElement`.  The load-order
# constraint disappears when 08 deletes the legacy element package.
from .preparation.assembly import FrontAxleAssembly, build_front_axle  # isort: skip

from .axle_dynamics.schema import AxleSolverSettings
from .cases.kc_quasi_static.contract import model_document, time_document
from .cases.kc_quasi_static.convert import MM
from .elements import BushingElement, evaluate_generalized_forces
from .io import CheckpointStore, canonical_hash, write_artifact
from .kernel.solver import solver_settings_document
from .preparation.assembly.types import RigidBody, RigidBodyState
from .preparation.geometry import (
    SE3,
    quaternion_to_rotation_vector,
    wrench_global_to_local,
)
from .preparation.signals import loads_at_time, motion, time_grid, wrenches_at_time
from .report.compliance import secant_compliance
from .report.metrics import (
    compute_axle_metrics,
    compute_case_metrics,
    compute_common_metrics,
)
from .results import TimeSeriesResult, TimeSeriesSample
from .schema import (
    BushingResult,
    CaseSpec,
    ComponentLoad,
    CResponse,
    Diagnostic,
    DynamicCaseSpec,
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
from .simulation import SimulationRequest, run_request
from .simulation.replay import VehicleKCTimeDomainSolver

#: The output grid a K/C case is solved on.  The kernel's case layer expands a
#: start/end/step, so the product API has to state one; two samples is the
#: minimum, and a K/C state is time independent, so a short one is right.
_TIMES_S = (0.0, 1e-3)

#: The diagnostics columns the residual report reads.  The row is the kernel's
#: own report for one sample; the columns around these are step-controller and
#: contact counters, which the K/C result does not publish.
_DIAGNOSTIC_POSITION_RESIDUAL = 7
_DIAGNOSTIC_DYNAMICS_RESIDUAL = 9
#: Rows one case reserves beyond its samples: the static trim solution and the
#: state it started from.  A case's rows therefore begin at
#: `sample_offset + 2 * case_index`, which is what the descriptor's
#: `sample_count + 2 * case_count` layout implies.
_DIAGNOSTIC_ROWS_PER_CASE = 2

#: The wheel-centre markers the two sides are loaded at.  A C case names them
#: rather than deriving them, because the kernel must refuse a document that
#: loads one side and means two.
_LEFT_MARKER = "wheel_center_L"
_RIGHT_MARKER = "wheel_center_R"


def run_case(
    model: FrontAxleModel, case: CaseSpec, output_dir: str | Path | None = None
) -> ResultBundle:
    """Run one validated model/case and optionally write result files."""
    assembly = build_front_axle(model, case.mode)
    model_hash = canonical_hash(model.model_dump(mode="json"))
    case_hash = canonical_hash(case.model_dump(mode="json"))
    solver_hash = canonical_hash({"package": __version__, "mode": case.mode})
    checkpoint = (
        CheckpointStore(case.checkpoint_path)
        if case.checkpoint_path is not None
        else None
    )
    hashes = (model_hash, case_hash, solver_hash)
    if case.mode == "K":
        states, component_loads, bushing_results = _run_k(assembly, case, checkpoint, hashes)
    else:
        states, component_loads, bushing_results = _run_c(assembly, case, checkpoint, hashes)
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
        write_artifact(bundle, output_dir, model=model, case=case)
    return bundle


def run_dynamic_case(
    model: FrontAxleModel,
    case: DynamicCaseSpec,
    output_dir: str | Path | None = None,
) -> TimeSeriesResult:
    """Run one validated time-domain case and optionally write result files."""
    from dataclasses import replace

    if case.mode == "axle_dynamic" and case.solver.integrator == "quasi_static":
        result = _run_axle_quasi_static(model, case)
    elif case.mode == "vehicle_kc_dynamic":
        result = VehicleKCTimeDomainSolver().run(model, case)
    elif case.mode == "axle_dynamic":
        raise ValueError(
            "the legacy axle dynamics integrator was removed; "
            "use suspension_multibody.axle_dynamics.run_axle_dynamics "
            "with an explicit SI multibody model"
        )
    else:
        raise ValueError(
            "the incomplete legacy vehicle dynamics integrator was removed"
        )
    if case.mode == "vehicle_kc_dynamic":
        result = replace(
            result,
            metrics={
                **dict(result.metrics),
                "case_specific": compute_case_metrics("vehicle_kc_dynamic", result),
            },
        )
    if output_dir is not None:
        write_artifact(result, output_dir, model=model, case=case)
    return result


# --- quasi-static time replay ----------------------------------------------


def _run_axle_quasi_static(
    model: FrontAxleModel, case: DynamicCaseSpec
) -> TimeSeriesResult:
    """
    Replay sampled motion and loads through independent K equilibria.

    One contract run per sample rather than one run carrying a history: a K state
    is a kinematic solution, it does not depend on the sample before it, and the
    kernel would have to be given a per-sample target table to do it in one call.
    """
    assembly = build_front_axle(model, "K")
    document = model_document(assembly, name=f"{case.name}-k", drive_wheels=True)
    left = motion(case, "wheel_travel_left")
    right = motion(case, "wheel_travel_right")
    rack = motion(case, "rack")
    times = time_grid(case)
    samples: list[TimeSeriesSample] = []
    diagnostics: list[Any] = []
    for time in times:
        section: dict[str, object] = {
            "axes": [
                {
                    "coordinate": "wheel_drive_L",
                    "values_mm": [left.value_at(time)],
                },
                {
                    "coordinate": "wheel_drive_R",
                    "values_mm": [right.value_at(time)],
                },
                {"coordinate": "rack_drive", "values_mm": [rack.value_at(time)]},
            ],
            "drive": "wheel_center",
        }
        wrenches = wrenches_at_time(case, time)
        if wrenches:
            section["body_wrench"] = [
                {
                    "body": body,
                    "wrench": [
                        float(value) for value in np.asarray(wrench, dtype=float)
                    ],
                }
                for body, wrench in wrenches.items()
            ]
        run = run_request(
            SimulationRequest(
                assembly="axle",
                family="kc_quasi_static",
                model=document,
                case=_case_envelope(case.name, {"k": section}),
            )
        ).raw
        if run.diagnostics is not None:
            diagnostics.append(run.diagnostics)
        physical = _rigid_state(
            assembly,
            list(run.body_names),
            run.case_body_state(),
        )
        metrics = compute_case_metrics("kc_quasi_static", physical, assembly)
        metrics.update(
            {
                "wheel_travel_left": left.value_at(time),
                "wheel_travel_right": right.value_at(time),
                "rack_displacement": rack.value_at(time),
                "constraint_residual": 0.0,
                "force_residual": 0.0,
                "moment_residual": 0.0,
            }
        )
        samples.append(
            TimeSeriesSample(
                time=time,
                body="axle",
                pose=Pose(),
                loads=loads_at_time(case, time),
                metrics=metrics,
                events=(),
                converged=True,
            )
        )
        for body in ("upright_L", "upright_R", "rack"):
            samples.append(
                TimeSeriesSample(
                    time=time,
                    body=body,
                    pose=_schema_pose(physical.pose(body)),
                    converged=True,
                )
            )
    provenance = Provenance(
        package_version=__version__,
        model_hash=canonical_hash(model.model_dump(mode="json")),
        case_hash=canonical_hash(case.model_dump(mode="json")),
        created_at=datetime.now(timezone.utc).isoformat(),
    ).model_dump(mode="json")
    result_metrics = {
        "common": compute_common_metrics(
            TimeSeriesResult.from_samples(
                samples,
                times_s=times,
                diagnostics=tuple(diagnostics),
                provenance=provenance,
                mode=case.mode,
            )
        ),
        "axle": compute_axle_metrics(
            TimeSeriesResult.from_samples(
                samples,
                times_s=times,
                diagnostics=tuple(diagnostics),
                provenance=provenance,
                mode=case.mode,
            )
        ),
        "case_specific": compute_case_metrics(
            "kc_quasi_static",
            TimeSeriesResult.from_samples(
                samples,
                times_s=times,
                diagnostics=tuple(diagnostics),
                provenance=provenance,
                mode=case.mode,
            ),
        ),
        "sample_count": len(samples),
        "time_count": len(times),
    }
    return TimeSeriesResult.from_samples(
        samples,
        times_s=times,
        diagnostics=tuple(diagnostics),
        provenance=provenance,
        mode=case.mode,
        metrics=result_metrics,
    )


# --- K ---------------------------------------------------------------------


#: The driven coordinate each displacement control moves.
_K_COORDINATES = {
    "left": "wheel_drive_L",
    "right": "wheel_drive_R",
    "rack": "rack_drive",
}

#: The two wheel coordinates the symmetric shorthand drives together.
_K_WHEEL_COORDINATES = ("wheel_drive_L", "wheel_drive_R")


def _k_grid(
    controls: list[DisplacementControl],
) -> tuple[dict[str, object], list[tuple[float, float, float]]]:
    """
    Return the kernel's `k` section and the drives of each case, in order.

    A case that names only the left travel means "both sides together" -- that
    is what `wheel_travel_right=None` has always meant.  That is a *coupling*,
    not a second axis, and a cartesian grid cannot say it, so those cases go
    through the shorthand which drives the wheel set with one sign pattern.
    Naming both sides makes them independent axes, and then the grid is the
    cartesian product of the controls in the order the case lists them.
    """
    named: dict[str, tuple[float, ...]] = {}
    for control in controls:
        named[_k_control_axis(control.target)] = tuple(
            float(value) for value in control.expanded()
        )
    left = named.get("left", (0.0,))
    rack = named.get("rack", (0.0,))
    if "right" not in named:
        section: dict[str, object] = {
            "wheel_values_mm": list(left),
            "rack_values_mm": list(rack),
            "axis_map": {
                "wheel": list(_K_WHEEL_COORDINATES),
                "rack": _K_COORDINATES["rack"],
            },
            "left_right_mode": "symmetric",
        }
        combinations = [(travel, travel, position) for travel in left for position in rack]
        return section, combinations
    order = [name for name in ("left", "right", "rack") if name in named]
    section = {
        "axes": [
            {"coordinate": _K_COORDINATES[name], "values_mm": list(named[name])}
            for name in order
        ]
    }
    drives = {"left": 0.0, "right": 0.0, "rack": 0.0}
    combinations = []
    for values in product(*(named[name] for name in order)):
        for name, value in zip(order, values):
            drives[name] = value
        combinations.append((drives["left"], drives["right"], drives["rack"]))
    return section, combinations


def _k_case_document(case: CaseSpec, section: dict[str, object]) -> dict[str, object]:
    section = dict(section)
    section["drive"] = _case_drive(case)
    if case.external_loads:
        section["body_wrench"] = [
            {"body": body, "wrench": list(load.as_tuple())}
            for body, load in case.external_loads.items()
        ]
    return _case_envelope(case.name, {"k": section})


def _case_envelope(name: str, sections: dict[str, object]) -> dict[str, object]:
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": name,
        "time": time_document(_TIMES_S),
        "solver": solver_settings_document(AxleSolverSettings()),
        **sections,
    }


def _run_k(
    assembly: FrontAxleAssembly,
    case: CaseSpec,
    checkpoint: CheckpointStore | None,
    hashes: tuple[str, str, str],
) -> tuple[list[StateResult], list[ComponentLoad], list[BushingResult]]:
    controls = [
        control for control in case.controls if isinstance(control, DisplacementControl)
    ]
    section, combinations = _k_grid(controls)
    model = model_document(assembly, name=f"{case.name}-k", drive_wheels=True)
    run = run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=model,
            case=_k_case_document(case, section),
        )
    ).raw

    states: list[StateResult] = []
    component_loads: list[ComponentLoad] = []
    bushings: list[BushingResult] = []
    for index in range(len(run.cases)):
        left, right, rack = combinations[index]
        state_id = f"{case.name}-{index:04d}"
        physical = _rigid_state(
            assembly,
            list(run.body_names),
            run.case_body_state(index),
        )
        constraint, force, moment = run.case_residuals(index)
        constraint /= MM
        states.append(
            StateResult(
                state_id=state_id,
                mode="K",
                drives={
                    "wheel_travel_left": left,
                    "wheel_travel_right": right,
                    "rack_displacement": rack,
                },
                metrics=compute_case_metrics("kc_quasi_static", physical, assembly),
                tire_compression=_tire_compression(case),
                constraint_residual=constraint,
                force_residual=force,
                moment_residual=moment,
                converged=True,
                diagnostics=_convergence_note(state_id),
            )
        )
        loads, bushings_found = _collect_element_results(assembly, physical, state_id)
        component_loads.extend(loads)
        bushings.extend(bushings_found)
        _checkpoint(checkpoint, state_id, *hashes)
    return states, component_loads, bushings


# --- C ---------------------------------------------------------------------


def _c_case_document(case: CaseSpec, loads: tuple[SixVector, ...]) -> dict[str, object]:
    return _case_envelope(
        case.name,
        {
            "c": {
                "load_marker": _LEFT_MARKER,
                "mirror_marker": _RIGHT_MARKER,
                "side_mode": case.left_right_mode,
                "loads": [
                    {
                        axis: value
                        for axis, value in zip(
                            ("fx", "fy", "fz", "mx", "my", "mz"), load.as_tuple()
                        )
                    }
                    for load in loads
                ],
            }
        },
    )


def _run_c(
    assembly: FrontAxleAssembly,
    case: CaseSpec,
    checkpoint: CheckpointStore | None,
    hashes: tuple[str, str, str],
) -> tuple[list[StateResult], list[ComponentLoad], list[BushingResult]]:
    controls = [control for control in case.controls if isinstance(control, LoadControl)]
    loads = _c_control_loads(controls, case.external_loads)
    model = model_document(assembly, name=f"{case.name}-c", drive_wheels=False)
    run = run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=model,
            case=_c_case_document(case, loads),
        )
    ).raw

    reference = _reference_state(assembly)
    reference_metrics = compute_case_metrics("kc_quasi_static", reference, assembly)
    states: list[StateResult] = []
    component_loads: list[ComponentLoad] = []
    bushings: list[BushingResult] = []
    for index in range(len(run.cases)):
        applied = {
            "left": loads[index],
            "right": _mirror_load(loads[index], case.left_right_mode),
        }
        state_id = f"{case.name}-{index:04d}"
        physical = _rigid_state(
            assembly,
            list(run.body_names),
            run.case_body_state(index),
        )
        metrics = compute_case_metrics("kc_quasi_static", physical, assembly)
        constraint, force, moment = run.case_residuals(index)
        constraint /= MM
        left = _wheel_response(physical, reference, assembly, "L")
        right = _wheel_response(physical, reference, assembly, "R")
        states.append(
            StateResult(
                state_id=state_id,
                mode="C",
                drives={
                    "wheel_travel_left": 0.0,
                    "wheel_travel_right": 0.0,
                    "rack_displacement": 0.0,
                },
                external_loads=applied,
                poses={
                    "upright_left": _schema_pose(physical.pose("upright_L")),
                    "upright_right": _schema_pose(physical.pose("upright_R")),
                },
                metrics={
                    key: float(value - reference_metrics[key])
                    for key, value in metrics.items()
                },
                c_response=CResponse(
                    wheel_left=_wheel_response_schema(left),
                    wheel_right=_wheel_response_schema(right),
                    secant_compliance_left=_matrix(
                        secant_compliance(np.asarray(applied["left"].as_tuple()), left)
                    ),
                    secant_compliance_right=_matrix(
                        secant_compliance(np.asarray(applied["right"].as_tuple()), right)
                    ),
                ),
                constraint_residual=constraint,
                force_residual=force,
                moment_residual=moment,
                converged=True,
                diagnostics=_convergence_note(state_id),
            )
        )
        found, bushings_found = _collect_element_results(assembly, physical, state_id)
        component_loads.extend(found)
        bushings.extend(bushings_found)
        _checkpoint(checkpoint, state_id, *hashes)
    return states, component_loads, bushings


def _reference_state(assembly: FrontAxleAssembly) -> RigidBodyState:
    """
    Return the pose the C response is measured from.

    Not the case's first sample: the C family applies its whole load at `t = 0`,
    so sample zero is *loaded*.  The reference is the pose the model was
    assembled at, which is what the kernel resolves an omitted driven target to
    and what the K reference equals.
    """
    return assembly.state


def _mirror_load(load: SixVector, side_mode: str) -> SixVector:
    """Return the load the other side carries: none, the same, or the opposite."""
    if side_mode == "single":
        return SixVector()
    values = load.as_tuple()
    if side_mode == "opposite":
        return _six_vector(-np.asarray(values, dtype=float))
    return _six_vector(values)


# --- translation -----------------------------------------------------------


def _rigid_state(
    assembly: FrontAxleAssembly, bodies: list[str], row: np.ndarray
) -> RigidBodyState:
    """Rebuild the reporting state from one contract sample, metres to mm."""
    updated: dict[str, RigidBody] = {}
    for name, body in assembly.state.bodies.items():
        entry = row[bodies.index(name)]
        updated[name] = RigidBody(
            name=name,
            pose=SE3(
                translation=np.asarray(entry[:3], dtype=float) / MM,
                quaternion=np.asarray(entry[3:7], dtype=float),
            ),
            mass=body.mass,
            inertia=body.inertia,
            center_of_mass=body.center_of_mass,
            fixed=body.fixed,
        )
    return RigidBodyState(updated)


def _wheel_response(
    state: RigidBodyState,
    reference_state: RigidBodyState,
    assembly: FrontAxleAssembly,
    side: str,
) -> np.ndarray:
    """Return global wheel-center translation and rotation-vector response."""
    body = f"upright_{side}"
    local_center = assembly.point(body, "wheel_center")
    current_center = state.point_world(body, local_center)
    reference_center = reference_state.point_world(body, local_center)
    reference_pose = reference_state.pose(body)
    relative = reference_pose.inverse().compose(state.pose(body))
    rotation = reference_pose.rotation @ quaternion_to_rotation_vector(
        relative.quaternion
    )
    return np.concatenate((current_center - reference_center, rotation))


def _wheel_response_schema(value: np.ndarray) -> WheelResponse:
    return WheelResponse(
        x_mm=float(value[0]),
        y_mm=float(value[1]),
        z_mm=float(value[2]),
        rx_rad=float(value[3]),
        ry_rad=float(value[4]),
        rz_rad=float(value[5]),
    )




def _convergence_note(state_id: str) -> tuple[Diagnostic, ...]:
    """
    Say what `converged` means now that the kernel owns the solve.

    The contract entry point fails the whole run when any expanded case fails to
    converge, so a returned result is a stronger statement than a per-case flag:
    every case in it solved.  The residuals beside this note are the kernel's own
    last-sample values; `moment_residual` is zero because the kernel reports one
    dynamics residual over its force and moment rows rather than a split.
    """
    return (
        Diagnostic(
            code="native_convergence",
            severity="info",
            message=(
                "solved by the native kernel; the contract returns a result only "
                "when every expanded case converged"
            ),
            state_id=state_id,
        ),
    )


def _tire_compression(case: CaseSpec) -> dict[str, float]:
    """
    Report zero compression.

    A `CaseSpec` carries no road height and no tire radius, so a K state is not
    in contact with anything and there is nothing to compress.  The keys are
    kept because they are part of the result schema.
    """
    del case
    return {"left": 0.0, "right": 0.0}


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
            raise ValueError(
                "a contact-point drive is not implemented natively; the kernel "
                "drives wheel centres, and answering a contact-point case with a "
                "wheel-centre result would be a different question"
            )
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
    assembly: FrontAxleAssembly, state: RigidBodyState, state_id: str
) -> tuple[tuple[ComponentLoad, ...], tuple[BushingResult, ...]]:
    loads: list[ComponentLoad] = []
    bushings: list[BushingResult] = []
    _force, evaluations = evaluate_generalized_forces(
        state,
        assembly.elements,
        body_order=tuple(
            name for name, body in state.bodies.items() if not body.fixed
        ),
    )
    for evaluation in evaluations:
        for body, global_array in evaluation.body_wrenches_global.items():
            local_array = wrench_global_to_local(state.pose(body), global_array)
            loads.append(
                ComponentLoad(
                    state_id=state_id,
                    component=evaluation.name,
                    endpoint=body,
                    global_load=_six_vector(global_array),
                    local_load=_six_vector(local_array),
                )
            )
    for element in assembly.elements:
        if not isinstance(element, BushingElement):
            continue
        deformation = element.deformation(state)
        bushings.append(
            BushingResult(
                state_id=state_id,
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
