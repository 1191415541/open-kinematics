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

import os
import uuid
from datetime import datetime, timezone
from itertools import product
from pathlib import Path
from typing import Any, Iterable, Literal, Mapping

import numpy as np
from suspension_contracts import pack_container

from . import __version__
from .axle_dynamics.schema import AxleSolverSettings
from .cases.kc_quasi_static.contract import (
    has_rack,
    model_document,
    time_document,
)
from .cases.kc_quasi_static.convert import MM
from .io import CheckpointStore, canonical_hash, write_artifact
from .kernel.solver import solver_settings_document
from .modeling.primitives.elements import BushingElement
from .modeling.primitives.joints import RigidBodyState
from .modeling.primitives.spatial import (
    SE3,
    quaternion_to_rotation_vector,
)
from .preparation.signals import loads_at_time, motion, time_grid, wrenches_at_time
from .report.compliance import secant_compliance
from .report.metrics import (
    compute_axle_metrics,
    compute_case_metrics,
    compute_common_metrics,
)
from .results import TimeSeriesResult, TimeSeriesSample
from .rigs import compose, get_rig
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
from .simulation import CompiledSimulation, SimulationRequest, run_compiled, run_request
from .simulation.replay import VehicleKCTimeDomainSolver
from .subsystems.runtime import SubsystemRuntime, wheel_centre_local

#: The output grid a K/C case is solved on.  The kernel's case layer expands a
#: start/end/step, so the product API has to state one; two samples is the
#: minimum, and a K/C state is time independent, so a short one is right.
_TIMES_S = (0.0, 1e-3)

# The diagnostics column indices used to be declared here as well as in
# `results.raw`, and nothing read this copy: the residual report goes through
# `RawContractResult.case_residuals`.  A layout constant with two homes has none,
# so the duplicate is gone and `results` is the only place that knows a column.

#: The wheel-centre markers the two sides are loaded at.  A C case names them
#: rather than deriving them, because the kernel must refuse a document that
#: loads one side and means two.
_LEFT_MARKER = "wheel_center_L"
_RIGHT_MARKER = "wheel_center_R"


def run_case(
    model: FrontAxleModel,
    case: CaseSpec,
    output_dir: str | Path | None = None,
    *,
    inputs: Mapping[str, Any] | None = None,
) -> ResultBundle:
    """
    Run one validated model/case and optionally write result files.

    ``inputs`` carries the hashes of the files a file-driven run read, and is
    written beside the result when an output directory is given.  It is optional
    because a model authored in Python read no documents: recording an empty set of
    inputs would be a claim, and not recording one is the truth.
    """
    assembly = _kc_assembly(model, case.mode, case.subsystems)
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
        write_artifact(bundle, output_dir, model=model, case=case, inputs=inputs)
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
    # The replay path has no subsystem set of its own: a `DynamicCaseSpec`
    # states motion, not assembly, so it runs the default axle.
    assembly = _kc_assembly(model, "K")
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
                **(
                    {"rack_displacement": rack.value_at(time)}
                    if has_rack(assembly)
                    else {}
                ),
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

#: The test bench the K/C family runs on.  Named once because the rig, and not
#: this module, is what declares which coordinates a run drives.
_KC_RIG = "kc_quasi_static"

def _kc_assembly(
    model: FrontAxleModel,
    mode: Literal["K", "C"],
    subsystems: frozenset[str] | None = None,
) -> SubsystemRuntime:
    """
    Return the assembly a K/C run is solved from, with its bench checked.

    The construction belongs to the family preparation: it builds the assembly
    through the study layer and resolves the bench against it.  This module
    authors its own contract documents -- a split older than the study layer --
    but the assembly those documents are written from has to be the one the study
    layer builds, or the two readings drift apart and the rig is never consulted.

    ``subsystems`` is the run's own answer to "which subsystems does this
    assembly carry", and it is what makes a *steering-less* single-axle run
    reachable through the public entry.  A model with no steering has no rack
    coordinate, and the run has to be able to say so; without this the only way
    to ask was to build the assembly by hand and never reach `run_case` at all.
    """
    from .preparation.kc_quasi_static import assembly_for

    if subsystems is None:
        return assembly_for(model, mode=mode, rig=_KC_RIG)
    from .subsystems import AssemblyRequest

    return assembly_for(
        model,
        mode=mode,
        rig=_KC_RIG,
        request=AssemblyRequest(mode=mode, subsystems=subsystems),
    )


def _k_drivable_coordinates(assembly: SubsystemRuntime) -> frozenset[str]:
    """
    Return the drive coordinates the K/C bench can drive on this assembly.

    The judgement is the bench's declaration shrunk to the assembly's
    capabilities: `rigs.compose` keeps a drive only when the assembly offers its
    coordinate, and the K/C bench asks for the wheel and rack coordinates this
    module drives.  Reading the bench rather than
    `AssemblyCapabilities.drive_coordinates` directly is what makes the rig the
    one place "which coordinates a run drives" is stated; the two agree because
    `rack_neutral` is the bench's own input and never a drive, so it does not
    reach here either way.  An assembly without steering loses the rack axis and
    its outputs together -- a run that carried a rack axis of zeros would look
    steered and not be.

    A capabilities-less assembly (an older caller, or one built outside the
    subsystem path) falls back to the full set, which is what every such assembly
    has always been able to drive.
    """
    capabilities = getattr(assembly, "capabilities", None)
    if capabilities is None:
        return frozenset(_K_COORDINATES.values())
    composition = compose(get_rig(_KC_RIG), capabilities)
    return frozenset(drive.coordinate for drive in composition.drives)


def _k_grid(
    controls: list[DisplacementControl],
    *,
    drivable: frozenset[str] | None = None,
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
    # A control for a coordinate the assembly cannot drive is dropped before the
    # grid is built, so the axis is absent rather than zero-valued.  Dropping it
    # here -- rather than letting the case layer pad it -- is what keeps the grid's
    # dimensionality a consequence of the assembly.
    if drivable is not None:
        named = {
            axis: values
            for axis, values in named.items()
            if _K_COORDINATES[axis] in drivable
        }
    left = named.get("left", (0.0,))
    # "No rack axis" and "a rack axis of zero" are different statements, and the
    # difference is visible downstream: an empty list means the coordinate does not
    # exist, a `[0.0]` means it exists and happens to sit at zero.  Only the latter
    # would make a run look steered when it is not.
    rack_present = drivable is None or _K_COORDINATES["rack"] in drivable
    if "right" not in named:
        # The shorthand is the kernel's `wheel_values_mm` / `rack_values_mm` /
        # `axis_map` spelling, and it needs a rack axis: its contract is "wheel
        # travel plus rack".  An assembly with no rack coordinate therefore uses
        # the *general* form, which names exactly the axes the grid moves.  Both
        # reach the same kernel expansion; the difference is that the document
        # keeps saying "there is no rack" instead of hiding it behind a zero that
        # would make the run look steered.
        #
        # The one thing the general form has to reproduce is the *coupling*: the
        # shorthand drives both wheels from one travel, so the general form names
        # both wheel coordinates with the same values rather than naming the left
        # one alone -- which would silently move half the axle.
        if rack_present:
            return (
                {
                    "wheel_values_mm": list(left),
                    "rack_values_mm": list(named.get("rack", (0.0,))),
                    "axis_map": {
                        "wheel": list(_K_WHEEL_COORDINATES),
                        "rack": _K_COORDINATES["rack"],
                    },
                    "left_right_mode": "symmetric",
                },
                [
                    (travel, travel, position)
                    for travel in left
                    for position in named.get("rack", (0.0,))
                ],
            )
        # The shorthand without its rack axis: the same wheel axis it always
        # drove, and no rack key at all.  The kernel reads a missing
        # `rack_values_mm` as "this model has no rack coordinate to sweep", which
        # is exactly what the assembly says -- so the document keeps the shorthand
        # (and its symmetric wheel coupling) without inventing a rack.
        return (
            {
                "wheel_values_mm": list(left),
                "axis_map": {"wheel": list(_K_WHEEL_COORDINATES)},
                "left_right_mode": "symmetric",
            },
            [(travel, travel, 0.0) for travel in left],
        )
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


def _k_axes_section(
    axes: list[tuple[str, tuple[float, ...]]],
) -> tuple[dict[str, object], list[tuple[float, float, float]]]:
    """
    Build the general `k.axes` grid, and the drives of each case in its order.

    The general form names the coordinates it moves one axis at a time, which is
    how a run with fewer axes than the shorthand assumes is expressed.  Drives
    come out in the order the product is taken in -- the last axis varying
    fastest -- so the caller's index still lines up with the kernel's case index.
    A coordinate the axes do not name stays at the value the model was assembled
    with; the one coupling that matters here is the wheel set, which the
    shorthand drives together and which this form therefore carries with it.
    """
    section: dict[str, object] = {
        "axes": [
            {"coordinate": _K_COORDINATES[name], "values_mm": list(values)}
            for name, values in axes
        ]
    }
    combinations: list[tuple[float, float, float]] = []
    for values in product(*(values for _, values in axes)):
        drives = {"left": 0.0, "right": 0.0, "rack": 0.0}
        for (name, _), value in zip(axes, values):
            drives[name] = value
        if "right" not in (name for name, _ in axes):
            drives["right"] = drives["left"]
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


def _k_drives(
    assembly: SubsystemRuntime,
    *,
    left: float,
    right: float,
    rack: float,
) -> dict[str, float]:
    """
    Return the driven coordinates a K/C state reports, without a rack it has none of.

    A channel that is always present is a channel a reader cannot tell from a real
    zero: `{"rack_displacement": 0.0}` on a steering-less axle claims the run
    steered and the rack sat at neutral, when in truth there is no rack.  The
    rig's whole contract is that an absent coordinate disappears rather than being
    filled; the result surface has to say the same thing or the shrink is only
    half done.  The wheel channels stay, because a single-axle bench always drives
    wheel travel.
    """
    from .cases.kc_quasi_static.contract import has_rack as _has_rack

    drives = {"wheel_travel_left": left, "wheel_travel_right": right}
    if _has_rack(assembly):
        drives["rack_displacement"] = rack
    return drives


def _compile_plan_run(
    *,
    rig: str,
    assembly: SubsystemRuntime,
    mode: Literal["K", "C"],
    name: str,
    drive_wheels: bool,
    drive_mode: str | None = None,
    case_document: dict[str, object],
):
    """
    Route one K/C run through the unified compile-and-submit pipeline.

    The three steps used to be one: `api` wrote the model document itself and
    handed it to the runner, which compiled it through the K/C family.  Now the
    *plan* states what the run is and the compilation layer turns the plan and the
    assembly into the pair of documents, so the K/C path is the same pipeline as
    every other one rather than a route beside it.

    The case document stays this module's: it is `api`'s own load-sweep and grid
    spelling, and moving it would be a rewrite of the K/C input format rather than
    a routing change.
    """
    from .compilation import KcStudyInputs, compile_plan, plan_for

    plan = plan_for(
        rig,
        mode=mode,
        inputs=KcStudyInputs(name=name),
        times_s=_TIMES_S,
        solver=AxleSolverSettings(),
        drive_wheels=drive_wheels,
        drive_mode=drive_mode,
    )
    model_emitted, _, model_blob, _, metadata = compile_plan(plan, assembly)
    compiled = CompiledSimulation(
        request=SimulationRequest(
            assembly="axle",
            rig=rig,
            family=plan.family,
            study=plan.study,
            model=assembly,
            case=case_document,
            name=name,
        ),
        model_document=model_emitted,
        case_document=case_document,
        model_payload=model_blob or pack_container(model_emitted),
        case_payload=pack_container(case_document),
        layout={"document_order": ["model", "case"], "payload_order": ["model", "case"]},
        metadata=metadata,
    )
    from .results.element_wrench import ELEMENT_WRENCH_SWITCH

    # The component table is decoded from the kernel's own facts, so this run
    # asks for them.  The switch is set around the call rather than exported as a
    # process default: the channel is a *fact* surface this reporting path needs,
    # not something every solve should pay for, and a run that does not ask for it
    # leaves the result document at its original contract version.
    previous = os.environ.get(ELEMENT_WRENCH_SWITCH)
    os.environ[ELEMENT_WRENCH_SWITCH] = "1"
    try:
        return run_compiled(compiled).raw
    finally:
        if previous is None:
            os.environ.pop(ELEMENT_WRENCH_SWITCH, None)
        else:
            os.environ[ELEMENT_WRENCH_SWITCH] = previous


def _run_k(
    assembly: SubsystemRuntime,
    case: CaseSpec,
    checkpoint: CheckpointStore | None,
    hashes: tuple[str, str, str],
) -> tuple[list[StateResult], list[ComponentLoad], list[BushingResult]]:
    controls = [
        control for control in case.controls if isinstance(control, DisplacementControl)
    ]
    section, combinations = _k_grid(controls, drivable=_k_drivable_coordinates(assembly))
    run = _compile_plan_run(
        rig=_KC_RIG,
        assembly=assembly,
        mode="K",
        name=f"{case.name}-k",
        drive_wheels=True,
        drive_mode=case.drive_mode,
        case_document=_k_case_document(case, section),
    )

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
                drives=_k_drives(assembly, left=left, right=right, rack=rack),
                metrics=compute_case_metrics("kc_quasi_static", physical, assembly),
                tire_compression=_tire_compression(run, index),
                constraint_residual=constraint,
                force_residual=force,
                moment_residual=moment,
                converged=True,
                diagnostics=_convergence_note(state_id),
            )
        )
        loads, bushings_found = _collect_element_results(
            run, assembly, physical, state_id, index
        )
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
    assembly: SubsystemRuntime,
    case: CaseSpec,
    checkpoint: CheckpointStore | None,
    hashes: tuple[str, str, str],
) -> tuple[list[StateResult], list[ComponentLoad], list[BushingResult]]:
    controls = [control for control in case.controls if isinstance(control, LoadControl)]
    loads = _c_control_loads(controls, case.external_loads)
    run = _compile_plan_run(
        rig=_KC_RIG,
        assembly=assembly,
        mode="C",
        name=f"{case.name}-c",
        drive_wheels=False,
        drive_mode=case.drive_mode,
        case_document=_c_case_document(case, loads),
    )

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
                drives=_k_drives(assembly, left=0.0, right=0.0, rack=0.0),
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
                tire_compression=_tire_compression(run, index),
                constraint_residual=constraint,
                force_residual=force,
                moment_residual=moment,
                converged=True,
                diagnostics=_convergence_note(state_id),
            )
        )
        found, bushings_found = _collect_element_results(
            run, assembly, physical, state_id, index
        )
        component_loads.extend(found)
        bushings.extend(bushings_found)
        _checkpoint(checkpoint, state_id, *hashes)
    return states, component_loads, bushings


def _reference_state(assembly: SubsystemRuntime) -> RigidBodyState:
    """
    Return the pose the C response is measured from.

    Not the case's first sample: the C family applies its whole load at `t = 0`,
    so sample zero is *loaded*.  The reference is the pose the model was
    assembled at, which is what the kernel resolves an omitted driven target to
    and what the K reference equals.
    """
    pose = assembly.state
    if pose is None:
        # A runtime built without a state has no assembled pose to measure from.
        # Saying so is better than returning a plausible zero pose, which would
        # make the C response look measured from the right place.
        raise ValueError(
            "the assembly carries no reference state, so the C response has no "
            "pose to be measured from"
        )
    return pose


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
    assembly: SubsystemRuntime, bodies: list[str], row: np.ndarray
) -> RigidBodyState:
    """
    Rebuild the reporting state from one contract sample, metres to mm.

    The row's slices and the unit conversion belong to `results`, which owns the
    column layout; this is the call site that says so.  ``row`` is the whole
    sample slice -- one row per body, seven wide -- and it is reshaped here rather
    than at the call site so every caller passes the same thing.
    """
    from .results.kc_state import rigid_state_from_row

    # The body list comes from the caller when it names one, and otherwise from the
    # assembled pose -- which has to exist for the row to be laid out against it.
    reference = assembly.state
    if reference is None:
        raise ValueError(
            "the assembly carries no reference state, so a sample cannot be laid "
            "out against its body order"
        )
    names = list(bodies) or list(reference.bodies)
    return rigid_state_from_row(assembly, names, np.asarray(row, dtype=float))


def _wheel_response(
    state: RigidBodyState,
    reference_state: RigidBodyState,
    assembly: SubsystemRuntime,
    side: str,
) -> np.ndarray:
    """Return global wheel-center translation and rotation-vector response."""
    body = f"upright_{side}"
    local_center = wheel_centre_local(assembly, body)
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
        ),
    )


def _tire_compression(run, case_index: int = 0) -> dict[str, float]:
    """
    Report the vertical tire compression the solve actually produced.

    This used to echo zeros with the reasoning that a `CaseSpec` carries no road
    height and no tire radius.  That was true of the *input*, and it was the
    wrong conclusion: the wheel load is a result of the solve, not of the input,
    and the model the run is solved on does declare tires.  The column the
    compression lives in is `results`' to know, so this reads through it.
    """
    from .results.kc_state import tire_compression_from_run

    return tire_compression_from_run(run, case_index)


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
    run: Any,
    assembly: SubsystemRuntime,
    state: RigidBodyState,
    state_id: str,
    case_index: int,
) -> tuple[tuple[ComponentLoad, ...], tuple[BushingResult, ...]]:
    """
    Report one case's element loads and bushing deformations.

    The loads are the kernel's own, decoded from the native ``element_wrench``
    channel -- the wrenches the solve applied, rather than a second evaluation of
    each element law in Python.  That is what makes the component table agree
    with the solve by construction, and it is also why a reaction on a *fixed*
    body (a chassis or bench mount) is now reported: the solve had that fact in
    hand and left it in the channel, where before only the movable end was read.

    The bushing deformation stays the element's own `deformation` method: it is a
    read of the state against the mount's declared pose, not a solve result the
    kernel answered differently, and it is the same record the reporting model has
    always carried.
    """
    from .results.kc_state import element_wrenches_from_run

    loads = tuple(
        ComponentLoad(
            state_id=state_id,
            component=name,
            endpoint=body,
            global_load=_six_vector(global_wrench),
            local_load=_six_vector(local_wrench),
        )
        for name, body, global_wrench, local_wrench in element_wrenches_from_run(
            run, case_index
        )
    )
    bushings = tuple(
        BushingResult(
            state_id=state_id,
            bushing=element.name,
            deformation=_six_vector(element.deformation(state)),
            load=_six_vector(
                -element.stiffness @ element.deformation(state) + element.preload
            ),
            strain_energy=0.5
            * float(element.deformation(state) @ element.stiffness @ element.deformation(state)),
            stiffness_id=element.name,
            zero_load_pose=_schema_pose(element.local_pose_a),
        )
        for element in assembly.elements
        if isinstance(element, BushingElement)
    )
    return loads, bushings

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
