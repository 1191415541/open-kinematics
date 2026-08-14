"""Diagnose full-vehicle force balance before a long-horizon run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from suspension_multibody.adams import (
    build_adams_vehicle_case,
    build_adams_vehicle_model,
    load_adams_full_vehicle_input,
    read_vehicle_reference_bundle,
    steering_signal_from_manifest,
)
from suspension_multibody.analysis.full_vehicle_dynamic import (
    _fit_c_mode_static_preloads,
    _gravity_wrench,
    _initial_state,
)
from suspension_multibody.core import ConstraintSystem
from suspension_multibody.core import wrench_global_to_local
from suspension_multibody.dynamics import (
    ContactTireElement,
    ConstrainedDynamicIntegrator,
    DynamicContext,
    DynamicElementAdapter,
    RoadSurface,
    build_vehicle_actuators,
    evaluate_tire_contact,
    evaluate_dynamic_element,
)
from suspension_multibody.dynamics.forces import sum_dynamic_wrenches
from suspension_multibody.model import build_vehicle


ROOT = Path("artifacts/adams/correlation-reference-real-si/handling-pac2002-v1/step_steer")
TASK_RAW = Path(".codex-tasks/20260805-optimize-full-mbd-speed/raw")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--end-time", type=float, default=0.117)
    parser.add_argument("--step-size", type=float, default=0.01)
    parser.add_argument("--internal-step-size", type=float)
    parser.add_argument("--max-corrector-iterations", type=int)
    parser.add_argument("--output", type=Path, default=TASK_RAW / "instability_diagnostic.json")
    args = parser.parse_args()
    if args.end_time <= 0.0 or args.step_size <= 0.0:
        raise ValueError("--end-time and --step-size must be positive")
    if args.internal_step_size is not None and args.internal_step_size <= 0.0:
        raise ValueError("--internal-step-size must be positive")
    if args.max_corrector_iterations is not None and args.max_corrector_iterations < 1:
        raise ValueError("--max-corrector-iterations must be at least one")

    reference = read_vehicle_reference_bundle(ROOT / "adams_reference_bundle.json")
    data = load_adams_full_vehicle_input(ROOT)
    model = build_adams_vehicle_model(data)
    case = build_adams_vehicle_case(
        data,
        model,
        case_name="step_steer",
        steering_input=steering_signal_from_manifest(reference.input_manifest),
        end_time=args.end_time,
        step_size=args.step_size,
    )
    solver_updates: dict[str, object] = {}
    if args.internal_step_size is not None:
        solver_updates["internal_step_size"] = args.internal_step_size
        solver_updates["min_internal_step_size"] = min(
            args.internal_step_size, 1.0e-4
        )
    if args.max_corrector_iterations is not None:
        solver_updates["max_corrector_iterations"] = args.max_corrector_iterations
    if solver_updates:
        case = case.model_copy(
            update={"solver": case.solver.model_copy(update=solver_updates)}
        )
    assembly = build_vehicle(model, mode="C")
    road = RoadSurface(case.road)
    contacts = tuple(
        ContactTireElement(
            name=f"contact_{wheel.name}",
            wheel_body=wheel.body,
            spin_axis_local=wheel.spin_axis.as_array(),
            tire_spec=wheel.tire,
            road=road,
            corner_index=index,
            wheel_center_local=wheel.center_local.as_array(),
        )
        for index, wheel in enumerate(model.wheels)
    )
    state = _initial_state(case, assembly, road, contacts)
    if assembly.mode == "C" and case.static_equilibrium:
        assembly = _fit_c_mode_static_preloads(assembly, state, contacts, case)
    actuators = build_vehicle_actuators(
        model,
        assembly,
        steering_input=case.steering_input,
        brake_input=case.brake_input,
        drive_input=case.drive_input,
    )
    elements = tuple(DynamicElementAdapter(element) for element in assembly.elements)
    elements += contacts + tuple(actuators)
    system = ConstraintSystem(assembly.constraints)
    integrator = ConstrainedDynamicIntegrator(case.solver)
    integrator._prepare_runtime(state)
    accelerations, multipliers, events = integrator._coupled_accelerations(
        state, 0.0, elements, assembly.constraints, None
    )
    evaluations = tuple(
        evaluate_dynamic_element(
            element,
            state,
            0.0,
            DynamicContext(case.solver.allow_static_element_downgrade),
        )
        for element in elements
    )
    generalized_forces_global = sum_dynamic_wrenches(evaluations)
    generalized_forces_report = {}
    for body in state.body_order():
        runtime = state.pose_state.bodies[body]
        global_wrench = generalized_forces_global.get(body, np.zeros(6)).copy()
        global_wrench += _gravity_wrench(state.pose_state, runtime, case)
        generalized_forces_report[body] = {
            "global": global_wrench.tolist(),
            "local": wrench_global_to_local(state.pose_state.pose(body), global_wrench).tolist(),
        }
    contacts_report = {}
    for index, wheel in enumerate(model.wheels):
        result = evaluate_tire_contact(
            state,
            wheel_body=wheel.body,
            spin_axis_local=wheel.spin_axis.as_array(),
            tire_spec=wheel.tire,
            road=road,
            time=0.0,
            corner_index=index,
            wheel_center_local=wheel.center_local.as_array(),
        )
        contacts_report[wheel.name] = {
            "active": result.active,
            "compression_mm": result.compression,
            "normal_load_n": result.normal_load,
        }
    force_elements = []
    for element in elements:
        evaluation = element.evaluate_dynamic(state, 0.0)
        force_norm = sum(
            float(np.linalg.norm(wrench[:3]))
            for wrench in evaluation.body_wrenches_global.values()
        )
        force_elements.append(
            {
                "name": evaluation.name,
                "force_norm_n": force_norm,
                "active": evaluation.active,
                "events": evaluation.events,
            }
        )
    force_elements.sort(key=lambda item: item["force_norm_n"], reverse=True)
    max_acceleration = max(
        float(np.linalg.norm(value[:3])) for value in accelerations.values()
    )
    global_accelerations = {
        body: state.pose_state.pose(body).rotation
        @ (
            value[:3]
            + np.cross(state.velocities[body][3:], state.velocities[body][:3])
        )
        for body, value in accelerations.items()
    }
    max_global_acceleration = max(
        float(np.linalg.norm(value)) for value in global_accelerations.values()
    )
    order = state.body_order()
    jacobian = system.jacobian(state.pose_state, order)
    velocity_vector = np.concatenate([state.velocities[body] for body in order])
    initial_velocity_residual = (
        float(np.max(np.abs(jacobian @ velocity_vector))) if jacobian.size else 0.0
    )
    payload: dict[str, object] = {
        "contract": "full-mbd-instability-diagnostic-v1",
        "case": "step_steer",
        "end_time_s": args.end_time,
        "output_step_s": args.step_size,
        "mode": assembly.mode,
        "body_count": len(assembly.bodies),
        "constraint_count": len(assembly.constraints),
        "constraint_rows": int(system.residual(state.pose_state).size),
        "initial_constraint_residual": float(
            np.max(np.abs(system.residual(state.pose_state)))
            if assembly.constraints
            else 0.0
        ),
        "initial_velocity_residual": initial_velocity_residual,
        "initial_max_linear_acceleration": max_acceleration,
        "initial_max_global_linear_acceleration": max_global_acceleration,
        "initial_global_linear_accelerations": {
            body: value.tolist() for body, value in global_accelerations.items()
        },
        "initial_accelerations": {
            body: value.tolist() for body, value in accelerations.items()
        },
        "initial_generalized_forces": generalized_forces_report,
        "initial_contacts": contacts_report,
        "top_force_elements": force_elements[:12],
        "initial_events": events,
        "initial_multiplier_norm": float(np.linalg.norm(multipliers)),
    }
    started = perf_counter()
    try:
        results = integrator.integrate(
            state,
            elements=elements,
            constraints=assembly.constraints,
        )
    except Exception as error:
        payload.update(
            {
                "status": "FAILED",
                "wall_until_failure_s": perf_counter() - started,
                "error": f"{type(error).__name__}: {error}",
                "failure_context": _serializable_failure(integrator.last_failure),
            }
        )
    else:
        constraint_residuals = [result.constraint_residual for result in results]
        velocity_residuals = [result.velocity_residual for result in results]
        recovery_events = tuple(
            event
            for result in results
            for event in result.events
            if event.startswith("velocity_recovery")
        )
        payload.update(
            {
                "status": "OK",
                "wall_s": perf_counter() - started,
                "sample_count": len(results),
                "max_position_residual": max(constraint_residuals),
                "max_velocity_residual": max(velocity_residuals),
                "recovery_event_count": len(recovery_events),
                "recovery_events": recovery_events,
            }
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))


def _serializable_failure(failure: dict[str, object] | None) -> dict[str, object] | None:
    """Drop state objects while preserving all numeric failure evidence."""
    if failure is None:
        return None
    result: dict[str, object] = {}
    for key, value in failure.items():
        if key == "state":
            continue
        if key in {"first_candidate", "last_candidate"} and isinstance(value, dict):
            result[key] = {item_key: item_value for item_key, item_value in value.items() if item_key != "state"}
        else:
            result[key] = value
    return result


if __name__ == "__main__":
    main()
