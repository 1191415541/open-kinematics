"""Machine-checkable preflight and runtime audit for dynamic axle equivalence."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any, cast

import numpy as np

from ..axle_dynamics import (
    BODY_STATE_COLUMNS,
    CONSTRAINT_WRENCH_COLUMNS,
    SPRING_OUTPUT_COLUMNS,
    TIRE_OUTPUT_COLUMNS,
    AxleDynamicsCase,
    AxleDynamicsModel,
)
from ..io import canonical_hash
from .axle_adams_model import (
    AxleAdamsDataset,
    axle_adams_blockers,
)
from .axle_contract import (
    FIXTURE_WRENCH_CONVENTION,
    DynamicAxleManifest,
)
from .axle_dynamic_history import (
    AdamsAxleResult,
    adams_axle_raw_channel_map,
)
from .time_domain import TimeHistory

ADAMS_AXLE_EQUIVALENCE_AUDIT_CONTRACT = "dynamic-axle-equivalence-audit-v2"

_GRID_TOLERANCE_S = 1e-12
_STATE_TOLERANCE = 1e-8
_ELEMENT_ABSOLUTE_TOLERANCE = 1e-4
_ELEMENT_RELATIVE_TOLERANCE = 1e-8


def audit_axle_equivalence(
    manifest: DynamicAxleManifest,
    dataset: AxleAdamsDataset | Mapping[str, object],
    *,
    native_history: TimeHistory | None = None,
    adams_history: TimeHistory | None = None,
    adams_result: AdamsAxleResult | None = None,
    native_evidence: Mapping[str, object] | None = None,
    adams_evidence: Mapping[str, object] | None = None,
    source_provenance: Mapping[str, object] | None = None,
    require_runtime: bool = False,
) -> dict[str, object]:
    """
    Audit every comparison-critical input before precision metrics run.

    With ``require_runtime=False`` this is a static manifest/dataset preflight.
    The final audit sets it to ``True`` and additionally requires the native
    and Adams histories, raw Adams result, and common initial-state evidence.
    """
    model = manifest.model
    case = manifest.case
    dataset_view = _dataset_view(dataset)
    blockers: list[str] = []

    identity, identity_blockers = _audit_identity(
        manifest,
        dataset_view,
        native_evidence,
        require_runtime=require_runtime,
    )
    blockers.extend(identity_blockers)

    coverage, coverage_blockers = _audit_model_coverage(
        model,
        case,
        dataset_view,
    )
    blockers.extend(coverage_blockers)

    road_input, road_blockers = _audit_road_input(model, case, dataset_view)
    blockers.extend(road_blockers)

    time_grid, grid_blockers = _audit_time_grid(
        case,
        dataset_view,
        native_history=native_history,
        adams_history=adams_history,
        adams_result=adams_result,
        require_runtime=require_runtime,
    )
    blockers.extend(grid_blockers)

    raw_bindings, raw_blockers = _audit_raw_bindings(model, dataset)
    blockers.extend(raw_blockers)

    channels, channel_blockers = _audit_channels(
        manifest,
        native_history=native_history,
        adams_history=adams_history,
        require_runtime=require_runtime,
    )
    blockers.extend(channel_blockers)

    initialization, initialization_blockers = _audit_initialization(
        model,
        case,
        native_evidence=native_evidence,
        adams_result=adams_result,
        require_runtime=require_runtime,
    )
    blockers.extend(initialization_blockers)

    output_conventions, convention_blockers = _audit_output_conventions(
        model,
        dataset_view,
        native_history=native_history,
        adams_history=adams_history,
        require_runtime=require_runtime,
    )
    blockers.extend(convention_blockers)

    solver_conditions, solver_blockers = _audit_solver_conditions(
        manifest,
        dataset_view,
        native_evidence=native_evidence,
        adams_evidence=adams_evidence,
        require_runtime=require_runtime,
    )
    blockers.extend(solver_blockers)

    provenance = _source_provenance(
        manifest,
        source_provenance,
    )
    unique_blockers = _unique(blockers)
    passed = not unique_blockers
    return {
        "contract": ADAMS_AXLE_EQUIVALENCE_AUDIT_CONTRACT,
        "status": "PASS" if passed else "BLOCKED",
        "equivalence_gate_passed": passed,
        "require_runtime": require_runtime,
        "blockers": unique_blockers,
        "shared_manifest_identity": identity,
        "source_database_provenance": provenance,
        "model_field_coverage": coverage,
        "initialization": initialization,
        "road_input": road_input,
        "time_grid": time_grid,
        "raw_bindings": raw_bindings,
        "channels": channels,
        "output_conventions": output_conventions,
        "solver_conditions": solver_conditions,
    }


def _dataset_view(
    dataset: AxleAdamsDataset | Mapping[str, object],
) -> dict[str, object]:
    if isinstance(dataset, AxleAdamsDataset):
        payload = dataset.as_dict()
        return {
            "manifest_sha256": dataset.manifest_sha256,
            "dataset_sha256": payload["dataset_sha256"],
            "dataset_hash_calculated": payload["dataset_sha256"],
            "conventions": dict(dataset.conventions),
            "entity_ids": dict(dataset.entity_ids),
            "requests": [dict(item) for item in dataset.requests],
            "model_text": dataset.model_text,
            "command_text": dataset.command_text,
        }
    payload = dict(dataset)
    expected_hash = payload.get("dataset_sha256")
    content = dict(payload)
    content.pop("dataset_sha256", None)
    return {
        "manifest_sha256": payload.get("manifest_sha256"),
        "dataset_sha256": expected_hash,
        "dataset_hash_calculated": canonical_hash(content),
        "conventions": payload.get("conventions", {}),
        "entity_ids": payload.get("entity_ids", {}),
        "requests": payload.get("requests", []),
        "model_text": None,
        "command_text": None,
    }


def _audit_solver_conditions(
    manifest: DynamicAxleManifest,
    dataset: Mapping[str, object],
    *,
    native_evidence: Mapping[str, object] | None,
    adams_evidence: Mapping[str, object] | None,
    require_runtime: bool,
) -> tuple[dict[str, object], list[str]]:
    """Separate exact discrete-method equality from converged physics equality."""
    case = manifest.case
    solver = cast(Mapping[str, object], manifest.payload["adams_solver"])
    integrator = str(solver.get("integrator", "")).strip().lower()
    native_integrator = case.solver.integrator
    alpha_value = solver.get("alpha")
    blockers: list[str] = []
    try:
        alpha = float(alpha_value)
    except (TypeError, ValueError):
        alpha = math.nan

    rho = float(case.solver.rho_inf)
    native_hht_alpha = float(case.solver.hht_alpha)
    if native_integrator == "hht":
        native_alpha_m = 0.0
        native_alpha_f = -native_hht_alpha
        native_gamma = 0.5 - native_hht_alpha
        native_beta = 0.25 * (1.0 - native_hht_alpha) ** 2
        # Adams converts a DIFF y' = f(...) into HHT on an auxiliary
        # integral x with x' = y. Eliminating x leaves this residual:
        # y'_{n+1} - f(y_{n+1-alpha_f}, u_{n+1-alpha_f}, t_{n+1-alpha_f}) = 0.
        native_tire_state = {
            "formulation": "hht_integrated_diff_state",
            "next_derivative_weight": 1.0,
            "previous_derivative_weight": 0.0,
            "state_next_weight": 1.0 - native_alpha_f,
            "evaluation_time_fraction": 1.0 - native_alpha_f,
            "gamma": native_gamma,
        }
    else:
        native_alpha_m = (2.0 * rho - 1.0) / (rho + 1.0)
        native_alpha_f = rho / (rho + 1.0)
        native_gamma = 0.5 - native_alpha_m + native_alpha_f
        native_beta = 0.25 * (1.0 - native_alpha_m + native_alpha_f) ** 2
        native_alpha_m_z = (3.0 - rho) / (2.0 * (1.0 + rho))
        native_alpha_f_z = 1.0 / (1.0 + rho)
        native_gamma_z = 0.5 + native_alpha_m_z - native_alpha_f_z
        native_tire_state = {
            "formulation": "ggl_first_order_generalized_alpha",
            "next_derivative_weight": native_alpha_m_z,
            "previous_derivative_weight": 1.0 - native_alpha_m_z,
            "state_next_weight": native_alpha_f_z,
            "evaluation_time_fraction": native_alpha_f_z,
            "gamma": native_gamma_z,
        }
    native_coefficients = {
        "second_order": {
            "alpha_m": native_alpha_m,
            "alpha_f": native_alpha_f,
            "gamma": native_gamma,
            "beta": native_beta,
        },
        "first_order_tire_state": native_tire_state,
    }

    if integrator != "hht":
        blockers.append("dynamic comparison requires Adams HHT explicitly")
    if not math.isfinite(alpha):
        blockers.append("Adams HHT alpha is missing or invalid")
    command_text = dataset.get("command_text")
    command_alpha_present = None
    if isinstance(command_text, str) and integrator == "hht" and math.isfinite(alpha):
        expected = f"integrator/hht, alpha = {alpha:.12g}"
        command_alpha_present = expected in command_text.lower()
        if not command_alpha_present:
            blockers.append("Adams command does not pin the declared HHT alpha")

    adams_coefficients: dict[str, object] = {
        "second_order": (
            {
                "alpha_m": 0.0,
                "alpha_f": -alpha,
                "gamma": 0.5 - alpha,
                "beta": 0.25 * (1.0 - alpha) ** 2,
            }
            if math.isfinite(alpha)
            else None
        ),
        "first_order_tire_state": (
            {
                "formulation": "hht_integrated_diff_state",
                "next_derivative_weight": 1.0,
                "previous_derivative_weight": 0.0,
                "state_next_weight": 1.0 + alpha,
                "evaluation_time_fraction": 1.0 + alpha,
                "gamma": 0.5 - alpha,
            }
            if integrator == "hht" and math.isfinite(alpha)
            else "Adams HHT first-order state scheme is unavailable"
        ),
    }
    coefficient_errors: dict[str, float] | None = None
    second_order_match = False
    if (
        isinstance(adams_coefficients["second_order"], Mapping)
        and math.isfinite(alpha)
    ):
        adams_second = cast(Mapping[str, float], adams_coefficients["second_order"])
        coefficient_errors = {
            name: abs(float(native_coefficients["second_order"][name]) - adams_second[name])
            for name in ("alpha_m", "alpha_f", "gamma", "beta")
        }
        second_order_match = max(coefficient_errors.values(), default=math.inf) <= 1e-12

    first_order_match = (
        isinstance(native_coefficients["first_order_tire_state"], Mapping)
        and isinstance(adams_coefficients["first_order_tire_state"], Mapping)
        and native_integrator == "hht"
        and integrator == "hht"
        and math.isfinite(alpha)
        and second_order_match
        and all(
            math.isclose(
                float(cast(Mapping[str, object], native_coefficients["first_order_tire_state"])[name]),
                float(cast(Mapping[str, object], adams_coefficients["first_order_tire_state"])[name]),
                rel_tol=0.0,
                abs_tol=1e-12,
            )
            for name in (
                "next_derivative_weight",
                "previous_derivative_weight",
                "state_next_weight",
                "evaluation_time_fraction",
                "gamma",
            )
        )
    )
    discrete_match = second_order_match and first_order_match
    if require_runtime and not second_order_match:
        blockers.append(
            "native and Adams second-order HHT coefficients are not identical"
        )
    if require_runtime and not first_order_match:
        blockers.append(
            "native and Adams DIFF-state HHT discretizations are not identical"
        )

    maximum_step = _finite_float(solver.get("maximum_step_s"))
    internal_step = float(case.solver.internal_step_s)
    public_step = float(case.times_s[1] - case.times_s[0])
    expected_ratio = public_step / internal_step
    fixed_iterations = solver.get("fixed_iterations")
    step_ratio = solver.get("step_ratio")
    fixed_step_declared = fixed_iterations is not None and step_ratio is not None
    fixed_step_conditions = (
        not case.solver.adaptive_step
        and maximum_step is not None
        and math.isclose(maximum_step, internal_step, rel_tol=0.0, abs_tol=1e-15)
        and fixed_step_declared
        and float(step_ratio) == float(int(step_ratio))
        and int(step_ratio) >= 1
        and math.isclose(expected_ratio, float(int(step_ratio)), rel_tol=0.0, abs_tol=1e-12)
    )
    if require_runtime and not fixed_step_conditions:
        blockers.append(
            "runtime comparison requires native fixed steps and Adams FIXIT/HRATIO "
            "with DTOUT/HRATIO equal to the native internal step"
        )
    command_fixed_present = None
    command_text = dataset.get("command_text")
    if (
        isinstance(command_text, str)
        and fixed_step_declared
        and integrator == "hht"
    ):
        command_lower = command_text.lower()
        command_fixed_present = (
            f"fixit = {int(fixed_iterations)}" in command_lower
            and f"hratio = {int(step_ratio)}" in command_lower
        )
        if require_runtime and not command_fixed_present:
            blockers.append("Adams command does not pin FIXIT and HRATIO")

    metadata = manifest.payload.get("case_metadata")
    metadata_map = cast(Mapping[str, object], metadata) if isinstance(metadata, Mapping) else {}
    comparison_basis = metadata_map.get("comparison_basis")
    native_convergence = _evidence_flag(native_evidence, "time_convergence_passed")
    adams_convergence = _evidence_flag(adams_evidence, "time_convergence_passed")
    if adams_convergence is None and isinstance(adams_evidence, Mapping):
        nested = adams_evidence.get("time_convergence")
        if isinstance(nested, Mapping):
            adams_convergence = _bool_or_none(nested.get("passed"))
    if require_runtime:
        if comparison_basis != "continuous_problem_convergence":
            blockers.append(
                "runtime precision comparison requires comparison_basis="
                "continuous_problem_convergence"
            )
        if native_convergence is not True:
            blockers.append("native time-convergence evidence did not pass")
        if adams_convergence is not True:
            blockers.append("Adams time-convergence evidence did not pass")

    return (
        {
            "passed": not blockers,
            "comparison_basis": comparison_basis,
            "native_integrator": native_integrator,
            "native_rho_inf": rho,
            "native_hht_alpha": (
                native_hht_alpha if native_integrator == "hht" else None
            ),
            "adams_integrator": integrator,
            "adams_hht_alpha": alpha if math.isfinite(alpha) else None,
            "native_coefficients": native_coefficients,
            "adams_coefficients": adams_coefficients,
            "coefficient_absolute_errors": coefficient_errors,
            "second_order_discrete_integrator_equivalent": second_order_match,
            "first_order_tire_discrete_integrator_equivalent": first_order_match,
            "discrete_integrator_equivalent": discrete_match,
            "discrete_integrator_equivalence_reason": (
                "native HHT eliminates the same auxiliary integral used by Adams "
                "for DIFF states"
                if discrete_match
                else "native and Adams HHT coefficients or DIFF-state formulations differ"
            ),
            "fixed_step_conditions": fixed_step_conditions,
            "fixed_step_declared": fixed_step_declared,
            "fixed_iterations": fixed_iterations,
            "step_ratio": step_ratio,
            "expected_step_ratio": expected_ratio,
            "adams_error_ignored_by_fixed_step": fixed_step_declared,
            "adams_command_fixed_step_present": command_fixed_present,
            "native_time_convergence_passed": native_convergence,
            "adams_time_convergence_passed": adams_convergence,
            "adams_command_alpha_present": command_alpha_present,
            "runtime_convergence_gate": (
                discrete_match
                and fixed_step_conditions
                and comparison_basis == "continuous_problem_convergence"
                and native_convergence is True
                and adams_convergence is True
            ),
        },
        blockers,
    )


def _finite_float(value: object) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _bool_or_none(value: object) -> bool | None:
    return value if isinstance(value, bool) else None


def _evidence_flag(
    evidence: Mapping[str, object] | None,
    key: str,
) -> bool | None:
    if evidence is None:
        return None
    direct = _bool_or_none(evidence.get(key))
    if direct is not None:
        return direct
    diagnostics = evidence.get("diagnostics")
    if isinstance(diagnostics, Mapping):
        return _bool_or_none(diagnostics.get(key))
    return None


def _audit_identity(
    manifest: DynamicAxleManifest,
    dataset: Mapping[str, object],
    native_evidence: Mapping[str, object] | None,
    *,
    require_runtime: bool,
) -> tuple[dict[str, object], list[str]]:
    blockers: list[str] = []
    manifest_match = dataset.get("manifest_sha256") == manifest.sha256
    if not manifest_match:
        blockers.append(
            "dataset manifest_sha256 does not match the shared dynamic manifest"
        )
    dataset_hash = dataset.get("dataset_sha256")
    calculated_hash = dataset.get("dataset_hash_calculated")
    dataset_hash_valid = (
        isinstance(dataset_hash, str)
        and isinstance(calculated_hash, str)
        and dataset_hash == calculated_hash
    )
    if not dataset_hash_valid:
        blockers.append("Adams dataset sidecar hash does not match its content")
    native_manifest = (
        native_evidence.get("manifest_sha256")
        if native_evidence is not None
        else None
    )
    native_manifest_match = (
        native_manifest == manifest.sha256 if native_evidence is not None else None
    )
    if require_runtime and native_manifest_match is not True:
        blockers.append("native evidence does not carry the shared manifest hash")
    return (
        {
            "passed": manifest_match and dataset_hash_valid and (
                not require_runtime or native_manifest_match is True
            ),
            "manifest_sha256": manifest.sha256,
            "dataset_manifest_sha256": dataset.get("manifest_sha256"),
            "dataset_sha256": dataset_hash,
            "dataset_hash_calculated": calculated_hash,
            "dataset_hash_valid": dataset_hash_valid,
            "native_manifest_sha256": native_manifest,
            "native_manifest_match": native_manifest_match,
        },
        blockers,
    )


def _audit_model_coverage(
    model: AxleDynamicsModel,
    case: AxleDynamicsCase,
    dataset: Mapping[str, object],
) -> tuple[dict[str, object], list[str]]:
    ids = dataset.get("entity_ids")
    model_text = dataset.get("model_text")
    blockers: list[str] = []
    covered: list[str] = []
    omitted_zero_effect: list[str] = []
    if not isinstance(ids, Mapping):
        return (
            {
                "passed": False,
                "covered": covered,
                "omitted_zero_effect": omitted_zero_effect,
                "missing_entity_ids": ["entity_ids"],
            },
            ["Adams dataset entity_ids is not a mapping"],
        )

    for blocker in axle_adams_blockers(model, case):
        blockers.append(blocker)

    missing: list[str] = []

    def require(key: str, label: str | None = None) -> None:
        value = ids.get(key)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            missing.append(label or key)
        else:
            covered.append(label or key)

    for body in model.bodies:
        if body.fixed:
            covered.append(f"body:{body.name}:fixed_on_ground")
        else:
            require(f"body:{body.name}:cm")
    for joint in model.joints:
        require(f"joint:{joint.name}")
        require(f"joint:{joint.name}:i")
        require(f"joint:{joint.name}:j")
    for spring in model.springs:
        require(f"spring:{spring.name}")
        require(f"spring:{spring.name}:i")
        require(f"spring:{spring.name}:j")
        if spring.damper_curve_velocity_m_per_s:
            require(f"spring:{spring.name}:damper_curve")
        else:
            covered.append(f"spring:{spring.name}:constant_damping")
    for bushing in model.bushings:
        if _bushing_has_force_terms(bushing):
            blockers.append(
                f"bushing {bushing.name!r} has nonzero terms and is not emitted"
            )
        else:
            omitted_zero_effect.append(f"bushing:{bushing.name}:zero_force_terms")
    for bar in model.anti_roll_bars:
        if bar.stiffness_n_m_per_rad != 0.0 or bar.damping_n_m_s_per_rad != 0.0:
            blockers.append(
                f"anti-roll bar {bar.name!r} has nonzero terms and is not emitted"
            )
        else:
            omitted_zero_effect.append(f"anti_roll_bar:{bar.name}:zero_force_terms")
    for tire in model.tires:
        for key in (
            f"tire:{tire.name}:centre",
            f"tire:{tire.name}:forward",
            f"tire:{tire.name}:spin",
            f"tire:{tire.name}:jfloat",
            f"tire:{tire.name}:gforce",
        ):
            require(key)
        for variable in (
            "penetration",
            "penetration_rate",
            "normal_force",
            "longitudinal_force",
            "lateral_force",
            "longitudinal_slip",
            "lateral_slip",
            "friction_utilization",
            "brush_x",
            "brush_y",
        ):
            require(f"tire:{tire.name}:{variable}")
    for tire in model.tires:
        for label in ("road_height", "road_velocity", "wheel_torque"):
            require(f"input:{label}:{tire.name}")
    body_names = {body.name: body for body in model.bodies}
    for name, wrenches in sorted(case.body_wrench_n_n_m.items()):
        body = body_names.get(name)
        if body is None:
            blockers.append(f"body wrench references unknown body {name!r}")
            continue
        nonzero = any(any(value != 0.0 for value in wrench) for wrench in wrenches)
        if body.fixed:
            if nonzero:
                blockers.append(
                    f"nonzero body wrench on fixed body {name!r} is not emitted"
                )
            else:
                omitted_zero_effect.append(f"input:body_wrench:{name}:fixed_zero")
            continue
        for component in range(6):
            require(f"input:body_wrench:{name}:{component}")
        require(f"input:body_wrench:{name}:jfloat")
        require(f"input:body_wrench:{name}:gforce")
    if model_text is not None:
        gravity = ", ".join(_number(value) for value in model.gravity_m_per_s2)
        gravity_present = (
            "ACCGRAV/IGRAV = " in str(model_text)
            and all(value in str(model_text) for value in gravity.split(", "))
        )
        if not gravity_present:
            blockers.append("model text does not emit the manifest gravity vector")
        else:
            covered.append("gravity_m_per_s2")
    else:
        covered.append("gravity_m_per_s2:sidecar_text_not_available")
    if missing:
        blockers.append("Adams dataset is missing entity ids: " + ", ".join(missing))
    return (
        {
            "passed": not blockers,
            "covered": covered,
            "omitted_zero_effect": omitted_zero_effect,
            "missing_entity_ids": missing,
            "generator_blockers": list(axle_adams_blockers(model, case)),
        },
        blockers,
    )


def _audit_road_input(
    model: AxleDynamicsModel,
    case: AxleDynamicsCase,
    dataset: Mapping[str, object],
) -> tuple[dict[str, object], list[str]]:
    blockers: list[str] = []
    tires = {tire.name for tire in model.tires}
    signals = {
        "road_height_m": case.road_height_m,
        "road_velocity_m_per_s": case.road_velocity_m_per_s,
        "wheel_torque_n_m": case.wheel_torque_n_m,
    }
    unknown_signals = {
        label: sorted(set(values) - tires)
        for label, values in signals.items()
        if set(values) - tires
    }
    for label, names in unknown_signals.items():
        blockers.append(f"{label} contains unknown tire inputs: {names}")
    harmonic_unknown = sorted(
        {road.tire for road in case.harmonic_roads} - tires
    )
    if harmonic_unknown:
        blockers.append(f"harmonic road references unknown tires: {harmonic_unknown}")
    harmonic_checks: list[dict[str, object]] = []
    for road in case.harmonic_roads:
        rate = 2.0 * math.pi * road.frequency_hz
        heights = tuple(
            road.offset_m
            + road.amplitude_m * math.sin(rate * time + road.phase_rad)
            for time in case.times_s
        )
        velocities = tuple(
            road.amplitude_m * rate * math.cos(rate * time + road.phase_rad)
            for time in case.times_s
        )
        actual_height = case.road_height_m.get(road.tire, ())
        actual_velocity = case.road_velocity_m_per_s.get(road.tire, ())
        height_error = _max_difference(actual_height, heights)
        velocity_error = _max_difference(actual_velocity, velocities)
        passed = (
            height_error <= _GRID_TOLERANCE_S
            and velocity_error <= _GRID_TOLERANCE_S
        )
        if not passed:
            blockers.append(
                f"harmonic road {road.tire!r} sampled values do not match its "
                "closed form"
            )
        harmonic_checks.append(
            {
                "tire": road.tire,
                "height_max_abs_error_m": height_error,
                "velocity_max_abs_error_m_per_s": velocity_error,
                "passed": passed,
            }
        )
    return (
        {
            "passed": not blockers,
            "tires": sorted(tires),
            "harmonic_roads": harmonic_checks,
            "input_entity_ids": sorted(
                key
                for key in cast(Mapping[str, object], dataset.get("entity_ids", {}))
                if str(key).startswith("input:")
            ),
            "unknown_signals": unknown_signals,
        },
        blockers,
    )


def _audit_time_grid(
    case: AxleDynamicsCase,
    dataset: Mapping[str, object],
    *,
    native_history: TimeHistory | None,
    adams_history: TimeHistory | None,
    adams_result: AdamsAxleResult | None,
    require_runtime: bool,
) -> tuple[dict[str, object], list[str]]:
    expected = np.asarray(case.times_s, dtype=float)
    differences = np.diff(expected)
    uniform = bool(
        len(differences) > 0
        and np.all(np.abs(differences - differences[0]) <= _GRID_TOLERANCE_S)
    )
    blockers: list[str] = []
    if not math.isclose(float(expected[0]), 0.0, abs_tol=_GRID_TOLERANCE_S):
        blockers.append("public time grid must start at t=0")
    if not uniform:
        blockers.append("public time grid is not uniform")
    if dataset.get("command_text") is not None and uniform:
        dt_text = f"dtout = {_number(float(differences[0]))}"
        if dt_text not in str(dataset["command_text"]):
            blockers.append("Adams command dtout does not equal the public grid")

    actuals: dict[str, object] = {}
    for label, values in (
        ("native_history", native_history.time if native_history else None),
        ("adams_history", adams_history.time if adams_history else None),
        ("adams_result", adams_result.times_s if adams_result else None),
    ):
        if values is None:
            actuals[label] = {"available": False, "matches_expected": None}
            continue
        actual = np.asarray(values, dtype=float)
        if actual.shape != expected.shape:
            matches = False
            max_error = math.inf
        else:
            max_error = float(np.max(np.abs(actual - expected)))
            matches = bool(max_error <= _GRID_TOLERANCE_S)
        if not matches:
            blockers.append(f"{label} time grid does not match the public grid")
        actuals[label] = {
            "available": True,
            "sample_count": int(actual.size),
            "matches_expected": matches,
            "max_abs_error_s": max_error,
        }
    if require_runtime:
        for label in ("native_history", "adams_history", "adams_result"):
            if not cast(Mapping[str, object], actuals[label])["available"]:
                blockers.append(f"runtime equivalence audit is missing {label}")
    return (
        {
            "passed": not blockers,
            "expected_start_s": float(expected[0]),
            "expected_end_s": float(expected[-1]),
            "expected_sample_count": int(expected.size),
            "expected_step_s": float(differences[0]) if uniform else None,
            "uniform": uniform,
            "actual": actuals,
        },
        blockers,
    )


def _audit_raw_bindings(
    model: AxleDynamicsModel,
    dataset: AxleAdamsDataset | Mapping[str, object],
) -> tuple[dict[str, object], list[str]]:
    blockers: list[str] = []
    try:
        mapping = adams_axle_raw_channel_map(model, dataset)
        channels = sorted(mapping)
    except (KeyError, TypeError, ValueError) as exc:
        mapping = {}
        channels = []
        blockers.append(f"raw Adams channel binding failed: {exc}")
    return (
        {
            "passed": not blockers,
            "channel_count": len(channels),
            "channels": channels,
            "bindings": {
                name: {"entity": channel.entity, "component": channel.component}
                for name, channel in mapping.items()
            },
        },
        blockers,
    )


def _audit_channels(
    manifest: DynamicAxleManifest,
    *,
    native_history: TimeHistory | None,
    adams_history: TimeHistory | None,
    require_runtime: bool,
) -> tuple[dict[str, object], list[str]]:
    contract = manifest.payload.get("channels")
    expected_payload = (
        cast(Mapping[str, object], contract).get("channels", {})
        if isinstance(contract, Mapping)
        else {}
    )
    expected_names = tuple(str(name) for name in expected_payload)
    expected_units = {
        str(name): str(cast(Mapping[str, object], value).get("unit"))
        for name, value in cast(Mapping[str, object], expected_payload).items()
        if isinstance(value, Mapping)
    }
    blockers: list[str] = []
    actuals: dict[str, object] = {}
    for label, history in (
        ("native_history", native_history),
        ("adams_history", adams_history),
    ):
        if history is None:
            actuals[label] = {"available": False, "matches_contract": None}
            continue
        names_match = tuple(history.channels) == expected_names
        names_set_match = set(history.channels) == set(expected_names)
        units_match = history.units == expected_units
        if not names_match:
            blockers.append(
                f"{label} channel names or order differ from the frozen contract"
            )
        if not units_match:
            blockers.append(f"{label} channel units differ from the frozen contract")
        actuals[label] = {
            "available": True,
            "channel_count": len(history.channels),
            "names_match": names_match,
            "names_set_match": names_set_match,
            "units_match": units_match,
            "matches_contract": names_match and units_match,
        }
    if require_runtime:
        for label in ("native_history", "adams_history"):
            if not cast(Mapping[str, object], actuals[label])["available"]:
                blockers.append(f"runtime equivalence audit is missing {label}")
    return (
        {
            "passed": not blockers,
            "expected_channel_count": len(expected_names),
            "expected_channels": list(expected_names),
            "actual": actuals,
        },
        blockers,
    )


def _audit_initialization(
    model: AxleDynamicsModel,
    case: AxleDynamicsCase,
    *,
    native_evidence: Mapping[str, object] | None,
    adams_result: AdamsAxleResult | None,
    require_runtime: bool,
) -> tuple[dict[str, object], list[str]]:
    blockers: list[str] = []
    mode = case.solver.initialization_mode
    if mode != "provided_consistent_state":
        blockers.append(
            "dynamic equivalence requires solver.initialization_mode="
            "'provided_consistent_state'"
        )
    evidence_initialization = (
        native_evidence.get("initialization")
        if native_evidence is not None
        else None
    )
    state = (
        evidence_initialization.get("state")
        if isinstance(evidence_initialization, Mapping)
        else None
    )
    state_hash = (
        evidence_initialization.get("state_sha256")
        if isinstance(evidence_initialization, Mapping)
        else None
    )
    state_hash_valid = (
        isinstance(state, Mapping)
        and isinstance(state_hash, str)
        and canonical_hash(state) == state_hash
    )
    if state is not None and not state_hash_valid:
        blockers.append("native initial-state evidence hash is invalid")
    if require_runtime and not isinstance(state, Mapping):
        blockers.append("runtime equivalence audit is missing native initial-state evidence")

    comparison: dict[str, object] = {
        "performed": False,
        "passed": None,
        "body_position_max_abs_error_m": None,
        "body_quaternion_max_angle_rad": None,
        "body_velocity_max_abs_error_m_per_s": None,
        "body_angular_velocity_max_abs_error_rad_per_s": None,
        "constraint_wrench_max_abs_error": None,
        "spring_output_max_abs_error_n": None,
        "tire_output_max_abs_error": None,
    }
    if isinstance(state, Mapping) and adams_result is not None:
        try:
            comparison = _compare_initial_state(model, state, adams_result)
        except (IndexError, KeyError, TypeError, ValueError) as exc:
            comparison = _incomplete_initial_state(str(exc))
        if not bool(comparison["passed"]):
            blockers.append("Adams and native initial states/outputs are not equivalent")
    elif require_runtime:
        blockers.append("runtime equivalence audit is missing Adams initial-state data")
    return (
        {
            "passed": not blockers,
            "mode": mode,
            "common_provided_state": mode == "provided_consistent_state",
            "native_state_hash_valid": state_hash_valid if state is not None else None,
            "comparison": comparison,
        },
        blockers,
    )


def _compare_initial_state(
    model: AxleDynamicsModel,
    state: Mapping[str, object],
    adams_result: AdamsAxleResult,
) -> dict[str, object]:
    bodies = state.get("bodies")
    constraints = state.get("constraint_wrench_on_body_b")
    springs = state.get("springs")
    tires = state.get("tires")
    if not all(
        isinstance(value, Mapping)
        for value in (bodies, constraints, springs, tires)
    ):
        return {
            "performed": True,
            "passed": False,
            "reason": "native initial-state evidence has an incomplete layout",
            "body_position_max_abs_error_m": math.inf,
            "body_quaternion_max_angle_rad": math.inf,
            "body_velocity_max_abs_error_m_per_s": math.inf,
            "body_angular_velocity_max_abs_error_rad_per_s": math.inf,
            "constraint_wrench_max_abs_error": math.inf,
            "spring_output_max_abs_error_n": math.inf,
            "tire_output_max_abs_error": math.inf,
        }
    body_position = 0.0
    body_quaternion = 0.0
    body_velocity = 0.0
    body_omega = 0.0
    for index, body in enumerate(model.bodies):
        native = bodies.get(body.name)
        if not isinstance(native, Mapping):
            return _incomplete_initial_state(f"missing body {body.name!r}")
        values = _mapping_values(native, BODY_STATE_COLUMNS)
        actual = np.asarray(adams_result.states[0, index, :], dtype=float)
        body_position = max(body_position, float(np.max(np.abs(values[:3] - actual[:3]))))
        body_quaternion = max(
            body_quaternion,
            _quaternion_angle(values[3:7], actual[3:7]),
        )
        body_velocity = max(body_velocity, float(np.max(np.abs(values[7:10] - actual[7:10]))))
        body_omega = max(body_omega, float(np.max(np.abs(values[10:13] - actual[10:13]))))
    constraint_error, constraint_scale = _mapped_array_error(
        constraints,
        adams_result.constraint_names,
        adams_result.constraint_wrench[0],
        CONSTRAINT_WRENCH_COLUMNS,
    )
    spring_error, spring_scale = _mapped_array_error(
        springs,
        adams_result.spring_names,
        adams_result.spring_output[0],
        SPRING_OUTPUT_COLUMNS,
    )
    tire_error, tire_scale = _mapped_array_error(
        tires,
        adams_result.tire_names,
        adams_result.tire_output[0],
        TIRE_OUTPUT_COLUMNS,
    )
    passed = (
        body_position <= _STATE_TOLERANCE
        and body_quaternion <= _STATE_TOLERANCE
        and body_velocity <= _STATE_TOLERANCE
        and body_omega <= _STATE_TOLERANCE
        and constraint_error <= _element_tolerance(constraint_scale)
        and spring_error <= _element_tolerance(spring_scale)
        and tire_error <= _element_tolerance(tire_scale)
    )
    return {
        "performed": True,
        "passed": passed,
        "body_position_max_abs_error_m": body_position,
        "body_quaternion_max_angle_rad": body_quaternion,
        "body_velocity_max_abs_error_m_per_s": body_velocity,
        "body_angular_velocity_max_abs_error_rad_per_s": body_omega,
        "constraint_wrench_max_abs_error": constraint_error,
        "spring_output_max_abs_error_n": spring_error,
        "tire_output_max_abs_error": tire_error,
        "constraint_wrench_scale": constraint_scale,
        "spring_output_scale_n": spring_scale,
        "tire_output_scale": tire_scale,
        "state_tolerance": _STATE_TOLERANCE,
        "element_absolute_tolerance": _ELEMENT_ABSOLUTE_TOLERANCE,
        "element_relative_tolerance": _ELEMENT_RELATIVE_TOLERANCE,
    }


def _audit_output_conventions(
    model: AxleDynamicsModel,
    dataset: Mapping[str, object],
    *,
    native_history: TimeHistory | None,
    adams_history: TimeHistory | None,
    require_runtime: bool,
) -> tuple[dict[str, object], list[str]]:
    conventions = dataset.get("conventions")
    conventions_map = conventions if isinstance(conventions, Mapping) else {}
    blockers: list[str] = []
    units_match = model.units == "SI" and "SI" in str(conventions_map.get("units", ""))
    coordinates_match = model.coordinate_system == "vehicle_x_rear_y_right_z_up"
    if not units_match:
        blockers.append("model and Adams dataset units are not both SI")
    if not coordinates_match:
        blockers.append("model coordinate system is not the frozen vehicle frame")
    if conventions_map.get("damper_curve_interpolation") != (
        "piecewise linear with constant extrapolation beyond the measured "
        "velocity endpoints, matching the native curve evaluator"
    ):
        blockers.append("damper curve interpolation convention is not frozen")
    if conventions_map.get("part_pose_reconstruction") != (
        "canonical body pose, velocity, and acceleration are read from explicit "
        "CM-marker VARIABLE expressions in the ground frame; no PART_XFORM "
        "reconstruction is applied"
    ):
        blockers.append("Adams CM body state convention is not frozen")
    if conventions_map.get("initial_state_canonicalization") != (
        "at the common t=0 sample, the canonical body quaternion is taken from "
        "the shared manifest to remove finite-precision Euler round-trip error; "
        "all later samples are read from Adams CM variables"
    ):
        blockers.append("Adams initial quaternion canonicalization is not frozen")
    if conventions_map.get("fixture_wrench_reconstruction") != (
        FIXTURE_WRENCH_CONVENTION
    ):
        blockers.append("fixture wrench reconstruction convention is not frozen")
    return (
        {
            "passed": not blockers,
            "model_units": model.units,
            "model_coordinate_system": model.coordinate_system,
            "dataset_units": conventions_map.get("units"),
            "dataset_coordinate_convention": conventions_map.get("part_reference_frame"),
            "damper_curve_interpolation": conventions_map.get(
                "damper_curve_interpolation"
            ),
            "part_pose_reconstruction": conventions_map.get(
                "part_pose_reconstruction"
            ),
            "initial_state_canonicalization": conventions_map.get(
                "initial_state_canonicalization"
            ),
            "fixture_wrench_reconstruction": conventions_map.get(
                "fixture_wrench_reconstruction"
            ),
            "native_history_units_present": (
                native_history.units is not None if native_history is not None else None
            ),
            "adams_history_units_present": (
                adams_history.units is not None if adams_history is not None else None
            ),
            "runtime_output_conventions_checked": require_runtime,
        },
        blockers,
    )


def _source_provenance(
    manifest: DynamicAxleManifest,
    source_provenance: Mapping[str, object] | None,
) -> dict[str, object]:
    metadata = manifest.payload.get("case_metadata")
    values: dict[str, object] = {
        **(
            dict(cast(Mapping[str, object], metadata))
            if isinstance(metadata, Mapping)
            else {}
        ),
        **(dict(source_provenance) if source_provenance is not None else {}),
    }
    declared = bool(values.get("parameter_provenance")) and bool(
        values.get("source_subsystem")
    )
    return {
        "status": "declared" if declared else "unknown",
        "parameter_provenance": values.get("parameter_provenance"),
        "source_subsystem": values.get("source_subsystem"),
        "is_equivalence_gate": False,
    }


def _mapped_array_error(
    native: Mapping[str, object],
    names: Sequence[str],
    actual: np.ndarray,
    columns: Sequence[str],
) -> tuple[float, float]:
    errors: list[float] = []
    scales: list[float] = []
    for index, name in enumerate(names):
        values = native.get(name)
        if isinstance(values, Mapping):
            expected = _mapping_values(values, columns)
        elif isinstance(values, Sequence) and not isinstance(values, (str, bytes)):
            try:
                expected = np.asarray([float(value) for value in values], dtype=float)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"initial-state evidence has invalid {name!r}") from exc
            if expected.size != len(columns):
                return math.inf, math.inf
        else:
            return math.inf, math.inf
        errors.append(float(np.max(np.abs(expected - actual[index]))))
        scales.append(float(np.max(np.abs(np.concatenate((expected, actual[index]))))))
    return max(errors, default=0.0), max(scales, default=0.0)


def _mapping_values(
    values: Mapping[str, object],
    columns: Sequence[str],
) -> np.ndarray:
    try:
        return np.asarray([float(values[column]) for column in columns], dtype=float)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"initial-state evidence is missing {columns}") from exc


def _incomplete_initial_state(reason: str) -> dict[str, object]:
    return {
        "performed": True,
        "passed": False,
        "reason": reason,
        "body_position_max_abs_error_m": math.inf,
        "body_quaternion_max_angle_rad": math.inf,
        "body_velocity_max_abs_error_m_per_s": math.inf,
        "body_angular_velocity_max_abs_error_rad_per_s": math.inf,
        "constraint_wrench_max_abs_error": math.inf,
        "spring_output_max_abs_error_n": math.inf,
        "tire_output_max_abs_error": math.inf,
    }


def _element_tolerance(max_error: float) -> float:
    return _ELEMENT_ABSOLUTE_TOLERANCE + _ELEMENT_RELATIVE_TOLERANCE * max(
        1.0, abs(max_error)
    )


def _quaternion_angle(left: Sequence[float], right: Sequence[float]) -> float:
    a = np.asarray(left, dtype=float)
    b = np.asarray(right, dtype=float)
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    dot = min(1.0, max(-1.0, abs(float(np.dot(a, b)))))
    return 2.0 * math.acos(dot)


def _max_difference(left: Sequence[float], right: Sequence[float]) -> float:
    if len(left) != len(right):
        return math.inf
    return float(np.max(np.abs(np.asarray(left, dtype=float) - right)))


def _bushing_has_force_terms(bushing: Any) -> bool:
    return bool(
        np.any(np.asarray(bushing.stiffness, dtype=float) != 0.0)
        or np.any(np.asarray(bushing.damping, dtype=float) != 0.0)
        or np.any(np.asarray(bushing.preload_in_frame_a_n_n_m, dtype=float) != 0.0)
    )


def _number(value: float) -> str:
    return f"{float(value):.12g}"


def _unique(values: Sequence[str]) -> list[str]:
    return list(dict.fromkeys(values))
