"""Guarded comparison of the true full-vehicle multibody solver with Adams."""

from __future__ import annotations

import hashlib
import json
import math
import re
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, cast

from ..schema import VehicleDynamicCase
from .full_vehicle_correlation import full_vehicle_time_history
from .full_vehicle_model import _direct_wheel_torque_signal_hash
from .time_domain import TimeHistory, TimeHistoryTolerance, compare_time_histories
from .vehicle_reference import VehicleReferenceBundle

FULL_VEHICLE_MBD_COMPARISON_CONTRACT = "full-vehicle-mbd-adams-comparison-v1"
FullVehicleComparisonStatus = Literal["READY", "BLOCKED"]

FULL_VEHICLE_HANDLING_CASES = (
    "steady_state_circle",
    "step_steer",
    "sine_steer",
    "double_lane_change",
)

# These are deliberately explicit. A 14/15-DOF parameter bundle is not enough
# to reconstruct the bodies, joints, contacts, and actuator state of a VehicleModel.
FULL_VEHICLE_PAIRING_FIELDS = (
    "adams_assembly_hash",
    "chassis_mass_com_inertia_hash",
    "suspension_geometry_and_joint_hash",
    "corner_suspension_parameters_hash",
    "wheel_mass_inertia_pose_hash",
    "pac2002_parameter_hash",
    "steering_input_mapping",
    "static_equilibrium_state_hash",
    "initial_forward_speed_mps",
    "initial_wheel_speeds_rad_s",
    "brake_drive_input_contract",
    "source_drive_brake_input_contract",
)

_REFERENCE_TIRE_MODELS = {
    "adams_builtin_pac2002": "exact_pac2002",
    "adams_generated_brush": "exact_native_brush",
}


@dataclass(frozen=True)
class FullVehiclePairingAudit:
    """Auditable result of the model and initial-condition pairing gate."""

    status: FullVehicleComparisonStatus
    case: str
    verified_fields: tuple[str, ...]
    missing_or_mismatched_fields: tuple[str, ...]
    notes: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        return {
            "contract": FULL_VEHICLE_MBD_COMPARISON_CONTRACT,
            "status": self.status,
            "case": self.case,
            "verified_fields": list(self.verified_fields),
            "missing_or_mismatched_fields": list(self.missing_or_mismatched_fields),
            "notes": list(self.notes),
        }


def audit_full_vehicle_pairing(
    reference: VehicleReferenceBundle,
    case: VehicleDynamicCase,
    *,
    vehicle_manifest: Mapping[str, object] | None = None,
) -> FullVehiclePairingAudit:
    """
    Check that an Adams reference can be compared to one full MBD case.

    The gate requires explicit initial-state and model-source fields.  A
    numerical comparison is never enabled by a simplified 14/15-DOF manifest.
    """
    verified: list[str] = []
    missing: list[str] = []
    notes: list[str] = []
    adams = reference.input_manifest

    if reference.category != "handling_stability":
        missing.append("reference_category_handling_stability")
    else:
        verified.append("reference_category_handling_stability")
    if reference.case != case.name:
        missing.append("case_name")
    else:
        verified.append("case_name")
    if adams.get("analysis_mode") != "full_vehicle_sdi_dynamic":
        missing.append("adams_analysis_mode_full_vehicle_sdi_dynamic")
    else:
        verified.append("adams_analysis_mode_full_vehicle_sdi_dynamic")
    reference_tire_model = adams.get("tire_model")
    expected_native_tire = (
        _REFERENCE_TIRE_MODELS.get(reference_tire_model)
        if isinstance(reference_tire_model, str)
        else None
    )
    if expected_native_tire is None:
        missing.append("supported_adams_tire_model")
    elif reference_tire_model == "adams_builtin_pac2002":
        verified.append("adams_builtin_pac2002")
    else:
        reference_kind = adams.get("reference_kind")
        if reference_kind != "independent_adams_car_native_brush":
            missing.append("independent_adams_native_brush_reference")
            notes.append(
                "native_brush requires an independently generated Adams/Car reference bundle; the PAC2002 bundle cannot be reused"
            )
        else:
            verified.append("independent_adams_native_brush_reference")
    raw_names = tuple(reference.raw_artifacts)
    if any(name.endswith(".adm") for name in raw_names) and any(
        name.endswith(".asy") for name in raw_names
    ):
        verified.append("adams_raw_compiled_model_evidence")
        notes.append(
            "the raw Adams .adm/.asy evidence is present; source-equivalence gates are still evaluated before comparison"
        )
    else:
        missing.append("adams_raw_compiled_model_evidence")

    if vehicle_manifest is None:
        missing.extend(FULL_VEHICLE_PAIRING_FIELDS)
        notes.append("no full-vehicle manifest was supplied")
    else:
        validated_fields = {
            "adams_assembly_hash",
            "steering_input_mapping",
            "static_equilibrium_state_hash",
            "initial_forward_speed_mps",
            "initial_wheel_speeds_rad_s",
            "brake_drive_input_contract",
            "source_drive_brake_input_contract",
        }
        for field in FULL_VEHICLE_PAIRING_FIELDS:
            if field not in vehicle_manifest or vehicle_manifest[field] is None:
                missing.append(field)
            elif field not in validated_fields:
                verified.append(field)
        expected_assembly = adams.get("assembly")
        supplied_assembly = vehicle_manifest.get("adams_assembly")
        if supplied_assembly != expected_assembly:
            missing.append("adams_assembly")
        else:
            verified.append("adams_assembly")
        supplied_tire = vehicle_manifest.get("tire_model")
        if supplied_tire != adams.get("tire_model"):
            missing.append("tire_model")
        else:
            verified.append("tire_model")
        supplied_hash = vehicle_manifest.get("adams_assembly_hash")
        reference_hash = _raw_artifact_hash(reference.raw_artifacts, supplied_assembly)
        if (
            isinstance(supplied_hash, str)
            and _is_sha256(supplied_hash)
            and reference_hash is not None
            and supplied_hash.lower() == reference_hash.lower()
        ):
            verified.append("adams_assembly_hash")
        else:
            missing.append("adams_assembly_hash")

        supplied_speed = vehicle_manifest.get("initial_forward_speed_mps")
        if _close_float(supplied_speed, case.initial_forward_speed_mps):
            verified.append("initial_forward_speed_mps")
        else:
            missing.append("initial_forward_speed_mps")

        supplied_wheels = vehicle_manifest.get("initial_wheel_speeds_rad_s")
        expected_wheels = dict(case.initial_wheel_speeds)
        if (
            isinstance(supplied_wheels, Mapping)
            and set(cast(Mapping[str, object], supplied_wheels)) == set(expected_wheels)
            and all(
                _close_float(cast(Mapping[str, object], supplied_wheels)[name], value)
                for name, value in expected_wheels.items()
            )
        ):
            verified.append("initial_wheel_speeds_rad_s")
        else:
            missing.append("initial_wheel_speeds_rad_s")

        supplied_mapping = vehicle_manifest.get("steering_input_mapping")
        steering_mapping = (
            cast(Mapping[str, object], supplied_mapping)
            if isinstance(supplied_mapping, Mapping)
            else {}
        )
        expected_scale = case.vehicle.steering.rack_displacement_per_steering_wheel_angle
        if (
            steering_mapping.get("input") == case.vehicle.steering.input
            and _close_float(steering_mapping.get("ratio"), case.vehicle.steering.ratio)
            and steering_mapping.get("actuator_mode", "rack_translation")
            == case.vehicle.steering.actuator_mode
            and (
                case.vehicle.steering.input != "steering_wheel_angle"
                or case.vehicle.steering.actuator_mode == "prescribed_rotation"
                or (
                    expected_scale is not None
                    and _close_float(
                        steering_mapping.get("rack_displacement_per_steering_wheel_angle"),
                        expected_scale,
                    )
                )
            )
        ):
            verified.append("steering_input_mapping")
        else:
            missing.append("steering_input_mapping")

        contract = vehicle_manifest.get("brake_drive_input_contract")
        input_contract = (
            cast(Mapping[str, object], contract)
            if isinstance(contract, Mapping)
            else {}
        )
        if (
            input_contract.get("brake") == "zero"
            and input_contract.get("drive") == "zero"
            and _signal_is_zero(case.brake_input)
            and _signal_is_zero(case.drive_input)
        ):
            verified.append("brake_drive_input_contract")
        else:
            missing.append("brake_drive_input_contract")

        static_payload = adams.get("initial_state")
        static_state = (
            cast(Mapping[str, object], static_payload)
            if isinstance(static_payload, Mapping)
            else {}
        )
        if (
            static_state.get("adams") == "static_equilibrium"
            and static_state.get("package_relative_coordinates") == "zero"
            and vehicle_manifest.get("static_equilibrium_state_hash") == _payload_hash(
                {
                    "adams": "static_equilibrium",
                    "package_relative_coordinates": "zero",
                }
            )
        ):
            verified.append("static_equilibrium_state_hash")
        else:
            missing.append("static_equilibrium_state_hash")
        _audit_source_equivalence(
            vehicle_manifest, verified, missing, notes, case=case
        )

    if case.static_equilibrium:
        verified.append("solver_static_equilibrium_state")
    elif _has_complete_initial_state(case):
        verified.append("solver_provided_consistent_state")
        notes.append(
            "the native solver uses a complete source-provided initial state instead of re-trimming it"
        )
    else:
        missing.append("solver_static_equilibrium_or_complete_initial_state")
    if case.initial_forward_speed_mps <= 0.0:
        missing.append("solver_initial_forward_velocity")
    else:
        verified.append("solver_initial_forward_velocity")
    required_wheels = {"front_left", "front_right", "rear_left", "rear_right"}
    supplied_wheels = {name for name, _ in case.initial_wheel_speeds}
    if supplied_wheels != required_wheels:
        missing.append("solver_initial_wheel_speeds")
    else:
        verified.append("solver_initial_wheel_speeds")
    notes.append("full-vehicle solver and Adams history are compared on the reference time grid")
    unique_missing = tuple(dict.fromkeys(missing))
    return FullVehiclePairingAudit(
        status="READY" if not unique_missing else "BLOCKED",
        case=case.name,
        verified_fields=tuple(dict.fromkeys(verified)),
        missing_or_mismatched_fields=unique_missing,
        notes=tuple(notes),
    )


def _has_complete_initial_state(case: VehicleDynamicCase) -> bool:
    """检查每个运行时刚体是否都有一个明确的初始状态."""
    expected = {case.vehicle.chassis.name}
    for prefix, axle in (
        ("front_", case.vehicle.front_axle),
        ("rear_", case.vehicle.rear_axle),
    ):
        expected.update(
            f"{prefix}{body.name}"
            for body in axle.bodies
            if body.name != "chassis"
        )
    expected.update(
        wheel.body
        for wheel in case.vehicle.wheels
        if wheel.mount_joint_kind != "fixed"
    )
    return {state.body for state in case.initial_states} == expected


def compare_full_vehicle_mbd_case(
    reference: VehicleReferenceBundle,
    case: VehicleDynamicCase,
    *,
    run: Any | None = None,
    vehicle_manifest: Mapping[str, object] | None = None,
    tolerances: Mapping[str, TimeHistoryTolerance] | None = None,
) -> dict[str, object]:
    """Compare one full MBD run, or return a blocking report before comparison."""
    audit = audit_full_vehicle_pairing(
        reference,
        case,
        vehicle_manifest=vehicle_manifest,
    )
    report: dict[str, object] = {
        "contract": FULL_VEHICLE_MBD_COMPARISON_CONTRACT,
        "status": audit.status,
        "case": reference.case,
        "pairing": audit.as_dict(),
        "adams_input_manifest_hash": reference.input_manifest_hash,
        "adams_producer": dict(reference.producer),
    }
    if audit.status == "BLOCKED":
        return report
    if run is None:
        raise ValueError("a full-vehicle run is required for a READY comparison")
    if tolerances is None:
        raise ValueError("tolerances are required for a READY comparison")
    length_scale = (
        1.0e-3 if case.vehicle.units.value == "engineering" else 1.0
    )
    steering_ratio = (
        case.vehicle.steering.rack_displacement_per_steering_wheel_angle
        or case.vehicle.steering.ratio
    )
    actual = _canonical_solver_history(
        full_vehicle_time_history(
            run,
            "handling_stability",
            steering_ratio_m_per_rad=steering_ratio * length_scale,
            chassis_center_of_mass_m=tuple(
                value * length_scale
                for value in case.vehicle.chassis.center_of_mass.as_tuple()
            ),
        )
    )
    reference_history = _apply_response_transform(
        reference.history, reference.input_manifest
    )
    actual = _apply_response_transform(actual, reference.input_manifest)
    report["comparison"] = compare_time_histories(
        reference_history,
        actual,
        tolerances,
    )
    report["status"] = "READY"
    return report


def write_full_vehicle_mbd_report(
    report: Mapping[str, object],
    path: str | Path,
) -> Path:
    """Write a deterministic JSON report for CI and acceptance evidence."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(dict(report), indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return destination


def _canonical_solver_history(history: TimeHistory) -> TimeHistory:
    factors = {
        "steering_angle": ("rad", "rad", 1.0),
        "lateral_acceleration": ("mm/s^2", "m/s^2", 1e-3),
        "yaw_rate": ("rad/s", "rad/s", 1.0),
        "body_roll": ("rad", "rad", 1.0),
    }
    units = history.units or {}
    channels: dict[str, tuple[float, ...]] = {}
    canonical_units: dict[str, str] = {}
    for name, values in history.channels.items():
        source, target, factor = factors[name]
        if units.get(name) != source:
            raise ValueError(
                f"full-vehicle solver channel {name!r} has unit {units.get(name)!r}; "
                f"expected {source!r}"
            )
        channels[name] = tuple(value * factor for value in values)
        canonical_units[name] = target
    return TimeHistory(
        time=history.time,
        channels=channels,
        units=canonical_units,
    )


def _apply_response_transform(
    history: TimeHistory, manifest: Mapping[str, object]
) -> TimeHistory:
    """Apply only transforms explicitly recorded by the Adams input case."""
    transform = manifest.get("response_transform")
    if not isinstance(transform, Mapping):
        return history
    channels = dict(history.channels)
    if transform.get("body_roll") == "subtract_initial_sample":
        values = channels.get("body_roll")
        if values:
            initial = values[0]
            channels["body_roll"] = tuple(value - initial for value in values)
    return TimeHistory(time=history.time, channels=channels, units=history.units)


def handling_case_names() -> tuple[str, ...]:
    """Return the fixed handling cases used by the Adams acceptance matrix."""
    return FULL_VEHICLE_HANDLING_CASES


def _close_float(value: object, expected: float, tolerance: float = 1e-9) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value)) and math.isclose(
        float(value), expected, rel_tol=tolerance, abs_tol=tolerance
    )


def _signal_is_zero(signal: object) -> bool:
    constant = getattr(signal, "constant", None)
    if constant is not None:
        return _close_float(constant, 0.0)
    values = getattr(signal, "values", ())
    return bool(values) and all(_close_float(value, 0.0) for value in values)


def _audit_source_equivalence(
    vehicle_manifest: Mapping[str, object],
    verified: list[str],
    missing: list[str],
    notes: list[str],
    *,
    case: VehicleDynamicCase | None = None,
) -> None:
    """Reject a numerical claim when imported Adams behavior is reduced."""
    normalization = vehicle_manifest.get("unit_normalization")
    if not isinstance(normalization, Mapping) or normalization.get("status") != "complete":
        missing.append("adams_unit_conversion")
        notes.append(
            "the Adams source-unit declarations are incomplete, so no cross-unit numerical comparison is allowed"
        )

    unsupported = vehicle_manifest.get("unsupported_adams_user_functions")
    if unsupported is None:
        missing.append("unsupported_adams_user_functions")
    elif unsupported:
        missing.append("unsupported_adams_user_functions")
        notes.append(
            "solver-active Adams USER() laws are present but are not all implemented by the native vehicle solver"
        )

    source_topology = vehicle_manifest.get("adams_source_topology")
    topology = (
        cast(Mapping[str, object], source_topology)
        if isinstance(source_topology, Mapping)
        else {}
    )
    if topology.get("unresolved_coupler_ids"):
        missing.append("exact_adams_coupler_mapping")
        notes.append(
            "Adams joint couplers are present but the native vehicle topology does not yet represent them"
        )

    force_mapping = vehicle_manifest.get("adams_force_law_mapping")
    if not isinstance(force_mapping, Mapping):
        missing.append("exact_adams_force_law_mapping")
    elif _contains_non_exact_marker(force_mapping):
        missing.append("exact_adams_force_law_mapping")
        notes.append(
            "at least one Adams force-law mapping is marked unsupported, approximate, or otherwise reduced"
        )

    reduction = vehicle_manifest.get("adams_model_reduction")
    mass_treatment = (
        reduction.get("mass_treatment")
        if isinstance(reduction, Mapping)
        else None
    )
    if not isinstance(reduction, Mapping):
        missing.append("complete_adams_body_mapping")
    elif (
        reduction.get("status") != "exact_part_mapping"
        or mass_treatment
        not in {"exact", "exact_with_fixed_wheel_mass_condensation"}
        or reduction.get("omitted_part_ids")
    ):
        missing.append("complete_adams_body_mapping")
        notes.append(
            "the native vehicle model does not yet contain an exact mapping for every Adams mass-bearing part"
        )
    if not isinstance(reduction, Mapping) or (
        reduction.get("steering_internal_treatment") != "exact_source_topology"
    ):
        missing.append("source_steering_topology_equivalence")
        notes.append(
            "the native direct-rack input omits source steering inertias, couplers, and torsion-bar compliance"
        )

    tire_model = vehicle_manifest.get("tire_model", "adams_builtin_pac2002")
    expected_tire_implementation = (
        _REFERENCE_TIRE_MODELS.get(tire_model)
        if isinstance(tire_model, str)
        else None
    )
    if expected_tire_implementation is None:
        missing.append("supported_native_tire_model")
    elif vehicle_manifest.get("native_tire_implementation") != expected_tire_implementation:
        missing.append(
            "exact_native_pac2002_tire"
            if tire_model == "adams_builtin_pac2002"
            else "exact_native_brush_tire"
        )
        notes.append(
            "the native vehicle path does not provide the exact tire implementation selected by the paired reference"
        )

    source_contract = vehicle_manifest.get("source_drive_brake_input_contract")
    if not isinstance(source_contract, Mapping):
        missing.append("source_drive_brake_input_contract")
        notes.append(
            "the Adams source drive/brake force inventory is missing, so zero native inputs cannot be declared equivalent"
        )
    else:
        source_channels = source_contract.get("source")
        native_mapping = source_contract.get("native_mapping")
        source_channels = (
            cast(Mapping[str, object], source_channels)
            if isinstance(source_channels, Mapping)
            else {}
        )
        native_mapping = (
            cast(Mapping[str, object], native_mapping)
            if isinstance(native_mapping, Mapping)
            else {}
        )
        source_drive = source_channels.get("drive")
        source_brake = source_channels.get("brake")
        source_drive = (
            cast(Mapping[str, object], source_drive)
            if isinstance(source_drive, Mapping)
            else {}
        )
        source_brake = (
            cast(Mapping[str, object], source_brake)
            if isinstance(source_brake, Mapping)
            else {}
        )
        replay = source_contract.get("replay")
        direct_replay = (
            native_mapping.get("drive") == "direct_wheel_torque_replay"
            and native_mapping.get("brake") == "direct_wheel_torque_replay"
        )
        if direct_replay:
            replay_data = (
                cast(Mapping[str, object], replay)
                if isinstance(replay, Mapping)
                else {}
            )
            expected_hash = replay_data.get("signal_sha256")
            actual_hash = (
                _direct_wheel_torque_signal_hash(
                    dict(case.wheel_drive_torque),
                    dict(case.wheel_brake_torque),
                )
                if case is not None
                else None
            )
            expected_drive = replay_data.get("drive_wheels")
            expected_brake = replay_data.get("brake_wheels")
            actual_drive = (
                tuple(sorted(name for name, _ in case.wheel_drive_torque))
                if case is not None
                else ()
            )
            actual_brake = (
                tuple(sorted(name for name, _ in case.wheel_brake_torque))
                if case is not None
                else ()
            )
            if (
                isinstance(expected_hash, str)
                and _is_sha256(expected_hash)
                and expected_hash == actual_hash
                and tuple(expected_drive or ()) == actual_drive
                and tuple(expected_brake or ()) == actual_brake
            ):
                verified.append("source_drive_brake_input_equivalence")
                notes.append(
                    "native direct wheel torques are hash-matched to the recorded Adams result"
                )
            else:
                missing.append("source_drive_brake_input_equivalence")
                notes.append(
                    "the declared direct wheel-torque replay does not match the recorded Adams signal hash"
                )
        elif (
            source_drive.get("status") == "zero"
            and source_brake.get("status") == "zero"
            and native_mapping.get("drive") == "zero"
            and native_mapping.get("brake") == "zero"
        ):
            verified.append("source_drive_brake_input_contract")
        else:
            missing.append("source_drive_brake_input_equivalence")
            notes.append(
                "the Adams source contains state-dependent or nonzero drive/brake SFORCE laws that are not represented by native zero inputs"
            )


def _contains_non_exact_marker(value: object) -> bool:
    """Find explicit reduction markers in a nested source mapping."""
    markers = ("unsupported", "approx", "proxy", "partial", "omitted", "lump")
    if isinstance(value, str):
        lowered = value.lower()
        return any(marker in lowered for marker in markers)
    if isinstance(value, Mapping):
        return any(
            _contains_non_exact_marker(key) or _contains_non_exact_marker(item)
            for key, item in value.items()
        )
    if isinstance(value, (tuple, list, set)):
        return any(_contains_non_exact_marker(item) for item in value)
    return False


def _is_sha256(value: str) -> bool:
    return re.fullmatch(r"[0-9a-fA-F]{64}", value) is not None


def _raw_artifact_hash(raw_artifacts: Mapping[str, object], filename: object) -> str | None:
    if not isinstance(filename, str):
        return None
    target = filename.rsplit("/", 1)[-1].rsplit("\\", 1)[-1].lower()
    for path, digest in raw_artifacts.items():
        if str(path).replace("\\", "/").rsplit("/", 1)[-1].lower() == target:
            return str(digest)
    return None


def _payload_hash(payload: Mapping[str, object]) -> str:
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
