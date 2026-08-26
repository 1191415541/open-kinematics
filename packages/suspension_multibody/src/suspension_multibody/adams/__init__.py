"""Adams/Car 2024.1 discovery and batch-validation adapter."""

from pathlib import Path
from typing import Mapping

from .adapter import AdamsBatchAdapter, Runner, RunnerExecution, SmokeResult, Tolerance
from .axle_adams_model import (
    AXLE_ADAMS_DATASET_CONTRACT,
    AXLE_ADAMS_GENERATOR,
    MAXIMUM_INPUT_BREAKPOINTS,
    AxleAdamsDataset,
    AxleAdamsExpressibilityError,
    axle_adams_blockers,
    build_axle_adams_dataset,
    write_axle_adams_dataset,
)
from .axle_channels import axle_history_from_result
from .axle_contract import (
    AXLE_MANIFEST_CONTRACT,
    AXLE_MANIFEST_GENERATOR,
    AxleChannelBindings,
    AxleMarkerBinding,
    DynamicAxleManifest,
    DynamicAxleManifestSettings,
    create_dynamic_axle_manifest,
    load_axle_acceptance_contract,
    load_axle_channel_contract,
    load_dynamic_axle_manifest_settings,
    read_dynamic_axle_manifest,
    validate_axle_channel_bindings,
    write_dynamic_axle_manifest,
)
from .axle_dynamic_history import (
    ADAMS_AXLE_HISTORY_CONTRACT,
    AdamsAxleResult,
    adams_axle_history_from_result,
    adams_axle_raw_channel_map,
    adams_axle_result_from_result,
)
from .axle_equivalence import (
    AXLE_COMPARISON_CONTRACT,
    AXLE_EVIDENCE_CONTRACT,
    AxleContactEvent,
    AxleEvidenceBundle,
    AxleInitializationEvidence,
    audit_axle_time_convergence,
    compare_axle_evidence,
    compare_strict_axle_histories,
    initialization_evidence_from_result,
    read_axle_evidence_bundle,
    run_native_axle_manifest,
    write_axle_evidence_bundle,
)
from .axle_equivalence_audit import (
    ADAMS_AXLE_EQUIVALENCE_AUDIT_CONTRACT,
    audit_axle_equivalence,
)
from .full_vehicle_correlation import full_vehicle_time_history
from .full_vehicle_mbd_comparison import (
    FULL_VEHICLE_HANDLING_CASES,
    FULL_VEHICLE_MBD_COMPARISON_CONTRACT,
    FULL_VEHICLE_PAIRING_FIELDS,
    FullVehiclePairingAudit,
    audit_full_vehicle_pairing,
    compare_full_vehicle_mbd_case,
    handling_case_names,
    write_full_vehicle_mbd_report,
)
from .full_vehicle_model import (
    AdamsFullVehicleInput,
    AdamsPartData,
    build_adams_vehicle_case,
    build_adams_vehicle_model,
    load_adams_full_vehicle_input,
    steering_signal_from_manifest,
)
from .probe import AdamsProfile, discover_profile, probe_profile
from .strict_c import validate_strict_c
from .strict_k import validate_strict_k
from .time_domain import (
    AdamsResultChannel,
    TimeHistory,
    TimeHistoryTolerance,
    compare_time_histories,
    history_from_dynamic_bundle,
    parse_adams_result_history,
    read_time_history,
    write_time_history,
)
from .time_domain_gate import (
    AXLE_RESPONSE_CHANNELS,
    AdamsTimeDomainAdapter,
    TimeDomainGateResult,
    TimeDomainRunner,
    command_time_domain_runner,
    validate_axle_time_domain,
)
from .vehicle_acceptance import (
    HANDLING_CASES,
    RIDE_CASES,
    EngineeringTolerance,
    VehicleAcceptanceCase,
    default_vehicle_acceptance_matrix,
    validate_vehicle_acceptance_matrix,
)
from .vehicle_handling import (
    HANDLING_ADAMS_CHANNELS,
    HandlingExecutionResult,
    run_adams_car_handling_case,
    validate_handling_execution,
)
from .vehicle_kc_time_domain import (
    VEHICLE_KC_CHANNELS,
    run_vehicle_kc_roll_adams,
    validate_vehicle_kc_time_domain,
)
from .vehicle_reference import (
    HANDLING_REFERENCE_CHANNELS,
    REFERENCE_BUNDLE_CONTRACT,
    RIDE_REFERENCE_CHANNELS,
    VehicleReferenceBundle,
    canonicalize_vehicle_history,
    read_vehicle_reference_bundle,
    write_vehicle_reference_bundle,
)
from .vehicle_ride import (
    RIDE_ADAMS_CHANNELS,
    RideExecutionResult,
    run_adams_car_ride_case,
    validate_ride_execution,
)

__all__ = [
    "AdamsBatchAdapter",
    "AdamsProfile",
    "AdamsResultChannel",
    "AdamsTimeDomainAdapter",
    "AXLE_ADAMS_DATASET_CONTRACT",
    "AXLE_ADAMS_GENERATOR",
    "AXLE_COMPARISON_CONTRACT",
    "AXLE_EVIDENCE_CONTRACT",
    "AXLE_MANIFEST_CONTRACT",
    "AXLE_MANIFEST_GENERATOR",
    "AXLE_RESPONSE_CHANNELS",
    "ADAMS_AXLE_HISTORY_CONTRACT",
    "ADAMS_AXLE_EQUIVALENCE_AUDIT_CONTRACT",
    "AxleAdamsDataset",
    "AdamsAxleResult",
    "AxleAdamsExpressibilityError",
    "AxleChannelBindings",
    "AxleContactEvent",
    "AxleEvidenceBundle",
    "AxleInitializationEvidence",
    "AxleMarkerBinding",
    "DynamicAxleManifest",
    "DynamicAxleManifestSettings",
    "EngineeringTolerance",
    "HANDLING_CASES",
    "HANDLING_ADAMS_CHANNELS",
    "HandlingExecutionResult",
    "HANDLING_REFERENCE_CHANNELS",
    "REFERENCE_BUNDLE_CONTRACT",
    "RIDE_CASES",
    "RIDE_ADAMS_CHANNELS",
    "RideExecutionResult",
    "RIDE_REFERENCE_CHANNELS",
    "Runner",
    "RunnerExecution",
    "SmokeResult",
    "TimeHistory",
    "TimeHistoryTolerance",
    "TimeDomainGateResult",
    "TimeDomainRunner",
    "Tolerance",
    "VehicleAcceptanceCase",
    "VehicleReferenceBundle",
    "VEHICLE_KC_CHANNELS",
    "MAXIMUM_INPUT_BREAKPOINTS",
    "axle_adams_blockers",
    "build_axle_adams_dataset",
    "write_axle_adams_dataset",
    "compare_time_histories",
    "compare_axle_evidence",
    "compare_strict_axle_histories",
    "canonicalize_vehicle_history",
    "command_time_domain_runner",
    "create_dynamic_axle_manifest",
    "discover_profile",
    "default_vehicle_acceptance_matrix",
    "history_from_dynamic_bundle",
    "initialization_evidence_from_result",
    "axle_history_from_result",
    "adams_axle_history_from_result",
    "adams_axle_result_from_result",
    "adams_axle_raw_channel_map",
    "audit_axle_equivalence",
    "audit_axle_time_convergence",
    "load_axle_acceptance_contract",
    "load_axle_channel_contract",
    "load_dynamic_axle_manifest_settings",
    "parse_adams_result_history",
    "probe_profile",
    "read_time_history",
    "read_axle_evidence_bundle",
    "read_dynamic_axle_manifest",
    "run_native_axle_manifest",
    "read_vehicle_reference_bundle",
    "run_vehicle_kc_roll_adams",
    "run_adams_car_handling_case",
    "run_adams_car_ride_case",
    "validate_strict_c",
    "validate_strict_k",
    "validate_axle_time_domain",
    "validate_axle_channel_bindings",
    "validate_vehicle_acceptance_matrix",
    "validate_vehicle_kc_time_domain",
    "validate_handling_execution",
    "validate_ride_execution",
    "full_vehicle_time_history",
    "AdamsFullVehicleInput",
    "AdamsPartData",
    "build_adams_vehicle_case",
    "build_adams_vehicle_model",
    "load_adams_full_vehicle_input",
    "steering_signal_from_manifest",
    "FULL_VEHICLE_HANDLING_CASES",
    "FULL_VEHICLE_MBD_COMPARISON_CONTRACT",
    "FULL_VEHICLE_PAIRING_FIELDS",
    "FullVehiclePairingAudit",
    "audit_full_vehicle_pairing",
    "compare_full_vehicle_mbd_case",
    "handling_case_names",
    "write_full_vehicle_mbd_report",
    "write_vehicle_reference_bundle",
    "write_axle_evidence_bundle",
    "write_dynamic_axle_manifest",
    "write_time_history",
]


def validate_profile(
    profile: str,
    *,
    smoke: bool = False,
    full: bool = False,
    require_installed: bool = False,
    reference: str | Path | Mapping[str, object] | None = None,
    runner: Runner | None = None,
    strict_k: bool = False,
    strict_c: bool = False,
    evidence_dir: str | Path | None = None,
) -> SmokeResult:
    """Probe a named Adams profile and optionally execute a validation gate."""
    if sum((smoke, full, strict_k, strict_c)) > 1:
        return SmokeResult(
            ok=False,
            message="--smoke, --full, --strict-k and --strict-c are mutually exclusive",
        )
    result = probe_profile(profile)
    if not result.available:
        message = result.message
        if require_installed:
            return SmokeResult(ok=False, message=message, profile=result)
        return SmokeResult(ok=True, message=f"{message}; skipped", profile=result)
    if smoke:
        smoke_result = AdamsBatchAdapter(result).smoke()
        return SmokeResult(
            ok=smoke_result.ok, message=smoke_result.message, profile=result
        )
    if strict_k:
        return validate_strict_k(result, evidence_dir=evidence_dir)
    if strict_c:
        return validate_strict_c(result, evidence_dir=evidence_dir)
    if full:
        if reference is None:
            from .reference import build_default_reference

            reference = build_default_reference(result)
        if runner is None:
            from .runner import run_default_adams

            runner = run_default_adams
        full_result = AdamsBatchAdapter(
            result,
            runner=runner,
            force_absolute_tolerance=200.0,
            compliance_absolute_tolerance=0.001,
        ).full(reference=reference)
        return SmokeResult(
            ok=full_result.ok,
            message=full_result.message,
            profile=result,
            output_path=full_result.output_path,
            report=full_result.report,
        )
    return SmokeResult(ok=True, message=result.message, profile=result)
