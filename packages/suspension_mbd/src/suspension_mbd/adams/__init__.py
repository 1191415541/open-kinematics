"""Adams/Car 2024.1 discovery and batch-validation adapter."""

from pathlib import Path
from typing import Mapping

from .adapter import AdamsBatchAdapter, Runner, RunnerExecution, SmokeResult, Tolerance
from .probe import AdamsProfile, discover_profile, probe_profile
from .strict_k import validate_strict_k

__all__ = [
    "AdamsBatchAdapter",
    "AdamsProfile",
    "Runner",
    "RunnerExecution",
    "SmokeResult",
    "Tolerance",
    "discover_profile",
    "probe_profile",
    "validate_strict_k",
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
    evidence_dir: str | Path | None = None,
) -> SmokeResult:
    """Probe a named Adams profile and optionally execute a validation gate."""
    if sum((smoke, full, strict_k)) > 1:
        return SmokeResult(
            ok=False,
            message="--smoke, --full and --strict-k are mutually exclusive",
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
