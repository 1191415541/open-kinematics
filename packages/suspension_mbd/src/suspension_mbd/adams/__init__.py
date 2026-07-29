"""Adams/Car 2024.1 discovery and batch-validation adapter."""

from pathlib import Path
from typing import Mapping

from .adapter import AdamsBatchAdapter, Runner, RunnerExecution, SmokeResult, Tolerance
from .probe import AdamsProfile, discover_profile, probe_profile

__all__ = [
    "AdamsBatchAdapter",
    "AdamsProfile",
    "Runner",
    "RunnerExecution",
    "SmokeResult",
    "Tolerance",
    "discover_profile",
    "probe_profile",
]


def validate_profile(
    profile: str,
    *,
    smoke: bool = False,
    full: bool = False,
    require_installed: bool = False,
    reference: str | Path | Mapping[str, object] | None = None,
    runner: Runner | None = None,
) -> SmokeResult:
    """Probe a named Adams profile and optionally execute a validation gate."""
    result = probe_profile(profile)
    if not result.available:
        message = result.message
        if require_installed:
            return SmokeResult(ok=False, message=message, profile=result)
        return SmokeResult(ok=True, message=f"{message}; skipped", profile=result)
    if smoke:
        smoke_result = AdamsBatchAdapter(result).smoke()
        return SmokeResult(ok=smoke_result.ok, message=smoke_result.message, profile=result)
    if full:
        full_result = AdamsBatchAdapter(result, runner=runner).full(reference=reference)
        return SmokeResult(
            ok=full_result.ok,
            message=full_result.message,
            profile=result,
            output_path=full_result.output_path,
        )
    return SmokeResult(ok=True, message=result.message, profile=result)
