"""
Submit frozen physics fixtures through ordinary documents and project labels.

The reporting projection is test-only; the production run always returns the
same ResultEnvelope as every other subsystem assembly.
"""
from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace
from typing import Any

from suspension_multibody.api import simulate
from suspension_multibody.authoring.migration import migrate_v1_dynamic_axle
from suspension_multibody.axle_dynamics.errors import NativeAxleError
from suspension_multibody.axle_dynamics.result import AxleDynamicsResult
from suspension_multibody.kernel import KernelContractError
from suspension_multibody.report.metrics import compute_case_metrics
from suspension_multibody.results.raw import RawContractResult

from ._frozen_result_projection import build_result, safe_failure_row


class _TestRun(SimpleNamespace):
    def __getattr__(self, name):
        return getattr(self.result, name)

__all__ = ["solve_axle"]

ASSEMBLY = "axle"
FAMILY = "axle_dynamic"


def solve_axle(model: Any, case: Any):
    """
    Submit one SI axle model and case through the unified runner.

    The returned envelope carries the ``AxleDynamicsResult`` on ``.result`` with
    its metrics, exactly as the retired entry's return value did.  A native
    failure raises the ``NativeAxleError`` the tests assert on, with the partial
    result, the failure-diagnostics row, the failed sample index, the status and
    the failed time all recovered from the partial run -- every field the old
    translation filled.
    """
    try:
        assembly, declared_case = migrate_v1_dynamic_axle(model, case)
        envelope = simulate(assembly, declared_case)
    except KernelContractError as error:
        raise _as_native_error(error, model, case) from error
    graph = envelope.compiled.model_document
    aliases = {row["name"]: row["name"].split(".", 1)[-1] for row in graph["bodies"]}
    projected = _original_ids(envelope.raw, model, aliases)
    result = build_result(model, case, projected)
    if not isinstance(result, AxleDynamicsResult):
        raise TypeError(
            "the axle dynamic reading did not produce AxleDynamicsResult; got "
            f"{type(result).__name__}"
        )
    enriched = replace(result, metrics=compute_case_metrics(FAMILY, envelope.result))
    return _TestRun(compiled=envelope.compiled, raw=envelope.raw, result=enriched,
        envelope=envelope.result)


def _as_native_error(
    error: KernelContractError, model: Any, case: Any
) -> NativeAxleError:
    """Translate a kernel refusal into the axle error the tests assert on."""
    partial = error.partial_raw_result
    if not isinstance(partial, RawContractResult):
        return NativeAxleError(str(error), status=3)
    manifest = partial.document.get("manifest", {})
    index = int(manifest.get("failed_sample_index", 0) or 0)
    status = int(manifest.get("failed_status", 3) or 3)
    partial_result = None
    try:
        decoded = build_result(model, case, partial, stop=index)
        partial_result = replace(
            decoded, metrics=compute_case_metrics(FAMILY, decoded)
        )
    except Exception:  # noqa: BLE001 - the original failure is the report
        partial_result = None
    return NativeAxleError(
        str(error),
        status=status,
        partial_result=partial_result,
        failure_diagnostics=safe_failure_row(partial, index),
        failed_sample_index=index,
        failed_time_s=float(manifest.get("failed_time_s") or 0.0),
    )


def _original_ids(raw, model, aliases):
    """Preserve frozen assertions with explicitly declared old-to-new IDs."""
    document = dict(raw.document)
    manifest = dict(document.get("manifest", {}))
    for key in ("body_names", "bodies"):
        if key in manifest:
            manifest[key] = [aliases.get(name, name) for name in manifest[key]]
    for key in ("constraint_names", "spring_names", "damper_names", "bump_stop_names", "bushing_names", "anti_roll_bar_names", "tire_names"):
        if key in manifest:
            manifest[key] = [name.split(".", 1)[-1] for name in manifest[key]]
    document["manifest"] = manifest
    model_document = dict(raw.model_document or {})
    model_document["tires"] = [
        {**row, "name": row["name"].split(".", 1)[-1]}
        for row in model_document.get("tires", ())
    ]
    return replace(raw, document=document, model_document=model_document)
