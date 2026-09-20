"""
Preparation for the ``axle_dynamic`` family.

The family's own model and case objects are already SI, so preparing a request
means authoring the two contract documents and their payloads -- the work
``AxleDynamicCompiler`` used to do inline.  The payload blobs are returned raw:
framing them into the container is the boundary's job, not this module's.

The prepared documents travel in the context as one ``axle_dynamic_prepared``
value rather than as separate document keys, because the family's compiler is
the only consumer and it frames both documents into one submission.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from ..axle_dynamics.schema import AxleDynamicsCase, AxleDynamicsModel
from ..simulation.preparation import PreparedSimulation
from ..simulation.request import SimulationRequest

ASSEMBLY = "axle"
FAMILY = "axle_dynamic"

#: The context key the family's compiler reads its preparation from.
PREPARED_KEY = "axle_dynamic_prepared"


@dataclass(frozen=True)
class AxleDynamicPrepared:
    """The contract documents one axle dynamic request was authored into."""

    model_document: dict[str, Any]
    model_payload: bytes
    case_document: dict[str, Any]
    case_payload: bytes


def _validate_identity(request: SimulationRequest) -> None:
    """Reject a request this family does not own."""
    if request.assembly != ASSEMBLY or request.family != FAMILY:
        raise ValueError(
            f"axle dynamic preparation expects {ASSEMBLY}/{FAMILY}, "
            f"got {request.assembly}/{request.family}"
        )


def prepare_request(request: SimulationRequest) -> PreparedSimulation:
    """Prepare the model and case documents for one axle dynamic request."""
    _validate_identity(request)
    model = request.model
    case = request.case
    if not isinstance(model, AxleDynamicsModel):
        raise TypeError(
            "axle dynamic preparation requires an AxleDynamicsModel, got "
            f"{type(model).__name__}"
        )
    if not isinstance(case, AxleDynamicsCase):
        raise TypeError(
            "axle dynamic preparation requires an AxleDynamicsCase, got "
            f"{type(case).__name__}"
        )

    from ..cases.axle_dynamic import case_document, model_document

    model_emitted, model_blob = model_document(model, name=request.name)
    case_emitted, case_blob = case_document(model, case, name=request.name)
    prepared = AxleDynamicPrepared(
        model_document=model_emitted,
        model_payload=model_blob,
        case_document=case_emitted,
        case_payload=case_blob,
    )
    return PreparedSimulation(
        request=request,
        value=prepared,
        context={PREPARED_KEY: prepared},
        metadata={"assembly": ASSEMBLY, "family": FAMILY},
    )


__all__ = [
    "ASSEMBLY",
    "FAMILY",
    "PREPARED_KEY",
    "AxleDynamicPrepared",
    "prepare_request",
]
