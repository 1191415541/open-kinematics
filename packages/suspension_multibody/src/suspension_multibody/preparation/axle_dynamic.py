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
    case = request.case
    model = _dynamic_model(request.model, request)
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


def _dynamic_model(
    source: Any, request: SimulationRequest
) -> AxleDynamicsModel:
    """
    Return the SI dynamic model one request's model resolves to.

    A caller may hand this family either an SI model it authored -- the original
    and still the only fully specifying input -- or an *already assembled* K/C
    axle, which is the study merge's point: `kc_quasi_static` and `axle_dynamic`
    are two readings of one assembly, so the same object must be able to feed
    both.  The conversion goes through `studies.bridge`, the single place the
    millimetre K/C assembly becomes an SI model, so this family adds an input
    route rather than a second conversion.

    The assembly's own mode is passed through rather than defaulted: mode belongs
    to the assembly, and asking a C assembly for the K reading is a caller error
    that must name itself here instead of silently producing the wrong model.

    The bench is resolved against the assembly it will run on, so an assembly
    handed to this family is subject to the same check as one handed to
    `kc_quasi_static`.  An authored SI model carries no capabilities to check
    against, and `check_assembly` accepts that case rather than inventing an
    answer.

    Driving and the case stay the caller's: an assembly carries bodies, joints,
    bushings and tires, not a driven-coordinate table or a time history, so a
    request built from one still authors its own driven coordinates.
    """
    from ..preparation.assembly import FrontAxleAssembly
    from ..rigs import check_assembly
    from ..studies import DYNAMIC, axle_dynamics_model, build_study_assembly

    if isinstance(source, AxleDynamicsModel):
        return source
    if isinstance(source, FrontAxleAssembly):
        study_assembly = build_study_assembly(
            source, study=DYNAMIC, mode=source.mode
        )
        check_assembly(
            ASSEMBLY,
            request.rig,
            getattr(study_assembly.assembly, "capabilities", None),
        )
        return axle_dynamics_model(study_assembly, name=request.name or "axle")
    raise TypeError(
        "axle dynamic preparation requires an AxleDynamicsModel or an "
        f"assembled FrontAxleAssembly, got {type(source).__name__}"
    )


__all__ = [
    "ASSEMBLY",
    "FAMILY",
    "PREPARED_KEY",
    "AxleDynamicPrepared",
    "prepare_request",
]
