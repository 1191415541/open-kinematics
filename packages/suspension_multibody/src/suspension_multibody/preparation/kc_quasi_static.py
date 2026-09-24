"""
Preparation for the ``axle`` ``kc_quasi_static`` family.

A K/C quasi-static request is authored from a front-axle assembly and the K/C
inputs the case layer expands.  Preparing it means deciding which of the two
physical models the documents describe -- the rigid kinematic set of a K sweep
or the compliant set of a C one -- building the assembly when the request names
a model rather than an assembly, and emitting both contract documents.

``drive_wheels`` is a property of the case being run rather than of the model,
so it is resolved once here and handed to the model and the case document
together: the driven coordinates the model declares and the axis map the case
refers to have to name the same rows.

The family's documents are already millimetres and the kernel scales on the way
in, so nothing is converted here.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

from ..axle_dynamics.schema import AxleSolverSettings
from ..schema import FrontAxleModel
from ..simulation.preparation import PreparedSimulation
from ..simulation.request import SimulationRequest
from .assembly import FrontAxleAssembly

ASSEMBLY = "axle"
FAMILY = "kc_quasi_static"

#: The output grid a K/C case is solved on when the request does not state one.
DEFAULT_TIMES_S = (0.0, 1e-3)


@dataclass(frozen=True)
class KcQuasiStaticCase:
    """The K/C inputs one quasi-static request is authored from."""

    name: str = "kc-quasi-static"
    wheel_values_mm: tuple[float, ...] = ()
    rack_values_mm: tuple[float, ...] = ()
    drive: str = "wheel_center"
    left_right_mode: str = "symmetric"
    paths: tuple[str, ...] = ()
    levels: int = 11
    maximum: float = 1.0
    side_mode: str = "single"
    times_s: tuple[float, ...] = DEFAULT_TIMES_S
    settings: AxleSolverSettings | None = None
    drive_wheels: bool | None = None


@dataclass(frozen=True)
class KcQuasiStaticPrepared:
    """The assembly and the contract documents one K/C request was authored into."""

    assembly: FrontAxleAssembly
    model_document: dict[str, Any]
    case_document: dict[str, Any]


def _validate_identity(request: SimulationRequest) -> None:
    """Reject a request this family does not own."""
    if request.assembly != ASSEMBLY or request.family != FAMILY:
        raise ValueError(
            f"kc quasi-static preparation expects {ASSEMBLY}/{FAMILY}, "
            f"got {request.assembly}/{request.family}"
        )


def assembly_for(
    model: Any, *, mode: Literal["K", "C"], rig: str
) -> FrontAxleAssembly:
    """
    Return the assembly this family runs, with its bench checked.

    Two jobs that belong together: the assembly is built through the study layer,
    so a quasi-static request and a dynamic one are configurations of one
    construction, and the bench the request names is resolved against it here
    rather than assumed.  Keeping them in one entry is what makes "this run is
    this assembly on this bench" checkable at the call site instead of a claim
    about two modules agreeing.

    `api` calls this directly.  It authors its own contract documents -- that
    split is older than this function -- but the assembly those documents are
    written from has to be the one the study layer builds, or the two readings
    drift and the rig check never runs.

    The mode is passed to the study layer rather than resolved here, so a C
    assembly asked for the K reading is refused by the same check that guards
    every other study entry.
    """
    from ..rigs import check_assembly
    from ..studies import QUASI_STATIC, build_study_assembly

    if not isinstance(model, (FrontAxleAssembly, FrontAxleModel)):
        raise TypeError(
            "kc quasi-static preparation requires a FrontAxleAssembly or "
            f"FrontAxleModel, got {type(model).__name__}"
        )
    assembly = build_study_assembly(model, study=QUASI_STATIC, mode=mode).assembly
    check_assembly(ASSEMBLY, rig, getattr(assembly, "capabilities", None))
    return assembly


def prepare_request(request: SimulationRequest) -> PreparedSimulation:
    """Assemble one front axle and author its K/C contract documents."""
    _validate_identity(request)
    case = request.case
    if not isinstance(case, KcQuasiStaticCase):
        raise TypeError(
            "kc quasi-static preparation requires a KcQuasiStaticCase, got "
            f"{type(case).__name__}"
        )
    drive_wheels = (
        bool(case.wheel_values_mm) if case.drive_wheels is None else case.drive_wheels
    )
    mode: Literal["K", "C"] = "K" if drive_wheels else "C"
    assembly = assembly_for(request.model, mode=mode, rig=request.rig)
    name = request.name or case.name

    from ..cases.kc_quasi_static import case_document, model_document

    model_emitted = model_document(assembly, name=name, drive_wheels=drive_wheels)
    case_emitted = case_document(
        assembly,
        family=FAMILY,
        name=name,
        wheel_values_mm=case.wheel_values_mm,
        rack_values_mm=case.rack_values_mm,
        drive=case.drive,
        left_right_mode=case.left_right_mode,
        paths=case.paths,
        levels=case.levels,
        maximum=case.maximum,
        side_mode=case.side_mode,
        times_s=case.times_s,
        settings=case.settings if case.settings is not None else AxleSolverSettings(),
        drive_wheels=drive_wheels,
    )
    prepared = KcQuasiStaticPrepared(
        assembly=assembly,
        model_document=model_emitted,
        case_document=case_emitted,
    )
    return PreparedSimulation(
        request=request,
        value=prepared,
        context={
            "kc_assembly": assembly,
            "model_document": model_emitted,
            "case_document": case_emitted,
        },
        metadata={"assembly": ASSEMBLY, "family": FAMILY},
    )


__all__ = [
    "ASSEMBLY",
    "assembly_for",
    "DEFAULT_TIMES_S",
    "FAMILY",
    "KcQuasiStaticCase",
    "KcQuasiStaticPrepared",
    "prepare_request",
]
