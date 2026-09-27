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
from ..subsystems.runtime import SubsystemRuntime

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

    assembly: SubsystemRuntime
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
    model: Any,
    *,
    mode: Literal["K", "C"],
    rig: str,
    request: Any = None,
    name: str = "axle",
) -> SubsystemRuntime:
    """
    Return the assembly this family runs, with its bench checked.

    The assembly is built through the **composition layer**, which is the same
    construction a dynamic run uses: a quasi-static request and a dynamic one are
    two readings of one build, and neither is a path beside the other.  The bench
    the request names is resolved against the built assembly here rather than
    assumed, so "this run is this assembly on this bench" is checkable at the call
    site instead of being a claim about two modules agreeing.

    `api` calls this directly.  It authors its own contract documents -- that
    split is older than this function -- but the assembly those documents are
    written from has to be the one the composition builds, or the two readings
    drift and the rig check never runs.

    The mode travels on the assembly request rather than being resolved here, so a
    C assembly asked for the K reading is refused by the same check that guards
    every other entry.

    ``request`` carries the *subsystem set* when the caller wants something other
    than the default axle, which is how a run can be asked for without steering.
    It is an ``AssemblyRequest`` and it is passed straight through, because it
    already decides the mode and re-deriving that would be a second answer to a
    question this function has just been told.
    """
    from ..rigs import check_assembly
    from ..subsystems.si_assembly import si_assembly_for_axle
    from ..subsystems.types import AssemblyRequest

    if not isinstance(model, (SubsystemRuntime, FrontAxleModel)):
        raise TypeError(
            "kc quasi-static preparation requires a FrontAxleModel or a composed "
            f"SubsystemRuntime, got {type(model).__name__}"
        )
    if request is not None:
        # A caller that states the subsystem set owns the mode with it, so the
        # two are checked for agreement rather than one silently winning.
        stated = getattr(request, "mode", mode)
        if stated != mode:
            raise ValueError(
                f"the run asks for mode {mode!r} and the assembly request says "
                f"{stated!r}; pass one or the other"
            )
    # The production entry builds through the composition layer, so the model a
    # K/C run solves is the one the composition produces.  This is the cutover the
    # architecture asks for: a run that went through the historical builder would
    # keep the old path alive no matter how complete the new one became.
    resolved = request if request is not None else AssemblyRequest(mode=mode)
    if isinstance(model, SubsystemRuntime):
        # An already-built runtime is handed back unchanged: a caller that built one
        # -- and the tests that compare two readings -- must not have it silently
        # rebuilt from a model it no longer carries.  Its own mode is still checked
        # against the reading being asked for, because a C assembly read as K would
        # produce documents describing a model nobody built.
        observed = getattr(model, "mode", None)
        if observed is not None and observed != mode:
            raise ValueError(
                f"the assembly was built for mode {observed!r} but this run asks "
                f"for mode {mode!r}; the mode belongs to the assembly"
            )
        assembly = model
    else:
        composed = si_assembly_for_axle(
            model, request=resolved, rig=rig, name=name
        )
        assembly = composed.assembly.physical
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
    from ..rigs import compose, get_rig

    model_emitted = model_document(assembly, name=name, drive_wheels=drive_wheels)
    # The drives the case may sweep are the rig's declaration shrunk to this
    # assembly's capabilities -- one judgement, made once, in `rigs.compose`.  The
    # case document is authored from that set rather than from a search over the
    # emitted coordinate names, so "which axes this run has" is a consequence of
    # the bench and the assembly and not of how a coordinate happens to be spelled.
    #
    # An assembly built outside the subsystem path reports no capabilities, and its
    # drives stay the bench's own declaration -- the same fallback `compose`'s
    # callers document, rather than a refusal that would break those callers.
    capabilities = getattr(assembly, "capabilities", None)
    if capabilities is None:
        declared_drives = tuple(
            drive.coordinate for drive in get_rig(request.rig).drives
        )
    else:
        declared_drives = tuple(
            drive.coordinate
            for drive in compose(get_rig(request.rig), capabilities).drives
        )
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
        drives=declared_drives,
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
