"""
One assembly, read by either study.

The requirement is that `kc_quasi_static` and `axle_dynamic` differ in how the
model is *read*, not in what the model *is*.  That is only true if both readings
start from the same object, so this module owns the single construction:

    assembly = build_study_assembly(model, study=...)

and both studies then ask that one object for their documents.  The two readings
are therefore the same runtime -- same bodies, same points, same constraints -- and a
divergence between them would have to be coded deliberately rather than appearing as
a side effect of two assembly functions drifting.

The mode is K or C and belongs to the assembly, not to the study: a dynamic run
of a compliant axle and a quasi-static run of a rigid one are both legitimate, so
the study is not allowed to imply the mode.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

from ..schema import FrontAxleModel
from .study import DYNAMIC, QUASI_STATIC, StudySpec, get_study

__all__ = [
    "StudyAssembly",
    "build_study_assembly",
    "study_case_document",
    "study_model_document",
]

#: Which K/C mode each study reads by default.
#:
#: A quasi-static sweep *is* the K or C reading -- the caller usually chooses per
#: run -- while a dynamic run has no such choice and uses the compliant set.
_DEFAULT_MODE: dict[str, str] = {
    QUASI_STATIC: "K",
    DYNAMIC: "K",
}


@dataclass(frozen=True)
class StudyAssembly:
    """
    One assembly, plus the study that is reading it.

    The assembly is a single object on purpose: a caller can read the same axle
    quasi-statically and dynamically without rebuilding it, which is what makes
    "the same model, two studies" checkable by identity rather than by comparing
    two independently built results and hoping.

    ``assembly`` is typed loosely because a composition and a bare runtime both
    legitimately reach here: the
    :class:`~suspension_multibody.subsystems.runtime.SubsystemRuntime` a composition
    carries, and one a caller composed directly.  Both expose the same faces --
    ``mode``, bodies, both constraint columns, bushings, elements -- and the readers
    below ask for those faces rather than for a class.
    """

    assembly: Any
    study: StudySpec
    mode: str

    @property
    def study_name(self) -> str:
        """Return the study's name."""
        return self.study.name

    @property
    def tire_activation(self) -> str:
        """Return how far this study lets a tire act."""
        return self.study.tire_activation


def build_study_assembly(
    source: FrontAxleModel | Any,
    *,
    study: str,
    mode: str | None = None,
    request: Any = None,
) -> StudyAssembly:
    """
    Build the one assembly a study reads, from a model or an existing assembly.

    Accepting an already-built assembly or runtime matters: a caller who wants to
    compare two readings can assemble once and hand the same value to both, so the
    comparison is between two *readings* and not between two assemblies.  Passing a
    model builds it through the **composition layer**, so the study reads the same
    model the rest of the package builds -- there is no second assembly path here to
    drift from it.

    A :class:`~suspension_multibody.subsystems.runtime.SubsystemRuntime` is
    accepted as well, because that is what a composition carries: it exposes the
    same faces (``mode``, bodies, both constraint columns, bushings, elements), so
    a caller holding a composition can hand it straight to a study instead of
    rebuilding the assembly the composition replaces.
    """
    spec = get_study(study)
    resolved_mode = mode or _DEFAULT_MODE[spec.name]
    if resolved_mode not in ("K", "C"):
        raise ValueError(f"mode must be K or C, got {resolved_mode!r}")
    mode_literal: Literal["K", "C"] = "K" if resolved_mode == "K" else "C"

    assembly: Any
    if _is_runtime(source):
        assembly = source
        observed = getattr(assembly, "mode", None)
        if observed is not None and observed != resolved_mode:
            raise ValueError(
                f"study {spec.name!r} was asked for mode {resolved_mode!r} but the "
                f"assembly was built in mode {observed!r}; the mode belongs to the "
                "assembly, so build the one you mean"
            )
    elif isinstance(source, FrontAxleModel):
        from ..subsystems.si_assembly import si_assembly_for_axle
        from ..subsystems.types import AssemblyRequest

        resolved_request = (
            request
            if request is not None
            else AssemblyRequest(mode=mode_literal)
        )
        assembly = si_assembly_for_axle(
            source, request=resolved_request
        ).assembly.physical
    else:
        raise TypeError(
            "a study assembly is built from a FrontAxleModel or a "
            f"SubsystemRuntime, got {type(source).__name__}"
        )
    return StudyAssembly(assembly=assembly, study=spec, mode=resolved_mode)


def _is_runtime(source: Any) -> bool:
    """Return whether ``source`` is the runtime face a composition carries."""
    from ..subsystems.runtime import SubsystemRuntime

    return isinstance(source, SubsystemRuntime)


def study_model_document(
    study_assembly: StudyAssembly, *, name: str | None = None
) -> dict[str, Any]:
    """
    Return the model *document* the quasi-static study reads the assembly through.

    Only the quasi-static reading has a document here.  The dynamic reading needs
    the SI multibody schema rather than this millimetre contract, so it goes
    through `studies.bridge.axle_dynamics_model` and then the dynamic family's own
    emitter -- asking this function for it is an error rather than a silent
    conversion, because the two documents are not interchangeable.
    """
    from ..cases.kc_quasi_static import model_document

    if study_assembly.study_name != QUASI_STATIC:
        raise ValueError(
            f"study {study_assembly.study_name!r} reads an SI dynamic model rather "
            "than the K/C contract document; use studies.bridge.axle_dynamics_model"
        )
    return model_document(
        study_assembly.assembly,
        name=name or "axle",
        drive_wheels=study_assembly.mode == "K",
    )


def study_case_document(
    study_assembly: StudyAssembly,
    *,
    name: str | None = None,
    **inputs: Any,
) -> dict[str, Any]:
    """Return the case document this study reads, for the given inputs."""
    from ..cases.kc_quasi_static import case_document

    if study_assembly.study_name == QUASI_STATIC:
        return case_document(
            study_assembly.assembly,
            family=QUASI_STATIC,
            name=name or "axle",
            drive_wheels=study_assembly.mode == "K",
            **inputs,
        )
    raise ValueError(
        f"study {study_assembly.study_name!r} reads a time history rather than a "
        "quasi-static grid, so its case is authored by the dynamic family"
    )
