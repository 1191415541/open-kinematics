"""
The solve plan: what a run is, once the bench and the study have been asked.

Everything that used to be *implied by the family name* lives here instead.  A
family name told the code three things at once -- which bench, which reading, and
which inputs -- and they are three independent questions:

* the **bench** (`rigs.rig.RigSpec`) says what drives and measures the model;
* the **study** (`studies.study.StudySpec`) says how the model is read: a grid of
  independent equilibria or one integrated history, and how far the tire is
  allowed to act;
* the **mode** (K or C) says which physical connection set the assembly carries.

A plan is the resolved answer to all of them, plus the sampling and the solver
settings, as a value.  It is what a compiler is handed: a plan and a model view,
never a template name and never a family name.

Two consequences are the point of the module:

* the same plan can be built for two different benches, so "the bench is not the
  family" is a property of a value rather than a claim in a document;
* the plan carries the tire activation, so the quasi-static reading's degenerate
  tire is a *plan* fact and not a second force law.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

from ..axle_dynamics.schema import AxleSolverSettings
from ..rigs.rig import RIGS, get_rig, rig_family
from ..studies.study import (
    DYNAMIC,
    QUASI_STATIC,
    TIRE_MODELS,
    StudyError,
    get_study,
    resolve_tire_activation,
)

__all__ = [
    "DEFAULT_TIMES_S",
    "KcStudyInputs",
    "SolvePlan",
    "plan_for",
]

#: The output grid a K/C case is solved on when the caller states none.  A K/C
#: state is time independent, so two samples is the minimum a uniform grid can
#: have and no more is needed.
DEFAULT_TIMES_S: tuple[float, ...] = (0.0, 1e-3)


@dataclass(frozen=True)
class KcStudyInputs:
    """
    The quasi-static inputs, independent of bench, study and model.

    These are what a caller sets: the sweep, the load paths, and the optional
    solver and grid overrides.  They travel on the plan rather than inside a
    request context so a compiler reads them from one place, and so a test can
    state a run without naming a bench or a family to get one.
    """

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


@dataclass(frozen=True)
class SolvePlan:
    """
    One run, resolved: bench, reading, physical set, sampling and outputs.

    ``mode`` is K or C and belongs to the assembly, so the plan carries it rather
    than deriving it: a compliant axle read quasi-statically is legitimate, and a
    study that implied C would make that combination unexpressible.

    ``drive_wheels`` is derived from the *study* and the bench's wheel-supplying
    capability, not from the family name.  That is what makes a newly authored
    bench route an existing reading without having to be called after it.
    """

    #: The bench that drives and measures this run.
    rig: str
    #: The case family the contract runs through.  Internal routing key.
    family: str
    #: The reading: `quasi_static` or `dynamic`.
    study: str
    #: The physical connection set of the assembly: K or C.
    mode: Literal["K", "C"] = "K"
    #: How far a tire is allowed to act, from the study's own declaration.
    tire_activation: str = "full"
    #: Whether the wheel centres are the driven coordinates.
    drive_wheels: bool = True
    #: How much of the assembly's mechanics the document carries.
    #:
    #: ``None`` means "let the legacy boolean decide", through the single mapping in
    #: `schema.case.drive_mode_for`, so `drive_wheels=True/False` and the three-way field
    #: cannot drift apart.  Stated as a three-way name rather than a second boolean because
    #: the readings differ in kind, not in degree: `kinematics` solves the constraints
    #: alone, `force_balance` balances the elastic elements, and `pad` lets the tires carry
    #: a wheel the ground moves under.
    drive_mode: str | None = None
    #: The output grid.
    times_s: tuple[float, ...] = DEFAULT_TIMES_S
    #: The solver settings the case document states.
    solver: AxleSolverSettings = field(default_factory=AxleSolverSettings)
    #: The family's own case inputs.
    inputs: KcStudyInputs = field(default_factory=KcStudyInputs)
    #: The tire law the model declares; validated, not selected, by the study.
    tire_model: str = "native_brush"
    #: What the caller asked the run to produce.  Empty means "whatever the
    #: assembly and the bench declare".
    outputs: tuple[str, ...] = ()
    #: The SI model and case a *dynamic* family authors from.
    #:
    #: A compile view of an assembly is kinematic: it carries bodies and joints
    #: but no inertias, tire laws or time history.  The axle-dynamics family needs
    #: all of them, so the objects it authors from are stated on the plan instead
    #: of being guessed from a family name -- which is exactly the guess this
    #: module exists to remove.
    dynamic_model: Any = None
    dynamic_case: Any = None
    #: Free-form note recorded in the compiled metadata.
    note: str = ""

    def __post_init__(self) -> None:
        if self.mode not in ("K", "C"):
            raise ValueError(f"solve plan mode must be K or C, got {self.mode!r}")
        if self.study not in (QUASI_STATIC, DYNAMIC):
            raise StudyError(
                f"unknown study {self.study!r}; the declared studies are "
                f"{QUASI_STATIC}, {DYNAMIC}"
            )
        if not self.rig:
            raise ValueError("a solve plan needs the bench it runs on")
        if not self.family:
            raise ValueError("a solve plan needs the family it routes through")
        if not self.times_s:
            raise ValueError("a solve plan needs an output grid")

    @property
    def quasi_static(self) -> bool:
        """Return whether this plan reads a grid of independent equilibria."""
        return self.study == QUASI_STATIC

    def describe(self) -> dict[str, Any]:
        """Return the plan's routing facts, for compiled metadata."""
        return {
            "rig": self.rig,
            "family": self.family,
            "study": self.study,
            "mode": self.mode,
            "tire_activation": self.tire_activation,
            "drive_wheels": self.drive_wheels,
        }


def plan_for(
    rig: str,
    *,
    study: str = "",
    mode: Literal["K", "C"] = "K",
    family: str = "",
    times_s: tuple[float, ...] = (),
    solver: AxleSolverSettings | None = None,
    inputs: KcStudyInputs | None = None,
    tire_model: str = "native_brush",
    outputs: tuple[str, ...] = (),
    drive_wheels: bool | None = None,
    drive_mode: str | None = None,
    dynamic_model: Any = None,
    dynamic_case: Any = None,
    note: str = "",
) -> SolvePlan:
    """
    Resolve one run from the bench it is on, and nothing else.

    The bench answers the family (unless the caller names one) and the study
    (unless the caller names one), which is the whole reason the two axes are
    separate: a caller says which bench it wants and everything else follows.
    A bench that declares no study fits either, and then the caller must say.

    ``drive_wheels`` follows the case's own inputs first -- a sweep of wheel
    values *is* driving the wheel centres -- and otherwise the bench's declared
    capability.  It is never read off the family name, which is what used to make
    a newly authored bench unable to route an existing reading.
    """
    if tire_model not in TIRE_MODELS:
        raise StudyError(
            f"unknown tire model {tire_model!r}; the studies accept "
            f"{', '.join(TIRE_MODELS)}"
        )
    resolved_family = str(family).strip().lower() or rig_family(rig)
    resolved_study = _resolve_study(rig, study)
    resolved_inputs = inputs if inputs is not None else KcStudyInputs()
    active_times = tuple(times_s) or tuple(resolved_inputs.times_s) or DEFAULT_TIMES_S

    if drive_wheels is None:
        drive_wheels = bool(resolved_inputs.wheel_values_mm) or _wheel_supplying(rig)

    # The study's tire activation comes from the study declaration and nowhere
    # else: it is the one place "how far a tire acts under this reading" is
    # stated, and the model supplies the law.
    activation = resolve_tire_activation(resolved_study, tire_model=tire_model)

    return SolvePlan(
        rig=str(rig).strip().lower(),
        family=resolved_family,
        study=resolved_study,
        mode=mode,
        tire_activation=activation,
        drive_wheels=bool(drive_wheels),
        drive_mode=drive_mode,
        times_s=active_times,
        solver=solver if solver is not None else AxleSolverSettings(),
        inputs=resolved_inputs,
        tire_model=tire_model,
        outputs=tuple(outputs),
        dynamic_model=dynamic_model,
        dynamic_case=dynamic_case,
        note=note,
    )


def _wheel_supplying(rig: str) -> bool:
    """Return whether a bench owns the wheels it drives."""
    key = str(rig).strip().lower()
    if key not in RIGS:
        return False
    return bool(get_rig(key).supplies_wheels)


def _resolve_study(rig: str, study: str) -> str:
    """
    Return the study a run reads by, resolving the bench's declaration.

    A bench that declares one settles it; a bench that declares none needs the
    caller to say, and a caller that says something the bench contradicts is
    refused rather than silently overridden -- the run's reading would otherwise
    depend on which of the two the reader happened to trust.
    """
    given = str(study).strip().lower()
    key = str(rig).strip().lower()
    declared = str(RIGS[key].study or "") if key in RIGS else ""
    if given and declared and given != declared:
        raise StudyError(
            f"rig {key!r} reads the model {declared!r}, not {given!r}; name the "
            "bench that takes the reading you want"
        )
    resolved = given or declared
    if not resolved:
        raise StudyError(
            f"rig {key!r} declares no study, so the caller must name one "
            f"({QUASI_STATIC} or {DYNAMIC})"
        )
    return get_study(resolved).name
