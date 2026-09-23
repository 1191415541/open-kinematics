"""
Studies: the quasi-static or dynamic reading of one axle assembly.

`kc_quasi_static` and `axle_dynamic` were two families with two model schemas and
two assembly paths.  The requirement is that they be *one* simulation whose study
decides how it is read:

* a **quasi-static** study solves a grid of independent equilibria -- each state
  stands alone, nothing is integrated, and the tire is a vertical spring;
* a **dynamic** study integrates a time history, and the tire works in full.

What must *not* differ is the model.  The same template, the same subsystems and
the same tire definition feed both; the study changes the reading, not the
assembly.  That is why this module exists: without a declared study object,
"quasi-static" is only a family name and the two paths drift apart.

The declaration is deliberately thin.  It states each study's semantics -- time,
tire activation -- and leaves every mechanism to the layers that own them.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

__all__ = [
    "DYNAMIC",
    "QUASI_STATIC",
    "STUDIES",
    "STUDY_NAMES",
    "StudyError",
    "StudySpec",
    "TireActivation",
    "TimeSemantics",
    "get_study",
    "resolve_tire_activation",
    "study_names",
]

#: How a study advances through its states.
TimeSemantics = Literal["independent_equilibria", "integrated"]

#: How far a tire's force law is allowed to act in a study.
#:
#: `vertical_only` is the quasi-static reading: the tire supplies its vertical
#: stiffness, its dimensions and its mass, and nothing else.  Decision D1 is
#: explicit that this is *the same* force law under a degenerate activation, not a
#: second, simpler law added to the kernel.  A study that activates the full law
#: and one that does not therefore share a single tire definition.
TireActivation = Literal["vertical_only", "full"]

#: The two study names, and the only ones `get_study` accepts.
QUASI_STATIC = "quasi_static"
DYNAMIC = "dynamic"


class StudyError(ValueError):
    """A study name or a study declaration is not usable."""


@dataclass(frozen=True)
class StudySpec:
    """
    What one study is, as a declaration.

    `time_semantics` and `tire_activation` are the two facts that actually
    separate the readings; what a family used to decide for itself (solver
    default, result shape) is downstream of them.
    """

    name: str
    time_semantics: TimeSemantics
    tire_activation: TireActivation
    description: str = ""


STUDIES: dict[str, StudySpec] = {
    QUASI_STATIC: StudySpec(
        name=QUASI_STATIC,
        time_semantics="independent_equilibria",
        tire_activation="vertical_only",
        description=(
            "A grid of independent equilibria.  Each state is solved from the "
            "model alone, so the sample grid schedules poses rather than "
            "recording a history, and the tire acts as a vertical spring that "
            "carries its own mass."
        ),
    ),
    DYNAMIC: StudySpec(
        name=DYNAMIC,
        time_semantics="integrated",
        tire_activation="full",
        description=(
            "One integrated time history.  The tire works in full: longitudinal "
            "and lateral slip are live, and each state depends on the one before "
            "it."
        ),
    ),
}

#: The study names in a stable order.
STUDY_NAMES: tuple[str, ...] = tuple(STUDIES)

#: The tire laws both studies accept.  D1 keeps this list identical for the two
#: studies on purpose: the quasi-static reading degrades a law, it does not
#: restrict the menu.
TIRE_MODELS: tuple[str, ...] = ("fiala", "pac2002", "native_brush")


def study_names() -> tuple[str, ...]:
    """Return the study names in a stable order."""
    return STUDY_NAMES


def get_study(name: str) -> StudySpec:
    """Return a study by name, naming the unknown one if it is not declared."""
    try:
        return STUDIES[str(name).strip().lower()]
    except KeyError as error:
        known = ", ".join(STUDY_NAMES)
        raise StudyError(
            f"unknown study {name!r}; the declared studies are {known}"
        ) from error


def resolve_tire_activation(study: str, *, tire_model: str) -> TireActivation:
    """
    Return how far `tire_model` acts under `study`.

    Both studies *accept* every law -- that is D1: the quasi-static study takes
    `fiala`, `pac2002` and `native_brush` exactly as the dynamic one does, then
    uses only their vertical part.  A quasi-static-only law would be a second law
    to keep in step; degrading an existing one cannot drift from it.

    `tire_model` is therefore validated but does not change the answer, and that
    is deliberate: a caller who names a law the kernel does not know learns so
    here rather than at solve time.
    """
    normalized = str(tire_model).strip().lower()
    if normalized not in TIRE_MODELS:
        raise StudyError(
            f"unknown tire model {tire_model!r}; the studies accept "
            f"{', '.join(TIRE_MODELS)}"
        )
    return get_study(study).tire_activation
