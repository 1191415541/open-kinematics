"""
Studies: one simulation, read quasi-statically or dynamically.

`kc_quasi_static` and `axle_dynamic` were two families.  A study says how the
*same* assembly is read, so the two share a model, a template and a tire
definition and differ only in how that model is advanced and how far its tire is
allowed to act.

* `study.py` -- the declaration: which studies exist, their time semantics, their
  tire activation, and which tire laws both accept.
* `assembly.py` -- the single assembly entry both studies are configured from, so
  "the same model, two studies" is a property of the code rather than a promise.
* `bridge.py` -- the one place the K/C assembly becomes an SI dynamic model, so
  the two readings cannot drift apart a line at a time.

Nothing here solves or emits a contract document; that stays with the family
layers.  This package only states what a study *is* and builds the one model both
readings start from.
"""

from .assembly import (
    StudyAssembly,
    build_study_assembly,
    study_case_document,
    study_model_document,
)
from .bridge import BridgeError, axle_dynamics_model
from .study import (
    DYNAMIC,
    QUASI_STATIC,
    STUDIES,
    STUDY_NAMES,
    TIRE_MODELS,
    StudyError,
    StudySpec,
    TimeSemantics,
    TireActivation,
    get_study,
    resolve_tire_activation,
    study_names,
)

__all__ = [
    "DYNAMIC",
    "QUASI_STATIC",
    "STUDIES",
    "STUDY_NAMES",
    "TIRE_MODELS",
    "BridgeError",
    "StudyAssembly",
    "StudyError",
    "StudySpec",
    "TireActivation",
    "TimeSemantics",
    "axle_dynamics_model",
    "build_study_assembly",
    "get_study",
    "resolve_tire_activation",
    "study_case_document",
    "study_model_document",
    "study_names",
]
