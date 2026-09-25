"""
Contract compilation: the solve plan, the model view, and the emitters.

This package is where a run stops being a family name and becomes a *value*.
Three inputs, no more:

* a :class:`~.plan.SolvePlan` -- the bench, the reading, the physical set, the
  sampling and the solver settings, all resolved;
* a :class:`~.model_view.ModelView` -- the model's entities, in one shape,
  whichever assembly produced them;
* an :class:`~.compile.Emitter` -- the family's own document authoring.

Nothing here solves, submits native or decodes a result.  A compiled submission
is documents and payloads, and what happens to them is the runner's business.

The division of labour is the point:

===========================  ==============================================
`plan.py`                    what the run *is*
`model_view.py`              what the model *is*
`compile.py`                 the documents those two imply
===========================  ==============================================

A family name appears in exactly one of them, as a registry key that selects an
emitter -- never as a condition inside an emitter.
"""

from .compile import (
    PAYLOAD_SCHEMA,
    CompilationError,
    Emitter,
    EmitterRegistry,
    compile_documents,
    compile_plan,
    default_emitters,
)
from .model_view import MM, ModelView, ViewError, view_of
from .plan import DEFAULT_TIMES_S, KcStudyInputs, SolvePlan, plan_for

__all__ = [
    "DEFAULT_TIMES_S",
    "MM",
    "PAYLOAD_SCHEMA",
    "CompilationError",
    "Emitter",
    "EmitterRegistry",
    "KcStudyInputs",
    "ModelView",
    "SolvePlan",
    "ViewError",
    "compile_documents",
    "compile_plan",
    "default_emitters",
    "plan_for",
    "view_of",
]
