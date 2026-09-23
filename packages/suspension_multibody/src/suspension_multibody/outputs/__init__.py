"""
Outputs: what a run produces, and what a report derives from that.

The layer exists to answer one question with a boundary instead of a convention:
**what did this run actually produce, and what is merely computed from it?**

* `declarations.py` -- an assembly declares its outputs, a rig declares its own,
  and the two are merged with conflict detection.  These are the *minimum-unit*
  outputs: the smallest things a run produces.
* `derived.py` -- a derived output is an expression over those minimum-unit
  outputs and nothing else.  The Adams-Car-request idea: a new report metric is a
  declaration naming an expression, not a new pipeline.
* `builtin.py` -- the existing `report/metrics` results, restated as derived
  outputs, with the old function recorded on each one so the migration can be
  checked value for value rather than asserted.

Nothing here solves, calls native, executes preparation or recomputes a
constitutive law.  A derived output that could do any of those would stop being
reproducible from the run's own outputs, which is the whole point of the layer.
"""

from .declarations import (
    DOMAINS,
    DeclarationError,
    DeclarationSet,
    OutputDeclaration,
    merge_declarations,
)
from .derived import (
    BUILTIN,
    DerivedOutput,
    DerivedOutputError,
    ExpressionRegistry,
    MinimumUnitOutputs,
    MissingOutputError,
    UndeclaredOutputError,
    UnknownOutputError,
    boolean,
    difference,
    evaluate,
    mean,
    ratio,
)

__all__ = [
    "BUILTIN",
    "DOMAINS",
    "DeclarationError",
    "DeclarationSet",
    "DerivedOutput",
    "DerivedOutputError",
    "ExpressionRegistry",
    "MinimumUnitOutputs",
    "MissingOutputError",
    "OutputDeclaration",
    "UndeclaredOutputError",
    "UnknownOutputError",
    "boolean",
    "difference",
    "evaluate",
    "mean",
    "merge_declarations",
    "ratio",
]
