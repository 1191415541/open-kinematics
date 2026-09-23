"""
Derived outputs: the Adams-Car-request idea, in one expression layer.

A derived output is a *computation over minimum-unit outputs* and nothing else.
That restriction is the whole design.  The alternative -- letting a report
function reach into a result, a solver or a decoded kernel array -- is how a
metric stops being reproducible from what the run produced, and how a report
starts silently depending on a solver detail it never declared.

Two things follow, and both are enforced rather than documented:

* **the evaluator's input is a mapping of minimum-unit outputs.**  A result
  object, a solver or a live model is refused by type, not by convention;
* **an expression may only read what its declaration lists.**  Reading anything
  else fails and names the output it was not allowed to read, so a derived output
  cannot quietly grow an undeclared dependency.

An expression is a plain callable taking the output view plus its parameters.
Deriving a new report metric is therefore: declare the outputs it reads, write a
small function, register it.  No new class, no new pipeline.
"""

from __future__ import annotations

import inspect
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any

__all__ = [
    "DerivedOutput",
    "DerivedOutputError",
    "ExpressionRegistry",
    "MinimumUnitOutputs",
    "MissingOutputError",
    "UndeclaredOutputError",
    "UnknownOutputError",
    "boolean",
    "difference",
    "evaluate",
    "mean",
    "ratio",
]

#: An expression reads a `MinimumUnitOutputs` view and returns a value.
Expression = Callable[..., Any]


class DerivedOutputError(ValueError):
    """A derived output is malformed, unknown, or was evaluated wrongly."""


class UnknownOutputError(DerivedOutputError):
    """A declaration names an output that is not registered."""


class UndeclaredOutputError(DerivedOutputError):
    """An expression read an output its declaration does not list."""


class MissingOutputError(DerivedOutputError):
    """An expression needed a minimum-unit output the run did not produce."""


class MinimumUnitOutputs(Mapping[str, Any]):
    """
    The only thing an expression is handed: the run's minimum-unit outputs.

    It is a `Mapping`, so an expression reads it the obvious way, and it is a
    *restricted* one: only the names the declaration lists resolve.  A read of
    anything else raises rather than returning a value, which is what keeps "this
    metric only uses declared inputs" a checked fact instead of a promise.

    A declared name the run did not produce also raises -- on the read, not up
    front.  Some inputs are conditional (a diagnostic is read only when the run
    says diagnostics exist), and refusing those would reject a correct
    expression; a genuinely missing input still fails, naming itself.

    It is deliberately not an output *producer*: there is no way to write into it,
    so an expression cannot invent an output for a later one to read.
    """

    def __init__(
        self, values: Mapping[str, Any], declared: tuple[str, ...], *, output: str = ""
    ) -> None:
        self._values = values
        self._declared = tuple(declared)
        self._allowed = frozenset(declared)
        self._output = output

    @property
    def declared(self) -> tuple[str, ...]:
        """Return the output names this view allows, in declaration order."""
        return self._declared

    def available(self) -> tuple[str, ...]:
        """Return the declared names the run actually produced, in order."""
        return tuple(name for name in self._declared if name in self._values)

    def __getitem__(self, name: str) -> Any:
        if name not in self._allowed:
            declared = ", ".join(self._declared) or "(none)"
            raise UndeclaredOutputError(
                f"expression read output {name!r}, which its declaration does not "
                f"list; it declares {declared}"
            )
        try:
            return self._values[name]
        except KeyError as error:
            produced = ", ".join(sorted(self._values)) or "(none)"
            raise MissingOutputError(
                f"derived output {self._output!r} needs minimum-unit output "
                f"{name!r}, which the run did not produce; it produced {produced}"
            ) from error

    def __iter__(self):
        return iter(self._declared)

    def __len__(self) -> int:
        return len(self._declared)

    def __contains__(self, name: object) -> bool:
        return name in self._allowed

    def __repr__(self) -> str:
        return f"MinimumUnitOutputs(declared={list(self._declared)})"


@dataclass(frozen=True)
class DerivedOutput:
    """
    One computed output: a name, its description, and how to compute it.

    `expression` names a registered callable rather than holding one, so the
    declaration stays data: it serialises, it can be listed, and two sides can
    compare it.  `parameters` are the numbers a caller may override -- a KC
    gradient's step size, a toe convention's sign -- and `defaults` the values it
    gets when the caller says nothing.

    `legacy` records the old function this output restates, when it restates one.
    It exists so the migration from `report/metrics` can be *checked* value for
    value instead of asserted.
    """

    name: str
    unit: str
    dimension: str
    expression: str
    #: Minimum-unit outputs this expression reads, in the order it reads them.
    reads: tuple[str, ...] = ()
    #: Parameter names with their values when the caller supplies none.
    defaults: Mapping[str, float | int | bool | str] = field(default_factory=dict)
    #: The old implementation this restates, as `"module.function"`.
    legacy: str = ""
    description: str = ""

    def __post_init__(self) -> None:
        if not self.name:
            raise DerivedOutputError("a derived output must carry a name")
        if not self.expression:
            raise DerivedOutputError(
                f"derived output {self.name!r} must name an expression"
            )
        duplicates = sorted({name for name in self.reads if self.reads.count(name) > 1})
        if duplicates:
            raise DerivedOutputError(
                f"derived output {self.name!r} reads {duplicates} more than once"
            )

    def parameters(self, supplied: Mapping[str, Any] | None = None) -> dict[str, Any]:
        """
        Return the parameters the expression is called with.

        A caller may override a declared default and may not invent a parameter:
        a typo'd keyword would otherwise silently do nothing, which is exactly
        the failure a report must not have.
        """
        values: dict[str, Any] = dict(self.defaults)
        for key, value in (supplied or {}).items():
            if key not in values:
                known = ", ".join(sorted(values)) or "(none)"
                raise DerivedOutputError(
                    f"derived output {self.name!r} does not declare parameter "
                    f"{key!r}; it declares {known}"
                )
            values[key] = value
        return values


class ExpressionRegistry:
    """
    The name-to-callable registry derived outputs refer to.

    Kept as its own object rather than a module-level dict so a test can register
    an expression without touching the built-in set, and so the built-ins are
    registered through the same door as everyone else's.
    """

    def __init__(self) -> None:
        self._expressions: dict[str, Expression] = {}
        self._outputs: dict[str, DerivedOutput] = {}

    # -- expressions ------------------------------------------------------

    def register_expression(
        self, name: str, expression: Expression, *, replace: bool = False
    ) -> Expression:
        """Register a callable under a name, refusing a silent overwrite."""
        if not name:
            raise DerivedOutputError("an expression must carry a name")
        if name in self._expressions and not replace:
            raise DerivedOutputError(
                f"expression {name!r} is already registered; pass replace=True "
                "to overwrite it"
            )
        self._expressions[name] = expression
        return expression

    def expression(self, name: str) -> Expression:
        """Return a registered expression, naming the unknown one if absent."""
        try:
            return self._expressions[name]
        except KeyError as error:
            known = ", ".join(sorted(self._expressions)) or "(none registered)"
            raise DerivedOutputError(
                f"expression {name!r} is not registered; known expressions are {known}"
            ) from error

    def expression_names(self) -> tuple[str, ...]:
        """Return registered expression names in stable order."""
        return tuple(sorted(self._expressions))

    # -- derived outputs --------------------------------------------------

    def register(self, output: DerivedOutput, *, replace: bool = False) -> DerivedOutput:
        """
        Register a derived output, checking it against what is already known.

        The check is the point: an expression must exist, and every output the
        declaration reads must be registered as a derived output or be a
        minimum-unit output the caller supplies.  A derived output that reads an
        unknown name is rejected where it is declared, not when a report runs.
        """
        if not output.name:
            raise DerivedOutputError("a derived output must carry a name")
        if output.name in self._outputs and not replace:
            raise DerivedOutputError(
                f"derived output {output.name!r} is already registered; pass "
                "replace=True to overwrite it"
            )
        expression = self.expression(output.expression)
        self._check_parameters_accepted(output, expression)
        self._outputs[output.name] = output
        return output

    def _check_parameters_accepted(
        self, output: DerivedOutput, expression: Expression
    ) -> None:
        """
        Refuse a declaration that names a parameter its expression cannot take.

        The declaration and the signature are two halves of one contract, and a
        mismatch between them is an authoring mistake: `difference(a=..., b=...)`
        is fine, `difference(axis=...)` is not.  Without this check the mistake
        survives registration and surfaces as a bare `TypeError` from deep inside
        a report run -- naming neither the output nor the parameter.  It is
        caught here instead, where the fix is obvious.

        Only a callable whose signature can be read is inspected; a builtin or a
        C-level callable is accepted as-is rather than guessed at.
        """
        try:
            signature = inspect.signature(expression)
        except (TypeError, ValueError):
            return
        parameters = signature.parameters
        if any(
            parameter.kind is inspect.Parameter.VAR_KEYWORD
            for parameter in parameters.values()
        ):
            return
        accepted = {
            name
            for name, parameter in parameters.items()
            if parameter.kind
            in (
                inspect.Parameter.POSITIONAL_OR_KEYWORD,
                inspect.Parameter.KEYWORD_ONLY,
            )
        }
        unknown = sorted(set(output.defaults) - accepted)
        if unknown:
            known = ", ".join(sorted(accepted)) or "(none)"
            raise DerivedOutputError(
                f"derived output {output.name!r} declares parameter(s) {unknown}, "
                f"which expression {output.expression!r} cannot accept; it accepts "
                f"{known}"
            )

    def get(self, name: str) -> DerivedOutput:
        """Return a registered derived output, naming the unknown one if absent."""
        try:
            return self._outputs[name]
        except KeyError as error:
            known = ", ".join(sorted(self._outputs)) or "(none registered)"
            raise UnknownOutputError(
                f"derived output {name!r} is not registered; known derived outputs "
                f"are {known}"
            ) from error

    def names(self) -> tuple[str, ...]:
        """Return registered derived-output names in stable order."""
        return tuple(sorted(self._outputs))

    def outputs(self) -> tuple[DerivedOutput, ...]:
        """Return the registered derived outputs in stable order."""
        return tuple(self._outputs[name] for name in self.names())

    def for_legacy(self, legacy: str) -> DerivedOutput:
        """Return the output that restates `"module.function"`, or raise."""
        for name in self.names():
            if self._outputs[name].legacy == legacy:
                return self._outputs[name]
        raise UnknownOutputError(
            f"no derived output restates {legacy!r}; registered legacy mappings are "
            f"{sorted({o.legacy for o in self._outputs.values() if o.legacy})}"
        )

    # -- evaluation -------------------------------------------------------

    def evaluate(
        self,
        name: str,
        minimum_unit_outputs: Mapping[str, Any],
        *,
        parameters: Mapping[str, Any] | None = None,
    ) -> Any:
        """
        Evaluate one derived output over a run's minimum-unit outputs.

        The first argument is a *mapping* on purpose.  A decoded result, a solver
        or a model is not a mapping of minimum-unit outputs, so it is refused
        here with a message saying so -- which is what makes "derived outputs use
        only minimum-unit outputs" a boundary rather than a habit.

        The outputs `name` reads that the run did not produce are resolved first,
        so a derived output may be written over minimum-unit outputs *or* over
        other derived outputs.  An axle-level metric such as track is naturally
        the difference of two per-side results, and forbidding that would force
        every metric to restate the whole chain.
        """
        if not isinstance(minimum_unit_outputs, Mapping):
            raise TypeError(
                "a derived output is evaluated from minimum-unit outputs only; "
                "expected a mapping of output name to value, found "
                f"{type(minimum_unit_outputs).__name__}"
            )
        output = self.get(name)
        resolved = self._resolve_reads(
            output, minimum_unit_outputs, dict(parameters or {}), (name,)
        )
        # The availability check happens where an input is *read*, not up front:
        # an expression may legitimately read a diagnostic only when the run says
        # diagnostics exist, and only an actual read of a missing output is an
        # error.  The view enforces both rules, so neither depends on the
        # expression's good manners.
        view = MinimumUnitOutputs(
            {**minimum_unit_outputs, **resolved},
            output.reads,
            output=output.name,
        )
        return self.expression(output.expression)(
            view, **output.parameters(parameters)
        )

    def _resolve_reads(
        self,
        output: DerivedOutput,
        values: Mapping[str, Any],
        parameters: Mapping[str, Any],
        chain: tuple[str, ...],
    ) -> dict[str, Any]:
        """
        Evaluate the declared inputs the run did not produce, where they are
        derived outputs themselves.

        `chain` carries the outputs already being resolved, so a declaration cycle
        is named instead of recursing until the stack runs out.
        """
        resolved: dict[str, Any] = {}
        for required in output.reads:
            if required in values or required in resolved:
                continue
            if required not in self._outputs:
                continue  # the view reports the genuinely missing input
            if required in chain:
                raise DerivedOutputError(
                    "derived outputs form a cycle: "
                    + " -> ".join((*chain, required))
                )
            nested = self._resolve_reads(
                self.get(required), values, parameters, (*chain, required)
            )
            resolved.update(nested)
            resolved[required] = self.expression(
                self.get(required).expression
            )(
                MinimumUnitOutputs(
                    {**values, **resolved},
                    self.get(required).reads,
                    output=required,
                ),
                **self.get(required).parameters(parameters),
            )
        return resolved

#: The built-in registry, filled by `outputs.builtin`.
BUILTIN = ExpressionRegistry()


# --- shared expressions ---------------------------------------------------
#
# These are the arithmetic any derived output tends to need.  They stay here,
# next to the evaluator, because a derived output is a *declaration* that names
# an expression and the caller should be able to register a plain function
# without importing anything else.


def difference(view: MinimumUnitOutputs, *, a: str, b: str) -> Any:
    """Return `view[a] - view[b]`, the shape most "change in X" metrics take."""
    return _minus(view[a], view[b])


def mean(view: MinimumUnitOutputs, *, names: tuple[str, ...]) -> Any:
    """Return the arithmetic mean of several declared outputs."""
    values = [view[name] for name in names]
    if not values:
        raise DerivedOutputError("mean needs at least one output name")
    total = values[0]
    for value in values[1:]:
        total = _plus(total, value)
    return _divide(total, len(values))


def ratio(view: MinimumUnitOutputs, *, numerator: str, denominator: str) -> Any:
    """Return `view[numerator] / view[denominator]`."""
    return _divide(view[numerator], view[denominator])


def boolean(view: MinimumUnitOutputs, *, name: str, comparison: str, value: float) -> bool:
    """Return whether one output satisfies a comparison against a constant."""
    observed = float(view[name])
    if comparison == ">":
        return observed > value
    if comparison == ">=":
        return observed >= value
    if comparison == "<":
        return observed < value
    if comparison == "<=":
        return observed <= value
    raise DerivedOutputError(
        f"unknown comparison {comparison!r}; use one of '>', '>=', '<', '<='"
    )


def _plus(left: Any, right: Any) -> Any:
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return left + right
    import numpy as np

    return np.asarray(left) + np.asarray(right)


def _minus(left: Any, right: Any) -> Any:
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return left - right
    import numpy as np

    return np.asarray(left) - np.asarray(right)


def _divide(left: Any, right: Any) -> Any:
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return left / right
    import numpy as np

    return np.asarray(left) / np.asarray(right)


def evaluate(
    registry: ExpressionRegistry,
    name: str,
    minimum_unit_outputs: Mapping[str, Any],
    *,
    parameters: Mapping[str, Any] | None = None,
) -> Any:
    """Evaluate `name` through `registry` (a thin, readable entry point)."""
    return registry.evaluate(name, minimum_unit_outputs, parameters=parameters)
