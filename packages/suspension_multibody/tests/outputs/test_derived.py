"""
Derived outputs: an expression over minimum-unit outputs, and nothing else.

The layer's whole claim is that a report metric is reproducible from what a run
produced.  These tests attack that claim from the outside: an expression that
reads something it never declared, an evaluator handed a result object instead of
a mapping, an input the run did not produce, a parameter nobody declared.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from suspension_multibody.outputs import (
    DerivedOutput,
    DerivedOutputError,
    ExpressionRegistry,
    MissingOutputError,
    UndeclaredOutputError,
    UnknownOutputError,
)


def _registry() -> ExpressionRegistry:
    registry = ExpressionRegistry()
    registry.register_expression(
        "difference", lambda view, *, a, b: float(view[a]) - float(view[b])
    )
    registry.register_expression("echo", lambda view, *, name: view[name])
    return registry


def _output(name: str, **overrides: object) -> DerivedOutput:
    fields: dict[str, object] = {
        "name": name,
        "unit": "mm",
        "dimension": "length",
        "expression": "difference",
        "reads": ("left_y", "right_y"),
        "defaults": {"a": "left_y", "b": "right_y"},
    }
    fields.update(overrides)
    return DerivedOutput(**fields)  # ty: ignore[invalid-argument-type]


def test_a_derived_output_is_evaluated_from_declared_inputs_only() -> None:
    registry = _registry()
    registry.register(_output("track"))
    assert registry.evaluate("track", {"left_y": 800.0, "right_y": -800.0}) == 1600.0


def test_reading_an_undeclared_output_fails_and_names_it() -> None:
    registry = _registry()
    registry.register_expression("greedy", lambda view: view["secret"])
    registry.register(
        _output("greedy_out", expression="greedy", reads=("left_y",), defaults={})
    )
    with pytest.raises(UndeclaredOutputError) as error:
        registry.evaluate("greedy_out", {"left_y": 1.0, "secret": 2.0})
    assert "secret" in str(error.value)
    assert "left_y" in str(error.value)


def test_a_non_mapping_input_is_refused_by_type() -> None:
    registry = _registry()
    registry.register(_output("track"))
    with pytest.raises(TypeError, match="minimum-unit outputs only"):
        registry.evaluate("track", SimpleNamespace(left_y=1.0, right_y=2.0))


def test_an_input_the_run_did_not_produce_fails_and_names_it() -> None:
    registry = _registry()
    registry.register(_output("track"))
    with pytest.raises(MissingOutputError) as error:
        registry.evaluate("track", {"left_y": 1.0})
    assert "right_y" in str(error.value)
    assert "did not produce" in str(error.value)


def test_an_unknown_derived_output_names_the_registered_ones() -> None:
    registry = _registry()
    registry.register(_output("track"))
    with pytest.raises(UnknownOutputError, match="track"):
        registry.evaluate("no_such_output", {})


def test_an_undeclared_parameter_is_refused() -> None:
    registry = _registry()
    registry.register(_output("track"))
    with pytest.raises(DerivedOutputError, match="does not declare parameter"):
        registry.evaluate(
            "track",
            {"left_y": 1.0, "right_y": 2.0},
            parameters={"surprise": 1.0},
        )


def test_a_declared_parameter_may_be_overridden() -> None:
    registry = _registry()
    registry.register_expression(
        "scaled", lambda view, *, name, factor: float(view[name]) * factor
    )
    registry.register(
        _output(
            "scaled_out",
            expression="scaled",
            reads=("left_y",),
            defaults={"name": "left_y", "factor": 2.0},
        )
    )
    assert registry.evaluate("scaled_out", {"left_y": 3.0}) == 6.0
    assert (
        registry.evaluate("scaled_out", {"left_y": 3.0}, parameters={"factor": 10.0})
        == 30.0
    )


def test_registering_an_unknown_expression_is_refused() -> None:
    registry = _registry()
    with pytest.raises(DerivedOutputError, match="is not registered"):
        registry.register(_output("orphan", expression="nowhere"))


def test_a_declaration_naming_a_parameter_its_expression_cannot_take_is_refused() -> None:
    """
    A parameter the expression does not accept must fail where it is declared.

    `difference` takes `a` and `b`; a declaration that also carries `typo` is an
    authoring mistake.  Left unchecked it would surface as a bare `TypeError` from
    inside a report run, naming neither the output nor the parameter -- so the
    check lives in `register`, and this test is what keeps it there.
    """
    registry = _registry()
    with pytest.raises(DerivedOutputError, match="cannot accept") as error:
        registry.register(
            _output(
                "misdeclared",
                defaults={"a": "left_y", "b": "right_y", "typo": 1.0},
            )
        )
    assert "typo" in str(error.value)
    assert "misdeclared" in str(error.value)


def test_an_expression_accepting_arbitrary_keywords_is_not_restricted() -> None:
    """A `**kwargs` expression legitimately accepts any declared parameter."""
    registry = ExpressionRegistry()
    registry.register_expression("flexible", lambda view, **keywords: len(keywords))
    registry.register(
        _output("flexible_out", expression="flexible", defaults={"anything": 1.0})
    )
    assert registry.evaluate("flexible_out", {}) == 1


def test_a_duplicate_registration_is_refused_unless_replacing() -> None:
    registry = _registry()
    registry.register(_output("track"))
    with pytest.raises(DerivedOutputError, match="already registered"):
        registry.register(_output("track"))
    registry.register(_output("track", unit="in"), replace=True)
    assert registry.get("track").unit == "in"


def test_a_derived_output_may_read_another_derived_output() -> None:
    """
    A metric may be written over another metric.

    Axle-level results are naturally differences of per-side results, so the
    evaluator resolves a declared input that is itself a derived output.  The
    resolved value is then readable, exactly as a produced output would be.
    """
    registry = _registry()
    registry.register_expression(
        "compose", lambda view, *, a, b: float(view[a]) - float(view[b])
    )
    registry.register(_output("track"))
    registry.register(
        _output(
            "track_shift",
            expression="compose",
            reads=("track", "left_y"),
            defaults={"a": "track", "b": "left_y"},
        )
    )
    assert (
        registry.evaluate("track_shift", {"left_y": 800.0, "right_y": -800.0}) == 800.0
    )

def test_a_declaration_cycle_is_named_rather_than_recursed() -> None:
    registry = _registry()
    registry.register(
        _output("first", reads=("second",), defaults={"a": "second", "b": "left_y"})
    )
    registry.register(
        _output("second", reads=("first",), defaults={"a": "first", "b": "left_y"})
    )
    with pytest.raises(DerivedOutputError, match="cycle"):
        registry.evaluate("first", {"left_y": 1.0})


def test_for_legacy_finds_the_output_that_restates_a_function() -> None:
    registry = _registry()
    registry.register(
        _output("track", legacy="report.metrics.case_specific.compute_k_metrics")
    )
    assert (
        registry.for_legacy("report.metrics.case_specific.compute_k_metrics").name
        == "track"
    )
    with pytest.raises(UnknownOutputError, match="no derived output restates"):
        registry.for_legacy("report.metrics.axle.compute_axle_metrics")
