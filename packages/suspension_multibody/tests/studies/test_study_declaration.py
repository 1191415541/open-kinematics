"""
A study is a declaration, and the two declarations differ in exactly two ways.

`kc_quasi_static` and `axle_dynamic` used to be families, which is why nothing
stopped their two assembly paths from drifting.  A study states how the *same*
model is read, so this file pins the declaration itself before any of it is used:
which studies exist, what each one's time semantics and tire activation are, and
that both accept the same tire laws (decision D1).
"""

from __future__ import annotations

import pytest

from suspension_multibody.studies import (
    DYNAMIC,
    QUASI_STATIC,
    STUDIES,
    TIRE_MODELS,
    StudyError,
    get_study,
    resolve_tire_activation,
    study_names,
)


def test_the_two_studies_are_declared() -> None:
    assert study_names() == (QUASI_STATIC, DYNAMIC)
    assert set(STUDIES) == {QUASI_STATIC, DYNAMIC}


def test_the_quasi_static_study_is_a_grid_of_independent_equilibria() -> None:
    spec = get_study(QUASI_STATIC)
    assert spec.time_semantics == "independent_equilibria"
    assert spec.tire_activation == "vertical_only"


def test_the_dynamic_study_integrates_a_history_and_uses_the_whole_tire() -> None:
    spec = get_study(DYNAMIC)
    assert spec.time_semantics == "integrated"
    assert spec.tire_activation == "full"


def test_an_unknown_study_is_refused_and_names_the_known_ones() -> None:
    with pytest.raises(StudyError, match="unknown study"):
        get_study("quasi_dynamic")
    with pytest.raises(StudyError, match=QUASI_STATIC):
        get_study("nope")


def test_a_study_name_is_matched_case_insensitively() -> None:
    assert get_study(" QUASI_STATIC ").name == QUASI_STATIC


def test_both_studies_accept_the_same_tire_laws() -> None:
    """
    Decision D1: the quasi-static study degrades a law, it does not restrict the
    menu.  A quasi-static-only law would be a second law to keep in step.
    """
    for study in study_names():
        for law in TIRE_MODELS:
            assert resolve_tire_activation(study, tire_model=law) in (
                "vertical_only",
                "full",
            )


def test_the_force_law_does_not_change_the_activation() -> None:
    """The degradation is a property of the study, not of the chosen law."""
    for law in TIRE_MODELS:
        assert resolve_tire_activation(QUASI_STATIC, tire_model=law) == "vertical_only"
        assert resolve_tire_activation(DYNAMIC, tire_model=law) == "full"


def test_a_law_the_kernel_does_not_know_is_refused_here() -> None:
    with pytest.raises(StudyError, match="unknown tire model"):
        resolve_tire_activation(QUASI_STATIC, tire_model="vertical_linear")


def test_the_declared_laws_are_the_three_the_kernel_knows() -> None:
    assert set(TIRE_MODELS) == {"fiala", "pac2002", "native_brush"}
