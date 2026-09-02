"""Force element schema validation tests."""

import pytest
from pydantic import ValidationError

from suspension_multibody.schema import Bushing6x6, LinearSpring


def _points() -> dict[str, object]:
    return {
        "body_a": "chassis",
        "body_b": "upright",
        "point_a": [0, 0, 0],
        "point_b": [0, 0, 1],
    }


def test_spring_accepts_free_length_or_preload() -> None:
    spring = LinearSpring(name="front", stiffness=100, free_length=200, **_points())
    assert spring.free_length == 200
    preload = LinearSpring(
        name="front", stiffness=100, reference_length=180, preload=50, **_points()
    )
    assert preload.preload == 50


def test_spring_rejects_ambiguous_length_definition() -> None:
    with pytest.raises(ValidationError, match="exactly one"):
        LinearSpring(
            name="front",
            stiffness=100,
            free_length=200,
            preload=50,
            reference_length=180,
            **_points(),
        )


def test_bushing_requires_symmetric_positive_semidefinite_matrix() -> None:
    with pytest.raises(ValidationError, match="symmetric"):
        nonsymmetric = [[0.0] * 6 for _ in range(6)]
        nonsymmetric[0][1] = 1.0
        Bushing6x6(name="mount", stiffness=nonsymmetric, body_a="a", body_b="b")
    matrix = tuple(tuple(10.0 if i == j else 0.0 for j in range(6)) for i in range(6))
    result = Bushing6x6(name="mount", stiffness=matrix, body_a="a", body_b="b")
    assert result.stiffness[0][0] == 10


def test_bushing_accepts_six_independent_force_curves() -> None:
    matrix = tuple(tuple(0.0 for _ in range(6)) for _ in range(6))
    curves = (
        (),
        (),
        ((-1.0, -100.0), (0.0, 0.0), (1.0, 100.0)),
        (),
        (),
        (),
    )
    result = Bushing6x6(
        name="mount",
        stiffness=matrix,
        body_a="a",
        body_b="b",
        force_curves=curves,
    )
    assert result.force_curves[2][1] == (0.0, 0.0)

    with pytest.raises(ValidationError, match="six axis curves"):
        Bushing6x6(
            name="mount",
            stiffness=matrix,
            body_a="a",
            body_b="b",
            force_curves=((),),
        )
