"""Model and case schema tests."""

import pytest
from pydantic import ValidationError

from suspension_multibody.schema import (
    CaseSpec,
    DisplacementControl,
    FrontAxleModel,
    MassSpec,
)


def test_model_defaults_to_left_side_and_mirrors_hardpoints() -> None:
    model = FrontAxleModel(
        hardpoints={"A": [1, -2, 3]}, mass=MassSpec(sprung_mass=1000)
    )
    assert model.hardpoints["A"].mirrored_y().y == 2


def test_model_rejects_right_side_input() -> None:
    with pytest.raises(ValidationError, match="left side"):
        FrontAxleModel(
            side="right", hardpoints={"A": [1, -2, 3]}, mass=MassSpec(sprung_mass=1000)
        )


def test_case_rejects_mixed_displacement_and_load_controls() -> None:
    with pytest.raises(ValidationError, match="conflicting controls"):
        CaseSpec(
            mode="K",
            controls=(
                DisplacementControl(target="rack", values=(0, 1)),
                {"kind": "load", "target": "rack", "values": [{"fx": 1}]},
            ),
        )


def test_case_requires_explicit_mode() -> None:
    with pytest.raises(ValidationError):
        CaseSpec()  # type: ignore[call-arg]
