"""Public run API tests."""

from pathlib import Path

from suspension_mbd.api import run_case
from suspension_mbd.schema import (
    CaseSpec,
    DisplacementControl,
    FrontAxleModel,
    MassSpec,
)


def _model() -> FrontAxleModel:
    return FrontAxleModel(
        hardpoints={
            "uca_front": [-100, -500, 400],
            "uca_rear": [100, -500, 400],
            "uca_outer": [0, -700, 450],
            "lca_front": [-120, -500, 150],
            "lca_rear": [120, -500, 150],
            "lca_outer": [0, -700, 150],
            "tierod_inner": [100, -400, 250],
            "tierod_outer": [50, -700, 250],
            "wheel_center": [0, -700, 300],
            "rack_center": [0, 0, 250],
        },
        mass=MassSpec(sprung_mass=1000),
    )


def test_run_case_writes_structured_output(tmp_path: Path) -> None:
    bundle = run_case(_model(), CaseSpec(name="demo", mode="K"), tmp_path)
    assert bundle.manifest.state_count == 1
    assert (tmp_path / "manifest.json").is_file()
    assert bundle.states[0].converged


def test_run_case_expands_displacement_controls_and_checkpoints(tmp_path: Path) -> None:
    checkpoint = tmp_path / "checkpoint.json"
    case = CaseSpec(
        name="grid",
        mode="K",
        controls=(
            DisplacementControl(target="wheel_travel_left", values=(-1.0, 1.0)),
            DisplacementControl(target="rack_displacement", values=(-0.5, 0.5)),
        ),
        checkpoint_path=str(checkpoint),
    )
    bundle = run_case(_model(), case)
    assert bundle.manifest.state_count == 4
    assert [state.state_id for state in bundle.states] == [
        "grid-0000",
        "grid-0001",
        "grid-0002",
        "grid-0003",
    ]
    assert checkpoint.is_file()
