"""Minimal API/IO end-to-end test."""

from pathlib import Path

from suspension_mbd.api import run_case
from suspension_mbd.io import read_table
from suspension_mbd.schema import CaseSpec, FrontAxleModel, MassSpec


def test_end_to_end_result_has_stable_state_table(tmp_path: Path) -> None:
    model = FrontAxleModel(
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
    run_case(model, CaseSpec(mode="K"), tmp_path)
    assert len(read_table(tmp_path / "states.parquet")) == 1
