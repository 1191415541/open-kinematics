"""K sweep expansion tests."""

from suspension_multibody.analysis import KGrid, run_k_grid
from suspension_multibody.model import build_front_axle
from suspension_multibody.schema import FrontAxleModel, MassSpec


def _assembly():
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
    return build_front_axle(model)


def test_default_k_grid_is_10_by_10() -> None:
    grid = KGrid()
    assert grid.state_count == 100
    results = run_k_grid(_assembly(), KGrid((0.0,), (0.0,)))
    assert len(results) == 1
    assert results[0].case_id == "k-00-00"
