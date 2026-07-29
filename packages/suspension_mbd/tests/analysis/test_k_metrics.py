"""K&C metric convention tests."""

from suspension_mbd.analysis import compute_k_metrics
from suspension_mbd.model import build_front_axle
from suspension_mbd.schema import FrontAxleModel, MassSpec


def test_static_symmetric_metrics_have_zero_differences() -> None:
    model = FrontAxleModel(
        hardpoints={
            "uca_front": [-1, -2, 4],
            "uca_rear": [1, -2, 4],
            "uca_outer": [0, -3, 4],
            "lca_front": [-1, -2, 1],
            "lca_rear": [1, -2, 1],
            "lca_outer": [0, -3, 1],
            "tierod_inner": [1, -2, 2],
            "tierod_outer": [0, -3, 2],
            "wheel_center": [0, -3, 3],
            "rack_center": [0, 0, 2],
        },
        mass=MassSpec(sprung_mass=1000),
    )
    assembly = build_front_axle(model)
    metrics = compute_k_metrics(assembly.state, assembly)
    assert metrics["camber_deg_difference"] == 0
    assert metrics["toe_deg_difference"] == 0
    assert metrics["track_mm"] == 6
