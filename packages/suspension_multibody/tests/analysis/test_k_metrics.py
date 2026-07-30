"""K&C metric convention tests."""

from dataclasses import replace

import numpy as np
import pytest

from suspension_multibody.analysis import compute_k_metrics
from suspension_multibody.core.rigid_body import RigidBodyState
from suspension_multibody.core.spatial import SE3
from suspension_multibody.model import build_front_axle
from suspension_multibody.schema import FrontAxleModel, MassSpec


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


def test_wheel_angles_use_lateral_axis_when_upright_is_steered() -> None:
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
    quaternion = np.array(
        [
            0.9997546608565223,
            -0.00010381443267525098,
            0.003797996815780008,
            0.021821607145468765,
        ]
    )
    bodies = dict(assembly.state.bodies)
    for side in ("L", "R"):
        body = bodies[f"upright_{side}"]
        bodies[body.name] = replace(body, pose=SE3(np.zeros(3), quaternion))
    metrics = compute_k_metrics(RigidBodyState(bodies), assembly)

    assert metrics["left_camber_deg"] == pytest.approx(-0.0023984589007616836)
    assert metrics["right_camber_deg"] == pytest.approx(0.0023984589007616836)
    assert metrics["left_toe_deg"] == pytest.approx(-2.500797637638747)
    assert metrics["right_toe_deg"] == pytest.approx(2.500797637638747)
