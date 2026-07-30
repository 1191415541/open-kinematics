"""Front axle topology and K/C mode tests."""

from suspension_multibody.model import build_front_axle
from suspension_multibody.schema import FrontAxleModel, MassSpec


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


def test_k_and_c_share_component_ids_and_mirror_geometry() -> None:
    k = build_front_axle(_model(), "K")
    c = build_front_axle(_model(), "C")
    assert k.component_ids == c.component_ids
    assert k.mode == "K"
    assert c.mode == "C"
    assert k.point("upper_arm_L", "outer")[1] == -700
    assert k.point("upper_arm_R", "outer")[1] == 700
    assert len(k.constraints) > len(c.bushings)
    assert all(connection.kind == "ideal" for connection in k.connections)
    assert any(connection.kind == "bushing" for connection in c.connections)


def test_front_axle_has_two_sides_and_rack() -> None:
    assembly = build_front_axle(_model())
    assert {"upper_arm_L", "upper_arm_R", "lower_arm_L", "lower_arm_R"}.issubset(
        assembly.bodies
    )
    assert "rack" in assembly.bodies
    assert len(assembly.connections) == 16
