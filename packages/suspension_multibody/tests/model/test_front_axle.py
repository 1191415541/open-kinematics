"""Front axle topology and K/C mode tests."""

from suspension_multibody.authoring import assemble_generic, migrate_v1_axle
from suspension_multibody.schema import MassSpec
from suspension_multibody.schema.model import AxleDeclaration


def _model() -> AxleDeclaration:
    return AxleDeclaration(
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
    documents = [migrate_v1_axle(_model(), mode=mode) for mode in ("K", "C")]
    k, c = [assemble_generic(doc) for doc in documents]
    assert k.bodies.keys() == c.bodies.keys()
    assert [doc.payload["mode"] for doc in documents] == ["K", "C"]
    points = documents[0].entries[0].subsystem.payload["hardpoints"]
    assert any(point[1] == -700 and point[2] == 450 for point in points.values())
    assert any(point[1] == 700 and point[2] == 450 for point in points.values())
    assert len(k.joints) > sum(row["type"] == "bushing" for row in c.elements)
    assert not any(row["type"] == "bushing" for row in k.elements)
    assert any(row["type"] == "bushing" for row in c.elements)


def test_front_axle_has_two_sides_and_rack() -> None:
    assembly = assemble_generic(migrate_v1_axle(_model()))
    assert {"upper_arm_L", "upper_arm_R", "lower_arm_L", "lower_arm_R"}.issubset(
        {name.rsplit(".", 1)[-1] for name in assembly.bodies}
    )
    assert "model.sub.json.rack" in assembly.bodies
    # 8 arm-mount rows, 2 outer joints, 2 rack-tie joints, 2 tie-upright joints, and
    # the two wheel spin joints with the wheel-centre row each hub adds.
    assert len(assembly.joints) == 16
