"""K-mode drive tests."""

from suspension_multibody.analysis import KModeSolver
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


def test_wheel_center_and_contact_point_drives() -> None:
    assembly = _assembly()
    solver = KModeSolver()
    wheel = solver.solve(assembly, drive="wheel_center", case_id="wc")
    contact = solver.solve(
        assembly, drive="contact_point", case_id="cp", road_z=0, tire_radius=300
    )
    assert wheel.equilibrium.converged
    assert contact.equilibrium.converged
    assert wheel.drive == "wheel_center"
    assert contact.drive == "contact_point"
    assert wheel.metrics["track_mm"] > 0


def test_opposite_wheel_travel_preserves_zero_mean_at_zero_rack() -> None:
    assembly = _assembly()
    left = KModeSolver().solve(assembly, wheel_travel_left=5, wheel_travel_right=-5)
    assert left.equilibrium.converged
    assert abs(left.metrics["wheel_center_z_difference"]) > 0
