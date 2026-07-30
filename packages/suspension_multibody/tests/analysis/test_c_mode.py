"""C-mode paths, side modes and K deltas."""

import numpy as np
import pytest

from suspension_multibody.analysis import CModeSolver, KReferenceCache, LoadPath
from suspension_multibody.elements import PointWrenchElement
from suspension_multibody.model import build_front_axle
from suspension_multibody.schema import (
    Bushing6x6,
    FrontAxleModel,
    MassSpec,
    Pose,
    SixVector,
    Vec3,
)

_HARDPOINTS = {
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
}


def _mount_pose(name: str) -> Pose:
    x, y, z = _HARDPOINTS[name]
    return Pose(translation=Vec3(x=x, y=y, z=z))


def _model(*, compliant: bool = False) -> FrontAxleModel:
    bushings = ()
    if compliant:
        stiffness = tuple(
            tuple(
                10_000.0
                if row == column and row < 3
                else 10_000_000.0
                if row == column
                else 0.0
                for column in range(6)
            )
            for row in range(6)
        )
        bushings = tuple(
            Bushing6x6(
                name=f"{body}_{index}",
                body_a="chassis",
                body_b=body,
                pose_a=_mount_pose(name),
                pose_b=_mount_pose(name),
                stiffness=stiffness,
            )
            for body, names in (
                ("upper_arm", ("uca_front", "uca_rear")),
                ("lower_arm", ("lca_front", "lca_rear")),
            )
            for index, name in enumerate(names)
        )
    return FrontAxleModel(
        hardpoints=_HARDPOINTS,
        mass=MassSpec(sprung_mass=1000),
        bushings=bushings,
    )


def _assembly():
    return build_front_axle(_model())


def _c_assembly():
    return build_front_axle(_model(compliant=True), "C")


def test_standard_path_has_11_symmetric_levels_and_zero() -> None:
    path = LoadPath("fz", "fz", 10.0)
    assert path.values() == tuple(-10.0 + 2.0 * i for i in range(11))


def test_c_modes_apply_single_symmetric_and_opposite_loads() -> None:
    solver = CModeSolver(np.eye(6) * 0.1)
    assembly = _assembly()
    cache = KReferenceCache()
    single = solver.solve(assembly, SixVector(fz=10), side_mode="single", k_cache=cache)
    symmetric = solver.solve(
        assembly, SixVector(fz=10), side_mode="symmetric", k_cache=cache
    )
    opposite = solver.solve(
        assembly, SixVector(fz=10), side_mode="opposite", k_cache=cache
    )
    assert single.load_right.fz == 0
    assert symmetric.load_right.fz == 10
    assert opposite.load_right.fz == -10
    assert np.isclose(single.deformation_left.fz, 1.0)
    assert len(cache.entries) == 1


def test_c_path_uses_one_k_reference_for_all_levels() -> None:
    cache = KReferenceCache()
    result = CModeSolver(np.eye(6) * 0.1).run_path(
        _assembly(), LoadPath("fy", "fy", 1.0), k_cache=cache
    )
    assert len(result) == 11
    assert len(cache.entries) == 1
    assert result[5].level == 0


def test_physical_c_mode_solves_geometry_and_secant_response() -> None:
    result = CModeSolver().solve(_c_assembly(), SixVector(fz=100.0), side_mode="single")

    assert result.solver_kind == "equilibrium"
    assert result.equilibrium is not None and result.equilibrium.converged
    assert abs(result.c_minus_k["left_wheel_center_z"]) > 1e-3
    assert result.secant_compliance_left[2, 2] != 0.0
    assert result.metrics["left_wheel_center_z"] != 0.0


def test_physical_c_mode_rejects_a_noncompliant_assembly() -> None:
    with pytest.raises(ValueError, match="build_front_axle"):
        CModeSolver().solve(_assembly(), SixVector(fz=1.0))


def test_physical_c_mode_converges_for_each_wheel_center_wrench_axis() -> None:
    assembly = _c_assembly()
    solver = CModeSolver()

    for axis in ("fx", "fy", "fz", "mx", "my", "mz"):
        result = solver.solve(assembly, SixVector(**{axis: 1.0}), case_id=axis)

        assert result.equilibrium is not None and result.equilibrium.converged


def test_point_wrench_includes_the_wheel_center_force_arm() -> None:
    assembly = _c_assembly()
    local_center = assembly.point("upright_L", "wheel_center")
    force = np.array([10.0, -20.0, 30.0])
    moment = np.array([4.0, 5.0, 6.0])

    evaluation = PointWrenchElement(
        name="wheel_center_load",
        body="upright_L",
        point_local=local_center,
        force_global=force,
        moment_global=moment,
    ).evaluate(assembly.state)

    assert np.allclose(
        evaluation.body_wrenches_global["upright_L"][3:],
        np.cross(local_center, force) + moment,
    )
