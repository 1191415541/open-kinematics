"""C-mode paths, side modes and K deltas."""

import numpy as np

from suspension_mbd.analysis import CModeSolver, KReferenceCache, LoadPath
from suspension_mbd.model import build_front_axle
from suspension_mbd.schema import FrontAxleModel, MassSpec, SixVector


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
    result = CModeSolver().run_path(
        _assembly(), LoadPath("fy", "fy", 1.0), k_cache=cache
    )
    assert len(result) == 11
    assert len(cache.entries) == 1
    assert result[5].level == 0
