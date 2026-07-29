"""K-reference cache tests."""

from suspension_mbd.analysis import KReferenceCache
from suspension_mbd.model import build_front_axle
from suspension_mbd.schema import FrontAxleModel, MassSpec


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


def test_reference_cache_reuses_zero_state() -> None:
    cache = KReferenceCache()
    first = cache.get_or_solve(_assembly())
    second = cache.get_or_solve(_assembly())
    assert first is second
    assert len(cache.entries) == 1
