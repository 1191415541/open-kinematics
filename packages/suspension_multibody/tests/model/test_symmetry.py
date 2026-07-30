"""Front axle symmetry tests."""

from suspension_multibody.model import mirror_hardpoints, side_hardpoints
from suspension_multibody.schema import Vec3


def test_mirror_preserves_xz_and_flips_y() -> None:
    source = {"A": Vec3(x=1, y=-2, z=3)}
    mirrored = mirror_hardpoints(source)
    assert mirrored["A__R"].as_tuple() == (1.0, 2.0, 3.0)
    assert side_hardpoints(source, "L")["A"].y == -2
    assert side_hardpoints(source, "R")["A"].y == 2
