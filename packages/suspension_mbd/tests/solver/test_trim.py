"""Forward and inverse trim tests."""

import numpy as np

from suspension_mbd.schema import MassSpec
from suspension_mbd.solver import TrimSolver, TrimTarget, target_wheel_load


def test_mass_to_default_wheel_load() -> None:
    target = target_wheel_load(MassSpec(sprung_mass=1000))
    assert np.isclose(target, (1000 + 100) * 9810 / 2)


def test_inverse_trim_solves_scalar_preload() -> None:
    target = TrimTarget("wheel_load", 25.0)
    result = TrimSolver().inverse_scalar(lambda preload: 2.0 * preload + 5.0, target)
    assert result.converged
    assert np.isclose(result.preloads["preload"], 10.0)
