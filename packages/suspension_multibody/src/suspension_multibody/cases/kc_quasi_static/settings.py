"""Declared default sampling and solver settings for K/C examples and gates."""

import numpy as np

from ...schema.solver import AxleSolverSettings

DEFAULT_TIMES = tuple(float(t) for t in np.linspace(0., 2e-3, 9))
DEFAULT_SETTINGS = AxleSolverSettings(internal_step_s=2.5e-4)
