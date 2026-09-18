"""
The load-path vocabulary a `c` (compliance) case is described with.

This is case *vocabulary*, not a solver: one scalar direction swept over an odd
number of symmetric levels.  It lives with the native K/C authoring layer rather
than with the Python C solver it used to sit beside, because the Adams gate and
the case documents both need to name a load path while the Python solver is on
its way out -- an `adams` module reaching into `analysis` would have made that
solver undeletable.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

__all__ = ["LeftRightMode", "LoadAxis", "LoadPath", "LOAD_AXES"]

LoadAxis = Literal["fx", "fy", "fz", "mx", "my", "mz"]
LeftRightMode = Literal["single", "symmetric", "opposite"]

#: The six scalar directions, in the order every load-path block uses.
LOAD_AXES: tuple[LoadAxis, ...] = ("fx", "fy", "fz", "mx", "my", "mz")


@dataclass(frozen=True)
class LoadPath:
    """One scalar six-component load path."""

    name: str
    axis: LoadAxis
    maximum: float
    levels: int = 11

    def values(self) -> tuple[float, ...]:
        if self.levels < 2 or self.levels % 2 == 0:
            raise ValueError("load path levels must be an odd number >= 3")
        return tuple(
            float(value)
            for value in np.linspace(-self.maximum, self.maximum, self.levels)
        )

    @classmethod
    def standard(cls) -> tuple[LoadPath, ...]:
        return tuple(
            cls(name=axis, axis=axis, maximum=1.0) for axis in LOAD_AXES
        )
