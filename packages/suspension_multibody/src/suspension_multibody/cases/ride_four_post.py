"""
Emit and run the `ride_four_post` contract documents.

A four-post rig prescribes the height of each wheel pad.  The family exists so
that the excitation is written down once, declaratively -- per corner offset,
amplitude, frequency and phase -- and the kernel expands it, rather than every
caller hand-sampling sinusoids and hoping to agree on the derivative.

The expansion is a pure function of the declaration, so it is checked the only
way such a thing can be: the same excitation is computed independently here and
handed to the *vehicle* family as an explicit table, and the two runs have to
agree.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from ..axle_dynamics.schema import AxleSolverSettings
from .vehicle_dynamic import _solver_block

__all__ = ["FourPostCorner", "case_document"]


@dataclass(frozen=True)
class FourPostCorner:
    """One pad: which wheel it carries and the shape it is driven with."""

    tire: str
    offset_m: float = 0.0
    amplitude_m: float = 0.0
    frequency_hz: float = 0.0
    phase_rad: float = 0.0


def case_document(
    *,
    name: str,
    corners: tuple[FourPostCorner, ...],
    times_s: tuple[float, ...],
    settings: AxleSolverSettings,
) -> dict[str, Any]:
    """Describe a four-post sweep as a case document."""
    times = np.asarray(times_s, dtype=float)
    if times.size < 2:
        raise ValueError("a four-post case needs at least two sample times")
    steps = np.diff(times)
    if not np.allclose(steps, steps[0], rtol=0.0, atol=1e-15):
        raise ValueError(
            "the contract time grid is a start/end/step, so the samples must be uniform"
        )
    if not corners:
        raise ValueError("a four-post case needs at least one corner")
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "ride_four_post",
        "name": name,
        "time": {
            "start_s": float(times[0]),
            "end_s": float(times[-1]),
            "step_s": float(steps[0]),
        },
        "solver": _solver_block(settings),
        "four_post": {
            "corners": [
                {
                    "tire": corner.tire,
                    "offset_m": float(corner.offset_m),
                    "amplitude_m": float(corner.amplitude_m),
                    "frequency_hz": float(corner.frequency_hz),
                    "phase_rad": float(corner.phase_rad),
                }
                for corner in corners
            ]
        },
    }


def corner_signals(
    corners: tuple[FourPostCorner, ...], times_s: tuple[float, ...]
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Return the height and velocity each corner is driven with.

    This is the same expansion the kernel performs, written independently, and
    it is what the equivalence test uses as the other side of the comparison.
    The velocity is the analytic derivative: a finite difference of the height
    is a different excitation.
    """
    times = np.asarray(times_s, dtype=float)
    height: dict[str, np.ndarray] = {}
    velocity: dict[str, np.ndarray] = {}
    for corner in corners:
        angular = 2.0 * np.pi * float(corner.frequency_hz)
        angle = angular * times + float(corner.phase_rad)
        height[corner.tire] = float(corner.offset_m) + float(corner.amplitude_m) * np.sin(angle)
        velocity[corner.tire] = float(corner.amplitude_m) * angular * np.cos(angle)
    return height, velocity
