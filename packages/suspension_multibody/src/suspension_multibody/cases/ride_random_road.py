"""
Emit and run the `ride_random_road` contract documents.

A random road is a superposition of spatial harmonics, and driving along it at
speed turns each harmonic into a temporal one.  This module is the authoring
side of that conversion: it writes the components down, and it carries the
independent expansion the equivalence check compares the kernel against.

Each wheel gets its own components rather than one profile plus a correlation
coefficient.  The general form costs nothing -- a correlated pair is the same
components on both wheels plus an independent set, and front-to-rear delay is a
phase -- and it does not need a second mechanism the first time someone wants an
uncorrelated profile.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from suspension_contracts import pack_container

from ..axle_dynamics.schema import AxleSolverSettings
from ..kernel import ContractRun, run_contract
from .vehicle_dynamic import _solver_block

__all__ = [
    "RandomRoadWheel",
    "RoadComponent",
    "case_document",
    "road_signals",
    "run_ride_random_road_contract",
]


@dataclass(frozen=True)
class RoadComponent:
    """One spatial harmonic of the profile."""

    amplitude_m: float
    wavelength_m: float
    phase_rad: float = 0.0


@dataclass(frozen=True)
class RandomRoadWheel:
    """The profile one wheel follows."""

    tire: str
    components: tuple[RoadComponent, ...]


def road_signals(
    wheels: tuple[RandomRoadWheel, ...],
    speed_mps: float,
    times_s: tuple[float, ...],
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Return the height and vertical velocity each wheel follows.

    This is the same expansion the kernel performs, written independently: a
    component of wavelength L traversed at v has angular frequency 2*pi*v/L, and
    the velocity is its analytic derivative.
    """
    times = np.asarray(times_s, dtype=float)
    height: dict[str, np.ndarray] = {}
    velocity: dict[str, np.ndarray] = {}
    for wheel in wheels:
        values = np.zeros_like(times)
        rates = np.zeros_like(times)
        for component in wheel.components:
            angular = 2.0 * np.pi * float(speed_mps) / float(component.wavelength_m)
            angle = angular * times + float(component.phase_rad)
            values = values + float(component.amplitude_m) * np.sin(angle)
            rates = rates + float(component.amplitude_m) * angular * np.cos(angle)
        height[wheel.tire] = values
        velocity[wheel.tire] = rates
    return height, velocity


def case_document(
    *,
    name: str,
    wheels: tuple[RandomRoadWheel, ...],
    speed_mps: float,
    times_s: tuple[float, ...],
    settings: AxleSolverSettings,
) -> dict[str, Any]:
    """Describe a random-road run as a case document."""
    times = np.asarray(times_s, dtype=float)
    if times.size < 2:
        raise ValueError("a random-road case needs at least two sample times")
    steps = np.diff(times)
    if not np.allclose(steps, steps[0], rtol=0.0, atol=1e-15):
        raise ValueError(
            "the contract time grid is a start/end/step, so the samples must be uniform"
        )
    if not wheels:
        raise ValueError("a random-road case needs at least one wheel")
    if speed_mps < 0.0:
        raise ValueError("a random-road case needs a non-negative speed")
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "ride_random_road",
        "name": name,
        "time": {
            "start_s": float(times[0]),
            "end_s": float(times[-1]),
            "step_s": float(steps[0]),
        },
        "solver": _solver_block(settings),
        "ride_random_road": {
            "speed_mps": float(speed_mps),
            "wheels": [
                {
                    "tire": wheel.tire,
                    "components": [
                        {
                            "amplitude_m": float(component.amplitude_m),
                            "wavelength_m": float(component.wavelength_m),
                            "phase_rad": float(component.phase_rad),
                        }
                        for component in wheel.components
                    ],
                }
                for wheel in wheels
            ],
        },
    }


def run_ride_random_road_contract(
    model_document: dict[str, Any], *, model_payload: bytes, case: dict[str, Any]
) -> ContractRun:
    """Run one random-road case against an already-built model payload."""
    return run_contract(
        model_document,
        case,
        model_payload=model_payload,
        case_payload=pack_container(case),
    )
