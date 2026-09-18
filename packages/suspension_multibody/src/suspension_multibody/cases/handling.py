"""
Emit and run the `handling` contract documents.

A handling manoeuvre is a steering input as a function of time.  This module is
the authoring side of it: a shape per actuator, in the units the document
declares, plus the independent expansion the equivalence check compares against.

Only open-loop manoeuvres are expressible here.  A closed-loop manoeuvre -- an
ISO lane change, say -- needs a driver following a path, which is a different
model rather than a different shape, and the kernel refuses those by name rather
than approximating them with a steering history that happens to look similar.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

import numpy as np
from suspension_contracts import pack_container

from ..axle_dynamics.schema import AxleSolverSettings
from ..kernel import ContractRun, run_contract
from .vehicle_dynamic import _solver_block

__all__ = ["SteeringShape", "case_document", "run_handling_contract", "steering_signals"]

ShapeKind = Literal["constant", "ramp", "step", "sine"]


@dataclass(frozen=True)
class SteeringShape:
    """One actuator's commanded history, as a shape rather than a table."""

    actuator: str
    shape: ShapeKind
    amplitude: float = 0.0
    start_s: float = 0.0
    rise_s: float = 0.0
    frequency_hz: float = 0.0
    phase_rad: float = 0.0


def steering_signals(
    shapes: tuple[SteeringShape, ...], times_s: tuple[float, ...]
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """
    Return the target and rate each actuator is driven with.

    This is the same expansion the kernel performs, written independently, and
    it is what the equivalence check uses as the other side of the comparison.
    The rate is the analytic derivative for the same reason it is in the kernel:
    a finite difference of the target is a different manoeuvre.
    """
    times = np.asarray(times_s, dtype=float)
    target: dict[str, np.ndarray] = {}
    rate: dict[str, np.ndarray] = {}
    for shape in shapes:
        if shape.shape == "constant":
            # A step at the start time, not a value from the beginning of the
            # run: the static trim starts from the assembling pose, so a
            # manoeuvre has to begin with the wheel straight.
            values = np.where(times >= shape.start_s, float(shape.amplitude), 0.0)
            rates = np.zeros_like(times)
        elif shape.shape in ("ramp", "step"):
            if shape.rise_s > 0.0:
                fraction = np.clip((times - shape.start_s) / shape.rise_s, 0.0, 1.0)
                inside = (times > shape.start_s) & (times < shape.start_s + shape.rise_s)
                rates = np.where(inside, float(shape.amplitude) / shape.rise_s, 0.0)
            else:
                fraction = (times >= shape.start_s).astype(float)
                rates = np.zeros_like(times)
            values = float(shape.amplitude) * fraction
        else:
            angular = 2.0 * np.pi * float(shape.frequency_hz)
            angle = angular * (times - shape.start_s) + float(shape.phase_rad)
            values = float(shape.amplitude) * np.sin(angle)
            rates = float(shape.amplitude) * angular * np.cos(angle)
        target[shape.actuator] = values
        rate[shape.actuator] = rates
    return target, rate


def case_document(
    *,
    name: str,
    shapes: tuple[SteeringShape, ...],
    times_s: tuple[float, ...],
    settings: AxleSolverSettings,
) -> dict[str, Any]:
    """Describe a handling manoeuvre as a case document."""
    times = np.asarray(times_s, dtype=float)
    if times.size < 2:
        raise ValueError("a handling case needs at least two sample times")
    steps = np.diff(times)
    if not np.allclose(steps, steps[0], rtol=0.0, atol=1e-15):
        raise ValueError(
            "the contract time grid is a start/end/step, so the samples must be uniform"
        )
    if not shapes:
        raise ValueError("a handling case needs at least one actuator shape")
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "handling",
        "name": name,
        "time": {
            "start_s": float(times[0]),
            "end_s": float(times[-1]),
            "step_s": float(steps[0]),
        },
        "solver": _solver_block(settings),
        "handling": {
            "steering": [
                {
                    "actuator": shape.actuator,
                    "shape": shape.shape,
                    "amplitude": float(shape.amplitude),
                    "start_s": float(shape.start_s),
                    "rise_s": float(shape.rise_s),
                    "frequency_hz": float(shape.frequency_hz),
                    "phase_rad": float(shape.phase_rad),
                }
                for shape in shapes
            ]
        },
    }


def run_handling_contract(
    model_document: dict[str, Any], *, model_payload: bytes, case: dict[str, Any]
) -> ContractRun:
    """Run one handling manoeuvre against an already-built model payload."""
    return run_contract(
        model_document,
        case,
        model_payload=model_payload,
        case_payload=pack_container(case),
    )
