"""Shared evaluation helpers for sampled quasi-static cases."""

from __future__ import annotations

import numpy as np

from ..schema import DynamicCaseSpec, SixVector, TimeSignal, WrenchInput


def time_grid(case: DynamicCaseSpec) -> tuple[float, ...]:
    """Return the requested output time grid."""
    start = case.solver.start_time
    end = case.solver.end_time
    step = case.solver.output_step or case.solver.step_size
    count = int(np.floor((end - start) / step + 1e-12))
    values = [start + index * step for index in range(count + 1)]
    if values[-1] < end - 1e-12:
        values.append(end)
    return tuple(float(value) for value in values)


def motion(case: DynamicCaseSpec, target: str) -> TimeSignal:
    """Return one prescribed motion signal or a zero signal."""
    aliases = {
        "left_wheel_travel": "wheel_travel_left",
        "right_wheel_travel": "wheel_travel_right",
        "rack_displacement": "rack",
    }
    normalized = aliases.get(target, target)
    for prescribed in case.prescribed_motions:
        current = aliases.get(prescribed.target, prescribed.target)
        if current == normalized:
            return prescribed.displacement
    return TimeSignal(constant=0.0)


def loads_at_time(case: DynamicCaseSpec, time: float) -> dict[str, SixVector]:
    """Evaluate schema-level wrench inputs at one time."""
    return {item.target: item.wrench.value_at(time) for item in case.wrench_inputs}


def wrenches_at_time(case: DynamicCaseSpec, time: float) -> dict[str, np.ndarray]:
    """Assemble global body-origin wrenches at one time."""
    totals: dict[str, np.ndarray] = {}
    for item in case.wrench_inputs:
        body = target_body(item)
        value = item.wrench.value_at(time).as_array()
        force = value[:3]
        moment = value[3:].copy()
        if item.moment_reference == "application_point":
            moment += np.cross(item.application_point.as_array(), force)
        wrench = np.concatenate((force, moment))
        totals[body] = totals.get(body, np.zeros(6)) + wrench
    return totals


def target_body(item: WrenchInput) -> str:
    """Map public axle aliases to assembly body names."""
    target = item.target.lower().replace("-", "_")
    aliases = {
        "left": "upright_L",
        "wheel_left": "upright_L",
        "wheel_travel_left": "upright_L",
        "left_wheel": "upright_L",
        "right": "upright_R",
        "wheel_right": "upright_R",
        "wheel_travel_right": "upright_R",
        "right_wheel": "upright_R",
        "rack": "rack",
    }
    return aliases.get(target, item.target)
