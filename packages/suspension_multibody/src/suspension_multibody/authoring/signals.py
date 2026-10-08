"""Sampling helpers for offline case authoring."""

from __future__ import annotations

import numpy as np

from ..schema import DynamicCaseSpec, SixVector, TimeSignal, WrenchInput


def time_grid(case: DynamicCaseSpec) -> tuple[float, ...]:
    """Return the declared output samples, including a partial final interval."""
    start, end = case.solver.start_time, case.solver.end_time
    step = case.solver.output_step or case.solver.step_size
    count = int(np.floor((end-start)/step+1e-12))
    values = [start+index*step for index in range(count+1)]
    if values[-1] < end-1e-12:
        values.append(end)
    return tuple(float(value) for value in values)


def motion(case: DynamicCaseSpec, target: str) -> TimeSignal:
    """Read one declared motion using the offline schema's aliases."""
    aliases = {"left_wheel_travel": "wheel_travel_left", "right_wheel_travel": "wheel_travel_right", "rack_displacement": "rack"}
    normalized = aliases.get(target, target)
    for prescribed in case.prescribed_motions:
        if aliases.get(prescribed.target, prescribed.target) == normalized:
            return prescribed.displacement
    return TimeSignal(constant=0.0)


def loads_at_time(case: DynamicCaseSpec, time: float) -> dict[str, SixVector]:
    """Evaluate declared wrench signals."""
    return {item.target: item.wrench.value_at(time) for item in case.wrench_inputs}


def wrenches_at_time(case: DynamicCaseSpec, time: float) -> dict[str, np.ndarray]:
    """Sum global wrenches, translating moments to each body's origin."""
    totals: dict[str, np.ndarray] = {}
    for item in case.wrench_inputs:
        body = target_body(item)
        value = item.wrench.value_at(time).as_array()
        moment = value[3:].copy()
        if item.moment_reference == "application_point":
            moment += np.cross(item.application_point.as_array(), value[:3])
        totals[body] = totals.get(body, np.zeros(6))+np.concatenate((value[:3], moment))
    return totals


def target_body(item: WrenchInput) -> str:
    """Translate the offline axle schema's documented target aliases."""
    target = item.target.lower().replace("-", "_")
    aliases = {name: "upright_"+side for side, names in (
        ("L", ("left", "wheel_left", "wheel_travel_left", "left_wheel")),
        ("R", ("right", "wheel_right", "wheel_travel_right", "right_wheel"))) for name in names}
    return aliases.get(target, item.target)
