"""
The existing report metrics, restated as derived outputs.

Every number `report/metrics` produces today is one of two things:

* a **minimum-unit output** -- something the run produced (a tire force column,
  the time grid, an upright pose, a wheel load);
* a **derived output** -- arithmetic over those.

This module draws that line once, for all 27 legacy functions, and records on
every derived output which legacy function it restates.  The migration from
"report code computes numbers" to "the run declares outputs, the report derives
from them" is therefore *checkable*: the same inputs run through both, and the
values are compared.  A restatement nobody compares is a rewrite.

Two deliberate consequences, recorded here rather than discovered later:

* **key presence becomes declaration.**  Legacy `compute_vehicle_metrics` omits
  `maximum_steering_output` when the result carries no steering output; here the
  output is always declared, and a run that did not produce the input cannot
  evaluate it.  That is the intended direction -- a declared output set that does
  not depend on which fields a particular result happened to populate -- and the
  adapter below keeps the legacy behaviour available by omitting absent inputs.
* **`status`, `reason` and the `available` flags are not re-derived.**  They are
  availability and dispatch facts about a run, not arithmetic over outputs.  They
  stay where the run's own evidence is read.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np

from .declarations import DeclarationSet, OutputDeclaration
from .derived import BUILTIN, DerivedOutput, MinimumUnitOutputs

__all__ = [
    "ASSEMBLY_OUTPUTS",
    "DERIVED_OUTPUTS",
    "LEGACY_CLASSIFICATION",
    "RIG_OUTPUTS",
    "kc_minimum_unit_outputs",
    "minimum_unit_outputs",
    "peak",
    "register_builtins",
    "residual_norm",
    "rms",
    "time_metrics",
]

#: The four vehicle corners, in the order the legacy code uses.
_WHEELS: tuple[str, ...] = ("front_left", "front_right", "rear_left", "rear_right")

#: The tire force columns the legacy reader selected by index.
_TIRE_FORCES: tuple[str, ...] = (
    "tire_normal_force",
    "tire_longitudinal_force",
    "tire_lateral_force",
)

#: Which aggregate keys each tire force column feeds.
_FORCE_SUFFIX: dict[str, str] = {
    "tire_normal_force": "normal_force_n",
    "tire_longitudinal_force": "longitudinal_force_n",
    "tire_lateral_force": "lateral_force_n",
}

#: The K&C pose inputs, per side, in the legacy reader's spelling.
_UPRIGHT_ROTATION = {"left": "upright_left_rotation", "right": "upright_right_rotation"}
_UPRIGHT_TRANSLATION = {
    "left": "upright_left_translation",
    "right": "upright_right_translation",
}
_WHEEL_CENTER_LOCAL = {
    "left": "wheel_center_left_local",
    "right": "wheel_center_right_local",
}


# --- statistics, with the legacy definitions ---------------------------------
#
# Reimplemented rather than imported: `report/metrics` depends on this package
# and not the other way round, so a derived output that reached back into it
# would invert the dependency the architecture is built on.  The definitions are
# copied, and the value-for-value test is what proves they still agree.


def _finite_array(values: Any, *, name: str = "values") -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        raise ValueError(f"{name} must not be empty")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def peak(values: Any, *, absolute: bool = True) -> float:
    """Return the maximum value or maximum absolute value."""
    array = _finite_array(values)
    return float(np.max(np.abs(array) if absolute else array))


def rms(values: Any) -> float:
    """Return the root-mean-square value of a finite sample array."""
    array = _finite_array(values)
    return float(np.sqrt(np.mean(np.square(array))))


def residual_norm(values: Any, *, axis: int = -1) -> np.ndarray:
    """Return Euclidean residual norms along one sample axis."""
    array = _finite_array(values)
    return np.linalg.norm(array, axis=axis)


def time_metrics(result: Any) -> dict[str, float | int]:
    """Summarise a time grid, in the legacy shape."""
    times = _finite_array(result.times_s, name="result.times_s").reshape(-1)
    return {
        "sample_count": int(times.size),
        "start_s": float(times[0]),
        "end_s": float(times[-1]),
        "duration_s": float(times[-1] - times[0]),
    }


# --- declarations ------------------------------------------------------------


def _declaration(
    name: str,
    unit: str,
    dimension: str,
    shape: tuple[int | None, ...],
    text: str,
    *,
    domain: str = "assembly",
) -> OutputDeclaration:
    return OutputDeclaration(
        name=name,
        unit=unit,
        dimension=dimension,
        domain=domain,
        shape=shape,
        description=text,
    )


#: What the assembly's subsystems produce.  The tire force columns are the three
#: the legacy reader indexed out of `tire_output`; the poses are what the K&C
#: metrics read; the rest are the run-level facts the report passes through.
ASSEMBLY_OUTPUTS = DeclarationSet(
    domain="assembly",
    outputs=(
        _declaration("time_s", "s", "time", (None,), "the accepted sample time grid"),
        *(
            _declaration(name, "N", "force", (None, None), f"tire {name} column")
            for name in _TIRE_FORCES
        ),
        *(
            _declaration(
                name, "-", "orientation", (3, 3), f"upright {side} pose rotation"
            )
            for side, name in _UPRIGHT_ROTATION.items()
        ),
        *(
            _declaration(
                name, "mm", "length", (3,), f"upright {side} pose translation"
            )
            for side, name in _UPRIGHT_TRANSLATION.items()
        ),
        *(
            _declaration(
                name, "mm", "length", (3,), f"wheel centre on the {side} upright"
            )
            for side, name in _WHEEL_CENTER_LOCAL.items()
        ),
        _declaration(
            "diagnostic_accepted",
            "bool",
            "count",
            (None,),
            "per-sample acceptance flags from the solver ledger",
        ),
        _declaration(
            "diagnostic_rejected_attempts",
            "count",
            "count",
            (None,),
            "per-sample rejected step attempts",
        ),
        _declaration(
            "diagnostic_newton_iterations",
            "count",
            "count",
            (None,),
            "per-sample Newton iterations",
        ),
        _declaration(
            "diagnostic_active_contacts",
            "count",
            "count",
            (None,),
            "per-sample active contact count",
        ),
        _declaration(
            "diagnostic_contact_events",
            "count",
            "count",
            (None,),
            "per-sample contact event count",
        ),
    ),
)

#: What the rig's own driving and instrumentation adds.  A wheel load and a
#: steering output are measured at the rig, not produced by the suspension.
RIG_OUTPUTS = DeclarationSet(
    domain="rig",
    outputs=(
        _declaration(
            "steering_output",
            "rad",
            "angle",
            (None, None),
            "steering angle samples",
            domain="rig",
        ),
        *(
            _declaration(
                f"wheel_load_{wheel}",
                "N",
                "force",
                (),
                f"{wheel} normal load",
                domain="rig",
            )
            for wheel in _WHEELS
        ),
        _declaration(
            "native_kernel_wall_time_s",
            "s",
            "time",
            (),
            "kernel wall-clock time",
            domain="rig",
        ),
        _declaration(
            "diagnostics_available",
            "bool",
            "count",
            (),
            "diagnostics flag",
            domain="rig",
        ),
    ),
)


# --- expressions -------------------------------------------------------------
#
# One expression per shape of computation, called with the parameters the
# declaration supplies.  Every one reads its inputs through the restricted view,
# so a read of something the declaration did not list fails there and then.


def sample_peak(view: MinimumUnitOutputs, *, name: str) -> float:
    """Return the peak of one declared output, absolute by default."""
    return peak(view[name])


def sample_rms(view: MinimumUnitOutputs, *, name: str) -> float:
    """Return the RMS of one declared output."""
    return rms(view[name])


def time_field(view: MinimumUnitOutputs, *, field: str) -> float | int:
    """Return one of the four time-grid summaries, without fabricating samples."""
    times = _finite_array(view["time_s"], name="result.times_s").reshape(-1)
    values = time_metrics(_TimesView(times))
    return values[field]


class _TimesView:
    """A minimal `times_s` carrier, so `time_metrics` reads one shape only."""

    def __init__(self, times: np.ndarray) -> None:
        self.times_s = times


def _accepted(view: MinimumUnitOutputs) -> np.ndarray:
    return np.asarray(view["diagnostic_accepted"], dtype=bool)


def convergence_field(view: MinimumUnitOutputs, *, field: str) -> float | int | bool:
    """Reproduce one `convergence_metrics` entry, including its availability rule."""
    if not bool(view["diagnostics_available"]):
        return field == "available" and False
    accepted = _accepted(view)
    if accepted.size == 0:
        return field == "available" and False
    rejected = np.asarray(view["diagnostic_rejected_attempts"], dtype=float)
    newton = np.asarray(view["diagnostic_newton_iterations"], dtype=float)
    if field == "available":
        return True
    if field == "accepted_fraction":
        return float(np.mean(accepted))
    if field == "accepted_sample_count":
        return int(np.count_nonzero(accepted))
    if field == "rejected_attempt_count":
        return int(np.sum(rejected))
    if field == "maximum_newton_iterations":
        return int(np.max(newton))
    raise ValueError(f"unknown convergence field {field!r}")


def contact_field(view: MinimumUnitOutputs, *, field: str) -> float | int | bool:
    """Reproduce one `contact_metrics` entry, including its availability rule."""
    if not bool(view["diagnostics_available"]):
        return field == "available" and False
    if field == "available":
        return True
    active = np.asarray(view["diagnostic_active_contacts"], dtype=float)
    events = np.asarray(view["diagnostic_contact_events"], dtype=float)
    if field == "maximum_active_contact_count":
        return int(np.max(active)) if active.size else 0
    if field == "contact_event_count":
        return int(np.sum(events)) if events.size else 0
    raise ValueError(f"unknown contact field {field!r}")


def _loads(view: MinimumUnitOutputs) -> dict[str, float]:
    """Read the four corner loads, applying the legacy finiteness rule."""
    values = {wheel: float(view[f"wheel_load_{wheel}"]) for wheel in _WHEELS}
    if any(not np.isfinite(value) for value in values.values()):
        raise ValueError("wheel loads must be finite")
    return values


def load_scalar(view: MinimumUnitOutputs, *, field: str) -> float:
    """Return one `wheel_load_metrics` entry, per corner or aggregated."""
    values = _loads(view)
    front = values["front_left"] + values["front_right"]
    rear = values["rear_left"] + values["rear_right"]
    left = values["front_left"] + values["rear_left"]
    right = values["front_right"] + values["rear_right"]
    table: dict[str, float] = {
        **{f"normal_load_{wheel}": value for wheel, value in values.items()},
        "normal_load_total": front + rear,
        "normal_load_front_axle": front,
        "normal_load_rear_axle": rear,
        "normal_load_left_side": left,
        "normal_load_right_side": right,
        "load_transfer_front_minus_rear": front - rear,
        "load_transfer_right_minus_left": right - left,
    }
    return table[field]


def steering_peak(view: MinimumUnitOutputs, *, absolute: bool = True) -> float:
    """Return the peak of the finite steering samples, as the legacy code did."""
    values = np.asarray(view["steering_output"], dtype=float)
    finite = values[np.isfinite(values)]
    return peak(finite, absolute=absolute)


def steering_rms(view: MinimumUnitOutputs) -> float:
    """Return the RMS of the finite steering samples."""
    values = np.asarray(view["steering_output"], dtype=float)
    finite = values[np.isfinite(values)]
    return rms(finite)


def wall_time(view: MinimumUnitOutputs) -> float:
    """Pass through the kernel wall time the rig measured."""
    return float(view["native_kernel_wall_time_s"])


# --- K&C geometry, in the legacy sign conventions -----------------------------


def wheel_center_component(
    view: MinimumUnitOutputs, *, rotation: str, translation: str, local: str, axis: int
) -> float:
    """Return one axis of the wheel centre the upright pose places."""
    matrix = np.asarray(view[rotation], dtype=float)
    origin = np.asarray(view[translation], dtype=float)
    center_local = np.asarray(view[local], dtype=float)
    return float((origin + matrix @ center_local)[axis])


def wheel_camber(view: MinimumUnitOutputs, *, rotation: str, outward: float) -> float:
    """Return the camber angle with the legacy left/right sign convention."""
    matrix = np.asarray(view[rotation], dtype=float)
    return -outward * float(
        np.degrees(np.arctan2(float(matrix[2, 1]), float(matrix[1, 1])))
    )


def wheel_toe(view: MinimumUnitOutputs, *, rotation: str, outward: float) -> float:
    """Return the toe angle with the legacy left/right sign convention."""
    matrix = np.asarray(view[rotation], dtype=float)
    return -outward * float(
        np.degrees(np.arctan2(float(matrix[0, 1]), float(matrix[1, 1])))
    )


def difference(view: MinimumUnitOutputs, *, a: str, b: str) -> float:
    """Return `view[a] - view[b]`."""
    return float(view[a]) - float(view[b])


def mean_of(view: MinimumUnitOutputs, *, a: str, b: str) -> float:
    """Return the mean of two declared outputs."""
    return 0.5 * (float(view[a]) + float(view[b]))


#: The K&C derived outputs: five per side, then the five axle-level values.
_KC_SIDE_OUTPUTS: tuple[tuple[str, str, int], ...] = (
    ("wheel_center_x", "x", 0),
    ("wheel_center_y", "y", 1),
    ("wheel_center_z", "z", 2),
)
_KC_AXLE_OUTPUTS: tuple[tuple[str, str, str], ...] = (
    ("track_mm", "right_wheel_center_y", "left_wheel_center_y"),
    ("wheel_center_z_difference", "right_wheel_center_z", "left_wheel_center_z"),
    ("camber_deg_difference", "right_camber_deg", "left_camber_deg"),
    ("toe_deg_difference", "right_toe_deg", "left_toe_deg"),
)

#: `side name -> the outward sign the legacy code used`.
_OUTWARD: dict[str, float] = {"left": -1.0, "right": 1.0}


def _kc_side_reads(side: str) -> tuple[str, ...]:
    return (_UPRIGHT_ROTATION[side], _UPRIGHT_TRANSLATION[side], _WHEEL_CENTER_LOCAL[side])


def _kc_outputs() -> list[DerivedOutput]:
    """Build the K&C derived outputs, per side then axle level."""
    outputs: list[DerivedOutput] = []
    for side, outward in _OUTWARD.items():
        rotation = _UPRIGHT_ROTATION[side]
        translation = _UPRIGHT_TRANSLATION[side]
        local = _WHEEL_CENTER_LOCAL[side]
        reads = _kc_side_reads(side)
        for suffix, _, axis in _KC_SIDE_OUTPUTS:
            outputs.append(
                DerivedOutput(
                    name=f"{side}_{suffix}",
                    unit="mm",
                    dimension="length",
                    expression="wheel_center_component",
                    reads=reads,
                    defaults={
                        "rotation": rotation,
                        "translation": translation,
                        "local": local,
                        "axis": axis,
                    },
                    legacy="report.metrics.case_specific.wheel_metrics",
                    description=f"{side} wheel centre {axis}",
                )
            )
        outputs.append(
            DerivedOutput(
                name=f"{side}_camber_deg",
                unit="deg",
                dimension="angle",
                expression="wheel_camber",
                reads=(rotation,),
                defaults={"rotation": rotation, "outward": outward},
                legacy="report.metrics.case_specific.wheel_metrics",
                description=f"{side} camber",
            )
        )
        outputs.append(
            DerivedOutput(
                name=f"{side}_toe_deg",
                unit="deg",
                dimension="angle",
                expression="wheel_toe",
                reads=(rotation,),
                defaults={"rotation": rotation, "outward": outward},
                legacy="report.metrics.case_specific.wheel_metrics",
                description=f"{side} toe",
            )
        )
    for name, a, b in _KC_AXLE_OUTPUTS:
        outputs.append(
            DerivedOutput(
                name=name,
                unit="mm" if name.startswith(("track", "wheel_center")) else "deg",
                dimension="length" if name.startswith(("track", "wheel_center")) else "angle",
                expression="difference",
                reads=(a, b),
                defaults={"a": a, "b": b},
                legacy="report.metrics.case_specific.compute_k_metrics",
                description=f"{name}, restated from the per-side outputs",
            )
        )
    outputs.append(
        DerivedOutput(
            name="wheel_center_z_mean",
            unit="mm",
            dimension="length",
            expression="mean_of",
            reads=("left_wheel_center_z", "right_wheel_center_z"),
            defaults={"a": "left_wheel_center_z", "b": "right_wheel_center_z"},
            legacy="report.metrics.case_specific.compute_k_metrics",
            description="mean wheel centre height",
        )
    )
    return outputs


# --- the restatement ---------------------------------------------------------


def _force_outputs() -> list[DerivedOutput]:
    outputs: list[DerivedOutput] = []
    for name in _TIRE_FORCES:
        suffix = _FORCE_SUFFIX[name]
        outputs.append(
            DerivedOutput(
                name=f"maximum_{suffix}",
                unit="N",
                dimension="force",
                expression="sample_peak",
                reads=(name,),
                defaults={"name": name},
                legacy="report.metrics.axle.compute_axle_metrics",
                description=f"peak {name}",
            )
        )
        outputs.append(
            DerivedOutput(
                name=f"rms_{suffix}",
                unit="N",
                dimension="force",
                expression="sample_rms",
                reads=(name,),
                defaults={"name": name},
                legacy="report.metrics.axle.compute_axle_metrics",
                description=f"RMS {name}",
            )
        )
    return outputs


def _time_outputs() -> list[DerivedOutput]:
    return [
        DerivedOutput(
            name=name,
            unit={"sample_count": "count", "start_s": "s", "end_s": "s", "duration_s": "s"}[
                name
            ],
            dimension="count" if name == "sample_count" else "time",
            expression="time_field",
            reads=("time_s",),
            defaults={"field": name},
            legacy="report.metrics.common.time_metrics",
            description=f"time grid {name}",
        )
        for name in ("sample_count", "start_s", "end_s", "duration_s")
    ]


def _convergence_outputs() -> list[DerivedOutput]:
    specification = {
        "available": ("bool", "count"),
        "accepted_fraction": ("-", "scalar"),
        "accepted_sample_count": ("count", "count"),
        "rejected_attempt_count": ("count", "count"),
        "maximum_newton_iterations": ("count", "count"),
    }
    return [
        DerivedOutput(
            name=f"convergence_{field}",
            unit=unit,
            dimension=dimension,
            expression="convergence_field",
            reads=(
                "diagnostics_available",
                "diagnostic_accepted",
                "diagnostic_rejected_attempts",
                "diagnostic_newton_iterations",
            ),
            defaults={"field": field},
            legacy="report.metrics.common.convergence_metrics",
            description=f"convergence {field}",
        )
        for field, (unit, dimension) in specification.items()
    ]


def _contact_outputs() -> list[DerivedOutput]:
    specification = {
        "available": ("bool", "count"),
        "maximum_active_contact_count": ("count", "count"),
        "contact_event_count": ("count", "count"),
    }
    return [
        DerivedOutput(
            name=f"contact_{field}",
            unit=unit,
            dimension=dimension,
            expression="contact_field",
            reads=(
                "diagnostics_available",
                "diagnostic_active_contacts",
                "diagnostic_contact_events",
            ),
            defaults={"field": field},
            legacy="report.metrics.common.contact_metrics",
            description=f"contact {field}",
        )
        for field, (unit, dimension) in specification.items()
    ]


def _load_outputs() -> list[DerivedOutput]:
    reads = tuple(f"wheel_load_{wheel}" for wheel in _WHEELS)
    names = (
        *(f"normal_load_{wheel}" for wheel in _WHEELS),
        "normal_load_total",
        "normal_load_front_axle",
        "normal_load_rear_axle",
        "normal_load_left_side",
        "normal_load_right_side",
        "load_transfer_front_minus_rear",
        "load_transfer_right_minus_left",
    )
    return [
        DerivedOutput(
            name=name,
            unit="N",
            dimension="force",
            expression="load_scalar",
            reads=reads,
            defaults={"field": name},
            legacy="report.metrics.vehicle.wheel_load_metrics",
            description=f"wheel load {name}",
        )
        for name in names
    ]


def _vehicle_outputs() -> list[DerivedOutput]:
    return [
        DerivedOutput(
            name="maximum_steering_output",
            unit="rad",
            dimension="angle",
            expression="steering_peak",
            reads=("steering_output",),
            legacy="report.metrics.vehicle.compute_vehicle_metrics",
            description="peak steering angle",
        ),
        DerivedOutput(
            name="rms_steering_output",
            unit="rad",
            dimension="angle",
            expression="steering_rms",
            reads=("steering_output",),
            legacy="report.metrics.vehicle.compute_vehicle_metrics",
            description="RMS steering angle",
        ),
        DerivedOutput(
            name="native_kernel_wall_time_s",
            unit="s",
            dimension="time",
            expression="wall_time",
            reads=("native_kernel_wall_time_s",),
            legacy="report.metrics.vehicle.compute_vehicle_metrics",
            description="kernel wall time, passed through from the rig",
        ),
    ]


#: Every derived output the moved metrics become.
DERIVED_OUTPUTS: tuple[DerivedOutput, ...] = tuple(
    _time_outputs()
    + _convergence_outputs()
    + _contact_outputs()
    + _force_outputs()
    + _load_outputs()
    + _vehicle_outputs()
    + _kc_outputs()
)

#: The 27 legacy functions of `report/metrics`, and what each becomes.  A
#: function is either restated by derived outputs, replaced by a declared
#: minimum-unit output, or kept as registry/dispatch surface with no numeric
#: result of its own.
LEGACY_CLASSIFICATION: dict[str, tuple[tuple[str, ...], str]] = {
    "report.metrics.axle._tire_column": (
        (),
        "private reader that indexed one column out of a decoded `tire_output`; "
        "the column is now a declared minimum-unit output, so nothing selects it",
    ),
    "report.metrics.axle.compute_axle_metrics": (
        ("maximum_normal_force_n", "rms_normal_force_n",
         "maximum_longitudinal_force_n", "rms_longitudinal_force_n",
         "maximum_lateral_force_n", "rms_lateral_force_n"),
        "restated by the six tire-force aggregates; it also composes the time, "
        "convergence and contact outputs and adds `status`/`performance_available`, "
        "which are availability facts rather than arithmetic",
    ),
    "report.metrics.axle.axle_metrics": (
        ("maximum_normal_force_n", "rms_normal_force_n",
         "maximum_longitudinal_force_n", "rms_longitudinal_force_n",
         "maximum_lateral_force_n", "rms_lateral_force_n"),
        "alias that delegates to `compute_axle_metrics`; restated by the same outputs",
    ),
    "report.metrics.case_specific.register_case_metric": (
        (),
        "registration API for family metric functions; produces no value",
    ),
    "report.metrics.case_specific.compute_case_metrics": (
        (),
        "dispatcher that looks a family implementation up and calls it; computes "
        "nothing itself",
    ),
    "report.metrics.case_specific.case_metric_for": (
        (),
        "family lookup API; produces no value",
    ),
    "report.metrics.case_specific.registered_case_metric_families": (
        (),
        "audit listing of the registered family names; produces no value",
    ),
    "report.metrics.case_specific.not_applicable_metrics": (
        (),
        "placeholder returned for the five families with no implementation; it "
        "explicitly fabricates no numeric key",
    ),
    "report.metrics.case_specific.wheel_metrics": (
        ("left_wheel_center_x", "left_wheel_center_y", "left_wheel_center_z",
         "left_camber_deg", "left_toe_deg",
         "right_wheel_center_x", "right_wheel_center_y", "right_wheel_center_z",
         "right_camber_deg", "right_toe_deg"),
        "one call produced five keys for one side; the ten outputs cover both "
        "sides, with the pose inputs declared per side",
    ),
    "report.metrics.case_specific.compute_k_metrics": (
        ("track_mm", "wheel_center_z_mean", "wheel_center_z_difference",
         "camber_deg_difference", "toe_deg_difference"),
        "composes the per-side wheel outputs and adds the five axle-level values, "
        "each restated here",
    ),
    "report.metrics.case_specific._summarize_series_values": (
        (),
        "summarises a per-sample metric series whose key names come from the "
        "sample metrics at run time; no fixed-name output can restate it, and its "
        "statistics reuse the same definitions",
    ),
    "report.metrics.case_specific._kc_quasi_static_metrics": (
        (),
        "family implementation: chooses the state/assembly or the replay-series "
        "path and composes their outputs; its replay summaries are per-sample-"
        "metric names, as above",
    ),
    "report.metrics.case_specific._axle_dynamic_metrics": (
        ("maximum_normal_force_n", "rms_normal_force_n",
         "maximum_longitudinal_force_n", "rms_longitudinal_force_n",
         "maximum_lateral_force_n", "rms_lateral_force_n"),
        "family hook delegating to `compute_axle_metrics`; restated by the same outputs",
    ),
    "report.metrics.case_specific._vehicle_dynamic_metrics": (
        ("maximum_steering_output", "rms_steering_output",
         "normal_load_total", "normal_load_front_axle", "normal_load_rear_axle",
         "normal_load_left_side", "normal_load_right_side",
         "load_transfer_front_minus_rear", "load_transfer_right_minus_left",
         "native_kernel_wall_time_s"),
        "family hook delegating to `compute_vehicle_metrics`; restated by the same outputs",
    ),
    "report.metrics.case_specific._register_defaults": (
        (),
        "module-level registration of the eight default families; produces no value",
    ),
    "report.metrics.common._finite_array": (
        (),
        "shared validation primitive (empty and non-finite rejection); "
        "reimplemented locally with the same messages",
    ),
    "report.metrics.common.peak": (
        (),
        "shared statistic; registered as the `sample_peak` expression",
    ),
    "report.metrics.common.rms": (
        (),
        "shared statistic; registered as the `sample_rms` expression",
    ),
    "report.metrics.common.residual_norm": (
        (),
        "shared statistic with no caller among the report metrics; reimplemented "
        "with the same semantics",
    ),
    "report.metrics.common.time_metrics": (
        ("sample_count", "start_s", "end_s", "duration_s"),
        "restated by the four time-grid outputs",
    ),
    "report.metrics.common.convergence_metrics": (
        ("convergence_available", "convergence_accepted_fraction",
         "convergence_accepted_sample_count", "convergence_rejected_attempt_count",
         "convergence_maximum_newton_iterations"),
        "restated by the five convergence outputs, including the availability rule",
    ),
    "report.metrics.common.contact_metrics": (
        ("contact_available", "contact_maximum_active_contact_count",
         "contact_contact_event_count"),
        "restated by the three contact outputs, including the availability rule",
    ),
    "report.metrics.common.compute_common_metrics": (
        ("sample_count", "start_s", "end_s", "duration_s",
         "convergence_available", "contact_available"),
        "composes the time, convergence and contact outputs and adds `status`; the "
        "no-samples branch is a dispatch decision",
    ),
    "report.metrics.vehicle._embedded_wheel_loads": (
        (),
        "reads wheel loads off a result object; the loads are now declared "
        "minimum-unit outputs",
    ),
    "report.metrics.vehicle.wheel_load_metrics": (
        ("normal_load_front_left", "normal_load_front_right",
         "normal_load_rear_left", "normal_load_rear_right", "normal_load_total",
         "normal_load_front_axle", "normal_load_rear_axle", "normal_load_left_side",
         "normal_load_right_side", "load_transfer_front_minus_rear",
         "load_transfer_right_minus_left"),
        "restated by the eleven wheel-load outputs; the four-corner check becomes "
        "the declared input set",
    ),
    "report.metrics.vehicle.compute_vehicle_metrics": (
        ("maximum_steering_output", "rms_steering_output",
         "normal_load_total", "native_kernel_wall_time_s"),
        "composes the axle outputs under an `axle_` prefix, the wheel-load outputs, "
        "the steering aggregates and the wall time; the prefixing and the absent-"
        "field branches are dispatch decisions",
    ),
    "report.metrics.vehicle.vehicle_metrics": (
        ("maximum_steering_output", "rms_steering_output", "normal_load_total"),
        "alias that delegates to `compute_vehicle_metrics`; restated by the same outputs",
    ),
}


def register_builtins() -> None:
    """
    Register the built-in expressions and derived outputs, idempotently.

    `replace=True` because this runs at import and a re-import must not fail;
    the contents are the module's own, so replacing them is a no-op in effect.
    """
    for name, expression in (
        ("sample_peak", sample_peak),
        ("sample_rms", sample_rms),
        ("time_field", time_field),
        ("convergence_field", convergence_field),
        ("contact_field", contact_field),
        ("load_scalar", load_scalar),
        ("steering_peak", steering_peak),
        ("steering_rms", steering_rms),
        ("wall_time", wall_time),
        ("wheel_center_component", wheel_center_component),
        ("wheel_camber", wheel_camber),
        ("wheel_toe", wheel_toe),
        ("difference", difference),
        ("mean_of", mean_of),
    ):
        BUILTIN.register_expression(name, expression, replace=True)
    for output in DERIVED_OUTPUTS:
        BUILTIN.register(output, replace=True)


register_builtins()


# --- the adapter from a decoded result to minimum-unit outputs ----------------


def kc_minimum_unit_outputs(state: Any, assembly: Any, side: str) -> dict[str, Any]:
    """
    Return the minimum-unit pose outputs for one side of a K&C state.

    A K&C state and its assembly are *inputs* to the reading, not outputs of a
    run; this is where their pose and wheel centre become declared outputs, so the
    geometry restatements read arrays rather than reach into a live model.
    """
    normalized = str(side).strip().upper()
    if normalized not in {"L", "R"}:
        raise ValueError(f"unknown wheel side {side!r}")
    name = "left" if normalized == "L" else "right"
    pose = state.pose(f"upright_{normalized}")
    return {
        _UPRIGHT_ROTATION[name]: np.asarray(pose.rotation, dtype=float),
        _UPRIGHT_TRANSLATION[name]: np.asarray(pose.translation, dtype=float),
        _WHEEL_CENTER_LOCAL[name]: np.asarray(
            assembly.point(f"upright_{normalized}", "wheel_center"), dtype=float
        ),
    }


def minimum_unit_outputs(
    result: Any,
    *,
    state: Any = None,
    assembly: Any = None,
    wheel_loads: Mapping[str, float] | None = None,
) -> dict[str, Any]:
    """
    Read a decoded result into the declared minimum-unit outputs.

    This is the boundary the layer needs: everything downstream is arithmetic
    over this mapping.  An input the result did not carry is *absent* rather than
    zero -- a derived output that needs it then refuses to evaluate, which is what
    keeps "the run produced this" distinguishable from "this happens to be 0.0".
    """
    values: dict[str, Any] = {}
    times = getattr(result, "times_s", None)
    if times is not None:
        values["time_s"] = np.asarray(times, dtype=float).reshape(-1)
    tire_output = getattr(result, "tire_output", None)
    if tire_output is not None:
        array = np.asarray(tire_output, dtype=float)
        if array.ndim == 3:
            for name, column in zip(_TIRE_FORCES, (4, 5, 6)):
                if array.shape[2] > column and array.shape[1] != 0:
                    values[name] = array[:, :, column]
    diagnostics = getattr(result, "diagnostics", None)
    values["diagnostics_available"] = diagnostics is not None
    if diagnostics is not None:
        if hasattr(diagnostics, "accepted"):
            values["diagnostic_accepted"] = np.asarray(diagnostics.accepted, dtype=bool)
            values["diagnostic_rejected_attempts"] = np.asarray(
                diagnostics.rejected_attempts, dtype=float
            )
            values["diagnostic_newton_iterations"] = np.asarray(
                diagnostics.newton_iterations, dtype=float
            )
        else:
            rows = np.asarray(diagnostics, dtype=float)
            if rows.ndim == 2 and rows.shape[1] >= 4:
                values["diagnostic_accepted"] = rows[:, 0] > 0.5
                values["diagnostic_rejected_attempts"] = rows[:, 2]
                values["diagnostic_newton_iterations"] = rows[:, 3]
            if rows.ndim == 2 and rows.shape[1] >= 12:
                values["diagnostic_active_contacts"] = rows[:, 10]
                values["diagnostic_contact_events"] = rows[:, 11]
        if hasattr(diagnostics, "active_contacts"):
            values["diagnostic_active_contacts"] = np.asarray(
                diagnostics.active_contacts, dtype=float
            )
            values["diagnostic_contact_events"] = np.asarray(
                diagnostics.contact_events, dtype=float
            )
    steering = getattr(result, "steering_output", None)
    if steering is not None:
        values["steering_output"] = np.asarray(steering, dtype=float)
    wall = getattr(result, "native_kernel_wall_time_s", None)
    if wall is not None:
        values["native_kernel_wall_time_s"] = float(wall)
    if wheel_loads is not None:
        for wheel in _WHEELS:
            if wheel in wheel_loads:
                values[f"wheel_load_{wheel}"] = float(wheel_loads[wheel])
    if state is not None and assembly is not None:
        for side in ("L", "R"):
            values.update(kc_minimum_unit_outputs(state, assembly, side))
    return values


def declared_names(*sets: DeclarationSet) -> Sequence[str]:
    """Return the declared minimum-unit output names across the given sets."""
    return tuple(name for declaration_set in sets for name in declaration_set.names())
