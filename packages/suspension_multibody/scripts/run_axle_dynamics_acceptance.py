"""
Run the frozen axle case matrix and report convergence and performance.

This script exercises only this solver. It never claims Adams accuracy: the
5 percent and wall-time gates require real Adams evidence, so when no Adams
evidence directory is supplied the report is explicitly BLOCKED for those
gates while still reporting this solver's own convergence and timing.
"""

from __future__ import annotations

import argparse
import json
import platform
import statistics
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any, cast

import numpy as np

from suspension_multibody.adams import (
    AxleChannelBindings,
    AxleMarkerBinding,
    compare_axle_evidence,
    create_dynamic_axle_manifest,
    load_axle_acceptance_contract,
    run_native_axle_manifest,
    write_dynamic_axle_manifest,
)
from suspension_multibody.axle_dynamics import (
    AxleBody,
    AxleBushing,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleJoint,
    AxleSolverSettings,
    AxleSpringDamper,
    AxleTire,
    NativeAxleError,
    native_build_metadata,
    run_axle_dynamics,
)

# Synthetic, clearly labelled parameters. SPEC forbids using these for any
# formal Adams accuracy conclusion; they exist to exercise the solver.
PARAMETER_PROVENANCE = "synthetic_labelled_not_for_formal_adams_accuracy"

_TRACK_HALF_M = 0.75
_WHEEL_CENTER_Z_M = 0.32
_TIRE_RADIUS_M = 0.32
_SPRUNG_Z_M = 0.55
_GRAVITY = 9.80665


def _inertia(
    xx: float, yy: float, zz: float
) -> tuple[tuple[float, float, float], ...]:
    return ((xx, 0.0, 0.0), (0.0, yy, 0.0), (0.0, 0.0, zz))


def _diagonal6(values: Sequence[float]) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(values[row] if row == column else 0.0 for column in range(6))
        for row in range(6)
    )


def build_axle_model() -> AxleDynamicsModel:
    """Build the synthetic twin-strut axle used by every frozen case."""
    bodies: list[AxleBody] = [
        AxleBody(
            name="fixture",
            mass_kg=0.0,
            inertia_kg_m2=_inertia(0.0, 0.0, 0.0),
            fixed=True,
        ),
        AxleBody(
            name="sprung",
            mass_kg=600.0,
            inertia_kg_m2=_inertia(180.0, 60.0, 200.0),
            position_m=(0.0, 0.0, _SPRUNG_Z_M),
        ),
    ]
    joints: list[AxleJoint] = []
    springs: list[AxleSpringDamper] = []
    tires: list[AxleTire] = []
    for side, sign in (("l", -1.0), ("r", 1.0)):
        lateral = sign * _TRACK_HALF_M
        bodies.append(
            AxleBody(
                name=f"carrier_{side}",
                mass_kg=18.0,
                inertia_kg_m2=_inertia(0.4, 0.4, 0.4),
                position_m=(0.0, lateral, _WHEEL_CENTER_Z_M),
            )
        )
        bodies.append(
            AxleBody(
                name=f"wheel_{side}",
                mass_kg=22.0,
                inertia_kg_m2=_inertia(0.7, 1.1, 0.7),
                position_m=(0.0, lateral, _WHEEL_CENTER_Z_M),
            )
        )
        joints.append(
            AxleJoint(
                name=f"strut_{side}",
                kind="prismatic",
                body_a="sprung",
                body_b=f"carrier_{side}",
                point_a_m=(0.0, lateral, _WHEEL_CENTER_Z_M - _SPRUNG_Z_M),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 0.0, 1.0),
                axis_b=(0.0, 0.0, 1.0),
            )
        )
        joints.append(
            AxleJoint(
                name=f"spin_{side}",
                kind="revolute",
                body_a=f"carrier_{side}",
                body_b=f"wheel_{side}",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 1.0, 0.0),
                axis_b=(0.0, 1.0, 0.0),
            )
        )
        springs.append(
            AxleSpringDamper(
                name=f"spring_{side}",
                body_a="sprung",
                body_b=f"wheel_{side}",
                point_a_m=(
                    0.0,
                    lateral,
                    _WHEEL_CENTER_Z_M - _SPRUNG_Z_M + 0.30,
                ),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=32_000.0,
                compression_damping_n_s_per_m=2600.0,
                rebound_damping_n_s_per_m=3800.0,
                free_length_m=0.40,
                minimum_length_m=0.20,
                maximum_length_m=0.42,
                compression_stop_stiffness_n_per_m=400_000.0,
                compression_stop_damping_n_s_per_m=2000.0,
                rebound_stop_stiffness_n_per_m=300_000.0,
                rebound_stop_damping_n_s_per_m=1500.0,
            )
        )
        tires.append(
            AxleTire(
                name=f"tire_{side}",
                body=f"wheel_{side}",
                unloaded_radius_m=_TIRE_RADIUS_M,
                maximum_compression_m=0.05,
                vertical_stiffness_n_per_m=260_000.0,
                vertical_damping_n_s_per_m=800.0,
                longitudinal_friction_coefficient=1.0,
                lateral_friction_coefficient=0.95,
                longitudinal_brush_stiffness_n_per_m=180_000.0,
                lateral_brush_stiffness_n_per_m=120_000.0,
                longitudinal_relaxation_length_m=0.25,
                lateral_relaxation_length_m=0.35,
                detached_relaxation_s=0.05,
            )
        )
    # The rig restrains the non-suspension directions of the sprung body so the
    # static problem is determinate; heave and roll stay free.
    restraint = AxleBushing(
        name="rig_restraint",
        body_a="fixture",
        body_b="sprung",
        point_a_m=(0.0, 0.0, _SPRUNG_Z_M),
        point_b_m=(0.0, 0.0, 0.0),
        reference_translation_in_frame_a_m=(0.0, 0.0, 0.0),
        reference_quaternion_a_to_b=(1.0, 0.0, 0.0, 0.0),
        stiffness=_diagonal6((4.0e6, 4.0e6, 0.0, 0.0, 4.0e6, 4.0e6)),
        damping=_diagonal6((2.0e4, 2.0e4, 0.0, 0.0, 2.0e4, 2.0e4)),
    )
    return AxleDynamicsModel(
        name="synthetic-twin-strut-axle",
        bodies=tuple(bodies),
        joints=tuple(joints),
        springs=tuple(springs),
        bushings=(restraint,),
        tires=tuple(tires),
    )


def build_bindings() -> AxleChannelBindings:
    """Bind every frozen channel role explicitly."""
    return AxleChannelBindings(
        sprung_body="sprung",
        fixture_reference_marker=AxleMarkerBinding(
            body="fixture", point_local_m=(0.0, 0.0, 0.0)
        ),
        left_wheel_center_marker=AxleMarkerBinding(
            body="wheel_l", point_local_m=(0.0, 0.0, 0.0)
        ),
        right_wheel_center_marker=AxleMarkerBinding(
            body="wheel_r", point_local_m=(0.0, 0.0, 0.0)
        ),
        left_wheel_spin_joint="spin_l",
        right_wheel_spin_joint="spin_r",
        left_spring="spring_l",
        right_spring="spring_r",
        left_damper="spring_l",
        right_damper="spring_r",
        left_tire="tire_l",
        right_tire="tire_r",
    )


_SINE_FREQUENCY_HZ = 8.0
# The frozen matrix classifies large_amplitude_high_frequency as `smooth`, which
# ACCEPTANCE.yaml defines as no contact active-set change. A sinusoidal road can
# only keep the tire loaded while its peak acceleration stays under
# g + static_preload/unsprung_mass, about 93 m/s^2 for this model. 12 mm at 8 Hz
# gives 30 m/s^2 and the largest wheel travel that still holds contact; a larger
# amplitude would lift the wheel and belong to the contact-event class instead.
_HIGH_FREQUENCY_HZ = 8.0
_HIGH_FREQUENCY_AMPLITUDE_M = 0.012


def _grid(duration_s: float) -> tuple[float, ...]:
    """Build the frozen 1 kHz public output grid."""
    count = int(round(duration_s * 1000.0)) + 1
    return tuple(index * 0.001 for index in range(count))


def _ramp(times: np.ndarray, start_s: float, rise_s: float) -> np.ndarray:
    return np.clip((times - start_s) / rise_s, 0.0, 1.0)


def _road_signals(
    case_name: str, times: np.ndarray
) -> tuple[dict[str, tuple[float, ...]], dict[str, tuple[float, ...]]]:
    """Return position-continuous road height and its exact velocity."""
    height: dict[str, np.ndarray] = {}
    velocity: dict[str, np.ndarray] = {}
    zero = np.zeros_like(times)
    if case_name == "road_step_finite_rise":
        rise_s = 0.05
        shape = 0.02 * _ramp(times, 0.05, rise_s)
        rate = np.where(
            (times >= 0.05) & (times <= 0.05 + rise_s), 0.02 / rise_s, 0.0
        )
        height = {"tire_l": shape, "tire_r": shape}
        velocity = {"tire_l": rate, "tire_r": rate}
    elif case_name == "road_pulse":
        rise_s = 0.04
        shape = 0.02 * (
            _ramp(times, 0.05, rise_s) - _ramp(times, 0.05 + rise_s, rise_s)
        )
        rate = (
            0.02
            / rise_s
            * (
                ((times >= 0.05) & (times <= 0.05 + rise_s)).astype(float)
                - (
                    (times >= 0.05 + rise_s)
                    & (times <= 0.05 + 2.0 * rise_s)
                ).astype(float)
            )
        )
        height = {"tire_l": shape, "tire_r": shape}
        velocity = {"tire_l": rate, "tire_r": rate}
    elif case_name in {"road_sine", "large_amplitude_high_frequency"}:
        frequency = (
            _SINE_FREQUENCY_HZ
            if case_name == "road_sine"
            else _HIGH_FREQUENCY_HZ
        )
        amplitude = (
            0.01
            if case_name == "road_sine"
            else _HIGH_FREQUENCY_AMPLITUDE_M
        )
        angular = 2.0 * np.pi * frequency
        envelope = _ramp(times, 0.0, 0.05)
        shape = amplitude * envelope * np.sin(angular * times)
        rate = amplitude * (
            envelope * angular * np.cos(angular * times)
            + np.where(times <= 0.05, 1.0 / 0.05, 0.0)
            * np.sin(angular * times)
        )
        height = {"tire_l": shape, "tire_r": shape}
        velocity = {"tire_l": rate, "tire_r": rate}
    elif case_name == "single_wheel_road":
        rise_s = 0.05
        shape = 0.02 * _ramp(times, 0.05, rise_s)
        rate = np.where(
            (times >= 0.05) & (times <= 0.05 + rise_s), 0.02 / rise_s, 0.0
        )
        height = {"tire_l": shape, "tire_r": zero}
        velocity = {"tire_l": rate, "tire_r": zero}
    elif case_name == "in_phase_road":
        angular = 2.0 * np.pi * 5.0
        envelope = _ramp(times, 0.0, 0.05)
        shape = 0.015 * envelope * np.sin(angular * times)
        rate = 0.015 * (
            envelope * angular * np.cos(angular * times)
            + np.where(times <= 0.05, 1.0 / 0.05, 0.0)
            * np.sin(angular * times)
        )
        height = {"tire_l": shape, "tire_r": shape}
        velocity = {"tire_l": rate, "tire_r": rate}
    elif case_name == "opposite_phase_road":
        angular = 2.0 * np.pi * 5.0
        envelope = _ramp(times, 0.0, 0.05)
        shape = 0.015 * envelope * np.sin(angular * times)
        rate = 0.015 * (
            envelope * angular * np.cos(angular * times)
            + np.where(times <= 0.05, 1.0 / 0.05, 0.0)
            * np.sin(angular * times)
        )
        height = {"tire_l": shape, "tire_r": -shape}
        velocity = {"tire_l": rate, "tire_r": -rate}
    elif case_name == "tire_liftoff_and_recontact":
        # A downward road ramp withdraws support, then restores it. The drop and
        # the restore rate are bounded so recontact stays inside the frozen tire
        # maximum compression; a deeper drop is a model limit, not a solver one.
        fall_s = 0.05
        rise_s = 0.10
        restore_s = 0.05 + fall_s + 0.10
        drop = -0.06 * (
            _ramp(times, 0.05, fall_s) - _ramp(times, restore_s, rise_s)
        )
        rate = -0.06 * (
            np.where(
                (times >= 0.05) & (times <= 0.05 + fall_s), 1.0 / fall_s, 0.0
            )
            - np.where(
                (times >= restore_s) & (times <= restore_s + rise_s),
                1.0 / rise_s,
                0.0,
            )
        )
        height = {"tire_l": drop, "tire_r": drop}
        velocity = {"tire_l": rate, "tire_r": rate}
    elif case_name == "combined_load":
        rise_s = 0.04
        shape = 0.012 * _ramp(times, 0.05, rise_s)
        rate = np.where(
            (times >= 0.05) & (times <= 0.05 + rise_s), 0.012 / rise_s, 0.0
        )
        height = {"tire_l": shape, "tire_r": zero}
        velocity = {"tire_l": rate, "tire_r": zero}
    if not height:
        return {}, {}
    return (
        {name: tuple(float(v) for v in values) for name, values in height.items()},
        {
            name: tuple(float(v) for v in values)
            for name, values in velocity.items()
        },
    )


def _wheel_torque(
    case_name: str, times: np.ndarray
) -> dict[str, tuple[float, ...]]:
    """Return drive or brake torque on the wheel spin axis."""
    if case_name == "braking":
        shape = -600.0 * _ramp(times, 0.05, 0.05)
    elif case_name == "driving":
        shape = 600.0 * _ramp(times, 0.05, 0.05)
    elif case_name == "combined_load":
        shape = -300.0 * _ramp(times, 0.05, 0.05)
    else:
        return {}
    values = tuple(float(v) for v in shape)
    return {"tire_l": values, "tire_r": values}


def _body_wrench(
    case_name: str, times: np.ndarray
) -> dict[str, tuple[tuple[float, ...], ...]]:
    """Return the external rig wrench applied to the sprung body."""
    if case_name == "lateral_or_steering":
        lateral = 2500.0 * _ramp(times, 0.05, 0.05)
    elif case_name == "combined_load":
        lateral = 1500.0 * _ramp(times, 0.05, 0.05)
    else:
        return {}
    return {
        "sprung": tuple(
            (0.0, float(value), 0.0, 0.0, 0.0, 0.0) for value in lateral
        )
    }


_CASE_DURATIONS = {
    "static_equilibrium": 0.20,
    "road_step_finite_rise": 0.40,
    "road_pulse": 0.40,
    "road_sine": 1.60,
    "single_wheel_road": 0.40,
    "in_phase_road": 0.60,
    "opposite_phase_road": 0.60,
    "braking": 0.30,
    "driving": 0.30,
    "lateral_or_steering": 0.30,
    "combined_load": 0.30,
    "tire_liftoff_and_recontact": 0.45,
    "large_amplitude_high_frequency": 0.40,
}


def build_case(case_name: str) -> AxleDynamicsCase:
    """Build one frozen case on the public 1 kHz grid."""
    grid = _grid(_CASE_DURATIONS[case_name])
    times = np.asarray(grid, dtype=float)
    height, velocity = _road_signals(case_name, times)
    return AxleDynamicsCase(
        name=case_name,
        times_s=grid,
        road_height_m=height,
        road_velocity_m_per_s=velocity,
        wheel_torque_n_m=_wheel_torque(case_name, times),
        body_wrench_n_n_m=_body_wrench(case_name, times),
        solver=AxleSolverSettings(
            initialization_mode="static_equilibrium",
            adaptive_step=True,
            internal_step_s=0.00025,
            minimum_step_s=1e-6,
            maximum_step_s=0.001,
        ),
    )


def _execution_environment() -> dict[str, object]:
    return {
        "cpu_model": platform.processor() or platform.machine(),
        "physical_core_count": 0,
        "thread_count": 1,
        "process_affinity": "inherited",
        "platform": platform.platform(),
        "native_build": native_build_metadata(),
    }


def _measure_wall_times(
    model: AxleDynamicsModel,
    case: AxleDynamicsCase,
    performance: Mapping[str, object],
) -> dict[str, object]:
    """Time the solver exactly on the frozen performance boundary."""
    warmups = int(cast(int, performance["warmup_runs"]))
    measured = int(cast(int, performance["measured_runs"]))
    for _ in range(warmups):
        run_axle_dynamics(model, case)
    samples: list[float] = []
    for _ in range(measured):
        started = time.perf_counter()
        run_axle_dynamics(model, case)
        samples.append(time.perf_counter() - started)
    duration_s = case.times_s[-1] - case.times_s[0]
    median = statistics.median(samples)
    return {
        "statistic": performance["statistic"],
        "warmup_runs": warmups,
        "measured_runs": measured,
        "samples_s": samples,
        "median_wall_time_s": median,
        "physical_duration_s": duration_s,
        "realtime_ratio": median / duration_s,
        "boundary": {
            "include_model_build": performance["include_model_build"],
            "include_static_trim": performance["include_static_trim"],
            "include_result_serialization": performance[
                "include_result_serialization"
            ],
        },
    }


def _run_case(
    case_name: str,
    output_dir: Path,
    acceptance: Mapping[str, object],
    *,
    measure_performance: bool,
) -> dict[str, object]:
    """Run one frozen case and collect this solver's own gate evidence."""
    model = build_axle_model()
    case = build_case(case_name)
    metadata: dict[str, object] = {
        "parameter_provenance": PARAMETER_PROVENANCE
    }
    if case_name == "road_sine":
        metadata["harmonic_frequency_hz"] = _SINE_FREQUENCY_HZ
    manifest = create_dynamic_axle_manifest(
        model,
        case,
        build_bindings(),
        adams_solver={
            "integrator": "HHT",
            "alpha": -0.3,
            "error": 1e-8,
            "maximum_step_s": case.solver.maximum_step_s,
        },
        execution_environment=_execution_environment(),
        case_metadata=metadata,
    )
    case_dir = output_dir / case_name
    case_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = write_dynamic_axle_manifest(
        manifest, case_dir / "dynamic_axle_manifest.json"
    )
    record: dict[str, object] = {
        "case": case_name,
        "manifest_sha256": manifest.sha256,
        "manifest_path": str(manifest_path),
    }
    try:
        evidence_path = run_native_axle_manifest(
            manifest_path,
            case_dir / "native",
            producer_id=f"native-acceptance-{case_name}",
        )
    except NativeAxleError as error:
        record.update(
            {
                "status": "FAILED",
                "native_status": error.status,
                "error": str(error),
                "failed_time_s": error.failed_time_s,
                "failure_diagnostics": error.named_failure_diagnostics,
            }
        )
        return record
    evidence = json.loads(Path(evidence_path).read_text(encoding="utf-8"))
    diagnostics = cast(Mapping[str, object], evidence["diagnostics"])
    passed = bool(
        diagnostics["solver_internal_gates_passed"]
        and diagnostics["energy_gate_passed"]
        and diagnostics["time_convergence_passed"]
    )
    record.update(
        {
            "status": "PASSED" if passed else "FAILED",
            "evidence_path": str(evidence_path),
            "solver_internal": diagnostics["solver_internal"],
            "energy": diagnostics["energy"],
            "time_convergence": diagnostics["time_convergence"],
        }
    )
    if measure_performance:
        record["performance"] = _measure_wall_times(
            model,
            case,
            cast(Mapping[str, object], acceptance["performance"]),
        )
    return record


def main(argv: Sequence[str] | None = None) -> int:
    """Run the frozen case matrix and write the acceptance report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--adams-evidence",
        default=None,
        help=(
            "directory holding one real Adams evidence bundle per case; "
            "without it the Adams 5 percent and speed gates stay BLOCKED"
        ),
    )
    parser.add_argument("--case", action="append", default=None)
    parser.add_argument(
        "--performance",
        action="store_true",
        help=(
            "collect the frozen median-of-N timing protocol; each case is then "
            "solved warmup+measured extra times, so the matrix takes hours"
        ),
    )
    args = parser.parse_args(argv)

    acceptance = load_axle_acceptance_contract()
    matrix = [
        str(entry["name"])
        for entry in cast(list[dict[str, Any]], acceptance["case_matrix"])
    ]
    selected = args.case or matrix
    unknown = sorted(set(selected) - set(matrix))
    if unknown:
        raise SystemExit(f"cases outside the frozen matrix: {unknown}")

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    cases = [
        _run_case(
            name,
            output_dir,
            acceptance,
            measure_performance=args.performance,
        )
        for name in selected
    ]

    adams_reports: list[dict[str, object]] = []
    blockers: list[str] = []
    if args.adams_evidence is None:
        blockers.append(
            "no real Adams evidence supplied: the 5 percent channel gates and "
            "the wall-time ratio gate cannot be evaluated"
        )
    else:
        adams_root = Path(args.adams_evidence)
        for record in cases:
            name = str(record["case"])
            adams_path = adams_root / name / "axle_evidence.json"
            native_path = record.get("evidence_path")
            if not adams_path.is_file() or native_path is None:
                blockers.append(f"case {name} has no real Adams evidence")
                continue
            adams_reports.append(
                compare_axle_evidence(
                    manifest_path=Path(str(record["manifest_path"])),
                    adams_evidence_path=adams_path,
                    native_evidence_path=Path(str(native_path)),
                )
            )

    if not args.performance:
        blockers.append(
            "the frozen median-of-N timing protocol was not run: pass "
            "--performance to collect it"
        )

    solver_passed = all(record["status"] == "PASSED" for record in cases)

    adams_passed = bool(adams_reports) and all(
        report["status"] == "PASSED" for report in adams_reports
    )
    report: dict[str, object] = {
        "schema_version": 1,
        "report_type": "axle_dynamics_acceptance",
        "parameter_provenance": PARAMETER_PROVENANCE,
        "solver_self_convergence_status": (
            "PASSED" if solver_passed else "FAILED"
        ),
        "adams_accuracy_status": (
            "PASSED" if adams_passed else "BLOCKED" if blockers else "FAILED"
        ),
        "adams_speed_status": (
            "PASSED" if adams_passed else "BLOCKED" if blockers else "FAILED"
        ),
        "accuracy_claim_permitted": adams_passed,
        "frozen_timing_protocol_collected": bool(args.performance),
        "blockers": blockers,
        "cases": cases,
        "adams_comparisons": adams_reports,
        "execution_environment": _execution_environment(),
    }
    report_path = output_dir / "acceptance_report.json"
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True), encoding="utf-8"
    )
    print(f"solver self-convergence: {report['solver_self_convergence_status']}")
    print(f"adams accuracy: {report['adams_accuracy_status']}")
    for blocker in blockers:
        print(f"BLOCKED: {blocker}")
    for record in cases:
        print(f"  {record['case']}: {record['status']}")
    print(report_path)
    return 0 if solver_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
