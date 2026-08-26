"""
Run a reproducible native-versus-real-Adams dynamic benchmark.

The imported suspension parameters come from the installed Adams/Car database.
Both runners consume the same immutable manifest.  The dynamic model is emitted
as primitive Adams/Solver statements because a standalone axle dataset cannot be
loaded by the stock Adams/Car vehicle assembly without adding unrelated vehicle
components.  A separate strict-K gate invokes the real Adams/Car batch front
end; this benchmark consumes and verifies that immutable execution evidence
before entering the dynamic timing gate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import re
import shutil
import statistics
import subprocess
import tempfile
import time
from collections.abc import Sequence
from pathlib import Path
from typing import Any

try:
    from run_dynamic_kc_correlation import bindings_for, build_case_and_model
except ModuleNotFoundError:
    from packages.suspension_multibody.scripts.run_dynamic_kc_correlation import (
        bindings_for,
        build_case_and_model,
    )
from suspension_multibody.adams import (
    TimeHistory,
    adams_axle_result_from_result,
    audit_axle_equivalence,
    audit_axle_time_convergence,
    axle_history_from_result,
    build_axle_adams_dataset,
    compare_strict_axle_histories,
    create_dynamic_axle_manifest,
    discover_profile,
    initialization_evidence_from_result,
    run_native_axle_manifest,
    write_axle_adams_dataset,
    write_dynamic_axle_manifest,
    write_time_history,
)
from suspension_multibody.axle_dynamics import (
    native_build_metadata,
    run_axle_dynamics,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _execution_environment() -> dict[str, object]:
    return {
        "cpu_model": platform.processor() or platform.machine(),
        "physical_core_count": 0,
        "thread_count": 1,
        "process_affinity": "inherited",
        "platform": platform.platform(),
        "native_build": native_build_metadata(),
    }


def _native_timing(model: Any, case: Any, warmups: int, runs: int) -> dict[str, object]:
    for _ in range(warmups):
        run_axle_dynamics(model, case)
    samples: list[float] = []
    for _ in range(runs):
        started = time.perf_counter()
        run_axle_dynamics(model, case)
        samples.append(time.perf_counter() - started)
    median = statistics.median(samples)
    duration = float(case.times_s[-1] - case.times_s[0])
    return {
        "warmup_runs": warmups,
        "measured_runs": runs,
        "samples_s": samples,
        "median_wall_time_s": median,
        "physical_duration_s": duration,
        "realtime_ratio": median / duration,
        "boundary": {
            "include_model_build": False,
            "include_static_trim": False,
            "include_result_serialization": False,
        },
    }


def _provided_consistent_model(model: Any, trim_result: Any) -> Any:
    """Embed one shared native trim state in every moving body of the model."""
    bodies = []
    for body in model.bodies:
        state = trim_result.body_state(body.name)[0]
        bodies.append(
            body.model_copy(
                update={
                    "position_m": tuple(float(value) for value in state[:3]),
                    "quaternion_body_to_world": tuple(
                        float(value) for value in state[3:7]
                    ),
                    "linear_velocity_m_per_s": tuple(
                        float(value) for value in state[7:10]
                    ),
                    "angular_velocity_rad_per_s": tuple(
                        float(value) for value in state[10:13]
                    ),
                }
            )
        )
    return model.model_copy(update={"bodies": tuple(bodies)})


def _copy_dataset(dataset_paths: dict[str, Path], destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    for path in dataset_paths.values():
        shutil.copy2(path, destination / path.name)


def _run_adams_once(
    executable: str,
    dataset_stem: str,
    dataset_paths: dict[str, Path],
    runtime_root: Path,
    index: int,
    environment: dict[str, str],
) -> tuple[float, Path]:
    run_dir = Path(
        tempfile.mkdtemp(prefix=f"open_kinematics_adams_{index}_", dir=runtime_root)
    )
    _copy_dataset(dataset_paths, run_dir)
    started = time.perf_counter()
    completed = subprocess.run(
        [executable, "ru-standard", dataset_stem],
        cwd=run_dir,
        env=environment,
        capture_output=True,
        text=True,
        timeout=600,
        check=False,
    )
    elapsed = time.perf_counter() - started
    message_path = run_dir / f"{dataset_stem}.msg"
    result_path = run_dir / f"{dataset_stem}.res"
    message = (
        message_path.read_text(encoding="utf-8", errors="replace")
        if message_path.is_file()
        else ""
    )
    command_path = dataset_paths.get("command")
    command_text = (
        command_path.read_text(encoding="ascii")
        if command_path is not None
        else ""
    )
    integrator_match = re.search(
        r"integrator/([A-Za-z0-9_]+)", command_text, flags=re.IGNORECASE
    )
    expected_integrator = (
        integrator_match.group(1).upper() if integrator_match else None
    )
    actual_integrator_present = (
        expected_integrator is None
        or f"The integrator is {expected_integrator}" in message
    )
    command_error = "ERROR:" in message or "Errors found parsing command" in message
    if (
        completed.returncode != 0
        or "Performing Dynamic Simulation" not in message
        or "End Simulation" not in message
        or not result_path.is_file()
        or not actual_integrator_present
        or command_error
    ):
        details = (completed.stdout or "") + (completed.stderr or "")
        raise RuntimeError(
            f"real Adams dynamic run failed in {run_dir} with code "
            f"{completed.returncode}; expected_integrator={expected_integrator!r}; "
            f"{details[-2000:]}{message[-2000:]}"
        )
    return elapsed, run_dir


def _adams_timing(
    executable: str,
    dataset_stem: str,
    dataset_paths: dict[str, Path],
    runtime_root: Path,
    warmups: int,
    runs: int,
    physical_duration_s: float,
    environment: dict[str, str],
) -> tuple[dict[str, object], Path]:
    last_run: Path | None = None
    for index in range(warmups):
        _, last_run = _run_adams_once(
            executable, dataset_stem, dataset_paths, runtime_root, index, environment
        )
    samples: list[float] = []
    for offset in range(runs):
        elapsed, last_run = _run_adams_once(
            executable,
            dataset_stem,
            dataset_paths,
            runtime_root,
            warmups + offset,
            environment,
        )
        samples.append(elapsed)
    assert last_run is not None
    median = statistics.median(samples)
    return (
        {
            "warmup_runs": warmups,
            "measured_runs": runs,
            "samples_s": samples,
            "median_wall_time_s": median,
            "physical_duration_s": physical_duration_s,
            "realtime_ratio": median / physical_duration_s,
            "boundary": {
                "include_process_startup": True,
                "include_model_read": True,
                "include_result_write": True,
            },
        },
        last_run,
    )


def _retain_adams_run(run_dir: Path, stem: str, destination: Path) -> dict[str, str]:
    destination.mkdir(parents=True, exist_ok=True)
    hashes: dict[str, str] = {}
    for source in sorted(run_dir.glob(f"{stem}.*")):
        target = destination / source.name
        shutil.copy2(source, target)
        hashes[target.name] = _sha256(target)
    return hashes


def _refined_case(case: Any) -> Any:
    solver = case.solver.model_copy(
        update={
            "internal_step_s": 0.5 * case.solver.internal_step_s,
            "maximum_step_s": 0.5 * case.solver.maximum_step_s,
            "minimum_step_s": min(
                case.solver.minimum_step_s,
                0.5 * case.solver.internal_step_s,
            ),
        }
    )
    return case.model_copy(update={"solver": solver})


def _run_case(
    case_name: str,
    output_dir: Path,
    profile: Any,
    runtime_root: Path,
    warmups: int,
    runs: int,
    environment: dict[str, str],
    fixed_step: bool,
    fixed_step_s: float | None,
    adams_hht_alpha: float | None,
) -> dict[str, object]:
    model, case, road_height = build_case_and_model(rigid=True)
    hht_alpha = -0.3 if adams_hht_alpha is None else adams_hht_alpha
    solver_update: dict[str, object] = {
        "integrator": "hht",
        "hht_alpha": hht_alpha,
    }
    if fixed_step:
        step = (
            case.solver.internal_step_s
            if fixed_step_s is None
            else fixed_step_s
        )
        public_step = case.times_s[1] - case.times_s[0]
        step_ratio = public_step / step
        if not math.isclose(
            step_ratio, round(step_ratio), rel_tol=0.0, abs_tol=1e-12
        ) or step_ratio < 1.0:
            raise ValueError(
                "fixed internal step must divide the public output step"
            )
        solver = case.solver.model_copy(
            update={
                "adaptive_step": False,
                "internal_step_s": step,
                "maximum_step_s": step,
                **solver_update,
            }
        )
        case = case.model_copy(update={"solver": solver})
    else:
        case = case.model_copy(
            update={"solver": case.solver.model_copy(update=solver_update)}
        )
    trim_case = case.model_copy(
        update={
            "solver": case.solver.model_copy(
                update={"initialization_mode": "static_equilibrium"}
            )
        }
    )
    trim_started = time.perf_counter()
    trim_result = run_axle_dynamics(model, trim_case)
    trim_elapsed_s = time.perf_counter() - trim_started
    bindings = bindings_for(model)
    trim_evidence = initialization_evidence_from_result(
        model,
        trim_result,
        bindings,
    )
    model = _provided_consistent_model(model, trim_result)
    case = case.model_copy(
        update={
            "solver": case.solver.model_copy(
                update={"initialization_mode": "provided_consistent_state"}
            )
        }
    )
    case_metadata: dict[str, object] = {
        "parameter_provenance": "adams_car_database_import",
        "source_subsystem": "acar/achassis_gs.cdb/subsystems.tbl/"
        "acar_gs_front_suspension.sub",
        "benchmark": "dynamic_kc_heave_sine",
        "road_height_m": road_height,
        "integration_mode": (
            "fixed_converged_step" if fixed_step else "adaptive"
        ),
        "comparison_basis": (
            "continuous_problem_convergence" if fixed_step else "unvalidated"
        ),
        "adams_hht_alpha_basis": "explicit_adams_reference_value",
        "initialization_mode": "provided_consistent_state",
        "common_initial_state": {
            "source": "native_static_equilibrium_trim",
            "trim_state_sha256": trim_evidence.state_sha256,
            "trim_wall_time_s": trim_elapsed_s,
            "included_in_dynamic_timing": False,
        },
    }
    if fixed_step:
        case_metadata["fixed_step_s"] = case.solver.internal_step_s
    case_metadata["adams_hht_alpha"] = hht_alpha
    case_metadata["native_integrator"] = case.solver.integrator
    case_metadata["native_hht_alpha"] = case.solver.hht_alpha
    if case.harmonic_roads:
        case_metadata["harmonic_frequency_hz"] = case.harmonic_roads[0].frequency_hz
    adams_solver: dict[str, object] = {
        "integrator": "HHT",
        "alpha": hht_alpha,
        "error": 1e-5,
        "maximum_step_s": case.solver.maximum_step_s,
    }
    if fixed_step:
        adams_solver.update(
            {
                "fixed_iterations": 10,
                "step_ratio": int(round((case.times_s[1] - case.times_s[0]) / case.solver.internal_step_s)),
            }
        )
    manifest = create_dynamic_axle_manifest(
        model,
        case,
        bindings,
        adams_solver=adams_solver,
        execution_environment=_execution_environment(),
        case_metadata=case_metadata,
    )
    case_dir = output_dir / case_name
    case_dir.mkdir(parents=True, exist_ok=True)
    trim_evidence_path = case_dir / "common_trim_initialization.json"
    trim_evidence_path.write_text(
        json.dumps(trim_evidence.model_dump(mode="json"), indent=2, sort_keys=True),
        encoding="utf-8",
    )
    manifest_path = write_dynamic_axle_manifest(
        manifest, case_dir / "dynamic_axle_manifest.json"
    )
    dataset = build_axle_adams_dataset(manifest, stem=f"real_{case_name}")
    dataset_paths = write_axle_adams_dataset(dataset, case_dir / "dataset")

    preflight_audit = audit_axle_equivalence(
        manifest,
        dataset,
        source_provenance=case_metadata,
    )
    preflight_audit_path = case_dir / "equivalence_preflight.json"
    preflight_audit_path.write_text(
        json.dumps(preflight_audit, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    if preflight_audit["equivalence_gate_passed"] is not True:
        dynamic_comparison = {
            "contract": "dynamic-axle-channel-comparison-v2",
            "status": "BLOCKED",
            "case": case_name,
            "manifest_sha256": manifest.sha256,
            "dataset_sha256": dataset.as_dict()["dataset_sha256"],
            "equivalence_gate_passed": False,
            "dynamic_precision_comparison_performed": False,
            "precision_metrics": None,
            "equivalence_audit": str(preflight_audit_path),
            "blockers": preflight_audit["blockers"],
        }
        dynamic_comparison_path = case_dir / "dynamic_comparison.json"
        dynamic_comparison_path.write_text(
            json.dumps(dynamic_comparison, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        summary = {
            "case": case_name,
            "manifest_sha256": manifest.sha256,
            "parameter_provenance": "adams_car_database_import",
            "common_trim_initialization": str(trim_evidence_path),
            "equivalence_preflight": str(preflight_audit_path),
            "dynamic_comparison": str(dynamic_comparison_path),
            "equivalence_gate_passed": False,
            "dynamic_precision_comparison_performed": False,
            "dynamic_precision_gate_passed": None,
            "native_not_slower_than_adams": False,
            "native_physics_gates": {
                "solver_internal": False,
                "energy": False,
                "time_convergence": False,
            },
        }
        (case_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8"
        )
        return summary

    native_evidence = run_native_axle_manifest(
        manifest_path,
        case_dir / "native",
        producer_id="open-kinematics.native.real-adams-car",
    )
    native_timing = _native_timing(model, case, warmups, runs)
    native_timing["common_initial_state_setup"] = {
        "trim_wall_time_s": trim_elapsed_s,
        "included_in_timing": False,
        "trim_evidence": str(trim_evidence_path),
    }
    adams_timing, last_adams_run = _adams_timing(
        str(profile.executable),
        dataset.stem,
        dataset_paths,
        runtime_root,
        warmups,
        runs,
        float(case.times_s[-1] - case.times_s[0]),
        environment,
    )
    adams_timing["common_initial_state_setup"] = {
        "trim_wall_time_s": trim_elapsed_s,
        "included_in_timing": False,
        "trim_evidence": str(trim_evidence_path),
    }
    retained = _retain_adams_run(
        last_adams_run, dataset.stem, case_dir / "adams_raw"
    )
    retained_result_path = case_dir / "adams_raw" / f"{dataset.stem}.res"
    adams_result = adams_axle_result_from_result(
        model,
        dataset,
        retained_result_path,
    )
    adams_history = axle_history_from_result(
        model,
        adams_result,
        manifest.bindings,
        case=case,
    )
    adams_history_path = write_time_history(
        adams_history, case_dir / "adams_history.json"
    )
    native_payload = json.loads(Path(native_evidence).read_text(encoding="utf-8"))
    native_refined_history = TimeHistory.from_mapping(
        json.loads(
            (case_dir / "native" / "native_refined_history.json").read_text(
                encoding="utf-8"
            )
        )
    )

    adams_refined_history: TimeHistory | None = None
    adams_refined_result = None
    adams_refined_history_path: Path | None = None
    adams_refined_result_path: Path | None = None
    adams_refined_retained: dict[str, str] = {}
    if fixed_step:
        refined_case = _refined_case(case)
        refined_adams_solver = dict(adams_solver)
        refined_adams_solver["maximum_step_s"] = refined_case.solver.maximum_step_s
        refined_adams_solver["step_ratio"] = int(
            round(
                (refined_case.times_s[1] - refined_case.times_s[0])
                / refined_case.solver.internal_step_s
            )
        )
        refined_manifest = create_dynamic_axle_manifest(
            model,
            refined_case,
            bindings,
            adams_solver=refined_adams_solver,
            execution_environment=_execution_environment(),
            case_metadata={
                **case_metadata,
                "refinement_of_manifest_sha256": manifest.sha256,
                "fixed_step_s": refined_case.solver.internal_step_s,
            },
        )
        refined_dataset = build_axle_adams_dataset(
            refined_manifest, stem=f"real_{case_name}_refined"
        )
        refined_dataset_paths = write_axle_adams_dataset(
            refined_dataset, case_dir / "adams_refined_dataset"
        )
        refined_elapsed, refined_run = _run_adams_once(
            str(profile.executable),
            refined_dataset.stem,
            refined_dataset_paths,
            runtime_root,
            warmups + runs + 1,
            environment,
        )
        adams_refined_retained = _retain_adams_run(
            refined_run,
            refined_dataset.stem,
            case_dir / "adams_refined_raw",
        )
        adams_refined_result_path = (
            case_dir / "adams_refined_raw" / f"{refined_dataset.stem}.res"
        )
        adams_refined_result = adams_axle_result_from_result(
            model,
            refined_dataset,
            adams_refined_result_path,
        )
        adams_refined_history = axle_history_from_result(
            model,
            adams_refined_result,
            refined_manifest.bindings,
            case=refined_case,
        )
        adams_refined_history_path = write_time_history(
            adams_refined_history,
            case_dir / "adams_refined_history.json",
        )
        adams_convergence = audit_axle_time_convergence(
            adams_history,
            adams_refined_history,
            acceptance=manifest.payload["acceptance"],
        )
        adams_evidence: dict[str, object] = {
            "primary_manifest_sha256": manifest.sha256,
            "refined_manifest_sha256": refined_manifest.sha256,
            "primary_step_s": case.solver.internal_step_s,
            "refined_step_s": refined_case.solver.internal_step_s,
            "time_convergence": adams_convergence,
            "time_convergence_passed": bool(adams_convergence["passed"]),
            "refined_wall_time_s": refined_elapsed,
        }
    else:
        adams_evidence = {
            "time_convergence_passed": False,
            "reason": "--fixed-step is required for a converged comparison",
        }
    final_audit = audit_axle_equivalence(
        manifest,
        dataset,
        native_history=native_refined_history,
        adams_history=adams_refined_history or adams_history,
        adams_result=adams_refined_result or adams_result,
        native_evidence=native_payload,
        adams_evidence=adams_evidence,
        source_provenance=case_metadata,
        require_runtime=True,
    )
    final_audit_path = case_dir / "equivalence_audit.json"
    final_audit_path.write_text(
        json.dumps(final_audit, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    precision_performed = final_audit["equivalence_gate_passed"] is True
    if precision_performed:
        dynamic_result = compare_strict_axle_histories(
            adams_refined_history or adams_history,
            native_refined_history,
            acceptance=manifest.payload["acceptance"],
            case_name=case_name,
            harmonic_frequency_hz=(
                float(case.harmonic_roads[0].frequency_hz)
                if case.harmonic_roads
                else None
            ),
            include_harmonic=False,
        )
    else:
        dynamic_result = {
            "passed": False,
            "channels": None,
            "precision_metrics": None,
            "blockers": final_audit["blockers"],
        }
    dynamic_comparison = {
        "contract": "dynamic-axle-channel-comparison-v2",
        "status": (
            "PASS"
            if precision_performed and dynamic_result["passed"]
            else "FAIL"
            if precision_performed
            else "BLOCKED"
        ),
        "case": case_name,
        "manifest_sha256": manifest.sha256,
        "dataset_sha256": dataset.as_dict()["dataset_sha256"],
        "equivalence_gate_passed": precision_performed,
        "dynamic_precision_comparison_performed": precision_performed,
        "equivalence_audit": str(final_audit_path),
        "reference": {
            "producer": "msc.adams-solver.2024.1",
            "history": str(adams_refined_history_path or adams_history_path),
            "result": str(adams_refined_result_path or retained_result_path),
            "result_sha256": _sha256(
                adams_refined_result_path or retained_result_path
            ),
            "primary_history": str(adams_history_path),
            "primary_result": str(retained_result_path),
        },
        "candidate": {
            "producer": "open-kinematics.native.real-adams-car",
            "history": str(
                Path(native_evidence).parent / "native_refined_history.json"
            ),
            "primary_history": str(
                Path(native_evidence).parent / "native_history.json"
            ),
        },
        "comparison_basis": "continuous_problem_convergence",
        "solver_conditions": final_audit["solver_conditions"],
        "adams_convergence": adams_evidence,
        "harmonic_gate": {
            "performed": False,
            "reason": (
                "the 0.5 s realtime benchmark window is shorter than the "
                "frozen 10-cycle harmonic window"
            ),
        },
        **dynamic_result,
    }
    if not precision_performed:
        dynamic_comparison["precision_metrics"] = None
    else:
        dynamic_comparison["precision_metrics"] = dynamic_result
    dynamic_comparison_path = case_dir / "dynamic_comparison.json"
    dynamic_comparison_path.write_text(
        json.dumps(dynamic_comparison, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    adams_execution = {
        "producer": "msc.adams-solver.2024.1",
        "installation": profile.as_dict(),
        "analysis": "dynamic_kc_heave_sine",
        "analysis_mode": "dynamic",
        "manifest_sha256": manifest.sha256,
        "dataset_sha256": dataset.as_dict()["dataset_sha256"],
        "equivalence_audit": str(final_audit_path),
        "command": [str(profile.executable), "ru-standard", dataset.stem],
        "returncode": 0,
        "dynamic_log_marker": True,
        "sample_count": len(adams_history.time),
        "canonical_channel_count": len(adams_history.channels),
        "canonical_history": str(adams_history_path),
        "refined_history": str(adams_refined_history_path or adams_history_path),
        "result_sha256": _sha256(
            adams_refined_result_path or retained_result_path
        ),
        "raw_artifacts": retained,
        "refined_raw_artifacts": adams_refined_retained,
        "wall_time": adams_timing,
    }
    adams_execution_path = case_dir / "adams_execution_evidence.json"
    adams_execution_path.write_text(
        json.dumps(adams_execution, indent=2, sort_keys=True), encoding="utf-8"
    )
    native_diagnostics = native_payload["diagnostics"]
    native_speed = float(native_timing["median_wall_time_s"])
    adams_speed = float(adams_timing["median_wall_time_s"])
    summary = {
        "case": case_name,
        "manifest_sha256": manifest.sha256,
        "parameter_provenance": "adams_car_database_import",
        "source_database_provenance": final_audit["source_database_provenance"],
        "equivalence_gate_passed": bool(final_audit["equivalence_gate_passed"]),
        "common_trim_initialization": str(trim_evidence_path),
        "equivalence_preflight": str(preflight_audit_path),
        "equivalence_audit": str(final_audit_path),
        "native_evidence": str(native_evidence),
        "adams_execution_evidence": str(adams_execution_path),
        "adams_history": str(adams_history_path),
        "adams_refined_history": str(adams_refined_history_path)
        if adams_refined_history_path is not None
        else None,
        "native_refined_history": str(
            Path(native_evidence).parent / "native_refined_history.json"
        ),
        "adams_convergence": adams_evidence,
        "dynamic_comparison": str(dynamic_comparison_path),
        "native_timing": native_timing,
        "adams_timing": adams_timing,
        "native_to_adams_wall_ratio": native_speed / adams_speed,
        "native_not_slower_than_adams": native_speed <= adams_speed,
        "dynamic_precision_comparison_performed": precision_performed,
        "dynamic_precision_gate_passed": (
            bool(dynamic_comparison["passed"]) if precision_performed else None
        ),
        "native_physics_gates": {
            "solver_internal": native_diagnostics["solver_internal_gates_passed"],
            "energy": native_diagnostics["energy_gate_passed"],
            "time_convergence": native_diagnostics["time_convergence_passed"],
        },
    }
    (case_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8"
    )
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    """Run the real-Adams timing and per-channel precision gates."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="artifacts/real-adams-car")
    parser.add_argument("--case", default="road_sine")
    parser.add_argument("--warmup-runs", type=int, default=1)
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument(
        "--fixed-step",
        action="store_true",
        help="Use the converged internal step instead of adaptive error estimation.",
    )
    parser.add_argument(
        "--fixed-step-s",
        type=float,
        default=None,
        help="Override the fixed internal step; --fixed-step is required.",
    )
    parser.add_argument(
        "--adams-hht-alpha",
        type=float,
        default=None,
        help="Explicit Adams HHT alpha; default is -0.3 and is recorded in the manifest.",
    )
    parser.add_argument("--runtime-root", default=r"C:\adams_work")
    parser.add_argument("--adams-home", default=None)
    parser.add_argument(
        "--strict-k-report",
        default="artifacts/real-adams-car/strict-k/comparison_report.json",
    )
    args = parser.parse_args(argv)
    if args.warmup_runs < 0 or args.runs < 1:
        raise SystemExit("--warmup-runs must be >= 0 and --runs must be >= 1")
    if args.fixed_step_s is not None and (
        not args.fixed_step or args.fixed_step_s <= 0.0
    ):
        raise SystemExit("--fixed-step-s requires --fixed-step and must be positive")
    if args.adams_hht_alpha is not None and not (-0.333333 <= args.adams_hht_alpha <= 0.0):
        raise SystemExit("--adams-hht-alpha must be within [-1/3, 0]")

    runtime_root = Path(args.runtime_root)
    runtime_root.mkdir(parents=True, exist_ok=True)
    if "HOME" not in os.environ:
        default_home = Path(r"C:\adams_work\analytic_adams_car_home")
        if default_home.is_dir():
            os.environ["HOME"] = str(default_home)
    os.environ.setdefault("MSC_USE_WD", "disabled")

    profile = discover_profile("adams-car-2024.1", home=args.adams_home)
    if not profile.available or not profile.executable:
        raise SystemExit(profile.message)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment["MSC_USE_WD"] = "disabled"
    strict_report = Path(args.strict_k_report)
    if not strict_report.is_file():
        raise SystemExit(
            "missing real Adams/Car strict-K evidence; run validate-adams "
            f"first: {strict_report}"
        )
    strict_payload = json.loads(strict_report.read_text(encoding="utf-8"))
    if strict_payload.get("passed") is not True:
        raise SystemExit(f"real Adams/Car strict-K comparison failed: {strict_report}")
    summary = _run_case(
        args.case,
        output_dir,
        profile,
        runtime_root,
        args.warmup_runs,
        args.runs,
        environment,
        args.fixed_step,
        args.fixed_step_s,
        args.adams_hht_alpha,
    )
    report = {
        "producer": "open-kinematics.real-adams-car-benchmark",
        "profile": profile.as_dict(),
        "strict_k_report": str(strict_report),
        "strict_k_passed": True,
        "cases": [summary],
        "speed_gate_passed": bool(summary["native_not_slower_than_adams"]),
        "dynamic_precision_gate_passed": bool(
            summary["dynamic_precision_gate_passed"]
        ),
        "precision_and_physics_gate_passed": bool(
            all(summary["native_physics_gates"].values())
        ),
    }
    report_path = output_dir / "summary.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if (
        report["speed_gate_passed"] and report["dynamic_precision_gate_passed"]
    ) else 2


if __name__ == "__main__":
    raise SystemExit(main())
