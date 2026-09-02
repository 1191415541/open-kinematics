"""
Generate and run the dynamic K&C correlation against real Adams Solver.

The suspension comes from the installed Adams Car database, so both solvers see
the same masses, inertias, hardpoints, bushing rates, spring rate and measured
damper curve.  The excitation is a dynamic wheel-travel sweep rather than a
quasi-static one, so the comparison exercises inertia and damping, not just
geometry.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import time
from pathlib import Path
from typing import Literal

import numpy as np

from suspension_multibody.adams import (
    AxleChannelBindings,
    AxleMarkerBinding,
    build_axle_adams_dataset,
    create_dynamic_axle_manifest,
    write_axle_adams_dataset,
)
from suspension_multibody.adams.car_import import read_adams_suspension
from suspension_multibody.adams.car_sla_model import (
    build_sla_axle_model,
    unsprung_corner_mass_kg,
)
from suspension_multibody.axle_dynamics import (
    AxleDynamicsCase,
    AxleHarmonicRoad,
    AxleSolverSettings,
    run_axle_dynamics,
)

ADAMS_ROOT = Path("G:/MSC.Software/Adams/2024_1")
SUBSYSTEM = (
    ADAMS_ROOT
    / "acar/achassis_gs.cdb/subsystems.tbl/acar_gs_front_suspension.sub"
)
# Adams records this wheel load for the front axle of this vehicle.
ADAMS_STATIC_LOAD_N = 5117.77
RIG_WHEEL_RADIUS_M = 0.300
TIRE_STIFFNESS_N_PER_M = 200_000.0

# The dynamic K&C excitation: a heave sweep at a frequency high enough that
# inertia and damper force matter, and an amplitude within suspension travel.
EXCITATION_HZ = 3.0
EXCITATION_AMPLITUDE_M = 0.010
DURATION_S = 0.5
OUTPUT_STEP_S = 0.001
CONVERGED_INTERNAL_STEP_S = 0.00003125


def build_case_and_model(
    rigid: bool,
    *,
    tire_model: Literal["native_brush"] = "native_brush",
):
    """Return the SI model, the sampled case, and the contact plane height."""
    if tire_model != "native_brush":
        raise ValueError(
            "standalone primitive Adams axle only implements native_brush; "
            "PAC2002 requires an independent Adams/Car reference"
        )
    suspension = read_adams_suspension(
        SUBSYSTEM,
        tire_unloaded_radius_m=RIG_WHEEL_RADIUS_M,
        tire_stiffness_n_per_m=TIRE_STIFFNESS_N_PER_M,
    )
    sprung_axle_mass_kg = 2.0 * (
        ADAMS_STATIC_LOAD_N / 9.80665 - unsprung_corner_mass_kg(suspension)
    )
    road_height_m = (
        suspension.hardpoints_m["wheel_center"][2] - RIG_WHEEL_RADIUS_M
    )
    model = build_sla_axle_model(
        suspension,
        sprung_mass_kg=sprung_axle_mass_kg,
        sprung_inertia_kg_m2=(180.0, 60.0, 200.0),
        sprung_height_m=0.85,
        road_height_m=road_height_m,
        rigid_hub_and_rig=rigid,
    )

    count = int(round(DURATION_S / OUTPUT_STEP_S)) + 1
    times = np.arange(count) * OUTPUT_STEP_S
    # The road rises and falls under both wheels together: in-phase heave.
    # Declared analytically so both solvers evaluate the identical closed form
    # instead of one interpolating the other's samples.
    case = AxleDynamicsCase(
        name="road_sine",
        times_s=tuple(float(value) for value in times),
        harmonic_roads=tuple(
            AxleHarmonicRoad(
                tire=tire.name,
                offset_m=road_height_m,
                amplitude_m=EXCITATION_AMPLITUDE_M,
                frequency_hz=EXCITATION_HZ,
            )
            for tire in model.tires
        ),
        solver=AxleSolverSettings(
            initialization_mode="static_equilibrium",
            adaptive_step=True,
            internal_step_s=CONVERGED_INTERNAL_STEP_S,
            maximum_step_s=OUTPUT_STEP_S,
            minimum_step_s=1e-7,
            max_newton_iterations=100,
            max_line_search_iterations=30,
            local_relative_tolerance=1e-4,
            local_position_tolerance_m=1e-6,
            local_velocity_tolerance_m_per_s=1e-5,
        ),
    )
    return model, case, road_height_m


def bindings_for(model) -> AxleChannelBindings:
    """Return channel bindings for the demo model."""
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
        left_damper="damper_l",
        right_damper="damper_r",
        left_tire="tire_l",
        right_tire="tire_r",
    )


def run_adams(work_dir: Path, stem: str) -> tuple[float, str]:
    """Run Adams Solver on the written dataset and return its wall time."""
    script = work_dir / "run_adams.ps1"
    # mdi.bat writes a temporary control script into the working directory and
    # invokes it by bare name, so "." must be on PATH or the solver never
    # starts.  Calling solver.exe directly fails the license check.
    script.write_text(
        "\r\n".join(
            [
                f'Set-Location "{work_dir}"',
                "$env:MSC_LICENSE_FILE = '27500@localhost'",
                "$env:PATH = '.;' + $env:PATH",
                "& cmd.exe /c "
                f"'{ADAMS_ROOT}\\common\\mdi.bat' ru-standard {stem}.acf",
            ]
        )
        + "\r\n",
        encoding="ascii",
    )
    started = time.perf_counter()
    completed = subprocess.run(
        [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(script),
        ],
        capture_output=True,
        text=True,
        timeout=3600,
    )
    elapsed = time.perf_counter() - started
    return elapsed, completed.stdout + completed.stderr


def main() -> int:
    """Run the dynamic K&C correlation workflow."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--work", default="C:/adams_work/kc")
    args = parser.parse_args()
    work = Path(args.work)
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)

    model, case, road_height_m = build_case_and_model(
        rigid=True,
        tire_model="native_brush",
    )
    manifest = create_dynamic_axle_manifest(
        model,
        case,
        bindings_for(model),
        adams_solver={
            "integrator": "HHT",
            "alpha": -0.3,
            "error": 1e-5,
            "maximum_step_s": OUTPUT_STEP_S,
        },
        execution_environment={
            "cpu_model": "host",
            "physical_core_count": 0,
            "thread_count": 1,
            "process_affinity": "default",
        },
        case_metadata={"harmonic_frequency_hz": EXCITATION_HZ},
    )

    dataset = build_axle_adams_dataset(manifest)
    paths = write_axle_adams_dataset(dataset, work)
    print(f"dataset written: {paths['model'].name}, {paths['command'].name}")

    native_started = time.perf_counter()
    run_axle_dynamics(model, case)
    native_elapsed = time.perf_counter() - native_started
    print(
        f"native: {native_elapsed:.3f} s for {DURATION_S} s "
        f"-> {native_elapsed / DURATION_S:.2f}x realtime"
    )

    adams_elapsed, log = run_adams(work, dataset.stem)
    message = work / f"{dataset.stem}.msg"
    print(f"adams wall: {adams_elapsed:.3f} s")
    print(f"adams files: {sorted(p.name for p in work.iterdir())}")
    if message.is_file():
        tail = message.read_text(errors="replace").splitlines()[-25:]
        print("--- adams message tail ---")
        print("\n".join(tail))
    else:
        print("--- adams stdout ---")
        print(log[-3000:])

    summary = {
        "native_wall_s": native_elapsed,
        "native_realtime_ratio": native_elapsed / DURATION_S,
        "adams_wall_s": adams_elapsed,
        "physical_duration_s": DURATION_S,
        "excitation_hz": EXCITATION_HZ,
        "excitation_amplitude_m": EXCITATION_AMPLITUDE_M,
        "road_height_m": road_height_m,
        "manifest_sha256": manifest.sha256,
    }
    (work / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
