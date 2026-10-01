"""p2-01 (a): freeze the torque path, case document -> kernel contract table.

Read-only.  This is the reference p2-05 measures against.

Run: uv run --no-sync python <this file> <output.json>
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "packages" / "suspension_multibody" / "tests"))

from suspension_multibody.preparation import vehicle_dynamic  # noqa: E402
from suspension_multibody.preparation.vehicle_dynamic import (  # noqa: E402
    _WHEEL_NAMES,
    _build_wheel_torque_signals,
    prepare_vehicle_run,
)
from suspension_multibody.schema import (  # noqa: E402
    DrivelineSpec,
    DynamicSolverSettings,
    TimeSignal,
    Vec3,
    VehicleDynamicCase,
)

_CONFTEST = ROOT / "packages" / "suspension_multibody" / "tests" / "conftest.py"
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("torque_path_snapshot.json")


def _full_vehicle():
    """
    Build the repository's canonical full vehicle, from the shared fixture.

    The fixture is a pytest fixture, so it is unwrapped here rather than called:
    the point is to freeze the *same* vehicle the suite uses, not a vehicle of
    this probe's own invention.

    The shared fixture leaves the rear rack free, which the preparation refuses
    (that refusal is p2-01 (b)'s evidence, in `raw/rear_steer_refusal.md`).
    Bolting it is what the suite's own vehicle fixture does
    (`tests/vehicle/test_native_vehicle.py:103`), and a torque-path snapshot has
    to be taken on a vehicle that actually prepares.
    """
    spec = importlib.util.spec_from_file_location("mb_conftest", _CONFTEST)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    model = module.full_vehicle_model.__wrapped__()
    # The shared fixture declares neither a driveline nor a braked wheel, so a
    # torque snapshot taken on it would record four zeros and prove nothing.  A
    # driveline and a braked wheel are added here so both torque columns carry
    # real values -- this is the *vehicle* the reference is taken on, stated
    # rather than left to a fixture default.
    wheels = tuple(
        wheel.model_copy(update={"braked": True}) for wheel in model.wheels
    )
    return model.model_copy(
        update={
            "rear_axle": model.rear_axle.model_copy(
                update={"rack_fixed_to_chassis": True}
            ),
            "wheels": wheels,
            "driveline": DrivelineSpec(
                driven_wheels=("rear_left", "rear_right"),
                maximum_drive_torque=400.0,
                maximum_brake_torque=2000.0,
                drive_split=(0.0, 0.0, 0.5, 0.5),
                front_brake_bias=0.6,
            ),
        }
    )


def main() -> None:
    model = _full_vehicle()
    # One real case: a drive demand and a brake demand, both constant, so the time
    # history is a flat line and any change in it is visible immediately.
    case = VehicleDynamicCase(
        name="p2-01-torque-path",
        solver=DynamicSolverSettings(
            start_time=0.0,
            end_time=0.02,
            step_size=0.01,
            internal_step_size=0.01,
            min_internal_step_size=0.01,
            adaptive_substepping=False,
            # The native vehicle path requires this integrator by name, and the
            # suite's own vehicle cases (`tests/vehicle/test_native_vehicle.py:224-231`)
            # use the same block.
            integrator="generalized_alpha",
            gravity=Vec3(x=0, y=0, z=0),
        ),
        vehicle=model,
        drive_input=TimeSignal(constant=0.5),
        brake_input=TimeSignal(constant=0.25),
        steering_input=TimeSignal(constant=0.0),
    )
    scale = vehicle_dynamic._length_scale(model.units)
    times = np.arange(
        case.solver.start_time,
        case.solver.end_time + case.solver.step_size * 0.5,
        case.solver.step_size,
    )

    drive, brake = _build_wheel_torque_signals(model, case, times, scale)
    prepared = prepare_vehicle_run(model, case)
    # The tables the contract actually carries.  They live on the prepared run
    # (`prepared.wheel_torque` / `prepared.brake_torque`, keyed by tire name) --
    # `cases/vehicle_dynamic.py:579/582` walks exactly these to write the
    # `role="wheel_torque"` / `role="brake_torque"` tables.  The axle case's own
    # `wheel_torque_n_m` field is the *pre-sampled input* and is empty on this
    # path, so reading it would have recorded a shape of `0` and meant nothing.
    drive_table = dict(prepared.wheel_torque)
    brake_table = dict(prepared.brake_torque)

    record = {
        "anchor": "EPIC.md F1 (route map :1506 is stale, see raw/anchors_F1_F5.md)",
        "defined_at": "preparation/vehicle_dynamic.py:1536 def _build_wheel_torque_signals",
        "called_at": "preparation/vehicle_dynamic.py:279 in prepare_vehicle_run",
        "wheel_names_constant": {
            "at": "preparation/vehicle_dynamic.py:73 _WHEEL_NAMES",
            "value": list(_WHEEL_NAMES),
        },
        "input": {
            "time_grid_start_s": float(times[0]),
            "time_grid_step_s": float(times[1] - times[0]),
            "time_grid_samples": int(times.size),
            "length_scale": float(scale),
            "drive_input": "TimeSignal(constant=0.5) normalized to [-1, 1]",
            "brake_input": "TimeSignal(constant=0.25) normalized to [0, 1]",
            "driveline": {
                "driven_wheels": list(model.driveline.driven_wheels),
                "drive_split": list(model.driveline.drive_split),
                "maximum_drive_torque": model.driveline.maximum_drive_torque,
                "maximum_brake_torque": model.driveline.maximum_brake_torque,
                "front_brake_bias": model.driveline.front_brake_bias,
            },
            "braked_wheels": sorted(w.name for w in model.wheels if w.braked),
        },
        "output": {
            "wheel_torque_n_m": {
                name: {
                    "samples": len(drive[name]),
                    "first": float(drive[name][0]),
                    "last": float(drive[name][-1]),
                }
                for name in sorted(drive)
            },
            "brake_torque_n_m": {
                name: {
                    "samples": len(brake[name]),
                    "first": float(brake[name][0]),
                    "last": float(brake[name][-1]),
                }
                for name in sorted(brake)
            },
        },
        "contract_table": {
            "written_at": (
                "cases/vehicle_dynamic.py:579/582 role=wheel_torque / role=brake_torque"
            ),
            "source_object": "PreparedVehicleRun.wheel_torque / .brake_torque",
            "wheel_torque_rows": len(drive_table),
            "brake_torque_rows": len(brake_table),
            "wheel_torque_per_tire_samples": {
                name: len(values) for name, values in sorted(drive_table.items())
            },
            "brake_torque_per_tire_samples": {
                name: len(values) for name, values in sorted(brake_table.items())
            },
            "wheel_torque_first_sample": {
                name: float(values[0]) for name, values in sorted(drive_table.items())
            },
            "brake_torque_first_sample": {
                name: float(values[0]) for name, values in sorted(brake_table.items())
            },
        },
        "note": (
            "p2-05's reference: with the same driver input the torque time history "
            "must stay identical, or the difference must be registered with an "
            "argument independent of the result bytes"
        ),
    }
    OUT.write_text(json.dumps(record, indent=2, sort_keys=True), encoding="utf-8")
    print(f"wrote {OUT}")
    print(json.dumps(record["output"], indent=2, sort_keys=True))


main()
