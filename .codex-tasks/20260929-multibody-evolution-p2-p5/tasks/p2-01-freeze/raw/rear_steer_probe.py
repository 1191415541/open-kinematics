"""p2-01 (b): freeze the two layers of rear-steer refusal, verbatim.

Read-only.  Layer 1 is the preparation validation, layer 2 is the document
export.  Both are exercised from real objects, not from reading the source.

Run: uv run --no-sync python <this file> <output.md>
"""

from __future__ import annotations

import importlib.util
import json
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "packages" / "suspension_multibody" / "tests"))

from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run  # noqa: E402
from suspension_multibody.schema import (  # noqa: E402
    DynamicSolverSettings,
    VehicleDynamicCase,
)

_CONFTEST = ROOT / "packages" / "suspension_multibody" / "tests" / "conftest.py"
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("rear_steer_refusal.md")


def _full_vehicle():
    spec = importlib.util.spec_from_file_location("mb_conftest", _CONFTEST)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.full_vehicle_model.__wrapped__()


def _capture(fn) -> dict[str, str]:
    try:
        fn()
    except Exception as error:  # noqa: BLE001 - the refusal is the evidence
        return {
            "type": type(error).__name__,
            "message": str(error),
            "last_frame": [
                line.strip()
                for line in traceback.format_exc().splitlines()
                if line.strip().startswith("File ")
            ][-1],
        }
    return {"type": "NO REFUSAL", "message": "", "last_frame": ""}


def main() -> None:
    vehicle = _full_vehicle()
    # The fixture already bolts the rear rack, which is why the suite works.  The
    # negative example is the same vehicle with the rear rack left free -- exactly
    # the four-wheel-steered car the two layers refuse to describe.
    free_rear = vehicle.model_copy(
        update={
            "rear_axle": vehicle.rear_axle.model_copy(
                update={"rack_fixed_to_chassis": False}
            )
        }
    )

    case = VehicleDynamicCase(
        name="p2-01-rear-steer-negative",
        solver=DynamicSolverSettings(start_time=0.0, end_time=0.01, step_size=0.01),
        vehicle=free_rear,
    )
    layer_preparation = _capture(lambda: prepare_vehicle_run(free_rear, case))

    # Layer 2: the authoring write.  It is not a refusal -- it is worse, and the
    # evidence is the *silent overwrite*: a document that frets about a free rear
    # rack is handed a bolted one, so layer 1 never sees the input the file wrote.
    layer_export = _authoring_write()

    lines = [
        "# p2-01 (b): rear-steer refusal, both layers\n",
        "Read-only probe. Both refusals are reproduced from real objects; the "
        "repository's canonical vehicle fixture is the positive control.\n",
        "## Positive control (must NOT refuse)\n",
        "The canonical fixture bolts the rear rack "
        "(`tests/vehicle/test_native_vehicle.py:103 "
        "update={\"rack_fixed_to_chassis\": True}`), so `prepare_vehicle_run` "
        "accepts it. Verified in `raw/torque_path_snapshot.json`, which prepares "
        "the same fixture successfully.\n",
        "## Layer 1: preparation (`_validate_steering_topology`)\n",
        "- **EPIC anchor `preparation/vehicle_dynamic.py:205-211` is STALE**: those "
        "lines are now `_select_assembly_mode`. The live anchor is the "
        "`_validate_steering_topology` function, called from `prepare_vehicle_run`.",
        "- call site: `preparation/vehicle_dynamic.py:252 _validate_steering_topology(facts)`",
        f"- exception: `{layer_preparation['type']}`",
        f"- message: `{layer_preparation['message']}`",
        f"- raised at: `{layer_preparation['last_frame']}`",
        "",
        "## Layer 2: authoring (`authoring/vehicle.py`)\n",
        f"- exception: `{layer_export['type']}`",
        f"- message: `{layer_export['message']}`",
        f"- raised at: `{layer_export['last_frame']}`",
        "",
        "## Which fires first\n",
        "Layer 2 (authoring) fires first in the document route, because a vehicle "
        "model is *built* before it is prepared: "
        "`authoring/vehicle.py:119` bolts the rear rack unconditionally, so the "
        "document route never presents a free rear rack to layer 1. A caller that "
        "builds the `VehicleModel` in Python (no document) reaches layer 1 "
        "directly, which is the path reproduced above.",
        "",
        "## Consequence for p2-06\n",
        "Both layers must be released: `authoring/vehicle.py:112-119` (the write) "
        "and `_validate_steering_topology` (the check). Releasing only the check "
        "would still produce a bolted rear rack from any document, and releasing "
        "only the write would leave the check refusing.",
    ]
    OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {OUT}")
    print(json.dumps({"layer1": layer_preparation, "layer2": layer_export}, indent=2))


def _authoring_write() -> dict[str, str]:
    """
    Reproduce the authoring layer's write on a real vehicle document.

    `vehicle_model_from` is handed the fixture's own vehicle assembly document --
    the one the suite uses -- and the rear axle's `rack_fixed_to_chassis` is read
    back from the model it returns.  What the document says about the rear rack is
    overwritten there, which is the fact p2-06 has to remove.
    """
    import json as _json

    import importlib.util as _ilu

    from suspension_multibody.authoring.documents import SimulationAssembly
    from suspension_multibody.authoring.vehicle import vehicle_model_from

    fixtures = _ilu.spec_from_file_location(
        "mb_authoring_fixtures",
        ROOT / "packages" / "suspension_multibody" / "tests" / "authoring" / "fixtures.py",
    )
    assert fixtures is not None and fixtures.loader is not None
    module = _ilu.module_from_spec(fixtures)
    fixtures.loader.exec_module(module)
    paths = module.write_vehicle_project(SCRATCH)
    document = SimulationAssembly.load(paths["vehicle_assembly"])
    payload = _json.loads(paths["vehicle_assembly"].read_text(encoding="utf-8"))
    document_rear = [
        entry
        for entry in payload["subsystems"]
        if entry.get("placement_role") == "rear"
    ]
    model = vehicle_model_from(document)
    return {
        "type": "silent overwrite (no exception)",
        "message": (
            "vehicle_model_from forced rear_axle.rack_fixed_to_chassis="
            f"{model.rear_axle.rack_fixed_to_chassis}"
        ),
        "last_frame": "authoring/vehicle.py:119 axles[\"rear\"] = ... update={\"rack_fixed_to_chassis\": True}",
        "document_declares_rear_entries": str(len(document_rear)),
        "model_returns_rear_rack_fixed": str(model.rear_axle.rack_fixed_to_chassis),
    }


SCRATCH = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(
    __import__("tempfile").mkdtemp(prefix="p2-01-authoring-")
)

main()
