"""Export the report layer's wheel-load channel inventory, before and after.

Run from the repository root::

    uv run --no-sync python <this file> --emit raw/four_wheel_before.json
    uv run --no-sync python <this file> --emit raw/four_wheel_after.json
    uv run --no-sync python <this file> --compare raw/four_wheel_before.json \
        raw/four_wheel_after.json

Every published number is written as its ``repr``, so the comparison is
bit-level rather than approximate.  ``--compare`` prints one line per channel and
per summary attribute that existed before the change, and exits non-zero if any
of them moved, vanished or changed position.

The four-wheel and three-axle loads come from the assemblies
``tests/physics/test_static_loads.py`` (p3-04) builds, so the report layer is
exercised on the frozen static-equilibrium contract rather than on a hand-written
mapping.  The rename experiment uses a placement (``bogie``) no axle in this
repository declares: if the channel names follow the wheel ends, ``bogie`` is
reported and no ``front``/``middle``/``rear`` spelling appears for it.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


def _repository_root() -> Path:
    """Return the repository root, checked by the tests it is about to import."""
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "packages" / "suspension_multibody" / "tests").is_dir():
            return candidate
    raise SystemExit("no repository root above this file")


ROOT = _repository_root()
sys.path.insert(0, str(ROOT / "packages" / "suspension_multibody"))

from suspension_multibody.outputs import builtin  # noqa: E402
from suspension_multibody.report.metrics import wheel_load_metrics  # noqa: E402
from suspension_multibody.report.wheel_loads import summarize_wheel_loads  # noqa: E402
from suspension_multibody.vehicle.static_loads import (  # noqa: E402
    compute_static_wheel_loads,
    compute_static_wheel_loads_for_assembly,
)
from tests.conftest import full_vehicle_model  # noqa: E402
from tests.physics.test_static_loads import (  # noqa: E402
    _corner_assembly,
    _three_axle_assembly,
)

#: The four-corner loads the metrics test uses, in the order the assembly states.
LEGACY_LOADS: dict[str, float] = {
    "front_left": 100.0,
    "front_right": 120.0,
    "rear_left": 80.0,
    "rear_right": 90.0,
}

#: The channel names the four-wheel report published before the change.
LEGACY_CHANNELS: tuple[str, ...] = (
    "normal_load_front_left",
    "normal_load_front_right",
    "normal_load_rear_left",
    "normal_load_rear_right",
    "normal_load_total",
    "normal_load_front_axle",
    "normal_load_rear_axle",
    "normal_load_left_side",
    "normal_load_right_side",
    "load_transfer_front_minus_rear",
    "load_transfer_right_minus_left",
)

#: The attribute names the four-wheel summary published before the change.  They
#: are read with ``getattr`` rather than from ``dataclasses.fields`` so the
#: comparison still sees one that a later change turns into a property.
LEGACY_SUMMARY_FIELDS: tuple[str, ...] = (
    "wheel_loads",
    "total",
    "front_axle",
    "rear_axle",
    "left_side",
    "right_side",
    "front_rear_delta",
    "right_left_delta",
)


def _channels(loads: dict[str, float]) -> dict[str, Any]:
    """Return the metric channel table as ``repr``s, or the refusal that replaced it."""
    try:
        values = wheel_load_metrics(loads)
    except Exception as error:  # noqa: BLE001 - a refusal is evidence too
        return {"ok": False, "error": f"{type(error).__name__}: {error}"}
    return {
        "ok": True,
        "channels": {name: repr(value) for name, value in values.items()},
    }


def _summary(loads: dict[str, float]) -> dict[str, Any]:
    """Return the summary's published attributes as ``repr``s, or its refusal."""
    try:
        summary = summarize_wheel_loads(loads)
    except Exception as error:  # noqa: BLE001 - a refusal is evidence too
        return {"ok": False, "error": f"{type(error).__name__}: {error}"}
    fields: dict[str, str] = {}
    for name in LEGACY_SUMMARY_FIELDS:
        try:
            fields[name] = repr(getattr(summary, name))
        except Exception as error:  # noqa: BLE001 - a missing field is a movement
            fields[name] = f"<{type(error).__name__}: {error}>"
    extra = {
        name: repr(getattr(summary, name))
        for name in ("axle_loads",)
        if hasattr(summary, name)
    }
    return {"ok": True, "fields": fields, "extra": extra}


def _case(loads: dict[str, float]) -> dict[str, Any]:
    return {
        "loads": {name: repr(value) for name, value in loads.items()},
        "metrics": _channels(loads),
        "summary": _summary(loads),
    }


def _assembly_case(result: Any) -> dict[str, Any]:
    loads = dict(result.wheel_loads)
    return {
        "wheel_loads": {name: repr(value) for name, value in loads.items()},
        **_case(loads),
    }


def inventory() -> dict[str, Any]:
    """Return the whole inventory: channels, summaries and the declared outputs."""
    # `conftest.full_vehicle_model` is a fixture; the wrapped function is the plain
    # builder a script can call.
    four_wheel = compute_static_wheel_loads(full_vehicle_model.__wrapped__())
    three_axle = compute_static_wheel_loads_for_assembly(_three_axle_assembly())
    corner = compute_static_wheel_loads_for_assembly(_corner_assembly())
    return {
        "fake_four_wheel": _case(dict(LEGACY_LOADS)),
        "assembly_four_wheel": _assembly_case(four_wheel),
        "assembly_three_axle": _assembly_case(three_axle),
        "assembly_single_wheel": _assembly_case(corner),
        "rename_experiment": _case(
            {
                "bogie_left": 250.0,
                "bogie_right": 150.0,
                "rear_left": 100.0,
                "rear_right": 100.0,
            }
        ),
        "derived_load_output_names": sorted(
            output.name
            for output in builtin.DERIVED_OUTPUTS
            if output.legacy == "report.metrics.vehicle.wheel_load_metrics"
        ),
        "derived_load_output_reads": {
            output.name: list(output.reads)
            for output in builtin.DERIVED_OUTPUTS
            if output.name.startswith(("normal_load", "load_transfer"))
        },
    }


def _legacy_view(case: dict[str, Any]) -> dict[str, Any]:
    """Return only the channels and attributes that existed before the change."""
    metrics = case["metrics"]
    summary = case["summary"]
    view: dict[str, Any] = {}
    if metrics.get("ok"):
        view["metrics"] = {
            name: metrics["channels"][name]
            for name in LEGACY_CHANNELS
            if name in metrics["channels"]
        }
        view["metrics_order"] = [
            name for name in metrics["channels"] if name in LEGACY_CHANNELS
        ]
        view["metrics_missing"] = [
            name for name in LEGACY_CHANNELS if name not in metrics["channels"]
        ]
    else:
        view["metrics"] = metrics["error"]
    if summary.get("ok"):
        view["summary"] = dict(summary["fields"])
    else:
        view["summary"] = summary["error"]
    return view


def compare(before: dict[str, Any], after: dict[str, Any]) -> int:
    """Print the per-channel comparison and return the number of moved channels."""
    moved = 0
    for case in ("fake_four_wheel", "assembly_four_wheel"):
        was = _legacy_view(before[case])
        now = _legacy_view(after[case])
        print(f"== {case}")
        for key in ("metrics", "summary"):
            old, new = was[key], now[key]
            if not isinstance(old, dict) or not isinstance(new, dict):
                same = old == new
                moved += 0 if same else 1
                print(f"  {'OK  ' if same else 'MOVE'} {key}: {old!s} -> {new!s}")
                continue
            for name in sorted(set(old) | set(new)):
                same = old.get(name) == new.get(name)
                moved += 0 if same else 1
                print(
                    f"  {'OK  ' if same else 'MOVE'} {key}.{name:<26}"
                    f" before={old.get(name)!s:<26} after={new.get(name)!s}"
                )
        order_ok = was.get("metrics_order") == now.get("metrics_order")
        moved += 0 if order_ok else 1
        print(
            f"  {'OK  ' if order_ok else 'MOVE'} legacy channel order "
            f"before={was.get('metrics_order')} after={now.get('metrics_order')}"
        )
        missing = now.get("metrics_missing")
        moved += 0 if not missing else len(missing)
        print(f"  {'OK  ' if not missing else 'MOVE'} legacy channels missing after: {missing}")
    print(f"\nmoved channels: {moved}")
    return moved


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--emit", type=Path)
    mode.add_argument("--compare", nargs=2, type=Path)
    args = parser.parse_args()

    if args.compare:
        before = json.loads(args.compare[0].read_text(encoding="utf-8"))
        after = json.loads(args.compare[1].read_text(encoding="utf-8"))
        moved = compare(before, after)
        print(
            "OK: every pre-existing channel kept its name, order and value"
            if moved == 0
            else "FAIL: a pre-existing channel moved"
        )
        return 0 if moved == 0 else 1

    payload = inventory()
    args.emit.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"wrote {args.emit}")
    print(json.dumps(payload["fake_four_wheel"]["metrics"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
