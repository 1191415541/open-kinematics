#!/usr/bin/env python
"""Freeze the assembly products of the seven shipped (assembly, bench) combinations.

Why this file exists
--------------------
Subtask 01 of the assembly-layer Epic has to turn "what the seven existing
combinations assemble today" into a file, so that every later subtask can say
"the products did not move" with evidence instead of with a claim.  The seven
names come from `suspension_multibody.rigs.rig.RIGS` and from nowhere else: a
second list here would be a second answer to "which benches ship", and the two
would drift.

What is frozen, and what is not
-------------------------------
The *products* of the assemblies those benches run, not the numbers a study
computes (the numerical freeze is `kc_baseline/` and `dynamic_hash_baseline`):

* the two axle benches (`kc_quasi_static`, `axle_dynamic`) each get their own
  products, because a wheel-supplying bench joins the assembly it loads: the
  carrier bodies and the link `subsystems/rig_link.py` builds are part of what
  the run solves, so a change there has to show up here.  Products are keyed
  `axle_K@<bench>` and `axle_C@<bench>` for exactly that reason -- one shared
  key would hide which bench moved.
* the five vehicle benches share one product (`vehicle`).  Their assembly is
  built from the model alone (`compose_vehicle_runtime` takes no bench); the
  bench binds itself later, in the study layer, which is outside this snapshot
  and is covered by subtask 05's own runtime comparison instead.  Each rig's
  entry says so in `bench_bound`.
* `rigs` records each bench's declaration (`family`/`study`/`supplies_wheels`/
  drives/outputs) and which products it runs on.

One fact about the products is worth stating because it looks like a missing
record and is not: a K assembly carries no force element.  Its force column is
the ideal-joint row (`constraints` / `ideal_constraints`), while the built
elements (`bushings` / `elements`) belong to the C reading.  That is the K/C
layering the Epic records as F9, so the snapshot records it as it is and judges
the force column as a whole.

Two runs, one payload
---------------------
The payload carries no generation time, no absolute path outside the repository
and no environment value, and it is serialised with sorted keys, so two runs on
the same source produce byte-identical output.  The run's own time is recorded
in `raw/snapshot_notes.md` instead, where a frozen input cannot see it.

Checking, and approved differences
----------------------------------
`--check` compares the freshly computed payload against the frozen one.  A
difference fails the check unless `raw/approved_deltas.json` holds an entry that
matches it *exactly*: same product, same pointer, same before value, same after
value.  There is no prefix rule -- a register entry covers the one difference it
was written for and nothing else, so a later change at the same place is not
silently covered by an older entry.

Every register entry must name `registered_by`, and only subtask 05 may register
anything: 05 is the one row allowed to move the existing products (the bench no
longer rewriting the assembly it loads).  A register that is incomplete or that
names another owner fails the check instead of being ignored -- a malformed
register must not look like an empty one.  The owner string is a procedural
guard, not a proof: subtask 07 audits the register, entry by entry, against the
per-step changes recorded in each subtask's `PROGRESS.md`.

Usage
-----
    uv run --no-sync python .../snapshot.py           # write the snapshot
    uv run --no-sync python .../snapshot.py --check   # compare, exit non-zero on a difference that is not registered
"""

from __future__ import annotations

import argparse
import dataclasses
import importlib.util
import json
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent
RAW = HERE / "raw"
SNAPSHOT = RAW / "assembly_snapshot.json"
APPROVED_DELTAS = RAW / "approved_deltas.json"

REPOSITORY_ROOT = HERE.parents[3]
TESTS = REPOSITORY_ROOT / "packages/suspension_multibody/tests"
VEHICLE_FIXTURE = TESTS / "vehicle/test_native_vehicle.py"
AXLE_FIXTURE = TESTS / "benchmark_fixture.py"
KC_FIXTURE = TESTS / "cases/kc_quasi_static/kc_fixtures.py"

#: The benches that supply their own wheels, in the order their products are
#: recorded.  A wheel-supplying bench joins the assembly it loads, so each of
#: these gets its own product keys; sorted, because a set's order is not.
AXLE_RIGS = ("axle_dynamic", "kc_quasi_static")

#: The two readings of the shared axle: K drives the wheel centres, C loads pads.
MODES = ("K", "C")

#: The vehicle-only tables.  They are recorded because they are what the later
#: subtasks move: which body carries each wheel centre, and where that centre sits.
VEHICLE_TABLES = (
    "wheel_specs",
    "wheel_centers",
    "wheel_body_names",
    "wheel_rotations_local",
    "body_aliases",
)

#: The entity classes every combination has to carry, and the classes that make
#: up its force column.  See the module docstring for why the K reading has no
#: entry in the second half of the force column.
REQUIRED_CLASSES = ("bodies", "points", "constraints")
FORCE_CLASSES = ("constraints", "ideal_constraints", "elements")

#: Fields a register entry must carry, and the only subtask allowed to register.
REGISTER_FIELDS = (
    "product",
    "pointer",
    "before",
    "after",
    "reason",
    "registered_by",
    "evidence",
)
REGISTER_OWNER = "05"


def _load_module(name: str, path: Path) -> Any:
    """Load a test fixture by explicit path.

    Imported by path rather than through the `tests` package: this script is
    evidence for a subtask, and evidence that needs a test package importable is
    evidence a reader cannot reproduce from the archive alone.
    """
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:  # pragma: no cover - defensive
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _normalise(value: Any) -> Any:
    """Return `value` as JSON-compatible data, without losing its type name.

    Determinism is the point: numpy scalars become Python scalars, arrays become
    lists, sets become sorted lists, and a dataclass or model becomes
    `{"__type__": name, ...fields}` so a product's field names survive into the
    snapshot.  Nothing here reads a clock or the environment.
    """
    if isinstance(value, (set, frozenset)):
        # A set's iteration order depends on the process's hash seed, so it is
        # recorded as a sorted list: a difference in *content* is what a later
        # subtask needs to see, and a difference in iteration order is noise.
        items = [_normalise(item) for item in value]
        items.sort(key=lambda item: json.dumps(item, sort_keys=True, ensure_ascii=False))
        return {"__type__": type(value).__name__, "__items__": items}
    if isinstance(value, np.ndarray):
        return [_normalise(item) for item in value.tolist()]
    if isinstance(value, np.generic):
        return _normalise(value.item())
    if value is None or isinstance(value, (bool, str, int, float)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _normalise(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_normalise(item) for item in value]
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return {"__type__": type(value).__name__, **_normalise(vars(value))}
    model_dump = getattr(value, "model_dump", None)
    if callable(model_dump):
        return {"__type__": type(value).__name__, **_normalise(model_dump(mode="json"))}
    fields = getattr(value, "__dict__", None)
    if isinstance(fields, dict) and fields:
        return {"__type__": type(value).__name__, **_normalise(vars(value))}
    return {"__type__": type(value).__name__, "__value__": str(value)}


def _items(runtime: Any, name: str) -> list[Any]:
    """Return one sequence field of a runtime, or nothing when it has none.

    A `SubsystemRuntime` carries a C column and a `VehicleRuntime` does not: the
    vehicle layer keeps the C column inside the axle runtimes it merged.  Asking
    for a field the product does not have is therefore not an error here, it is a
    statement about which product this is.
    """
    return [_normalise(item) for item in getattr(runtime, name, ())]


def _product(runtime: Any) -> dict[str, Any]:
    """Return one assembly runtime's products, as the contract document lists them."""
    points = [
        {"body": body, "label": label, "value": _normalise(np.asarray(point))}
        for (body, label), point in runtime.points.items()
    ]
    # `points` is a mapping, so its order is not a product property; sorting by
    # `(body, label)` is what makes the comparison a comparison of content.
    points.sort(key=lambda row: (str(row["body"]), str(row["label"])))
    product: dict[str, Any] = {
        "mode": _normalise(getattr(runtime, "mode", None)),
        "body_names": [str(name) for name in runtime.bodies],
        "bodies": {str(name): _normalise(body) for name, body in runtime.bodies.items()},
        "points": points,
        "constraints": _items(runtime, "constraints"),
        "ideal_constraints": _items(runtime, "ideal_constraints"),
        "bushings": _items(runtime, "bushings"),
        "elements": _items(runtime, "elements"),
        "connections": _items(runtime, "connections"),
        "capabilities": _normalise(getattr(runtime, "capabilities", None)),
        "counts": {
            "bodies": len(runtime.bodies),
            **{
                klass: len(getattr(runtime, klass, ()))
                for klass in (
                    "points",
                    "constraints",
                    "ideal_constraints",
                    "bushings",
                    "elements",
                    "connections",
                )
            },
        },
    }
    for name in VEHICLE_TABLES:
        if hasattr(runtime, name):
            product[name] = _normalise(getattr(runtime, name))
    if hasattr(runtime, "axle_assemblies"):
        product["axle_assemblies"] = sorted(runtime.axle_assemblies)
    return product


def _axle_products() -> dict[str, Any]:
    """Return both readings of the shared axle, once per wheel-supplying bench."""
    from suspension_multibody.preparation.kc_quasi_static import assembly_for

    benchmark = _load_module("freeze_benchmark_fixture", AXLE_FIXTURE)
    compliant = _load_module("freeze_kc_fixture", KC_FIXTURE)
    models = {"K": benchmark.benchmark_model(), "C": compliant._compliant_model()}
    products: dict[str, Any] = {}
    for rig in AXLE_RIGS:
        for mode in MODES:
            products[f"axle_{mode}@{rig}"] = _product(
                assembly_for(models[mode], mode=mode, rig=rig)
            )
    return products


def _vehicle_product() -> dict[str, Any]:
    """Return the full vehicle's assembly, in the mode its own model selects."""
    from suspension_multibody.preparation.vehicle_dynamic import _select_assembly_mode
    from suspension_multibody.subsystems.vehicle_assembly import compose_vehicle_runtime

    fixture = _load_module("freeze_vehicle_fixture", VEHICLE_FIXTURE)
    model = fixture._vehicle()
    mode = _select_assembly_mode(model, "auto")
    return {"vehicle": _product(compose_vehicle_runtime(model, mode=mode))}


def _rig_table(assemblies: Mapping[str, Any]) -> dict[str, Any]:
    """Return each bench's declaration and the products it runs on.

    The products are matched by the key they carry rather than by a second rule
    here: an axle bench runs the products that name it, and every other bench runs
    the vehicle product.  `bench_bound` says whether the bench is already part of
    the assembly the product describes -- for the wheel-supplying benches it is,
    and for the rest the binding happens in the study layer, outside this file.
    """
    from suspension_multibody.rigs.rig import RIGS

    table: dict[str, Any] = {}
    for name, spec in RIGS.items():
        keys = [key for key in assemblies if key.endswith(f"@{name}")]
        if not keys:
            if spec.supplies_wheels:
                # A bench that supplies wheels is part of the assembly it loads,
                # so it needs its own products.  Saying so out loud is what keeps
                # a newly added bench from being quietly recorded as the shared
                # vehicle product, which would look like coverage and not be it.
                raise RuntimeError(
                    f"bench {name!r} supplies wheels but has no product of its own; "
                    "add it to AXLE_RIGS in this script"
                )
            keys = ["vehicle"]
        bound = bool(keys and not keys[0].startswith("vehicle"))
        table[name] = {
            "family": spec.family,
            "route": spec.route,
            "study": spec.study,
            "supplies_wheels": spec.supplies_wheels,
            "drives": [drive.coordinate for drive in spec.drives],
            "outputs": list(spec.outputs),
            "assemblies": keys,
            "bench_bound": bound,
            "bench_binding": (
                "the bench joins the assembly before the products above exist "
                "(subsystems/si_assembly.py merges the rig link)"
                if bound
                else "the assembly is built from the model alone; the bench binds "
                "itself in the study layer, outside this snapshot"
            ),
        }
    return table


def compute_payload() -> dict[str, Any]:
    """Return the frozen payload: the assembly products plus the bench table."""
    assemblies: dict[str, Any] = {}
    assemblies.update(_axle_products())
    assemblies.update(_vehicle_product())
    rigs = _rig_table(assemblies)
    return {
        "_meta": {
            "generated_by": "tasks/20260929-01-freeze/snapshot.py",
            "command": "uv run --no-sync python <this file> [--check]",
            "rig_source": "suspension_multibody.rigs.rig.RIGS",
            "rig_count": len(rigs),
            "axle_fixture": (
                "packages/suspension_multibody/tests/benchmark_fixture.py"
                "::benchmark_model"
            ),
            "compliant_fixture": (
                "packages/suspension_multibody/tests/cases/kc_quasi_static/"
                "kc_fixtures.py::_compliant_model"
            ),
            "vehicle_fixture": (
                "packages/suspension_multibody/tests/vehicle/"
                "test_native_vehicle.py::_vehicle"
            ),
            "axle_entry": (
                "preparation.kc_quasi_static.assembly_for(model, mode, rig=<bench>) "
                "- called once per wheel-supplying bench"
            ),
            "vehicle_entry": (
                "subsystems.vehicle_assembly.compose_vehicle_runtime"
                "(model, mode=_select_assembly_mode(model, 'auto'))"
            ),
            "product_keys": (
                "axle_<mode>@<bench> for the wheel-supplying benches, so a change "
                "that reaches only one bench cannot hide behind a shared key; "
                "vehicle for the rest"
            ),
            "coverage_boundary": (
                "a vehicle bench binds itself in the study layer, which this "
                "snapshot does not cover: for the five vehicle benches 05 must use "
                "its own runtime comparison, not this file alone"
            ),
            "force_column": (
                "a K reading carries no element: its force column is the ideal-joint "
                "row (constraints/ideal_constraints); bushings/elements belong to the "
                "C reading - the F9 layering"
            ),
            "serialisation": "json.dumps(indent=2, sort_keys=True, ensure_ascii=False)",
            "determinism": (
                "no timestamp, no environment value, no absolute path is stored; the "
                "run's own time is recorded in raw/snapshot_notes.md"
            ),
            "check_scope": (
                "differences are judged on the assemblies and rigs nodes; _meta "
                "differences are reported but do not fail the check"
            ),
            "approved_deltas": (
                "raw/approved_deltas.json - an entry covers exactly one difference "
                "(product + pointer + before + after), must name registered_by, and "
                "only subtask 05 may register"
            ),
        },
        "assemblies": assemblies,
        "rigs": rigs,
    }


def _diff(pointer: str, expected: Any, actual: Any, out: list[dict[str, Any]]) -> None:
    """Append one row per difference between `expected` and `actual`."""
    if isinstance(expected, Mapping) and isinstance(actual, Mapping):
        for key in sorted(set(expected) | set(actual)):
            child = f"{pointer}/{key}"
            if key not in expected:
                out.append({"pointer": child, "before": None, "after": actual[key]})
            elif key not in actual:
                out.append({"pointer": child, "before": expected[key], "after": None})
            else:
                _diff(child, expected[key], actual[key], out)
        return
    if isinstance(expected, list) and isinstance(actual, list):
        if len(expected) != len(actual):
            out.append(
                {
                    "pointer": pointer,
                    "before": f"length {len(expected)}",
                    "after": f"length {len(actual)}",
                }
            )
            return
        for index, (before, after) in enumerate(zip(expected, actual, strict=True)):
            _diff(f"{pointer}/{index}", before, after, out)
        return
    if expected != actual:
        out.append({"pointer": pointer, "before": expected, "after": actual})


def _node_differences(
    frozen: Mapping[str, Any],
    current: Mapping[str, Any],
    node: str,
    label: str = "",
) -> list[dict[str, Any]]:
    """Return the differences inside one node, labelled by its owner entry.

    `label` namespaces the owner (`rig:<name>` for a bench) so a product and a
    bench that happen to share a name cannot be confused in the register.
    """
    rows: list[dict[str, Any]] = []
    before = frozen.get(node, {})
    after = current.get(node, {})
    for key in sorted(set(before) | set(after)):
        owner = f"{label}{key}"
        if key not in before:
            rows.append(
                {"product": owner, "pointer": "", "before": None, "after": after[key]}
            )
            continue
        if key not in after:
            rows.append(
                {"product": owner, "pointer": "", "before": before[key], "after": None}
            )
            continue
        found: list[dict[str, Any]] = []
        _diff("", before[key], after[key], found)
        rows.extend({"product": owner, **row} for row in found)
    return rows


def _load_approved() -> list[Mapping[str, Any]]:
    """Return the register, raising when it is not a list of entries."""
    if not APPROVED_DELTAS.exists():
        return []
    payload = json.loads(APPROVED_DELTAS.read_text(encoding="utf-8"))
    if not isinstance(payload, list):
        raise RuntimeError(f"{APPROVED_DELTAS} must hold a JSON array")
    return payload


def _register_problems(entries: list[Mapping[str, Any]]) -> list[str]:
    """Return why the register cannot be trusted, entry by entry.

    A register that is incomplete or that names another owner has to fail the
    check: ignoring it would make a malformed register look like an empty one,
    which is the failure mode a register exists to prevent.
    """
    problems: list[str] = []
    for index, entry in enumerate(entries):
        if not isinstance(entry, Mapping):
            # `null`, a string or a number in the register is a mistake, and it
            # has to name itself rather than raise somewhere further down.
            problems.append(f"entry {index}: is not an object ({type(entry).__name__})")
            continue
        missing = [field for field in REGISTER_FIELDS if field not in entry]
        if missing:
            problems.append(f"entry {index}: missing {', '.join(missing)}")
        owner = str(entry.get("registered_by", ""))
        if not owner.startswith(REGISTER_OWNER):
            problems.append(
                f"entry {index}: registered_by {owner!r} is not subtask "
                f"{REGISTER_OWNER} (only {REGISTER_OWNER} may move a product)"
            )
    return problems


def _approved(entry: Mapping[str, Any], row: Mapping[str, Any]) -> bool:
    """Return whether one register entry covers exactly this difference.

    Exact match on all four parts, and no prefix rule: an entry written for one
    difference must not silently cover a different change at the same place.
    """
    return (
        str(entry.get("product")) == str(row.get("product"))
        and str(entry.get("pointer", "")) == str(row.get("pointer", ""))
        and entry.get("before") == row.get("before")
        and entry.get("after") == row.get("after")
    )


def _assert_products_are_not_empty(payload: Mapping[str, Any]) -> list[str]:
    """Return the benches whose products are missing an entity class.

    A snapshot that freezes nothing would still compare equal to itself forever,
    so every combination has to carry bodies, points and connection rows.  The
    force column is judged as a whole (see `FORCE_CLASSES`): the K reading has no
    element by design, and demanding one would turn the layering into a failure.
    """
    complaints: list[str] = []
    assemblies = payload.get("assemblies", {})
    for rig, spec in payload.get("rigs", {}).items():
        for key in spec.get("assemblies", []):
            product = assemblies.get(key)
            if product is None:
                complaints.append(f"{rig}: no assembly product {key}")
                continue
            counts = product.get("counts", {})
            for klass in REQUIRED_CLASSES:
                if not counts.get(klass):
                    complaints.append(f"{rig}/{key}: {klass} is empty")
            if not sum(counts.get(klass, 0) for klass in FORCE_CLASSES):
                complaints.append(f"{rig}/{key}: no force row at all")
    return complaints


def _write(payload: Mapping[str, Any]) -> Path:
    """Write the payload, creating `raw/` when it is not there yet."""
    RAW.mkdir(parents=True, exist_ok=True)
    SNAPSHOT.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return SNAPSHOT


def _check() -> int:
    """Compare the freshly computed payload against the frozen one."""
    if not SNAPSHOT.exists():
        print(f"FAIL: {SNAPSHOT} does not exist; run without --check first")
        return 2
    frozen = json.loads(SNAPSHOT.read_text(encoding="utf-8"))
    current = compute_payload()

    rows = _node_differences(frozen, current, "assemblies")
    rows += _node_differences(frozen, current, "rigs", label="rig:")

    frozen_meta = frozen.get("_meta", {})
    current_meta = current.get("_meta", {})
    if frozen_meta != current_meta:
        print("NOTE: _meta differs (a change of method, not of product):")
        for key in sorted(set(frozen_meta) | set(current_meta)):
            if frozen_meta.get(key) != current_meta.get(key):
                print(
                    f"  _meta/{key}: {frozen_meta.get(key)!r} -> "
                    f"{current_meta.get(key)!r}"
                )

    approved = _load_approved()
    problems = _register_problems(approved)
    if problems:
        for problem in problems:
            print(f"FAIL: register {problem}")
        return 4

    if not rows:
        print("OK: the seven combinations assemble exactly what the snapshot froze")
        return 0

    unapproved = [
        row for row in rows if not any(_approved(entry, row) for entry in approved)
    ]
    print(f"differences: {len(rows)} (registered: {len(rows) - len(unapproved)})")
    for row in rows:
        registered = any(_approved(entry, row) for entry in approved)
        mark = "registered" if registered else "UNREGISTERED"
        print(
            f"  [{mark}] {row['product']}{row['pointer']}: "
            f"{json.dumps(row['before'], ensure_ascii=False)} -> "
            f"{json.dumps(row['after'], ensure_ascii=False)}"
        )
    if unapproved:
        print(
            "FAIL: unregistered product differences; register them in "
            f"{APPROVED_DELTAS.name} from the subtask that owns the change "
            f"(only {REGISTER_OWNER} may)"
        )
        return 1
    print("OK: every difference is registered in raw/approved_deltas.json")
    return 0


def main(argv: list[str] | None = None) -> int:
    """Write the snapshot, or check the products against the frozen one."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="compare against the frozen snapshot instead of writing it",
    )
    args = parser.parse_args(argv)

    if args.check:
        return _check()

    payload = compute_payload()
    complaints = _assert_products_are_not_empty(payload)
    if complaints:
        for complaint in complaints:
            print(f"FAIL: {complaint}")
        return 3
    path = _write(payload)
    print(f"wrote {path}")
    for rig, spec in payload["rigs"].items():
        parts = ", ".join(
            f"{key}:{payload['assemblies'][key]['counts']['bodies']}b"
            f"/{payload['assemblies'][key]['counts']['elements']}e"
            for key in spec["assemblies"]
        )
        print(f"  {rig} ({spec['route']}) bound={spec['bench_bound']}: {parts}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
