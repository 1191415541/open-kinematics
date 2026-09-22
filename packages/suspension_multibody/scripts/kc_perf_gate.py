#!/usr/bin/env python
"""
Performance gate for the native K/C workloads.

``--record-native`` freezes the current implementation's wall clock together
with the machine identity; ``--check-native`` re-measures and fails when the new
measurement exceeds the recorded budget.

Two workloads, both through the contract boundary:

* ``k-100`` -- the 10x10 wheel-travel x rack grid the benchmark geometry
  defines, 100 states.  This is the same amount of work the retired Python
  benchmark measured, so the two numbers are comparable;
* ``c-66`` -- the six physical C load paths x 11 levels on the compliant
  fixture, 66 states.  The Python side's ``c-6600`` was a deliberately
  *nonphysical* proxy solver used to exercise table generation and cache
  behaviour; a proxy has no native analogue, so the native C workload is the
  real one and keeps a budget of its own rather than being scored against a
  number measured for a different amount of work.
"""

from __future__ import annotations

import argparse
import json
import platform
import statistics
import time
from pathlib import Path
from typing import Any, Callable

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BASELINE = (
    PACKAGE_ROOT / "tests" / "data" / "kc_perf_baseline_native.json"
)
#: The declarative benchmark-axle fixture, read by explicit path: this gate must
#: not import a test package.
BENCHMARK_FIXTURE = PACKAGE_ROOT / "tests/data/benchmark_axle.json"
#: Wall-clock allowance over the recorded baseline.  A gate that demands exact
#: parity with a two-sample median fails on machine load alone (a shared laptop
#: drifts by well over 10%), which teaches nothing and hides real regressions
#: behind noise.  The allowance is deliberately loose enough to absorb that
#: drift and tight enough that a regression of the kind this epic cares about (a
#: new solver path) cannot slip through.
BUDGET_FACTOR = 1.25
#: The gate compares the *best* of `--repeats` runs because wall-clock noise is
#: one-sided: load only ever makes a run slower.  The default is five rather than
#: three because a single background spike can span three, and a gate that fails
#: on a busy desktop teaches people to ignore it -- measured here, the same
#: workloads read x0.92 on a quiet machine and x1.5 when the desktop was busy.
DEFAULT_REPEATS = 5


def _benchmark_payload() -> dict[str, Any]:
    """Read the declarative benchmark-axle fixture by explicit path."""
    return json.loads(BENCHMARK_FIXTURE.read_text(encoding="utf-8"))


def _native_workloads() -> dict[str, Callable[[], int]]:
    """Return one callable per workload, each returning its state count."""
    import importlib.util

    from suspension_multibody.cases.kc_quasi_static import case_document, model_document
    from suspension_multibody.cases.kc_quasi_static.load_paths import LoadPath
    from suspension_multibody.cases.kc_quasi_static.workflow import (
        DEFAULT_SETTINGS,
        DEFAULT_TIMES,
    )
    from suspension_multibody.preparation.assembly import build_front_axle
    from suspension_multibody.schema import FrontAxleModel
    from suspension_multibody.simulation import SimulationRequest, run_request
    fixture = PACKAGE_ROOT / "tests/cases/kc_quasi_static/kc_fixtures.py"
    spec = importlib.util.spec_from_file_location("kc_perf_fixture", fixture)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    # The axle and the 10x10 grid come from the declarative fixture every K/C
    # gate reads, by explicit path: a gate must not import a test package.
    payload = _benchmark_payload()
    # A physical C sweep needs a compliance to answer with, so it uses the
    # compliant fixture every other native C gate uses.
    k_assembly = build_front_axle(FrontAxleModel.model_validate(payload["model"]), "K")
    compliant = getattr(module, "_compliant_model")()  # test fixture, not a public API
    c_assembly = build_front_axle(compliant, "C")
    wheel_values = tuple(float(value) for value in payload["grid"]["wheel_values_mm"])
    rack_values = tuple(float(value) for value in payload["grid"]["rack_values_mm"])
    axes = tuple(path.name for path in LoadPath.standard())

    def k_100() -> int:
        model = model_document(k_assembly, name="native-k", drive_wheels=True)
        case = case_document(
            k_assembly,
            family="kc_quasi_static",
            name="kc-k",
            wheel_values_mm=wheel_values,
            rack_values_mm=rack_values,
            times_s=DEFAULT_TIMES,
            settings=DEFAULT_SETTINGS,
            drive_wheels=True,
        )
        run = run_request(
            SimulationRequest(
                assembly="axle",
                family="kc_quasi_static",
                model=model,
                case=case,
            )
        ).raw
        return len(run.cases)

    def c_66() -> int:
        model = model_document(c_assembly, name="native-c", drive_wheels=False)
        case = case_document(
            c_assembly,
            family="kc_quasi_static",
            name="kc-c",
            paths=axes,
            levels=11,
            maximum=1.0,
            side_mode="single",
            times_s=DEFAULT_TIMES,
            settings=DEFAULT_SETTINGS,
            drive_wheels=False,
        )
        run = run_request(
            SimulationRequest(
                assembly="axle",
                family="kc_quasi_static",
                model=model,
                case=case,
            )
        ).raw
        return len(run.cases)

    return {"k-100": k_100, "c-66": c_66}


def measure(repeats: int) -> dict[str, Any]:
    """Time the native workloads and return the medians and state counts."""
    workloads = _native_workloads()
    samples: dict[str, list[float]] = {name: [] for name in workloads}
    states: dict[str, int] = {}
    for _ in range(repeats):
        for name, workload in workloads.items():
            started = time.perf_counter()
            states[name] = int(workload())
            samples[name].append(time.perf_counter() - started)
    return {
        "implementation": "native",
        "platform": platform.platform(),
        "processor": platform.processor(),
        "python": platform.python_version(),
        "repeats": repeats,
        "states": states,
        "median_seconds": {
            name: statistics.median(values) for name, values in samples.items()
        },
        "min_seconds": {name: min(values) for name, values in samples.items()},
        "samples_seconds": samples,
    }


def main() -> int:
    """Record or check the native benchmark budget."""
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=False)
    mode.add_argument("--record", "--record-native", action="store_true")
    mode.add_argument("--check", "--check-native", action="store_true")
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument("--repeats", type=int, default=DEFAULT_REPEATS)
    args = parser.parse_args()
    record = bool(args.record)
    check = bool(args.check) or not record

    measured = measure(args.repeats)
    for name, seconds in measured["median_seconds"].items():
        print(
            f"  {name}: median {seconds:.4f} s, best {measured['min_seconds'][name]:.4f} s"
            f" over {args.repeats} runs ({measured['states'][name]} states)"
        )

    if record:
        args.baseline.parent.mkdir(parents=True, exist_ok=True)
        args.baseline.write_text(
            json.dumps(measured, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"recorded performance baseline -> {args.baseline}")
        return 0

    if not check:
        return 0
    if not args.baseline.is_file():
        raise SystemExit(f"no baseline at {args.baseline}; run --record-native first")
    expected = json.loads(args.baseline.read_text(encoding="utf-8"))
    failures: list[str] = []
    for name, seconds in measured["min_seconds"].items():
        allowed = expected.get("min_seconds", expected["median_seconds"])[name]
        budget = allowed * BUDGET_FACTOR
        ratio = seconds / allowed if allowed else float("inf")
        print(f"  {name}: {seconds:.4f} s vs baseline {allowed:.4f} s (x{ratio:.3f})")
        if seconds > budget:
            failures.append(
                f"{name}: {seconds:.4f} s exceeds budget {budget:.4f} s "
                f"(x{ratio:.3f} of baseline, allowance x{BUDGET_FACTOR})"
            )
    if failures:
        print("\nFAIL")
        for failure in failures:
            print(f"  {failure}")
        return 1
    print("\nOK: benchmarks are within the recorded budget")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
