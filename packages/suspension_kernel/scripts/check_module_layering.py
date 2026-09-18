#!/usr/bin/env python
"""
Module-layering gate for the suspension kernel C++ sources.

The kernel is built as one static library per module.  This gate reads the
project's own header include graph and enforces four rules:

1. no module-level dependency cycle (the module graph must stay a DAG);
2. no module header pulls in another module's ``functions.hpp`` aggregate;
3. no header includes itself;
4. no dependency edge appears that is not in the recorded baseline.

The baseline exists because the current tree is *not* clean: it was recorded
from the tree as it stands so that later refactoring can only remove edges.
``--strict`` additionally requires zero rule-1/2/3 violations and is the gate
the modularisation phase must reach.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

KERNEL_ROOT = Path(__file__).resolve().parents[1]
INCLUDE_ROOT = KERNEL_ROOT / "cpp" / "include"
AXLE_ROOT = KERNEL_ROOT / "cpp" / "axle_dynamics"
BASELINE = KERNEL_ROOT / "layering_baseline.json"

TIRE_SUBMODULES = ("brush", "common", "fiala", "pac2002")


def _project_includes(path: Path) -> list[str]:
    found: list[str] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        stripped = line.strip()
        if not stripped.startswith("#include"):
            continue
        _, _, tail = stripped.partition("include")
        tail = tail.strip()
        if not tail.startswith('"'):
            continue
        end = tail.find('"', 1)
        if end == -1:
            continue
        found.append(tail[1:end])
    return found


def _resolve(include: str) -> Path | None:
    if include.startswith(("mb_", "abi/")):
        candidate = INCLUDE_ROOT / include
        return candidate if candidate.is_file() else None
    candidate = AXLE_ROOT / include
    return candidate if candidate.is_file() else None


def _module_of(path: Path) -> str:
    try:
        rel = path.relative_to(INCLUDE_ROOT)
    except ValueError:
        return "abi"  # cpp/axle_dynamics/*.hpp is the same public C ABI surface
    parts = rel.parts
    if parts[0] == "mb_tire" and len(parts) > 2 and parts[1] in TIRE_SUBMODULES:
        return f"mb_tire_{parts[1]}"
    return parts[0]


def _headers() -> list[Path]:
    files = sorted(INCLUDE_ROOT.rglob("*.hpp"))
    files.extend(sorted(AXLE_ROOT.glob("*.hpp")))
    return files


def scan() -> dict[str, object]:
    """Scan the tree and return the module graph state."""
    edges: set[tuple[str, str]] = set()
    self_includes: list[str] = []
    aggregate_includes: list[str] = []
    for header in _headers():
        owner = _module_of(header)
        for include in _project_includes(header):
            target = _resolve(include)
            if target is None:
                continue
            rel = str(header.relative_to(KERNEL_ROOT)).replace("\\", "/")
            if target.resolve() == header.resolve():
                self_includes.append(rel)
                continue
            other = _module_of(target)
            if owner != other:
                edges.add((owner, other))
            if (
                header.name == "functions.hpp"
                and target.name == "functions.hpp"
                and owner != other
            ):
                aggregate_includes.append(rel)
    return {
        "edges": sorted(edges),
        "self_includes": sorted(self_includes),
        "aggregate_includes": sorted(aggregate_includes),
        "cycles": find_cycles(edges),
    }


def find_cycles(edges: set[tuple[str, str]]) -> list[list[str]]:
    """Return the non-trivial strongly connected components."""
    graph: dict[str, set[str]] = defaultdict(set)
    for source, target in edges:
        graph[source].add(target)
    index: dict[str, int] = {}
    low: dict[str, int] = {}
    stack: list[str] = []
    on_stack: set[str] = set()
    counter = [0]
    cycles: list[list[str]] = []

    def strong_connect(node: str) -> None:
        index[node] = low[node] = counter[0]
        counter[0] += 1
        stack.append(node)
        on_stack.add(node)
        for neighbour in sorted(graph.get(node, ())):
            if neighbour not in index:
                strong_connect(neighbour)
                low[node] = min(low[node], low[neighbour])
            elif neighbour in on_stack:
                low[node] = min(low[node], index[neighbour])
        if low[node] == index[node]:
            component: list[str] = []
            while True:
                member = stack.pop()
                on_stack.discard(member)
                component.append(member)
                if member == node:
                    break
            if len(component) > 1:
                cycles.append(sorted(component))

    for node in sorted(set(graph) | {t for ts in graph.values() for t in ts}):
        if node not in index:
            strong_connect(node)
    return cycles


def load_baseline() -> dict[str, object]:
    """Load the recorded layering baseline."""
    if not BASELINE.is_file():
        raise SystemExit(
            f"no layering baseline at {BASELINE}; run --record-baseline first"
        )
    return json.loads(BASELINE.read_text(encoding="utf-8"))


def report(state: dict[str, object]) -> None:
    """Print a summary of the module graph state."""
    edges = state["edges"]
    print(f"modules with outbound edges: {len({e[0] for e in edges})}")  # ty: ignore[not-iterable]
    print(f"distinct module edges      : {len(edges)}")  # ty: ignore[invalid-argument-type]
    print(f"self-including headers     : {len(state['self_includes'])}")  # ty: ignore[invalid-argument-type]
    print(f"cross-aggregate includes   : {len(state['aggregate_includes'])}")  # ty: ignore[invalid-argument-type]
    print(f"module cycles (SCC size>1) : {len(state['cycles'])}")  # ty: ignore[invalid-argument-type]
    if state["cycles"]:
        for cycle in state["cycles"]:  # ty: ignore[not-iterable]
            print("  cycle: " + " <-> ".join(cycle))


def main() -> int:
    """Run the gate and return its exit status."""
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--record-baseline", action="store_true")
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--strict", action="store_true")
    mode.add_argument("--report", action="store_true")
    args = parser.parse_args()

    state = scan()
    if args.record_baseline:
        BASELINE.write_text(
            json.dumps(state, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(f"recorded baseline -> {BASELINE}")
        report(state)
        return 0

    if args.report or not (args.check or args.strict):
        report(state)
        return 0

    baseline = load_baseline()
    known = {(s, t) for s, t in baseline["edges"]}  # ty: ignore[not-iterable]
    current = {(s, t) for s, t in state["edges"]}  # ty: ignore[not-iterable]
    new_edges = sorted(current - known)
    failures: list[str] = []
    if new_edges:
        failures.append(f"new module edges: {new_edges}")
    for key in ("self_includes", "aggregate_includes", "cycles"):
        if len(state[key]) > len(baseline[key]):  # ty: ignore[invalid-argument-type]
            failures.append(
                f"{key} grew: {len(baseline[key])} -> {len(state[key])}"  # ty: ignore[invalid-argument-type]
            )
    if args.strict:
        for key in ("self_includes", "aggregate_includes", "cycles"):
            if state[key]:
                failures.append(f"{key} not empty: {len(state[key])}")  # ty: ignore[invalid-argument-type]
    report(state)
    if failures:
        print("\nFAIL")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print("\nOK: layering matches the recorded baseline")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
