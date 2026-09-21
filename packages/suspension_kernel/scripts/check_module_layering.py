#!/usr/bin/env python
"""
Module-layering gate for the suspension kernel C++ sources.

The kernel is built as one static library per module.  This gate scans the real
dependency evidence -- every module header *and* every translation unit under
``cpp/src`` -- and enforces these rules:

1. no module-level dependency cycle (the module graph must stay a DAG);
2. no module header pulls in another module's ``functions.hpp`` aggregate;
3. no header includes itself;
4. no dependency edge appears that is not in the recorded baseline, where the
   header edges and the source edges are compared separately;
5. a legacy module may only exist while the migration mode can trace it to its
   declared successors (``migration_map``); the final mode refuses it.

A *source edge* is a module-level dependency of a translation unit: the module
that owns the ``.cpp``, the module that owns a project ``#include`` it resolves,
and the module that declares a symbol the unit actually references.  The source
edges are computed from the scan, never copied from the baseline; the baseline
only says which edges are already known.

The baseline exists because the current tree is *not* clean: it was recorded
from the tree as it stands so that later refactoring can only remove edges.
``--strict`` additionally requires zero rule-1/2/3 violations and is the gate
the modularisation phase must reach.  ``--strict --final`` is the end-state
gate: it also requires the target module set to exist and every legacy module
and legacy include/source path to be gone.

Modes are explicit CLI flags on purpose -- a migration exemption must never be
reachable through the environment.

Commands used by the epic (03-09 call the same entry points):

    --report            print the scan (headers, cpp files, both edge sets)
    --check             migration mode: compare with the baseline
    --strict            migration mode: the above plus zero cycles/self/aggregate
    --strict --final    end state: the above plus legacy modules/paths absent
    --record-baseline   re-record the scan; curated fields are preserved

``layering_baseline.json`` must be reviewed by hand before it is committed:
``--record-baseline`` never writes the migration map, the legacy module
registration or the per-edge explanations, and it prints the edge diff so an
unexplained new edge cannot hide behind a re-recording.
"""

from __future__ import annotations

import argparse
import json
import re
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, cast

KERNEL_ROOT = Path(__file__).resolve().parents[1]
INCLUDE_ROOT = KERNEL_ROOT / "cpp" / "include"
SOURCE_ROOT = KERNEL_ROOT / "cpp" / "src"
AXLE_ROOT = KERNEL_ROOT / "cpp" / "axle_dynamics"
BASELINE = KERNEL_ROOT / "layering_baseline.json"

TIRE_SUBMODULES = ("brush", "common", "fiala", "pac2002")

MODE_MIGRATION = "migration"
MODE_FINAL = "final"

#: Module names the modularisation targets must provide.  They are *not*
#: required to exist before 03-08: the migration mode reports them as missing and
#: only the final mode fails on them.
TARGET_MODULES = (
    "mb_numeric",
    "mb_dual",
    "mb_config",
    "mb_linear",
    "mb_joint",
    "mb_input",
    "mb_solve_static",
    "mb_solve_dynamic",
    "mb_element",
    "mb_assembly",
    "mb_force",
)

#: Module names the modularisation replaces.  They may only exist while the
#: migration mode can trace them into ``MIGRATION_MAP``.
LEGACY_MODULES = (
    "mb_base",
    "mb_vehicle",
    "mb_suspension",
    "mb_integrator",
    "mb_static",
    "mb_linalg",
    "mb_constraint",
)

#: The declared successor of each legacy module, from the epic's C++ migration
#: matrix.  A legacy edge is *explained* when both of its legacy endpoints are
#: covered here; the mapping lives in code so a baseline edit cannot invent an
#: explanation the gate does not know about.
MIGRATION_MAP: dict[str, dict[str, object]] = {
    "mb_base": {
        "successors": ("mb_numeric", "mb_dual", "mb_config"),
        "reason": "vectors/quaternions/rotations/curves -> mb_numeric, the dual "
        "algebra -> mb_dual, env/version/runtime diagnostics -> mb_config",
    },
    "mb_linalg": {
        "successors": ("mb_linear",),
        "reason": "factorisation and linear solves -> mb_linear",
    },
    "mb_constraint": {
        "successors": ("mb_joint",),
        "reason": "constraint rows and analytic Jacobians -> mb_joint",
    },
    "mb_suspension": {
        "successors": ("mb_element",),
        "reason": "spring/damper/bushing/anti-roll elements -> mb_element",
    },
    "mb_vehicle": {
        "successors": ("mb_element", "mb_force", "mb_assembly"),
        "reason": "steering and drive/brake -> mb_element, the external force "
        "bus -> mb_force, vehicle registration and build_model -> mb_assembly",
    },
    "mb_integrator": {
        "successors": ("mb_solve_dynamic",),
        "reason": "residual, Newton step, step control and events -> "
        "mb_solve_dynamic",
    },
    "mb_static": {
        "successors": ("mb_solve_static",),
        "reason": "static trim, projection and least-squares helpers -> "
        "mb_solve_static",
    },
}

BASELINE_VERSION = 2

#: A declaration candidate is matched against the *statement* text collected
#: outside function bodies, so a call inside an inline body is not mistaken for
#: a declaration.
_SYMBOL_KEYWORDS = frozenset(
    {
        "if",
        "for",
        "while",
        "switch",
        "return",
        "sizeof",
        "catch",
        "do",
        "else",
        "case",
        "assert",
        "static_assert",
        "void",
        "int",
        "bool",
        "auto",
    }
)
#: A statement already ended at its ``;``, so the declaration is the whole
#: statement text.
_SYMBOL_DECLARATION = re.compile(
    r"(?:^|[\s*&])([A-Za-z_]\w*)\s*\([^;{}]*\)\s*(?:const)?\s*\Z", re.S
)
_IDENTIFIER = re.compile(r"[A-Za-z_]\w*")


def _records(value: object) -> list[dict[str, object]]:
    """Return a JSON list of records from the scan or the baseline."""
    return cast("list[dict[str, object]]", value)


def _names(value: object) -> set[str]:
    """Return a JSON list of module names as a set."""
    return {str(name) for name in cast("Iterable[object]", value)}
_IDENTIFIER = re.compile(r"[A-Za-z_]\w*")
#: A member access is not a project symbol reference: the owning class already
#: comes from an included header.
_MEMBER_ACCESS = re.compile(r"(?:->|\.)\s*[A-Za-z_]\w*")
#: Only these introduce a scope whose statements are declarations.
_TYPE_SCOPE = re.compile(r"\b(namespace|class|struct|union|enum|extern)\b")
#: Marker for a skipped function body: a statement containing it is not a
#: declaration.
_BODY_MARKER = chr(0)


def _strip_comments(text: str) -> str:
    """Remove comments and literals so prose cannot look like a symbol use."""
    out: list[str] = []
    index = 0
    length = len(text)
    while index < length:
        if text.startswith("//", index):
            newline = text.find("\n", index)
            index = length if newline == -1 else newline
            continue
        if text.startswith("/*", index):
            end = text.find("*/", index)
            index = length if end == -1 else end + 2
            continue
        char = text[index]
        if char in "\"'":
            closing = text.find(char, index + 1)
            index = length if closing == -1 else closing + 1
            out.append(" ")
            continue
        out.append(char)
        index += 1
    return "".join(out)
_BODY_MARKER = chr(0)


def _namespace_scope_statements(text: str) -> list[str]:
    """
    Split a header into the statements declared outside function bodies.

    Comments, string literals and function bodies are removed, so the only
    declaration patterns left are at namespace scope.
    """
    statements: list[str] = []
    current: list[str] = []
    #: Enclosing scopes: "type" for namespace/class bodies, "body" for removed
    #: function bodies.
    scopes: list[str] = []
    index = 0
    length = len(text)
    while index < length:
        char = text[index]
        if text.startswith("//", index):
            newline = text.find("\n", index)
            index = length if newline == -1 else newline
            continue
        if text.startswith("/*", index):
            end = text.find("*/", index)
            index = length if end == -1 else end + 2
            continue
        if char in "\"'":
            closing = text.find(char, index + 1)
            index = length if closing == -1 else closing + 1
            continue
        if char == "{":
            signature = "".join(current).rstrip()
            # Only a namespace or class body keeps its statements; everything
            # else is a function body, a lambda or an initialiser and is dropped.
            if scopes and scopes[-1] == "body":
                scopes.append("body")
                current = []
            elif _TYPE_SCOPE.search(signature) or not signature:
                scopes.append("type")
            else:
                scopes.append("body")
                current.append(_BODY_MARKER)
            index += 1
            continue
        if char == "}":
            if scopes:
                scopes.pop()
            current = []
            index += 1
            continue
        if scopes and scopes[-1] == "body":
            index += 1
            continue
        if char == ";":
            statements.append("".join(current))
            current = []
            index += 1
            continue
        current.append(char)
        index += 1
    return statements


@dataclass(frozen=True)
class Finding:
    """One gate violation, with a machine-checkable kind."""

    kind: str
    subject: str
    message: str


# --------------------------------------------------------------------------- #
# paths and modules
# --------------------------------------------------------------------------- #


def _tire_module(parts: tuple[str, ...]) -> str | None:
    if parts and parts[0] == "mb_tire" and len(parts) > 2 and parts[1] in TIRE_SUBMODULES:
        return f"mb_tire_{parts[1]}"
    return None


def _header_module(
    path: Path, include_root: Path, source_root: Path | None = None
) -> str:
    """Return the module that owns a public or private header path."""
    try:
        rel = path.relative_to(include_root)
    except ValueError:
        if source_root is not None and path.is_relative_to(source_root):
            return source_module(path, source_root)
        # cpp/axle_dynamics/*.hpp is the same public C ABI surface.
        return "abi"
    tire = _tire_module(rel.parts)
    return tire if tire else rel.parts[0]

def source_module(path: Path, source_root: Path) -> str:
    """Return the module that owns a translation unit (``cpp/src/<dir>``)."""
    rel = path.relative_to(source_root)
    parts = rel.parts
    if parts[0] == "abi":
        return "abi"
    tire = _tire_module(("mb_tire", *parts[1:]))
    if tire:
        return tire
    return f"mb_{parts[0]}"


def legacy_paths() -> dict[str, tuple[str, str]]:
    """Return the legacy include/source path of every legacy module."""
    return {
        module: (f"cpp/include/{module}", f"cpp/src/{module[3:]}")
        for module in LEGACY_MODULES
    }


# --------------------------------------------------------------------------- #
# scanning
# --------------------------------------------------------------------------- #


def _project_includes(path: Path) -> list[str]:
    """Return the quoted project includes of one file, in source order."""
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


def _resolve(
    include: str,
    *,
    include_root: Path,
    source_root: Path,
    axle_root: Path,
    includer: Path,
) -> Path | None:
    """Resolve a quoted include the way the compiler does for this tree."""
    if include.startswith(("mb_", "abi/")):
        candidate = include_root / include
        if candidate.is_file():
            return candidate
    local = includer.parent / include
    if local.is_file():
        return local
    candidate = axle_root / include
    if candidate.is_file():
        return candidate
    # A source-relative include that names a project header, e.g.
    # ``mb_model/types.hpp`` reached from a unit in another module directory.
    candidate = source_root / include
    return candidate if candidate.is_file() else None


def _module_of(
    path: Path, *, include_root: Path, source_root: Path, axle_root: Path
) -> str | None:
    if path.is_relative_to(include_root):
        return _header_module(path, include_root, source_root)
    if path.is_relative_to(source_root):
        return source_module(path, source_root)
    if path.is_relative_to(axle_root):
        return "abi"
    return None


def _headers(include_root: Path, source_root: Path, axle_root: Path) -> list[Path]:
    files = sorted(include_root.rglob("*.hpp"))
    files.extend(sorted(source_root.rglob("*.hpp")))
    files.extend(sorted(axle_root.glob("*.hpp")))
    return files


def _translation_units(source_root: Path) -> list[Path]:
    return sorted(source_root.rglob("*.cpp"))


def _declared_symbols(header: Path) -> set[str]:
    """Return the free-function names a header declares."""
    text = header.read_text(encoding="utf-8", errors="replace")
    names: set[str] = set()
    for statement in _namespace_scope_statements(text):
        if _BODY_MARKER in statement:
            continue
        head, separator, _ = statement.partition("(")
        if separator and ("=" in head or len(head.split()) < 2):
            # An initialiser or a function-like macro, not a declaration.
            continue
        for match in _SYMBOL_DECLARATION.finditer(statement):
            names.add(match.group(1))
    return names - _SYMBOL_KEYWORDS



def scan(root: Path | str = KERNEL_ROOT) -> dict[str, object]:
    """
    Scan ``root`` and return the module graph state.

    Header edges come from the include graph of the module headers; source edges
    come from the translation units: the module owning a project ``#include``
    and the module declaring a referenced symbol.  Both are derived from the
    files, so a new dependency is visible without touching the baseline.
    """
    root = Path(root)
    include_root = root / "cpp" / "include"
    source_root = root / "cpp" / "src"
    axle_root = root / "cpp" / "axle_dynamics"
    headers = _headers(include_root, source_root, axle_root)
    units = _translation_units(source_root)

    header_edges: set[tuple[str, str]] = set()
    self_includes: list[str] = []
    aggregate_includes: list[str] = []
    header_modules: dict[Path, str] = {}
    declared: dict[str, set[str]] = defaultdict(set)
    direct_includes: dict[Path, list[str]] = {}

    for header in headers:
        owner = _header_module(header, include_root, source_root)
        header_modules[header] = owner
        declared[owner] |= _declared_symbols(header)
        includes = _project_includes(header)
        direct_includes[header] = includes
        for include in includes:
            target = _resolve(
                include,
                include_root=include_root,
                source_root=source_root,
                axle_root=axle_root,
                includer=header,
            )
            if target is None:
                continue
            rel = str(header.relative_to(root)).replace("\\", "/")
            if target.resolve() == header.resolve():
                self_includes.append(rel)
                continue
            other = _module_of(
                target,
                include_root=include_root,
                source_root=source_root,
                axle_root=axle_root,
            )
            if other is None or other == owner:
                continue
            header_edges.add((owner, other))
            if (
                header.name == "functions.hpp"
                and target.name == "functions.hpp"
                and owner != other
            ):
                aggregate_includes.append(rel)

    def reachable_modules(header: Path, seen: set[Path]) -> set[str]:
        """Modules reachable from a header through project includes."""
        modules: set[str] = set()
        for include in direct_includes.get(header, ()):
            target = _resolve(
                include,
                include_root=include_root,
                source_root=source_root,
                axle_root=axle_root,
                includer=header,
            )
            if target is None or target in seen:
                continue
            seen.add(target)
            other = _module_of(
                target,
                include_root=include_root,
                source_root=source_root,
                axle_root=axle_root,
            )
            if other is not None:
                modules.add(other)
            if target.suffix == ".hpp":
                modules |= reachable_modules(target, seen)
        return modules

    source_edges: set[tuple[str, str]] = set()
    evidence_index: dict[tuple[str, str, str], dict[str, object]] = {}
    for unit in units:
        owner = source_module(unit, source_root)
        relative = str(unit.relative_to(root)).replace("\\", "/")
        text = unit.read_text(encoding="utf-8", errors="replace")
        per_target_includes: dict[str, list[str]] = defaultdict(list)
        closure: set[str] = set()
        for include in _project_includes(unit):
            target = _resolve(
                include,
                include_root=include_root,
                source_root=source_root,
                axle_root=axle_root,
                includer=unit,
            )
            if target is None:
                continue
            other = _module_of(
                target,
                include_root=include_root,
                source_root=source_root,
                axle_root=axle_root,
            )
            if other is not None and other != owner:
                per_target_includes[other].append(include)
                source_edges.add((owner, other))
            if target.suffix == ".hpp":
                closure |= reachable_modules(target, {target})

        referenced = set(_IDENTIFIER.findall(_MEMBER_ACCESS.sub(" ", _strip_comments(text))))
        per_target_symbols: dict[str, list[str]] = defaultdict(list)
        # A symbol the unit's own module declares is not a cross-module
        # reference, even when another module's header declares it as well.
        own_symbols = declared.get(owner, set())
        for module in sorted(closure):
            if module == owner:
                continue
            hits = sorted((declared.get(module, set()) & referenced) - own_symbols)
            if not hits:
                continue
            source_edges.add((owner, module))
            per_target_symbols[module] = hits

        for target in sorted(set(per_target_includes) | set(per_target_symbols)):
            kinds = []
            if per_target_includes.get(target):
                kinds.append("tu_include")
            if per_target_symbols.get(target):
                kinds.append("symbol_reference")
            evidence_index[(relative, owner, target)] = {
                "source": relative,
                "module": owner,
                "target": target,
                "kinds": kinds,
                "includes": sorted(per_target_includes.get(target, [])),
                "symbols": per_target_symbols.get(target, []),
            }

    modules = set(header_modules.values()) | {
        source_module(u, source_root) for u in units
    }
    graph = header_edges | source_edges
    mutual_edges = sorted(
        [min(source, target), max(source, target)]
        for source, target in graph
        if (target, source) in graph and source < target
    )
    return {
        "root": str(root),
        "headers": [str(h.relative_to(root)).replace("\\", "/") for h in headers],
        "cpp_files": [str(u.relative_to(root)).replace("\\", "/") for u in units],
        "modules": sorted(modules),
        "header_edges": sorted(header_edges),
        "source_edges": sorted(source_edges),
        # Kept for readers of the pre-source-edge schema: the union of both sets.
        "edges": sorted(graph),
        "source_edge_evidence": [
            evidence_index[key] for key in sorted(evidence_index)
        ],
        # Pairs that depend on each other: the reverse-edge ledger.
        "mutual_edges": mutual_edges,
        "self_includes": sorted(self_includes),
        "aggregate_includes": sorted(aggregate_includes),
        "cycles": find_cycles(graph),
    }


def find_cycles(edges: Iterable[tuple[str, str]]) -> list[list[str]]:
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


def module_presence(root: Path | str = KERNEL_ROOT) -> set[str]:
    """Return the modules the tree currently provides."""
    return set(scan(root)["modules"])  # ty: ignore[invalid-argument-type]


def missing_target_modules(present: Iterable[str]) -> list[str]:
    """Return the target modules that do not exist yet."""
    present = set(present)
    return [module for module in TARGET_MODULES if module not in present]


def present_legacy_modules(present: Iterable[str]) -> list[str]:
    """Return the legacy modules that are still in the tree."""
    present = set(present)
    return [module for module in LEGACY_MODULES if module in present]

def present_legacy_paths(root: Path | str = KERNEL_ROOT) -> list[str]:
    """Return the legacy include/source directories that still exist."""
    root = Path(root)
    found = [
        relative
        for paths in legacy_paths().values()
        for relative in paths
        if (root / relative).is_dir()
    ]
    return sorted(found)


# --------------------------------------------------------------------------- #
# baseline
# --------------------------------------------------------------------------- #


def load_baseline(path: Path = BASELINE) -> dict[str, object]:
    """Load the reviewed layering baseline."""
    if not path.is_file():
        raise SystemExit(f"no layering baseline at {path}; run --record-baseline first")
    return json.loads(path.read_text(encoding="utf-8"))


def baseline_state(scan_state: dict[str, object]) -> dict[str, object]:
    """Reduce a scan to the fields the baseline records."""
    return {
        "version": BASELINE_VERSION,
        "header_edges": scan_state["header_edges"],
        "source_edges": scan_state["source_edges"],
        # Compatibility alias for readers of the pre-source-edge schema.
        "edges": scan_state["edges"],
        # One record per module pair: the files, includes and symbols that
        # produce the edge.
        "source_edge_evidence": aggregate_evidence(scan_state),
        "mutual_edges": scan_state["mutual_edges"],
        "self_includes": scan_state["self_includes"],
        "aggregate_includes": scan_state["aggregate_includes"],
        "cycles": scan_state["cycles"],
    }


def aggregate_evidence(state: dict[str, object]) -> list[dict[str, object]]:
    """Group the per-file source-edge evidence by module pair."""
    grouped: dict[tuple[str, str], dict[str, set[str]]] = {}
    for record in state["source_edge_evidence"]:  # ty: ignore[not-iterable]
        key = (str(record["module"]), str(record["target"]))
        entry = grouped.setdefault(
            key,
            {"kinds": set(), "sources": set(), "includes": set(), "symbols": set()},
        )
        entry["kinds"].update(str(kind) for kind in record["kinds"])
        entry["sources"].add(str(record["source"]))
        entry["includes"].update(str(item) for item in record["includes"])
        entry["symbols"].update(str(item) for item in record["symbols"])
    return [
        {
            "module": module,
            "target": target,
            "kinds": sorted(grouped[(module, target)]["kinds"]),
            "sources": sorted(grouped[(module, target)]["sources"]),
            "includes": sorted(grouped[(module, target)]["includes"]),
            "symbols": sorted(grouped[(module, target)]["symbols"]),
        }
        for module, target in sorted(grouped)
    ]


def _curated_keys() -> tuple[str, ...]:
    """Fields a re-recording must never invent."""
    return (
        "migration_map",
        "legacy_modules",
        "kept_modules",
        "target_modules",
        "notes",
        "header_only_edges",
        "reviewed_new_edges",
    )

def _edge_key(edge: object) -> tuple[str, str]:
    source, target = edge  # type: ignore[misc]
    return str(source), str(target)


def _baseline_edges(baseline: dict[str, object], key: str) -> set[tuple[str, str]]:
    raw = baseline.get(key)
    if raw is None:
        return set()
    return {_edge_key(edge) for edge in raw}  # ty: ignore[not-iterable]


def _reviewed_entries(baseline: dict[str, object]) -> list[dict[str, object]]:
    """Return the reviewed ledger of edges added after the header-only baseline."""
    entries = _records(baseline.get("reviewed_new_edges", []))
    return [dict(entry) for entry in entries]  # ty: ignore[not-iterable]


def evaluate(
    state: dict[str, object],
    baseline: dict[str, object],
    *,
    mode: str = MODE_MIGRATION,
    strict: bool = False,
) -> list[Finding]:
    """Return the findings of one gate run; an empty list means the gate passes."""
    findings: list[Finding] = []

    if baseline.get("version") != BASELINE_VERSION:
        findings.append(
            Finding(
                "baseline_schema",
                "version",
                f"baseline version {baseline.get('version')!r} is not "
                f"{BASELINE_VERSION}; migrate it from a reviewed scan",
            )
        )
    for key in ("header_edges", "source_edges"):
        if key not in baseline:
            findings.append(
                Finding(
                    "baseline_schema",
                    key,
                    f"baseline has no {key}: the gate cannot compare that edge set",
                )
            )

    # Every source edge must carry its evidence record, and every edge added
    # after the header-only baseline must be in the reviewed ledger: a
    # re-recording of the edge lists cannot explain a dependency away.
    evidence_edges = {
        (str(record["module"]), str(record["target"]))
        for record in _records(baseline.get("source_edge_evidence", []))
    }
    for source, target in sorted(_baseline_edges(state, "source_edges")):
        if (source, target) not in evidence_edges:
            findings.append(
                Finding(
                    "missing_source_evidence",
                    f"{source}->{target}",
                    f"{source} -> {target} has no source_edge_evidence record",
                )
            )
    header_only = _baseline_edges(baseline, "header_only_edges")
    reviewed = {_edge_key(entry["edge"]) for entry in _reviewed_entries(baseline)}
    for key in ("header_edges", "source_edges"):
        for edge in sorted(_baseline_edges(state, key)):
            if edge in header_only or edge in reviewed:
                continue
            findings.append(
                Finding(
                    "unreviewed_edge",
                    f"{edge[0]}->{edge[1]}",
                    f"{edge[0]} -> {edge[1]} is not in the header-only baseline and "
                    "has no reviewed_new_edges entry",
                )
            )

    header_known = _baseline_edges(baseline, "header_edges")
    source_known = _baseline_edges(baseline, "source_edges")
    # Fall back to the compatibility alias so an un-migrated baseline is still
    # compared rather than silently accepted.
    known = header_known | source_known | _baseline_edges(baseline, "edges")

    for kind, key, registered in (
        ("new_header_edge", "header_edges", header_known),
        ("new_source_edge", "source_edges", source_known),
    ):
        for source, target in sorted(_baseline_edges(state, key)):
            if (source, target) in registered:
                continue
            if (target, source) in known:
                findings.append(
                    Finding(
                        "reverse_edge",
                        f"{source}->{target}",
                        f"{source} -> {target} reverses the recorded edge "
                        f"{target} -> {source}",
                    )
                )
            else:
                findings.append(
                    Finding(
                        kind,
                        f"{source}->{target}",
                        f"{source} -> {target} is not in the baseline {key}",
                    )
                )

    # ``self_includes``/``aggregate_includes`` must stay empty; ``cycles`` may
    # still trace to a legacy module in the migration phase.
    registered_legacy = _names(baseline.get("legacy_modules", []))
    for key in ("self_includes", "aggregate_includes"):
        current = set(map(str, cast("Iterable[object]", state[key])))
        recorded = set(map(str, cast("Iterable[object]", baseline.get(key, []))))
        added = current - recorded
        if added:
            findings.append(
                Finding(
                    key.rstrip("s"),
                    key,
                    f"{key} changed: new entries {sorted(added)}",
                )
            )
        if strict and current:
            findings.append(
                Finding(
                    key.rstrip("s"),
                    key,
                    f"{key} not empty: {len(current)}",  # ty: ignore[invalid-argument-type]
                )
            )

    current_cycles = [tuple(cycle) for cycle in state["cycles"]]  # ty: ignore[not-iterable]
    recorded_cycles = {tuple(cycle) for cycle in baseline.get("cycles", [])}  # ty: ignore[not-iterable]
    for cycle in current_cycles:
        traceable = any(module in registered_legacy for module in cycle)
        if mode == MODE_FINAL:
            findings.append(
                Finding(
                    "cycle",
                    " <-> ".join(cycle),
                    f"the final mode forbids the module cycle {' <-> '.join(cycle)}",
                )
            )
        elif not traceable:
            findings.append(
                Finding(
                    "cycle",
                    " <-> ".join(cycle),
                    f"module cycle {' <-> '.join(cycle)} touches no legacy module, "
                    "so the migration map does not explain it",
                )
            )
        elif strict and cycle not in recorded_cycles:
            findings.append(
                Finding(
                    "cycle",
                    " <-> ".join(cycle),
                    f"module cycle {' <-> '.join(cycle)} is not in the baseline",
                )
            )

    mutual = {tuple(pair) for pair in state["mutual_edges"]}  # ty: ignore[not-iterable]
    recorded_mutual = {tuple(pair) for pair in baseline.get("mutual_edges", [])}  # ty: ignore[not-iterable]
    for pair in sorted(mutual):
        if mode != MODE_FINAL and pair in recorded_mutual:
            continue
        findings.append(
            Finding(
                "reverse_edge",
                f"{pair[0]}<->{pair[1]}",
                f"{pair[0]} and {pair[1]} depend on each other",
            )
        )

    present = _names(state["modules"])
    current_legacy = present_legacy_modules(present)
    for module in current_legacy:
        if module not in registered_legacy:
            findings.append(
                Finding(
                    "unregistered_legacy_module",
                    module,
                    f"legacy module {module} is present but not registered in the "
                    "baseline legacy_modules",
                )
            )
    registered_modules = (
        set(baseline.get("legacy_modules", ()))  # ty: ignore[invalid-argument-type]
        | set(baseline.get("kept_modules", ()))  # ty: ignore[invalid-argument-type]
        | set(TARGET_MODULES)
    )
    for module in sorted(present - registered_modules):
        findings.append(
            Finding(
                "unregistered_module",
                module,
                f"module {module} is not a target module, a kept module or a "
                "registered legacy module",
            )
        )

    mapping = baseline.get("migration_map", {})
    if mapping != _serialisable_migration_map():
        findings.append(
            Finding(
                "migration_map_drift",
                "migration_map",
                "baseline migration_map differs from the mapping declared in the gate",
            )
        )
    explained = set(mapping) if isinstance(mapping, dict) else set()
    for key in ("header_edges", "source_edges"):
        for source, target in sorted(_baseline_edges(state, key)):
            for endpoint in (source, target):
                if endpoint in LEGACY_MODULES and endpoint not in explained:
                    findings.append(
                        Finding(
                            "unexplained_legacy_edge",
                            f"{source}->{target}",
                            f"{source} -> {target} touches {endpoint}, which the "
                            "migration map does not explain",
                        )
                    )

    if mode == MODE_FINAL:
        for module in current_legacy:
            findings.append(
                Finding(
                    "legacy_module_present",
                    module,
                    f"the final mode forbids the legacy module {module}",
                )
            )
        for relative in present_legacy_paths(Path(str(state["root"]))):
            findings.append(
                Finding(
                    "legacy_path_present",
                    relative,
                    f"the final mode forbids the legacy path {relative}",
                )
            )
        for module in missing_target_modules(present):
            findings.append(
                Finding(
                    "missing_target_module",
                    module,
                    f"the final mode requires the target module {module}",
                )
            )
    return findings


def tolerated_findings(
    state: dict[str, object], baseline: dict[str, object]
) -> list[Finding]:
    """
    Return what the migration mode tolerates but still reports.

    A module cycle that includes a registered legacy module is what 03-08
    remove; it is listed so the tolerated state is visible rather than assumed.
    """
    registered_legacy = _names(baseline.get("legacy_modules", []))
    tolerated: list[Finding] = []
    for cycle in state["cycles"]:  # ty: ignore[not-iterable]
        legacy_members = [module for module in cycle if module in registered_legacy]
        if not legacy_members:
            continue
        tolerated.append(
            Finding(
                "legacy_cycle",
                " <-> ".join(cycle),
                f"cycle {' <-> '.join(cycle)} is traced to "
                f"{', '.join(legacy_members)} and must disappear with the migration",
            )
        )
    tolerated.extend(stale_evidence(state, baseline))
    return tolerated


def _serialisable_migration_map() -> dict[str, object]:
    return {
        module: {
            "successors": list(entry["successors"]),  # ty: ignore[invalid-argument-type]
            "reason": entry["reason"],
        }
        for module, entry in MIGRATION_MAP.items()
    }


def legacy_edge_trace(state: dict[str, object]) -> list[dict[str, object]]:
    """Trace every edge with a legacy endpoint to its declared successors."""
    trace: list[dict[str, object]] = []
    for key in ("header_edges", "source_edges"):
        for source, target in sorted(_baseline_edges(state, key)):
            endpoints = [
                module for module in (source, target) if module in LEGACY_MODULES
            ]
            if not endpoints:
                continue
            trace.append(
                {
                    "kind": key.removesuffix("s"),
                    "edge": [source, target],
                    "legacy_endpoints": endpoints,
                    "successors": {
                        module: list(MIGRATION_MAP[module]["successors"])  # ty: ignore[invalid-argument-type]
                        for module in endpoints
                    },
                }
            )
    return trace


def stale_evidence(state: dict[str, object], baseline: dict[str, object]) -> list[Finding]:
    """Report recorded evidence whose kind the current scan no longer produces."""
    current: dict[tuple[str, str], set[str]] = defaultdict(set)
    for record in _records(state["source_edge_evidence"]):
        current[(str(record["module"]), str(record["target"]))] |= _names(
            record["kinds"]
        )
    stale: list[Finding] = []
    for record in _records(baseline.get("source_edge_evidence", [])):
        key = (str(record["module"]), str(record["target"]))
        recorded = _names(record["kinds"])
        if key in current and not recorded & current[key]:
            stale.append(
                Finding(
                    "stale_source_evidence",
                    f"{key[0]}->{key[1]}",
                    f"the baseline records {sorted(recorded)} for {key[0]} -> "
                    f"{key[1]}, the scan now shows {sorted(current[key])}",
                )
            )
    return stale


# --------------------------------------------------------------------------- #
# reporting and CLI
# --------------------------------------------------------------------------- #


def _counts(state: dict[str, object]) -> dict[str, int]:
    evidence = _records(state["source_edge_evidence"])
    kinds: dict[str, int] = defaultdict(int)
    both_kinds = 0
    for record in evidence:  # ty: ignore[not-iterable]
        record_kinds = _names(record["kinds"])
        for kind in record_kinds:
            kinds[kind] += 1
        if {"tu_include", "symbol_reference"} <= record_kinds:
            both_kinds += 1
    return {
        "headers": len(state["headers"]),  # ty: ignore[invalid-argument-type]
        "cpp_files": len(state["cpp_files"]),  # ty: ignore[invalid-argument-type]
        "modules": len(state["modules"]),  # ty: ignore[invalid-argument-type]
        "header_edges": len(state["header_edges"]),  # ty: ignore[invalid-argument-type]
        "source_edges": len(state["source_edges"]),  # ty: ignore[invalid-argument-type]
        "source_edge_evidence": len(evidence),
        "source_edge_tu_include": kinds["tu_include"],
        "source_edge_symbol_reference": kinds["symbol_reference"],
        "source_edge_both_kinds": both_kinds,
        "source_edge_kind_mentions": sum(kinds.values()),
        "self_includes": len(state["self_includes"]),  # ty: ignore[invalid-argument-type]
        "mutual_edges": len(state["mutual_edges"]),  # ty: ignore[invalid-argument-type]
        "aggregate_includes": len(state["aggregate_includes"]),  # ty: ignore[invalid-argument-type]
        "cycles": len(state["cycles"]),  # ty: ignore[invalid-argument-type]
    }


def report(
    state: dict[str, object],
    *,
    mode: str = MODE_MIGRATION,
    registered_legacy: Iterable[str] = (),
) -> None:
    """Print a summary of the module graph state."""
    counts = _counts(state)
    present = _names(state["modules"])
    legacy = present_legacy_modules(present)
    unregistered = sorted(set(legacy) - set(registered_legacy))
    print(f"kernel root                : {state['root']}")
    print(f"mode                       : {mode}")
    print(f"headers                    : {counts['headers']}")
    print(f"cpp translation units      : {counts['cpp_files']}")
    print(f"modules present            : {counts['modules']}")
    print(f"header edges               : {counts['header_edges']}")
    print(f"source edges               : {counts['source_edges']}")
    print(
        f"source edge evidence       : {counts['source_edge_evidence']} records "
        f"(tu_include {counts['source_edge_tu_include']}, "
        f"symbol_reference {counts['source_edge_symbol_reference']}, "
        f"both {counts['source_edge_both_kinds']}; "
        f"kind mentions {counts['source_edge_kind_mentions']})"
    )
    print(f"target modules missing     : {len(missing_target_modules(present))}")
    print(f"legacy modules present     : {len(legacy)}")
    print(f"legacy modules unregistered: {len(unregistered)} {unregistered}")
    print(f"mutual (reverse) edges     : {counts['mutual_edges']}")
    print(f"self-including headers     : {counts['self_includes']}")
    print(f"cross-aggregate includes   : {counts['aggregate_includes']}")
    print(f"module cycles (SCC size>1) : {counts['cycles']}")
    for pair in state["mutual_edges"]:  # ty: ignore[not-iterable]
        print(f"  mutual: {pair[0]} <-> {pair[1]}")
    for cycle in state["cycles"]:  # ty: ignore[not-iterable]
        print("  cycle: " + " <-> ".join(cycle))
    for key in ("self_includes", "aggregate_includes"):
        for item in state[key]:  # ty: ignore[not-iterable]
            print(f"  {key}: {item}")


def report_findings(findings: list[Finding]) -> None:
    """Print the findings of a gate run."""
    if not findings:
        print("\nOK: layering matches the recorded baseline")
        return
    print("\nFAIL")
    for finding in findings:
        print(f"  - [{finding.kind}] {finding.message}")



def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--record-baseline", action="store_true")
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--strict", action="store_true")
    mode.add_argument("--report", action="store_true")
    parser.add_argument(
        "--final",
        action="store_true",
        help="end-state gate: legacy modules and legacy paths must be gone",
    )
    parser.add_argument("--json", action="store_true", help="print the scan as JSON")
    parser.add_argument("--kernel-root", default=None)
    parser.add_argument("--baseline", default=None)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Run the gate and return its exit status."""
    args = _parse_args(argv)
    kernel_root = Path(args.kernel_root).resolve() if args.kernel_root else KERNEL_ROOT
    baseline_path = Path(args.baseline).resolve() if args.baseline else BASELINE
    mode = MODE_FINAL if args.final else MODE_MIGRATION

    state = scan(kernel_root)
    if args.json:
        print(json.dumps(state, indent=2, sort_keys=True))
        return 0

    if args.record_baseline:
        previous = load_baseline(baseline_path) if baseline_path.is_file() else {}
        recorded = baseline_state(state)
        for key in _curated_keys():
            if key in previous:
                recorded[key] = previous[key]
        # A missing migration map is filled from the gate constant so a first
        # recording is usable; the module registrations and the reviewed ledger
        # are never invented, so a re-recording cannot register a legacy module
        # or explain a new edge by itself.
        recorded.setdefault("migration_map", _serialisable_migration_map())
        missing = [
            key
            for key in ("legacy_modules", "kept_modules", "header_only_edges")
            if key not in recorded
        ]
        if missing:
            print(
                "review these fields by hand before committing the baseline: "
                f"{missing}"
            )
        if "reviewed_new_edges" not in recorded:
            print(
                "no reviewed_new_edges ledger: every edge outside header_only_edges "
                "will fail the gate until it is reviewed"
            )
        recorded["notes"] = (
            "Recorded from the scan of the tree as it stands.  The migration map, "
            "the legacy module registration and the per-edge explanations are "
            "human-curated and must be reviewed by hand; --record-baseline never "
            "writes them."
        )
        baseline_path.write_text(
            json.dumps(recorded, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(f"recorded baseline -> {baseline_path}")
        for key in ("header_edges", "source_edges"):
            added = _baseline_edges(recorded, key) - _baseline_edges(previous, key)
            print(f"  {key} the recording adds : {sorted(added)}")
            print(
                f"  {key} the recording drops: "
                f"{sorted(_baseline_edges(previous, key) - _baseline_edges(recorded, key))}"
            )
        report(
            state,
            mode=mode,
            registered_legacy=set(recorded.get("legacy_modules", ())),  # ty: ignore[invalid-argument-type]
        )
        return 0

    baseline = load_baseline(baseline_path) if baseline_path.is_file() else {}
    registered = _names(baseline.get("legacy_modules", []))
    if args.report or not (args.check or args.strict):
        report(state, mode=mode, registered_legacy=registered)
        if args.strict or args.check:
            findings = evaluate(state, baseline, mode=mode, strict=args.strict)
            report_findings(findings)
            return 1 if findings else 0
        return 0

    findings = evaluate(state, baseline, mode=mode, strict=args.strict)
    report(state, mode=mode, registered_legacy=registered)
    trace = legacy_edge_trace(state)
    if trace:
        print(f"legacy edges traced to successors : {len(trace)}")
    for finding in tolerated_findings(state, baseline):
        print(f"  tolerated [{finding.kind}] {finding.message}")
    report_findings(findings)
    return 1 if findings else 0


if __name__ == "__main__":
    raise SystemExit(main())
