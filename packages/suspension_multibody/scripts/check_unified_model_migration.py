"""Capture the pre-migration checkout and gate the unified model cutover."""

from __future__ import annotations

import argparse
import ast
import hashlib
import inspect
import io
import json
import os
import subprocess
import sys
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "packages/suspension_multibody"
EPIC = ROOT / ".codex-tasks/20261006-unified-generic-subsystems"
PREVIOUS = ROOT / ".codex-tasks/20261006-generic-multibody-evolution/raw"
TARGETS = {
    "simulate", "simulate_generic", "compile_generic", "assemble_generic",
    "GenericSubsystemAssembler", "assembly_for", "si_assembly_for_axle",
    "axle_declaration_for", "vehicle_declaration_from", "adapt_legacy_prepared_request",
    "link_wheel_supplying_rig", "merge_rig_link", "model_view", "element_rows",
    "run_request", "run_plan", "compile_request", "compile_plan", "compile_documents",
    "run_compiled", "decoder_for", "decode_result", "decode_axle_result",
    "decode_vehicle_result", "kc_case_run", "prepare_request",
}
RETIREMENT_FILES = {
    "api.py", "__init__.py", "authoring/__init__.py", "authoring/solver.py",
    "authoring/generic.py", "axle_dynamics/contract_run.py", "compilation/__init__.py",
    "compilation/compile.py", "preparation/kc_quasi_static.py", "results/__init__.py",
    "results/axle.py", "results/decoder.py", "results/vehicle.py", "rigs/bench.py",
    "simulation/__init__.py", "simulation/dispatch.py", "simulation/runner.py",
    "studies/assembly.py", "subsystems/__init__.py", "subsystems/assembler.py",
    "subsystems/element_build.py", "subsystems/entry.py", "subsystems/explicit.py",
    "subsystems/rig_link.py", "subsystems/si_assembly.py", "subsystems/template_runtime.py",
    "subsystems/torque_elements.py", "vehicle/service.py",
}
EXECUTION_SYMBOLS = {
    "run_request", "run_plan", "compile_plan", "si_assembly_for_axle", "assembly_for",
    "kc_case_run", "replay_case", "compose_axle", "prepare_vehicle_run", "vehicle_dynamics_run",
}
ROUTE_TARGETS = (
    ("authoring/loader.py", "DocumentLoader.load"),
    ("compilation/resolved.py", "compile_resolved"),
    ("simulation/runner.py", "run_compiled"),
    ("results/envelope.py", "ResultEnvelope"),
)
CURRENT_SYMBOLS = {
    "simulate", "validate", "run_compiled", "compile_resolved", "DocumentLoader",
    "ResultEnvelope", "migrate_v1_axle", "migrate_v1_case", "migrate_v1_kc_case",
    "migrate_v1_vehicle_case", "migrate_v1_vehicle_kc_case", "migrate_v1_axle_dynamic",
    "frame_pose", "body_state", "tire_state", "element_wrench", "case_body_state",
    "case_samples", "case_residuals", "constraint_ids", "named_blocks",
    "_k_grid_states", "k_records", "c_records",
    "write_mapping", "validate_mapping",
}


def execution_names(tree: ast.AST) -> set[str]:
    """Include aliased imports and method calls in execution boundary checks."""
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names.update(alias.name.rsplit(".", 1)[-1] for alias in node.names)
        elif isinstance(node, ast.Call):
            if isinstance(node.func, ast.Name):
                names.add(node.func.id)
            elif isinstance(node.func, ast.Attribute):
                names.add(node.func.attr)
    return names


def current_references(path: Path) -> list[dict[str, Any]]:
    """Capture actual replacement calls/imports, not a common target list."""
    text = path.read_text(encoding="utf-8-sig")
    if path.suffix == ".md":
        return [{"line": line, "kind": "text", "symbols": sorted(CURRENT_SYMBOLS.intersection(text.split()))}
            for line, text in enumerate(text.splitlines(), 1)
            if any(symbol in text for symbol in CURRENT_SYMBOLS)]
    tree = ast.parse(text)
    result = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            symbols = [alias.name for alias in node.names if alias.name in CURRENT_SYMBOLS]
        elif isinstance(node, ast.Call):
            name = node.func.id if isinstance(node.func, ast.Name) else node.func.attr if isinstance(node.func, ast.Attribute) else ""
            symbols = [ast.unparse(node.func)] if name in CURRENT_SYMBOLS else []
        else:
            continue
        if symbols:
            result.append({"line": getattr(node, "lineno"), "kind": type(node).__name__, "symbols": symbols})
    return result


def digest(data: bytes) -> str:
    """Return the content identity used throughout the evidence."""
    return hashlib.sha256(data).hexdigest()


def git(*arguments: str) -> bytes:
    """Read Git state without touching the index or checkout."""
    return subprocess.run(
        ["git", "-c", "core.quotepath=false", *arguments], cwd=ROOT,
        capture_output=True, check=True,
    ).stdout


def location(symbol: Any) -> dict[str, Any]:
    """Locate one runtime registration in the actual checkout."""
    filename = inspect.getsourcefile(symbol)
    if filename is None:
        raise ValueError(f"no source for {symbol!r}")
    return {
        "file": Path(filename).relative_to(ROOT).as_posix(),
        "line": inspect.getsourcelines(symbol)[1],
        "symbol": f"{symbol.__module__}.{symbol.__qualname__}",
    }


def registrations() -> dict[str, Any]:
    """Enumerate native protocols and the single Python execution chain."""
    from suspension_multibody.authoring.loader import DocumentLoader
    from suspension_multibody.compilation.resolved import compile_resolved
    from suspension_multibody.kernel.capabilities import kernel_capability_document
    from suspension_multibody.results.raw import _decode_contract_run
    from suspension_multibody.rigs.rig import RIGS
    from suspension_multibody.studies.study import STUDIES

    protocols = sorted(kernel_capability_document()["case_families"])
    prepared = [
        {
            "assembly": "generic", "family": family,
            "entry": "authoring.loader.DocumentLoader.load",
            "registration": location(DocumentLoader.load),
            "build": "authoring.generic.assemble_generic",
            "submit": "simulation.runner.run_compiled",
            "decode": "results.raw._decode_contract_run -> ResultEnvelope",
            "target": "ResolvedModel -> unified compiler -> ResultEnvelope",
        } for family in protocols
    ]
    return {
        "preparations": prepared,
        "compilers": [{"key": ["generic", key], **location(compile_resolved)} for key in protocols],
        "emitters": [{"key": key, **location(compile_resolved)} for key in protocols],
        "rigs": [
            {"key": key, "study": value.study,
             "route": value.route, "drives": list(value.coordinate_names())}
            for key, value in RIGS.items()
        ],
        "combinations": [["generic", key] for key in RIGS],
        "studies": [
            {"key": key, "time_semantics": value.time_semantics,
             "tire_activation": value.tire_activation}
            for key, value in STUDIES.items()
        ],
        "decoder": location(_decode_contract_run),
        "generic": {
            "entry": "api.simulate",
            "compiler": "compilation.resolved.compile_resolved",
            "families": protocols,
            "registered_with_automotive": True,
        },
        "comparison": {"entry": "cases.comparison", "kind": "independent comparison"},
    }


def consumers() -> list[dict[str, Any]]:
    """Record calls, imports and text consumers with exact source locations."""
    result: list[dict[str, Any]] = []
    for base in (PACKAGE / "src", PACKAGE / "scripts", PACKAGE / "tests", PACKAGE / "examples", ROOT / "scripts", ROOT / "examples"):
        if not base.exists():
            continue
        for path in sorted(base.rglob("*.py")):
            if "__pycache__" in path.parts:
                continue
            tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
            relative = path.relative_to(ROOT).as_posix()
            category = "test" if "/tests/" in relative else "production"
            if "/adams/" in relative:
                category = "adams"
            elif "/report/" in relative:
                category = "report"
            elif "/examples/" in relative or relative.startswith("examples/"):
                category = "example"
            for node in ast.walk(tree):
                symbols: list[str] = []
                if isinstance(node, ast.Call):
                    func = node.func
                    name = func.id if isinstance(func, ast.Name) else func.attr if isinstance(func, ast.Attribute) else ""
                    if name in TARGETS:
                        symbols.append(ast.unparse(func))
                elif isinstance(node, ast.ImportFrom):
                    symbols.extend(
                        f"{node.module}.{alias.name}" for alias in node.names
                        if alias.name in TARGETS or any(
                            part in (node.module or "") for part in ("si_assembly", "rig_link", "presets.legacy", "model_view")
                        )
                    )
                if symbols:
                    result.append({
                        "file": relative, "line": getattr(node, "lineno", 0), "kind": type(node).__name__,
                        "symbols": symbols, "category": category,
                        "target": "unified document/resolver/compiler/measurement",
                    })
    for path in sorted((PACKAGE / "docs").rglob("*.md")):
        for line, text in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
            matched = sorted(name for name in TARGETS if name in text)
            if matched:
                result.append({"file": path.relative_to(ROOT).as_posix(), "line": line,
                               "symbols": matched, "category": "documentation", "kind": "text"})
    tests = [row for row in result if row["category"] == "test"]
    for row in result:
        names = {str(value).rsplit(".", 1)[-1] for value in row["symbols"]}
        row["tests"] = sorted({
            test["file"] for test in tests
            if names.intersection(str(value).rsplit(".", 1)[-1] for value in test["symbols"])
        })
    return result


def snapshot(directory: Path) -> dict[str, Any]:
    """Preserve complete patches and non-ignored untracked file contents."""
    directory.mkdir(parents=True, exist_ok=True)
    metadata: dict[str, Any] = {}
    commands = {
        "head": ("rev-parse", "HEAD"), "branch": ("branch", "--show-current"),
        "status": ("status", "--porcelain=v1", "-z"),
        "staged": ("diff", "--cached", "--binary", "--full-index"),
        "unstaged": ("diff", "--binary", "--full-index"),
    }
    for name, arguments in commands.items():
        content = git(*arguments)
        (directory / f"{name}.txt").write_bytes(content)
        metadata[name] = {"sha256": digest(content), "bytes": len(content)}
    untracked = git("ls-files", "--others", "--exclude-standard", "-z")
    files = [ROOT / name.decode("utf-8") for name in untracked.split(b"\0") if name]
    with zipfile.ZipFile(directory / "untracked.zip", "w", compression=zipfile.ZIP_DEFLATED) as archive:
        metadata["untracked"] = []
        for path in files:
            content = path.read_bytes()
            relative = path.relative_to(ROOT).as_posix()
            archive.writestr(relative, content)
            metadata["untracked"].append({"file": relative, "sha256": digest(content)})
    return metadata


def fingerprints() -> dict[str, str]:
    """Fingerprint source and every frozen data file, without rewriting it."""
    result: dict[str, str] = {}
    for package in ("suspension_multibody", "suspension_kernel", "suspension_contracts"):
        for component in ("src", "cpp", "scripts"):
            base = ROOT / "packages" / package / component
            if not base.exists():
                continue
            for path in sorted(base.rglob("*")):
                if path.is_file() and path.suffix in {".py", ".json", ".hpp", ".cpp", ".h", ".c"} and "__pycache__" not in path.parts:
                    result[path.relative_to(ROOT).as_posix()] = digest(path.read_bytes())
    return result


def compiled_examples(directory: Path) -> list[dict[str, Any]]:
    """Save actual pre-migration native documents; this does not run a solver."""
    from suspension_multibody.api import validate
    from suspension_multibody.authoring.documents import AssemblyDocument

    result = []
    directory.mkdir(parents=True, exist_ok=True)
    for path in sorted((PACKAGE / "examples/generic_multibody").glob("*.assembly.json")):
        document = AssemblyDocument.load(path)
        case = json.loads((path.parent / "dynamic.case.json").read_text(encoding="utf-8"))
        compiled = validate(document, case)
        payload = compiled.model_payload
        filename = f"{path.stem}.container"
        (directory / filename).write_bytes(payload)
        result.append({"input": path.relative_to(ROOT).as_posix(),
                       "model_sha256": digest(payload), "case": case,
                       "artifact": str(directory / filename)})
    if not result:
        raise ValueError("no current generic compiled examples captured")
    return result


def capture(output: Path) -> None:
    """Capture a recoverable inventory and report incomplete reads as failures."""
    evidence: dict[str, Any] = {
        "schema_version": 1, "producer": str(Path(__file__).relative_to(ROOT)),
        "captured_at": datetime.now(timezone.utc).isoformat(), "snapshot_complete": False,
        "errors": [],
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        previous_bytes = output.read_bytes()
        previous = json.loads(previous_bytes)
        if previous["snapshot_complete"]:
            directory = output.parent / "worktree-before"
            for name in ("head", "branch", "status", "staged", "unstaged"):
                if digest((directory / f"{name}.txt").read_bytes()) != previous["git"][name]["sha256"]:
                    raise ValueError(f"original snapshot changed: {name}")
            with zipfile.ZipFile(directory / "untracked.zip") as archive:
                for row in previous["git"]["untracked"]:
                    if digest(archive.read(row["file"])) != row["sha256"]:
                        raise ValueError(f"original untracked snapshot changed: {row['file']}")
            print(f"PASS inventory: original complete snapshot retained; {output}")
            return
        (output.parent / f"inventory-failed-{digest(previous_bytes)[:12]}.json").write_bytes(previous_bytes)
        evidence["git"] = previous["git"]
        evidence["previous_errors"] = previous["errors"]
    try:
        if "git" not in evidence:
            evidence["git"] = snapshot(output.parent / "worktree-before")
        source = fingerprints()
        evidence["source_fingerprints"] = source
        evidence["production_fingerprint"] = digest(json.dumps(source, sort_keys=True).encode())
        evidence["registrations"] = registrations()
        evidence["consumers"] = consumers()
        old = json.loads((PREVIOUS / "baseline-fingerprints.json").read_text(encoding="utf-8"))
        frozen = {}
        for filename, expected in old.items():
            path = ROOT / filename.replace("\\", "/")
            actual = digest(path.read_bytes())
            if actual != expected:
                raise ValueError(f"previous frozen file changed: {filename}")
            frozen[path.relative_to(ROOT).as_posix()] = actual
        for path in (PACKAGE / "tests/data").rglob("*.json"):
            if "baseline" in path.as_posix():
                frozen[path.relative_to(ROOT).as_posix()] = digest(path.read_bytes())
        evidence["frozen_fingerprints"] = frozen
        evidence["known_failures"] = (PREVIOUS / "baseline-failures.md").read_text(encoding="utf-8")
        evidence["compiled_examples"] = compiled_examples(output.parent / "compiled-before")
        evidence["snapshot_complete"] = True
    except Exception as error:
        evidence["errors"].append(f"{type(error).__name__}: {error}")
        raise
    finally:
        output.write_text(json.dumps(evidence, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"PASS inventory: {len(evidence['consumers'])} references, "
          f"{len(evidence['frozen_fingerprints'])} frozen files; {output}")


def write_mapping(inventory: Path) -> None:
    """Bind every original non-test reference to current source and live gates."""
    evidence = json.loads(inventory.read_text(encoding="utf-8"))
    rows = []
    source_root = PACKAGE / "src/suspension_multibody"
    for original in evidence["consumers"]:
        if original["category"] == "test":
            continue
        source = ROOT / original["file"]
        relative = source.relative_to(source_root).as_posix() if source.is_relative_to(source_root) else ""
        pending = relative in RETIREMENT_FILES
        if source.suffix == ".py" and not pending:
            tree = ast.parse(source.read_text(encoding="utf-8-sig"))
            calls = {node.func.id if isinstance(node.func, ast.Name) else node.func.attr
                for node in ast.walk(tree) if isinstance(node, ast.Call)
                and isinstance(node.func, (ast.Name, ast.Attribute))}
            imports = {alias.name for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) for alias in node.names}
            old = (calls | imports) & EXECUTION_SYMBOLS
            if old:
                raise ValueError(f"external consumer still executes legacy route: {source}: {sorted(old)}")
        references = current_references(source)
        if not pending and not references:
            raise ValueError(f"consumer has no replacement source references: {source}")
        names = {name.rsplit(".", 1)[-1] for name in original["symbols"]}
        destination = (ROUTE_TARGETS[3] if any(name.startswith("decode") for name in names)
            else ROUTE_TARGETS[1] if any(name.startswith("compile") for name in names)
            else ROUTE_TARGETS[2] if any(name.startswith("run") for name in names)
            else ROUTE_TARGETS[0])
        rows.append({"file": original["file"], "line": original["line"], "kind": original["kind"],
            "symbols": original["symbols"], "category": original["category"],
            "disposition": "retire_at_cutover" if pending else "migrated",
            "source_sha256": digest(source.read_bytes()),
            "replacement_references": references,
            "target": [{"file": (source_root / filename).relative_to(ROOT).as_posix(), "symbol": symbol,
                "source_sha256": digest((source_root / filename).read_bytes())} for filename, symbol in (destination,)],
            "evidence": ["packages/suspension_multibody/tests/architecture/test_unified_model_cutover.py",
                "packages/suspension_multibody/tests/simulation/test_single_model_pipeline.py",
                "packages/suspension_multibody/scripts/case_parity_check.py"]})
    output = inventory.parent / "migration-consumers.json"
    output.write_text(json.dumps({"schema_version": 3, "inventory_sha256": digest(inventory.read_bytes()),
        "consumers": rows, "retirement_files": sorted(RETIREMENT_FILES),
        "protocols": sorted({row["family"] for row in evidence["registrations"]["preparations"]})},
        ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"consumer mapping generated: {len(rows)} rows; verification is performed by --check")


def validate_mapping(evidence: dict[str, Any], mapping: dict[str, Any]) -> None:
    """Reject invented rows, missing symbols, stale sources and absent targets."""
    original = [row for row in evidence["consumers"] if row["category"] != "test"]
    def identity(row: dict[str, Any]) -> tuple[Any, ...]:
        return row["file"], row["line"], row["kind"], tuple(row["symbols"]), row["category"]
    expected = {identity(row) for row in original}
    keys = [identity(row) for row in mapping["consumers"]]
    if len(keys) != len(set(keys)):
        raise ValueError("duplicate consumer mapping")
    if expected != set(keys):
        raise ValueError(f"consumer mapping mismatch: missing={expected-set(keys)} extra={set(keys)-expected}")
    for row in mapping["consumers"]:
        source = ROOT / row["file"]
        if digest(source.read_bytes()) != row["source_sha256"]:
            raise ValueError(f"consumer source changed after mapping: {row['file']}")
        references = current_references(source)
        if row.get("replacement_references") != references:
            raise ValueError(f"consumer replacement references changed: {row['file']}")
        if row["disposition"] == "migrated" and not references:
            raise ValueError(f"consumer has no replacement source references: {row['file']}")
        if row["disposition"] == "retire_at_cutover":
            relative = source.relative_to(PACKAGE / "src/suspension_multibody").as_posix()
            if relative not in RETIREMENT_FILES:
                raise ValueError(f"unplanned consumer retirement: {row['file']}")
        if row["disposition"] not in {"migrated", "retire_at_cutover"} or not row.get("target") or not row.get("evidence"):
            raise ValueError(f"consumer has no concrete target/evidence: {row['file']}:{row['line']}")
        for target in row["target"]:
            path = ROOT / target["file"]
            if digest(path.read_bytes()) != target["source_sha256"]:
                raise ValueError(f"target source changed after mapping: {path}")
            tree = ast.parse(path.read_text(encoding="utf-8-sig"))
            name = target["symbol"].split(".")[-1]
            if not any(isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name == name for node in ast.walk(tree)):
                raise ValueError(f"mapping target does not exist: {target}")
        for path in row["evidence"]:
            if not (ROOT / path).is_file():
                raise ValueError(f"mapping evidence does not exist: {path}")
    expected_protocols = {row["family"] for row in evidence["registrations"]["preparations"]}
    if set(mapping["protocols"]) != expected_protocols:
        raise ValueError("consumer mapping does not cover all inventoried execution protocols")
    # Check consumers introduced after the original snapshot too.
    for row in consumers():
        if row["category"] in {"test", "documentation"}:
            continue
        path = ROOT / row["file"]
        source_root = PACKAGE / "src/suspension_multibody"
        if path.is_relative_to(source_root) and path.relative_to(source_root).as_posix() in RETIREMENT_FILES:
            continue
        # Inventory capture deliberately describes the frozen, pre-cutover registry.
        if path == Path(__file__):
            continue
        old = execution_names(ast.parse(path.read_text(encoding="utf-8-sig"))) & EXECUTION_SYMBOLS
        if old:
            raise ValueError(f"current consumer executes legacy route: {path}: {sorted(old)}")


def cutover(inventory: Path) -> None:
    """Require a complete consumer mapping before any production retirement."""
    evidence = json.loads(inventory.read_text(encoding="utf-8"))
    if not evidence.get("snapshot_complete"):
        raise ValueError("migration inventory is incomplete")
    for filename, expected in evidence["frozen_fingerprints"].items():
        if digest((ROOT / filename).read_bytes()) != expected:
            raise ValueError(f"frozen file changed: {filename}")
    mapping_path = inventory.parent / "migration-consumers.json"
    mapping = json.loads(mapping_path.read_text(encoding="utf-8"))
    if mapping.get("inventory_sha256") != digest(inventory.read_bytes()):
        raise ValueError("consumer mapping was not generated from this inventory")
    validate_mapping(evidence, mapping)
    migration = ROOT / "packages/suspension_multibody/src/suspension_multibody/authoring/migration.py"
    tree = ast.parse(migration.read_text(encoding="utf-8"))
    forbidden = {"run_request", "run_compiled", "run_contract", "submit", "solve", "NativeContractBackend"}
    leaked = execution_names(tree) & forbidden
    if leaked:
        raise ValueError(f"offline migration imports or calls execution symbols: {leaked}")
    commands = [
        [sys.executable, str(EPIC / "planning/checks.py"), "numeric"],
        [sys.executable, str(EPIC / "planning/checks.py"), "structural"],
        ["uv", "run", "--no-sync", "pytest",
         "packages/suspension_multibody/tests/simulation/test_single_model_pipeline.py",
         "packages/suspension_multibody/tests/physics/test_wheel_spin_boundary.py",
         "packages/suspension_multibody/tests/architecture/test_unified_model_cutover.py",
         "packages/suspension_multibody/tests/simulation/test_document_route_reaches_every_registered_family.py",
         "-q", "-p", "no:cacheprovider"],
    ]
    checks = []
    for index, command in enumerate(commands):
        completed = subprocess.run(command, cwd=ROOT, capture_output=True, text=True,
            encoding="utf-8", errors="replace", env={**os.environ, "PYTHONUTF8": "1"}, check=False)
        log = inventory.parent / f"cutover-{index+1}.log"
        log.write_text(completed.stdout + completed.stderr, encoding="utf-8")
        print(completed.stdout, end="")
        print(completed.stderr, end="")
        checks.append({"command": command, "returncode": completed.returncode,
            "log": log.name, "log_sha256": digest(log.read_bytes())})
        report = {"inventory_sha256": digest(inventory.read_bytes()), "mapping_sha256": digest(mapping_path.read_bytes()),
            "consumer_count": len(mapping["consumers"]), "checks": checks, "passed": completed.returncode == 0 and len(checks) == len(commands)}
        (inventory.parent / "cutover-validation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        if completed.returncode:
            raise SystemExit(completed.returncode)
    print("PASS: all inventoried consumers and live cutover gates")


def main() -> None:
    """Run capture or the explicit pre-delete gate."""
    if isinstance(sys.stdout, io.TextIOWrapper):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if isinstance(sys.stderr, io.TextIOWrapper):
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser()
    parser.add_argument("--inventory", nargs="?", const="capture")
    parser.add_argument("--output", type=Path, default=EPIC / "raw/inventory.json")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--write-mapping", action="store_true")
    args = parser.parse_args()
    if args.write_mapping:
        if args.inventory in (None, "capture"):
            parser.error("--write-mapping requires --inventory PATH")
        write_mapping(Path(args.inventory))
    elif args.check:
        if args.inventory in (None, "capture"):
            parser.error("--check requires --inventory PATH")
        cutover(Path(args.inventory))
    elif args.inventory == "capture":
        capture(args.output)
    else:
        parser.error("use --inventory or --check --inventory PATH")


if __name__ == "__main__":
    main()
