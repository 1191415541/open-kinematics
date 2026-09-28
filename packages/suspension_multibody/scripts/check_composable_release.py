"""
The release probe: whether the product works once it has left this repository.

Every other gate in ``scripts/`` reads the source tree, which means every one of
them can pass while the package is unusable once installed.  This probe asks the
questions a tree-relative gate structurally cannot:

1. **the migration list is complete.**  The Python boundary gate runs in its
   ``--final`` mode and its findings are reported with their release condition.
   The retained findings are registered below; a *new* one fails, and so does a
   registered one that has quietly disappeared without the registry being
   updated.
2. **the documentation examples execute.**  The fenced ``python runnable`` blocks
   in ``docs/composable_extension_examples.md`` are extracted, written out and run
   in a fresh interpreter.  A doc block that no longer runs fails the release.
3. **the current-state documentation matches the tree.**  Every module directory
   this file claims must exist, and every retired one must be gone.
4. **the three packages build, install and run in isolation.**  Wheels are built
   into a scratch directory, installed into a scratch virtual environment, and one
   real native K solve runs **from a directory outside the repository** with no
   repository path on ``sys.path``.  That last part is the whole point: an
   editable install hides a missing file, and a run inside the tree hides a
   missing package.

What this probe deliberately does not do:

* it does not treat "the wheel file exists" as evidence.  A wheel is built, then
  installed, then *run*;
* it does not download anything.  The install is ``--offline`` and resolves
  against the local cache and the freshly built wheels; a dependency that is not
  already available is reported as a blocker rather than fetched;
* it does not re-record a numerical baseline.  The isolated run asserts
  convergence and a finite state, which is what "the product runs" means; the
  numerical gates are their own scripts.

    uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py
    uv run --no-sync python packages/suspension_multibody/scripts/check_composable_release.py --list
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

#: Repository root, from ``packages/suspension_multibody/scripts/<this file>``.
ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = ROOT / "packages" / "suspension_multibody"
SOURCE_ROOT = PACKAGE_ROOT / "src" / "suspension_multibody"

BOUNDARY_GATE = PACKAGE_ROOT / "tests" / "architecture" / "legacy_surface_gate.py"
EXAMPLES = PACKAGE_ROOT / "docs" / "composable_extension_examples.md"
NATIVE_BUILD = PACKAGE_ROOT / "scripts" / "build_axle_native.py"
ISOLATED_RUN = PACKAGE_ROOT / "tests" / "architecture" / "isolated_native_probe.py"

#: Fence used for a block that must execute.
FENCE = "```python runnable"

#: The Python boundary findings the final mode still reports, by path and symbol.
#:
#: Each entry is a *known* retention with a recorded release condition, not a
#: tolerated violation.  The probe fails on a finding that is not listed here,
#: and also on a listed finding that has disappeared, so the list can only shrink
#: -- the same discipline the migration-mode registry uses.
EXPECTED_BOUNDARY_FINDINGS: tuple[tuple[str, str], ...] = ()
#: The Python boundary findings the final mode still reports, by path and symbol.
#:
#: Empty: the last retention was the A1 ``elements/`` import in ``api.py``, and it
#: is gone now that the component-load reporting decodes the native
#: element-wrench channel instead of evaluating the element laws in Python.  The
#: tuple stays because the discipline it encodes does -- the probe fails on a
#: finding that is not listed here and on a listed finding that has disappeared,
#: so the list can only shrink and a new retention cannot be added silently.

#: The retired package directories the final mode must not find.
#:
#: ``analysis`` joined this list once its two constructions moved into
#: ``vehicle/``: the static wheel-load split and the roll-centre geometry are
#: vehicle-level derived quantities, not kernel solves, so neither ever needed
#: the static-solve ABI entry the old release condition asked for.
RETIRED_PACKAGES: tuple[str, ...] = ("core", "model", "metrics", "analysis")

#: The live modules the current-state documentation claims, and that must exist.
DOCUMENTED_PACKAGES: tuple[str, ...] = (
    "modeling",
    "templates",
    "connections",
    "rigs",
    "subsystems",
    "compilation",
    "simulation",
    "results",
    "outputs",
    "report",
    "studies",
    "preparation",
    "cases",
    "schema",
    "kernel",
    "vehicle",
)

#: The wheels the release consists of, in build order.
WHEEL_PACKAGES: tuple[str, ...] = (
    "suspension-contracts",
    "suspension-kernel",
    "suspension-multibody",
)


@dataclass
class Check:
    """One release check and what it found."""

    name: str
    ok: bool
    detail: str = ""

    def line(self) -> str:
        return f"[{'PASS' if self.ok else 'FAIL'}] {self.name}: {self.detail}"


class ProbeError(RuntimeError):
    """A release check could not be completed, which is itself a failure."""


# --------------------------------------------------------------------------- #
# 1. the migration list
# --------------------------------------------------------------------------- #


def run_boundary_gate(*, final: bool) -> str:
    """Run the Python boundary gate and return its stdout."""
    command = [sys.executable, str(BOUNDARY_GATE), "--check"]
    if final:
        command.append("--final")
    completed = subprocess.run(
        command, cwd=ROOT, capture_output=True, text=True, check=False
    )
    if not final and completed.returncode != 0:
        raise ProbeError(
            "the migration-mode boundary gate failed, so the migration list is not "
            f"exact:\n{completed.stdout}{completed.stderr}"
        )
    return completed.stdout


def parse_findings(text: str) -> list[tuple[str, str]]:
    """Return the ``(path, symbol)`` pairs a final-mode gate run reported."""
    findings: list[tuple[str, str]] = []
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped.startswith("- ") or "legacy_module_import" not in stripped:
            continue
        # `- src/.../api.py:42 [production/legacy_module_import/elements]`
        location, _, rest = stripped[2:].partition(" [")
        path, _, _line = location.partition(":")
        symbol = rest.rstrip("]").rsplit("/", 1)[-1]
        findings.append((normalise_source_path(path), symbol))
    return findings


def normalise_source_path(path: str) -> str:
    """
    Reduce a reported path to the part that identifies the module.

    The gate prints an absolute path, so the repository and package prefixes are
    stripped first; what is left is the part a registry entry is keyed by
    (``src/suspension_multibody/api.py`` or ``tests/.../test_elements.py``).
    """
    text = path.replace("\\", "/")
    for prefix in (str(PACKAGE_ROOT), str(ROOT)):
        text = text.replace(prefix.replace("\\", "/") + "/", "")
    return text


def is_production(relative: str) -> bool:
    """Return whether a reported path is production or script code."""
    return relative.startswith(("src/", "scripts/"))


def check_migration_list() -> Check:
    """Return whether the boundary findings are exactly the registered ones."""
    run_boundary_gate(final=False)
    text = run_boundary_gate(final=True)
    reported = parse_findings(text)
    expected = set(EXPECTED_BOUNDARY_FINDINGS)
    unexpected = [pair for pair in reported if pair not in expected]
    missing = sorted(expected - set(reported))
    if unexpected:
        detail = "; ".join(f"new unregistered finding {path}/{symbol}" for path, symbol in unexpected)
        return Check("migration list", False, detail)
    if missing:
        detail = "; ".join(
            f"{path}/{symbol} is registered but no longer reported -- remove it"
            for path, symbol in missing
        )
        return Check("migration list", False, detail)
    retired = [
        name for name in RETIRED_PACKAGES if (SOURCE_ROOT / name).exists()
    ]
    if retired:
        return Check(
            "migration list", False, f"retired package(s) still present: {retired}"
        )
    production = [pair for pair in reported if is_production(pair[0])]
    return Check(
        "migration list",
        True,
        f"{len(reported)} retained legacy imports "
        f"({len(production)} production, {len(reported) - len(production)} test) "
        f"and {len(RETIRED_PACKAGES)} retired packages gone",
    )


# --------------------------------------------------------------------------- #
# 2. the documentation examples
# --------------------------------------------------------------------------- #


def extract_examples(path: Path) -> list[tuple[str, str]]:
    """Return ``(label, source)`` for every ``python runnable`` block."""
    text = path.read_text(encoding="utf-8")
    blocks: list[tuple[str, str]] = []
    lines = text.splitlines()
    index = 0
    while index < len(lines):
        line = lines[index].strip()
        if line == FENCE:
            index += 1
            body: list[str] = []
            while index < len(lines) and lines[index].strip() != "```":
                body.append(lines[index])
                index += 1
            source = "\n".join(body) + "\n"
            match = re.search(r'E-\d+', source)
            blocks.append((match.group(0) if match else f"block-{len(blocks) + 1}", source))
        index += 1
    return blocks


def check_examples(*, scratch: Path) -> Check:
    """Every documented example runs in a fresh interpreter."""
    if not EXAMPLES.is_file():
        return Check("documentation examples", False, f"{EXAMPLES} is missing")
    blocks = extract_examples(EXAMPLES)
    if not blocks:
        return Check("documentation examples", False, f"{EXAMPLES} has no runnable block")
    failures: list[str] = []
    for label, source in blocks:
        path = scratch / f"example_{label}.py"
        path.write_text(source, encoding="utf-8")
        environment = dict(os.environ)
        # The example runs against the workspace source, which is what an author
        # has; the isolated install is checked separately, in check_isolation.
        environment["PYTHONPATH"] = str(SOURCE_ROOT.parent)
        completed = subprocess.run(
            [sys.executable, str(path)],
            cwd=ROOT,
            capture_output=True,
            text=True,
            env=environment,
            check=False,
        )
        if completed.returncode != 0:
            tail = (completed.stderr or completed.stdout).strip().splitlines()[-4:]
            failures.append(f"{label}: {' | '.join(tail)}")
    if failures:
        return Check("documentation examples", False, "; ".join(failures))
    return Check(
        "documentation examples",
        True,
        f"{len(blocks)} example(s) executed: {', '.join(label for label, _ in blocks)}",
    )


# --------------------------------------------------------------------------- #
# 3. the current-state documentation
# --------------------------------------------------------------------------- #


def documented_roots() -> set[str]:
    """Return the directories a documentation table says this package holds."""
    found: set[str] = set()
    for name in DOCUMENTED_PACKAGES:
        if (SOURCE_ROOT / name).is_dir():
            found.add(name)
    return found


def check_documentation_matches_tree() -> Check:
    """
    Check that the current-state docs describe the tree that is actually here.

    The check is structural on purpose: a file that *names* a module it no longer
    has is the documentation failure this catches, and prose cannot be read by a
    machine.  What the machine can hold is that each documented root exists and
    each retired one does not.
    """
    missing = [
        name for name in DOCUMENTED_PACKAGES if not (SOURCE_ROOT / name).is_dir()
    ]
    retired_present = [
        name for name in RETIRED_PACKAGES if (SOURCE_ROOT / name).is_dir()
    ]
    if missing:
        return Check("documentation roots", False, f"documented but absent: {missing}")
    if retired_present:
        return Check(
            "documentation roots",
            False,
            f"documented as retired but present: {retired_present}",
        )
    for relative in (
        "README.md",
        "CONTEXT.md",
        "CONTEXT-MAP.md",
        "docs/composable_extension_examples.md",
    ):
        root_doc = ROOT / relative
        package_doc = PACKAGE_ROOT / relative
        if not (root_doc if relative == "CONTEXT-MAP.md" else package_doc).is_file():
            return Check("documentation roots", False, f"{relative} is missing")
    return Check(
        "documentation roots",
        True,
        f"{len(DOCUMENTED_PACKAGES)} documented modules present, "
        f"{len(RETIRED_PACKAGES)} retired ones absent",
    )


# --------------------------------------------------------------------------- #
# 4. build, install and run in isolation
# --------------------------------------------------------------------------- #


def run_command(
    command: list[str], *, cwd: Path, capture: bool = True
) -> subprocess.CompletedProcess[str]:
    """Run one command, raising with its output when it fails."""
    completed = subprocess.run(
        command, cwd=str(cwd), capture_output=True, text=True, check=False
    )
    if completed.returncode != 0:
        output = "\n".join(
            part for part in (completed.stdout, completed.stderr) if part
        ).strip()
        if capture:
            raise ProbeError(f"{' '.join(command)} failed:\n{output[-2000:]}")
    return completed


def check_isolation(*, scratch: Path, rebuild_native: bool) -> Check:
    """
    Build the three wheels, install them in a scratch venv, and run natively.

    The order matters and is not incidental: the native library is built first,
    because the wheels embed the built mirror, and a wheel built before the
    library is a wheel carrying the previous one.
    """
    if rebuild_native:
        run_command(
            [sys.executable, str(NATIVE_BUILD)], cwd=ROOT
        )
    wheels = scratch / "wheels"
    wheels.mkdir(parents=True, exist_ok=True)
    for package in WHEEL_PACKAGES:
        run_command(
            ["uv", "build", "--package", package, "-o", str(wheels)], cwd=ROOT
        )
    built = sorted(wheels.glob("*.whl"))
    if len(built) != len(WHEEL_PACKAGES):
        raise ProbeError(
            f"expected {len(WHEEL_PACKAGES)} wheels, found "
            f"{[path.name for path in built]}"
        )
    for wheel in built:
        names = wheel_contents(wheel)
        if "native/suspension_kernel" not in " ".join(names):
            if wheel.name.startswith(("suspension_kernel", "suspension_multibody")):
                raise ProbeError(f"{wheel.name} does not ship the native library")

    venv = scratch / "venv"
    run_command(["uv", "venv", str(venv), "--python", "3.12"], cwd=scratch)
    python = venv / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    # `--offline` keeps this a release check rather than a fetch: a dependency
    # that is not already cached is a blocker to report, not something to pull.
    run_command(
        [
            "uv",
            "pip",
            "install",
            "--python",
            str(python),
            "--offline",
            "--find-links",
            str(wheels),
            *WHEEL_PACKAGES,
        ],
        cwd=scratch,
    )

    outside = scratch / "outside"
    outside.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ISOLATED_RUN, outside / ISOLATED_RUN.name)
    environment = dict(os.environ)
    # The isolation is the point: no repository path, and the probe's own
    # variables gone, so an accidental reliance on the source tree shows up.
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    completed = subprocess.run(
        [str(python), ISOLATED_RUN.name],
        cwd=str(outside),
        capture_output=True,
        text=True,
        env=environment,
        check=False,
    )
    if completed.returncode != 0:
        tail = (completed.stderr or completed.stdout).strip().splitlines()[-6:]
        return Check(
            "isolated install",
            False,
            f"the out-of-tree native run failed: {' | '.join(tail)}",
        )
    payload = json.loads(completed.stdout)
    if payload.get("repo_on_path"):
        return Check(
            "isolated install",
            False,
            f"the repository was on sys.path: {payload['repo_on_path']}",
        )
    if not payload.get("converged") or not payload.get("finite"):
        return Check(
            "isolated install",
            False,
            f"the out-of-tree run did not converge finitely: {payload}",
        )
    return Check(
        "isolated install",
        True,
        f"{len(built)} wheels built, installed offline, and one native K state "
        f"converged outside the tree (residual {payload['residual']:.3e})",
    )


def wheel_contents(wheel: Path) -> list[str]:
    """Return the member names of a wheel."""
    import zipfile

    with zipfile.ZipFile(wheel) as archive:
        return archive.namelist()


# --------------------------------------------------------------------------- #
# entry point
# --------------------------------------------------------------------------- #


def main(argv: list[str] | None = None) -> int:
    """Run the release probe and return its exit status."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--list",
        action="store_true",
        help="list the checks that would run, without running them",
    )
    parser.add_argument(
        "--skip-isolation",
        action="store_true",
        help="run the tree checks only (the isolated build is the slow half)",
    )
    parser.add_argument(
        "--skip-native-rebuild",
        action="store_true",
        help="reuse the current native library instead of rebuilding it",
    )
    parser.add_argument(
        "--scratch",
        default=None,
        help="directory for wheels, the venv and the extracted examples",
    )
    args = parser.parse_args(argv)

    checks = [
        "migration list -- the boundary findings are the registered ones",
        "documentation examples -- every `python runnable` block executes",
        "documentation roots -- the current-state docs match the tree",
        "isolated install -- three wheels build, install offline and solve natively",
    ]
    if args.list:
        for check in checks:
            print(f"  - {check}")
        return 0

    results: list[Check] = []
    owned = args.scratch is None
    scratch = Path(
        args.scratch
        if args.scratch
        else tempfile.mkdtemp(prefix="suspension-release-")
    )
    scratch.mkdir(parents=True, exist_ok=True)
    print(f"release probe: {ROOT}")
    print(f"scratch      : {scratch}\n")
    try:
        for check in (
            check_migration_list,
            check_documentation_matches_tree,
        ):
            try:
                results.append(check())
            except ProbeError as error:
                results.append(Check(check.__name__, False, str(error)))
        try:
            results.append(check_examples(scratch=scratch))
        except ProbeError as error:
            results.append(Check("documentation examples", False, str(error)))
        if not args.skip_isolation:
            try:
                results.append(
                    check_isolation(
                        scratch=scratch, rebuild_native=not args.skip_native_rebuild
                    )
                )
            except ProbeError as error:
                results.append(Check("isolated install", False, str(error)))
    finally:
        if owned and not any(not result.ok for result in results):
            shutil.rmtree(scratch, ignore_errors=True)

    for result in results:
        print(result.line())
    failed = [result for result in results if not result.ok]
    if failed:
        print(f"\nFAIL: {len(failed)} of {len(results)} release checks failed")
        return 1
    print(f"\nOK: {len(results)} release checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
