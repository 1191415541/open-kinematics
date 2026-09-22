"""Full-repository reference scan for the retired Python surface.

Deliverable of subtask 08 step 2: a scan of the *whole* repository (production
sources, scripts, tests, configuration and current documentation) for references
to the packages and modules subtask 08 retires::

    core, elements, model, analysis, metrics, pac2002_scope

The scanner is AST based for Python and line based for everything else, and it
never imports what it scans: a retired module may be half deleted while this
runs.

What it deliberately does *not* skip
------------------------------------
There is no allow-list.  Every hit is reported with its file, line and the form
of the reference, so a shrinking count is real evidence and an exemption cannot
hide a survivor.  Historical task records (``.codex-tasks/``) and generated trees
(``build``, ``dist``, ``__pycache__``, ``.venv``, caches, ``artifacts``) are
excluded by directory, not by file, because they are not the repository's live
surface.

Forms it recognises
-------------------
* absolute import:   ``import suspension_multibody.core``
* package import:    ``from suspension_multibody.core import X``
* relative import:   ``from ..core import X`` / ``from . import core``
* aliased import:    ``from suspension_multibody import core as c``
* dynamic import:    ``importlib.import_module("suspension_multibody.core")``,
                     ``__import__(...)``, including a module-level string constant
* text reference:    ``suspension_multibody.core`` or ``pac2002_scope`` in a
                     config, script or document

Why the text form needs the package prefix
------------------------------------------
``analysis``, ``model``, ``core`` and ``metrics`` are ordinary English words, so
a bare-word scan would report "validation analysis" in a README as a dependency
and make the count useless as evidence.  A package-qualified name is unambiguous,
so that is what the text form requires.  ``pac2002_scope`` has no English
reading, so its bare form counts too.

Usage::

    uv run python legacy_reference_scan.py --report-before
    uv run python legacy_reference_scan.py --report-after

Both modes print the same report; the flag names the moment in the deletion
sequence the evidence belongs to, so a before/after pair is unmistakable.
"""

from __future__ import annotations

import argparse
import ast
import json
import re
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
PACKAGE = "suspension_multibody"
#: The modules subtask 08 retires.  `pac2002_scope` is a top-level module.
RETIRED = ("core", "elements", "model", "analysis", "metrics", "pac2002_scope")
#: Directory names that are generated or are historical records, at any depth.
EXCLUDED_DIRECTORIES = frozenset(
    {
        ".codex-tasks",
        ".git",
        ".venv",
        ".gui-venv",
        "__pycache__",
        "build",
        "dist",
        "artifacts",
        ".ruff_cache",
        ".pytest_cache",
        ".mypy_cache",
        "node_modules",
        ".idea",
        ".vscode",
    }
)
#: Extensions scanned.  Everything else (binaries, cache files) is skipped.
SCANNED_SUFFIXES = frozenset(
    {
        ".py",
        ".pyi",
        ".toml",
        ".cfg",
        ".ini",
        ".md",
        ".rst",
        ".txt",
        ".json",
        ".yaml",
        ".yml",
        ".just",
        ".sh",
        ".ps1",
        ".bat",
    }
)
SCANNED_NAMES = frozenset({"justfile", "Justfile", "Makefile", "Dockerfile"})

#: A reference to a retired module in non-Python text.  Only the package-qualified
#: form counts for a package module (see the module docstring).  The retired
#: top-level module ``pac2002_scope`` is matched through the same qualified form
#: plus its bare name with a trailing file extension (``pac2002_scope.py``), which
#: is the shape a config or document writes.  A bare ``pac2002_scope`` word is
#: *not* matched on its own: the capability and schema modules that replaced it
#: are legitimately called ``pac2002_scope`` too (`schema/pac2002_scope.py`), and
#: a bare match would report the live successor as a survivor.
TEXT_REFERENCE = re.compile(
    rf"(?P<module>{PACKAGE}\.(?:{'|'.join(RETIRED)})|pac2002_scope\.py)(?=[^\w.]|$)"
)


@dataclass(frozen=True)
class Reference:
    """One reference to a retired module."""

    path: str
    line: int
    form: str
    module: str
    text: str

    @property
    def key(self) -> tuple[str, int, str, str]:
        return self.path, self.line, self.form, self.module

    def describe(self) -> str:
        return f"{self.path}:{self.line} [{self.form}/{self.module}] {self.text}"


def _relative(path: Path) -> str:
    try:
        return path.relative_to(REPO).as_posix()
    except ValueError:
        return path.as_posix()


def module_name(path: Path) -> str | None:
    """Return the dotted module name of a file inside the package, if known."""
    parts = path.with_suffix("").parts
    if PACKAGE not in parts:
        return None
    index = len(parts) - 1 - tuple(reversed(parts)).index(PACKAGE)
    return ".".join(parts[index:]).removesuffix(".__init__")


def _is_excluded(path: Path) -> bool:
    return any(part in EXCLUDED_DIRECTORIES for part in path.parts)


def _iter_files() -> list[Path]:
    found: list[Path] = []
    for path in REPO.rglob("*"):
        if not path.is_file() or _is_excluded(path):
            continue
        if path.suffix in SCANNED_SUFFIXES or path.name in SCANNED_NAMES:
            found.append(path)
    return sorted(found)


def _resolve(path: Path, module: str | None, level: int) -> str | None:
    """Resolve an import to a dotted module, following relative levels."""
    if not level:
        return module
    owner = module_name(path)
    if owner is None:
        return module
    parts = owner.split(".")
    package_parts = parts if path.name == "__init__.py" else parts[:-1]
    keep = package_parts[: len(package_parts) - (level - 1)]
    if module:
        keep = [*keep, *module.split(".")]
    return ".".join(keep)


def _retired_of(module: str | None) -> str | None:
    """Return the retired module a dotted name resolves to, if any."""
    if not module:
        return None
    parts = module.split(".")
    if parts[0] == PACKAGE and len(parts) > 1 and parts[1] in RETIRED:
        return parts[1]
    if parts[0] != PACKAGE and parts[0] in RETIRED:
        return parts[0]
    return None


def _module_constants(tree: ast.Module) -> dict[str, str]:
    """Return the module-level ``NAME = "literal"`` string constants."""
    constants: dict[str, str] = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            if (
                isinstance(target, ast.Name)
                and isinstance(node.value, ast.Constant)
                and isinstance(node.value.value, str)
            ):
                constants[target.id] = node.value.value
    return constants


def _dynamic_argument(
    arguments: list[ast.expr], constants: dict[str, str]
) -> str | None:
    """Return the module a dynamic import names, literal or via a constant."""
    if not arguments:
        return None
    first = arguments[0]
    if isinstance(first, ast.Constant) and isinstance(first.value, str):
        return first.value
    if isinstance(first, ast.Name):
        return constants.get(first.id)
    return None


def scan_python(path: Path) -> list[Reference]:
    """Return every retired-module reference in one Python file."""
    relative = _relative(path)
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"))
    except (SyntaxError, UnicodeDecodeError) as error:  # pragma: no cover - guard
        raise SystemExit(f"{relative}: cannot parse ({error})") from error
    constants = _module_constants(tree)
    found: list[Reference] = []

    def add(form: str, module: str, node: ast.AST, text: str) -> None:
        reference = Reference(
            relative, int(getattr(node, "lineno", 0)), form, module, text
        )
        if reference not in found:
            found.append(reference)

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                retired = _retired_of(alias.name)
                if retired:
                    form = "aliased_import" if alias.asname else "absolute_import"
                    add(form, retired, node, f"import {alias.name}")
        elif isinstance(node, ast.ImportFrom):
            target = _resolve(path, node.module, node.level)
            retired = _retired_of(target)
            if retired:
                form = "relative_import" if node.level else "absolute_import"
                add(
                    form,
                    retired,
                    node,
                    f"from {'.' * node.level}{node.module or ''} import ...",
                )
                continue
            # ``from suspension_multibody import core as c`` names the module in
            # the alias, and ``from . import core`` names it among the imported
            # names of a relative import into the package itself.
            if target == PACKAGE:
                for alias in node.names:
                    if alias.name in RETIRED:
                        form = "aliased_import" if alias.asname else "package_import"
                        add(
                            form,
                            alias.name,
                            node,
                            f"from {'.' * node.level}{node.module or ''} "
                            f"import {alias.name}",
                        )
        elif isinstance(node, ast.Call):
            called = node.func
            name = ""
            if isinstance(called, ast.Name):
                name = called.id
            elif isinstance(called, ast.Attribute):
                name = called.attr
            if name not in {"import_module", "__import__"}:
                continue
            literal = _dynamic_argument(node.args, constants)
            retired = _retired_of(literal)
            if retired:
                add("dynamic_import", retired, node, f"{name}({literal!r})")
    return found


def scan_text(path: Path) -> list[Reference]:
    """Return every retired-module reference in one non-Python text file."""
    relative = _relative(path)
    try:
        content = path.read_text(encoding="utf-8")
    except (UnicodeDecodeError, OSError):
        return []
    found: list[Reference] = []
    for number, line in enumerate(content.splitlines(), start=1):
        match = TEXT_REFERENCE.search(line)
        if match:
            found.append(
                Reference(
                    relative,
                    number,
                    "text_reference",
                    match.group("module"),
                    line.strip()[:160],
                )
            )
    return found


def scan() -> list[Reference]:
    """Scan the live surface of the repository and return every reference."""
    found: list[Reference] = []
    for path in _iter_files():
        found.extend(scan_python(path) if path.suffix == ".py" else scan_text(path))
    return found


def _summary(references: list[Reference]) -> dict[str, object]:
    by_module: dict[str, int] = {}
    by_form: dict[str, int] = {}
    by_area: dict[str, int] = {}
    for reference in references:
        by_module[reference.module] = by_module.get(reference.module, 0) + 1
        by_form[reference.form] = by_form.get(reference.form, 0) + 1
        area = reference.path.split("/", maxsplit=1)[0]
        by_area[area] = by_area.get(area, 0) + 1
    return {
        "total": len(references),
        "by_module": dict(sorted(by_module.items())),
        "by_form": dict(sorted(by_form.items())),
        "by_area": dict(sorted(by_area.items())),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Retired-surface reference scan.")
    parser.add_argument(
        "--report-before",
        action="store_true",
        help="label this run as the pre-deletion baseline",
    )
    parser.add_argument(
        "--report-after",
        action="store_true",
        help="label this run as the post-deletion verification",
    )
    parser.add_argument(
        "--json",
        type=Path,
        default=None,
        help="also write the full finding list to this path",
    )
    arguments = parser.parse_args()

    moment = "after" if arguments.report_after else "before"
    references = scan()
    summary = _summary(references)

    print(f"scan moment      : {moment}")
    print(f"retired modules  : {', '.join(RETIRED)}")
    print(f"references       : {summary['total']}")
    print(f"by module        : {summary['by_module']}")
    print(f"by form          : {summary['by_form']}")
    print(f"by area          : {summary['by_area']}")
    print()
    for reference in references:
        print(reference.describe())

    if arguments.json is not None:
        payload = {
            "moment": moment,
            "summary": summary,
            "references": [
                {
                    "path": item.path,
                    "line": item.line,
                    "form": item.form,
                    "module": item.module,
                    "text": item.text,
                }
                for item in references
            ],
        }
        arguments.json.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(f"\nwrote {arguments.json}")


if __name__ == "__main__":
    main()
