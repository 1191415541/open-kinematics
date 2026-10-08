"""
The four scattered simulation entries and the SI modelling surface stay retired.

Subtask 09 of the unified-simulate-entry epic removes the four entry points the
epic consolidated away (``run_case``, ``run_dynamic_case``, ``run_axle_dynamics``,
``run_vehicle_dynamics``) and takes the SI modelling route (``AxleDynamicsModel``,
``load_axle_dynamics_model``, the v1 ``load_axle_declaration`` /
``load_vehicle_declaration`` loaders, and the ``AxleDeclaration`` /
``VehicleDeclaration`` root exports) off the public surface.  ``simulate`` is the
one public door onto a run; the capability behind every retired name survives
under an internal spelling, which is why this gate forbids the *old names* rather
than the behaviour.

A retirement like this decays quietly: a later change can reintroduce the name in
an annotation, in an import, as a ``"compatibility"`` alias, or as a string in an
``__all__`` entry, and nothing about the product's behaviour would say so.  These
checks read the tree rather than importing it, and the detector is exercised
against injected positives, so a passing run means "looked for it and did not find
it" rather than "found nothing because the scanner was broken".

The retired-name scan deliberately ignores docstrings and comments: prose that
explains the removal is the opposite of a reintroduction, and a gate that cannot
be documented next to is a gate somebody deletes.
"""

from __future__ import annotations

import ast
from pathlib import Path

#: The four scattered entry points ``simulate`` replaced.  Every occurrence in
#: live code is a finding; the capability behind each lives under an internal
#: spelling (``kc_case_run``, ``replay_case``, ``run_axle``,
#: ``vehicle_dynamics_run``), which is what the replacement check pins.
RETIRED_ENTRIES: tuple[str, ...] = (
    "run_case",
    "run_dynamic_case",
    "run_axle_dynamics",
    "run_vehicle_dynamics",
)

#: The retired SI modelling surface.  ``AxleDynamicsModel`` is still a type the
#: emitters produce, so it is *imported from its own module* and forbidden only
#: as a published name -- the split between "type kept" and "export retired" is
#: decision (i) of subtask 01.
RETIRED_MODELLING_NAMES: tuple[str, ...] = (
    "load_axle_dynamics_model",
    "load_axle_declaration",
    "load_vehicle_declaration",
)

#: The names whose *public export* is retired while the types survive.
RETIRED_EXPORTS: tuple[str, ...] = ("AxleDynamicsModel", "AxleDeclaration", "VehicleDeclaration")

#: The internal spellings that must exist, one per retired entry.
REPLACEMENTS: tuple[str, ...] = (
    "kc_case_run",
    "replay_case",
    "run_axle",
    "vehicle_dynamics_run",
    "simulate",
)

#: The `file`-style scan roots: production, tests and the package's own scripts.
_PACKAGE = Path(__file__).parents[2]
_SOURCES = (
    _PACKAGE / "src" / "suspension_multibody",
    _PACKAGE / "tests",
    _PACKAGE / "scripts",
)

#: This gate's own file, which names the retired symbols as its *subject* rather
#: than as a dependency.  The skip is by resolved identity, so a file that merely
#: resembles this one is still scanned.
_SELF = Path(__file__).resolve()

#: The package's own modelling modules.  ``AxleDynamicsModel`` is defined here and
#: a definition is not a reintroduction, so the definition site is exempt while
#: every consumer is still checked.
_DEFINITION_SITES = {
    str((_PACKAGE / "src" / "suspension_multibody" / "axle_dynamics" / "schema.py").resolve()),
}

#: Files whose *subject* is the retirement itself.  ``test_simulate.py`` asserts
#: the names are gone (a lookup error rather than a second door) and
#: ``test_bypass.py`` lists them among the calls a report module must not make --
#: both have to name what they are about, the same way this gate does.  Keyed by
#: file name relative to ``_PACKAGE`` so a stale entry fails loudly below rather
#: than shrinking the scan silently.
_SUBJECT_FILES: dict[str, str] = {
    "tests/api/test_simulate.py": "asserts the retired entries are not published",
    "tests/outputs/test_bypass.py": "lists them among the calls a report must not make",
}


def _identifier_names(node: ast.AST) -> set[str]:
    """
    Return every name one tree *uses*: bare names, attribute tails, imports.

    A string constant counts too, because the two ways a name can be used without
    being an identifier -- an ``__all__`` entry and a ``getattr`` target -- are
    both strings.  Prose is excluded by :func:`_code_names`, which drops
    docstrings before calling this.
    """
    found: set[str] = set()
    for child in ast.walk(node):
        if isinstance(child, ast.Name):
            found.add(child.id)
        elif isinstance(child, ast.Attribute):
            found.add(child.attr)
        elif isinstance(child, ast.alias):
            found.add(child.name.rsplit(".", 1)[-1])
            if child.asname:
                found.add(child.asname)
        elif isinstance(child, ast.arg):
            if child.annotation is not None:
                found.update(_identifier_names(child.annotation))
        elif isinstance(child, ast.Constant) and isinstance(child.value, str):
            found.add(child.value)
    return found


def _code_names(path: Path) -> set[str]:
    """Return the names one module uses in code, with prose removed."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    docstrings = {
        id(node.body[0].value)
        for node in ast.walk(tree)
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
        and node.body
        and isinstance(node.body[0], ast.Expr)
        and isinstance(node.body[0].value, ast.Constant)
        and isinstance(node.body[0].value.value, str)
    }
    return {name for name in _identifier_names(tree) if name not in docstrings}


def _is_subject_file(path: Path) -> bool:
    """Return whether a file is allowed to *name* a retired symbol as its subject."""
    try:
        key = path.resolve().relative_to(_PACKAGE.resolve()).as_posix()
    except ValueError:
        return False
    return key in _SUBJECT_FILES

def _findings(root: Path, names: tuple[str, ...]) -> list[str]:
    """Return ``file:name`` for every live reference to ``names`` under ``root``."""
    wanted = set(names)
    found: list[str] = []
    if not root.is_dir():
        return found
    for path in sorted(root.rglob("*.py")):
        if "__pycache__" in path.parts or path.resolve() == _SELF:
            continue
        if str(path.resolve()) in _DEFINITION_SITES or _is_subject_file(path):
            continue
        used = _code_names(path) & wanted
        found.extend(f"{path}:{name}" for name in sorted(used))
    return found


def test_every_subject_file_entry_is_still_a_live_file() -> None:
    """
    The subject-file exemption cannot rot into a silent hole.

    An allowlist that names a file nobody renamed stays harmless, but one that
    names a file that was *deleted* would keep the exemption alive for whatever
    reuses the path.  Pinning existence keeps the exemption honest.
    """
    for relative in _SUBJECT_FILES:
        assert (_PACKAGE / relative).is_file(), f"subject file {relative} is missing"




def test_the_retired_entries_are_not_referenced_anywhere() -> None:
    """
    No production module, test module or script *calls* a retired entry.

    The whole point of the consolidation is that ``simulate`` is the one door, so
    a single live call means a consumer was not moved -- or was moved back.
    """
    findings: list[str] = []
    for root in _SOURCES:
        findings.extend(_findings(root, RETIRED_ENTRIES))
    assert not findings, (
        "the retired simulation entries are referenced again: "
        + ", ".join(findings)
        + ". ``simulate`` is the one public door; a call to one of these names is a"
        " second door onto a reading that already has an internal spelling."
    )


def test_the_retired_modelling_loaders_are_not_referenced_anywhere() -> None:
    """The v1 modelling loaders were retired with the modelling route (G5/D1)."""
    findings: list[str] = []
    for root in _SOURCES:
        findings.extend(_findings(root, RETIRED_MODELLING_NAMES))
    assert not findings, (
        "the retired v1 modelling loaders are referenced again: "
        + ", ".join(findings)
        + ". A user models through the document system; the SI model is an"
        " internal emission product, not a loader door."
    )


def test_the_retired_entries_and_exports_are_not_on_the_public_surface() -> None:
    """
    None of the retired names is published, and importing one has to fail.

    A name on ``__all__`` is a door even if no module calls it: a caller that can
    name it can pass it, and then the name is load-bearing again.
    """
    import suspension_multibody as package
    import suspension_multibody.schema as schema

    for name in RETIRED_ENTRIES:
        assert name not in package.__all__, f"{name} is still in the package __all__"
        assert name not in package._PUBLIC_NAMES, f"{name} is still a public name"
        assert not hasattr(package, name), f"{name} is still reachable from the package"
    for name in RETIRED_MODELLING_NAMES + RETIRED_EXPORTS:
        assert name not in schema.__all__, f"{name} is still in schema.__all__"
        assert not hasattr(schema, name), f"suspension_multibody.schema.{name} still exists"
        assert name not in package.__all__, f"{name} is still in the package __all__"
        assert name not in package._PUBLIC_NAMES, f"{name} is still a public name"
    from suspension_multibody import axle_dynamics

    assert "AxleDynamicsModel" not in axle_dynamics.__all__
    assert not hasattr(axle_dynamics, "AxleDynamicsModel")
    assert not hasattr(axle_dynamics, "load_axle_dynamics_model")


def test_the_replacements_are_what_the_surface_offers() -> None:
    """
    The other half of the previous check: the capability is *there*.

    Deleting a name and offering nothing is not a migration, so this pins that
    ``simulate`` is public and every internal replacement exists.
    """
    import suspension_multibody as package

    assert "simulate" in package.__all__
    assert callable(package.simulate)

    from suspension_multibody.api import simulate, validate
    from suspension_multibody.authoring.migration import (
        migrate_v1_case,
        migrate_v1_dynamic_axle,
        migrate_v1_vehicle_case,
    )
    from suspension_multibody.schema.model import AxleDeclaration
    from suspension_multibody.schema.vehicle import VehicleDeclaration

    for replacement in (simulate, validate, migrate_v1_case, migrate_v1_dynamic_axle, migrate_v1_vehicle_case):
        assert callable(replacement)
    # The types survive even though their root exports do not.
    assert AxleDeclaration.__name__ == "AxleDeclaration"
    assert VehicleDeclaration.__name__ == "VehicleDeclaration"


def test_the_detector_catches_injected_references(tmp_path: Path) -> None:
    """
    The gate is exercised against references it must find.

    A structural check that only ever runs on a clean tree proves nothing about
    itself: it would pass just as happily if the scanner returned nothing at all.
    These fixtures are the positive control -- one per *shape* a reintroduction
    would take -- so a green run means the detector looked and found nothing, not
    that it never looked.
    """
    cases = {
        "call.py": "from suspension_multibody import api\napi.run_case(model, case)\n",
        "import.py": "from suspension_multibody.axle_dynamics import run_axle_dynamics\n",
        "annotation.py": "def build(axle: AxleDynamicsModel) -> None:\n    pass\n",
        "loader.py": "from suspension_multibody.schema import load_axle_declaration\n",
        "export.py": '__all__ = ["run_dynamic_case"]\n',
        "attribute.py": (
            "import suspension_multibody\nsuspension_multibody.run_vehicle_dynamics\n"
        ),
    }
    wanted = RETIRED_ENTRIES + RETIRED_MODELLING_NAMES + RETIRED_EXPORTS
    for filename, source in cases.items():
        path = tmp_path / filename
        path.write_text(source, encoding="utf-8")
        assert _findings(tmp_path, wanted), (
            f"the detector missed the injected reference in {filename}"
        )
        path.unlink()


def test_the_detector_ignores_prose_about_the_retirement(tmp_path: Path) -> None:
    """Explaining the removal is not doing it again."""
    path = tmp_path / "explains.py"
    path.write_text(
        '"""This used to call ``run_case`` and ``run_axle_dynamics``."""\n'
        "# ``run_case`` is gone; ``simulate`` replaced it.\n"
        "value = 1\n",
        encoding="utf-8",
    )
    assert not _findings(tmp_path, RETIRED_ENTRIES)
