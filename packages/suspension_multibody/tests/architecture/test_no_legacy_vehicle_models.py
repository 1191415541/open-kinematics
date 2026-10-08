"""
The retired axle and vehicle model classes are gone, and stay gone.

``FrontAxleModel`` and ``VehicleModel`` were the two classes the composition
used to name directly: an assembly input had to *be* one of them.  The
composition now reads its inputs through ``AxleInput``/``VehicleInput``, and the
v1 file format is read into ``AxleDeclaration``/``VehicleDeclaration`` -- so the
two classes are not the interface between the document layer and the assembly
layer any more, and every consumer was migrated before they were deleted.

A removal like this decays quietly: a later change can reintroduce the name in
an annotation, in an import, or as a "compatibility" alias, and nothing about
the product's behaviour would say so.  These checks are the structural evidence
instead, and they read the tree rather than importing it, so a name that only
appears in an unimported module is still caught.

The checks are AST based, and the detector is exercised against injected
positive examples in :func:`test_the_detector_catches_injected_references`, so a
passing run means "looked for it and did not find it" rather than "found
nothing because the scanner was broken".
"""

from __future__ import annotations

import ast
from pathlib import Path

#: The two retired class names.  Every occurrence in live code is a finding: the
#: composition's protocols are what replaced them, and a module that names
#: either class again has put the dependency back.
RETIRED_NAMES = ("FrontAxleModel", "VehicleModel")

#: The retired loader names.  The format survived; the *names* did not, because
#: what comes back is a declaration rather than a model.
RETIRED_LOADERS = ("load_model", "load_vehicle_model")

#: The retired bridge names on the document path.
RETIRED_BRIDGES = ("front_axle_model_from", "front_axle_model_for")

#: Every name this gate forbids in live code.
_RETIRED = RETIRED_NAMES + RETIRED_LOADERS + RETIRED_BRIDGES

_PACKAGE = Path(__file__).parents[2]
_SOURCES = (
    _PACKAGE / "src" / "suspension_multibody",
    _PACKAGE / "tests",
    _PACKAGE / "scripts",
    _PACKAGE.parents[1] / "scripts",
)

#: This gate's own file, which names the retired symbols as its *subject* rather
#: than as a dependency.  The skip is by resolved identity, so a file that merely
#: resembles this one is still scanned.
_SELF = Path(__file__).resolve()


def _identifier_names(node: ast.AST) -> set[str]:
    """
    Return every name one tree *uses*: bare names, attribute tails, imports.

    A string constant counts too, because the two ways a name can be used
    without being an identifier -- an ``__all__`` entry and a ``getattr`` target
    -- are both strings.  Prose is excluded by :func:`_code_names`, which drops
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
    """
    Return the names one module uses in code, with prose removed.

    Docstrings and comments are how the retirement is explained, so they must
    not count: a comment saying "this used to be ``FrontAxleModel``" is the
    opposite of a reintroduction.  Every docstring node is therefore dropped
    before the names are collected, and comments never reach the AST at all.
    """
    tree = ast.parse(path.read_text(encoding="utf-8"))
    docstrings = {
        id(node.body[0].value)
        for node in ast.walk(tree)
        if isinstance(
            node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)
        )
        and node.body
        and isinstance(node.body[0], ast.Expr)
        and isinstance(node.body[0].value, ast.Constant)
        and isinstance(node.body[0].value.value, str)
    }
    return {name for name in _identifier_names(tree) if name not in docstrings}


def _findings(root: Path, names: tuple[str, ...]) -> list[str]:
    """Return ``file:name`` for every live reference to ``names`` under ``root``."""
    wanted = set(names)
    found: list[str] = []
    if not root.is_dir():
        return found
    for path in sorted(root.rglob("*.py")):
        if "__pycache__" in path.parts or path.resolve() == _SELF:
            continue
        used = _code_names(path) & wanted
        found.extend(f"{path}:{name}" for name in sorted(used))
    return found


def test_the_retired_models_are_not_referenced_anywhere() -> None:
    """
    No production module, test module or script names a retired symbol.

    The whole point of the migration is that nothing needs them, so a single
    live reference means some consumer was not moved -- or was moved back.
    """
    findings: list[str] = []
    for root in _SOURCES:
        findings.extend(_findings(root, _RETIRED))
    assert not findings, (
        "retired axle/vehicle model names are referenced again: "
        + ", ".join(findings)
        + ". The composition reads its inputs through AxleInput/VehicleInput and"
        " the v1 format is read into AxleDeclaration/VehicleDeclaration, so a"
        " reference to either old name puts a dependency back on the interface"
        " this work removed."
    )


def test_the_retired_names_are_not_exported() -> None:
    """
    The public surface does not offer them, under any module.

    Importing has to fail, because a caller that can still import the class can
    still pass one -- and then the class is load-bearing again without any
    module naming it in code.
    """
    import suspension_multibody as package
    import suspension_multibody.schema as schema

    for name in RETIRED_NAMES + RETIRED_LOADERS:
        assert not hasattr(schema, name), f"suspension_multibody.schema.{name} still exists"
        assert name not in schema.__all__, f"{name} is still in schema.__all__"
        assert name not in package.__all__, f"{name} is still in the package __all__"
        assert name not in package._PUBLIC_NAMES, f"{name} is still a public name"


def test_the_replacement_types_are_what_the_public_surface_offers() -> None:
    """
    The other half of the previous check: the replacements are *there*.

    Deleting a name and offering nothing is not a migration, so this pins that
    the declared replacement is reachable.  The two declaration types are now
    reached from their own modules rather than from the package root: subtask 09
    retired the root export alongside the four scattered entry points, because a
    name on ``__all__`` is a second door onto the modelling surface.  The types
    themselves are not deleted -- composition reads them internally -- so the
    check moves with the export instead of disappearing.
    """
    import suspension_multibody as package
    from suspension_multibody.schema.model import AxleDeclaration
    from suspension_multibody.schema.vehicle import VehicleDeclaration

    assert AxleDeclaration.__name__ == "AxleDeclaration"
    assert VehicleDeclaration.__name__ == "VehicleDeclaration"
    assert "AxleDeclaration" not in package.__all__
    assert "VehicleDeclaration" not in package.__all__
    assert "AxleDeclaration" not in package._PUBLIC_NAMES
    assert "VehicleDeclaration" not in package._PUBLIC_NAMES


def test_the_detector_catches_injected_references(tmp_path: Path) -> None:
    """
    The gate is exercised against references it must find.

    A structural check that only ever runs on a clean tree proves nothing about
    itself: it would pass just as happily if the scanner returned nothing at
    all.  These fixtures are the positive control -- one per *shape* a
    reintroduction would take -- so a green run means the detector looked and
    found nothing, not that it never looked.
    """
    cases = {
        "import.py": "from suspension_multibody.schema import FrontAxleModel\n",
        "annotation.py": "def build(axle: FrontAxleModel) -> None:\n    pass\n",
        "constructor.py": "model = VehicleModel(chassis=None)\n",
        "loader.py": "from suspension_multibody import load_vehicle_model\n",
        "bridge.py": (
            "from suspension_multibody.authoring.solver import front_axle_model_for\n"
        ),
        "attribute.py": (
            "import suspension_multibody\nsuspension_multibody.schema.FrontAxleModel\n"
        ),
        "export.py": '__all__ = ["VehicleModel"]\n',
    }
    for filename, source in cases.items():
        path = tmp_path / filename
        path.write_text(source, encoding="utf-8")
        assert _findings(tmp_path, _RETIRED), (
            f"the detector missed the injected reference in {filename}"
        )
        path.unlink()


def test_the_detector_ignores_prose_about_the_retirement(tmp_path: Path) -> None:
    """
    The detector's other control: explaining the removal is not doing it again.

    Without this, the gate would forbid the very comments that tell a reader why
    the class is gone -- and a check that cannot be documented next to is a
    check somebody deletes.
    """
    (tmp_path / "prose.py").write_text(
        '"""This module used to hold ``FrontAxleModel``; it holds declarations now."""\n'
        "\n"
        "\n"
        "def build() -> None:\n"
        '    """``VehicleModel`` was the old shape; see the declaration instead."""\n'
        "    # load_model used to live here.\n",
        encoding="utf-8",
    )
    assert not _findings(tmp_path, _RETIRED)
