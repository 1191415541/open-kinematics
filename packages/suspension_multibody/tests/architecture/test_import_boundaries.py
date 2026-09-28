"""
The low modelling layer does not drag the authoring chain in with it.

This is the property subtask 02 exists to provide, and it cannot be checked by
importing inside the test process: by the time pytest runs, the package root has
usually been imported already and the answer is contaminated.  So each check
runs a *fresh interpreter* and asks what came along.

Two separate claims are made:

1. **The low layer is a leaf.**  Importing ``modeling`` and its submodules must
   not load ``templates``, ``subsystems``, ``rigs``, ``connections``,
   ``preparation``, ``simulation``, ``kernel`` or ``report``.  A declaration has
   to be usable without the machinery that assembles it.

2. **Import order does not matter.**  Each public entry point is imported first,
   alone, in its own interpreter, and then the low layer on top of it.  The
   historical failure mode was that entering ``elements`` or ``cases`` first left
   one of them half-built when another asked for a name; the ordering patch in
   ``api.py`` only hid it.

The gates read the module list rather than the source text, because the failure
being guarded against is a *runtime* one: a module that is imported has run.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parents[4]

#: Packages the low layer must not import, directly or transitively.
FORBIDDEN_FOR_LOW_LAYER = (
    "suspension_multibody.templates",
    "suspension_multibody.subsystems",
    "suspension_multibody.rigs",
    "suspension_multibody.connections",
    "suspension_multibody.preparation",
    "suspension_multibody.simulation",
    "suspension_multibody.kernel",
    "suspension_multibody.report",
)

#: Low-layer modules, imported in a fresh process.
LOW_LAYER_MODULES = (
    "suspension_multibody.modeling",
    "suspension_multibody.modeling.identity",
    "suspension_multibody.modeling.ports",
    "suspension_multibody.modeling.instance",
    "suspension_multibody.modeling.assembly",
    "suspension_multibody.modeling.units",
    "suspension_multibody.modeling.primitives",
    "suspension_multibody.modeling.primitives.joints",
    "suspension_multibody.modeling.primitives.spatial",
)

#: Entry points that have historically been able to break the import order.
#:
#: Deliberately *every* top-level subpackage, not only the ones a caller is
#: expected to touch: the failure this guards against is a cycle between two
#: modules that are each individually fine, and a list of "important" entries
#: misses exactly the pair nobody thought to name.  A cycle through
#: ``preparation`` and ``results`` survived a narrower list, which is why the
#: list is now derived from the package rather than curated.
PUBLIC_ENTRY_POINTS = (
    "suspension_multibody.adams",
    "suspension_multibody.adapters",
    "suspension_multibody.api",
    "suspension_multibody.axle_dynamics",
    "suspension_multibody.cases",
    "suspension_multibody.connections",
    "suspension_multibody.io",
    "suspension_multibody.joints",
    "suspension_multibody.kernel",
    "suspension_multibody.modeling",
    "suspension_multibody.outputs",
    "suspension_multibody.preparation",
    "suspension_multibody.properties",
    "suspension_multibody.report",
    "suspension_multibody.results",
    "suspension_multibody.rigs",
    "suspension_multibody.schema",
    "suspension_multibody.simulation",
    "suspension_multibody.studies",
    "suspension_multibody.subsystems",
    "suspension_multibody.templates",
    "suspension_multibody.vehicle",
)


def _run(script: str) -> subprocess.CompletedProcess[str]:
    """Run a snippet in a fresh interpreter rooted at the workspace."""
    return subprocess.run(
        [sys.executable, "-c", script],
        cwd=ROOT,
        capture_output=True,
        check=False,
        text=True,
        # Decode explicitly: this repository's path is non-ASCII, and a locale
        # codec raises on bytes the child prints.
        encoding="utf-8",
        errors="replace",
    )


def _import_then_report(module: str, then: str | None = None) -> list[str]:
    """Import ``module`` (and optionally ``then``) in a fresh process."""
    follow = "" if then is None else f"importlib.import_module({then!r})"
    script = f"""
import importlib, json, sys
importlib.import_module({module!r})
{follow}
print(json.dumps(sorted(
    name for name in sys.modules if name.startswith("suspension_multibody")
)))
"""
    completed = _run(script)
    assert completed.returncode == 0, (
        f"importing {module!r} failed:\n{completed.stderr}"
    )
    return json.loads(completed.stdout)


@pytest.mark.parametrize("module", LOW_LAYER_MODULES)
def test_low_layer_module_imports_without_the_authoring_chain(module: str) -> None:
    loaded = _import_then_report(module)
    leaked = [
        name
        for name in loaded
        if any(
            name == forbidden or name.startswith(forbidden + ".")
            for forbidden in FORBIDDEN_FOR_LOW_LAYER
        )
    ]
    assert not leaked, (
        f"importing {module} pulled in {leaked}; the low layer must be usable "
        "without the chain that assembles it"
    )


@pytest.mark.parametrize("entry", PUBLIC_ENTRY_POINTS)
def test_public_entry_point_imports_in_a_fresh_process(entry: str) -> None:
    """Each entry point must be importable first, on its own."""
    _import_then_report(entry)


@pytest.mark.parametrize("entry", PUBLIC_ENTRY_POINTS)
def test_low_layer_still_imports_after_any_entry_point(entry: str) -> None:
    """
    Whatever is entered first, the low layer must still be reachable.

    This is the ordering half of the old defect: a name that was only defined
    because of the order the package happened to be entered in.
    """
    loaded = _import_then_report(entry, "suspension_multibody.modeling")
    assert "suspension_multibody.modeling" in loaded


def test_every_ordered_pair_of_entry_points_imports() -> None:
    """
    The pairwise sweep, because the defect was order-dependent.

    Two entries at a time is enough to expose a cycle: a genuine A<->B loop
    shows up as a half-initialised module for at least one of the two orders.
    """
    failures: list[str] = []
    for first in PUBLIC_ENTRY_POINTS:
        for second in PUBLIC_ENTRY_POINTS:
            if first == second:
                continue
            script = f"""
import importlib, sys
importlib.import_module({first!r})
importlib.import_module({second!r})
"""
            completed = _run(script)
            if completed.returncode != 0:
                failures.append(
                    f"{first} then {second}: {completed.stderr.strip().splitlines()[-1]}"
                )
    assert not failures, "import order is still load-bearing:\n" + "\n".join(failures)


def test_the_package_root_imports_without_adams() -> None:
    """Importing the product must never start a licence server."""
    script = """
import importlib, json, sys
importlib.import_module("suspension_multibody")
print(json.dumps(sorted(
    name for name in sys.modules
    if name == "suspension_multibody.adams"
    or name.startswith("suspension_multibody.adams.")
)))
"""
    completed = _run(script)
    assert completed.returncode == 0, completed.stderr
    assert json.loads(completed.stdout) == []


def test_low_layer_does_not_import_the_package_root_eagerly() -> None:
    """
    Importing a low-layer module must not execute the package's public surface.

    ``suspension_multibody/__init__.py`` imports ``api``, which imports most of
    the product.  A low-layer module imported through the package therefore pays
    for the whole chain -- which is what made the old ordering constraint
    load-bearing in the first place.
    """
    script = """
import importlib, json, sys
importlib.import_module("suspension_multibody.modeling.identity")
print(json.dumps(sorted(
    name for name in sys.modules if name.startswith("suspension_multibody")
)))
"""
    completed = _run(script)
    assert completed.returncode == 0, completed.stderr
    loaded = json.loads(completed.stdout)
    # Only the package root and the low layer itself may be present.
    unexpected = [
        name
        for name in loaded
        if name != "suspension_multibody"
        and not name.startswith("suspension_multibody.modeling")
    ]
    assert not unexpected, (
        "importing a low-layer module executed the package root's public "
        f"surface: {unexpected}"
    )
