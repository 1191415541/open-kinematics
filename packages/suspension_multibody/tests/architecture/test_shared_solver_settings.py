"""
06: the integration settings are shared, and a document declares its own units.

Two properties this subtask delivers, held as structure rather than as behaviour,
because both are about *where* something lives:

* the solver settings are reachable without importing the axle package -- they
  describe how a run is advanced, which every family does, so a family that never
  mentions an axle should not have to import one to say how it integrates;
* the emitters declare the unit their contract is in and leave the scaling to the
  kernel -- a conversion inside an emitter is a rounding step between the document
  and the frozen numbers.

Both are static: a dependency is visible in the source, and a conversion is
visible as arithmetic.  A run-time check would only catch the paths a test
happened to exercise.
"""
from __future__ import annotations

import ast
import re
from pathlib import Path

SOURCE = Path(__file__).parents[2] / "src" / "suspension_multibody"

#: The neutral module the shared settings live in.
_NEUTRAL = SOURCE / "schema" / "solver.py"

#: The families that read the settings but do not own an axle.
_SHARED_READERS = (
    "api.py",
    "compilation/plan.py",
    "cases/kc_quasi_static/settings.py",
    "authoring/migration.py",
    "compilation/resolved.py",
    "cases/handling.py",
    "cases/ride_four_post.py",
    "cases/ride_random_road.py",
)


def _imports(path: Path) -> set[str]:
    """Return the module paths one file imports from."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    found: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            found.add(node.module)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                found.add(alias.name)
    return found


def test_the_settings_live_in_the_neutral_layer():
    """The class is defined where every family can reach it."""
    assert _NEUTRAL.is_file()
    tree = ast.parse(_NEUTRAL.read_text(encoding="utf-8"))
    classes = {node.name for node in ast.walk(tree) if isinstance(node, ast.ClassDef)}
    assert "AxleSolverSettings" in classes


def test_the_axle_schema_no_longer_defines_the_settings():
    """
    The axle schema re-exports rather than defines.

    A definition here is what made a shared setting look private to one family.
    """
    axle = SOURCE / "axle_dynamics" / "schema.py"
    tree = ast.parse(axle.read_text(encoding="utf-8"))
    classes = {node.name for node in ast.walk(tree) if isinstance(node, ast.ClassDef)}
    assert "AxleSolverSettings" not in classes
    assert "schema.solver" in _imports(axle)


def test_no_reader_takes_the_settings_from_the_axle_schema():
    """
    The readers reach the settings from the neutral layer, not from the axle.

    This is the property the move exists for: "how a run is advanced" is not a
    fact about axles.  A dynamic family may still import the axle schema for its
    element types -- that is what those types are for -- but the *settings* must
    come from where they are defined, so a change to them reaches every family
    through one path.
    """
    offenders = []
    for relative in _SHARED_READERS:
        path = SOURCE / relative
        if not path.is_file():
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, ast.ImportFrom) or not node.module:
                continue
            if not node.module.endswith("axle_dynamics.schema"):
                continue
            if any(alias.name == "AxleSolverSettings" for alias in node.names):
                offenders.append(f"{relative}:{node.lineno}")
    assert not offenders, (
        "these imports still take the shared settings from the axle schema: "
        f"{offenders}"
    )


def test_the_settings_are_the_same_class_both_ways():
    """An existing import keeps working, and it is the same object."""
    from suspension_multibody.axle_dynamics import AxleSolverSettings as via_axle
    from suspension_multibody.schema.solver import AxleSolverSettings as neutral

    assert via_axle is neutral


def test_the_settings_defaults_are_unchanged():
    """
    The move must not move a number.

    The integrator and the tolerances decide what the solver does, so a changed
    default would be a changed result -- and the frozen baselines are the ones
    these values produced.
    """
    from suspension_multibody.schema.solver import AxleSolverSettings

    settings = AxleSolverSettings()
    assert settings.integrator == "ggl_generalized_alpha"
    assert settings.rho_inf == 0.8
    assert settings.hht_alpha == -0.3
    assert settings.initialization_mode == "static_equilibrium"
    assert settings.internal_step_s == 0.00025
    assert settings.max_newton_iterations == 20


#: An emitter that scaled a length by hand would carry one of these factors.
#: The kernel scales from the document's own ``units`` block, so an emitter that
#: does it too is a second, quieter conversion.
_HAND_SCALE = re.compile(r"\*\s*(1000|0\.001)|/\s*(1000|0\.001)")


def test_no_emitter_scales_a_length_by_hand():
    """
    The emitters declare a unit; they do not convert one.

    A length written in millimetres and read back is a multiply and a divide by a
    number that is not a power of two, and the round trip costs an ulp -- which a
    run that has to reproduce a frozen byte-for-byte hash cannot afford.  The
    document therefore states its unit and the kernel scales on the way in, which
    is why no emitter may carry the factor itself.
    """
    offenders: list[str] = []
    for path in sorted((SOURCE / "cases").rglob("*.py")):
        text = path.read_text(encoding="utf-8")
        for number, line in enumerate(text.splitlines(), start=1):
            stripped = line.strip()
            if stripped.startswith("#") or "units" in stripped:
                continue
            if _HAND_SCALE.search(stripped):
                offenders.append(f"{path.relative_to(SOURCE)}:{number}")
    assert not offenders, (
        "these emitter lines scale a length by hand instead of declaring the "
        f"document's units: {offenders}"
    )


def test_every_emitter_states_the_units_it_writes_in():
    """
    A document says what its numbers are, and the kernel honours it.

    This is the other half of the same rule: if the emitter may not scale, the
    document has to carry the unit, or nothing does.
    """
    for name in ("compilation/resolved.py", "authoring/generic.py"):
        path = SOURCE / name
        text = path.read_text(encoding="utf-8")
        assert "units" in text, f"{name} does not state the units it writes in"
