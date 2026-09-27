"""
The production run path does not convert a K/C model into a dynamic one.

Subtasks 01-06 left two readings of one axle reachable through two assembly
paths, and step 06 noted that switching the production entry point over was 07's
work.  The conversion itself -- `studies.bridge.axle_dynamics_model`, the one
place millimetres and inertias become the SI dynamic schema -- is still correct
and still needed: a caller who has an *assembled* K/C axle and wants to advance it
in time has to be able to.  What must not happen is a *production run* depending
on it, because that is what "two live paths" means: every run would go through
the bridge, and the compiled pipeline would be a second implementation beside it.

The tests here therefore check the shape of the dependency graph rather than a
behaviour, and they use the source tree because that is where a dependency is
visible.  A run-time check would only catch the paths a test happened to exercise.
"""

from __future__ import annotations

import ast
from pathlib import Path

SOURCE = Path(__file__).parents[2] / "src" / "suspension_multibody"

#: The names that make up the K/C-to-dynamic conversion.
_BRIDGE_NAMES = frozenset({"axle_dynamics_model"})

#: The production entry modules a run goes through.
_ENTRY_MODULES = (
    "api.py",
    "cli.py",
    "axle_dynamics/contract_run.py",
    "vehicle/service.py",
    "simulation/runner.py",
    "simulation/preparation.py",
    "simulation/compiler.py",
    "simulation/dispatch.py",
    "preparation/kc_quasi_static.py",
)


def _called_names(path: Path) -> set[str]:
    """Return every plainly-named function this module calls."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    found: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            name = getattr(node.func, "id", None) or getattr(node.func, "attr", None)
            if isinstance(name, str):
                found.add(name)
    return found


def test_no_run_entry_reaches_the_kc_to_dynamic_conversion() -> None:
    """
    Removing the production bridge dependency, as a property of the tree.

    Every module a run goes through is checked, so the guarantee does not depend
    on which entry a future caller happens to use.  The conversion stays reachable
    from the input adapter -- that is an input route -- but no *run* module names
    it.
    """
    offenders: list[str] = []
    for relative in _ENTRY_MODULES:
        path = SOURCE / relative
        assert path.is_file(), f"the entry module list is stale: {relative} is gone"
        hits = _called_names(path) & _BRIDGE_NAMES
        if hits:
            offenders.append(f"{relative} calls {sorted(hits)}")
    assert not offenders, (
        "a production run path still converts a K/C model into a dynamic one: "
        + "; ".join(offenders)
        + ". The compiled pipeline is meant to be the only run path; a conversion "
        "on the way through it is the second assembly the EPIC exists to remove."
    )


def test_the_conversion_survives_only_as_an_input_adapter() -> None:
    """
    The other half, and it is the reason the check above can be strict.

    Deleting the conversion would make "an assembled axle, advanced in time"
    inexpressible, which is the study merge's whole point.  So the check is not
    "nothing mentions it" but "only the input adapter does", and this pins which
    module that is.
    """
    callers: set[str] = set()
    for path in SOURCE.rglob("*.py"):
        relative = path.relative_to(SOURCE).as_posix()
        if _called_names(path) & _BRIDGE_NAMES:
            callers.add(relative)
    assert callers == {"preparation/axle_dynamic.py"}, sorted(callers)


def test_the_input_adapter_still_converts_an_assembled_axle() -> None:
    """
    The route the previous test protects, exercised rather than described.

    A massed K/C assembly handed to the dynamic family must produce an SI model
    with the same bodies -- that is the study merge as a caller can reach it.
    """
    import json

    from suspension_multibody.preparation.axle_dynamic import _dynamic_model
    from suspension_multibody.schema import FrontAxleModel
    from suspension_multibody.simulation import SimulationRequest
    from suspension_multibody.subsystems.si_assembly import si_assembly_for_axle
    from suspension_multibody.subsystems.types import AssemblyRequest

    payload = json.loads(
        (Path(__file__).parents[1] / "data" / "benchmark_axle.json").read_text(
            encoding="utf-8"
        )
    )
    raw = dict(payload["model"])
    raw["bodies"] = [
        {
            "name": name,
            "mass": 100.0,
            "inertia": [[100.0, 0, 0], [0, 100.0, 0], [0, 0, 100.0]],
        }
        for name in (
            "rack",
            "upper_arm_L",
            "lower_arm_L",
            "upright_L",
            "tie_rod_L",
            "upper_arm_R",
            "lower_arm_R",
            "upright_R",
            "tie_rod_R",
        )
    ]
    assembly = si_assembly_for_axle(
        FrontAxleModel.model_validate(raw),
        request=AssemblyRequest(mode="K"),
    ).assembly.physical
    request = SimulationRequest(
        assembly="axle", rig="axle_dynamic", family="axle_dynamic", model=assembly
    )

    model = _dynamic_model(assembly, request)
    assert [body.name for body in model.bodies] == list(assembly.bodies)
    assert model.units == "SI"


def test_an_axle_declaring_no_inertia_is_refused_rather_than_advanced() -> None:
    """
    The adapter's other duty: refusing what it cannot read.

    The benchmark fixture declares no inertia, and the bridge's whole point is
    that it says so instead of inventing a mass -- a model that solves and means
    nothing is worse than a refusal.
    """
    import json

    import pytest

    from suspension_multibody.preparation.axle_dynamic import _dynamic_model
    from suspension_multibody.schema import FrontAxleModel
    from suspension_multibody.simulation import SimulationRequest
    from suspension_multibody.studies import BridgeError
    from suspension_multibody.subsystems.si_assembly import si_assembly_for_axle
    from suspension_multibody.subsystems.types import AssemblyRequest

    payload = json.loads(
        (Path(__file__).parents[1] / "data" / "benchmark_axle.json").read_text(
            encoding="utf-8"
        )
    )
    assembly = si_assembly_for_axle(
        FrontAxleModel.model_validate(payload["model"]),
        request=AssemblyRequest(mode="K"),
    ).assembly.physical
    request = SimulationRequest(
        assembly="axle", rig="axle_dynamic", family="axle_dynamic", model=assembly
    )
    with pytest.raises(BridgeError, match="carries no mass"):
        _dynamic_model(assembly, request)
