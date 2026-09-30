"""
The bench is an excitation and a clamp, and it does not rewrite the assembly.

Subtask 05's contract (decision D3): binding a bench to an assembly may add the
bench's **own** entities and nothing else.  Every body, point, constraint and
element the assembly had must come back with the same ownership, the same numbers
and the same geometry -- because "the bench is in the model" has to stay a
statement about the excitation, not about which model was solved.

The comparison here is item-wise and per rig, rather than the frozen snapshot's:
the snapshot covers the two wheel-supplying axle benches, and the five vehicle
benches bind themselves in the study layer, so `snapshot.py` says explicitly that
they are outside its coverage and that this subtask has to do its own before/after
comparison.  This is that comparison.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.rigs.rig import RIGS
from suspension_multibody.schema import FrontAxleModel
from suspension_multibody.subsystems.rig_link import (
    link_wheel_supplying_rig,
    merge_rig_link,
)
from suspension_multibody.subsystems.si_assembly import si_assembly_for_axle
from suspension_multibody.subsystems.types import AssemblyRequest

FIXTURE = Path(__file__).parents[1] / "data" / "benchmark_axle.json"

#: The benches that supply the wheels themselves.
_WHEEL_BENCHES = tuple(name for name, spec in RIGS.items() if spec.supplies_wheels)


def _model(*, with_tire: bool = True) -> FrontAxleModel:
    """Return the benchmark axle, optionally carrying a declared tire."""
    raw = json.loads(FIXTURE.read_text(encoding="utf-8"))["model"]
    if with_tire:
        raw["tires"] = [
            {
                "stiffness": 200.0,
                "unloaded_radius": 320.0,
                "contact_point": {"x": 0.0, "y": 0.0, "z": 0.0},
                "local_axis": {"x": 0.0, "y": 0.0, "z": 1.0},
            }
        ]
    return FrontAxleModel.model_validate(raw)


def _fingerprints(runtime) -> dict[str, dict[str, str]]:
    """
    Return a comparable fingerprint of every entity a runtime carries.

    `repr` rather than a structural diff on purpose: the entities are frozen
    dataclasses holding arrays, so two of them are the same entity exactly when
    their text is the same -- and a comparison that decomposes them field by field
    would be a second description of their shape, which is the thing that drifts.
    """
    return {
        "bodies": {name: repr(body) for name, body in runtime.bodies.items()},
        "points": {str(key): repr(value) for key, value in runtime.points.items()},
        "constraints": {str(index): repr(row) for index, row in enumerate(runtime.constraints)},
        "ideal_constraints": {
            str(index): repr(row) for index, row in enumerate(runtime.ideal_constraints)
        },
        "elements": {str(index): repr(row) for index, row in enumerate(runtime.elements)},
        "connections": {
            str(index): repr(row) for index, row in enumerate(runtime.connections)
        },
    }


def _composed(runtime, rig_name: str):
    """Return the runtime with one bench linked and merged into it."""
    composed = si_assembly_for_axle(_model(), request=AssemblyRequest(mode="K"), rig=rig_name)
    bench = composed.rig
    link = link_wheel_supplying_rig(runtime, bench, mode="K")
    return merge_rig_link(runtime, link)


@pytest.mark.parametrize("rig_name", _WHEEL_BENCHES)
def test_a_wheel_bench_adds_only_its_own_entities(rig_name: str) -> None:
    """
    Item-wise: the assembly comes back unchanged, and the addition is the bench's.

    The two halves are stated separately, because either alone can be true for the
    wrong reason -- "nothing changed" would also hold for a bench that contributed
    nothing at all, and "the bench contributed" says nothing about whether it
    rewrote what was there.
    """
    plain = si_assembly_for_axle(_model(), request=AssemblyRequest(mode="K")).assembly.physical
    merged = _composed(plain, rig_name)

    before = _fingerprints(plain)
    after = _fingerprints(merged)

    # The bench brought its carriers, and nothing else.
    assert set(after["bodies"]) - set(before["bodies"]) == {
        "wheel_carrier_L",
        "wheel_carrier_R",
    }
    assert set(after["points"]) - set(before["points"]) == {
        "('wheel_carrier_L', 'center')",
        "('wheel_carrier_L', 'contact')",
        "('wheel_carrier_R', 'center')",
        "('wheel_carrier_R', 'contact')",
    }
    assert len(after["constraints"]) - len(before["constraints"]) == 2  # the two welds
    assert len(after["connections"]) - len(before["connections"]) == 2

    # Every entity the assembly already had is the same entity: same owner, same
    # numbers, same geometry.  Compared by content, not by count.
    for kind in ("bodies", "points", "constraints", "ideal_constraints", "elements"):
        for key, fingerprint in before[kind].items():
            assert after[kind][key] == fingerprint, f"{kind}/{key} was rewritten"


def test_the_bench_attaches_through_the_wheel_centre_fixture() -> None:
    """
    The one thing the bench adds that touches the assembly is a clamp at the wheel
    centre -- the contact-point fixture D3 allows, and the only place it may touch.

    Anything else it added would be the bench editing the model, which is what the
    immutability rule is about; the weld's point is the wheel centre in world
    coordinates, which is what makes it a fixture rather than a re-definition.
    """
    plain = si_assembly_for_axle(_model(), request=AssemblyRequest(mode="K")).assembly.physical
    merged = _composed(plain, "kc_quasi_static")

    added = merged.constraints[len(plain.constraints) :]
    assert [getattr(row, "name", "") for row in added] == [
        "wheel_carrier_L_weld",
        "wheel_carrier_R_weld",
    ]
    for row in added:
        assert type(row).__name__ == "WeldJoint"
        assert row.body_b.startswith("wheel_carrier_")
        # One physical place, resolved per body by the emitter's world-coordinate
        # convention: a fixture states a point, not two local offsets.
        assert np.array_equal(row.point_a, row.point_b)
    # And the assembly's own tire is still where the assembly put it.
    assert [getattr(e, "wheel_body", None) for e in merged.elements] == [
        getattr(e, "wheel_body", None) for e in plain.elements
    ]


@pytest.mark.parametrize("rig_name", sorted(RIGS))
def test_binding_any_bench_leaves_the_assembly_it_loads_unchanged(rig_name: str) -> None:
    """
    All seven benches, including the five the frozen snapshot does not cover.

    A vehicle bench binds its actuators in the study layer rather than here, so at
    this layer it adds nothing at all -- which is a claim about the *assembly*: the
    model a vehicle bench loads is the model the vehicle composed, byte for byte.
    """
    plain = si_assembly_for_axle(_model(with_tire=False), request=AssemblyRequest(mode="K")).assembly.physical
    composed = si_assembly_for_axle(
        _model(with_tire=False), request=AssemblyRequest(mode="K"), rig=rig_name
    )
    merged = composed.assembly.physical

    before = _fingerprints(plain)
    after = _fingerprints(merged)
    for kind in ("bodies", "points", "constraints", "ideal_constraints", "elements"):
        for key, fingerprint in before[kind].items():
            assert after[kind][key] == fingerprint, f"{rig_name}: {kind}/{key} was rewritten"
    if rig_name in _WHEEL_BENCHES:
        assert set(after["bodies"]) > set(before["bodies"])
    else:
        assert set(after["bodies"]) == set(before["bodies"])
