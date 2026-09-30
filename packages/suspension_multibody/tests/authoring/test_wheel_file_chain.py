"""
The wheel role's file carrier: a document's wheel subsystem reaches the assembly.

Subtask 04b's subject.  04 made `subsystems/wheel.py` the wheel end's single
producer and left the read channel open (`getattr(request, "wheel_template")`);
what was missing was anywhere for a *file's* wheel subsystem to be put.  The two
halves that close it are:

* `AssemblyRequest.wheel_template`, with the role table entry that lets
  `role_instance("wheel")` answer, and
* `_FILE_ROLE_TEMPLATES` naming wheel, so `assembly_request_for` forwards a
  document's wheel entry the way it forwards steering and chassis.

The test asks the assembled thing, not the plumbing: the wheel role's own
contribution names the bodies the *file* declared, which the built-in template
cannot produce (it declares no parts at all), and the axle then condenses them
into the body carrying the wheel centre -- so the file route describes wheels
without moving anything the frozen baselines were recorded against.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from suspension_multibody.authoring.documents import (
    SimulationAssembly,
    SubsystemDocument,
)
from suspension_multibody.authoring.solver import (
    assembly_request_for,
    front_axle_model_for,
)
from suspension_multibody.subsystems.si_assembly import (
    _wheel_bodies,
    axle_contributions_and_order,
)
from suspension_multibody.subsystems.types import AssemblyRequest
from tests.authoring.fixtures import write_axle_project

#: The wheel body the file describes, and what it weighs.
_WHEEL_MASS = 12.0


def _wheel_documents() -> tuple[dict[str, object], dict[str, object]]:
    """Return the wheel template and wheel subsystem a document describes."""
    template = {
        "document": "template",
        "schema_version": 1,
        "name": "file_wheel",
        "functional_role": "wheel",
        "allowed_placement_roles": ["any"],
        # The wheel body is the template's own part, which is the whole point: the
        # built-in declares none, so a body here can only have come from the file.
        "bodies": [{"name": "wheel_L", "mass": _WHEEL_MASS}],
        "hardpoints": [
            {"name": "wheel_center", "owner": "wheel_L", "label": "wheel_center"},
            {"name": "spin_axis", "owner": "wheel_L", "label": "spin_axis"},
        ],
        "joints": [],
        "elements": [],
        "property_slots": [],
        "ports": [{"name": "wheel_center", "role": "wheel_center", "owner": "wheel_L"}],
    }
    subsystem = {
        "document": "subsystem",
        "schema_version": 1,
        "name": "wheel_end",
        "template": "wheel.tpl.json",
        "functional_role": "wheel",
        "placement_role": "any",
        "hardpoints": {
            # The wheel centre is the place the wheel hangs, in the model's own
            # coordinates; the spin-axis mount is required by the role contract and
            # is not placed by anything here.
            "wheel_center": [0.0, -700.0, 300.0],
            "spin_axis": [0.0, 0.0, 300.0],
        },
        "property_bindings": {},
    }
    return template, subsystem


def _project_with_a_wheel_file(tmp_path: Path) -> dict[str, Path]:
    """Write the fixture project, plus a wheel subsystem the assembly places."""
    paths = write_axle_project(tmp_path)
    template, subsystem = _wheel_documents()
    (tmp_path / "wheel.tpl.json").write_text(json.dumps(template), encoding="utf-8")
    (tmp_path / "wheel.sub.json").write_text(json.dumps(subsystem), encoding="utf-8")
    payload = json.loads(paths["assembly"].read_text(encoding="utf-8"))
    payload["subsystems"].append(
        {"ref": "wheel.sub.json", "functional_role": "wheel", "placement_role": "any"}
    )
    paths["assembly"].write_text(json.dumps(payload), encoding="utf-8")
    return paths


def test_a_default_request_carries_no_wheel_template() -> None:
    """
    The field is optional, and its absence is the built-in: D9's state.

    `role_instance` answers `None` for a role the request says nothing about, and
    `None` means "that role's own default" -- for the wheel role, the registered
    built-in.  So every existing caller is unaffected by the field existing.
    """
    request = AssemblyRequest(mode="K")
    assert request.wheel_template is None
    assert request.role_instance("wheel") is None


def test_a_document_s_wheel_subsystem_reaches_the_request(tmp_path: Path) -> None:
    """
    The file's wheel entry is forwarded, in the same form as steering and chassis.

    Both halves are asserted: the role is part of the request's subsystem set (the
    document placed it), and the template that arrives is the *file's* -- resolved
    by name through the template document rather than the built-in.
    """
    paths = _project_with_a_wheel_file(tmp_path)
    request = assembly_request_for(SimulationAssembly.load(paths["assembly"]))

    assert "wheel" in request.subsystems
    instance = request.role_instance("wheel")
    assert instance is not None
    assert instance.template.name == "file_wheel"
    # The file writes one side; the conversion mirrors it, exactly as it does for
    # every other file template.
    assert [part.name for part in instance.template.parts] == ["wheel_L", "wheel_R"]


def test_the_file_s_wheel_body_is_produced_and_then_condensed(tmp_path: Path) -> None:
    """
    End to end: the file's wheel is built by the wheel role, and the axle folds it in.

    The two claims are ordered, because the second is what makes the first safe:
    the wheel role's contribution names the file's own ``wheel_L``/``wheel_R``
    (which the built-in cannot produce), and the composed runtime does *not* carry
    them -- the composition condensed them into the body that carries the wheel
    centre, which is what keeps the frozen single-axle products where they were.
    """
    paths = _project_with_a_wheel_file(tmp_path)
    request = assembly_request_for(SimulationAssembly.load(paths["assembly"]))
    model = front_axle_model_for(SubsystemDocument.load(paths["subsystem"]))

    contributions, _ = axle_contributions_and_order(model, request=request)
    assert sorted(_wheel_bodies(contributions)) == ["wheel_L", "wheel_R"]

    from suspension_multibody.subsystems.entry import compose_axle

    runtime = compose_axle(model, request=request)
    assert "wheel_L" not in runtime.bodies
    assert "wheel_R" not in runtime.bodies
    # The wheel's mass ended up on the body that carries the wheel centre: this
    # file's suspension declares no hub, so that body is the upright.
    # The wheel's mass ended up on the body that carries the wheel centre: this
    # file's suspension declares no hub, so that body is the upright, whose own
    # declared 20 kg the wheel's 12 kg is added to.
    assert runtime.bodies["upright_L"].mass == pytest.approx(20.0 + _WHEEL_MASS, rel=1e-12)
    assert runtime.bodies["upright_R"].mass == pytest.approx(20.0 + _WHEEL_MASS, rel=1e-12)
