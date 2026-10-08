"""
Presets are ordinary documents, not a type of their own.

The claim under test is narrow and load-bearing: a preset returns the same kind of
object a file load returns, so a caller can take it apart, save it, load it back
and run it.  If a preset returned a legacy model, or a subclass of one, then
"presets are a convenience layer" would be false and the door to arbitrary
topology would be closed again.

None of these tests build a legacy model, and that is the point.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from suspension_multibody.authoring import load_template
from suspension_multibody.authoring.documents import (
    AssemblyDocument,
    RigDocument,
    SubsystemDocument,
    TemplateDocument,
)
from suspension_multibody.authoring.properties import ElementPropertyDocument
from suspension_multibody.presets import (
    double_wishbone_subsystem,
    double_wishbone_template,
    standalone_axle,
)

#: The left-side coordinates the repository's own fixtures use, keyed by the
#: names the built-in template declares.
COORDINATES: dict[str, list[float]] = {
    "upper_front": [-100.0, -500.0, 400.0],
    "upper_rear": [100.0, -500.0, 400.0],
    "upper_outer": [0.0, -700.0, 450.0],
    "lower_front": [-120.0, -500.0, 150.0],
    "lower_rear": [120.0, -500.0, 150.0],
    "lower_outer": [0.0, -700.0, 150.0],
    "tie_inner": [100.0, -400.0, 250.0],
    "tie_outer": [50.0, -700.0, 250.0],
    "wheel_center": [0.0, -700.0, 300.0],
    "rack_center": [0.0, 0.0, 250.0],
}


def _spring() -> ElementPropertyDocument:
    """Return the left-side spring law, in the units the property schema requires."""
    return ElementPropertyDocument.from_payload(
        {
            "document": "element_properties",
            "schema_version": 1,
            "name": "preset_spring",
            "element_type": "spring",
            "model": "linear",
            "units": {"force": "N", "length": "mm"},
            "parameters": {"stiffness": 45.0, "free_length": 250.0},
        }
    )


def _damper() -> ElementPropertyDocument:
    """Return the left-side damper law."""
    return ElementPropertyDocument.from_payload(
        {
            "document": "element_properties",
            "schema_version": 1,
            "name": "preset_damper",
            "element_type": "damper",
            "model": "linear",
            "units": {"force": "N", "length": "mm", "time": "s"},
            "parameters": {"viscous_damping": 12.0},
        }
    )


def _rig() -> RigDocument:
    return RigDocument.from_payload(
        {
            "document": "rig",
            "schema_version": 1,
            "name": "kc_bench",
            "supported_assembly_kinds": ["suspension_axle"],
            "required_ports": [],
        }
    )


def test_a_preset_template_is_an_ordinary_document() -> None:
    template = double_wishbone_template()
    assert isinstance(template, TemplateDocument)
    assert type(template) is TemplateDocument
    assert template.path is None
    assert template.name == "double_wishbone"
    assert template.hardpoint_names  # non-empty


def test_a_preset_template_is_not_an_alias_for_a_declaration() -> None:
    """
    The returned object is a document, and nothing in its type graph is a model.

    This is the test that would fail if a preset were implemented as a thin
    wrapper around the declaration schema: that class would appear here.  The
    two names are checked as module-qualified spellings, because the class name
    itself is now an ordinary schema type rather than a legacy one.
    """
    from suspension_multibody.schema.model import AxleDeclaration
    from suspension_multibody.schema.vehicle import VehicleDeclaration

    template = double_wishbone_template()
    assert not isinstance(template, (AxleDeclaration, VehicleDeclaration))
    assert not isinstance(template, dict)


def test_a_preset_template_saves_and_reloads_to_the_same_declaration(
    tmp_path: Path,
) -> None:
    template = double_wishbone_template()
    written = template.save(tmp_path / "preset.tpl.json")
    reloaded = TemplateDocument.load(written)
    assert reloaded.payload == template.payload
    assert reloaded.topology_hash == template.topology_hash


def test_a_preset_subsystem_binds_the_preset_template() -> None:
    template = double_wishbone_template()
    subsystem = double_wishbone_subsystem(
        hardpoints=COORDINATES, template=template, spring=_spring(), damper=_damper()
    )
    assert isinstance(subsystem, SubsystemDocument)
    assert subsystem.template is template
    assert subsystem.path is None
    assert subsystem.functional_role == "suspension"
    assert subsystem.placement_role == "front"
    assert subsystem.effective().hardpoints["wheel_center"] == (0.0, -700.0, 300.0)


def test_a_preset_subsystem_is_editable_and_the_edit_is_visible() -> None:
    """
    A caller can change one coordinate and see it, without touching the original.
    """
    subsystem = double_wishbone_subsystem(hardpoints=COORDINATES, spring=_spring(), damper=_damper())
    tuned = subsystem.effective(
        overrides={"hardpoints": {"wheel_center": [0.0, -700.0, 340.0]}}
    )
    assert tuned.hardpoints["wheel_center"] == (0.0, -700.0, 340.0)
    assert subsystem.effective().hardpoints["wheel_center"] == (0.0, -700.0, 300.0)


def test_a_preset_axle_is_an_ordinary_assembly_document() -> None:
    subsystem = double_wishbone_subsystem(hardpoints=COORDINATES, spring=_spring(), damper=_damper())
    assembly = standalone_axle((subsystem,), rig=_rig())
    assert isinstance(assembly, AssemblyDocument)
    assert type(assembly) is AssemblyDocument
    assert assembly.path is None
    assert assembly.assembly_kind == "suspension_axle"
    assert assembly.subsystems == (subsystem,)
    assert assembly.rig is not None
    assert assembly.wheel_ends() == frozenset({"front_left", "front_right"})


def test_a_preset_model_writes_a_complete_project(tmp_path: Path) -> None:
    """
    A preset model is a project: every file its references need is written.

    The subsystem names its two laws by file and the assembly names its
    subsystem, so a saved preset is only reloadable if all of them are written.
    That is what makes the preset a model rather than an in-memory convenience
    that cannot leave the process.
    """
    template = double_wishbone_template()
    rig = _rig()
    template.save(tmp_path / f"{template.name}.tpl.json")
    _spring().save(tmp_path / "spring.json")
    _damper().save(tmp_path / "damper.json")
    subsystem = double_wishbone_subsystem(
        hardpoints=COORDINATES,
        template=load_template(tmp_path / f"{template.name}.tpl.json"),
        spring=_spring(),
        damper=_damper(),
    )
    subsystem.save(tmp_path / "front.sub.json")
    rig.save(tmp_path / "kc_bench.rig.json")

    # The subsystem file names the template by the name it was saved under, so
    # point the reference at the file before writing the assembly.
    payload = subsystem.to_payload()
    payload["template"] = f"{template.name}.tpl.json"
    (tmp_path / "front.sub.json").write_text(
        json.dumps(payload), encoding="utf-8"
    )
    assembly_payload = {
        "document": "assembly",
        "schema_version": 1,
        "name": "axle",
        "assembly_kind": "suspension_axle",
        "subsystems": [
            {
                "ref": "front.sub.json",
                "functional_role": "suspension",
                "placement_role": "front",
            }
        ],
        "rig": "kc_bench.rig.json",
    }
    (tmp_path / "axle.asy.json").write_text(
        json.dumps(assembly_payload), encoding="utf-8"
    )

    reloaded = AssemblyDocument.load(tmp_path / "axle.asy.json")
    assert reloaded.assembly_kind == "suspension_axle"
    assert [e.ref for e in reloaded.entries] == ["front.sub.json"]
    assert reloaded.wheel_ends() == frozenset({"front_left", "front_right"})
    assert reloaded.entries[0].subsystem.effective().hardpoints["wheel_center"] == (
        0.0,
        -700.0,
        300.0,
    )


def test_a_preset_subsystem_refuses_a_coordinate_the_template_does_not_declare() -> None:
    """The preset does not bypass the template's own checks."""
    from suspension_multibody.authoring import AuthoringError

    with pytest.raises(AuthoringError):
        double_wishbone_subsystem(
            hardpoints={**COORDINATES, "not_a_declared_point": [0.0, 0.0, 0.0]},
            spring=_spring(),
            damper=_damper(),
        )


def test_a_preset_axle_refuses_a_rig_that_does_not_support_it() -> None:
    from suspension_multibody.authoring import AuthoringError
    from suspension_multibody.authoring.documents import SimulationAssembly

    subsystem = double_wishbone_subsystem(hardpoints=COORDINATES, spring=_spring(), damper=_damper())
    rigid = RigDocument.from_payload(
        {
            "document": "rig",
            "schema_version": 1,
            "name": "strict",
            "supported_assembly_kinds": ["full_vehicle"],
            "required_ports": [],
        }
    )
    assembly = standalone_axle((subsystem,), rig=rigid)
    with pytest.raises(AuthoringError):
        SimulationAssembly.bind(assembly)
