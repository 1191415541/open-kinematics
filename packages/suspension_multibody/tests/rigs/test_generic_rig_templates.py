"""Bench templates are normal serializable subsystem declarations."""

import pytest

from suspension_multibody.authoring import (
    AssemblyDocument,
    TemplateDocument,
    assemble_generic,
)
from suspension_multibody.authoring.migration import save_migrated_assembly
from suspension_multibody.rigs import RIGS

from ._generic import rig_source


@pytest.mark.parametrize("name", tuple(RIGS))
def test_each_registered_bench_roundtrips_and_assembles(name, tmp_path):
    source = rig_source(name)
    rig = source.entries[-1].subsystem
    template = rig.template
    assert TemplateDocument.load(template.save(tmp_path / "rig.tpl.json")).payload == template.payload
    path = save_migrated_assembly(source, tmp_path / "project")
    memory, files = assemble_generic(source), assemble_generic(AssemblyDocument.load(path))
    assert memory.model_document() == files.model_document()
    assert set(files.bodies) == {"specimen.member", "rig.fixture"}
    assert not files.tires and not files.joints


def test_binding_uses_ports_and_arbitrary_body_names():
    graph = assemble_generic(rig_source(body="arbitrary_part"))
    assert all(row["owner"] == "specimen.arbitrary_part" for row in graph.inputs)
    assert all(row["port"].startswith("specimen/") for row in graph.inputs)
