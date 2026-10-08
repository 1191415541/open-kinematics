"""Memory composition carries the complete file graph with no lazy file reads."""

import shutil

import pytest

from suspension_multibody.authoring import AssemblyDocument, assemble_generic
from suspension_multibody.authoring.migration import save_migrated_assembly
from suspension_multibody.connections.matcher import BindingError
from tests.connections.test_links import declared_links

from ._generic import axle_source


@pytest.mark.parametrize("mode", ["K", "C"])
def test_file_and_memory_agree_on_all_resolved_values(tmp_path, mode):
    original = axle_source(mode)
    loaded = AssemblyDocument.load(save_migrated_assembly(original, tmp_path / "project"))
    assert [(e.ref, e.functional_role, e.placement_role) for e in original.entries] == [(e.ref, e.functional_role, e.placement_role) for e in loaded.entries]
    first, second = [assemble_generic(doc).resolved_model() for doc in (original, loaded)]
    assert first.to_document() == second.to_document()
    assert first.fingerprint == second.fingerprint
    shutil.rmtree(tmp_path / "project")
    assert assemble_generic(loaded).resolved_model().fingerprint == first.fingerprint


def test_both_routes_refuse_the_same_missing_port(tmp_path):
    original = declared_links(ports=())
    loaded = AssemblyDocument.load(save_migrated_assembly(original, tmp_path))
    for source in (original, loaded):
        with pytest.raises(BindingError, match="mount"):
            assemble_generic(source)
