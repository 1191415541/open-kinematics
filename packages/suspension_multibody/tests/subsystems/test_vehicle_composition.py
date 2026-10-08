"""File and Python vehicles share one complete resolved graph."""

import pytest

from suspension_multibody.authoring import (
    AssemblyDocument,
    assemble_generic,
    migrate_v1_vehicle,
)
from suspension_multibody.authoring.migration import save_migrated_assembly
from tests.vehicle.vehicle_fixtures import _vehicle


@pytest.mark.parametrize("mode", ["K", "C"])
def test_every_physical_channel_matches_between_file_and_memory(mode, tmp_path):
    source = migrate_v1_vehicle(_vehicle(), mode=mode)
    path = save_migrated_assembly(source, tmp_path / "project")
    memory, files = [assemble_generic(document) for document in (source, AssemblyDocument.load(path))]
    assert memory.model_document() == files.model_document()
    for section in ("bodies", "joints", "elements", "frames", "ports", "coordinates", "tires"):
        assert memory.resolved_model().to_document()[section] == files.resolved_model().to_document()[section]
    assert memory.resolved_model().fingerprint == files.resolved_model().fingerprint


@pytest.mark.parametrize("variable", ["SUSPENSION_MULTIBODY_CONDENSE_WELDS", "SUSPENSION_MULTIBODY_DROP_ISOLATED_BODIES"])
def test_environment_does_not_select_another_vehicle_path(monkeypatch, variable):
    source = migrate_v1_vehicle(_vehicle())
    before = assemble_generic(source).model_document()
    for value in ("0", "1"):
        monkeypatch.setenv(variable, value)
        assert assemble_generic(source).model_document() == before


def test_invalid_mode_is_rejected():
    with pytest.raises(ValueError, match="mode"):
        migrate_v1_vehicle(_vehicle(), mode="X")
