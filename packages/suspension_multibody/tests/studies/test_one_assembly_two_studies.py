"""Study plans read the same resolved physical model through one compiler."""

import pytest

from suspension_multibody.authoring import (
    DocumentLoader,
    assemble_generic,
    migrate_v1_axle,
)
from suspension_multibody.compilation import compile_resolved
from suspension_multibody.modeling.resolved import ResolvedSolvePlan
from suspension_multibody.schema import RigidBodySpec
from suspension_multibody.studies import StudyError, get_study
from tests.benchmark_fixture import benchmark_model


def plan(study):
    return ResolvedSolvePlan({"schema_version": 1, "name": "reading", "study": study,
        "samples": [0, .001], "solver": {}, "boundaries": [], "inputs": [], "outputs": []})


@pytest.mark.parametrize("mode", ["K", "C"])
def test_studies_keep_one_model_and_identical_physical_documents(mode):
    graph = assemble_generic(migrate_v1_axle(benchmark_model(), mode=mode))
    model = graph.resolved_model()
    compiled = [compile_resolved(model, plan(study)) for study in ("quasi_static", "dynamic")]
    assert compiled[0].model_document == compiled[1].model_document
    assert model.to_document()["bodies"] == graph.model_document()["bodies"]
    assert compiled[0].case_document["family"] != compiled[1].case_document["family"]
    assert all(row["mass"] == graph.bodies[row["name"]].mass for row in model.to_document()["bodies"])


def test_loader_and_direct_assembly_resolve_the_same_model():
    source = migrate_v1_axle(benchmark_model())
    from tests.authoring.test_generic_multibody import _case
    model = DocumentLoader().load(source, _case()).resolve()
    assert model.fingerprint == assemble_generic(source).resolved_model().fingerprint


def test_invalid_study_and_missing_dynamic_mass_are_named():
    with pytest.raises(StudyError, match="unknown study"):
        get_study("missing")
    model = benchmark_model()
    model = model.model_copy(update={"topology": "explicit", "joints": (),
        "bodies": (RigidBodySpec(name="unmassed"),)})
    with pytest.raises(ValueError, match="free body has no declared mass"):
        migrate_v1_axle(model)
