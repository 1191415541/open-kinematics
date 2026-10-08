"""Rig entities reach the common model and their source remains traceable."""

import pytest

from suspension_multibody.authoring import assemble_generic
from tests.rigs._generic import wheel_specimen


def test_declared_bench_reaches_the_native_document():
    graph = assemble_generic(wheel_specimen("kc_quasi_static"))
    assert set(graph.fragments[-1].bodies) == {"rig.fixture"}
    assert graph.fragments[-1].provenance.template == "kc_quasi_static_rig"
    assert "rig.fixture" in graph.bodies
    assert any(row["name"] == "rig.fixture" for row in graph.model_document()["bodies"])
    assert {row["port"] for row in graph.inputs} == {"left/hub", "right/hub"}
    assert not any(row["name"].startswith("rig.") for row in graph.tires)


@pytest.mark.parametrize("value", ["0", "1", "true", "yes", ""])
def test_environment_switch_cannot_change_declared_entities(monkeypatch, value):
    source = wheel_specimen("kc_quasi_static")
    before = assemble_generic(source).model_document()
    monkeypatch.setenv("SUSPENSION_MULTIBODY_RIG_ENTITIES", value)
    assert assemble_generic(source).model_document() == before


def test_omitting_a_bench_omits_only_its_own_entities():
    graph = assemble_generic(wheel_specimen())
    assert "rig.fixture" not in graph.bodies
    assert not graph.inputs
    assert len(graph.tires) == 2
