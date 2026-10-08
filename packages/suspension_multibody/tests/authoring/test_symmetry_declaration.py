"""Symmetry expands declaration data once and preserves explicit asymmetry."""

import numpy as np
import pytest

from suspension_multibody.authoring import GenericSubsystemAssembler, TemplateDocument
from suspension_multibody.authoring.errors import AuthoringError

from .test_generic_multibody import _subsystem, _template


def test_mirrored_declaration_materializes_both_sides_once(tmp_path):
    payload = _template(symmetry="mirrored_xz")
    payload["bodies"][0]["symmetric"] = False
    original = TemplateDocument.from_payload(payload)
    loaded = TemplateDocument.load(original.save(tmp_path / "mirror.json"))
    assert loaded.mirrors and loaded.declared_sides == ("left",)
    graph = GenericSubsystemAssembler().assemble(_subsystem(loaded.to_payload()))
    assert list(graph.bodies) == ["support", "carriage_L", "carriage_R"]
    assert [row["name"] for row in graph.joints] == ["guide_L", "guide_R"]


def test_explicit_asymmetric_bodies_are_not_mirrored_again():
    payload = _template(symmetry="asymmetric")
    payload["bodies"] += [{"name": "right", "mass": 10, "position": [0, .6, .2]}]
    graph = GenericSubsystemAssembler().assemble(_subsystem(payload))
    assert set(graph.bodies) == {"support", "carriage", "right"}
    np.testing.assert_array_equal(graph.bodies["right"].pose.translation, [0, .6, .2])


def test_single_corner_and_incoherent_mirror_declarations():
    payload = _template(sides=["left"], mirror=False)
    document = TemplateDocument.from_payload(payload)
    assert document.declared_sides == ("left",) and not document.mirrors
    assert set(GenericSubsystemAssembler().assemble(_subsystem(payload)).bodies) == {"support", "carriage"}
    with pytest.raises(AuthoringError, match="mirror writes the right side from the left"):
        TemplateDocument.from_payload(_template(symmetry="mirrored_xz", sides=["right"], mirror=True))
    with pytest.raises(AuthoringError, match="mirrors nothing"):
        TemplateDocument.from_payload(_template(symmetry="mirrored_xz", sides=["left", "right"], mirror=True))
