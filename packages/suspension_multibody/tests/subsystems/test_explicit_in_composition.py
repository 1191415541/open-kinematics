"""Arbitrary declared joints use the same compiler without inferred automotive rows."""

import numpy as np
import pytest

from suspension_multibody.api import validate
from suspension_multibody.authoring import (
    GenericSubsystemAssembler,
    TemplateDocument,
    assemble_generic,
)
from suspension_multibody.modeling.primitives import (
    CylindricalJoint,
    InPlaneJoint,
    PrismaticJoint,
    UniversalJoint,
    WeldJoint,
)
from tests.authoring.test_generic_multibody import (
    _assembly,
    _case,
    _subsystem,
    _template,
)
from tests.authoring.test_unified_subsystem_templates import subsystem


@pytest.mark.parametrize("kind,expected", [("universal", UniversalJoint), ("cylindrical", CylindricalJoint),
    ("inplane", InPlaneJoint), ("prismatic", PrismaticJoint), ("fixed", WeldJoint)])
def test_declared_joint_family_encodes_and_materializes(kind, expected):
    payload = _template(elements=[], property_slots=[])
    payload["joints"][0]["type"] = kind
    if kind == "universal":
        payload["joints"][0]["axis_b"] = [1, 0, 0]
    built = assemble_generic(_assembly({"unit": _subsystem(payload)}))
    row = built.joints[0]
    assert row["type"] == kind
    assert isinstance(GenericSubsystemAssembler.materialize_joint(row), expected)
    assert validate(_assembly({"unit": _subsystem(payload)}), _case()).model_document["joints"] == [row]
    np.testing.assert_array_equal(row["point_a"], [0, 0, 0])


@pytest.mark.parametrize("name", ["rack", "rack_housing", "upright", "arbitrary_body"])
def test_body_names_do_not_create_a_hidden_rack_guide(name):
    payload = _template(bodies=[{"name": name, "mass": 10}], hardpoints=[], joints=[], elements=[], ports=[], property_slots=[])
    built = assemble_generic(_assembly({"unit": subsystem(TemplateDocument.from_payload(payload))}))
    assert set(built.bodies) == {"unit." + name}
    assert built.joints == () and built.coordinates == ()


def test_explicit_compile_blocks_retired_execution_imports(monkeypatch):
    import builtins

    original = builtins.__import__
    def reject(name, *args, **kwargs):
        if name.startswith(("suspension_multibody.preparation", "suspension_multibody.subsystems")):
            raise AssertionError("retired import: " + name)
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, "__import__", reject)
    assert validate(_assembly(), _case()).metadata["compiler"] == "ResolvedModelCompiler"
