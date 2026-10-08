"""Generic assembly membership is data; physical compatibility is port-based."""

import pytest

from suspension_multibody.api import validate
from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    TemplateDocument,
    assemble_generic,
)
from tests.authoring.test_generic_multibody import _assembly, _case, _subsystem
from tests.authoring.test_unified_subsystem_templates import assembly
from tests.subsystems._torque import locked_case, torque_assembly


@pytest.mark.parametrize("role", ["brake", "drive"])
def test_a_single_wheel_bench_accepts_peer_torque_subsystems(role):
    compiled = validate(torque_assembly(role), locked_case())
    assert len(compiled.model_document["tires"]) == 1
    assert len(compiled.model_document["elements"]) == 1


@pytest.mark.parametrize("role", ["generic", "suspension", "chassis", "steering", "wheel", "brake", "drive"])
def test_business_labels_do_not_impose_membership_rules(role):
    original = _subsystem()
    template = original.template.to_payload()
    template["functional_role"] = role
    payload = original.to_payload()
    payload["functional_role"] = role
    declared = SubsystemDocument.from_payload(payload, template=TemplateDocument.from_payload(template), properties=original.properties)
    assembled = assembly({"single": declared})
    assert len(assemble_generic(assembled).bodies) == 2
    assert validate(assembled, _case()).metadata["compiler"] == "ResolvedModelCompiler"


def test_duplicate_instances_are_rejected_even_with_different_business_labels():
    original = _assembly()
    payload = original.to_payload()
    payload["subsystems"] *= 2
    with pytest.raises(ValueError, match="repeats"):
        assemble_generic(AssemblyDocument.from_payload(payload, subsystems={e.ref: e.subsystem for e in original.entries}))
