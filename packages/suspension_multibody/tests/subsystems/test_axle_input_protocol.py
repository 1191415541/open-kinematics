"""The core accepts document facts without automotive model inheritance."""

import ast
import inspect

from suspension_multibody.api import simulate
from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    TemplateDocument,
    generic,
)
from suspension_multibody.compilation import resolved as compiler
from tests.authoring.test_generic_multibody import _assembly, _case


def test_plain_payloads_reach_the_same_solver():
    source = _assembly()
    documents = {row.ref: SubsystemDocument.from_payload(row.subsystem.to_payload(),
        template=TemplateDocument.from_payload(row.subsystem.template.to_payload()), properties=row.subsystem.properties)
        for row in source.entries}
    loaded = AssemblyDocument.from_payload(source.to_payload(), subsystems=documents)
    first, second = [simulate(row, _case()) for row in (source, loaded)]
    assert first.compiled.model_payload == second.compiled.model_payload
    assert first.result.model_fingerprint == second.result.model_fingerprint


def test_core_modules_do_not_import_automotive_declarations():
    for module in (generic, compiler):
        tree = ast.parse(inspect.getsource(module))
        names = {alias.name for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) for alias in node.names}
        assert not names.intersection({"AxleDeclaration", "VehicleDeclaration", "AxleInput", "VehicleInput"})


def test_role_labels_do_not_select_a_different_graph_builder():
    source = _assembly()
    payload = source.to_payload()
    payload["name"] = "non-automotive-fixture"
    changed = AssemblyDocument.from_payload(payload, subsystems={row.ref: row.subsystem for row in source.entries})
    assert simulate(changed, _case()).result.model_fingerprint == simulate(source, _case()).result.model_fingerprint
