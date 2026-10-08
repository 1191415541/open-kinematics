"""Wheel drive endpoints come from explicit ports, independent of body spelling."""

import pytest

from suspension_multibody.authoring import AssemblyDocument, assemble_generic
from suspension_multibody.authoring.migration import migrate_v1_kc_case
from suspension_multibody.connections.matcher import BindingError
from tests.benchmark_fixture import benchmark_model


def test_the_explicit_port_answers_without_a_body_name_search():
    document, _ = migrate_v1_kc_case(benchmark_model(), mode="K", wheel_values_mm=(0.,), rack_values_mm=(0.,))
    graph = assemble_generic(document)
    rig = next(entry for entry in document.entries if entry.ref.startswith("kc_rig"))
    for side in ("L", "R"):
        port = graph.ports[rig.pairings["wheel_"+side]]
        joint = next(row for row in graph.joints if row.get("target") == rig.ref+".wheel_drive_"+side)
        assert joint["body_a"] == port.owner.local


def test_an_ambiguous_port_requires_an_explicit_pairing():
    document, _ = migrate_v1_kc_case(benchmark_model(), mode="K", wheel_values_mm=(0.,), rack_values_mm=(0.,))
    payload = document.to_payload()
    wheel = next(entry for entry in document.entries if entry.functional_role == "wheel")
    duplicate = {**next(row for row in payload["subsystems"] if row["ref"] == wheel.ref), "ref": "another_wheel"}
    payload["subsystems"].append(duplicate)
    rig = next(row for row in payload["subsystems"] if row["ref"].startswith("kc_rig"))
    rig["pairings"] = [row for row in rig["pairings"] if row["requirement_role"] != "wheel_L"]
    documents = {entry.ref: entry.subsystem for entry in document.entries}
    documents["another_wheel"] = wheel.subsystem
    with pytest.raises(BindingError, match="ambiguous|matches|count|candidate"):
        assemble_generic(AssemblyDocument.from_payload(payload, subsystems=documents))


def test_a_missing_explicit_port_is_refused():
    document, _ = migrate_v1_kc_case(benchmark_model(), mode="K", wheel_values_mm=(0.,), rack_values_mm=(0.,))
    payload = document.to_payload()
    rig = next(row for row in payload["subsystems"] if row["ref"].startswith("kc_rig"))
    rig["pairings"][0]["port"] = "missing.center"
    with pytest.raises(BindingError, match="missing.center"):
        assemble_generic(AssemblyDocument.from_payload(payload, subsystems={entry.ref: entry.subsystem for entry in document.entries}))
