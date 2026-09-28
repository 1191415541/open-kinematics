"""
Provenance on the result side, the rig's link to a registered bench, and the
role field's compatibility during its rename.

Each of these is the last mile of a plan phase: a hash the run records, a file
that names the bench it drives, and a field that had to be renamed without
breaking every template that still spells it the old way.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from suspension_multibody import api
from suspension_multibody.authoring import (
    AuthoringError,
    Project,
    RigDocument,
    SimulationAssembly,
    SubsystemDocument,
)
from suspension_multibody.authoring.solver import front_axle_model_for
from suspension_multibody.schema import CaseSpec
from suspension_multibody.templates import (
    DOUBLE_WISHBONE,
    template_from_json,
    template_to_json,
)

from .fixtures import write_axle_project


def _case() -> CaseSpec:
    return CaseSpec(
        mode="K",
        subsystems=frozenset({"suspension", "chassis", "steering", "wheel"}),
    )


def test_a_file_driven_run_records_the_hashes_of_its_inputs(tmp_path: Path) -> None:
    """
    Acceptance 11: the result carries the hashes of the files that produced it.

    The record goes in a sidecar, so a run that read no documents still writes
    exactly the artifact it always wrote -- which is what keeps every recorded
    baseline byte-for-byte unchanged.
    """
    paths = write_axle_project(tmp_path)
    project = Project.load(tmp_path)
    model = front_axle_model_for(SubsystemDocument.load(paths["subsystem"]))
    out = tmp_path / "run"

    api.run_case(model, _case(), out, inputs=project.provenance("front_axle"))

    recorded = json.loads((out / "inputs.json").read_text(encoding="utf-8"))
    assert recorded["files"]["front.sub.json"] == project.hashes()["front.sub.json"]
    assert recorded["model"]["subsystems"]["front.sub.json"]["topology"]
    manifest = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["inputs_file"] == "inputs.json"


def test_a_python_authored_run_writes_no_inputs_record(tmp_path: Path) -> None:
    """No documents were read, so nothing claims otherwise."""
    paths = write_axle_project(tmp_path)
    model = front_axle_model_for(SubsystemDocument.load(paths["subsystem"]))
    out = tmp_path / "run"
    api.run_case(model, _case(), out)
    assert not (out / "inputs.json").exists()
    manifest = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
    assert "inputs_file" not in manifest


def test_a_rig_file_may_name_a_registered_bench(tmp_path: Path) -> None:
    """
    Phase 6: the file says *which* bench it drives, and the bench stays the code
    that already drives, loads and measures.
    """
    from suspension_multibody.rigs import get_rig

    paths = write_axle_project(tmp_path)
    payload = json.loads(paths["rig"].read_text(encoding="utf-8"))
    payload["bench"] = "kc_quasi_static"
    paths["rig"].write_text(json.dumps(payload), encoding="utf-8")

    document = RigDocument.load(paths["rig"])
    assert document.bench == "kc_quasi_static"
    assert document.bench_spec() is get_rig("kc_quasi_static")


def test_a_rig_naming_an_unknown_bench_is_refused(tmp_path: Path) -> None:
    paths = write_axle_project(tmp_path)
    payload = json.loads(paths["rig"].read_text(encoding="utf-8"))
    payload["bench"] = "no_such_bench"
    paths["rig"].write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(AuthoringError, match="not a registered test bench"):
        RigDocument.load(paths["rig"])


def test_a_rig_without_a_bench_names_none(tmp_path: Path) -> None:
    """A rig that declares capabilities but no bench is legal; it just binds to none."""
    paths = write_axle_project(tmp_path)
    document = RigDocument.load(paths["rig"])
    assert document.bench is None
    assert document.bench_spec() is None


def test_template_role_reads_both_names_and_writes_both() -> None:
    """
    Phase 2: ``role`` becomes ``functional_role`` without breaking its readers.

    The migration is additive: the document carries both names, the reader accepts
    either, and the attribute that says what a template *does* is available under
    the name the file format uses.
    """
    assert DOUBLE_WISHBONE.functional_role == DOUBLE_WISHBONE.role == "suspension"
    payload = template_to_json(DOUBLE_WISHBONE)
    assert payload["functional_role"] == payload["role"] == "suspension"

    without_new_name = {key: value for key, value in payload.items() if key != "functional_role"}
    assert template_from_json(without_new_name).role == "suspension"

    without_old_name = {key: value for key, value in payload.items() if key != "role"}
    assert template_from_json(without_old_name).functional_role == "suspension"


def _rig_payload(paths: dict[str, Path]) -> dict[str, object]:
    """Return the fixture rig, editable before it is written back."""
    return json.loads(paths["rig"].read_text(encoding="utf-8"))


def test_a_rig_declares_the_actuators_its_bench_drives(tmp_path: Path) -> None:
    """
    Phase 6: a rig file's actuators are checked against the bench it names.

    The bench stays the code that drives, loads and measures; the file says which
    of that bench's coordinates it asks for.  A coordinate the bench has no drive
    for is refused by name, because the alternative is a file that reads as if it
    drove something and drives nothing.
    """
    paths = write_axle_project(tmp_path)
    payload = _rig_payload(paths)
    payload["bench"] = "kc_quasi_static"
    payload["actuators"] = ["wheel_drive_L", "rack_drive"]
    paths["rig"].write_text(json.dumps(payload), encoding="utf-8")

    document = RigDocument.load(paths["rig"])
    assert document.actuators == ("wheel_drive_L", "rack_drive")

    payload["actuators"] = ["brake_torque"]
    paths["rig"].write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(AuthoringError, match="not driven by bench"):
        RigDocument.load(paths["rig"])


def test_a_rig_cannot_require_a_port_it_declares_no_support_for(tmp_path: Path) -> None:
    """A support is the rig's own side of an attachment, so a required port needs one."""
    paths = write_axle_project(tmp_path)
    payload = _rig_payload(paths)
    payload["supports"] = [{"name": "chassis_reference", "role": "chassis_reference"}]
    paths["rig"].write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(AuthoringError, match="not declared in supports"):
        RigDocument.load(paths["rig"])

    payload["supports"].append(
        {
            "name": "wheel_centre",
            "role": "wheel_centre",
            "capabilities": ["wheel_supplying"],
        }
    )
    paths["rig"].write_text(json.dumps(payload), encoding="utf-8")
    document = RigDocument.load(paths["rig"])
    assert document.supports == {
        "chassis_reference": (),
        "wheel_centre": ("wheel_supplying",),
    }


def test_optional_ports_are_bound_and_measured_channels_are_declared(
    tmp_path: Path,
) -> None:
    """
    A simulation assembly resolves both halves of the interface.

    The required port is bound because the assembly without it was refused, and
    the optional one is bound because the assembly happens to offer it; the
    channels the rig reports travel with the resolved assembly rather than
    staying in the file unread.
    """
    paths = write_axle_project(tmp_path)
    payload = _rig_payload(paths)
    # The qualified spelling names the role as well as the port, which is how a
    # rig asks for the chassis's reference rather than the suspension's: both
    # templates declare an unqualified `chassis_reference`.
    payload["optional_ports"] = ["chassis:chassis_reference"]
    paths["rig"].write_text(json.dumps(payload), encoding="utf-8")

    simulation = SimulationAssembly.load(paths["assembly"])
    assert simulation.bindings == {
        "wheel_centre": "front.sub.json",
        "chassis:chassis_reference": "chassis.sub.json",
    }
    assert simulation.channels == ("wheel_travel", "camber", "toe")
    assert simulation.rig.optional_ports == frozenset({"chassis:chassis_reference"})
