"""Each pad step reports the wheel center and the native contact-law facts."""

import numpy as np
import pytest

from suspension_multibody.api import simulate
from suspension_multibody.authoring import AssemblyDocument, SubsystemDocument
from tests.simulation.test_quasi_static_tire import _source

PAD_HEIGHTS_MM = (0., 10., 20.)


def _pad_run(*, active=True):
    source = _source()
    entries = {row.ref: row.subsystem for row in source.entries}
    subsystems = {}
    for side, y in (("left", -.75), ("right", .75)):
        for ref, subsystem in entries.items():
            payload = subsystem.to_payload()
            payload["name"] += "_" + side
            payload["hardpoints"]["center"] = [0, y, .334]
            subsystems[ref+"_"+side] = SubsystemDocument.from_payload(payload,
                template=subsystem.template, properties=subsystem.properties)
    payload = source.to_payload()
    payload["subsystems"] = [dict(row, ref=row["ref"]+"_"+side)
        for side in ("left", "right") for row in payload["subsystems"]]
    for row in payload["subsystems"]:
        if row["ref"].startswith("wheel_"):
            side = row["ref"].removeprefix("wheel_")
            row["pairings"] = [{"requirement_role": "carrier", "port": "support_"+side+".carrier"},
                {"requirement_role": "road", "port": "support_"+side+".road"}]
    assembly = AssemblyDocument.from_payload(payload, subsystems=subsystems)
    case = {"schema_version": 1, "name": "pad", "study": "quasi_static", "protocol": "kc_quasi_static",
        "samples": [0, .001], "inputs": [], "outputs": [],
        "solver": {"initialization_mode": "provided_consistent_state"},
        "boundaries": [{"name": "hold_"+side, "coordinate": "wheel_"+side+".spin",
            "mode": "locked", "units": "rad", "value": 0} for side in ("left", "right")],
        "element_activation": [{"entity": "wheel_"+side+".tire", "active": active} for side in ("left", "right")],
        "excitation": {"drive_mode": "pad", "k": {"pad_height_mm": list(PAD_HEIGHTS_MM)}} if active else
            {"c": {"load_marker": "wheel_left.hub", "loads": [{"fz": 0}]}}}
    return simulate(assembly, case).result


def _outputs(result, index):
    if not result.tire_ids:
        return {}
    row = result.cases[index]
    sample = int(row["sample_offset"])+int(row["sample_count"])-1
    outputs = {}
    for side in ("left", "right"):
        state = result.tire_state("wheel_"+side+".tire")[sample]
        center = result.frame_pose("wheel_"+side+".center")[sample, :3, 3]
        contact = center - [0, 0, .334-state[2]]
        outputs[side] = {**{f"wheel_center_{axis}_mm": float(center[i]*1000) for i, axis in enumerate("xyz")},
            **{f"contact_{axis}_mm": float(contact[i]*1000) for i, axis in enumerate("xyz")},
            "tire_load_n": float(state[4])}
    return outputs


def test_every_pad_height_reports_its_three_outputs():
    result = _pad_run()
    assert len(result.cases) == len(PAD_HEIGHTS_MM)
    for index in range(len(PAD_HEIGHTS_MM)):
        outputs = _outputs(result, index)
        assert set(outputs) == {"left", "right"}
        for values in outputs.values():
            assert set(values) == {"wheel_center_x_mm", "wheel_center_y_mm", "wheel_center_z_mm",
                "tire_load_n", "contact_x_mm", "contact_y_mm", "contact_z_mm"}
            assert all(np.isfinite(value) for value in values.values())


def test_the_contact_point_lies_on_the_pad():
    result = _pad_run()
    for index, height in enumerate(PAD_HEIGHTS_MM):
        for values in _outputs(result, index).values():
            assert values["contact_z_mm"] == pytest.approx(height, abs=1e-9)


def test_the_tire_load_follows_the_compression():
    result = _pad_run()
    loads = [_outputs(result, index)["left"]["tire_load_n"] for index in range(3)]
    assert all(later > earlier for earlier, later in zip(loads, loads[1:]))
    for index, height in enumerate(PAD_HEIGHTS_MM):
        for side in ("left", "right"):
            row = result.cases[index]
            sample = int(row["sample_offset"])+int(row["sample_count"])-1
            compression = result.tire_state("wheel_"+side+".tire")[sample, 2]
            assert compression == pytest.approx(height/1000, abs=1e-12)
            assert _outputs(result, index)[side]["tire_load_n"] == pytest.approx(200_000*compression, rel=1e-9, abs=1e-9)


def test_a_run_without_tires_reports_nothing_rather_than_zero():
    result = _pad_run(active=False)
    assert result.tire_ids == ()
    assert "tire_output" not in result.named_blocks
    assert _outputs(result, 0) == {}
