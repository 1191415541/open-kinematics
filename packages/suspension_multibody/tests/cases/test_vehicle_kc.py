"""An ordinary vehicle K/C rig preserves zero pose and exact driven travel."""

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring.migration import migrate_v1_vehicle_kc_case
from suspension_multibody.modeling.primitives import quaternion_to_matrix
from suspension_multibody.schema import Bushing6x6, Pose
from tests.vehicle.vehicle_fixtures import _case, _positioned_vehicle, _vehicle

_TIMES = tuple(np.linspace(0., .02, 21).tolist())
_STIFFNESS = tuple(tuple(10_000. if r == c and r < 3 else 10_000_000. if r == c else 0.
    for c in range(6)) for r in range(6))


def _compliant(axle):
    bushings = tuple(Bushing6x6(name=f"{body}_{index}", body_a="chassis", body_b=body,
        pose_a=Pose(translation=axle.hardpoints[point]), pose_b=Pose(translation=axle.hardpoints[point]),
        stiffness=_STIFFNESS) for body, names in (
            ("upper_arm", ("UPPER_INBOARD_FRONT", "UPPER_INBOARD_REAR")),
            ("lower_arm", ("LOWER_INBOARD_FRONT", "LOWER_INBOARD_REAR")))
        for index, point in enumerate(names))
    return axle.model_copy(update={"bushings": bushings})


@pytest.fixture(scope="module")
def prepared():
    rigid = _positioned_vehicle(_vehicle())
    model = rigid.model_copy(update={"front_axle": _compliant(rigid.front_axle), "rear_axle": _compliant(rigid.rear_axle)})
    return _case(model)


def _documents(source, *, wheels=(0.,), mode="symmetric"):
    return migrate_v1_vehicle_kc_case(source, wheel_values_mm=wheels,
        rack_values_mm=(0.,), times_s=_TIMES, left_right_mode=mode)


def _run(source, **kwargs):
    assembly, case = _documents(source, **kwargs)
    run = simulate(assembly, case)
    return run.raw, run.compiled.model_document


def _driven_value(raw, model, target):
    joint = next(row for row in model["joints"] if row.get("target") == target)
    bodies = list(raw.document["manifest"]["bodies"])
    state = raw.block("body_state")[-1]
    a, b = (state[bodies.index(joint["body_"+end])] for end in ("a", "b"))
    ra, rb = quaternion_to_matrix(a[3:7]), quaternion_to_matrix(b[3:7])
    return float(np.dot(a[:3]+ra@joint["point_a"]-b[:3]-rb@joint["point_b"], rb@joint["axis_b"]))


def test_a_zero_sweep_returns_the_assembling_pose(prepared):
    raw, model = _run(prepared)
    names = list(raw.document["manifest"]["bodies"])
    for body in model["bodies"]:
        np.testing.assert_allclose(raw.block("body_state")[-1, names.index(body["name"]), :3], body["position"], rtol=0, atol=1e-9)


def test_a_symmetric_bump_advances_every_wheel_drive_by_the_travel(prepared):
    zero, model = _run(prepared)
    bump, _ = _run(prepared, wheels=(10.,))
    targets = [row["target"] for row in model["joints"] if row["type"] == "driven_translation" and "wheel_drive" in row["target"]]
    assert len(targets) == 4
    for target in targets:
        assert _driven_value(bump, model, target)-_driven_value(zero, model, target) == pytest.approx(.010, abs=1e-6)


def test_the_mode_redistributes_the_same_travel(prepared):
    zero, model = _run(prepared)
    opposite, _ = _run(prepared, wheels=(10.,), mode="opposite")
    for row in model["joints"]:
        if row["type"] != "driven_translation" or "wheel_drive" not in row["target"]:
            continue
        sign = 1 if row["target"].endswith("left") else -1
        assert _driven_value(opposite, model, row["target"])-_driven_value(zero, model, row["target"]) == pytest.approx(sign*.010, abs=1e-6)


def test_an_unknown_mode_is_refused(prepared):
    with pytest.raises(Exception, match="left_right_mode"):
        validate(*_documents(prepared, mode="diagonal"))


def test_the_declared_rig_contributes_one_drive_per_wheel_and_the_rack(prepared):
    assembly, case = _documents(prepared)
    compiled = validate(assembly, case)
    driven = [row for row in compiled.model_document["joints"] if row["type"] == "driven_translation"]
    assert len(driven) == 5
    assert compiled.case_document["k"] == case.to_payload()["excitation"]["k"]
    assert compiled.metadata["compiler"] == "ResolvedModelCompiler"


def test_file_and_python_vehicle_kc_inputs_have_one_compiler(prepared, tmp_path):
    assembly, case = _documents(prepared)
    path = case.save(tmp_path/"case.json")
    memory, file = validate(assembly, case), validate(assembly, path)
    assert memory.model_payload == file.model_payload
    assert memory.case_payload == file.case_payload
    wrong = case.to_payload()
    wrong["protocol"] = "handling"
    with pytest.raises(ValueError, match="time-history protocol"):
        validate(assembly, wrong)
