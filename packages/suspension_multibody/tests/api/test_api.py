"""Public simulation API tests use ordinary model and case documents."""

from pathlib import Path

import numpy as np

from suspension_multibody.api import simulate
from suspension_multibody.authoring.migration import migrate_v1_kc_case
from suspension_multibody.io import read_artifact, write_artifact
from suspension_multibody.schema import Bushing6x6, MassSpec, Pose
from suspension_multibody.schema.model import AxleDeclaration


def kc_model() -> AxleDeclaration:
    return AxleDeclaration(
        hardpoints={
            "uca_front": [-100, -500, 400], "uca_rear": [100, -500, 400],
            "uca_outer": [0, -700, 450], "lca_front": [-120, -500, 150],
            "lca_rear": [120, -500, 150], "lca_outer": [0, -700, 150],
            "tierod_inner": [100, -400, 250], "tierod_outer": [50, -700, 250],
            "wheel_center": [0, -700, 300], "rack_center": [0, 0, 250],
        },
        mass=MassSpec(sprung_mass=1000),
    )


def _c_model() -> AxleDeclaration:
    model = kc_model()
    stiffness = tuple(tuple(10_000.0 if row == column and row < 3 else
        10_000_000.0 if row == column else 0.0 for column in range(6)) for row in range(6))
    return model.model_copy(update={"bushings": tuple(
        Bushing6x6(name=f"{body}_{index}", body_a="chassis", body_b=body,
            pose_a=Pose(translation=model.hardpoints[name]), pose_b=Pose(translation=model.hardpoints[name]),
            stiffness=stiffness)
        for body, names in (("upper_arm", ("uca_front", "uca_rear")), ("lower_arm", ("lca_front", "lca_rear")))
        for index, name in enumerate(names))})


def test_run_case_writes_structured_output(tmp_path: Path) -> None:
    source, case = migrate_v1_kc_case(kc_model(), mode="K")
    run = simulate(source, case)
    loaded = read_artifact(write_artifact(run.result, tmp_path))
    assert loaded["manifest"]["status"] == "success"
    assert len(run.result.cases) == 1
    assert run.result.case_residuals() == (0.0, 0.0, 0.0)


def test_run_case_expands_displacement_controls_and_checkpoints(tmp_path: Path) -> None:
    source, case = migrate_v1_kc_case(kc_model(), mode="K", wheel_values_mm=(-1.0, 1.0), rack_values_mm=(-.5, .5))
    run = simulate(source, case)
    assert len(run.result.cases) == 4
    assert len({row["name"] for row in run.result.cases}) == 4
    stored = read_artifact(write_artifact(run.result, tmp_path / "checkpoint"))
    np.testing.assert_array_equal(stored["arrays"]["body_state"], run.result.named_blocks["body_state"])
    for index in range(4):
        position, force, moment = run.result.case_residuals(index)
        assert 0 <= position < 1e-8
        assert force >= 0 and moment >= 0


def test_run_case_c_retains_physical_response_and_element_loads() -> None:
    source, case = migrate_v1_kc_case(_c_model(), mode="C", paths=("fz",), levels=3, maximum=100)
    payload = case.to_payload()
    payload["excitation"]["c"] = {"loads": [{"fz": 100}], "load_marker": "wheel.sub.json.wheel_center_L"}
    run = simulate(source, payload)
    assert run.status == "success"
    assert run.result.named_blocks["element_wrench"].shape[0] == len(run.result.times_s)
    frame = "wheel.sub.json.wheel_center_L"
    initial = next(row for row in run.result.model.to_document()["frames"] if row["name"] == frame)
    body = next(row for row in run.result.model.to_document()["bodies"] if row["name"] == initial["body"])
    declared_z = body["position"][2] + initial["point"][2]
    assert abs(run.result.frame_pose(frame)[-1, 2, 3]-declared_z) > 1e-6
    bushings = [row["name"] for row in run.compiled.model_document["elements"] if row["type"] == "bushing"]
    assert bushings
    assert any(np.any(run.result.element_state(entity)[:, 6:12]) for entity in bushings)
