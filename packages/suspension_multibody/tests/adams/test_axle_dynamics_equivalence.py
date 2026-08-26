from __future__ import annotations

import json
import re
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.adams import (
    AxleChannelBindings,
    AxleContactEvent,
    AxleInitializationEvidence,
    AxleMarkerBinding,
    TimeHistory,
    adams_axle_raw_channel_map,
    audit_axle_equivalence,
    axle_adams_blockers,
    axle_history_from_result,
    build_axle_adams_dataset,
    compare_axle_evidence,
    compare_strict_axle_histories,
    create_dynamic_axle_manifest,
    initialization_evidence_from_result,
    load_axle_acceptance_contract,
    load_axle_channel_contract,
    read_axle_evidence_bundle,
    read_dynamic_axle_manifest,
    run_native_axle_manifest,
    write_axle_adams_dataset,
    write_axle_evidence_bundle,
    write_dynamic_axle_manifest,
)
from suspension_multibody.axle_dynamics import (
    AxleAntiRollBar,
    AxleBody,
    AxleBushing,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleDynamicsResult,
    AxleJoint,
    AxleRunDiagnostics,
    AxleSolverSettings,
    AxleSpringDamper,
    AxleTire,
)
from suspension_multibody.io import canonical_hash


def _model() -> AxleDynamicsModel:
    inertia = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    return AxleDynamicsModel(
        name="equivalence-fixture",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=inertia,
                fixed=True,
            ),
            AxleBody(
                name="sprung",
                mass_kg=100.0,
                inertia_kg_m2=inertia,
                position_m=(0.0, 0.0, 0.5),
            ),
            AxleBody(
                name="wheel_l",
                mass_kg=10.0,
                inertia_kg_m2=inertia,
                position_m=(0.0, -0.7, 0.3),
            ),
            AxleBody(
                name="wheel_r",
                mass_kg=10.0,
                inertia_kg_m2=inertia,
                position_m=(0.0, 0.7, 0.3),
            ),
        ),
        joints=(
            AxleJoint(
                name="spin_l",
                kind="revolute",
                body_a="fixture",
                body_b="wheel_l",
                point_a_m=(0.0, -0.7, 0.3),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 1.0, 0.0),
                axis_b=(0.0, 1.0, 0.0),
            ),
            AxleJoint(
                name="spin_r",
                kind="revolute",
                body_a="fixture",
                body_b="wheel_r",
                point_a_m=(0.0, 0.7, 0.3),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 1.0, 0.0),
                axis_b=(0.0, 1.0, 0.0),
            ),
        ),
        springs=(
            AxleSpringDamper(
                name="spring_l",
                body_a="fixture",
                body_b="wheel_l",
                point_a_m=(0.0, -0.7, 0.6),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=10_000.0,
                compression_damping_n_s_per_m=100.0,
                rebound_damping_n_s_per_m=100.0,
                free_length_m=0.3,
            ),
            AxleSpringDamper(
                name="spring_r",
                body_a="fixture",
                body_b="wheel_r",
                point_a_m=(0.0, 0.7, 0.6),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=10_000.0,
                compression_damping_n_s_per_m=100.0,
                rebound_damping_n_s_per_m=100.0,
                free_length_m=0.3,
            ),
        ),
        tires=(
            _tire("tire_l", "wheel_l"),
            _tire("tire_r", "wheel_r"),
        ),
    )


def _tire(name: str, body: str) -> AxleTire:
    return AxleTire(
        name=name,
        body=body,
        unloaded_radius_m=0.3,
        maximum_compression_m=0.05,
        vertical_stiffness_n_per_m=100_000.0,
        vertical_damping_n_s_per_m=100.0,
        longitudinal_friction_coefficient=1.0,
        lateral_friction_coefficient=1.0,
        longitudinal_brush_stiffness_n_per_m=100_000.0,
        lateral_brush_stiffness_n_per_m=100_000.0,
        longitudinal_relaxation_length_m=0.2,
        lateral_relaxation_length_m=0.2,
        detached_relaxation_s=0.02,
    )


def _bindings() -> AxleChannelBindings:
    return AxleChannelBindings(
        sprung_body="sprung",
        fixture_reference_marker=AxleMarkerBinding(
            body="fixture",
            point_local_m=(0.0, 0.0, 0.0),
        ),
        left_wheel_center_marker=AxleMarkerBinding(
            body="wheel_l",
            point_local_m=(0.0, 0.0, 0.0),
        ),
        right_wheel_center_marker=AxleMarkerBinding(
            body="wheel_r",
            point_local_m=(0.0, 0.0, 0.0),
        ),
        left_wheel_spin_joint="spin_l",
        right_wheel_spin_joint="spin_r",
        left_spring="spring_l",
        right_spring="spring_r",
        left_damper="spring_l",
        right_damper="spring_r",
        left_tire="tire_l",
        right_tire="tire_r",
    )


def _case(
    name: str = "road_step_finite_rise",
    *,
    duration_s: float = 0.002,
) -> AxleDynamicsCase:
    count = int(round(duration_s * 1000.0)) + 1
    return AxleDynamicsCase(
        name=name,
        times_s=tuple(index * 0.001 for index in range(count)),
        solver=AxleSolverSettings(
            adaptive_step=False,
            internal_step_s=0.00025,
        ),
    )


def _manifest(
    case: AxleDynamicsCase | None = None,
    *,
    case_metadata: dict[str, object] | None = None,
):
    return create_dynamic_axle_manifest(
        _model(),
        case or _case(),
        _bindings(),
        adams_solver={
            "integrator": "HHT",
            "alpha": -0.3,
            "error": 1e-8,
            "maximum_step_s": 0.00025,
        },
        execution_environment={
            "cpu_model": "fixture-cpu",
            "physical_core_count": 8,
            "thread_count": 1,
            "process_affinity": "0",
        },
        case_metadata=case_metadata,
    )


def _result() -> AxleDynamicsResult:
    times = np.asarray((0.0, 0.001, 0.002))
    states = np.zeros((3, 4, 19))
    states[:, :, 3] = 1.0
    states[:, 1, 2] = (0.5, 0.501, 0.502)
    states[:, 1, 9] = (0.0, 1.0, 1.0)
    states[:, 1, 15] = (0.0, 0.0, 0.0)
    states[:, 2, :3] = np.asarray(
        ((0.0, -0.7, 0.3), (0.0, -0.7, 0.301), (0.0, -0.7, 0.302))
    )
    states[:, 3, :3] = np.asarray(
        ((0.0, 0.7, 0.3), (0.0, 0.7, 0.299), (0.0, 0.7, 0.298))
    )
    states[:, 2, 11] = (1.0, 2.0, 3.0)
    states[:, 3, 11] = (-1.0, -2.0, -3.0)
    constraint = np.zeros((3, 2, 6))
    constraint[:, 0, 2] = 100.0
    constraint[:, 1, 2] = 120.0
    spring = np.zeros((3, 2, 7))
    spring[:, :, 0] = 0.3
    spring[:, 0, 2] = 50.0
    spring[:, 1, 2] = 60.0
    spring[:, 0, 3] = 5.0
    spring[:, 1, 3] = 6.0
    spring[:, 0, 4] = 2.0
    spring[:, 1, 5] = -3.0
    spring[:, 0, 6] = 57.0
    spring[:, 1, 6] = 63.0
    tire = np.zeros((3, 2, 12))
    tire[:, :, 0] = 1.0
    tire[:, 0, 4:7] = (1000.0, 10.0, 20.0)
    tire[:, 1, 4:7] = (1100.0, 11.0, 21.0)
    zeros = np.zeros(3)
    return AxleDynamicsResult(
        times_s=times,
        body_names=("fixture", "sprung", "wheel_l", "wheel_r"),
        constraint_names=("spin_l", "spin_r"),
        spring_names=("spring_l", "spring_r"),
        bushing_names=(),
        anti_roll_bar_names=(),
        tire_names=("tire_l", "tire_r"),
        states=states,
        constraint_wrench=constraint,
        spring_output=spring,
        bushing_output=np.zeros((3, 0, 12)),
        anti_roll_output=np.zeros((3, 0, 3)),
        diagnostics=AxleRunDiagnostics(
            accepted=np.ones(3),
            internal_steps=np.ones(3),
            rejected_attempts=zeros,
            newton_iterations=np.ones(3),
            minimum_accepted_step_s=np.full(3, 0.00025),
            maximum_accepted_step_s=np.full(3, 0.00025),
            last_accepted_step_s=np.full(3, 0.00025),
            position_residual=zeros,
            velocity_residual=zeros,
            dynamics_residual=zeros,
            active_contacts=np.full(3, 2.0),
            contact_events=zeros,
            local_error_ratio=zeros,
            energy_residual=zeros,
            failure_code=zeros,
            pinned_null_directions=zeros,
        ),
        tire_output=tire,
        energy=np.zeros((3, 14)),
    )


def _zero_history(case: AxleDynamicsCase) -> TimeHistory:
    contract = load_axle_channel_contract()
    return TimeHistory(
        time=case.times_s,
        channels={
            name: tuple(0.0 for _ in case.times_s)
            for name in contract["channels"]
        },
        units={
            name: values["unit"]
            for name, values in contract["channels"].items()
        },
    )


def _initialization(translation_x: float = 0.0) -> AxleInitializationEvidence:
    state = {"q": [translation_x, 0.0, 0.0]}
    return AxleInitializationEvidence(
        translations_m={"sprung": (translation_x, 0.0, 0.0)},
        rotation_vectors_rad={"sprung": (0.0, 0.0, 0.0)},
        wheel_loads_n={"left": 1000.0, "right": 1000.0},
        component_forces_n={"spring": 100.0},
        component_moments_n_m={"joint": 10.0},
        constraint_position_max_m=0.0,
        constraint_velocity_max_m_per_s=0.0,
        state=state,
        state_sha256=canonical_hash(state),
    )


def _diagnostics() -> dict[str, object]:
    return {
        "run_completed": True,
        "solver_internal_gates_passed": True,
        "energy_gate_passed": True,
        "time_convergence_passed": True,
    }


def _raw_adams(directory: Path) -> list[Path]:
    raw = directory / "raw"
    raw.mkdir(parents=True)
    paths = []
    for suffix in (".adm", ".cmd", ".msg", ".res"):
        path = raw / f"case{suffix}"
        path.write_text("independent Adams fixture", encoding="ascii")
        paths.append(path)
    return paths


def _raw_native(directory: Path) -> list[Path]:
    directory.mkdir(parents=True, exist_ok=True)
    paths = [directory / "manifest.json", directory / "arrays.npz"]
    paths[0].write_text("native manifest fixture", encoding="ascii")
    paths[1].write_bytes(b"native arrays fixture")
    return paths


def test_frozen_contract_has_33_channels_and_no_interpolation() -> None:
    acceptance = load_axle_acceptance_contract()
    channels = load_axle_channel_contract()

    assert len(acceptance["core_channels"]) == 33
    assert tuple(acceptance["core_channels"]) == tuple(channels["channels"])
    assert acceptance["comparison"]["interpolation"] == "none"


def test_manifest_hashes_every_shared_input_and_round_trips(tmp_path: Path) -> None:
    manifest = _manifest()
    path = write_dynamic_axle_manifest(manifest, tmp_path / "manifest.json")

    loaded = read_dynamic_axle_manifest(path)
    changed = _manifest(
        AxleDynamicsCase(
            **{
                **_case().model_dump(),
                "road_height_m": {
                    "tire_l": (0.0, 0.001, 0.0),
                },
            }
        )
    )

    assert loaded.sha256 == manifest.sha256
    assert loaded.bindings == _bindings()
    assert changed.sha256 != manifest.sha256


def test_manifest_rejects_cross_side_component_binding() -> None:
    invalid = AxleChannelBindings(
        **{
            **_bindings().model_dump(),
            "left_spring": "spring_r",
        }
    )

    with pytest.raises(
        ValueError,
        match="left_spring and right_spring must be different",
    ):
        create_dynamic_axle_manifest(
            _model(),
            _case(),
            invalid,
            adams_solver={
                "integrator": "HHT",
                "alpha": -0.3,
                "error": 1e-8,
                "maximum_step_s": 0.00025,
            },
            execution_environment={
                "cpu_model": "fixture-cpu",
                "physical_core_count": 8,
                "thread_count": 1,
                "process_affinity": "0",
            },
        )


def test_manifest_rejects_changed_component_hash(tmp_path: Path) -> None:
    manifest = _manifest()
    payload = manifest.as_dict()
    payload["component_hashes"]["case_sha256"] = "0" * 64  # type: ignore[index]
    content = dict(payload)
    content.pop("manifest_sha256")
    payload["manifest_sha256"] = canonical_hash(content)
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="component hashes"):
        read_dynamic_axle_manifest(path)


def test_native_result_exports_all_frozen_physical_channels() -> None:
    history = axle_history_from_result(_model(), _result(), _bindings())

    assert tuple(history.channels) == tuple(
        load_axle_acceptance_contract()["core_channels"]
    )
    assert history.channels["sprung_body.heave"] == pytest.approx(
        (0.0, 0.001, 0.002)
    )
    assert history.channels["left.wheel_spin"] == pytest.approx((1.0, 2.0, 3.0))
    assert history.channels["right.wheel_spin"] == pytest.approx(
        (-1.0, -2.0, -3.0)
    )
    assert history.channels["left.spring_force"] == (52.0, 52.0, 52.0)
    assert history.channels["right.damper_force"] == (6.0, 6.0, 6.0)


def test_fixture_history_uses_common_momentum_balance() -> None:
    result = _result()
    states = result.states.copy()
    states[:, :, :3] = states[0:1, :, :3]
    states[:, :, 7:13] = 0.0
    result = replace(
        result,
        states=states,
        tire_output=np.zeros_like(result.tire_output),
    )

    history = axle_history_from_result(
        _model(),
        result,
        _bindings(),
        case=_case(),
    )

    for name in (
        "fixture.force_x",
        "fixture.force_y",
        "fixture.force_z",
        "fixture.moment_x",
        "fixture.moment_y",
        "fixture.moment_z",
    ):
        assert history.channels[name] == pytest.approx((0.0, 0.0, 0.0))


def test_native_initialization_evidence_contains_complete_hashed_state() -> None:
    evidence = initialization_evidence_from_result(
        _model(),
        _result(),
        _bindings(),
    )

    assert evidence.state_sha256 == canonical_hash(evidence.state)
    assert set(evidence.translations_m) == {
        "fixture",
        "sprung",
        "wheel_l",
        "wheel_r",
    }
    assert evidence.wheel_loads_n == {"left": 1000.0, "right": 1100.0}
    assert "constraint:spin_l:force_z" in evidence.component_forces_n


def test_strict_comparison_rejects_any_time_grid_difference() -> None:
    reference = _zero_history(_case())
    candidate = TimeHistory(
        time=(0.0, 0.0011, 0.002),
        channels=reference.channels,
        units=reference.units,
    )

    with pytest.raises(ValueError, match="interpolation is forbidden"):
        compare_strict_axle_histories(
            reference,
            candidate,
            case_name="road_step_finite_rise",
        )


def test_harmonic_gate_uses_least_squares_amplitude_and_phase() -> None:
    frequency_hz = 5.0
    case = _case("road_sine", duration_s=4.0)
    contract = load_axle_channel_contract()
    time = np.asarray(case.times_s)
    reference_signal = np.sin(2.0 * np.pi * frequency_hz * time)
    candidate_signal = np.sin(
        2.0 * np.pi * frequency_hz * time + np.deg2rad(3.0)
    )
    reference = TimeHistory(
        time=case.times_s,
        channels={
            name: tuple(reference_signal)
            for name in contract["channels"]
        },
        units={
            name: values["unit"]
            for name, values in contract["channels"].items()
        },
    )
    candidate = TimeHistory(
        time=case.times_s,
        channels={
            name: tuple(candidate_signal)
            for name in contract["channels"]
        },
        units=reference.units,
    )

    report = compare_strict_axle_histories(
        reference,
        candidate,
        case_name="road_sine",
        harmonic_frequency_hz=frequency_hz,
    )

    harmonic = report["harmonic"]
    assert isinstance(harmonic, dict)
    assert harmonic["channels"]["sprung_body.heave"]["phase_absolute_error_deg"] == (
        pytest.approx(3.0, abs=1e-9)
    )
    assert not harmonic["passed"]


def test_contact_event_gate_uses_internal_event_times() -> None:
    case = _case("tire_liftoff_and_recontact", duration_s=0.02)
    history = _zero_history(case)
    report = compare_strict_axle_histories(
        history,
        history,
        case_name=case.name,
        reference_events=(
            AxleContactEvent(tire="tire_l", transition="exit", time_s=0.0073),
            AxleContactEvent(tire="tire_l", transition="enter", time_s=0.0124),
        ),
        candidate_events=(
            AxleContactEvent(tire="tire_l", transition="exit", time_s=0.00735),
            AxleContactEvent(tire="tire_l", transition="enter", time_s=0.01245),
        ),
    )

    assert report["contact_events"]["passed"]  # type: ignore[index]


def test_evidence_requires_raw_files_and_detects_hash_changes(tmp_path: Path) -> None:
    manifest = _manifest()
    directory = tmp_path / "adams"
    raw = _raw_adams(directory)
    evidence_path = write_axle_evidence_bundle(
        output_dir=directory,
        manifest=manifest,
        producer_id="adams-2024.1-case",
        producer_kind="msc.adams",
        history=_zero_history(manifest.case),
        initialization=_initialization(),
        contact_events=(),
        diagnostics=_diagnostics(),
        raw_artifacts=raw,
    )
    read_axle_evidence_bundle(evidence_path, manifest=manifest)
    raw[-1].write_text("changed", encoding="ascii")

    with pytest.raises(ValueError, match="hash changed"):
        read_axle_evidence_bundle(evidence_path, manifest=manifest)


def test_initialization_failure_blocks_transient_comparison(tmp_path: Path) -> None:
    manifest = _manifest()
    manifest_path = write_dynamic_axle_manifest(
        manifest,
        tmp_path / "dynamic_axle_manifest.json",
    )
    adams_dir = tmp_path / "adams"
    native_dir = tmp_path / "native"
    adams_path = write_axle_evidence_bundle(
        output_dir=adams_dir,
        manifest=manifest,
        producer_id="adams-independent",
        producer_kind="msc.adams",
        history=_zero_history(manifest.case),
        initialization=_initialization(),
        contact_events=(),
        diagnostics=_diagnostics(),
        raw_artifacts=_raw_adams(adams_dir),
    )
    native_path = write_axle_evidence_bundle(
        output_dir=native_dir,
        manifest=manifest,
        producer_id="native-independent",
        producer_kind="open-kinematics.native",
        history=_zero_history(manifest.case),
        initialization=_initialization(0.001),
        contact_events=(),
        diagnostics=_diagnostics(),
        raw_artifacts=_raw_native(native_dir),
    )

    report = compare_axle_evidence(
        manifest_path=manifest_path,
        adams_evidence_path=adams_path,
        native_evidence_path=native_path,
    )

    assert report["status"] == "BLOCKED"
    assert not report["transient_comparison_performed"]
    assert report["failure_attribution"] == [
        "initialization_or_parameter_mismatch"
    ]


def test_native_runner_writes_convergence_and_event_evidence(tmp_path: Path) -> None:
    manifest = _manifest()
    manifest_path = write_dynamic_axle_manifest(
        manifest,
        tmp_path / "dynamic_axle_manifest.json",
    )

    evidence_path = run_native_axle_manifest(
        manifest_path,
        tmp_path / "native",
        producer_id="native-fixture-runner",
    )
    evidence = read_axle_evidence_bundle(evidence_path, manifest=manifest)

    assert evidence.producer_kind == "open-kinematics.native"
    assert evidence.diagnostics["run_completed"] is True
    assert "time_convergence" in evidence.diagnostics
    assert (tmp_path / "native" / "native_result" / "arrays.npz").is_file()


def _evaluate_adams_expression(expression: str, time: float) -> float:
    """Evaluate a generated ramp-sum expression with Python arithmetic."""
    python = expression.replace("MAX(", "max(").replace("TIME", "t")
    if re.search(r"[A-Za-z_]+\(", python.replace("max(", "")):
        raise AssertionError(f"expression is not a plain ramp sum: {expression}")
    return float(eval(python, {"__builtins__": {}}, {"max": max, "t": time}))


def test_both_models_come_from_one_manifest_and_generation_is_deterministic() -> None:
    manifest = _manifest()

    dataset = build_axle_adams_dataset(manifest)
    repeated = build_axle_adams_dataset(manifest)

    assert dataset.model_text == repeated.model_text
    assert dataset.as_dict() == repeated.as_dict()
    assert manifest.sha256 in dataset.model_text
    assert manifest.sha256 in dataset.command_text
    for body in manifest.model.bodies:
        if body.fixed:
            continue
        assert f"! body {body.name}" in dataset.model_text
        assert f"body:{body.name}:cm" in dataset.entity_ids
    for joint in manifest.model.joints:
        assert f"joint:{joint.name}" in dataset.entity_ids
    for spring in manifest.model.springs:
        assert f"spring:{spring.name}" in dataset.entity_ids
    for tire in manifest.model.tires:
        assert f"tire:{tire.name}:gforce" in dataset.entity_ids
        assert f"tire:{tire.name}:brush_x" in dataset.entity_ids
    statements = {
        line.split("/", 1)[0]
        for line in dataset.model_text.splitlines()
        if line and not line.startswith(("!", ",", " "))
    }
    assert statements <= {
        "ADAMS",
        "UNITS",
        "PART",
        "MARKER",
        "JOINT",
        "SFORCE",
        "VARIABLE",
        "DIFF",
        "GFORCE",
        "ACCGRAV",
        "REQUEST",
        "OUTPUT",
        "RESULTS",
        "END",
    }
    assert "USER(" not in dataset.model_text
    assert "integrator/hht, alpha = -0.3" in dataset.command_text
    assert "ROUTINE" not in dataset.model_text


def test_hht_manifest_requires_explicit_alpha() -> None:
    with pytest.raises(ValueError, match="adams_solver.*alpha"):
        create_dynamic_axle_manifest(
            _model(),
            _case(),
            _bindings(),
            adams_solver={
                "integrator": "HHT",
                "error": 1e-8,
                "maximum_step_s": 0.00025,
            },
            execution_environment={
                "cpu_model": "fixture-cpu",
                "physical_core_count": 8,
                "thread_count": 1,
                "process_affinity": "0",
            },
        )
def test_generated_dataset_requests_every_channel_input() -> None:
    dataset = build_axle_adams_dataset(_manifest())

    keys = {str(request["key"]) for request in dataset.requests}
    assert keys == {
        "body:sprung:pose",
        "body:sprung:rate",
        "body:sprung:acceleration",
        "body:wheel_l:pose",
        "body:wheel_l:rate",
        "body:wheel_l:acceleration",
        "body:wheel_r:pose",
        "body:wheel_r:rate",
        "body:wheel_r:acceleration",
        "joint:spin_l:wrench",
        "joint:spin_r:wrench",
        "spring:spring_l:output",
        "spring:spring_r:output",
        "tire:tire_l:normal",
        "tire:tire_l:tangential",
        "tire:tire_r:normal",
        "tire:tire_r:tangential",
    }
    spring = next(
        request
        for request in dataset.requests
        if request["key"] == "spring:spring_l:output"
    )
    assert spring["components"] == [
        "length",
        "length_rate",
        "elastic_force",
        "damping_force",
        "compression_stop_elastic_force",
        "rebound_stop_elastic_force",
        "total_axial_force",
    ]


def test_tire_brush_raw_channels_use_diff_entities() -> None:
    manifest = _manifest()
    dataset = build_axle_adams_dataset(manifest)

    channels = adams_axle_raw_channel_map(manifest.model, dataset)

    assert channels["tire:tire_l:brush_x"].entity.startswith("DIFF_")
    assert channels["tire:tire_l:brush_y"].entity.startswith("DIFF_")
    assert channels["tire:tire_l:normal_force"].entity.startswith("VARIABLE_")


def test_road_input_is_an_exact_piecewise_linear_ramp_sum() -> None:
    case = AxleDynamicsCase(
        **{
            **_case(duration_s=0.01).model_dump(),
            "road_height_m": {
                "tire_l": (
                    0.0,
                    0.0,
                    0.0,
                    0.005,
                    0.01,
                    0.01,
                    0.01,
                    0.01,
                    0.01,
                    0.01,
                    0.01,
                ),
            },
        }
    )
    dataset = build_axle_adams_dataset(_manifest(case))

    expression = _road_height_function(dataset.model_text, "tire_l")
    for time, expected in zip(case.times_s, case.road_height_m["tire_l"]):
        assert _evaluate_adams_expression(expression, time) == pytest.approx(
            expected, abs=1e-15
        )
    assert _evaluate_adams_expression(expression, 0.0025) == pytest.approx(
        0.0025, abs=1e-15
    )
    assert _evaluate_adams_expression(expression, 0.05) == pytest.approx(
        0.01, abs=1e-15
    )


def test_measured_damper_curve_is_emitted_without_constant_fit() -> None:
    spring = _model().springs[0].model_copy(
        update={
            "damper_curve_velocity_m_per_s": (-1.0, 0.0, 1.0),
            "damper_curve_force_n": (120.0, 10.0, -160.0),
        }
    )
    curve_model = AxleDynamicsModel(
        **{
            **_model().model_dump(),
            "springs": (spring, _model().springs[1]),
        }
    )
    manifest = create_dynamic_axle_manifest(
        curve_model,
        _case(),
        _bindings(),
        adams_solver={
            "integrator": "HHT",
            "alpha": -0.3,
            "error": 1e-8,
            "maximum_step_s": 0.00025,
        },
        execution_environment={
            "cpu_model": "fixture-cpu",
            "physical_core_count": 8,
            "thread_count": 1,
            "process_affinity": "0",
        },
    )

    dataset = build_axle_adams_dataset(manifest)
    curve_id = dataset.entity_ids["spring:spring_l:damper_curve"]

    assert "piecewise-linear damper curve for spring_l" in dataset.model_text
    assert "MAX(0, (VR(" in dataset.model_text
    assert "))--" not in dataset.model_text
    assert f"VARVAL({curve_id})" in dataset.model_text
    assert dataset.conventions["damper_curve_interpolation"]
    curve_line = next(
        line
        for line in dataset.model_text.splitlines()
        if line.startswith(", FUNCTION = ") and "MAX(0, (VR(" in line
    )
    expression = re.sub(
        r"VR\(\d+, \d+\)",
        "TIME",
        curve_line[len(", FUNCTION = ") :],
    )
    for velocity, force in zip(
        spring.damper_curve_velocity_m_per_s,
        spring.damper_curve_force_n,
    ):
        assert _evaluate_adams_expression(expression, velocity) == pytest.approx(
            force, abs=1e-9
        )
    assert _evaluate_adams_expression(expression, -2.0) == pytest.approx(
        spring.damper_curve_force_n[0], abs=1e-9
    )
    assert _evaluate_adams_expression(expression, 2.0) == pytest.approx(
        spring.damper_curve_force_n[-1], abs=1e-9
    )


def test_nonzero_bushing_cannot_be_silently_dropped() -> None:
    stiffness = tuple(
        tuple(1000.0 if row == column == 0 else 0.0 for column in range(6))
        for row in range(6)
    )
    bushing = AxleBushing(
        name="nonzero_bushing",
        body_a="fixture",
        body_b="sprung",
        point_a_m=(0.0, 0.0, 0.0),
        point_b_m=(0.0, 0.0, 0.5),
        reference_translation_in_frame_a_m=(0.0, 0.0, 0.0),
        reference_quaternion_a_to_b=(1.0, 0.0, 0.0, 0.0),
        stiffness=stiffness,
        damping=tuple(tuple(0.0 for _ in range(6)) for _ in range(6)),
    )
    model = AxleDynamicsModel(
        **{**_model().model_dump(), "bushings": (bushing,)}
    )

    blockers = axle_adams_blockers(model, _case())

    assert any("no exact BUSHING/FIELD emitter" in blocker for blocker in blockers)


def test_equivalence_audit_blocks_independent_static_trim() -> None:
    manifest = _manifest()
    dataset = build_axle_adams_dataset(manifest, stem="audit_static")

    report = audit_axle_equivalence(manifest, dataset)

    assert report["status"] == "BLOCKED"
    assert report["equivalence_gate_passed"] is False
    assert any("provided_consistent_state" in item for item in report["blockers"])
    assert report["source_database_provenance"]["is_equivalence_gate"] is False


def test_equivalence_audit_passes_static_manifest_with_common_state_mode() -> None:
    case = _case().model_copy(
        update={
            "solver": AxleSolverSettings(
                adaptive_step=False,
                internal_step_s=0.00025,
                initialization_mode="provided_consistent_state",
            )
        }
    )
    manifest = _manifest(case)
    dataset = build_axle_adams_dataset(manifest, stem="audit_provided")

    report = audit_axle_equivalence(manifest, dataset)

    assert report["status"] == "PASS"
    assert report["equivalence_gate_passed"] is True
    assert report["shared_manifest_identity"]["dataset_hash_valid"] is True
    assert report["model_field_coverage"]["passed"] is True
    assert report["raw_bindings"]["channel_count"] > 0
    assert report["solver_conditions"]["discrete_integrator_equivalent"] is False


def test_equivalence_audit_requires_adams_refinement_at_runtime() -> None:
    case = _case().model_copy(
        update={
            "solver": AxleSolverSettings(
                adaptive_step=False,
                internal_step_s=0.00025,
                initialization_mode="provided_consistent_state",
            )
        }
    )
    manifest = _manifest(
        case,
        case_metadata={"comparison_basis": "continuous_problem_convergence"},
    )
    dataset = build_axle_adams_dataset(manifest, stem="audit_convergence")

    report = audit_axle_equivalence(
        manifest,
        dataset,
        native_evidence={
            "manifest_sha256": manifest.sha256,
            "diagnostics": {"time_convergence_passed": True},
        },
        require_runtime=True,
    )

    assert report["equivalence_gate_passed"] is False
    assert any("Adams time-convergence evidence" in item for item in report["blockers"])


def test_equivalence_audit_runtime_gate_requires_both_histories_and_state() -> None:
    case = _case().model_copy(
        update={
            "solver": AxleSolverSettings(
                adaptive_step=False,
                internal_step_s=0.00025,
                initialization_mode="provided_consistent_state",
            )
        }
    )
    manifest = _manifest(case)
    dataset = build_axle_adams_dataset(manifest, stem="audit_runtime")

    report = audit_axle_equivalence(
        manifest,
        dataset,
        require_runtime=True,
    )

    assert report["status"] == "BLOCKED"
    assert report["time_grid"]["passed"] is False
    assert report["channels"]["passed"] is False
    assert any("runtime equivalence audit" in item for item in report["blockers"])


def _road_height_function(model_text: str, tire: str) -> str:
    lines = model_text.splitlines()
    start = lines.index(f"! road_height of tire {tire}")
    body: list[str] = []
    for line in lines[start + 2 :]:
        if line.startswith(", FUNCTION = "):
            body.append(line[len(", FUNCTION = ") :])
        elif line.startswith(", ") and body:
            body.append(line[2:])
        else:
            break
    return " ".join(body)


def test_generator_refuses_every_inexpressible_element() -> None:
    bar_model = AxleDynamicsModel(
        **{
            **_model().model_dump(),
            "anti_roll_bars": (
                AxleAntiRollBar(
                    name="bar",
                    body_a="wheel_l",
                    body_b="wheel_r",
                    axis_a=(0.0, 1.0, 0.0),
                    reference_quaternion_a_to_b=(1.0, 0.0, 0.0, 0.0),
                    stiffness_n_m_per_rad=1000.0,
                    damping_n_m_s_per_rad=0.0,
                ).model_dump(),
            ),
        }
    )
    shared_state = AxleDynamicsCase(
        **{
            **_case().model_dump(),
            "solver": AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.00025,
            ).model_dump(),
        }
    )

    assert any(
        "anti-roll bar" in blocker
        for blocker in axle_adams_blockers(bar_model, _case())
    )
    assert not any(
        "initialization_mode" in blocker
        for blocker in axle_adams_blockers(_model(), shared_state)
    )
    dataset = build_axle_adams_dataset(_manifest(shared_state))
    assert "sim/static" not in dataset.command_text


def test_generator_refuses_a_road_input_it_cannot_express_exactly() -> None:
    case = _case("road_sine", duration_s=0.2)
    time = np.asarray(case.times_s)
    sine = tuple(float(value) for value in 0.01 * np.sin(2.0 * np.pi * 5.0 * time))
    sine_case = AxleDynamicsCase(
        **{**case.model_dump(), "road_height_m": {"tire_l": sine}}
    )

    blockers = axle_adams_blockers(_model(), sine_case)

    assert any("piecewise-linear breakpoints" in blocker for blocker in blockers)


def test_dataset_files_and_sidecar_are_written(tmp_path: Path) -> None:
    dataset = build_axle_adams_dataset(_manifest())

    paths = write_axle_adams_dataset(dataset, tmp_path / "adams")
    sidecar = json.loads(paths["sidecar"].read_text(encoding="utf-8"))

    assert paths["model"].read_text(encoding="ascii") == dataset.model_text
    assert paths["command"].read_text(encoding="ascii") == dataset.command_text
    assert sidecar["dataset_sha256"] == dataset.as_dict()["dataset_sha256"]
    assert "not a correlation claim" in str(
        sidecar["conventions"]["unverified_note"]
    )
    assert sidecar["conventions"]["validity_conditions"]
