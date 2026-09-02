from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.model import build_vehicle
from suspension_multibody.schema import (
    AerodynamicDragSpec,
    BumpStop,
    Bushing6x6,
    DrivelineSpec,
    DynamicSolverSettings,
    FrontAxleModel,
    InitialBodyState,
    LinearSpring,
    MassSpec,
    Pose,
    Quaternion,
    RigidBodySpec,
    RoadSurfaceSpec,
    SixVector,
    StaticDamper,
    SteeringSystemSpec,
    TimeSignal,
    TireModelSpec,
    UnitSystem,
    Vec3,
    VehicleDynamicCase,
    VehicleModel,
    WheelSpec,
)
from suspension_multibody.vehicle_dynamics import (
    _build_elements,
    _build_joints,
    _build_road,
    _build_wheel_torque_signals,
    _initial_body_state,
    _shift_point,
    run_vehicle_dynamics,
)

_BODY_NAMES = (
    "rack",
    "upper_arm_L",
    "upper_arm_R",
    "lower_arm_L",
    "lower_arm_R",
    "upright_L",
    "upright_R",
    "tie_rod_L",
    "tie_rod_R",
)


def _axle(name: str, x: float, dampers: tuple[StaticDamper, ...] = ()) -> FrontAxleModel:
    return FrontAxleModel(
        name=name,
        hardpoints={
            "UPPER_INBOARD_FRONT": Vec3(x=x, y=-500, z=500),
            "UPPER_INBOARD_REAR": Vec3(x=x + 150, y=-500, z=500),
            "UPPER_OUTBOARD": Vec3(x=x, y=-750, z=350),
            "LOWER_INBOARD_FRONT": Vec3(x=x, y=-500, z=100),
            "LOWER_INBOARD_REAR": Vec3(x=x + 150, y=-500, z=100),
            "LOWER_OUTBOARD": Vec3(x=x, y=-750, z=100),
            "TIE_ROD_INBOARD": Vec3(x=x, y=-450, z=250),
            "TIE_ROD_OUTBOARD": Vec3(x=x, y=-750, z=250),
            "WHEEL_CENTER": Vec3(x=x, y=-750, z=300),
            "RACK_CENTER": Vec3(x=x, y=0, z=250),
        },
        mass=MassSpec(sprung_mass=600),
        bodies=tuple(
            RigidBodySpec(
                name=body,
                mass=100,
                inertia=((100, 0, 0), (0, 100, 0), (0, 0, 100)),
            )
            for body in _BODY_NAMES
        ),
        dampers=dampers,
    )


def _tire() -> TireModelSpec:
    return TireModelSpec(
        kind="native_brush",
        unloaded_radius=300,
        maximum_compression=250,
        vertical_stiffness=200,
        cornering_stiffness=80_000,
        longitudinal_stiffness=120_000,
        relaxation_length=300,
    )


def _vehicle(
    *,
    front_dampers: tuple[StaticDamper, ...] = (),
    rear_dampers: tuple[StaticDamper, ...] = (),
) -> VehicleModel:
    rear_axle = _axle("rear", -1400, rear_dampers).model_copy(
        update={"rack_fixed_to_chassis": True}
    )
    return VehicleModel(
        chassis=RigidBodySpec(
            name="chassis",
            mass=1200,
            inertia=((1_000_000, 0, 0), (0, 1_200_000, 0), (0, 0, 1_500_000)),
        ),
        front_axle=_axle("front", 1400, front_dampers),
        rear_axle=rear_axle,
        wheels=tuple(
            WheelSpec(
                name=name,
                body=f"wheel_{name}",
                center_local=Vec3(),
                mass=20,
                axial_inertia=2,
                tire=_tire(),
            )
            for name in ("front_left", "front_right", "rear_left", "rear_right")
        ),
        steering=SteeringSystemSpec(ratio=16, rack_damping=0),
    )


def _positioned_axle(axle: FrontAxleModel) -> FrontAxleModel:
    def point(name: str, side: str) -> Vec3:
        value = axle.hardpoints[name]
        return value if side == "L" else value.mirrored_y()

    def mean(names: tuple[str, ...], side: str = "L") -> np.ndarray:
        return np.mean(
            np.asarray([point(name, side).as_array() for name in names]), axis=0
        )

    origins: dict[str, np.ndarray] = {"rack": mean(("RACK_CENTER",))}
    for side in ("L", "R"):
        origins.update(
            {
                f"upper_arm_{side}": mean(
                    ("UPPER_INBOARD_FRONT", "UPPER_INBOARD_REAR", "UPPER_OUTBOARD"),
                    side,
                ),
                f"lower_arm_{side}": mean(
                    ("LOWER_INBOARD_FRONT", "LOWER_INBOARD_REAR", "LOWER_OUTBOARD"),
                    side,
                ),
                f"upright_{side}": mean(
                    ("UPPER_OUTBOARD", "LOWER_OUTBOARD", "WHEEL_CENTER"), side
                ),
                f"tie_rod_{side}": mean(
                    ("TIE_ROD_INBOARD", "TIE_ROD_OUTBOARD"), side
                ),
            }
        )

    bodies = tuple(
        body.model_copy(
            update={
                "pose": body.pose.model_copy(
                    update={
                        "translation": Vec3(
                            x=float(origins[body.name][0]),
                            y=float(origins[body.name][1]),
                            z=float(origins[body.name][2]),
                        )
                    }
                )
            }
        )
        for body in axle.bodies
    )
    return axle.model_copy(update={"bodies": bodies})


def _positioned_vehicle(model: VehicleModel) -> VehicleModel:
    return model.model_copy(
        update={
            "front_axle": _positioned_axle(model.front_axle),
            "rear_axle": _positioned_axle(model.rear_axle),
        }
    )


def _with_ride_springs(model: VehicleModel) -> VehicleModel:
    front = model.front_axle
    rear = model.rear_axle
    front_spring = LinearSpring(
        name="ride_spring",
        body_a="chassis",
        body_b="lower_arm",
        point_a=front.hardpoints["LOWER_INBOARD_FRONT"],
        point_b=front.hardpoints["LOWER_OUTBOARD"],
        stiffness=100.0,
        free_length=450.0,
    )
    rear_spring = front_spring.model_copy(
        update={
            "point_a": rear.hardpoints["LOWER_INBOARD_FRONT"],
            "point_b": rear.hardpoints["LOWER_OUTBOARD"],
        }
    )
    return model.model_copy(
        update={
            "front_axle": front.model_copy(update={"springs": (front_spring,)}),
            "rear_axle": rear.model_copy(update={"springs": (rear_spring,)}),
        }
    )


def _case(
    model: VehicleModel,
    *,
    name: str = "native-vehicle",
    brake: float = 0.0,
    wheel_speeds: tuple[tuple[str, float], ...] = (),
    steering: TimeSignal | None = None,
) -> VehicleDynamicCase:
    return VehicleDynamicCase(
        name=name,
        vehicle=model,
        solver=DynamicSolverSettings(
            end_time=0.001,
            step_size=0.001,
            internal_step_size=0.001,
            min_internal_step_size=0.001,
            adaptive_substepping=False,
            integrator="generalized_alpha",
            gravity=Vec3(x=0, y=0, z=0),
        ),
        road=RoadSurfaceSpec(kind="plane"),
        steering_input=steering or TimeSignal(constant=0.0),
        brake_input=TimeSignal(constant=brake),
        initial_wheel_speeds=wheel_speeds,
    )


def _pac2002_model(
    *, combined: bool, parameter_source: str = "user"
) -> VehicleModel:
    base = _positioned_vehicle(_vehicle())
    coefficients = {
        "FNOMIN": 4_850.0,
        "PCX1": 1.65,
        "PDX1": 1.0,
        "PKX1": 22.3,
        "PCY1": 1.3,
        "PDY1": 1.0,
        "PKY1": -21.9,
        "RBX1": 10.0,
        "RBX2": 0.0,
        "RCX1": 1.2,
        "REX1": 0.2,
        "RBY1": 8.0,
        "RBY2": 0.0,
        "RBY3": 0.0,
        "RCY1": 1.1,
        "REY1": 0.1,
    }
    if not combined:
        coefficients.update(RBX1=0.0, RBY1=0.0)
    tire = _tire().model_copy(
        update={
            "kind": "pac2002",
            "parameter_source": parameter_source,
            "pac2002_coefficients": coefficients,
        }
    )
    return base.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(update={"tire": tire})
                for wheel in base.wheels
            )
        }
    )


def _uniform_velocity_initial_states(
    model: VehicleModel,
) -> tuple[InitialBodyState, ...]:
    assembly = build_vehicle(model, mode="K")
    states: list[InitialBodyState] = []
    for name, body in assembly.bodies.items():
        quaternion = body.pose.quaternion
        states.append(
            InitialBodyState(
                body=name,
                pose=Pose(
                    translation=Vec3(
                        x=float(body.pose.translation[0]),
                        y=float(body.pose.translation[1]),
                        z=float(body.pose.translation[2]),
                    ),
                    rotation=Quaternion(
                        w=float(quaternion[0]),
                        x=float(quaternion[1]),
                        y=float(quaternion[2]),
                        z=float(quaternion[3]),
                    ),
                ),
                # 工程单位为 mm/s；同时施加纵向和侧向速度以激活联合滑移。
                velocity=SixVector(fx=10_000.0, fy=5_000.0),
            )
        )
    return tuple(states)


def test_native_vehicle_runs_two_suspensions_and_four_wheels() -> None:
    model = _vehicle()

    result = run_vehicle_dynamics(model, _case(model))

    assert len(result.body_names) == 23
    assert result.tire_names == (
        "front_left",
        "front_right",
        "rear_left",
        "rear_right",
    )
    assert result.steering_state("front_rack").shape == (2, 4)
    assert np.all(result.diagnostics.accepted)
    assert np.all(np.isfinite(result.states))


def test_pac2002_selected_combined_slip_changes_force() -> None:
    def run(combined: bool):
        model = _pac2002_model(combined=combined)
        case = _case(model).model_copy(
            update={
                "road": RoadSurfaceSpec(
                    kind="plane", origin=Vec3(z=1.0)
                ),
                "initial_states": _uniform_velocity_initial_states(model),
            }
        )
        return run_vehicle_dynamics(model, case)

    pure = run(False)
    combined = run(True)
    assert np.all(pure.diagnostics.accepted)
    assert np.all(combined.diagnostics.accepted)
    pure_tire = pure.axle.tire_output[-1]
    combined_tire = combined.axle.tire_output[-1]
    assert np.max(np.abs(combined_tire[:, 5:7] - pure_tire[:, 5:7])) > 1.0e-5
    assert np.all(combined_tire[:, 9] > 0.0)


def test_pac2002_initial_relaxation_state_matches_current_slip() -> None:
    model = _pac2002_model(combined=False)
    case = _case(model).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": _uniform_velocity_initial_states(model),
        }
    )

    result = run_vehicle_dynamics(model, case)
    initial_tire = result.axle.tire_output[0]

    assert np.all(result.diagnostics.accepted)
    assert np.max(np.abs(initial_tire[:, 5:7])) > 1.0e-3
    assert np.max(np.abs(initial_tire[:, 10:12])) > 1.0e-6


def test_adams_pac2002_source_uses_local_relaxation_state() -> None:
    base = _pac2002_model(combined=False, parameter_source="adams_builtin")
    model = base.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(update={"axial_inertia": 200_000.0})
                for wheel in base.wheels
            )
        }
    )
    case = _case(model).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": _uniform_velocity_initial_states(model),
        }
    )

    result = run_vehicle_dynamics(model, case)
    current = result.axle.tire_output[-1]

    assert np.all(result.diagnostics.accepted)
    assert np.all(np.abs(current[:, 7]) > 1.0e-3)
    kinematic = np.clip(-current[:, 7] / 10.0, -1.0, 1.0)
    assert np.max(np.abs(current[:, 10] - kinematic)) > 1.0e-4


def test_adams_pac2002_preserves_source_static_offset() -> None:
    coefficients = {
        "PHX1": 0.01,
        "PVX1": 0.02,
        "PHY1": 0.01,
        "PVY1": 0.02,
    }
    base = _pac2002_model(combined=False)
    user_model = base.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(
                    update={
                        "tire": wheel.tire.model_copy(
                            update={
                                "pac2002_coefficients": {
                                    **wheel.tire.pac2002_coefficients,
                                    **coefficients,
                                }
                            }
                        )
                    }
                )
                for wheel in base.wheels
            )
        }
    )
    source_model = user_model.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(
                    update={
                        "tire": wheel.tire.model_copy(
                            update={"parameter_source": "adams_builtin"}
                        )
                    }
                )
                for wheel in user_model.wheels
            )
        }
    )
    user_case = _case(user_model).model_copy(
        update={"road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0))}
    )
    source_case = _case(source_model).model_copy(
        update={"road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0))}
    )

    user_result = run_vehicle_dynamics(user_model, user_case)
    source_result = run_vehicle_dynamics(source_model, source_case)

    assert np.all(user_result.diagnostics.accepted)
    assert np.all(source_result.diagnostics.accepted)
    assert np.max(np.abs(user_result.axle.tire_output[0, :, 5:7])) < 1e-10
    assert np.max(np.abs(source_result.axle.tire_output[0, :, 5:7])) > 1e-8


def test_adams_pac2002_uses_source_gyroscopic_moment_parameters() -> None:
    base = _pac2002_model(combined=False, parameter_source="adams_builtin")

    def with_gyro(qtz1: float, mbelt: float) -> VehicleModel:
        return base.model_copy(
            update={
                "wheels": tuple(
                    wheel.model_copy(
                        update={
                            "tire": wheel.tire.model_copy(
                                update={
                                    "pac2002_coefficients": {
                                        **wheel.tire.pac2002_coefficients,
                                        "LGYR": 1.0,
                                        "QTZ1": qtz1,
                                        "MBELT": mbelt,
                                    }
                                }
                            )
                        }
                    )
                    for wheel in base.wheels
                )
            }
        )

    disabled = with_gyro(0.0, 0.0)
    enabled = with_gyro(0.2, 5.4)

    def run(model: VehicleModel):
        case = _case(model).model_copy(
            update={
                "solver": _case(model).solver.model_copy(
                    update={
                        "end_time": 0.001,
                        "step_size": 0.001,
                        "internal_step_size": 0.001,
                        "min_internal_step_size": 0.001,
                    }
                ),
                "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
                "initial_states": _uniform_velocity_initial_states(model),
            }
        )
        return run_vehicle_dynamics(model, case)

    disabled_result = run(disabled)
    enabled_result = run(enabled)

    assert np.all(disabled_result.diagnostics.accepted)
    assert np.all(enabled_result.diagnostics.accepted)
    delta = (
        enabled_result.axle.tire_output[-1, :, 14]
        - disabled_result.axle.tire_output[-1, :, 14]
    )
    assert np.max(np.abs(delta)) > 1.0e-8


def test_pac2002_aligning_moment_changes_single_wheel_response() -> None:
    baseline = _pac2002_model(combined=False)
    aligning = baseline.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(
                    update={
                        "tire": wheel.tire.model_copy(
                            update={
                                "pac2002_coefficients": {
                                    **wheel.tire.pac2002_coefficients,
                                    "QDZ1": 0.08,
                                    "QDZ6": 0.01,
                                    "QCZ1": 1.0,
                                }
                            }
                        )
                    }
                )
                if wheel.name == "front_left" else wheel
                for wheel in baseline.wheels
            )
        }
    )
    baseline_case = _case(baseline).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": _uniform_velocity_initial_states(baseline),
        }
    )
    aligning_case = _case(aligning).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": _uniform_velocity_initial_states(aligning),
        }
    )

    baseline_result = run_vehicle_dynamics(baseline, baseline_case)
    aligning_result = run_vehicle_dynamics(aligning, aligning_case)

    assert np.all(baseline_result.diagnostics.accepted)
    assert np.all(aligning_result.diagnostics.accepted)
    assert np.max(
        np.abs(aligning_result.states[-1]-baseline_result.states[-1])
    ) > 1.0e-10


def test_pac2002_chrono_overturning_and_rolling_moments_change_response() -> None:
    # 20 kg、300 mm 轮端使用物理量级的转动惯量，避免附加滚阻矩在
    # 1 ms 回归步长中产生非物理的超大角加速度。
    base = _pac2002_model(combined=False)
    baseline = base.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(update={"axial_inertia": 200_000.0})
                for wheel in base.wheels
            )
        }
    )
    chrono_terms = baseline.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(
                    update={
                        "tire": wheel.tire.model_copy(
                            update={
                                "pac2002_coefficients": {
                                    **wheel.tire.pac2002_coefficients,
                                    # Chrono CalcMx：接触坐标系中的载荷相关倾覆力矩。
                                    "QSX1": 0.02,
                                    # Chrono CalcMy：滚动阻力力矩项。
                                    "QSY1": 0.01,
                                }
                            }
                        )
                    }
                )
                if wheel.name == "front_left" else wheel
                for wheel in baseline.wheels
            )
        }
    )
    baseline_case = _case(baseline).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": _uniform_velocity_initial_states(baseline),
        }
    )
    chrono_case = _case(chrono_terms).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": _uniform_velocity_initial_states(chrono_terms),
        }
    )

    baseline_result = run_vehicle_dynamics(baseline, baseline_case)
    chrono_result = run_vehicle_dynamics(chrono_terms, chrono_case)

    assert np.all(baseline_result.diagnostics.accepted)
    assert np.all(chrono_result.diagnostics.accepted)
    assert np.max(
        np.abs(chrono_result.states[-1]-baseline_result.states[-1])
    ) > 1.0e-10


def test_reduced_kkt_matches_dense_fallback(monkeypatch: pytest.MonkeyPatch) -> None:
    model = _vehicle()
    case = _case(model)

    monkeypatch.setenv("SUSPENSION_AXLE_DISABLE_REDUCED_KKT", "1")
    dense = run_vehicle_dynamics(model, case)
    monkeypatch.delenv("SUSPENSION_AXLE_DISABLE_REDUCED_KKT")
    reduced = run_vehicle_dynamics(model, case)

    np.testing.assert_allclose(
        reduced.states, dense.states, rtol=2.0e-6, atol=2.0e-8
    )
    np.testing.assert_allclose(
        reduced.axle.tire_output, dense.axle.tire_output,
        rtol=2.0e-6, atol=2.0e-8,
    )
    np.testing.assert_allclose(
        reduced.axle.energy, dense.axle.energy, rtol=2.0e-6, atol=2.0e-8
    )
    assert np.all(reduced.diagnostics.accepted)


def test_native_vehicle_snaps_internal_time_roundoff_at_output_end() -> None:
    model = _vehicle()
    base_case = _case(model)
    case = base_case.model_copy(
        update={
            "solver": base_case.solver.model_copy(
                update={
                    "end_time": 0.37,
                    "step_size": 0.01,
                    "output_step": 0.01,
                    "internal_step_size": 0.001,
                    "min_internal_step_size": 0.001,
                }
            )
        }
    )

    result = run_vehicle_dynamics(model, case)

    assert result.times_s[-1] == pytest.approx(0.37)
    assert np.all(result.diagnostics.accepted)


def test_native_attachment_points_respect_body_origin_and_orientation() -> None:
    angle = np.pi / 3.0
    model = _vehicle()
    upper_arm = next(
        body for body in model.front_axle.bodies if body.name == "upper_arm_L"
    ).model_copy(
        update={
            "pose": Pose(
                translation=Vec3(x=800.0, y=-250.0, z=120.0),
                rotation=Quaternion(
                    w=np.cos(angle / 2.0), z=np.sin(angle / 2.0)
                ),
            ),
            "center_of_mass": Vec3(x=100.0, y=40.0, z=60.0),
        }
    )
    front_bodies = tuple(
        upper_arm if body.name == "upper_arm_L" else body
        for body in model.front_axle.bodies
    )
    model = model.model_copy(
        update={
            "front_axle": model.front_axle.model_copy(
                update={"bodies": front_bodies}
            )
        }
    )
    assembly = build_vehicle(model, mode="K")
    case = _case(model)
    bodies, body_frames = _initial_body_state(assembly, case, 1.0e-3)

    body_name = "front_upper_arm_L"
    local_point = assembly.points[(body_name, "inner_front")]
    local_point = np.asarray(
        _shift_point(body_name, local_point, body_frames, 1.0e-3)
    )
    runtime_body = assembly.bodies[body_name]
    native_body = next(body for body in bodies if body.name == body_name)
    rotation = body_frames[body_name].rotation
    reconstructed = np.asarray(native_body.position_m) + rotation @ local_point
    expected = runtime_body.pose.transform_point(
        assembly.points[(body_name, "inner_front")]
    ) * 1.0e-3

    np.testing.assert_allclose(reconstructed, expected, atol=1.0e-12)
    np.testing.assert_allclose(
        np.asarray(native_body.position_m),
        runtime_body.pose.translation * 1.0e-3
        + rotation @ (runtime_body.center_of_mass * 1.0e-3),
        atol=1.0e-12,
    )

    native_joint = next(
        joint
        for joint in _build_joints(assembly, body_frames, 1.0e-3)
        if joint.name == "front_uca_mount_L_inner_front"
    )
    np.testing.assert_allclose(native_joint.point_b_m, local_point, atol=1.0e-12)


def test_native_vehicle_passes_spring_and_stop_curves_to_vehicle_abi() -> None:
    base = _positioned_vehicle(_vehicle())
    front = base.front_axle
    spring = LinearSpring(
        name="curve_spring",
        body_a="chassis",
        body_b="lower_arm",
        point_a=front.hardpoints["LOWER_INBOARD_FRONT"],
        point_b=front.hardpoints["LOWER_OUTBOARD"],
        stiffness=100.0,
        free_length=450.0,
        force_curve=((0.0, 0.0), (100.0, 100.0), (200.0, 350.0)),
    )
    stop = BumpStop(
        name="curve_stop",
        body_a="chassis",
        body_b="lower_arm",
        point_a=front.hardpoints["LOWER_INBOARD_FRONT"],
        point_b=front.hardpoints["LOWER_OUTBOARD"],
        clearance=25.0,
        stiffness=1_000.0,
        direction="bump",
        force_curve=((0.0, 0.0), (10.0, 100.0), (20.0, 500.0)),
    )
    model = base.model_copy(
        update={
            "front_axle": front.model_copy(
                update={"springs": (spring,), "stops": (stop,)}
            )
        }
    )
    assembly = build_vehicle(model, mode="K")
    _bodies, body_frames = _initial_body_state(assembly, _case(model), 1.0e-3)
    springs, _bushings = _build_elements(assembly, body_frames, 1.0e-3)
    mapped_spring = next(
        item for item in springs if item.name == "front_curve_spring_L"
    )
    mapped_stop = next(item for item in springs if item.name == "front_curve_stop_L")

    np.testing.assert_allclose(
        mapped_spring.elastic_curve_deflection_m,
        (-0.2, -0.1, 0.0),
    )
    np.testing.assert_allclose(mapped_spring.elastic_curve_force_n, (-350.0, -100.0, 0.0))
    np.testing.assert_allclose(
        mapped_stop.compression_stop_curve_penetration_m,
        (0.0, 0.01, 0.02),
    )
    np.testing.assert_allclose(
        mapped_stop.compression_stop_curve_force_n,
        (0.0, 100.0, 500.0),
    )

    result = run_vehicle_dynamics(model, _case(model))

    assert np.all(result.diagnostics.accepted)
    assert result.spring_state("front_curve_spring_L").shape == (2, 7)
    assert result.spring_state("front_curve_stop_L").shape == (2, 7)


def test_native_vehicle_passes_bushing_curves_and_coordinates_to_vehicle_abi() -> None:
    base = _positioned_vehicle(_vehicle())
    front = base.front_axle
    point = front.hardpoints["UPPER_INBOARD_FRONT"]
    bushing = Bushing6x6(
        name="curve_bushing",
        body_a="chassis",
        body_b="upper_arm",
        pose_a=Pose(translation=point),
        pose_b=Pose(
            translation=Vec3(x=point.x, y=point.y, z=point.z + 50.0)
        ),
        stiffness=((0.0,) * 6,) * 6,
        damping=(0.0,) * 6,
        rotation_coordinates="cardan_xyz",
        force_curves=(
            (),
            (),
            ((-100.0, -100.0), (0.0, 0.0), (100.0, 100.0)),
            (),
            (),
            (),
        ),
    )
    model = base.model_copy(
        update={
            "front_axle": front.model_copy(update={"bushings": (bushing,)})
        }
    )

    result = run_vehicle_dynamics(model, _case(model))

    assert np.all(result.diagnostics.accepted)
    bushing_state = result.bushing_state("front_curve_bushing_L")
    assert bushing_state.shape == (2, 12)
    assert bushing_state[0, 2] == pytest.approx(0.05, abs=1e-8)
    assert bushing_state[0, 8] == pytest.approx(-50.0, abs=1e-6)


def test_native_vehicle_static_trim_balances_gravity_with_four_tire_contacts() -> None:
    model = _with_ride_springs(_positioned_vehicle(_vehicle()))
    base_case = _case(model)
    case = base_case.model_copy(
        update={
            "static_equilibrium": True,
            "solver": base_case.solver.model_copy(
                update={
                    "gravity": Vec3(x=0.0, y=0.0, z=-9810.0),
                    "projection_max_iterations": 60,
                    "projection_backtracking": 20,
                }
            ),
        }
    )

    result = run_vehicle_dynamics(model, case)

    assert np.all(result.diagnostics.accepted)
    assert np.all(result.diagnostics.active_contacts == 4)
    assert np.max(result.diagnostics.position_residual) < 1.0e-8
    assert np.max(result.diagnostics.dynamics_residual) < 1.0e-7
    assert np.max(result.diagnostics.pinned_null_directions) >= 1


def test_static_trim_accepts_zero_speed_drag_and_roundoff_brake_torque() -> None:
    model = _with_ride_springs(_positioned_vehicle(_vehicle())).model_copy(
        update={
            "aerodynamic_drag": AerodynamicDragSpec(
                air_density=1.225,
                drag_coefficient=0.32,
                frontal_area=2.2,
            ),
            "steering": SteeringSystemSpec(
                ratio=16.0,
                rack_damping=0.0,
                actuator_mode="prescribed_translation",
                actuator_reaction_body="chassis",
                actuator_axis_local=Vec3(y=1.0),
            ),
        }
    )
    base_case = _case(model, steering=TimeSignal(constant=2.0))
    case = base_case.model_copy(
        update={
            "static_equilibrium": True,
            "wheel_brake_torque": tuple(
                (wheel.name, TimeSignal(constant=1.0e-14))
                for wheel in model.wheels
            ),
            "solver": base_case.solver.model_copy(
                update={
                    "gravity": Vec3(x=0.0, y=0.0, z=-9810.0),
                    "projection_max_iterations": 60,
                    "projection_backtracking": 20,
                }
            ),
        }
    )

    result = run_vehicle_dynamics(model, case)

    assert np.all(result.diagnostics.accepted)
    assert np.all(result.diagnostics.active_contacts == 4)


def test_native_vehicle_combines_trim_road_steering_and_drive() -> None:
    model = _with_ride_springs(_positioned_vehicle(_vehicle())).model_copy(
        update={
            "driveline": DrivelineSpec(
                driven_wheels=("front_left", "front_right"),
                maximum_drive_torque=800.0,
                maximum_brake_torque=1_000.0,
                front_brake_bias=0.6,
                drive_split=(0.5, 0.5, 0.0, 0.0),
            ),
            "steering": SteeringSystemSpec(ratio=16.0, rack_damping=200.0),
        }
    )
    base_case = _case(
        model,
        steering=TimeSignal(
            times=(0.0, 0.003, 0.006), values=(0.0, 0.0, 2.0)
        ),
    )
    case = base_case.model_copy(
        update={
            "drive_input": TimeSignal(
                times=(0.0, 0.003, 0.006), values=(0.0, 0.0, 0.2)
            ),
            "road": RoadSurfaceSpec(kind="sine", amplitude=3.0, wavelength=2_000.0),
            "static_equilibrium": True,
            "solver": base_case.solver.model_copy(
                update={
                    "end_time": 0.006,
                    "step_size": 0.001,
                    "internal_step_size": 0.001,
                    "min_internal_step_size": 0.001,
                    "adaptive_substepping": False,
                    "gravity": Vec3(x=0.0, y=0.0, z=-9810.0),
                }
            ),
        }
    )

    result = run_vehicle_dynamics(model, case)

    assert np.all(result.diagnostics.accepted)
    assert np.all(result.diagnostics.active_contacts == 4)
    assert np.max(result.diagnostics.position_residual) < 1.0e-7
    assert np.max(result.diagnostics.dynamics_residual) < 1.0e-6
    assert result.steering_state("front_rack")[-1, 2] == 0.002
    assert np.all(np.isfinite(result.body_state("wheel_front_left")))


def test_four_post_sampled_signals_do_not_duplicate_analytic_profile() -> None:
    signals = tuple(
        TimeSignal(times=(0.0, 0.001), values=(0.0, 10.0))
        for _ in range(4)
    )
    road = RoadSurfaceSpec(
        kind="four_post",
        origin=Vec3(z=2.0),
        amplitude=10.0,
        bump_length=100.0,
        corner_height_signals=signals,  # type: ignore[arg-type]
    )

    buffers, height, _velocity = _build_road(
        road, np.asarray((0.0, 0.001)), 1.0e-3
    )

    assert buffers.kind == 0
    assert height["front_left"] == (0.002, 0.012)


def test_engineering_damper_preload_is_converted_before_si_scaling() -> None:
    damper = StaticDamper(
        name="gas_damper",
        body_a="chassis",
        body_b="lower_arm",
        point_a=Vec3(x=0, y=-500, z=100),
        point_b=Vec3(x=0, y=-700, z=100),
        gas_stiffness=10.0,
        gas_reference_length=100.0,
        gas_reference_force=50.0,
        preload=20.0,
        friction=5.0,
    )
    model = _vehicle(front_dampers=(damper,))
    assembly = build_vehicle(model, mode="C")
    bodies, shifts = _initial_body_state(assembly, _case(model), 1.0e-3)
    del bodies
    springs, _bushings = _build_elements(assembly, shifts, 1.0e-3)

    mapped = next(spring for spring in springs if spring.name == "front_gas_damper_L")
    assert mapped.free_length_m == 0.0925


def test_brake_signal_is_a_nonnegative_magnitude() -> None:
    model = _vehicle().model_copy(
        update={
            "driveline": DrivelineSpec(
                maximum_brake_torque=1000.0,
                front_brake_bias=0.6,
            )
        }
    )
    case = _case(
        model,
        brake=1.0,
        wheel_speeds=(("front_left", -10.0),),
    )

    drive, brake = _build_wheel_torque_signals(
        model, case, np.asarray((0.0, 0.001)), 1.0e-3
    )

    assert drive["front_left"] == (0.0, 0.0)
    assert brake["front_left"] == (0.3, 0.3)


def test_direct_wheel_torque_signals_override_global_distribution() -> None:
    model = _vehicle().model_copy(
        update={
            "driveline": DrivelineSpec(
                driven_wheels=("front_left", "front_right"),
                maximum_drive_torque=1_000.0,
                maximum_brake_torque=1_000.0,
                front_brake_bias=0.6,
                drive_split=(0.5, 0.5, 0.0, 0.0),
            )
        }
    )
    case = _case(model).model_copy(
        update={
            "wheel_drive_torque": (
                ("rear_left", TimeSignal(constant=250.0)),
            ),
            "wheel_brake_torque": (
                ("front_right", TimeSignal(constant=40.0)),
            ),
        }
    )

    drive, brake = _build_wheel_torque_signals(
        model, case, np.asarray((0.0, 0.001)), 1.0e-3
    )

    assert drive == {
        "front_left": (0.0, 0.0),
        "front_right": (0.0, 0.0),
        "rear_left": (0.25, 0.25),
        "rear_right": (0.0, 0.0),
    }
    assert brake == {
        "front_left": (0.0, 0.0),
        "front_right": (0.04, 0.04),
        "rear_left": (0.0, 0.0),
        "rear_right": (0.0, 0.0),
    }

    mixed_drive = _case(model, brake=1.0).model_copy(
        update={
            "wheel_drive_torque": (
                ("rear_left", TimeSignal(constant=250.0)),
            )
        }
    )
    drive, brake = _build_wheel_torque_signals(
        model, mixed_drive, np.asarray((0.0, 0.001)), 1.0e-3
    )
    assert drive["rear_left"] == (0.25, 0.25)
    assert brake["front_left"] == (0.3, 0.3)

    mixed_brake = _case(model).model_copy(
        update={
            "drive_input": TimeSignal(constant=0.5),
            "wheel_brake_torque": (
                ("rear_right", TimeSignal(constant=40.0)),
            ),
        }
    )
    drive, brake = _build_wheel_torque_signals(
        model, mixed_brake, np.asarray((0.0, 0.001)), 1.0e-3
    )
    assert drive["front_left"] == (0.25, 0.25)
    assert brake["rear_right"] == (0.04, 0.04)


def test_native_brake_opposes_the_instantaneous_wheel_spin() -> None:
    model = _vehicle().model_copy(
        update={
            "driveline": DrivelineSpec(
                maximum_brake_torque=0.01,
                front_brake_bias=0.6,
            )
        }
    )
    positive = run_vehicle_dynamics(
        model,
        _case(model, brake=1.0, wheel_speeds=(("front_left", 10.0),)),
    )
    negative = run_vehicle_dynamics(
        model,
        _case(model, brake=1.0, wheel_speeds=(("front_left", -10.0),)),
    )

    assert positive.body_state("wheel_front_left")[-1, 11] < 10.0
    assert negative.body_state("wheel_front_left")[-1, 11] > -10.0


def test_native_vehicle_applies_a_rack_displacement_target() -> None:
    model = _vehicle()
    result = run_vehicle_dynamics(
        model,
        _case(
            model,
            steering=TimeSignal(times=(0.0, 0.001), values=(0.0, 1.0)),
        ),
    )

    steering = result.steering_state("front_rack")
    assert steering[-1, 2] == 0.001
    assert np.isfinite(steering[-1, 0])


def test_si_vehicle_requires_explicit_gravity() -> None:
    model = _vehicle().model_copy(
        update={
            "units": UnitSystem.SI,
            "front_axle": _vehicle().front_axle.model_copy(
                update={"units": UnitSystem.SI}
            ),
            "rear_axle": _vehicle().rear_axle.model_copy(
                update={"units": UnitSystem.SI}
            ),
        }
    )

    case = VehicleDynamicCase(
        solver=DynamicSolverSettings(
            end_time=0.001,
            step_size=0.001,
            internal_step_size=0.001,
            min_internal_step_size=0.001,
            adaptive_substepping=False,
            integrator="generalized_alpha",
        ),
        vehicle=model,
    )
    with pytest.raises(ValueError, match="gravity.*explicitly"):
        run_vehicle_dynamics(model, case)
