"""Road contact and unilateral tire tests."""

import numpy as np
import pytest

from suspension_multibody.core import SE3, RigidBody, RigidBodyState
from suspension_multibody.dynamics import (
    ContactTireElement,
    DynamicRigidBodyState,
    RoadSurface,
    evaluate_tire_contact,
)
from suspension_multibody.schema import RoadSurfaceSpec, TimeSignal, TireModelSpec


def _state(z: float, velocity: np.ndarray | None = None) -> DynamicRigidBodyState:
    body = RigidBody("wheel", pose=SE3(np.array([0.0, 0.0, z]), np.array([1.0, 0.0, 0.0, 0.0])))
    return DynamicRigidBodyState(
        RigidBodyState({"wheel": body}),
        velocities={"wheel": np.zeros(6) if velocity is None else velocity},
    )


def test_contact_generates_compression_only_normal_load() -> None:
    tire = TireModelSpec(kind="fiala", unloaded_radius=300.0, vertical_stiffness=200.0)
    result = evaluate_tire_contact(
        _state(250.0),
        wheel_body="wheel",
        spin_axis_local=np.array([0.0, 1.0, 0.0]),
        tire_spec=tire,
        road=RoadSurface(RoadSurfaceSpec()),
        time=0.0,
    )

    assert result.active
    assert result.compression == 50.0
    assert result.normal_load == 10_000.0
    assert result.forces.fz == 10_000.0
    assert np.allclose(result.normal, [0.0, 0.0, 1.0])


def test_free_rolling_wheel_has_zero_longitudinal_slip() -> None:
    radius = 300.0
    forward_speed = 1_000.0
    state = _state(
        250.0,
        np.array([forward_speed, 0.0, 0.0, 0.0, -forward_speed / radius, 0.0]),
    )
    result = evaluate_tire_contact(
        state,
        wheel_body="wheel",
        spin_axis_local=np.array([0.0, 1.0, 0.0]),
        tire_spec=TireModelSpec(kind="fiala", unloaded_radius=radius),
        road=RoadSurface(RoadSurfaceSpec()),
        time=0.0,
    )

    assert result.slip_ratio == pytest.approx(0.0, abs=1e-12)
    assert result.slip_angle == pytest.approx(0.0, abs=1e-12)
    assert result.forces.fx == pytest.approx(0.0, abs=1e-9)


def test_contact_is_inactive_above_road_and_element_returns_zero_wrench() -> None:
    tire = TireModelSpec(kind="pac2002", unloaded_radius=300.0)
    state = _state(301.0)
    result = evaluate_tire_contact(
        state,
        wheel_body="wheel",
        spin_axis_local=np.array([0.0, 1.0, 0.0]),
        tire_spec=tire,
        road=RoadSurface(RoadSurfaceSpec()),
        time=0.0,
    )
    evaluation = ContactTireElement(
        "tire",
        "wheel",
        np.array([0.0, 1.0, 0.0]),
        tire,
        RoadSurface(RoadSurfaceSpec()),
    ).evaluate_dynamic(state, 0.0)

    assert not result.active
    assert result.normal_load == 0.0
    assert evaluation.body_wrenches_global == {}
    assert evaluation.events == ("tire_unloaded",)


def test_sine_road_changes_surface_normal() -> None:
    road = RoadSurface(
        RoadSurfaceSpec(kind="sine", amplitude=10.0, wavelength=1_000.0)
    )
    query = road.query(np.array([100.0, 0.0, 0.0]), 0.0)

    assert query.point[2] > 0.0
    assert not np.allclose(query.normal, [0.0, 0.0, 1.0])


def test_four_post_road_accepts_independent_corner_signals() -> None:
    road = RoadSurface(
        RoadSurfaceSpec(
            kind="four_post",
            corner_height_signals=(
                TimeSignal(times=(0.0, 1.0), values=(0.0, 10.0)),
                TimeSignal(times=(0.0, 1.0), values=(0.0, 20.0)),
                TimeSignal(times=(0.0, 1.0), values=(0.0, 30.0)),
                TimeSignal(times=(0.0, 1.0), values=(0.0, 40.0)),
            ),
        )
    )

    query = road.query(np.zeros(3), 0.5, corner_index=2)
    assert query.point[2] == 15.0
    assert np.allclose(query.velocity, [0.0, 0.0, 30.0])


def test_contact_uses_wheel_center_local_and_applies_aligning_moment() -> None:
    tire = TireModelSpec(
        kind="fiala",
        unloaded_radius=300.0,
        vertical_stiffness=200.0,
        pneumatic_trail=50.0,
    )
    state = _state(50.0, np.array([0.0, 10.0, 0.0, 0.0, 0.0, 0.0]))
    result = evaluate_tire_contact(
        state,
        wheel_body="wheel",
        wheel_center_local=np.array([0.0, 0.0, 200.0]),
        spin_axis_local=np.array([0.0, 1.0, 0.0]),
        tire_spec=tire,
        road=RoadSurface(RoadSurfaceSpec()),
        time=0.0,
    )
    evaluation = ContactTireElement(
        "tire",
        "wheel",
        np.array([0.0, 1.0, 0.0]),
        tire,
        RoadSurface(RoadSurfaceSpec()),
        wheel_center_local=np.array([0.0, 0.0, 200.0]),
    ).evaluate_dynamic(state, 0.0)

    assert result.compression == 50.0
    assert result.forces.mz != 0.0
    assert evaluation.body_wrenches_global["wheel"][5] == result.forces.mz


def test_road_normal_must_point_upward() -> None:
    with pytest.raises(ValueError, match="point upward"):
        RoadSurfaceSpec(normal=(0.0, 0.0, -1.0))
