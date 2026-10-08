"""Static support balance for explicit one, four and six contact-frame layouts."""
import numpy as np
import pytest

from suspension_multibody.modeling.resolved import ResolvedModel
from suspension_multibody.vehicle.static_loads import (
    IncompatibleStaticLoadsError,
    compute_static_wheel_loads,
    compute_static_wheel_loads_for_assembly,
)

G = 9.81


def _model(points, *, center=(0, 0, .6), mass=1200):
    return ResolvedModel({"schema_version": 1, "name": "support-layout", "units": "SI",
        "bodies": [{"name": "load", "mass": mass, "inertia": np.eye(3).tolist(),
            "position": list(center), "quaternion": [1, 0, 0, 0]}],
        "frames": [{"name": name, "body": "ground", "point": list(point), "quaternion": [1, 0, 0, 0]}
            for name, point in points.items()], "joints": [], "elements": []})


def _solve(model, **kwargs):
    frames = {row["name"]: row["name"] for row in model.to_document()["frames"]}
    return compute_static_wheel_loads_for_assembly(model, contact_frames=frames, gravity=G, **kwargs)


def _three_axle_assembly():
    return _model({placement+"_"+side: [x, y, 0]
        for placement, x in (("front", 1.4), ("middle", 0), ("rear", -1.4))
        for side, y in (("left", -.75), ("right", .75))})


def _corner_assembly(*, side_y=0):
    return _model({"single_left": [0, side_y, 0]})


def _equilibrium_rows(result, acceleration=(0, 0, 0)):
    names = tuple(result.support_points)
    matrix = np.array([np.ones(len(names)),
        [result.support_points[name][0]-result.center_of_mass[0] for name in names],
        [result.support_points[name][1]-result.center_of_mass[1] for name in names]])
    height = result.center_of_mass[2]
    rhs = result.total_mass*np.array([G+acceleration[2], -height*acceleration[0], -height*acceleration[1]])
    return np.abs(matrix@np.array([result.wheel_loads[name] for name in names])-rhs)


def test_three_axle_six_contact_points_balance():
    result = _solve(_three_axle_assembly())
    assert set(result.support_points) == {placement+"_"+side for placement in ("front", "middle", "rear") for side in ("left", "right")}
    assert set(result.wheel_loads) == set(result.support_points)
    assert result.rank == 3 and not result.unique
    assert result.residual <= result.residual_tolerance
    assert np.all(_equilibrium_rows(result) <= result.residual_tolerance)
    assert sum(result.wheel_loads.values()) == pytest.approx(result.total_mass*G, abs=result.residual_tolerance)
    for value in result.wheel_loads.values():
        assert value == pytest.approx(result.total_mass*G/6, rel=1e-9, abs=1e-6)


def test_three_axle_six_contact_points_transfer_load_longitudinally():
    model = _three_axle_assembly()
    static = _solve(model)
    accelerated = _solve(model, acceleration=np.array([1., 0, 0]))
    assert accelerated.rank == 3
    assert accelerated.residual <= accelerated.residual_tolerance
    assert np.all(_equilibrium_rows(accelerated, (1., 0, 0)) <= accelerated.residual_tolerance)
    assert accelerated.wheel_loads["front_left"] < static.wheel_loads["front_left"]
    assert accelerated.wheel_loads["rear_left"] > static.wheel_loads["rear_left"]


def test_single_contact_point_under_the_centre_of_mass_solves_and_is_unique():
    result = _solve(_corner_assembly())
    assert len(result.support_points) == 1
    np.testing.assert_allclose(result.center_of_mass[:2], result.support_points["single_left"][:2], atol=1e-9)
    assert result.rank == 1 == len(result.support_points)
    assert result.unique and result.residual == 0
    assert result.wheel_loads["single_left"] == pytest.approx(result.total_mass*G, abs=result.residual_tolerance)
    assert np.all(_equilibrium_rows(result) <= result.residual_tolerance)


@pytest.mark.parametrize(("side_y", "acceleration"), [(0, [0, 1., 0]), (-.7, [-3., 2., 0])])
def test_single_contact_point_reports_incompatible_loads_by_name(side_y, acceleration):
    with pytest.raises(IncompatibleStaticLoadsError) as caught:
        _solve(_corner_assembly(side_y=side_y), acceleration=np.array(acceleration))
    error = caught.value
    assert error.contact_points == 1 and error.residual > error.tolerance
    assert repr(error.residual) in str(error)
    assert repr(error.tolerance) in str(error)
    assert "1 contact point(s)" in str(error)
    assert "rank" not in str(error)
    assert isinstance(error, ValueError)


def test_the_rank_of_a_single_contact_point_matrix_does_not_change_with_the_loads():
    model = _corner_assembly()
    result = _solve(model)
    with pytest.raises(IncompatibleStaticLoadsError):
        _solve(model, acceleration=np.array([0, 1., 0]))
    point = result.support_points["single_left"]
    matrix = np.array([[1], [point[0]-result.center_of_mass[0]], [point[1]-result.center_of_mass[1]]])
    assert np.linalg.matrix_rank(matrix) == result.rank == 1


def test_four_wheel_vehicle_keeps_the_minimum_norm_split():
    points = {placement+"_"+side: [x, y, 0] for placement, x in (("front", 1.4), ("rear", -1.4)) for side, y in (("left", -.75), ("right", .75))}
    result = _solve(_model(points))
    assert set(result.wheel_loads) == set(points)
    assert len(result.support_points) == 4 and result.rank == 3
    assert not result.unique
    assert result.residual <= result.residual_tolerance
    assert np.all(_equilibrium_rows(result) <= result.residual_tolerance)
    for value in result.wheel_loads.values():
        assert value == pytest.approx(result.summary.total/4, rel=1e-9, abs=1e-8)


def test_the_contact_points_are_the_explicit_frame_mapping():
    graph = _three_axle_assembly()
    result = _solve(graph)
    for frame in graph.to_document()["frames"]:
        np.testing.assert_array_equal(result.support_points[frame["name"]], frame["point"])


def test_the_model_entry_reads_the_same_resolved_graph():
    model = _three_axle_assembly()
    before = model.to_document()
    frames = {row["name"]: row["name"] for row in before["frames"]}
    one = compute_static_wheel_loads(model, contact_frames=frames, gravity=G)
    two = _solve(model)
    assert one.wheel_loads == two.wheel_loads
    assert model.to_document() == before


@pytest.mark.parametrize("contacts", [{}, {"one": "missing"}, {"one": "single_left", "two": "single_left"}])
def test_invalid_contact_selection_is_refused(contacts):
    with pytest.raises(ValueError):
        compute_static_wheel_loads(_corner_assembly(), contact_frames=contacts)


def test_rotated_body_local_contact_uses_its_declared_pose():
    graph = _corner_assembly().to_document()
    graph["bodies"][0]["quaternion"] = [np.cos(np.pi/4), 0, 0, np.sin(np.pi/4)]
    graph["frames"][0].update(body="load", point=[0, 0, -.6])
    result = _solve(ResolvedModel(graph))
    np.testing.assert_allclose(result.support_points["single_left"], [0, 0, 0], atol=1e-12)
    assert result.unique
