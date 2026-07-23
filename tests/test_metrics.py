import numpy as np

from kinematics.core.constants import TEST_TOLERANCE
from kinematics.core.enums import Axis, PointID
from kinematics.io.geometry_loader import load_geometry
from kinematics.io.sweep_loader import parse_sweep_file
from kinematics.main import solve_sweep
from kinematics.metrics.catalog import get_default_corner_metrics
from kinematics.metrics.context import MetricContext
from kinematics.metrics.main import compute_metrics_for_state_from_suspension
from kinematics.points.derived.manager import DerivedPointsManager
from kinematics.suspensions.double_wishbone import DoubleWishboneSuspension


def _shift_x(vec3: object, delta_x: float) -> tuple[float, float, float]:
    """
    Shift a 3D point along the world X axis by a fixed amount.
    """
    shifted = np.asarray(vec3, dtype=np.float64).copy()
    shifted[0] += delta_x
    return (float(shifted[0]), float(shifted[1]), float(shifted[2]))


def _translate_double_wishbone_x(
    suspension: DoubleWishboneSuspension, delta_x: float
) -> DoubleWishboneSuspension:
    """
    Build a rigidly translated copy of a double wishbone suspension.

    Hardpoints and any configuration points that live in world coordinates
    are shifted together so the translated suspension is geometrically
    identical to the original one.
    """
    hardpoints = {
        point_id: position.copy() + np.array([delta_x, 0.0, 0.0], dtype=np.float64)
        for point_id, position in suspension.hardpoints.items()
    }

    config = suspension.config
    translated_config = None
    if config is not None:
        config_updates: dict[str, object] = {
            "cg_position": _shift_x(config.cg_position, delta_x)
        }

        if config.camber_shim is not None:
            translated_shim = config.camber_shim.model_copy(
                update={
                    "shim_face_point_a": _shift_x(
                        config.camber_shim.shim_face_point_a, delta_x
                    ),
                    "shim_face_point_b": _shift_x(
                        config.camber_shim.shim_face_point_b, delta_x
                    ),
                }
            )
            config_updates["camber_shim"] = translated_shim

        translated_config = config.model_copy(update=config_updates)

    return DoubleWishboneSuspension(
        name=suspension.name,
        version=suspension.version,
        units=suspension.units,
        hardpoints=hardpoints,
        config=translated_config,
    )


def test_front_view_metrics_are_invariant_to_rigid_x_translation(
    double_wishbone_geometry_file,
    test_data_dir,
) -> None:
    suspension = load_geometry(double_wishbone_geometry_file)
    assert isinstance(suspension, DoubleWishboneSuspension)

    sweep_config = parse_sweep_file(test_data_dir / "sweep.yaml")
    states, _ = solve_sweep(suspension, sweep_config)

    translated = _translate_double_wishbone_x(suspension, 100.0)
    translated_states, _ = solve_sweep(translated, sweep_config)

    original_metrics = [
        compute_metrics_for_state_from_suspension(state, suspension) for state in states
    ]
    translated_metrics = [
        compute_metrics_for_state_from_suspension(state, translated)
        for state in translated_states
    ]

    comparison_index = next(
        index
        for index, metrics in enumerate(original_metrics)
        if metrics["fvic_y_mm"] is not None
    )

    for column_name in ("fvic_y_mm", "fvic_z_mm", "fvsa_length_mm"):
        original_value = original_metrics[comparison_index][column_name]
        translated_value = translated_metrics[comparison_index][column_name]
        assert original_value is not None, f"{column_name} is None in original"
        assert translated_value is not None, f"{column_name} is None in translated"
        np.testing.assert_allclose(
            original_value,
            translated_value,
            atol=TEST_TOLERANCE,
            rtol=TEST_TOLERANCE,
            err_msg=f"{column_name} changed under rigid X translation",
        )


def test_svsa_uses_signed_side_view_distance(
    double_wishbone_geometry_file,
    test_data_dir,
) -> None:
    suspension = load_geometry(double_wishbone_geometry_file)
    assert isinstance(suspension, DoubleWishboneSuspension)

    states, _ = solve_sweep(suspension, parse_sweep_file(test_data_dir / "sweep.yaml"))
    state = next(
        state
        for state in states
        if suspension.compute_side_view_instant_center(state) is not None
    )
    side_view_ic = suspension.compute_side_view_instant_center(state)
    assert side_view_ic is not None
    contact_patch = state.get(PointID.CONTACT_PATCH_CENTER)

    dx = side_view_ic[Axis.X] - contact_patch[Axis.X]
    dz = side_view_ic[Axis.Z] - contact_patch[Axis.Z]
    assert abs(dz) > TEST_TOLERANCE
    expected = np.sign(dx) * np.hypot(dx, dz)

    metrics = compute_metrics_for_state_from_suspension(state, suspension)

    np.testing.assert_allclose(
        metrics["svsa_length_mm"],
        expected,
        atol=TEST_TOLERANCE,
    )


def test_roll_center_metrics_use_front_view_force_line(
    double_wishbone_geometry_file,
    test_data_dir,
) -> None:
    suspension = load_geometry(double_wishbone_geometry_file)
    assert isinstance(suspension, DoubleWishboneSuspension)

    states, _ = solve_sweep(suspension, parse_sweep_file(test_data_dir / "sweep.yaml"))
    state = next(
        state
        for state in states
        if suspension.compute_front_view_instant_center(state) is not None
    )
    front_view_ic = suspension.compute_front_view_instant_center(state)
    assert front_view_ic is not None
    contact_patch = state.get(PointID.CONTACT_PATCH_CENTER)
    centerline_fraction = -contact_patch[Axis.Y] / (
        front_view_ic[Axis.Y] - contact_patch[Axis.Y]
    )
    expected_roll_center = contact_patch + centerline_fraction * (
        front_view_ic - contact_patch
    )

    metrics = compute_metrics_for_state_from_suspension(state, suspension)

    np.testing.assert_allclose(
        metrics["roll_center_height_mm"],
        expected_roll_center[Axis.Z] - contact_patch[Axis.Z],
        atol=TEST_TOLERANCE,
    )
    np.testing.assert_allclose(
        metrics["roll_center_lateral_offset_mm"],
        expected_roll_center[Axis.Y],
        atol=TEST_TOLERANCE,
    )


def test_anti_pitch_uses_side_view_swing_arm_and_cg_geometry(
    double_wishbone_geometry_file,
    test_data_dir,
) -> None:
    suspension = load_geometry(double_wishbone_geometry_file)
    assert isinstance(suspension, DoubleWishboneSuspension)
    assert suspension.config is not None

    states, _ = solve_sweep(suspension, parse_sweep_file(test_data_dir / "sweep.yaml"))
    state = next(
        state
        for state in states
        if suspension.compute_side_view_instant_center(state) is not None
    )
    side_view_ic = suspension.compute_side_view_instant_center(state)
    assert side_view_ic is not None
    contact_patch = state.get(PointID.CONTACT_PATCH_CENTER)
    cg_position = np.asarray(suspension.config.cg_position, dtype=np.float64)
    expected = (
        -np.sign(contact_patch[Axis.X] - cg_position[Axis.X])
        * (side_view_ic[Axis.Z] - contact_patch[Axis.Z])
        / (side_view_ic[Axis.X] - contact_patch[Axis.X])
        * suspension.config.wheelbase
        / (cg_position[Axis.Z] - contact_patch[Axis.Z])
        * 100.0
    )

    metrics = compute_metrics_for_state_from_suspension(state, suspension)

    np.testing.assert_allclose(
        metrics["anti_pitch_pct"],
        expected,
        atol=TEST_TOLERANCE,
    )


def test_track_change_is_measured_from_design_track(
    double_wishbone_geometry_file,
) -> None:
    suspension = load_geometry(double_wishbone_geometry_file)
    assert isinstance(suspension, DoubleWishboneSuspension)

    state = suspension.initial_state().copy()
    design_wheel_center = state.get(PointID.WHEEL_CENTER).copy()
    lateral_shift_mm = 12.5
    for point_id in (PointID.AXLE_INBOARD, PointID.AXLE_OUTBOARD):
        state[point_id] = state.get(point_id) + np.array(
            [0.0, lateral_shift_mm, 0.0],
            dtype=np.float64,
        )
    DerivedPointsManager(suspension.derived_spec()).update_in_place(state.positions)
    current_wheel_center = state.get(PointID.WHEEL_CENTER)

    metrics = compute_metrics_for_state_from_suspension(state, suspension)
    expected = 2.0 * (
        abs(current_wheel_center[Axis.Y]) - abs(design_wheel_center[Axis.Y])
    )

    np.testing.assert_allclose(
        metrics["track_change_mm"],
        expected,
        atol=TEST_TOLERANCE,
    )


def test_parallel_wishbone_planes_produce_null_ic_metrics(
    double_wishbone_geometry_file,
) -> None:
    suspension = load_geometry(double_wishbone_geometry_file)
    assert isinstance(suspension, DoubleWishboneSuspension)

    state = suspension.initial_state().copy()
    plane_offset = np.array([0.0, 0.0, 300.0], dtype=np.float64)

    # Make the upper wishbone plane a translated copy of the lower
    # wishbone plane so the planes are parallel and have no unique
    # instant-axis intersection.
    state[PointID.UPPER_WISHBONE_INBOARD_FRONT] = (
        state[PointID.LOWER_WISHBONE_INBOARD_FRONT] + plane_offset
    )
    state[PointID.UPPER_WISHBONE_INBOARD_REAR] = (
        state[PointID.LOWER_WISHBONE_INBOARD_REAR] + plane_offset
    )
    state[PointID.UPPER_WISHBONE_OUTBOARD] = (
        state[PointID.LOWER_WISHBONE_OUTBOARD] + plane_offset
    )

    assert suspension.compute_instant_axis(state) is None
    assert suspension.compute_side_view_instant_center(state) is None
    assert suspension.compute_front_view_instant_center(state) is None

    metrics = compute_metrics_for_state_from_suspension(state, suspension)

    assert metrics["svic_x_mm"] is None
    assert metrics["svic_z_mm"] is None
    assert metrics["svsa_length_mm"] is None
    assert metrics["fvic_y_mm"] is None
    assert metrics["fvic_z_mm"] is None
    assert metrics["fvsa_length_mm"] is None
    assert metrics["roll_center_height_mm"] is None
    assert metrics["roll_center_lateral_offset_mm"] is None
    assert metrics["anti_pitch_pct"] is None
    assert metrics["track_change_mm"] == 0.0


def test_steering_axis_ground_intersection_uses_contact_patch_height(
    double_wishbone_geometry_file,
) -> None:
    """
    The steering-axis ground intersection should be evaluated on the
    horizontal plane through the contact patch, not on world Z = 0.
    """
    suspension = load_geometry(double_wishbone_geometry_file)
    assert isinstance(suspension, DoubleWishboneSuspension)
    assert suspension.config is not None

    state = suspension.initial_state().copy()

    lower = state.get(PointID.LOWER_WISHBONE_OUTBOARD).copy()
    upper = state.get(PointID.UPPER_WISHBONE_OUTBOARD).copy()
    direction = upper - lower

    contact_patch = state.get(PointID.CONTACT_PATCH_CENTER).copy()
    contact_patch[2] = 123.456
    state[PointID.CONTACT_PATCH_CENTER] = contact_patch

    expected_t = (contact_patch[2] - lower[2]) / direction[2]
    expected_intersection = lower + expected_t * direction

    ctx = MetricContext(state=state, suspension=suspension, config=suspension.config)
    actual_intersection = ctx.steering_axis_ground_intersection

    assert actual_intersection is not None
    np.testing.assert_allclose(
        actual_intersection,
        expected_intersection,
        atol=TEST_TOLERANCE,
        err_msg="Steering-axis intersection should use contact patch Z height",
    )


def test_scrub_radius_uses_ground_plane_wheel_lateral_direction(
    double_wishbone_geometry_file,
) -> None:
    """
    Scrub radius should use the wheel lateral direction in the ground
    plane, not the full 3D axle direction.
    """
    suspension = load_geometry(double_wishbone_geometry_file)
    assert isinstance(suspension, DoubleWishboneSuspension)
    assert suspension.config is not None

    state = suspension.initial_state().copy()
    axle_inboard = state.get(PointID.AXLE_INBOARD).copy()

    # Force a state with both steer and camber so the ground-plane
    # projection differs measurably from the raw 3D axle direction.
    state[PointID.AXLE_OUTBOARD] = axle_inboard + np.array(
        [120.0, 150.0, 120.0],
        dtype=np.float64,
    )
    DerivedPointsManager(suspension.derived_spec()).update_in_place(state.positions)

    metrics = compute_metrics_for_state_from_suspension(state, suspension)
    scrub_radius = metrics["scrub_radius_mm"]
    roadwheel_angle = metrics["roadwheel_angle_deg"]
    camber = metrics["camber_deg"]

    assert scrub_radius is not None
    assert roadwheel_angle is not None
    assert camber is not None
    assert abs(roadwheel_angle) > 1.0
    assert abs(camber) > 1.0

    ctx = MetricContext(state=state, suspension=suspension, config=suspension.config)
    ground_pt = ctx.steering_axis_ground_intersection
    assert ground_pt is not None

    displacement = ground_pt - ctx.contact_patch_center
    wheel_lateral_ground = ctx.wheel_axis.copy()
    wheel_lateral_ground[2] = 0.0
    wheel_lateral_ground /= np.linalg.norm(wheel_lateral_ground)

    expected_scrub_radius = -float(np.dot(displacement, wheel_lateral_ground))
    old_3d_axle_projection = -float(np.dot(displacement, ctx.wheel_axis))

    np.testing.assert_allclose(
        scrub_radius,
        expected_scrub_radius,
        atol=TEST_TOLERANCE,
        err_msg="Scrub radius should use wheel lateral direction on the ground plane",
    )
    assert not np.isclose(
        scrub_radius,
        old_3d_axle_projection,
        atol=1e-3,
    ), "Scrub radius should not use the full 3D axle direction"


class TestSignConventionsAndKnownValues:
    """
    Direct validation tests for metric sign conventions and
    known-value cases using the test geometry.
    """

    def test_camber_sign_negative_means_top_tilted_inward(
        self, double_wishbone_geometry_file
    ) -> None:
        """
        The test geometry has the upper ball joint inboard of the lower,
        tilting the top of the wheel inward. Camber must be negative.
        """
        suspension = load_geometry(double_wishbone_geometry_file)
        state = suspension.initial_state()
        metrics = compute_metrics_for_state_from_suspension(state, suspension)

        camber = metrics["camber_deg"]
        assert camber is not None
        assert camber < 0, f"Expected negative camber (top tilted inward), got {camber}"

    def test_camber_known_value_at_design_position(
        self, double_wishbone_geometry_file
    ) -> None:
        """
        Verify the camber value at design position against a hand-checked
        reference. The axle vector has a small Z component over a 150 mm
        lateral span, giving roughly -1.9 degrees.
        """
        suspension = load_geometry(double_wishbone_geometry_file)
        state = suspension.initial_state()
        metrics = compute_metrics_for_state_from_suspension(state, suspension)

        camber = metrics["camber_deg"]
        assert camber is not None, "camber_deg is None"
        np.testing.assert_allclose(
            camber,
            -1.909,
            atol=TEST_TOLERANCE,
            err_msg="Camber at design position",
        )

    def test_caster_sign_positive_means_top_tilted_rearward(
        self, double_wishbone_geometry_file
    ) -> None:
        """
        The test geometry has the upper ball joint behind the lower
        (X = -25 vs X = 0), tilting the steering axis top rearward.
        Caster must be positive.
        """
        suspension = load_geometry(double_wishbone_geometry_file)
        state = suspension.initial_state()
        metrics = compute_metrics_for_state_from_suspension(state, suspension)

        caster = metrics["caster_deg"]
        assert caster is not None
        assert caster > 0, (
            f"Expected positive caster (top tilted rearward), got {caster}"
        )

    def test_caster_known_value_at_design_position(
        self, double_wishbone_geometry_file
    ) -> None:
        """
        Verify the caster value at design position. The steering axis
        from lower (0, 900, 200) to upper (-25, 750, 500) gives roughly
        4.76 degrees.
        """
        suspension = load_geometry(double_wishbone_geometry_file)
        state = suspension.initial_state()
        metrics = compute_metrics_for_state_from_suspension(state, suspension)

        caster = metrics["caster_deg"]
        assert caster is not None, "caster_deg is None"
        np.testing.assert_allclose(
            caster,
            4.764,
            atol=TEST_TOLERANCE,
            err_msg="Caster at design position",
        )

    def test_roadwheel_angle_zero_at_design_position(
        self, double_wishbone_geometry_file
    ) -> None:
        """
        At the design position with no steering input the axle is
        purely lateral, so the roadwheel angle must be zero.
        """
        suspension = load_geometry(double_wishbone_geometry_file)
        state = suspension.initial_state()
        metrics = compute_metrics_for_state_from_suspension(state, suspension)

        roadwheel_angle = metrics["roadwheel_angle_deg"]
        assert roadwheel_angle is not None, "roadwheel_angle_deg is None"
        np.testing.assert_allclose(
            roadwheel_angle,
            0.0,
            atol=TEST_TOLERANCE,
            err_msg="Roadwheel angle at design position",
        )

    def test_roadwheel_angle_positive_means_turned_inward(
        self, double_wishbone_geometry_file, test_data_dir
    ) -> None:
        """
        During a toe-in sweep (positive roadwheel angle), the front
        of the wheel points toward the vehicle center. Verify the first
        sweep step produces a positive angle for the left-side suspension.
        """
        suspension = load_geometry(double_wishbone_geometry_file)
        sweep_config = parse_sweep_file(test_data_dir / "sweep.yaml")
        states, _ = solve_sweep(suspension, sweep_config)

        first_metrics = compute_metrics_for_state_from_suspension(states[0], suspension)
        last_metrics = compute_metrics_for_state_from_suspension(states[-1], suspension)

        first_rwa = first_metrics["roadwheel_angle_deg"]
        last_rwa = last_metrics["roadwheel_angle_deg"]
        assert first_rwa is not None
        assert last_rwa is not None

        # The sweep goes from positive to negative roadwheel angle,
        # confirming both sign directions.
        assert first_rwa > 0, "Expected positive roadwheel angle at start of sweep"
        assert last_rwa < 0, "Expected negative roadwheel angle at end of sweep"


def test_default_corner_metric_catalog_matches_trusted_set() -> None:
    column_names = [metric.column_name for metric in get_default_corner_metrics()]

    expected = [
        "camber_deg",
        "caster_deg",
        "kpi_deg",
        "scrub_radius_mm",
        "mechanical_trail_mm",
        "roadwheel_angle_deg",
        "svic_x_mm",
        "svic_z_mm",
        "svsa_length_mm",
        "fvic_y_mm",
        "fvic_z_mm",
        "fvsa_length_mm",
        "roll_center_height_mm",
        "roll_center_lateral_offset_mm",
        "anti_pitch_pct",
        "track_change_mm",
    ]
    assert column_names == expected
