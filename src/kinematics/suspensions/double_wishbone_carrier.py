"""
Double wishbone suspension with a separate carrier and a two-point steering axis.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import partial
from typing import TYPE_CHECKING, ClassVar, Sequence

import numpy as np

from kinematics.constraints import (
    AngleConstraint,
    Constraint,
    DistanceConstraint,
    PointOnLineConstraint,
)
from kinematics.core.constants import EPS_GEOMETRIC
from kinematics.core.enums import Axis, PointID, ShimType
from kinematics.core.types import Vec3, WorldAxisSystem
from kinematics.core.vector_utils.geometric import (
    compute_point_point_distance,
    intersect_line_with_axis_aligned_plane,
    intersect_line_with_vertical_plane,
    intersect_two_planes,
    plane_from_three_points,
)
from kinematics.points.derived.definitions import (
    get_axle_midpoint,
    get_contact_patch_center,
    get_wheel_center,
    get_wheel_inboard,
    get_wheel_outboard,
)
from kinematics.points.derived.manager import DerivedPointsManager, DerivedPointsSpec
from kinematics.state import SuspensionState
from kinematics.suspensions.base import Suspension

if TYPE_CHECKING:
    from kinematics.visualization.main import LinkVisualization


@dataclass
class DoubleWishboneCarrierSuspension(Suspension):
    """
    Double wishbone suspension with a separate carrier and upright.

    The wishbone outboard points are the upper/lower carrier mounts. The carrier also
    defines a steering-axis line using two points. The upright is a separate rigid body
    attached to that carrier axis and carries the axle and track rod outer pickup.
    """

    TYPE_KEY: ClassVar[str] = "double_wishbone_carrier"
    ALIASES: ClassVar[frozenset[str]] = frozenset()

    REQUIRED_POINTS: ClassVar[frozenset[PointID]] = frozenset(
        {
            PointID.LOWER_WISHBONE_INBOARD_FRONT,
            PointID.LOWER_WISHBONE_INBOARD_REAR,
            PointID.LOWER_WISHBONE_OUTBOARD,
            PointID.UPPER_WISHBONE_INBOARD_FRONT,
            PointID.UPPER_WISHBONE_INBOARD_REAR,
            PointID.UPPER_WISHBONE_OUTBOARD,
            PointID.TRACKROD_INBOARD,
            PointID.TRACKROD_OUTBOARD,
            PointID.AXLE_INBOARD,
            PointID.AXLE_OUTBOARD,
            PointID.CARRIER_STEERING_AXIS_LOWER,
            PointID.CARRIER_STEERING_AXIS_UPPER,
        }
    )

    OPTIONAL_POINTS: ClassVar[frozenset[PointID]] = frozenset(
        {
            PointID.PUSHROD_INBOARD,
            PointID.PUSHROD_OUTBOARD,
        }
    )

    SUPPORTED_SHIMS: ClassVar[frozenset[ShimType]] = frozenset()

    OUTPUT_POINTS: ClassVar[tuple[PointID, ...]] = (
        PointID.LOWER_WISHBONE_INBOARD_FRONT,
        PointID.LOWER_WISHBONE_INBOARD_REAR,
        PointID.LOWER_WISHBONE_OUTBOARD,
        PointID.UPPER_WISHBONE_INBOARD_FRONT,
        PointID.UPPER_WISHBONE_INBOARD_REAR,
        PointID.UPPER_WISHBONE_OUTBOARD,
        PointID.CARRIER_STEERING_AXIS_LOWER,
        PointID.CARRIER_STEERING_AXIS_UPPER,
        PointID.TRACKROD_INBOARD,
        PointID.TRACKROD_OUTBOARD,
        PointID.AXLE_INBOARD,
        PointID.AXLE_OUTBOARD,
        PointID.AXLE_MIDPOINT,
        PointID.WHEEL_CENTER,
        PointID.WHEEL_INBOARD,
        PointID.WHEEL_OUTBOARD,
        PointID.CONTACT_PATCH_CENTER,
    )

    FREE_POINTS: ClassVar[tuple[PointID, ...]] = (
        PointID.UPPER_WISHBONE_OUTBOARD,
        PointID.LOWER_WISHBONE_OUTBOARD,
        PointID.CARRIER_STEERING_AXIS_LOWER,
        PointID.CARRIER_STEERING_AXIS_UPPER,
        PointID.AXLE_INBOARD,
        PointID.AXLE_OUTBOARD,
        PointID.TRACKROD_OUTBOARD,
        PointID.TRACKROD_INBOARD,
    )

    def free_points(self) -> Sequence[PointID]:
        return self.FREE_POINTS

    def initial_state(self) -> SuspensionState:
        if self._initial_state is not None:
            return self._initial_state

        positions = self.get_hardpoints_as_arrays()
        derived_manager = DerivedPointsManager(self.derived_spec())
        derived_manager.update_in_place(positions)
        self._initial_state = SuspensionState(
            positions=positions,
            free_points=set(self.free_points()),
        )
        return self._initial_state

    def constraints(self) -> list[Constraint]:
        initial_state = self.initial_state()
        positions = initial_state.positions
        constraints: list[Constraint] = []

        length_pairs = [
            (PointID.UPPER_WISHBONE_INBOARD_FRONT, PointID.UPPER_WISHBONE_OUTBOARD),
            (PointID.UPPER_WISHBONE_INBOARD_REAR, PointID.UPPER_WISHBONE_OUTBOARD),
            (PointID.LOWER_WISHBONE_INBOARD_FRONT, PointID.LOWER_WISHBONE_OUTBOARD),
            (PointID.LOWER_WISHBONE_INBOARD_REAR, PointID.LOWER_WISHBONE_OUTBOARD),
            (PointID.CARRIER_STEERING_AXIS_LOWER, PointID.CARRIER_STEERING_AXIS_UPPER),
            (PointID.CARRIER_STEERING_AXIS_LOWER, PointID.LOWER_WISHBONE_OUTBOARD),
            (PointID.CARRIER_STEERING_AXIS_UPPER, PointID.UPPER_WISHBONE_OUTBOARD),
            (PointID.CARRIER_STEERING_AXIS_LOWER, PointID.UPPER_WISHBONE_OUTBOARD),
            (PointID.CARRIER_STEERING_AXIS_UPPER, PointID.LOWER_WISHBONE_OUTBOARD),
            (PointID.AXLE_INBOARD, PointID.AXLE_OUTBOARD),
            (PointID.TRACKROD_INBOARD, PointID.TRACKROD_OUTBOARD),
            (PointID.TRACKROD_OUTBOARD, PointID.UPPER_WISHBONE_OUTBOARD),
            (PointID.TRACKROD_OUTBOARD, PointID.LOWER_WISHBONE_OUTBOARD),
            (PointID.TRACKROD_OUTBOARD, PointID.AXLE_INBOARD),
            (PointID.TRACKROD_OUTBOARD, PointID.AXLE_OUTBOARD),
            (PointID.AXLE_INBOARD, PointID.UPPER_WISHBONE_OUTBOARD),
            (PointID.AXLE_INBOARD, PointID.LOWER_WISHBONE_OUTBOARD),
            (PointID.AXLE_OUTBOARD, PointID.UPPER_WISHBONE_OUTBOARD),
            (PointID.AXLE_OUTBOARD, PointID.LOWER_WISHBONE_OUTBOARD),
        ]
        for point_a, point_b in length_pairs:
            constraints.append(
                DistanceConstraint(
                    point_a,
                    point_b,
                    compute_point_point_distance(positions[point_a], positions[point_b]),
                )
            )

        upright_span = (
            positions[PointID.LOWER_WISHBONE_OUTBOARD]
            - positions[PointID.UPPER_WISHBONE_OUTBOARD]
        )
        axle_axis = positions[PointID.AXLE_OUTBOARD] - positions[PointID.AXLE_INBOARD]
        carrier_axis = (
            positions[PointID.CARRIER_STEERING_AXIS_UPPER]
            - positions[PointID.CARRIER_STEERING_AXIS_LOWER]
        )
        constraints.append(
            AngleConstraint(
                PointID.UPPER_WISHBONE_OUTBOARD,
                PointID.LOWER_WISHBONE_OUTBOARD,
                PointID.AXLE_INBOARD,
                PointID.AXLE_OUTBOARD,
                _safe_angle_between(upright_span, axle_axis),
            )
        )
        constraints.append(
            AngleConstraint(
                PointID.CARRIER_STEERING_AXIS_LOWER,
                PointID.CARRIER_STEERING_AXIS_UPPER,
                PointID.AXLE_INBOARD,
                PointID.AXLE_OUTBOARD,
                _safe_angle_between(carrier_axis, axle_axis),
            )
        )

        constraints.append(
            PointOnLineConstraint(
                point_id=PointID.TRACKROD_INBOARD,
                line_point=positions[PointID.TRACKROD_INBOARD],
                line_direction=WorldAxisSystem.Y,
            )
        )

        return constraints

    def derived_spec(self) -> DerivedPointsSpec:
        if self.config is None:
            raise ValueError("Cannot compute derived spec without config")

        wheel_cfg = self.config.wheel
        tire_radius = wheel_cfg.tire.nominal_radius
        functions = {
            PointID.AXLE_MIDPOINT: get_axle_midpoint,
            PointID.WHEEL_CENTER: partial(
                get_wheel_center, wheel_offset=wheel_cfg.offset
            ),
            PointID.WHEEL_INBOARD: partial(
                get_wheel_inboard, wheel_width=wheel_cfg.tire.section_width
            ),
            PointID.WHEEL_OUTBOARD: partial(
                get_wheel_outboard, wheel_width=wheel_cfg.tire.section_width
            ),
            PointID.CONTACT_PATCH_CENTER: partial(
                get_contact_patch_center, tire_radius=tire_radius
            ),
        }
        dependencies = {
            PointID.AXLE_MIDPOINT: {PointID.AXLE_INBOARD, PointID.AXLE_OUTBOARD},
            PointID.WHEEL_CENTER: {PointID.AXLE_INBOARD, PointID.AXLE_OUTBOARD},
            PointID.WHEEL_INBOARD: {PointID.WHEEL_CENTER, PointID.AXLE_INBOARD},
            PointID.WHEEL_OUTBOARD: {PointID.WHEEL_CENTER, PointID.AXLE_INBOARD},
            PointID.CONTACT_PATCH_CENTER: {
                PointID.WHEEL_CENTER,
                PointID.AXLE_INBOARD,
                PointID.AXLE_OUTBOARD,
            },
        }
        return DerivedPointsSpec(functions=functions, dependencies=dependencies)

    def compute_instant_axis(self, state: SuspensionState) -> tuple[Vec3, Vec3] | None:
        upper_plane = plane_from_three_points(
            state.positions[PointID.UPPER_WISHBONE_INBOARD_FRONT],
            state.positions[PointID.UPPER_WISHBONE_INBOARD_REAR],
            state.positions[PointID.UPPER_WISHBONE_OUTBOARD],
        )
        lower_plane = plane_from_three_points(
            state.positions[PointID.LOWER_WISHBONE_INBOARD_FRONT],
            state.positions[PointID.LOWER_WISHBONE_INBOARD_REAR],
            state.positions[PointID.LOWER_WISHBONE_OUTBOARD],
        )
        if upper_plane is None or lower_plane is None:
            return None
        return intersect_two_planes(
            n1=upper_plane[0],
            d1=upper_plane[1],
            n2=lower_plane[0],
            d2=lower_plane[1],
        )

    def steering_axis_points(self, state: SuspensionState) -> tuple[Vec3, Vec3]:
        return (
            state.get(PointID.CARRIER_STEERING_AXIS_LOWER),
            state.get(PointID.CARRIER_STEERING_AXIS_UPPER),
        )

    def compute_side_view_instant_center(self, state: SuspensionState) -> Vec3 | None:
        instant_axis = self.compute_instant_axis(state)
        if instant_axis is None:
            return None
        axis_point, axis_direction = instant_axis
        wheel_center_y = float(state.positions[PointID.WHEEL_CENTER][Axis.Y])
        return intersect_line_with_vertical_plane(
            axis_point, axis_direction, wheel_center_y
        )

    def compute_front_view_instant_center(self, state: SuspensionState) -> Vec3 | None:
        instant_axis = self.compute_instant_axis(state)
        if instant_axis is None:
            return None
        axis_point, axis_direction = instant_axis
        wheel_center_x = float(state.positions[PointID.WHEEL_CENTER][Axis.X])
        return intersect_line_with_axis_aligned_plane(
            axis_point, axis_direction, Axis.X, wheel_center_x
        )

    def get_visualization_links(self) -> list[LinkVisualization]:
        from kinematics.visualization.main import LinkVisualization

        return [
            LinkVisualization(
                points=[
                    PointID.UPPER_WISHBONE_INBOARD_FRONT,
                    PointID.UPPER_WISHBONE_OUTBOARD,
                    PointID.UPPER_WISHBONE_INBOARD_REAR,
                ],
                color="dodgerblue",
                label="Upper Wishbone",
            ),
            LinkVisualization(
                points=[
                    PointID.LOWER_WISHBONE_INBOARD_FRONT,
                    PointID.LOWER_WISHBONE_OUTBOARD,
                    PointID.LOWER_WISHBONE_INBOARD_REAR,
                ],
                color="dodgerblue",
                label="Lower Wishbone",
            ),
            LinkVisualization(
                points=[
                    PointID.LOWER_WISHBONE_OUTBOARD,
                    PointID.CARRIER_STEERING_AXIS_LOWER,
                    PointID.CARRIER_STEERING_AXIS_UPPER,
                    PointID.UPPER_WISHBONE_OUTBOARD,
                    PointID.LOWER_WISHBONE_OUTBOARD,
                ],
                color="slategrey",
                label="Carrier",
                linewidth=3.5,
                marker="s",
                markersize=9.0,
            ),
            LinkVisualization(
                points=[
                    PointID.TRACKROD_OUTBOARD,
                    PointID.UPPER_WISHBONE_OUTBOARD,
                    PointID.LOWER_WISHBONE_OUTBOARD,
                    PointID.TRACKROD_OUTBOARD,
                ],
                color="mediumseagreen",
                label="Upright",
                linewidth=2.5,
                linestyle="--",
                marker="^",
                markersize=8.0,
            ),
            LinkVisualization(
                points=[
                    PointID.CARRIER_STEERING_AXIS_LOWER,
                    PointID.CARRIER_STEERING_AXIS_UPPER,
                ],
                color="darkorange",
                label="Steering Axis",
            ),
            LinkVisualization(
                points=[PointID.TRACKROD_INBOARD, PointID.TRACKROD_OUTBOARD],
                color="darkorange",
                label="Track Rod",
            ),
            LinkVisualization(
                points=[PointID.AXLE_INBOARD, PointID.AXLE_OUTBOARD],
                color="forestgreen",
                label="Axle",
            ),
            LinkVisualization(
                points=[PointID.CONTACT_PATCH_CENTER],
                color="black",
                label="Contact Patch",
                linewidth=0.0,
                marker="o",
                markersize=15.0,
            ),
        ]


def _safe_angle_between(first: np.ndarray, second: np.ndarray) -> float:
    first_norm = float(np.linalg.norm(first))
    second_norm = float(np.linalg.norm(second))
    if first_norm < EPS_GEOMETRIC or second_norm < EPS_GEOMETRIC:
        return 0.0
    first_unit = first / first_norm
    second_unit = second / second_norm
    dot = float(np.clip(np.dot(first_unit, second_unit), -1.0, 1.0))
    return float(np.arccos(dot))
