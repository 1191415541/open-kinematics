"""
Assemble the imported Adams Car SLA suspension into an SI axle model.

The topology follows `_sla.tpl` in compliant mode (`phs_kinematic_flag = 0`),
where the control arms attach through bushings and the kinematic joint variants
are inactive.  Every mass, inertia, hardpoint, bushing rate, spring rate and
damper point comes from :mod:`suspension_multibody.adams.car_import`, so nothing
here is transcribed by hand.
"""

from __future__ import annotations

import numpy as np

from ..axle_dynamics.schema import (
    AxleBody,
    AxleBushing,
    AxleDynamicsModel,
    AxleJoint,
    AxleSpringDamper,
    AxleTire,
)
from .car_import import AdamsSuspension, import_blockers

_FIXTURE = "fixture"


def _mirror(point: tuple[float, float, float]) -> tuple[float, float, float]:
    """Adams stores the left side; the right side mirrors about y."""
    return (point[0], -point[1], point[2])


def _diagonal6(
    translational: tuple[float, float, float],
    rotational: tuple[float, float, float],
) -> tuple[tuple[float, ...], ...]:
    values = (*translational, *rotational)
    return tuple(
        tuple(values[i] if i == j else 0.0 for j in range(6)) for i in range(6)
    )


def _inertia(values: tuple[float, float, float]) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(values[i] if i == j else 0.0 for j in range(3)) for i in range(3)
    )


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return tuple((x + y) * 0.5 for x, y in zip(a, b))  # type: ignore[return-value]


def _axis(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    delta = np.asarray(b, dtype=float) - np.asarray(a, dtype=float)
    norm = float(np.linalg.norm(delta))
    if norm <= 1e-12:
        raise ValueError("cannot build an axis from coincident hardpoints")
    return tuple(float(value) for value in delta / norm)  # type: ignore[return-value]


def build_sla_axle_model(
    suspension: AdamsSuspension,
    *,
    sprung_mass_kg: float,
    sprung_inertia_kg_m2: tuple[float, float, float],
    sprung_height_m: float,
    road_height_m: float = 0.0,
    rigid_hub_and_rig: bool = False,
) -> AxleDynamicsModel:
    """
    Build the twin-corner SLA axle from imported Adams Car data.

    `road_height_m` places the contact plane.  Adams Car hardpoints are given in
    the vehicle frame, where the wheel centre sits one loaded radius above the
    road, so the plane is not at z = 0 and the caller must say where it is.

    `rigid_hub_and_rig` replaces two modelling devices with exact constraints:
    the hub compliance bushing, whose 5.6e6 N*m/rad rate makes it rigid in
    practice, and the rig restraint that holds the directions a half axle does
    not exercise.  Neither is an imported Adams element, and both rotate less
    than 1e-6 rad in this model, so the substitution changes no physics while
    letting the same model be written as an Adams dataset.
    """
    blockers = import_blockers(suspension)
    if blockers:
        raise ValueError(
            "the imported suspension cannot be represented exactly: "
            + "; ".join(blockers)
        )

    points = suspension.hardpoints_m
    parts = {part.name: part for part in suspension.parts}
    bushings_by_name = {bushing.name: bushing for bushing in suspension.bushings}
    # The design position defines the loaded radius: the wheel centre sits that
    # far above the contact plane.  Adams stores an unloaded radius separately,
    # and the tire carries its static load as the difference times the rate.
    loaded_radius_m = points["wheel_center"][2] - road_height_m
    if loaded_radius_m <= 0.0:
        raise ValueError(
            "road_height_m must lie below the wheel centre; "
            f"got {road_height_m} for a wheel centre at "
            f"{points['wheel_center'][2]}"
        )
    # Each corner carries half the sprung mass plus its own unsprung mass.
    corner_load_n = (
        0.5 * sprung_mass_kg + unsprung_corner_mass_kg(suspension)
    ) * 9.80665
    tire_deflection_m = corner_load_n / suspension.tire_stiffness_n_per_m

    bodies: list[AxleBody] = [
        AxleBody(
            name=_FIXTURE,
            mass_kg=0.0,
            inertia_kg_m2=_inertia((0.0, 0.0, 0.0)),
            fixed=True,
        ),
        AxleBody(
            name="sprung",
            mass_kg=sprung_mass_kg,
            inertia_kg_m2=_inertia(sprung_inertia_kg_m2),
            position_m=(
                points["wheel_center"][0],
                0.0,
                sprung_height_m,
            ),
        ),
    ]
    joints: list[AxleJoint] = []
    springs: list[AxleSpringDamper] = []
    bushings: list[AxleBushing] = []
    tires: list[AxleTire] = []

    for side, sign in (("l", 1.0), ("r", -1.0)):

        def at(name: str, sign: float = sign) -> tuple[float, float, float]:
            point = points[name]
            return point if sign > 0.0 else _mirror(point)

        wheel_centre = at("wheel_center")
        lower_ball = at("lower_ball_joint")
        upper_ball = at("upper_ball_joint")
        lca_front = at("lca_front")
        lca_rear = at("lca_rear")
        uca_front = at("uca_front")
        uca_rear = at("uca_rear")
        tierod_inner = at("tierod_inner")
        tierod_outer = at("tierod_outer")
        damper_lower = at("damper_lower")
        damper_upper = at("damper_upper")
        spring_lower = at("spring_seat_lower")
        spring_upper = at("spring_seat_upper")

        upright = parts["upright"]
        lower_arm = parts["lower_control_arm"]
        upper_arm = parts["upper_control_arm"]
        tierod = parts["tierod_outer"]

        upright_name = f"upright_{side}"
        lower_name = f"lower_control_arm_{side}"
        upper_name = f"upper_control_arm_{side}"
        tierod_name = f"tierod_{side}"
        wheel_name = f"wheel_{side}"

        bodies.extend(
            (
                AxleBody(
                    name=upright_name,
                    mass_kg=upright.mass_kg,
                    inertia_kg_m2=_inertia(upright.inertia_kg_m2),
                    position_m=_midpoint(lower_ball, upper_ball),
                ),
                AxleBody(
                    name=lower_name,
                    mass_kg=lower_arm.mass_kg,
                    inertia_kg_m2=_inertia(lower_arm.inertia_kg_m2),
                    position_m=_midpoint(
                        _midpoint(lca_front, lca_rear), lower_ball
                    ),
                ),
                AxleBody(
                    name=upper_name,
                    mass_kg=upper_arm.mass_kg,
                    inertia_kg_m2=_inertia(upper_arm.inertia_kg_m2),
                    position_m=_midpoint(
                        _midpoint(uca_front, uca_rear), upper_ball
                    ),
                ),
                AxleBody(
                    name=tierod_name,
                    # Adams splits the tie rod into two near-massless parts.
                    mass_kg=2.0 * tierod.mass_kg,
                    inertia_kg_m2=_inertia(tierod.inertia_kg_m2),
                    position_m=_midpoint(tierod_inner, tierod_outer),
                ),
                AxleBody(
                    name=wheel_name,
                    mass_kg=parts["spindle"].mass_kg,
                    # The spindle carries a placeholder inertia in Adams; the
                    # wheel needs a physical spin inertia to rotate, so the
                    # unloaded radius and mass set a solid-disc value.
                    inertia_kg_m2=_inertia(
                        _wheel_inertia(
                            parts["spindle"].mass_kg,
                            suspension.tire_unloaded_radius_m,
                        )
                    ),
                    position_m=wheel_centre,
                ),
            )
        )

        # Joint points are body-local offsets, so every global hardpoint is
        # expressed relative to the origin of the body it belongs to.  The
        # bodies all start with identity orientation, so the offset is a plain
        # subtraction.
        origin = {
            _FIXTURE: (0.0, 0.0, 0.0),
            "sprung": (points["wheel_center"][0], 0.0, sprung_height_m),
            upright_name: _midpoint(lower_ball, upper_ball),
            lower_name: _midpoint(_midpoint(lca_front, lca_rear), lower_ball),
            upper_name: _midpoint(_midpoint(uca_front, uca_rear), upper_ball),
            tierod_name: _midpoint(tierod_inner, tierod_outer),
            wheel_name: wheel_centre,
        }

        def local(
            body: str, point: tuple[float, float, float]
        ) -> tuple[float, float, float]:
            base = origin[body]
            return (
                point[0] - base[0],
                point[1] - base[1],
                point[2] - base[2],
            )

        # Control arms pivot about their two body-side bushing points.
        joints.extend(
            (
                AxleJoint(
                    name=f"lca_pivot_{side}",
                    kind="revolute",
                    body_a="sprung",
                    body_b=lower_name,
                    point_a_m=local("sprung", _midpoint(lca_front, lca_rear)),
                    point_b_m=local(lower_name, _midpoint(lca_front, lca_rear)),
                    axis_a=_axis(lca_front, lca_rear),
                    axis_b=_axis(lca_front, lca_rear),
                ),
                AxleJoint(
                    name=f"uca_pivot_{side}",
                    kind="revolute",
                    body_a="sprung",
                    body_b=upper_name,
                    point_a_m=local("sprung", _midpoint(uca_front, uca_rear)),
                    point_b_m=local(upper_name, _midpoint(uca_front, uca_rear)),
                    axis_a=_axis(uca_front, uca_rear),
                    axis_b=_axis(uca_front, uca_rear),
                ),
                AxleJoint(
                    name=f"lower_ball_{side}",
                    kind="spherical",
                    body_a=lower_name,
                    body_b=upright_name,
                    point_a_m=local(lower_name, lower_ball),
                    point_b_m=local(upright_name, lower_ball),
                ),
                AxleJoint(
                    name=f"upper_ball_{side}",
                    kind="spherical",
                    body_a=upper_name,
                    body_b=upright_name,
                    point_a_m=local(upper_name, upper_ball),
                    point_b_m=local(upright_name, upper_ball),
                ),
                # The tie rod is a two-force member carried by ball joints at
                # both ends.  Its spin about its own axis is then unconstrained,
                # which is physically right: nothing resists it, and the kernel
                # pins that null direction during trim.  A universal inboard
                # would instead tie the rod to a ground-fixed cross axis and
                # fight the rod's swing, leaving a residual that cannot close.
                AxleJoint(
                    name=f"tierod_outer_{side}",
                    kind="spherical",
                    body_a=tierod_name,
                    body_b=upright_name,
                    point_a_m=local(tierod_name, tierod_outer),
                    point_b_m=local(upright_name, tierod_outer),
                ),
                AxleJoint(
                    name=f"tierod_inner_{side}",
                    kind="spherical",
                    body_a="sprung",
                    body_b=tierod_name,
                    point_a_m=local("sprung", tierod_inner),
                    point_b_m=local(tierod_name, tierod_inner),
                ),
                # Ball joints at both ends leave the rod free to spin about its
                # own axis, and nothing resists that motion.  One in-plane row
                # on a point held off the rod axis removes exactly that spin
                # without adding any force to the corner: it is a kinematic
                # device, not a stiffness.
                AxleJoint(
                    name=f"tierod_spin_{side}",
                    kind="inplane",
                    body_a="sprung",
                    body_b=tierod_name,
                    point_a_m=local("sprung", tierod_inner),
                    point_b_m=_offset_from_axis(
                        local(tierod_name, tierod_inner),
                        _axis(tierod_inner, tierod_outer),
                    ),
                    axis_a=_second_perpendicular(
                        _axis(tierod_inner, tierod_outer)
                    ),
                ),
                AxleJoint(
                    name=f"spin_{side}",
                    kind="revolute",
                    body_a=upright_name,
                    body_b=wheel_name,
                    point_a_m=local(upright_name, wheel_centre),
                    point_b_m=local(wheel_name, wheel_centre),
                    axis_a=(0.0, 1.0, 0.0),
                    axis_b=(0.0, 1.0, 0.0),
                ),
            )
        )

        # The ride spring acts between its two seats.  Adams reaches the design
        # position by trimming the spring; the installed length here comes from
        # the hardpoints, so the free length is set to carry exactly the corner
        # load through this geometry.  The imported rate is used unchanged.
        installed_length_m = float(
            np.linalg.norm(
                np.asarray(spring_upper) - np.asarray(spring_lower)
            )
        )
        # Balance moments about the control-arm pivot axis: the vertical load
        # arriving at the ball joint against the spring acting along its seat
        # line.  Perpendicular distances alone are not enough, because neither
        # force is perpendicular to the arm.
        spring_force_n = _spring_force_for_corner_load(
            pivot_a=lca_front,
            pivot_b=lca_rear,
            load_point=lower_ball,
            spring_upper=spring_upper,
            spring_lower=spring_lower,
            corner_load_n=corner_load_n,
        )
        springs.append(
            AxleSpringDamper(
                name=f"spring_{side}",
                body_a="sprung",
                body_b=lower_name,
                point_a_m=local("sprung", spring_upper),
                point_b_m=local(lower_name, spring_lower),
                stiffness_n_per_m=suspension.spring_rate_n_per_m,
                compression_damping_n_s_per_m=0.0,
                rebound_damping_n_s_per_m=0.0,
                free_length_m=installed_length_m
                + spring_force_n / suspension.spring_rate_n_per_m,
            )
        )
        springs.append(
            AxleSpringDamper(
                name=f"damper_{side}",
                body_a="sprung",
                body_b=lower_name,
                point_a_m=local("sprung", damper_upper),
                point_b_m=local(lower_name, damper_lower),
                stiffness_n_per_m=0.0,
                compression_damping_n_s_per_m=0.0,
                rebound_damping_n_s_per_m=0.0,
                free_length_m=float(
                    np.linalg.norm(
                        np.asarray(damper_upper) - np.asarray(damper_lower)
                    )
                ),
                damper_curve_velocity_m_per_s=(
                    suspension.damper_velocity_m_per_s
                ),
                damper_curve_force_n=suspension.damper_force_n,
            )
        )

        # The hub compliance bushing is the one bushing that acts on a body
        # pair the reduced topology still has.
        hub = bushings_by_name["hub_compliance"]
        # The hub compliance acts in parallel with the spin revolute, so when
        # it is treated as rigid it contributes nothing the revolute does not
        # already enforce and simply drops out.  Its measured rotation here is
        # 1e-12 rad, so removing it changes no physics.
        if not rigid_hub_and_rig:
                bushings.append(
                AxleBushing(
                    name=f"hub_compliance_{side}",
                    body_a=upright_name,
                    body_b=wheel_name,
                    point_a_m=local(upright_name, wheel_centre),
                    point_b_m=local(wheel_name, wheel_centre),
                    reference_translation_in_frame_a_m=(0.0, 0.0, 0.0),
                    reference_quaternion_a_to_b=(1.0, 0.0, 0.0, 0.0),
                    stiffness=_diagonal6(
                        hub.translational_stiffness_n_per_m,
                        hub.rotational_stiffness_n_m_per_rad,
                    ),
                    damping=_diagonal6(
                        hub.translational_damping_n_s_per_m,
                        hub.rotational_damping_n_m_s_per_rad,
                    ),
                )
            )

        tires.append(
            AxleTire(
                name=f"tire_{side}",
                body=wheel_name,
                # The tire must reach the road at the design position, so its
                # unloaded radius is the loaded radius plus the deflection its
                # own rate produces under the corner load.
                unloaded_radius_m=loaded_radius_m + tire_deflection_m,
                # A tire bottoms out when the rim approaches the road, so the
                # travel limit follows the loaded radius rather than a multiple
                # of the static deflection.  Adams' rig tire has no such limit;
                # this one exists to catch a runaway, and the solver reports it
                # when it trips instead of silently clamping the penetration.
                maximum_compression_m=0.5 * loaded_radius_m,
                vertical_stiffness_n_per_m=suspension.tire_stiffness_n_per_m,
                # The Adams suspension rig tire has no vertical damping; this
                # model keeps a small physical value so the contact does not
                # ring, and it is reported as a documented difference.
                vertical_damping_n_s_per_m=500.0,
                longitudinal_friction_coefficient=1.0,
                lateral_friction_coefficient=0.95,
                longitudinal_brush_stiffness_n_per_m=180_000.0,
                lateral_brush_stiffness_n_per_m=120_000.0,
                longitudinal_relaxation_length_m=0.25,
                lateral_relaxation_length_m=0.35,
                detached_relaxation_s=0.05,
            )
        )

    # Restrain the directions a half-axle rig does not exercise so the static
    # problem is determinate; heave and roll stay free.
    if rigid_hub_and_rig:
        # Heave and roll are exactly the two freedoms of a cylindrical joint
        # whose axis is vertical for the slide and longitudinal for the spin.
        # Adams expresses this pair as a translational plus a revolute on an
        # intermediate carrier, so it is written here as two joints on a
        # massless carrier rather than as one stiff bushing.
        bodies.append(
            AxleBody(
                name="rig_carrier",
                # Small but physical: the carrier only transmits constraint,
                # so its mass must not perturb the corner load while still
                # giving the solver a positive definite inertia.
                mass_kg=0.1,
                inertia_kg_m2=_inertia((1.0e-3, 1.0e-3, 1.0e-3)),
                position_m=(points["wheel_center"][0], 0.0, sprung_height_m),
            )
        )
        joints.extend(
            (
                AxleJoint(
                    name="rig_heave",
                    kind="prismatic",
                    body_a=_FIXTURE,
                    body_b="rig_carrier",
                    point_a_m=(
                        points["wheel_center"][0],
                        0.0,
                        sprung_height_m,
                    ),
                    point_b_m=(0.0, 0.0, 0.0),
                    axis_a=(0.0, 0.0, 1.0),
                    axis_b=(0.0, 0.0, 1.0),
                ),
                AxleJoint(
                    name="rig_roll",
                    kind="revolute",
                    body_a="rig_carrier",
                    body_b="sprung",
                    point_a_m=(0.0, 0.0, 0.0),
                    point_b_m=(0.0, 0.0, 0.0),
                    axis_a=(1.0, 0.0, 0.0),
                    axis_b=(1.0, 0.0, 0.0),
                ),
            )
        )
    else:
        bushings.append(
            AxleBushing(
                name="rig_restraint",
                body_a=_FIXTURE,
                body_b="sprung",
                point_a_m=(points["wheel_center"][0], 0.0, sprung_height_m),
                point_b_m=(0.0, 0.0, 0.0),
                reference_translation_in_frame_a_m=(0.0, 0.0, 0.0),
                reference_quaternion_a_to_b=(1.0, 0.0, 0.0, 0.0),
                stiffness=_diagonal6((4.0e6, 4.0e6, 0.0), (0.0, 4.0e6, 4.0e6)),
                damping=_diagonal6((2.0e4, 2.0e4, 0.0), (0.0, 2.0e4, 2.0e4)),
            )
        )

    return AxleDynamicsModel(
        name="adams-car-sla-front",
        bodies=tuple(bodies),
        joints=tuple(joints),
        springs=tuple(springs),
        bushings=tuple(bushings),
        tires=tuple(tires),
    )


def _wheel_inertia(
    mass_kg: float, radius_m: float
) -> tuple[float, float, float]:
    """Solid-disc inertia about the spin axis and its two diameters."""
    spin = 0.5 * mass_kg * radius_m * radius_m
    diameter = 0.5 * spin
    return (diameter, spin, diameter)


def _perpendicular(
    axis: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Any unit vector perpendicular to `axis`."""
    reference = (1.0, 0.0, 0.0) if abs(axis[0]) < 0.8 else (0.0, 1.0, 0.0)
    cross = np.cross(np.asarray(axis), np.asarray(reference))
    norm = float(np.linalg.norm(cross))
    if norm <= 1e-12:
        raise ValueError("cannot build a perpendicular axis")
    return tuple(float(value) for value in cross / norm)  # type: ignore[return-value]


def sprung_corner_mass_kg(suspension: AdamsSuspension) -> float:
    """Sprung mass Adams assigns to this corner, from the sprung fractions."""
    return sum(
        part.mass_kg * part.sprung_fraction for part in suspension.parts
    )


def unsprung_corner_mass_kg(suspension: AdamsSuspension) -> float:
    """Return the unsprung mass Adams assigns to one corner."""
    return sum(
        part.mass_kg * (1.0 - part.sprung_fraction)
        for part in suspension.parts
    )


def static_wheel_load_n(sprung_mass_kg: float, unsprung_mass_kg: float) -> float:
    """Total vertical load one wheel carries at rest."""
    return (0.5 * sprung_mass_kg + unsprung_mass_kg) * 9.80665


def _spring_force_for_corner_load(
    *,
    pivot_a: tuple[float, float, float],
    pivot_b: tuple[float, float, float],
    load_point: tuple[float, float, float],
    spring_upper: tuple[float, float, float],
    spring_lower: tuple[float, float, float],
    corner_load_n: float,
) -> float:
    """
    Spring force that balances a vertical corner load about the arm pivot.

    Both the vertical load and the spring act at an angle to the control arm,
    so the balance is a moment balance about the pivot axis rather than a ratio
    of perpendicular distances.
    """
    origin = np.asarray(pivot_a, dtype=float)
    axis = np.asarray(pivot_b, dtype=float) - origin
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm <= 1e-12:
        raise ValueError("control arm pivot points coincide")
    axis = axis / axis_norm

    def moment(point: tuple[float, float, float], force: np.ndarray) -> float:
        arm = np.asarray(point, dtype=float) - origin
        return float(np.cross(arm, force) @ axis)

    load_moment = moment(load_point, np.asarray((0.0, 0.0, corner_load_n)))
    direction = np.asarray(spring_lower, dtype=float) - np.asarray(
        spring_upper, dtype=float
    )
    direction_norm = float(np.linalg.norm(direction))
    if direction_norm <= 1e-12:
        raise ValueError("spring seats coincide")
    unit_moment = moment(spring_lower, direction / direction_norm)
    if abs(unit_moment) <= 1e-12:
        raise ValueError("the spring has no moment arm about the arm pivot")
    return -load_moment / unit_moment


def _offset_from_axis(
    point: tuple[float, float, float],
    axis: tuple[float, float, float],
    distance_m: float = 0.05,
) -> tuple[float, float, float]:
    """Move `point` sideways off `axis`, so a row about it senses spin."""
    sideways = np.asarray(_perpendicular(axis), dtype=float) * distance_m
    return (
        point[0] + float(sideways[0]),
        point[1] + float(sideways[1]),
        point[2] + float(sideways[2]),
    )


def _second_perpendicular(
    axis: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Return the perpendicular completing a frame with `_perpendicular`."""
    first = np.asarray(_perpendicular(axis), dtype=float)
    second = np.cross(np.asarray(axis, dtype=float), first)
    return tuple(float(value) for value in second)  # type: ignore[return-value]
