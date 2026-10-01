"""
Evidence for raw/numerics_and_comparison.md.

Measures, in order:

1. the differentiation step's measured error and the Jacobian's singular-value floor;
2. the exact reduction: a planar four-bar, where today's front-view construction is
   the exact answer, so the engine can be checked against it with no argument at all;
3. the side-by-side comparison on the double-wishbone axle (shipped geometry plus two
   hardened variants), decomposed so that every contribution is a number:
   (a) today's construction as it stands -- the *mean* of the arm's two inboard
       hardpoints;
   (b) the same arithmetic with the mean replaced by the arm's real pivot, which
       isolates the contribution of the mean;
   (c1) the engine's axis on the left four-bar alone (the two arm joints, the exact
        analogue of what today's construction intersects);
   (c2) the engine's axis on the whole axle (16 declarations);
   (d) the engine's *minimum-norm* member, which is what the engine returns by
       default and is NOT the comparison member;
4. the size of the motion family and the invariance of the axis under the drive.

THE COMPARISON MEMBER.  ``solve_rigid_motion`` returns the minimum-Euclidean-norm
member of an affine family of motions.  Today's construction is a front-view
construction for a rotation about ``x``, so the member that can be compared with it
is the one whose upright angular velocity is a rotation about ``x``, obtained by
adding a free member to the minimum-norm one.  Free members act on the upright's
angular velocity through a linear map, so the reachable angular velocities are the
affine set ``{b + M c}``; the comparison member is the minimum-``|c|`` solution of

    minimise over c:   | M_perp c + b_perp |

and the reported quality is ``|M_perp c + b_perp| / |b|``.  That is the fraction of
the upright's angular speed that is *not* a roll about ``x``.  It is ~1e-16 when the
arms' real pivots are parallel to ``x`` (geometries A and C) and ~1e-1 when they are
not (geometry B), and it is reported rather than hidden.

The engine's axis is read from the *upright*, because that is the body whose motion
is the suspension's own; the wheel hub also carries the wheel's spin joint, and a
spin is a rotation about the wheel axis whose pitch is nonzero, so its screw is a
different (and correct) object.

Run:  uv run --no-sync python <this file>
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from fixtures import double_wishbone_axle_model, planar_four_bar, two_d_intersection  # noqa: E402

from suspension_multibody.subsystems.entry import compose_axle  # noqa: E402
from suspension_multibody.vehicle import screw_kinematics as sk  # noqa: E402
from suspension_multibody.vehicle.roll_centers import _hardpoint, _instant_center  # noqa: E402

HARDENING_CASES = (
    ("A shipped geometry", {}),
    ("B upper inboard rear moved to (1700,-500,520)", {"UPPER_INBOARD_REAR": (1700.0, -500.0, 520.0)}),
    ("C lower outboard moved to (1400,-780,90)", {"LOWER_OUTBOARD": (1400.0, -780.0, 90.0)}),
)

#: The front-view plane both sides' spindles share; the comparison pierces here.
FRONT_VIEW_PLANE_X = 1400.0

#: The left suspension arm joints alone: the two lines today's construction intersects.
ARM_JOINTS = (
    "uca_mount_L_inner_front",
    "lca_mount_L_inner_front",
    "upper_arm_L_outer_joint",
    "lower_arm_L_outer_joint",
)


def section(title: str) -> None:
    print(f"\n===== {title} =====")


def pierce(axis_point: np.ndarray, direction: np.ndarray, plane_x: float) -> np.ndarray:
    """Where the axis line crosses the plane ``x = plane_x``, as ``[y, z]``."""
    parameter = (plane_x - float(axis_point[0])) / float(direction[0])
    crossing = np.asarray(axis_point, dtype=float) + parameter * np.asarray(direction, dtype=float)
    return crossing[1:]


def today_without_the_mean(model, side: str) -> np.ndarray:
    """
    Today's arithmetic with the inboard mean replaced by the arm's real pivot.

    ``roll_centers._instant_center`` builds each arm line from the *mean* of the
    arm's two inboard hardpoints and the arm's outboard hardpoint.  The assembled
    mechanism's arm pivots on a single revolute joint at its inboard-front
    hardpoint, so this function states the same construction with that pivot; the
    difference between the two is exactly the contribution of the mean.
    """
    upper_inner = _hardpoint(model, "upper_front", side)[1:]
    lower_inner = _hardpoint(model, "lower_front", side)[1:]
    upper_outer = _hardpoint(model, "upper_outer", side)[1:]
    lower_outer = _hardpoint(model, "lower_outer", side)[1:]
    return two_d_intersection(upper_inner, upper_outer, lower_inner, lower_outer)


def comparison_member(constraints, axle, bodies, pivot_body: str, anchor):
    """
    Return ``(screw, off_axis_fraction, motion)`` for the member of the motion family
    whose pivot-body angular velocity is closest to a rotation about ``x``.

    See the module docstring for the selection; ``off_axis_fraction`` is the residual
    of that selection divided by the achieved angular speed, so ``0`` means the member
    really is a roll about ``x`` and order ``1`` means it is not.
    """
    motion = sk.solve_rigid_motion(
        constraints,
        axle.state,
        [sk.TangentDrive(pivot_body, 3, 1.0)],
        points=axle.points,
        bodies=bodies,
        pivot_body=pivot_body,
        pivot_point=anchor,
    )
    index = 6 * bodies.index(pivot_body)
    pose = axle.state.pose(pivot_body)
    velocity = np.asarray(motion.velocity, dtype=float)
    base = pose.rotation @ velocity[index + 3 : index + 6]
    free = np.asarray(motion.null_space, dtype=float)
    sensitivity = np.array(
        [pose.rotation @ free[index + 3 : index + 6, column] for column in range(free.shape[1])]
    ).T
    # Scale-free statement of "the family reaches a rotation about x": the unknown is
    # both the family coefficients and the angular speed about x, so the fit cannot be
    # satisfied by inflating the coefficients.
    design = np.column_stack((sensitivity, -np.array([1.0, 0.0, 0.0])))
    solution, *_ = np.linalg.lstsq(design, -base, rcond=None)
    speed = float(solution[-1])
    residual = float(np.linalg.norm(design @ solution + base))
    trial = velocity + free @ solution[: free.shape[1]] if free.shape[1] else velocity
    omega = pose.rotation @ trial[index + 3 : index + 6]
    twin = sk.Twist(
        omega=omega,
        velocity=pose.rotation @ trial[index : index + 3] + np.cross(pose.translation, omega),
    )
    divisor = abs(speed) if abs(speed) > 0.0 else float(np.linalg.norm(base))
    return sk.screw_from_twist(twin, anchor=motion.pivot_point), residual / divisor, motion


def main() -> None:
    runtime = compose_axle(double_wishbone_axle_model(), "K")

    section("1 differentiation step: measured error and the Jacobian's singular-value floor")
    reference, _ = sk.constraint_jacobian(runtime.constraints, runtime.state, step=1e-5)
    for step in (1e-4, 1e-5, 1e-6, 1e-7, 1e-8):
        jacobian, _ = sk.constraint_jacobian(runtime.constraints, runtime.state, step=step)
        singular = np.linalg.svd(jacobian, compute_uv=False)
        rank = int(np.count_nonzero(singular > sk.RANK_TOLERANCE * singular[0]))
        print(
            f"h={step:g}: |J(h)-J(1e-5)|_inf={float(np.max(np.abs(jacobian - reference))):.3e} "
            f"|J|_inf={float(np.max(np.abs(jacobian))):.3e} "
            f"sv_max={singular[0]:.6e} sv_min={singular[-1]:.6e} rank={rank}"
        )
    epsilon = float(np.finfo(float).eps)
    print(
        f"roundoff scale   eps*|f|/h = {epsilon:.3e} * 1e3 / 1e-6 = {epsilon * 1e3 / 1e-6:.3e} mm "
        "(|f| ~ 1e3 is the largest residual entry, |J|_inf above)"
    )
    print(f"truncation scale h^2*f'''/6 = (1e-6)^2 / 6            = {1e-6**2 / 6.0:.3e} (dimensionless, f''' ~ 1)")

    section("2 exact reduction: planar four-bar, where today's construction is exact")
    for label, upper_inner, upper_outer, lower_inner, lower_outer in (
        ("symmetric", (-500.0, 500.0), (-700.0, 300.0), (-500.0, 100.0), (-700.0, 0.0)),
        ("arm_skew", (-500.0, 500.0), (-760.0, 320.0), (-470.0, 90.0), (-690.0, -10.0)),
    ):
        constraints, state = planar_four_bar(upper_inner, upper_outer, lower_inner, lower_outer)
        motion = sk.solve_rigid_motion(
            constraints, state, [sk.TangentDrive("upright", 2, 1.0)], pivot_body="upright"
        )
        expected = two_d_intersection(upper_inner, upper_outer, lower_inner, lower_outer)
        got = motion.screw.point[1:]
        print(
            f"{label}: engine=({got[0]!r}, {got[1]!r}) "
            f"today's 2d arithmetic=({expected[0]!r}, {expected[1]!r}) "
            f"abs_delta={float(np.linalg.norm(got - expected)):.6e} mm "
            f"rel_delta={float(np.linalg.norm(got - expected) / np.linalg.norm(expected)):.3e} "
            f"direction={np.round(motion.screw.direction, 12).tolist()} pitch={motion.screw.pitch:.3e}"
        )

    section("3 side-by-side table on the double-wishbone axle")
    for label, overrides in HARDENING_CASES:
        model = double_wishbone_axle_model(**overrides)
        axle = compose_axle(model, "K")
        today = _instant_center(model, "L")
        without_mean = today_without_the_mean(model, "L")
        print(f"\n-- {label}   (front-view plane x = {FRONT_VIEW_PLANE_X!r}, the spindle plane)")
        for item in axle.constraints:
            if item.name in ("uca_mount_L_inner_front", "lca_mount_L_inner_front"):
                print(f"   arm pivot {item.name:26s} axis_a = {np.round(item.axis_a, 9).tolist()}")
        print(f"   (a) today  _instant_center(L) as it stands    [y,z] = ({today[0]!r}, {today[1]!r})")
        print(f"   (b) today  arithmetic with the real pivot     [y,z] = ({without_mean[0]!r}, {without_mean[1]!r})")
        print(
            f"       (b)-(a) = today's inboard-mean substitution: "
            f"dy = {without_mean[0] - today[0]!r} mm, dz = {without_mean[1] - today[1]!r} mm"
        )
        for level, constraints in (
            ("c1 left 4-bar only (the two arm lines)", tuple(item for item in axle.constraints if item.name in ARM_JOINTS)),
            ("c2 full axle (16 declarations)", axle.constraints),
        ):
            bodies = sk.free_bodies(constraints, axle.state)
            screw, fraction, motion = comparison_member(
                constraints, axle, bodies, "upright_L", ("upright_L", "spindle")
            )
            piercing = pierce(screw.point, screw.direction, FRONT_VIEW_PLANE_X)
            print(f"   ({level})  nullity={motion.nullity}")
            print(f"       axis in the front-view plane [y,z] = ({piercing[0]!r}, {piercing[1]!r})")
            print(f"       minus (a) today    dy = {piercing[0] - today[0]!r} mm, dz = {piercing[1] - today[1]!r} mm")
            print(f"       minus (b) pivot    dy = {piercing[0] - without_mean[0]!r} mm, dz = {piercing[1] - without_mean[1]!r} mm")
            print(f"       non-roll-about-x fraction |omega_perp|/|omega| = {fraction:.3e}")
            print(f"       omega = {np.round(screw.direction * screw.angular_speed, 12).tolist()}")
            print(f"       axis point = {np.round(screw.point, 12).tolist()}  pitch = {screw.pitch!r}")

        minimum_norm = sk.solve_rigid_motion(
            axle.constraints,
            axle.state,
            [sk.TangentDrive("upright_L", 3, 1.0)],
            points=axle.points,
            bodies=sk.free_bodies(axle.constraints, axle.state),
            pivot_body="upright_L",
            pivot_point=("upright_L", "spindle"),
        )
        mn_piercing = pierce(minimum_norm.screw.point, minimum_norm.screw.direction, FRONT_VIEW_PLANE_X)
        print("   (d) the minimum-norm family member -- NOT the comparison member")
        print(f"       omega = {np.round(minimum_norm.omega_rel, 12).tolist()}")
        print(f"       axis in the front-view plane [y,z] = ({mn_piercing[0]!r}, {mn_piercing[1]!r})")
        print(f"       minus (a) today    dy = {mn_piercing[0] - today[0]!r} mm, dz = {mn_piercing[1] - today[1]!r} mm")

        mirrored_bodies = sk.free_bodies(axle.constraints, axle.state)
        mirrored_screw, mirrored_fraction, _ = comparison_member(
            axle.constraints, axle, mirrored_bodies, "upright_R", ("upright_R", "spindle")
        )
        mirrored_piercing = pierce(mirrored_screw.point, mirrored_screw.direction, FRONT_VIEW_PLANE_X)
        print(
            f"   (e) the mirror side, same selection: axis yz = "
            f"({mirrored_piercing[0]!r}, {mirrored_piercing[1]!r})  "
            f"fraction = {mirrored_fraction:.3e}"
        )

    section("4 the family the drives leave free, and the axis's invariance")
    bodies = sk.free_bodies(runtime.constraints, runtime.state)
    base = sk.solve_rigid_motion(
        runtime.constraints,
        runtime.state,
        [sk.TangentDrive("upright_L", 3, 1.0)],
        points=runtime.points,
        bodies=bodies,
        pivot_body="upright_L",
        pivot_point=("upright_L", "spindle"),
    )
    print(f"stacked rank={base.rank} nullity={base.nullity} (free members)")
    index = 6 * base.bodies.index("upright_L")
    pose = runtime.state.pose("upright_L")
    spread = []
    for column in range(base.null_space.shape[1]):
        for sign in (1.0, -1.0):
            offset = (base.velocity + sign * 1.0e-3 * base.null_space[:, column])[index : index + 6]
            omega = pose.rotation @ offset[3:]
            velocity = pose.rotation @ offset[:3] + np.cross(pose.translation, omega)
            spread.append(np.cross(omega, velocity) / float(omega @ omega))
    spread = np.asarray(spread)
    print(
        "axis-point spread over +-1e-3 along every free direction (mm):",
        np.round(spread.max(axis=0) - spread.min(axis=0), 12).tolist(),
    )
    for label, drives in (
        ("drive the wheel centre vertically", [sk.PointDrive("wheel_hub_L", "wheel_center", np.array([0.0, 0.0, 1.0]))]),
        ("drive the rack laterally", [sk.TangentDrive("rack", 1, 1.0)]),
        ("drive the upright's local x-rotation", [sk.TangentDrive("upright_L", 3, 1.0)]),
    ):
        motion = sk.solve_rigid_motion(
            runtime.constraints,
            runtime.state,
            drives,
            points=runtime.points,
            bodies=bodies,
            pivot_body="upright_L",
            pivot_point=("upright_L", "spindle"),
        )
        unit = motion.screw.direction if motion.screw.direction[0] >= 0 else -motion.screw.direction
        print(
            f"{label:36s}: |omega|={float(np.linalg.norm(motion.omega_rel)):.10e} "
            f"axis point={np.round(motion.screw.point, 9).tolist()} "
            f"sign-normalised direction={np.round(unit, 12).tolist()} pitch={motion.screw.pitch:.10e}"
        )


if __name__ == "__main__":
    main()
