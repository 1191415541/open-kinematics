"""
Evidence for raw/engine_contract.md and raw/axis_assertions.md.

Prints: the engine's public surface, one real minimal call on the assembled
double-wishbone axle, the analytic-solution check on single joints, and the
rank / singular-value floor of the assembled mechanism.

Run:  uv run --no-sync python <this file>
"""

from __future__ import annotations

import inspect
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from fixtures import double_wishbone_axle_model, planar_four_bar, two_d_intersection  # noqa: E402

from suspension_multibody.modeling.primitives import (  # noqa: E402
    DistanceConstraint,
    PrismaticJoint,
    RevoluteJoint,
    RigidBody,
    RigidBodyState,
)
from suspension_multibody.subsystems.entry import compose_axle  # noqa: E402
from suspension_multibody.vehicle import screw_kinematics as sk  # noqa: E402


def section(title: str) -> None:
    print(f"\n===== {title} =====")


def main() -> None:
    print("ENGINE MODULE:", sk.__file__)

    section("public surface as inspect sees it")
    for name in sk.__all__:
        value = getattr(sk, name)
        kind = "class" if inspect.isclass(value) else ("def" if callable(value) else "const")
        try:
            rendered = str(inspect.signature(value))
        except (TypeError, ValueError):
            rendered = ""
        print(f"{kind:6s} {name}{rendered}")

    section("minimal call on the assembled double-wishbone axle")
    runtime = compose_axle(double_wishbone_axle_model(), "K")
    motion = sk.solve_rigid_motion(
        runtime.constraints,
        runtime.state,
        [sk.PointDrive("wheel_hub_L", "wheel_center", np.array([0.0, 0.0, 1.0]))],
        points=runtime.points,
        pivot_body="upright_L",
        pivot_point=("upright_L", "spindle"),
    )
    print("constraint declarations        :", len(runtime.constraints))
    print("residual rows                  :", sk.constraint_residual(runtime.constraints, runtime.state).size)
    print("movable bodies (columns)       :", len(motion.bodies), list(motion.bodies))
    print("jacobian shape                 :", motion.jacobian.shape)
    print("rank / nullity                 :", motion.rank, "/", motion.nullity)
    print("constraint_residual |J q|_inf  :", repr(motion.constraint_residual))
    print("drive_residual                 :", repr(motion.drive_residual))
    print("reference                      :", repr(motion.reference))
    print("pivot_body                     :", repr(motion.pivot_body))
    print("pivot_point (world anchor)     :", repr(motion.pivot_point))
    print("omega_rel                      :", repr(motion.omega_rel))
    print("|omega_rel|                    :", repr(float(np.linalg.norm(motion.omega_rel))))
    print("v_rel (at the anchor)          :", repr(motion.v_rel))
    print("screw.point                    :", repr(motion.screw.point))
    print("screw.direction                :", repr(motion.screw.direction))
    print("screw.pitch                    :", repr(motion.screw.pitch))
    print("screw.angular_speed            :", repr(motion.screw.angular_speed))
    print("velocity (full q_dot)          :", repr(motion.velocity))
    print("null_space shape               :", motion.null_space.shape)
    print("twists (absolute, ground ref)  :")
    for body in sorted(motion.twists):
        print(f"    {body:22s} omega={motion.twists[body].omega!r} velocity={motion.twists[body].velocity!r}")
    print("twist_of('upright_L')          :", repr(motion.twist_of("upright_L")))

    section("the same motion read from the wheel hub (it also carries the wheel spin)")
    hub = sk.solve_rigid_motion(
        runtime.constraints,
        runtime.state,
        [sk.PointDrive("wheel_hub_L", "wheel_center", np.array([0.0, 0.0, 1.0]))],
        points=runtime.points,
        pivot_body="wheel_hub_L",
        pivot_point=("wheel_hub_L", "wheel_center"),
    )
    print("omega_rel   :", repr(hub.omega_rel))
    print("screw.point :", repr(hub.screw.point))
    print("screw.dir   :", repr(hub.screw.direction))
    print("screw.pitch :", repr(hub.screw.pitch))

    section("the same call in the ideal_constraints column and with an extra drive")
    ideal = sk.solve_rigid_motion(
        runtime.ideal_constraints,
        runtime.state,
        [sk.PointDrive("wheel_hub_L", "wheel_center", np.array([0.0, 0.0, 1.0]))],
        points=runtime.points,
        pivot_body="upright_L",
        pivot_point=("upright_L", "spindle"),
    )
    print("ideal: rank/nullity            :", ideal.rank, "/", ideal.nullity)
    print("ideal: screw.point             :", repr(ideal.screw.point))
    steadied = sk.solve_rigid_motion(
        runtime.constraints,
        runtime.state,
        [
            sk.PointDrive("wheel_hub_L", "wheel_center", np.array([0.0, 0.0, 1.0])),
            sk.TangentDrive("rack", 1, 0.0),
        ],
        points=runtime.points,
        pivot_body="upright_L",
        pivot_point=("upright_L", "spindle"),
    )
    print("with rack drive: rank/nullity  :", steadied.rank, "/", steadied.nullity)
    print("with rack drive: screw.point   :", repr(steadied.screw.point))

    section("rank and singular-value floor of the assembled mechanism")
    for label, constraints in (
        ("runtime.constraints", runtime.constraints),
        ("runtime.ideal_constraints", runtime.ideal_constraints),
    ):
        jacobian, _ = sk.constraint_jacobian(constraints, runtime.state)
        singular = np.linalg.svd(jacobian, compute_uv=False)
        keep = singular > sk.RANK_TOLERANCE * singular[0]
        rank = int(np.count_nonzero(keep))
        rejected = repr(singular[rank]) if rank < singular.size else "none (full column rank)"
        print(
            f"{label}: J{jacobian.shape} rank={rank} "
            f"sv_max={singular[0]!r} sv_min_nonzero={singular[keep][-1]!r} "
            f"sv_first_rejected={rejected}"
        )

    section("analytic case: a single revolute joint's instant axis IS its axis")
    for axis, point, component, label in (
        ((0.0, 1.0, 0.0), (100.0, 20.0, 30.0), 4, "axis +y, spin about local y"),
        ((1.0, 0.0, 0.0), (10.0, -220.0, 45.0), 3, "axis +x, spin about local x"),
        ((0.3, 0.4, 0.5), (-5.0, 60.0, 5.0), 5, "skew axis, spin about local z"),
    ):
        unit = np.asarray(axis, dtype=float)
        unit = unit / np.linalg.norm(unit)
        state = RigidBodyState(
            {"ground": RigidBody("ground", fixed=True), "arm": RigidBody("arm")}
        )
        constraints = (
            RevoluteJoint(
                "ground", np.asarray(point, dtype=float), np.asarray(axis, dtype=float),
                "arm", np.asarray(point, dtype=float), np.asarray(axis, dtype=float),
            ),
        )
        single = sk.solve_rigid_motion(
            constraints, state, [sk.TangentDrive("arm", component, 1.0)]
        )
        offset = single.screw.point - np.asarray(point, dtype=float)
        offset_perp = offset - unit * float(offset @ unit)
        angle = np.degrees(np.arccos(min(1.0, abs(float(single.screw.direction @ unit)))))
        print(f"{label}:")
        print(f"    declared axis  : {axis} through {point}")
        print(f"    screw.direction: {single.screw.direction!r}  angle_to_axis={angle!r} deg")
        print(f"    screw.point    : {single.screw.point!r}")
        print(f"    distance_of_screw_point_to_axis={float(np.linalg.norm(offset_perp))!r} mm")
        print(f"    pitch={single.screw.pitch!r}  |omega|={single.screw.angular_speed!r}")
        print(f"    constraint_residual={single.constraint_residual!r} drive_residual={single.drive_residual!r}")
        print(f"    rank/nullity={single.rank}/{single.nullity}")

    section("refusals: a pure translation has no axis, an unregistered type has no row")
    state = RigidBodyState(
        {"ground": RigidBody("ground", fixed=True), "slider": RigidBody("slider")}
    )
    axis = np.array([0.0, 1.0, 0.0])
    try:
        sk.solve_rigid_motion(
            (PrismaticJoint("ground", np.zeros(3), axis, "slider", np.zeros(3), axis),),
            state,
            [sk.TangentDrive("slider", 1, 1.0)],
        )
        print("prismatic: NO ERROR (unexpected)")
    except sk.KinematicError as error:
        print("prismatic refused:", error)
    try:
        sk.solve_rigid_motion(
            (DistanceConstraint("ground", np.zeros(3), "slider", np.zeros(3), 100.0),),
            state,
            [sk.TangentDrive("slider", 1, 1.0)],
        )
        print("distance: NO ERROR (unexpected)")
    except sk.KinematicError as error:
        print("distance refused:", error)

    section("planar four-bar: the exact front-view reduction")
    for label, upper_inner, upper_outer, lower_inner, lower_outer in (
        ("symmetric", (-500.0, 500.0), (-700.0, 300.0), (-500.0, 100.0), (-700.0, 0.0)),
        ("arm_skew", (-500.0, 500.0), (-760.0, 320.0), (-470.0, 90.0), (-690.0, -10.0)),
    ):
        constraints, planar_state = planar_four_bar(upper_inner, upper_outer, lower_inner, lower_outer)
        planar = sk.solve_rigid_motion(
            constraints, planar_state, [sk.TangentDrive("upright", 2, 1.0)], pivot_body="upright"
        )
        expected = two_d_intersection(upper_inner, upper_outer, lower_inner, lower_outer)
        got = planar.screw.point[1:]
        print(f"{label}:")
        print(f"    engine  (y,z)  : {got!r}")
        print(f"    2d      (y,z)  : {expected!r}")
        print(f"    abs_delta      : {float(np.linalg.norm(got - expected))!r} mm")
        print(f"    rel_delta      : {float(np.linalg.norm(got - expected) / np.linalg.norm(expected))!r}")
        print(f"    direction/pitch: {planar.screw.direction!r} / {planar.screw.pitch!r}")
        print(f"    rank/nullity   : {planar.rank}/{planar.nullity} residual={planar.constraint_residual!r}")


if __name__ == "__main__":
    main()
