"""
The acceptance fixture for the pad-driven K/C work.

Two models, because the K and C readings are physically different -- not merely two
settings of one model:

* **K** drives the wheel centres through the *rigid kinematic set*, so the arm mounts are
  ideal constraints and the model carries no bushings
  (`cases/kc_quasi_static/contract.py` says exactly this);
* **C** loads the *compliant* set, where the arm mounts are carried by bushings.

Handing C the rigid model fails its static equilibrium -- measured, `c-fx--1.00` stops at
`force_residual = 0.026635` -- because the arm ends have nowhere compliant to react.

Both models carry a **spring** and **no tires**, for reasons the snapshot's README
records; briefly:

* the spring is what makes mode A (no force elements) distinguishable from mode B (force
  elements at work).  Measured A/B difference `1.307305e-03`, independent of stiffness;
* no tires, because the contract emits them unconditionally, so a tire would make
  mode A differ from the frozen snapshot by the tire alone.

Neither model declares `road`: that field is added to `FrontAxleModel` by another task.
"""

from __future__ import annotations

from suspension_multibody.schema import (
    Bushing6x6,
    FrontAxleModel,
    LinearSpring,
    MassSpec,
    Pose,
    Vec3,
)

__all__ = [
    "HARDPOINTS",
    "fixture_model",
    "fixture_name",
    "rigid_model",
    "compliant_model",
]

#: The axle's hardpoints, in the left-side convention the schema requires.
HARDPOINTS: dict[str, Vec3] = {
    "uca_front": Vec3(x=-100.0, y=-500.0, z=400.0),
    "uca_rear": Vec3(x=100.0, y=-500.0, z=400.0),
    "uca_outer": Vec3(x=0.0, y=-700.0, z=450.0),
    "lca_front": Vec3(x=-120.0, y=-500.0, z=150.0),
    "lca_rear": Vec3(x=120.0, y=-500.0, z=150.0),
    "lca_outer": Vec3(x=0.0, y=-700.0, z=150.0),
    "tierod_inner": Vec3(x=100.0, y=-400.0, z=250.0),
    "tierod_outer": Vec3(x=50.0, y=-700.0, z=250.0),
    "wheel_center": Vec3(x=0.0, y=-700.0, z=300.0),
    "rack_center": Vec3(x=0.0, y=0.0, z=250.0),
}

fixture_name = "pad_kc_acceptance_fixture"

#: The spring's geometry: chassis down to the lower arm, so it reacts the direction the
#: suspension actually moves.
_SPRING_STIFFNESS = 50.0
_SPRING_FREE_LENGTH = 200.0
_SPRING_AT_CHASSIS = Vec3(x=0.0, y=-500.0, z=400.0)
_SPRING_AT_ARM = Vec3(x=0.0, y=-650.0, z=200.0)

#: The bushing mount points, by arm.  These are the names the compliant set carries.
_MOUNTS: tuple[tuple[str, tuple[str, ...]], ...] = (
    ("upper_arm", ("uca_front", "uca_rear")),
    ("lower_arm", ("lca_front", "lca_rear")),
)


def _spring() -> LinearSpring:
    """
    Return the one spring, declared the way the schema expects.

    Body names are **unsided** (`lower_arm`, not `lower_arm_L`): the subsystem's
    `resolve_body` maps a role name onto the generated per-side body, and it short-circuits
    when the name is already present -- so naming a side here would build the right side's
    spring against the *left* body, which solves, converges, and puts the load in the wrong
    place.
    """
    return LinearSpring(
        name="spring_L",
        body_a="chassis",
        point_a=_SPRING_AT_CHASSIS,
        body_b="lower_arm",
        point_b=_SPRING_AT_ARM,
        stiffness=_SPRING_STIFFNESS,
        # Exactly one of free_length / reference_length is allowed: the element refuses
        # both at once because they are two ways of stating one reference.
        free_length=_SPRING_FREE_LENGTH,
    )


def rigid_model() -> FrontAxleModel:
    """The K model: ideal constraints at the arm mounts, plus one spring."""
    return FrontAxleModel(
        name=fixture_name,
        hardpoints=dict(HARDPOINTS),
        mass=MassSpec(sprung_mass=1000),
        springs=(_spring(),),
    )


def compliant_model() -> FrontAxleModel:
    """
    The C model: the same axle with bushing-carried arm mounts.

    The bushings are stiff in translation and very stiff in rotation, which is what lets
    the C reading distribute load through them.  The values follow the project's own
    compliant fixture so this model matches the reading the C gates already exercise.
    """
    stiffness = tuple(
        tuple(
            10_000.0
            if row == column and row < 3
            else 10_000_000.0
            if row == column
            else 0.0
            for column in range(6)
        )
        for row in range(6)
    )

    def mount(hardpoint: str) -> Pose:
        point = HARDPOINTS[hardpoint]
        return Pose(translation=Vec3(x=point.x, y=point.y, z=point.z))

    bushings = tuple(
        Bushing6x6(
            name=f"{body}_{index}",
            body_a="chassis",
            body_b=body,
            pose_a=mount(hardpoint),
            pose_b=mount(hardpoint),
            stiffness=stiffness,
        )
        for body, hardpoints in _MOUNTS
        for index, hardpoint in enumerate(hardpoints)
    )
    return FrontAxleModel(
        name=f"{fixture_name}_compliant",
        hardpoints=dict(HARDPOINTS),
        mass=MassSpec(sprung_mass=1000),
        springs=(_spring(),),
        bushings=bushings,
    )


def fixture_model(mode: str = "K") -> FrontAxleModel:
    """Return the model the given reading needs: rigid for K, compliant for C."""
    return rigid_model() if mode.upper() == "K" else compliant_model()
