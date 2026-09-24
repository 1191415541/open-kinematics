"""
The explicit channel between the K/C assembly and the SI dynamic model.

There are two model schemas and there is no way around that: the K/C contract is
millimetres and kinematic, while the native dynamics schema is SI and carries
inertias, bushings and tire laws.  What *was* avoidable -- and what the study work
exists to remove -- is that the two were reached by two independent assembly
paths, so "the same axle, read two ways" was true in name only.

This module is the one place where the K/C assembly becomes a dynamic model.
Keeping it in one place, and saying so, is the point: a divergence between the two
readings is now a change *here*, which a reviewer sees, instead of the two paths
drifting a line at a time.

Two rules make the mapping checkable rather than plausible:

* **the dynamic model is derived, never re-specified.**  Bodies, points, joints and
  elements all come from the assembly this module is handed, so nothing the
  assembly does not contain can appear in the dynamic model;
* **a construct with no dynamic counterpart is refused by name.**  Dropping a
  constraint quietly would produce a model that solves and is wrong.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Literal, Protocol, runtime_checkable

import numpy as np

from ..axle_dynamics.schema import (
    AxleBody,
    AxleBushing,
    AxleDynamicsModel,
    AxleJoint,
    AxleSpringDamper,
    AxleTire,
)
from ..preparation.assembly import FrontAxleAssembly
from ..preparation.assembly.types import (
    BallJoint,
    ConstantVelocityJoint,
    Constraint,
    CylindricalJoint,
    InPlaneJoint,
    PrismaticJoint,
    RevoluteJoint,
    UniversalJoint,
    WeldJoint,
)

__all__ = ["BridgeError", "MM", "axle_dynamics_model"]


@runtime_checkable
class _TwoBodyConstraint(Protocol):
    """
    The shape every joint kind this bridge converts has: two bodies, two points.

    The assembly declarations do not share a base class for this: `BallJoint`
    inherits it while the revolute, universal and prismatic joints each declare
    their own fields.  Stating the shape once is what lets `_joint` read bodies and
    points under a type checker as well as at run time.
    """

    name: str
    body_a: str
    body_b: str
    point_a: Any
    point_b: Any

if TYPE_CHECKING:
    from .assembly import StudyAssembly

#: Millimetres per metre.  The K/C contract is in mm and the dynamic schema in SI,
#: and nothing in the assembly records which system authored it, so the scale is
#: stated once here rather than guessed at each call site.
MM = 1000.0


class BridgeError(ValueError):
    """The assembly carries something the dynamic schema cannot express."""


#: The joint kinds the SI dynamic schema accepts, as its own `Literal` spells them.
JointKind = Literal[
    "spherical",
    "revolute",
    "prismatic",
    "fixed",
    "universal",
    "constant_velocity",
    "cylindrical",
    "inplane",
]

#: Assembly constraint class -> the dynamic schema's joint kind.
_JOINT_KINDS: tuple[tuple[type, JointKind], ...] = (
    (BallJoint, "spherical"),
    (RevoluteJoint, "revolute"),
    (PrismaticJoint, "prismatic"),
    (WeldJoint, "fixed"),
    (UniversalJoint, "universal"),
    (ConstantVelocityJoint, "constant_velocity"),
    (CylindricalJoint, "cylindrical"),
    (InPlaneJoint, "inplane"),
)

#: The element types that become one SI spring/damper entry.  A corner carries
#: its spring and its damper on the same two points, and the schema has one entry
#: for both, so the two are named together.
_SPRING_DAMPER_ELEMENTS: frozenset[str] = frozenset(
    {"LinearSpringElement", "StaticDamperElement"}
)

#: Every element type this bridge can read.  Anything outside this set is
#: refused by name rather than dropped -- see `_refuse_unreadable_elements`.
_READABLE_ELEMENTS: frozenset[str] = (
    _SPRING_DAMPER_ELEMENTS
    | {"BushingElement", "VerticalTireElement"}
)


def _joint_kind(constraint: Constraint) -> JointKind:
    """Return the dynamic joint kind for one assembly constraint."""
    for cls, kind in _JOINT_KINDS:
        if isinstance(constraint, cls):
            return kind
    raise BridgeError(
        f"assembly constraint {type(constraint).__name__} "
        f"({getattr(constraint, 'name', '<unnamed>')}) has no dynamic joint kind; "
        "add the mapping rather than dropping the constraint"
    )


def _quat(values) -> tuple[float, float, float, float]:
    """Return a fixed-length float quaternion, the shape the SI schema stores."""
    array = np.asarray(values, dtype=float).reshape(-1)
    if array.size != 4:
        raise BridgeError(f"expected a four-component quaternion, got {array.size}")
    return (float(array[0]), float(array[1]), float(array[2]), float(array[3]))


def _vec3(values) -> tuple[float, float, float]:
    """Return a fixed-length float triple, the shape the SI schema stores."""
    array = np.asarray(values, dtype=float).reshape(-1)
    if array.size != 3:
        raise BridgeError(f"expected a three-component vector, got {array.size}")
    return (float(array[0]), float(array[1]), float(array[2]))


def _tuple(matrix: np.ndarray) -> tuple:
    """Return a nested float tuple, the shape the SI schema stores matrices in."""
    return tuple(tuple(float(value) for value in row) for row in np.asarray(matrix))


def _body(body_name: str, body) -> AxleBody:
    """
    Convert one assembly body to the SI dynamic schema.

    A free body with no mass is refused here rather than given a nominal value.
    That is not a formality: the K/C contract is kinematic and a fixture may
    declare bodies it never gives inertia to, while the SI schema -- and the
    solver behind it -- need a real mass.  Inventing one would produce a dynamic
    model that solves and means nothing, so the reading stops and says which body
    it is missing.
    """
    if not body.fixed and not float(body.mass) > 0.0:
        raise BridgeError(
            f"assembly body {body_name!r} is free but carries no mass, so it has no "
            "dynamic reading; the K/C contract is kinematic and the model must "
            "declare body inertia before it can be advanced in time"
        )
    inertia = np.asarray(body.inertia, dtype=float) / (MM * MM)
    return AxleBody(
        name=body_name,
        mass_kg=float(body.mass),
        inertia_kg_m2=_tuple(inertia),
        position_m=_vec3(np.asarray(body.pose.translation) / MM),
        quaternion_body_to_world=_quat(body.pose.quaternion),
        fixed=bool(body.fixed),
    )


def _joint(constraint: Constraint) -> AxleJoint:
    """
    Convert one assembly constraint to an SI dynamic joint.

    The assembly declarations do not share one base class per family: `BallJoint`
    inherits the two-body-points shape while the revolute, universal and prismatic
    joints each declare their own fields.  Narrowing by the *shape* the joint kind
    needs is therefore what lets the bodies and points be read directly, and a
    constraint without that shape is refused by name rather than guessed at.
    """
    if not isinstance(constraint, _TwoBodyConstraint):
        raise BridgeError(
            f"assembly constraint {type(constraint).__name__} "
            f"({getattr(constraint, 'name', '<unnamed>')}) has no two body points "
            "and therefore no dynamic joint reading"
        )
    axis_a = np.asarray(getattr(constraint, "axis_a", (0.0, 0.0, 1.0)), dtype=float)
    axis_b = np.asarray(getattr(constraint, "axis_b", axis_a), dtype=float)
    # The two optional fields are passed by name rather than splatted: a splatted
    # mapping is opaque to a type checker, and these are the only two the schema
    # takes, so naming them keeps the call checked.
    target = (
        float(constraint.angle_target)
        if isinstance(constraint, ConstantVelocityJoint)
        else 0.0
    )
    secondary_a = getattr(constraint, "axis_a_secondary", None)
    secondary_b = getattr(constraint, "axis_b_secondary", None)
    return AxleJoint(
        name=constraint.name,
        kind=_joint_kind(constraint),
        body_a=constraint.body_a,
        body_b=constraint.body_b,
        point_a_m=_vec3(np.asarray(constraint.point_a) / MM),
        point_b_m=_vec3(np.asarray(constraint.point_b) / MM),
        axis_a=_vec3(axis_a),
        axis_b=_vec3(axis_b),
        axis_a_secondary=(
            _vec3(secondary_a) if secondary_a is not None else (0.0, 1.0, 0.0)
        ),
        axis_b_secondary=(
            _vec3(secondary_b) if secondary_b is not None else (1.0, 0.0, 0.0)
        ),
        constant_velocity_angle_target=target,
    )


def _bushing(element) -> AxleBushing:
    """Convert one assembly bushing element to the SI dynamic schema."""
    # Translational stiffness is N/mm in the contract and N/m in the schema;
    # rotational entries are N*m/rad in both.  Scaling the translational block
    # only is what keeps the two readings physically the same spring.
    scale = np.diag([MM, MM, MM, 1.0, 1.0, 1.0])
    stiffness = np.asarray(element.stiffness, dtype=float)
    damping = np.asarray(element.damping, dtype=float)
    pose_a = element.local_pose_a
    pose_b = element.local_pose_b
    return AxleBushing(
        name=element.name,
        body_a=element.body_a,
        body_b=element.body_b,
        point_a_m=tuple(float(v) for v in np.asarray(pose_a.translation) / MM),
        point_b_m=tuple(float(v) for v in np.asarray(pose_b.translation) / MM),
        frame_a_to_body_quaternion=tuple(
            float(v) for v in np.asarray(pose_a.quaternion)
        ),
        frame_b_to_body_quaternion=tuple(
            float(v) for v in np.asarray(pose_b.quaternion)
        ),
        reference_translation_in_frame_a_m=tuple(
            float(v) for v in np.asarray(pose_b.translation) / MM
        ),
        reference_quaternion_a_to_b=tuple(
            float(v) for v in np.asarray(pose_b.quaternion)
        ),
        stiffness=_tuple(scale @ stiffness),
        damping=_tuple(scale @ damping),
    )


def _vertical_tire(element) -> AxleTire:
    """
    Convert a vertical tire element to an SI tire under vertical-only activation.

    This is the quasi-static reading of a tire, and it is the *same* tire the
    dynamic study uses -- decision D1.  A static equilibrium has no slip, so the
    lateral and longitudinal terms below contribute no force; the kernel requires
    them positive, which is why they are placeholders rather than zeros.  Nothing
    is added to the kernel and nothing is switched off inside it: the activation
    difference lives entirely in what the model asks for.

    The mass is the tire's own (decision D2), which is what lets the vertical
    reading carry the wheel's inertia without the wheel-end body owning it.
    """
    radius_m = float(element.unloaded_radius)
    return AxleTire(
        name=element.name,
        body=element.wheel_body,
        center_local_m=tuple(
            float(v) for v in np.asarray(element.wheel_center_local) / MM
        ),
        unloaded_radius_m=radius_m,
        maximum_compression_m=radius_m * 0.5,
        vertical_stiffness_n_per_m=float(element.stiffness) * MM,
        vertical_damping_n_s_per_m=0.0,
        longitudinal_friction_coefficient=1.0,
        lateral_friction_coefficient=1.0,
        longitudinal_brush_stiffness_n_per_m=1.0,
        lateral_brush_stiffness_n_per_m=1.0,
        longitudinal_relaxation_length_m=1.0,
        lateral_relaxation_length_m=1.0,
        detached_relaxation_s=1.0,
        model_kind="native_brush",
    )


def axle_dynamics_model(
    study_assembly: StudyAssembly, *, name: str = "axle"
) -> AxleDynamicsModel:
    """
    Return the SI dynamic model of one study assembly.

    Everything comes from the assembly: its bodies, points, constraints and
    elements.  The result is therefore a *reading* of the assembly, and a
    construct this schema cannot express raises rather than disappearing.

    That last sentence is load-bearing, and it was not true of the first
    version: elements were selected by name and everything else was dropped
    silently, so an assembly carrying springs produced a dynamic model without
    them -- a model that solves and is wrong.  The element walk below therefore
    accounts for every element the assembly can hold and refuses the ones this
    schema has no field for.
    """
    assembly: FrontAxleAssembly = study_assembly.assembly
    bodies = tuple(_body(key, value) for key, value in assembly.bodies.items())
    joints = tuple(_joint(constraint) for constraint in assembly.constraints)
    bushings = tuple(
        _bushing(element)
        for element in assembly.bushings
        if type(element).__name__ == "BushingElement"
    )
    tires = tuple(
        _vertical_tire(element)
        for element in assembly.elements
        if type(element).__name__ == "VerticalTireElement"
    )
    springs = tuple(
        _spring(element)
        for element in assembly.elements
        if type(element).__name__ in _SPRING_DAMPER_ELEMENTS
    )
    _refuse_unreadable_elements(assembly)
    return AxleDynamicsModel(
        name=name,
        bodies=bodies,
        joints=joints,
        bushings=bushings,
        tires=tires,
        springs=springs,
    )


def _spring(element) -> AxleSpringDamper:
    """
    Convert a spring or a damper element to the SI spring/damper entry.

    The two are separate entries even on the same two points: the schema carries
    stiffness and damping in one record, but a spring contributes no damping and
    a damper no stiffness, so listing both gives the same force sum a merged
    record would.  What matters is that neither is dropped.

    The free length needs care, because the two schemas state the offset
    differently.  An assembly spring is `k * (length - reference) + preload`,
    while the SI entry is `k * (length - free_length)`.  A preloaded or
    reference-length spring therefore has to be shifted by `preload / k`: taking
    the reference length as the free length would move the corner's resting
    position and quietly change the wheel load.
    """
    if type(element).__name__ != "LinearSpringElement":
        # A damper carries no elastic term, so its free length cannot affect a
        # force and is given the schema's neutral value.
        return AxleSpringDamper(
            name=element.name,
            body_a=element.body_a,
            body_b=element.body_b,
            point_a_m=_vec3(np.asarray(element.point_a) / MM),
            point_b_m=_vec3(np.asarray(element.point_b) / MM),
            stiffness_n_per_m=0.0,
            compression_damping_n_s_per_m=float(element.viscous_damping),
            rebound_damping_n_s_per_m=float(element.viscous_damping),
            free_length_m=0.0,
        )
    reference = (
        element.free_length
        if element.free_length is not None
        else element.reference_length
    )
    stiffness = float(element.stiffness)
    # The shift is in the assembly's own millimetres, so only the result is
    # converted to metres below.
    free_length = float(reference) - float(element.preload) / stiffness
    return AxleSpringDamper(
        name=element.name,
        body_a=element.body_a,
        body_b=element.body_b,
        point_a_m=_vec3(np.asarray(element.point_a) / MM),
        point_b_m=_vec3(np.asarray(element.point_b) / MM),
        stiffness_n_per_m=stiffness * MM,
        compression_damping_n_s_per_m=0.0,
        rebound_damping_n_s_per_m=0.0,
        free_length_m=free_length / MM,
    )


def _refuse_unreadable_elements(assembly: FrontAxleAssembly) -> None:
    """
    Raise for any element this schema has no field for, naming it.

    The alternative is what the first version did -- drop it -- and a dropped
    element is the one failure mode a dynamic model cannot show: the run still
    converges, the numbers still look plausible, and the missing spring is
    invisible.  Naming the type and the element is what makes the gap fixable.
    """
    readable = _READABLE_ELEMENTS
    unreadable = sorted(
        {
            f"{type(element).__name__} ({getattr(element, 'name', '<unnamed>')})"
            for element in (*assembly.elements, *assembly.bushings)
            if type(element).__name__ not in readable
        }
    )
    if unreadable:
        raise BridgeError(
            "the dynamic schema has no reading for "
            + ", ".join(unreadable)
            + "; add the mapping rather than dropping the element, because a "
            "model that solves without it is wrong rather than incomplete"
        )
