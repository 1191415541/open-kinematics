"""
Emit the contract documents the kernel reads.

This is the Python half of the document boundary, and it is the *authoring*
half: it describes the model and the case, and it stops there.  Nothing here
computes a residual, a constitutive law or a Jacobian, and nothing here knows
the kernel's ctypes layout.

Two conventions are load-bearing and are therefore stated once, here:

* **Everything is body-local.**  A joint point is local to its first body, a
  joint axis to the body it is written against, a marker to its body.  The
  assembly already carries global and local forms, so the conversion happens
  once, where the geometry is.
* **Everything is millimetres.**  The document's ``units`` block says so, and
  the kernel scales on the way in.  Writing metres here would silently produce
  a model a thousand times too small.

The two families the native K/C workflow runs need *different models*, and the
difference is physical rather than cosmetic:

* K drives the wheel centres through the rigid kinematic set, so the model
  carries ``ideal_constraints`` with the paired ball joints folded into their
  equivalent revolutes, and no bushings;
* C loads the compliant set, where the arm mounts are carried by bushings
  instead, so the model carries ``constraints`` and the bushings.
"""

from __future__ import annotations

import numpy as np

from ...joints import (
    DRIVEN_KINDS,
    JOINT_KINDS,
    JointTableError,
    definition_for,
    validate_joint_axes,
)
from ...kernel.solver import solver_settings_document as _solver_settings_document
from ...modeling.primitives.joints import (
    ConstantVelocityJoint,
    CylindricalJoint,
    InPlaneJoint,
    PrismaticJoint,
    RevoluteJoint,
    UniversalJoint,
    WeldJoint,
)
from ...subsystems.capabilities import kernel_axis
from .convert import (
    NativeKcError,
    _local_axis,
    _local_point,
    collapse_spherical_pairs,
    rotation_to_quaternion,
)

UNITS = {"length": "mm", "mass": "kg", "time": "s", "angle": "rad"}
#: The joint table lives in `joints/`; this module no longer keeps its own copy.
#: `_JOINT_KINDS` used to map only three of the eight types and reject the rest,
#: which is what made "any assembly can use any joint" false in practice.
_JOINT_KINDS = JOINT_KINDS
#: Distinct from the joint table: a driven coordinate is a *prescribed* degree
#: of freedom, not a joint in the assembly's sense, but it travels in the same
#: list because that is where the kernel expects to find it.
_DRIVEN_KINDS = {
    kind: definition.kernel_name for kind, definition in DRIVEN_KINDS.items()
}


def _constraint_axes(
    constraint, body_a, body_b
) -> dict[str, list[float]]:
    """
    Return the axis fields one assembly constraint carries, in body-local form.

    Each branch mirrors the dataclass it reads: `InPlaneJoint` has only `axis_a`
    (the plane normal), `ConstantVelocityJoint` carries a primary and a secondary
    axis on each body, and the rest carry one axis per body.  A type with no axis
    returns an empty mapping -- the caller decides which of them are required.
    """
    if isinstance(constraint, InPlaneJoint):
        return {"axis_a": _unit_axis(_local_axis(constraint.axis_a, body_a))}
    if isinstance(constraint, ConstantVelocityJoint):
        return {
            "axis_a": _unit_axis(_local_axis(constraint.axis_a, body_a)),
            "axis_b": _unit_axis(_local_axis(constraint.axis_b, body_b)),
            "axis_a_secondary": _unit_axis(
                _local_axis(constraint.axis_a_secondary, body_a)
            ),
            "axis_b_secondary": _unit_axis(
                _local_axis(constraint.axis_b_secondary, body_b)
            ),
        }
    if isinstance(
        constraint, (RevoluteJoint, PrismaticJoint, UniversalJoint, CylindricalJoint)
    ):
        return {
            "axis_a": _unit_axis(_local_axis(constraint.axis_a, body_a)),
            "axis_b": _unit_axis(_local_axis(constraint.axis_b, body_b)),
        }
    return {}
UNITS = {"length": "mm", "mass": "kg", "time": "s", "angle": "rad"}
#: The joint table lives in `joints/`; this module no longer keeps its own copy.
#: `_JOINT_KINDS` used to map only three of the eight types and reject the rest,
#: which is what made "any assembly can use any joint" false in practice.
_JOINT_KINDS = JOINT_KINDS
#: Distinct from the joint table: a driven coordinate is a *prescribed* degree
#: of freedom, not a joint in the assembly's sense, but it travels in the same
#: list because that is where the kernel expects to find it.
_DRIVEN_KINDS = {kind: definition.kernel_name for kind, definition in DRIVEN_KINDS.items()}


def _vec3(values) -> list[float]:
    return [float(v) for v in np.asarray(values, dtype=float)[:3]]


def _unit_axis(values) -> list[float]:
    axis = np.asarray(values, dtype=float)[:3]
    return [float(v) for v in axis / np.linalg.norm(axis)]


#: The three readings, and what each carries into the document.
#:
#: * ``kinematics``    -- the constraint equations alone.  No elastic element and no tire
#:   enters the residual, so the run is pure geometry.  Defined on the **K** reading,
#:   where the arm mounts are ideal constraints; on C the mounts *are* the bushing column,
#:   so removing it would leave the mechanism unconstrained.
#: * ``force_balance`` -- the elastic elements balance against the driven targets.  This is
#:   the default, and it is what makes the arm mounts and the spring react load.
#: * ``pad``           -- a ground height is driven instead of the wheel centre, so the
#:   tires carry the wheel.  Tires therefore belong to this mode only; a wheel-centre-driven
#:   reading *places* the wheel rather than carrying it.
ELASTIC_MODES: frozenset[str] = frozenset({"force_balance", "pad"})
TIRE_MODES: frozenset[str] = frozenset({"pad"})

#: The hub's spin joint, as 方式 A's suspension template names it.
_SPIN_JOINT_STEM = "wheel_spin_joint_"


def _quasi_static_joints(constraints) -> list:
    """
    Return one K/C reading's constraints, with the wheel's spin held.

    方式 A turns the hub on a revolute against the upright, and that is the
    topology a *dynamic* reading needs: the wheel rolls, and the spin is the
    degree of freedom the rolling is expressed in.  A K/C reading is a bench
    reading -- the wheel is mounted rigidly on the suspension and the tire
    carries the compliance to the road -- so there the spin carries neither load
    nor stiffness, and leaving it free makes the static KKT singular.  Measured
    on the compliant C sweep, the free spin stalls the trim at
    ``force_residual = 1.05e-8`` against a 1e-8 tolerance with a 1e-15 pose
    step: a residual outside the Jacobian's range, which no Newton step removes
    and which fails every C load path.

    So the reading states what the bench means and emits the spin joint as the
    rigid attachment it is on a bench.  The *assembly* keeps the revolute -- this
    is a reading of the model rather than a change to it -- and the dynamic
    readings that need the spin free author their own documents.
    """
    return [
        WeldJoint(
            constraint.body_a,
            constraint.point_a,
            constraint.body_b,
            constraint.point_b,
            name=constraint.name,
        )
        if getattr(constraint, "name", "").startswith(_SPIN_JOINT_STEM)
        else constraint
        for constraint in constraints
    ]


def model_document(
    assembly,
    *,
    name: str = "front-axle",
    drive_wheels: bool = True,
    drive_mode: str | None = None,
) -> dict[str, object]:
    """
    Describe one axle assembly as a multibody model document.

    ``drive_wheels`` selects which physical model the document describes: the
    rigid kinematic set of the K family, or the compliant set of the C family.
    It is not a solver setting, which is why it lives here rather than in the
    case document.

    ``drive_mode`` selects how much of the assembly's mechanics enters the
    document -- see ``ELASTIC_MODES``/``TIRE_MODES``.  When it is ``None`` the
    legacy boolean decides, through the one mapping in
    :func:`~suspension_multibody.schema.case.drive_mode_for`, so the two spellings
    cannot drift apart.
    """
    from ...schema.case import drive_mode_for

    mode = drive_mode if drive_mode is not None else drive_mode_for(drive_wheels)
    if mode not in ("kinematics", "force_balance", "pad"):
        raise NativeKcError(
            f"unknown drive_mode {mode!r}; expected one of "
            "'kinematics', 'force_balance', 'pad'"
        )

    bodies = []
    for body_name, body in assembly.bodies.items():
        inertia = np.asarray(body.inertia, dtype=float)
        if body.fixed:
            mass, inertia = 0.0, np.eye(3)
        else:
            mass = float(body.mass) if body.mass > 0.0 else 1.0
            if not np.all(np.isfinite(inertia)) or np.linalg.norm(inertia) < 1e-9:
                inertia = np.eye(3)
        bodies.append(
            {
                "name": body_name,
                "mass": mass,
                "inertia": [[float(v) for v in row] for row in inertia],
                "fixed": bool(body.fixed),
                "position": _vec3(body.pose.translation),
                "quaternion": list(rotation_to_quaternion(body.pose.rotation)),
            }
        )

    joint_source = _quasi_static_joints(
        collapse_spherical_pairs(assembly.ideal_constraints)
        if drive_wheels
        else assembly.constraints
    )
    joints = []
    joints = []
    for constraint in joint_source:
        try:
            definition = definition_for(type(constraint).__name__)
        except JointTableError as exc:
            # The joint table owns the message; this keeps the kc layer's own
            # error type so callers that catch `NativeKcError` still work.
            raise NativeKcError(str(exc)) from exc
        body_a = assembly.bodies[constraint.body_a]
        body_b = assembly.bodies[constraint.body_b]
        entry: dict[str, object] = {
            "name": constraint.name,
            "type": definition.kernel_name,
            "body_a": constraint.body_a,
            "body_b": constraint.body_b,
            "point_a": _vec3(_local_point(constraint.point_a, body_a)),
            "point_b": _vec3(_local_point(constraint.point_b, body_b)),
        }
        # The axes each type needs come from the table, so a joint added to the
        # assembly layer cannot be encoded with the wrong field set.  The
        # defaults the kernel substitutes for a missing axis are plausible
        # numbers, which is why an omission has to fail here instead.
        axes = _constraint_axes(constraint, body_a, body_b)
        validate_joint_axes(
            joint_name=constraint.name,
            kernel_name=definition.kernel_name,
            axes=axes,
        )
        entry.update(axes)
        if isinstance(constraint, ConstantVelocityJoint):
            entry["convel_angle_target"] = float(constraint.angle_target)
        joints.append(entry)

    elements = (
        _element_entries(assembly, drive_wheels=drive_wheels)
        if mode in ELASTIC_MODES
        else []
    )

    markers = [
        {
            "name": f"wheel_center_{side}",
            "body": body,
            "point": _vec3(assembly.point(body, "wheel_center")),
        }
        for side in ("L", "R")
        if (body := wheel_centre_body(assembly, side)) is not None
    ]

    _, driven_joints = _driven_coordinates(assembly, drive_wheels=drive_wheels)
    document: dict[str, object] = {
        "contract": "multibody-model",
        "contract_version": 1,
        "kind": "model",
        "name": name,
        "units": dict(UNITS),
        "gravity": [0.0, 0.0, 0.0],
        "bodies": bodies,
        "joints": joints + driven_joints,
        "elements": elements,
        # Tires belong to the pad reading only.  A wheel-centre-driven reading *places*
        # the wheel, so a tire there would be a second, competing statement about where
        # the wheel is -- and measured, the two cannot both hold: a K run with a tire
        # declared stops at "static equilibrium initialization failed" with
        # `force_residual = 0` and a 20 mm position residual, i.e. the force balances and
        # the prescribed travel is what cannot be met.
        "tires": _tire_entries(assembly, markers) if mode in TIRE_MODES else [],
        "markers": markers,
        # A `c` path loads the wheel centre, which is a point of the upright
        # rather than the upright's origin.  Naming the marker here is what
        # makes the kernel take the lever arm at the pose it solves for;
        # pre-transferring it at the reference pose freezes the arm and loses
        # the first-order geometry of the swept upright.
        "body_wrench_markers": [marker["name"] for marker in markers],
    }
    return document


#: The tire properties a vertical declaration does not carry.  The kernel
#: requires each of them strictly positive even where a static solve uses only
#: the vertical branch, so they are the *neutral* values rather than zeros:
#: unit friction, unit brush stiffness, unit relaxation lengths.  They are not
#: tuning knobs -- a K/C state has no slip for them to act on -- and writing a
#: zero would be refused by the model reader rather than ignored by the solve.
_TIRE_NEUTRAL_COEFFICIENTS: dict[str, float] = {
    "longitudinal_friction_coefficient": 1.0,
    "lateral_friction_coefficient": 1.0,
    "longitudinal_brush_stiffness": 1.0,
    "lateral_brush_stiffness": 1.0,
    "longitudinal_relaxation_length": 1.0,
    "lateral_relaxation_length": 1.0,
    "detached_relaxation_s": 1.0,
}


def _tire_entries(
    assembly, markers: list[dict[str, object]]
) -> list[dict[str, object]]:
    """
    Describe the assembly's vertical tires as contract tire entries.

    This is the K/C path's half of decision D1: a quasi-static state has no
    slip, so the tire acts through its vertical branch alone, but it is *the
    same law* the dynamic study uses rather than a second one.  Leaving the
    tires out -- which is what this used to do -- meant the vertical response
    never entered the residual at all, so changing the tire changed nothing and
    the run looked right while carrying no wheel load.

    The declaration is what the model has: a wheel body, a wheel-centre point, a
    vertical stiffness and an unloaded radius.  Everything the vertical branch
    cannot use is written as its neutral value and named above, so the entry
    says "these are inert here" instead of leaving a reader to infer it from a
    zero that the model reader would in fact refuse.

    ``maximum_compression`` is half the radius, which is the bound the model's
    own schema and the solver's contact loop both need to be physical.  The
    declared tire has no compression limit of its own, so the value is stated
    rather than invented per call.
    """
    entries: list[dict[str, object]] = []
    centers = {str(marker["body"]): marker for marker in markers}
    for element in getattr(assembly, "elements", ()):
        if type(element).__name__ != "VerticalTireElement":
            continue
        body = str(element.wheel_body)
        radius = float(element.unloaded_radius)
        marker = centers.get(body)
        center = (
            marker["point"]
            if marker is not None
            else _vec3(element.wheel_center_local)
        )
        parameters: dict[str, object] = {
            "center_local": [float(value) for value in center],
            # The axis the vertical branch acts along.  The K/C model's up
            # direction is +Z and its tires are declared upright, so this is
            # the declaration rather than a measurement.
            "spin_axis_local": [0.0, 1.0, 0.0],
            "forward_axis_local": [1.0, 0.0, 0.0],
            "unloaded_radius": radius,
            "maximum_compression": radius * 0.5,
            "vertical_stiffness": float(element.stiffness),
            "vertical_damping": 0.0,
            **_TIRE_NEUTRAL_COEFFICIENTS,
        }
        entries.append(
            {
                "name": str(element.name),
                "model": "native_brush",
                "body": body,
                "parameters": parameters,
            }
        )
    return entries


def _bushing_element(bushing) -> dict[str, object]:
    """One bushing, with its stiffness blocks in their own units."""
    return {
        "name": bushing.name,
        "type": "bushing",
        "body_a": bushing.body_a,
        "body_b": bushing.body_b,
        "parameters": {
            "point_a": _vec3(np.asarray(bushing.local_pose_a.translation, dtype=float)),
            "point_b": _vec3(np.asarray(bushing.local_pose_b.translation, dtype=float)),
            "frame_a_quaternion": [float(v) for v in bushing.local_pose_a.quaternion],
            "frame_b_quaternion": [float(v) for v in bushing.local_pose_b.quaternion],
            "stiffness": [
                [float(v) for v in row] for row in np.asarray(bushing.stiffness, dtype=float)
            ],
            "damping": [
                [float(v) for v in row] for row in np.asarray(bushing.damping, dtype=float)
            ],
            "preload": [float(v) for v in np.asarray(bushing.preload, dtype=float)[:6]],
        },
    }


def _element_entries(assembly, *, drive_wheels: bool) -> list[dict[str, object]]:
    """
    Describe the assembly's force elements for the document.

    What goes in depends on the reading, and the difference is physical rather than
    cosmetic:

    * **K** drives the wheel centres through the *rigid kinematic set*, so the arm mounts
      are ideal constraints.  Emitting the bushings as well would double-count the mounts,
      so the elastic column the K document carries is the suspension's own springs and
      bars -- and the *kinematic* reading carries none of them.
    * **C** loads the *compliant* set, where the arm mounts **are** bushings.  Those are
      the elements that carry the wheel load, so they belong in the document.

    Springs are emitted by both readings now.  Before this, they were emitted by neither:
    measured, the module had zero occurrences of `spring`, so a K or C solve had no spring
    reaction at all and the arm mounts were the only compliance in play.  A spring whose
    assembled length differs from its free length applies a force, and a residual that
    cannot see it cannot balance it.

    The bushing list comes from `assembly.elements`, not from `assembly.bushings`: the two
    hold the same objects (measured 16 and 16), so iterating both would emit every mount
    twice.
    """
    entries: list[dict[str, object]] = []
    for element in getattr(assembly, "elements", ()):
        kind = type(element).__name__
        if kind == "LinearSpringElement":
            entries.append(_spring_element(element))
        elif kind == "StaticDamperElement":
            entries.append(_damper_element(element))
        elif kind == "BumpStopElement":
            entries.append(_bump_stop_element(element))
        elif kind == "AntiRollBarElement":
            entries.append(_anti_roll_element(element))
        elif kind == "BushingElement" and not drive_wheels:
            # The C reading's mounts.  In K these are ideal constraints instead, so the
            # same named bushings would be a second, redundant description of one joint.
            entries.append(_bushing_element(element))
    return entries


def _spring_element(spring) -> dict[str, object]:
    """
    One elastic element, in the document's millimetres.

    `free_length` and `reference_length` are two spellings of the same reference, and the
    element refuses both at once; whichever was given is what the document states.  A
    `preload` is carried separately because it is a force, not a length.
    """
    parameters: dict[str, object] = {
        "point_a": _vec3(np.asarray(spring.point_a, dtype=float)),
        "point_b": _vec3(np.asarray(spring.point_b, dtype=float)),
        "stiffness": float(spring.stiffness),
        "preload": float(spring.preload),
    }
    if spring.free_length is not None:
        parameters["free_length"] = float(spring.free_length)
    if spring.reference_length is not None:
        parameters["reference_length"] = float(spring.reference_length)
    if spring.force_curve:
        parameters["elastic_curve"] = [
            [float(deflection), float(force)] for deflection, force in spring.force_curve
        ]
    return {
        "name": spring.name,
        "type": "spring",
        "body_a": spring.body_a,
        "body_b": spring.body_b,
        "parameters": parameters,
    }


def _damper_element(damper) -> dict[str, object]:
    """
    One dissipative element.

    Every coefficient is stated, including the ones a *static* solve cannot use: the
    reading has zero velocity, so the damping terms contribute nothing.  They are written
    rather than omitted because the document is a description of the element, and a reader
    that had to infer "absent means zero" would be reading a different element than the
    one the assembly built.
    """
    parameters: dict[str, object] = {
        "point_a": _vec3(np.asarray(damper.point_a, dtype=float)),
        "point_b": _vec3(np.asarray(damper.point_b, dtype=float)),
        "compression_damping": float(damper.viscous_damping),
        "rebound_damping": float(damper.viscous_damping),
        "gas_stiffness": float(damper.gas_stiffness),
        "gas_reference_force": float(damper.gas_reference_force),
        "preload": float(damper.preload),
        "friction": float(damper.friction),
        "extension_sign": float(damper.extension_sign),
    }
    if damper.gas_reference_length is not None:
        parameters["gas_reference_length"] = float(damper.gas_reference_length)
    if damper.force_curve:
        parameters["damper_curve"] = [
            [float(velocity), float(force)] for velocity, force in damper.force_curve
        ]
    return {
        "name": damper.name,
        "type": "damper",
        "body_a": damper.body_a,
        "body_b": damper.body_b,
        "parameters": parameters,
    }


def _bump_stop_element(stop) -> dict[str, object]:
    """
    One unilateral element.

    `direction` is the string `bump`/`rebound` in the modelling layer and a signed number
    in the contract -- the kernel compares it numerically to decide which side of
    `clearance` engages, so passing the word through would compare a string to a number.
    """
    parameters: dict[str, object] = {
        "point_a": _vec3(np.asarray(stop.point_a, dtype=float)),
        "point_b": _vec3(np.asarray(stop.point_b, dtype=float)),
        "clearance": float(stop.clearance),
        "stiffness": float(stop.stiffness),
        "direction": 1.0 if str(stop.direction).lower() == "bump" else -1.0,
    }
    if stop.force_curve:
        parameters["stop_curve"] = [
            [float(penetration), float(force)] for penetration, force in stop.force_curve
        ]
    return {
        "name": stop.name,
        "type": "bump_stop",
        "body_a": stop.body_a,
        "body_b": stop.body_b,
        "parameters": parameters,
    }


def _anti_roll_element(bar) -> dict[str, object]:
    """
    One anti-roll bar, as the **link law** the assembly actually builds.

    The modelling layer's bar is a link: it applies a vertical force pair proportional to
    the vertical separation of its two attachment points.  The kernel's own
    `anti_roll_bar` is a different statement -- a *torsional* bar taking an axis and a
    reference quaternion -- and `preparation/vehicle_dynamic.py` refuses to convert
    between them by name.

    So the link law is expressed as what it is: a **bushing with a single non-zero
    stiffness entry**, `K[2][2]`.  A bushing's force is `K6x6` times the relative
    displacement at its own point pair, so a matrix that is zero everywhere except the
    vertical term produces exactly the vertical pair the link law produces.  Measured at
    dz = +/-10 mm with K = 10 N/mm: the link law gives [0, 0, +/-100] and the z-only
    bushing gives [0, 0, +/-100].

    The identity holds because a bushing evaluates in its own frame, and these attachments
    are declared with the unit quaternion -- so the frame's z *is* the world z.  A tilted
    pair would make the two diverge, which is why the quaternions are written out rather
    than defaulted.
    """
    stiffness = float(bar.stiffness)
    matrix = [[0.0] * 6 for _ in range(6)]
    matrix[2][2] = stiffness
    identity = [1.0, 0.0, 0.0, 0.0]
    return {
        "name": bar.name,
        "type": "bushing",
        "body_a": bar.left_body,
        "body_b": bar.right_body,
        "parameters": {
            "point_a": _vec3(np.asarray(bar.left_point, dtype=float)),
            "point_b": _vec3(np.asarray(bar.right_point, dtype=float)),
            "frame_a_quaternion": list(identity),
            "frame_b_quaternion": list(identity),
            "stiffness": matrix,
            "damping": [[0.0] * 6 for _ in range(6)],
            "preload": [0.0] * 6,
        },
    }


def _driven_coordinates(assembly, *, drive_wheels: bool):
    """
    Describe the prescribed degrees of freedom, as joints plus their signal names.

    A driven coordinate names its target signal rather than carrying targets:
    how far the wheel travels is a *case* input, and the case layer turns the
    travel into the absolute separation the kernel's constraint row measures.
    The axis is written in the reaction body's frame because that is the frame
    the relative separation is measured in.

    The rack row is emitted only when the assembly actually has a rack.  This is
    the shrink the rig has always promised and the document did not deliver: a
    single-axle assembly with no steering subsystem carries no `rack` body, so
    asking it for one raised `KeyError: ('rack', 'center')` -- a run that failed
    for a reason with no relation to what the caller asked.  "The assembly has no
    rack axis" now means the axis is *absent*, not that it is neutral or zero.
    """
    reaction_body = "chassis" if "chassis" in assembly.bodies else "ground"
    coordinates: list[tuple[str, dict[str, object]]] = []

    def add(name: str, kind: str, body: str, point_local_mm, axis_local) -> None:
        entry = {
            "name": name,
            "type": _DRIVEN_KINDS[kind],
            "body_a": body,
            "body_b": reaction_body,
            "point_a": _vec3(point_local_mm),
            "point_b": [0.0, 0.0, 0.0],
            "axis_b": _unit_axis(axis_local),
            "reference_quaternion": [1.0, 0.0, 0.0, 0.0],
            "target": name,
        }
        coordinates.append((name, entry))

    if drive_wheels:
        for side in ("L", "R"):
            body = wheel_centre_body(assembly, side)
            if body is None:
                continue
            add(
                f"wheel_drive_{side}",
                "translation",
                body,
                assembly.point(body, "wheel_center"),
                (0.0, 0.0, 1.0),
            )
        rack_name = "rack_drive"
    else:
        rack_name = "rack_neutral"
    if has_rack(assembly):
        add(
            rack_name,
            "translation",
            "rack",
            assembly.point("rack", "center"),
            (0.0, 1.0, 0.0),
        )
    return coordinates, [entry for _, entry in coordinates]


def has_rack(assembly) -> bool:
    """
    Return whether an assembly carries a rack body and a rack-centre point.

    The question is asked of the *assembly*, not of a role name or a family: an
    assembly builds its rack only when it carries the steering subsystem, and a
    document that asked for the coordinate anyway would be asking for a body that
    does not exist.  Both halves are required -- a body with no centre point has
    no axis to drive along, which is the other half of the same mistake.
    """
    bodies = getattr(assembly, "bodies", {})
    points = getattr(assembly, "points", {})
    return "rack" in bodies and ("rack", "center") in points


#: The body names a wheel centre is conventionally attached to, in the order the
#: lookup tries them.  A model may name the wheel-carrying body anything; these
#: are the spellings the built-in topologies use, and the *point* is what decides
#: -- a body that declares no `wheel_center` is not the carrier, whatever it is
#: called.
_WHEEL_CENTRE_BODIES: tuple[str, ...] = (
    "wheel_hub",
    "upright",
    "knuckle",
    "hub_carrier",
    "trailing_arm",
)


def wheel_centre_body(assembly, side: str) -> str | None:
    """
    Return the body that carries one side's wheel centre, whatever it is called.

    This used to be spelled `f"upright_{side}"` at every call site, which is a
    *template name*: a trailing-arm axle whose wheel-carrying body is
    `trailing_arm_L` produced a document that asked for a body it did not have.
    A compiler that knows a template's part names cannot compile a topology it has
    not seen -- which is exactly what a fixed set of templates would have hidden.

    The search is by *declaration*: a body carrying a `wheel_center` point is the
    one the wheel centre belongs to.  The conventional names are tried first so
    the answer for the built-in topologies is the same as it always was, and the
    full body list is then searched so a novel name is found too.  Two candidates
    are refused rather than guessed at: a model with two wheel-carrying bodies per
    side is ambiguous, and picking one would silently attach the wheel to the
    wrong part.
    """
    candidates = [f"{stem}_{side}" for stem in _WHEEL_CENTRE_BODIES]
    decided = [name for name in candidates if _carries_wheel_centre(assembly, name)]
    if not decided:
        decided = sorted(
            name
            for name in getattr(assembly, "bodies", {})
            if name.endswith(f"_{side}") and _carries_wheel_centre(assembly, name)
        )
    if len(decided) > 1:
        raise NativeKcError(
            f"side {side} declares more than one wheel centre ({', '.join(decided)}); "
            "a wheel centre belongs to exactly one body, and picking one would "
            "attach the wheel to the wrong part"
        )
    return decided[0] if decided else None


def _carries_wheel_centre(assembly, body: str) -> bool:
    """Return whether one body declares the wheel-centre point."""
    return (
        body in getattr(assembly, "bodies", {})
        and ("wheel_center" in {label for _, label in assembly.points if _ == body})
    )


def solver_settings_document(settings, *, times_s=None) -> dict[str, object]:
    """Compatibility wrapper for the shared neutral solver serializer."""
    del times_s
    return _solver_settings_document(settings)


def time_document(times_s) -> dict[str, object]:
    """Return the output grid as a start/end/step the case layer can re-derive."""
    values = [float(value) for value in times_s]
    if len(values) < 2:
        raise NativeKcError("a case needs at least two sample times")
    step = values[1] - values[0]
    for left, right in zip(values, values[1:]):
        if abs((right - left) - step) > 1e-15:
            raise NativeKcError("the case layer only expands a uniform time grid")
    return {"start_s": values[0], "end_s": values[-1], "step_s": step}


def case_document(
    assembly,
    *,
    family: str,
    name: str,
    wheel_values_mm: tuple[float, ...] = (),
    rack_values_mm: tuple[float, ...] = (),
    drive: str = "wheel_center",
    left_right_mode: str = "symmetric",
    paths: tuple[str, ...] = (),
    levels: int = 11,
    maximum: float = 1.0,
    side_mode: str = "single",
    times_s: tuple[float, ...] = (),
    settings=None,
    drive_wheels: bool | None = None,
    drives: tuple[str, ...] | None = None,
) -> dict[str, object]:
    """
    Describe a ``kc_quasi_static`` request as a case document.

    ``drives`` is the set of drive coordinates the *resolved run* actually moves,
    in the rig's own order, and it is what the axis map is built from.  The set
    comes from shrinking the rig's declaration to the assembly's capabilities
    (``rigs.compose``), so a coordinate the assembly cannot offer never appears
    here at all.

    It used to be derived by searching the emitted names for a prefix
    (``value.startswith("wheel_drive_")``), which made the grouping a property of
    the *spelling*: a coordinate renamed in the rig, or a group the kernel grows,
    would fall out of the map silently and the grid would lose an axis without
    reporting it.  ``subsystems.capabilities.kernel_axis`` states the mapping once
    and this function reads it.

    Passing ``None`` keeps the historical behaviour for callers that have no rig
    composition in hand; the coordinates are then taken from the assembly itself.
    """
    if family not in ("kc_quasi_static",):
        raise NativeKcError(f"unsupported case family {family!r}")
    document: dict[str, object] = {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": family,
        "name": name,
    }
    if drive_wheels is None:
        drive_wheels = bool(wheel_values_mm)
    driven, _ = _driven_coordinates(assembly, drive_wheels=drive_wheels)
    driven_names = [coordinate for coordinate, _ in driven]
    resolved_drives = tuple(driven_names) if drives is None else tuple(drives)
    if times_s:
        document["time"] = time_document(times_s)
    if settings is not None:
        document["solver"] = solver_settings_document(settings, times_s=times_s)
    if wheel_values_mm or rack_values_mm:
        # The axis map names the driven coordinates the grid moves, grouped the way
        # the kernel reads them.  A group with no surviving coordinate is *absent*
        # rather than empty-or-zero: an assembly without steering has no rack axis,
        # and writing one at 0.0 would make the run look steered when it is not.
        axis_map: dict[str, object] = {}
        for coordinate in resolved_drives:
            group = kernel_axis(coordinate)
            if group is None:
                continue
            if group == "wheel":
                axis_map.setdefault("wheel", [])
                axis_map["wheel"].append(coordinate)  # type: ignore[union-attr]
            else:
                axis_map[group] = coordinate
        document["k"] = {
            "wheel_values_mm": [float(v) for v in wheel_values_mm],
            "rack_values_mm": [float(v) for v in rack_values_mm],
            "drive": drive,
            "left_right_mode": left_right_mode,
            "axis_map": axis_map,
        }
    if paths:
        document["c"] = {
            "paths": [str(p) for p in paths],
            "levels": int(levels),
            "maximum": float(maximum),
            "side_mode": side_mode,
            "load_marker": "wheel_center_L",
        }
    return document
