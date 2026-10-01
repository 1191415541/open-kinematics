"""
Hardpoint lookup, mirroring and body-local conversion, shared by the assembly
layer and the subsystems.

These helpers used to live inside the retired axle builder.  The
subsystems need them too, and a subsystem may not import the module that imports
it, so they live here and the assembly re-exports the names it always had.  The
code is unchanged: same aliases, same mirroring, same error text.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Literal

import numpy as np

from ..modeling.primitives.joints import RigidBody
from ..modeling.primitives.spatial import SE3
from ..schema import Pose, Vec3

__all__ = [
    "HARDPOINT_ALIASES",
    "as_array",
    "body_from_part",
    "body_from_spec",
    "body_without_spec",
    "local_point",
    "local_pose",
    "lookup_hardpoint",
    "mirror_hardpoints",
    "mirror_point",
    "resolve_body",
    "side_hardpoints",
]

HARDPOINT_ALIASES: dict[str, tuple[str, ...]] = {
    # `upper_front`/`upper_rear`/`lower_front`/`lower_rear` also accept the
    # role's own name.  The Geometry Contract adapter emits the role as the
    # hardpoint key, and without this entry a contract-derived model cannot be
    # assembled at all -- the lookup fails with "missing required front-axle
    # hardpoint for upper_front".  Widening the list here rather than changing
    # the adapter's emitted keys leaves the adapter's public output untouched,
    # and a widened alias list removes no spelling.  The other six roles need
    # nothing: their adapters already emit a spelling the table accepts.
    "upper_front": (
        "UPPER_INBOARD_FRONT",
        "UPPER_INNER_FRONT",
        "UCA_FRONT",
        "UCA_INNER_FRONT",
        "UPPER_FRONT",
    ),
    "upper_rear": (
        "UPPER_INBOARD_REAR",
        "UPPER_INNER_REAR",
        "UCA_REAR",
        "UCA_INNER_REAR",
        "UPPER_REAR",
    ),
    "upper_outer": ("UPPER_OUTBOARD", "UPPER_OUTER", "UCA_OUTER"),
    "lower_front": (
        "LOWER_INBOARD_FRONT",
        "LOWER_INNER_FRONT",
        "LCA_FRONT",
        "LCA_INNER_FRONT",
        "LOWER_FRONT",
    ),
    "lower_rear": (
        "LOWER_INBOARD_REAR",
        "LOWER_INNER_REAR",
        "LCA_REAR",
        "LCA_INNER_REAR",
        "LOWER_REAR",
    ),
    "lower_outer": ("LOWER_OUTBOARD", "LOWER_OUTER", "LCA_OUTER"),
    "tie_inner": ("TIE_ROD_INBOARD", "TIE_ROD_INNER", "TIEROD_INNER", "RACK_TIE_ROD"),
    "tie_outer": ("TIE_ROD_OUTBOARD", "TIE_ROD_OUTER", "TIEROD_OUTER"),
    "wheel_center": ("WHEEL_CENTER", "WHEEL_CENTRE", "WHEEL_CG"),
    "rack_center": ("RACK_CENTER", "RACK_CENTRE", "RACK_REFERENCE"),
}


def as_array(point: Vec3 | np.ndarray | list[float]) -> np.ndarray:
    """Return a hardpoint as a float array."""
    if isinstance(point, Vec3):
        return np.asarray(point.as_tuple(), dtype=float)
    return np.asarray(point, dtype=float)


def mirror_hardpoints(hardpoints: dict[str, Vec3]) -> dict[str, Vec3]:
    """Return left hardpoints plus deterministic ``__R`` mirrored copies."""
    mirrored = dict(hardpoints)
    for name, point in hardpoints.items():
        mirrored[f"{name}__R"] = point.mirrored_y()
    return mirrored


#: The suffix a model uses to state a right-side hardpoint explicitly.
_RIGHT_SUFFIX = "__R"


def side_hardpoints(
    hardpoints: dict[str, Vec3], side: Literal["L", "R"]
) -> dict[str, Vec3]:
    """
    Return one side's hardpoints with a common, side-independent key set.

    Mirroring is the **default**, not the rule: a hardpoint written ``name__R``
    states the right side's own coordinates, and that statement wins over the
    mirror.  A left/right-asymmetric axle is therefore expressible in the model's
    own table -- one side mirrored where the shorthand is right, the other side
    declared where it is not -- without giving up the shorthand for the symmetric
    case.  The rule is the explicit topology's (`subsystems/explicit.py`), stated
    once here so both routes read one model the same way.

    The ``__R`` keys never appear in the result: the key set stays the same on
    both sides, which is what lets every consumer look a role up without asking
    which side it is on.
    """
    declared: dict[str, Vec3] = {
        name[: -len(_RIGHT_SUFFIX)]: point
        for name, point in hardpoints.items()
        if name.endswith(_RIGHT_SUFFIX)
    }
    if side == "L":
        return {
            name: point
            for name, point in hardpoints.items()
            if not name.endswith(_RIGHT_SUFFIX)
        }
    return {
        name: declared.get(name, point.mirrored_y())
        for name, point in hardpoints.items()
        if not name.endswith(_RIGHT_SUFFIX)
    }


def lookup_hardpoint(hardpoints: dict[str, Vec3], role: str) -> Vec3:
    """Resolve a hardpoint role through its alias list."""
    normalized = {
        key.upper().replace("-", "_"): value for key, value in hardpoints.items()
    }
    for alias in HARDPOINT_ALIASES[role]:
        if alias in normalized:
            return normalized[alias]
    raise ValueError(f"missing required front-axle hardpoint for {role}")


def mirror_point(value: Vec3, side: Literal["L", "R"]) -> np.ndarray:
    """Return a hardpoint in the requested side's coordinates."""
    point = value.mirrored_y() if side == "R" else value
    return point.as_array()


def local_point(
    bodies: Mapping[str, object], body: str, point_global: np.ndarray
) -> np.ndarray:
    """Convert an imported global hardpoint into a body-local point."""
    pose = getattr(bodies[body], "pose")
    return pose.inverse().transform_point(point_global)  # type: ignore[union-attr]


def local_pose(
    value: Pose,
    side: Literal["L", "R"],
    body: str,
    bodies: Mapping[str, object],
) -> SE3:
    """Convert a schema attachment pose from vehicle to body coordinates."""
    global_translation = mirror_point(value.translation, side)
    return SE3(
        local_point(bodies, body, global_translation),
        np.asarray(value.rotation.as_tuple(), dtype=float),
    )


def body_from_spec(name: str, spec: object) -> RigidBody:
    """Create a body using the schema reference frame and mass properties."""
    pose_spec = getattr(spec, "pose")
    return RigidBody(
        name=name,
        pose=SE3(
            pose_spec.translation.as_array(),
            np.asarray(pose_spec.rotation.as_tuple(), dtype=float),
        ),
        mass=float(getattr(spec, "mass")),
        inertia=np.asarray(getattr(spec, "inertia"), dtype=float),
        center_of_mass=getattr(spec, "center_of_mass").as_array(),
        fixed=bool(getattr(spec, "fixed")),
    )


def body_without_spec(name: str) -> RigidBody:
    """Create a bare body with an identity pose and no mass properties."""
    return RigidBody(name=name, pose=SE3.identity())


def body_from_part(
    name: str, part: object, *, center_of_mass: np.ndarray | None = None
) -> RigidBody:
    """
    Create a body from the template's own declaration of it.

    A template that declares a mass for a part it *invents* -- the wheel hub, the
    steering housing -- is saying "this body exists and weighs this", and nothing
    else in the flow can say it: the model describes the parts a user authors, and
    a part the topology adds is the template's to weigh.  A template that declares
    none leaves the body bare, which is right for the arms, the upright and the
    rack: those are exactly the parts a model is expected to describe, and a
    template that invented a mass for them would be answering a question nobody
    asked it.

    ``center_of_mass`` is where the template attaches the part -- the hardpoint its
    own connections name -- because a body's frame and its mass are separate
    questions: the frame is the assembly's default (the identity, as for every other
    body with no spec), while the mass has to sit where the part is.  Leaving it at
    the frame origin is not a harmless simplification: a free body whose mass is off
    the axis it turns on carries a gravity moment no constraint can react, so a trim
    has no equilibrium to find -- and a pair of bodies welded together cannot even
    share one rigid motion while their centres of mass sit a metre apart.

    The mass is spread over :data:`_PART_RADIUS_OF_GYRATION_MM`, since a declaration
    of mass alone says nothing about shape and a *point* mass is the one shape no
    part has: the identity inertia the assembly otherwise defaults to is orders of
    magnitude off the body's own mass, and that is what ill-conditions the equations
    the body appears in.
    """
    mass = float(getattr(part, "mass", 0.0))
    if mass <= 0.0:
        return body_without_spec(name)
    return RigidBody(
        name=name,
        pose=SE3.identity(),
        mass=mass,
        inertia=np.eye(3) * mass * _PART_RADIUS_OF_GYRATION_MM**2,
        center_of_mass=(
            np.zeros(3)
            if center_of_mass is None
            else np.asarray(center_of_mass, dtype=float)
        ),
        fixed=bool(getattr(part, "fixed", False)),
    )


#: The radius of gyration an invented part's mass is spread over, in mm.
#:
#: The part a template invents is a *light* placeholder: the template is saying the
#: body exists and carries the mass it declares, and nothing about its shape, so the
#: mass is spread over a small radius rather than concentrated into a point.  A point
#: mass is the one shape no part has, and it is what the assembly would otherwise
#: default to -- an inertia orders of magnitude off the body's own mass, which is
#: what ill-conditions the equations the body appears in.  A model that knows the
#: hub's own mass properties declares a spec for it and takes over from here.
_PART_RADIUS_OF_GYRATION_MM = 8.0

#: Schema body name -> generated body name stem.
# The alias table that used to sit here is gone (stage four).  It mapped a
# model's vocabulary onto this library's body names -- `wheel`/`knuckle`/`spindle`
# onto `upright`, `uca` onto `upper_arm` -- which is exactly the "guess an
# identity from a name" rule the assembly layer forbids.  Its job is now done by
# *declarations*: which member carries a wheel end, or an anti-roll bar's
# droplink, is stated by the suspension template's own ports and connections
# (`templates/builtin.py::_PORTS`, `::_CONNECTIONS`), and a subsystem reads the
# declaration instead of matching a spelling.  A model that names a body is
# naming a body this assembly carries; if it does not, that is an error to report
# rather than a name to translate.

def resolve_body(
    name: str, side: Literal["L", "R"], bodies: Mapping[str, object]
) -> str:
    """
    Resolve a schema body name to a generated side-specific body.

    A name that spells the chassis resolves to `ground` when the assembly carries
    no chassis body, and that is the whole point of the fallback: a single-axle
    assembly has no chassis (requirement 2 -- its inner mounts and everything that
    used to hang off a fixed chassis attach to `ground`, and a full-vehicle reading
    maps that ground onto the vehicle body), while a spring, damper or bushing
    authored against `chassis` keeps its own spelling.  A model does not have to be
    re-authored to be read as an axle, which is what makes one model serve both
    assemblies.
    """
    if name in bodies:
        return name
    normalized = name.strip().lower().replace("-", "_")
    if normalized in _GROUND_ALIASES and "ground" in bodies:
        return "ground"
    candidate = f"{normalized}_{side}"
    if candidate in bodies:
        return candidate
    for suffix in ("_l", "_r", " left", " right"):
        if normalized.endswith(suffix):
            stem = normalized[: -len(suffix)].rstrip()
            candidate = f"{stem}_{side}"
            if candidate in bodies:
                return candidate
    raise ValueError(f"unknown force-element body {name!r}")


#: The spellings a force element may use for the body an axle does not carry.
#:
#: The chassis is the vehicle body: a full-vehicle assembly has one and an axle has
#: none, so a name in this set is resolved to `ground` only when no such body
#: exists.  Listed rather than derived, because "this name means the chassis" is a
#: fact about the schema's vocabulary and not something the name can be asked
#: about.
_GROUND_ALIASES: frozenset[str] = frozenset({"chassis", "vehicle_body"})
