"""
Hardpoint lookup, mirroring and body-local conversion, shared by the assembly
layer and the subsystems.

These helpers used to live inside ``preparation/assembly/front_axle.py``.  The
subsystems need them too, and a subsystem may not import the module that imports
it, so they live here and the assembly re-exports the names it always had.  The
code is unchanged: same aliases, same mirroring, same error text.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Literal

import numpy as np

from ..preparation.assembly.types import RigidBody
from ..preparation.geometry import SE3
from ..schema import Pose, Vec3

__all__ = [
    "HARDPOINT_ALIASES",
    "as_array",
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
    "upper_front": (
        "UPPER_INBOARD_FRONT",
        "UPPER_INNER_FRONT",
        "UCA_FRONT",
        "UCA_INNER_FRONT",
    ),
    "upper_rear": (
        "UPPER_INBOARD_REAR",
        "UPPER_INNER_REAR",
        "UCA_REAR",
        "UCA_INNER_REAR",
    ),
    "upper_outer": ("UPPER_OUTBOARD", "UPPER_OUTER", "UCA_OUTER"),
    "lower_front": (
        "LOWER_INBOARD_FRONT",
        "LOWER_INNER_FRONT",
        "LCA_FRONT",
        "LCA_INNER_FRONT",
    ),
    "lower_rear": (
        "LOWER_INBOARD_REAR",
        "LOWER_INNER_REAR",
        "LCA_REAR",
        "LCA_INNER_REAR",
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


def side_hardpoints(
    hardpoints: dict[str, Vec3], side: Literal["L", "R"]
) -> dict[str, Vec3]:
    """Return one side's hardpoints with a common, side-independent key set."""
    if side == "L":
        return dict(hardpoints)
    return {name: point.mirrored_y() for name, point in hardpoints.items()}


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


#: Schema body name -> generated body name stem.
_BODY_ALIASES: dict[str, str] = {
    "uca": "upper_arm",
    "upper": "upper_arm",
    "lca": "lower_arm",
    "lower": "lower_arm",
    "wheel": "upright",
    "knuckle": "upright",
    "spindle": "upright",
    "tie": "tie_rod",
    "tierod": "tie_rod",
}

def resolve_body(
    name: str, side: Literal["L", "R"], bodies: Mapping[str, object]
) -> str:
    """Resolve a schema body name to a generated side-specific body."""
    if name in bodies:
        return name
    normalized = name.strip().lower().replace("-", "_")
    base = _BODY_ALIASES.get(normalized, normalized)
    candidate = f"{base}_{side}"
    if candidate in bodies:
        return candidate
    for suffix in ("_l", "_r", " left", " right"):
        if normalized.endswith(suffix):
            stem = normalized[: -len(suffix)].rstrip()
            candidate = f"{_BODY_ALIASES.get(stem, stem)}_{side}"
            if candidate in bodies:
                return candidate
    raise ValueError(f"unknown force-element body {name!r}")
